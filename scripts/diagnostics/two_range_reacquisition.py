"""Tagged private-state lifecycle for two-range branch reacquisition."""

from collections.abc import Mapping
from numbers import Integral, Real

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    FRAME_DT_SECONDS,
    INNOVATION_REFERENCE_QUANTILE,
    _complete_converged_solver_result,
    canonical_spd_covariance,
    finalize_attempt,
    make_unavailable_output,
    normalized_innovation,
    propagate_estimator_prior,
    solve_finite_budget_wnls,
    two_circle_candidates,
    candidate_acceptance,
)


METHOD_ID = "two_range_private_branch_reacquisition"
BRANCH_IDS = ("circle_negative", "circle_positive")
PRIVATE_STATE_FIELDS = (
    "status",
    "estimate",
    "modeled_covariance",
    "source_fresh_frame",
    "propagated_to_frame",
    "age_frames",
)


def _finite_vector(value: object) -> np.ndarray | None:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if vector.shape != (2,) or not np.isfinite(vector).all():
        return None
    return vector


def branch_gate_passes(q_value: object) -> bool:
    if isinstance(q_value, bool) or not isinstance(q_value, Real):
        return False
    q = float(q_value)
    return bool(
        np.isfinite(q)
        and q >= 0.0
        and q <= INNOVATION_REFERENCE_QUANTILE
    )


def validate_solver_branches(branches: object) -> tuple[bool, str]:
    if (
        not isinstance(branches, list)
        or len(branches) != 2
        or any(not isinstance(branch, Mapping) for branch in branches)
        or any(
            not _complete_converged_solver_result(
                branch.get("solver_result"),
            )
            for branch in branches
        )
    ):
        return False, "two_range_branch_solver_invalid"
    estimates = [
        np.asarray(branch["solver_result"]["estimate"], dtype=float)
        for branch in branches
    ]
    if np.linalg.norm(estimates[0] - estimates[1]) <= 1e-9:
        return False, "two_range_solver_branches_merged"
    return True, "accepted"


def _rejected_attempt(
    reason: str,
    *,
    branches: list[dict] | None = None,
    prior_used: bool = False,
    selected_branch: dict | None = None,
) -> dict:
    return {
        "attempt_status": "rejected",
        "status": "rejected",
        "failure_reason": reason,
        "branches": [] if branches is None else branches,
        "selected_branch_id": (
            None if selected_branch is None else selected_branch["branch_id"]
        ),
        "selected_candidate": selected_branch,
        "candidate": None,
        "prior_used_for_branch_selection": prior_used,
        "prior_used_in_fim": False,
        "prior_used_for_continuous_update": False,
    }


def solve_two_range_reacquisition(
    *,
    robot_id: int,
    reference_positions: object,
    reference_covariances: object,
    measurements: object,
    reference_keys: object,
    private_prior: object,
    ranging_sigma: float,
    base_anchor_provenance: object,
) -> dict:
    if (
        isinstance(robot_id, bool)
        or not isinstance(robot_id, Integral)
        or robot_id < 1
    ):
        return _rejected_attempt("two_range_robot_id_invalid")
    try:
        positions = np.asarray(reference_positions, dtype=float)
        ranges = np.asarray(measurements, dtype=float)
        raw_covariances = tuple(reference_covariances)
        raw_keys = tuple(reference_keys)
    except (TypeError, ValueError, OverflowError):
        return _rejected_attempt("two_range_input_invalid")
    canonical_covariances = tuple(
        canonical_spd_covariance(value) for value in raw_covariances
    )
    if len(canonical_covariances) != 2 or any(
        value is None for value in canonical_covariances
    ):
        return _rejected_attempt("two_range_reference_covariance_invalid")
    covariances = np.asarray(canonical_covariances, dtype=float)
    if (
        positions.shape != (2, 2)
        or ranges.shape != (2,)
        or not np.isfinite(positions).all()
        or not np.isfinite(ranges).all()
        or np.any(ranges <= 0.0)
    ):
        return _rejected_attempt("two_range_input_invalid")
    keys = tuple(
        (str(key[0]), int(key[1]))
        if (
            isinstance(key, (list, tuple))
            and len(key) == 2
            and not isinstance(key[1], bool)
            and isinstance(key[1], Integral)
        )
        else None
        for key in raw_keys
    )
    if (
        len(keys) != 2
        or any(key is None for key in keys)
        or any(
            key[0] != "uav" or key[1] < 1 or key[1] >= robot_id
            for key in keys
        )
        or len(set(keys)) != 2
    ):
        return _rejected_attempt("two_range_reference_keys_invalid")
    order = tuple(sorted(range(2), key=lambda index: keys[index][1]))
    positions = positions[list(order)]
    covariances = covariances[list(order)]
    ranges = ranges[list(order)]
    keys = tuple(keys[index] for index in order)
    prior = canonical_private_state(private_prior)
    if prior is None:
        return _rejected_attempt("two_range_private_prior_invalid")
    if not isinstance(base_anchor_provenance, (list, tuple)) or any(
        isinstance(root, bool) or not isinstance(root, Integral) or root < 0
        for root in base_anchor_provenance
    ):
        return _rejected_attempt("two_range_base_anchor_provenance_invalid")
    provenance = sorted({int(root) for root in base_anchor_provenance})
    if len(provenance) < 2:
        return _rejected_attempt("two_range_base_anchor_provenance_invalid")
    starts = two_circle_candidates(
        positions[0], ranges[0], positions[1], ranges[1],
    )
    if len(starts) != 2:
        return _rejected_attempt("two_range_circle_geometry_invalid")
    if np.linalg.norm(starts[0] - starts[1]) <= 1e-9:
        return _rejected_attempt("two_range_circle_starts_not_distinct")
    branches = []
    for branch_id, start in zip(BRANCH_IDS, starts, strict=True):
        result = solve_finite_budget_wnls(
            positions,
            covariances,
            ranges,
            start,
            ranging_sigma,
        )
        branches.append(
            {
                "branch_id": branch_id,
                "circle_start": np.asarray(start, dtype=float).tolist(),
                "solver_result": result,
                "q_branch": None,
                "passes_branch_gate": None,
            },
        )
    branches_valid, branch_reason = validate_solver_branches(branches)
    if not branches_valid:
        return _rejected_attempt(branch_reason, branches=branches)
    innovations = [
        normalized_innovation(
            branch["solver_result"]["estimate"],
            branch["solver_result"]["covariance"],
            prior,
        )
        for branch in branches
    ]
    if any(not innovation["valid"] for innovation in innovations):
        reason = next(
            str(innovation["failure_reason"])
            for innovation in innovations
            if not innovation["valid"]
        )
        return _rejected_attempt(reason, branches=branches)
    for branch, innovation in zip(branches, innovations, strict=True):
        branch["q_branch"] = float(innovation["q_innov"])
        branch["passes_branch_gate"] = branch_gate_passes(
            branch["q_branch"],
        )
    passing = [
        branch for branch in branches if branch["passes_branch_gate"] is True
    ]
    if len(passing) != 1:
        return _rejected_attempt(
            (
                "two_range_no_branch_passes"
                if not passing
                else "two_range_multiple_branches_pass"
            ),
            branches=branches,
            prior_used=True,
        )
    selected_branch = passing[0]
    accepted, reason, _ = candidate_acceptance(
        selected_branch["solver_result"],
        live_prediction=None,
        active_reference_count=2,
        base_anchor_provenance=provenance,
        allow_two_reference_reacquisition=True,
    )
    if not accepted:
        return _rejected_attempt(
            reason,
            branches=branches,
            prior_used=True,
            selected_branch=selected_branch,
        )
    selected_result = selected_branch["solver_result"]
    candidate = {
        "estimate": np.asarray(
            selected_result["estimate"], dtype=float,
        ).tolist(),
        "modeled_covariance": np.asarray(
            selected_result["covariance"], dtype=float,
        ).tolist(),
        "epsilon": float(selected_result["epsilon"]),
        "base_anchor_provenance": provenance,
    }
    return {
        "attempt_status": "accepted",
        "status": "accepted",
        "failure_reason": None,
        "branches": branches,
        "selected_branch_id": selected_branch["branch_id"],
        "selected_candidate": selected_branch,
        "candidate": candidate,
        "prior_used_for_branch_selection": True,
        "prior_used_in_fim": False,
        "prior_used_for_continuous_update": False,
    }


def canonical_private_state(value: object) -> dict | None:
    if not isinstance(value, Mapping):
        return None
    if set(value) != set(PRIVATE_STATE_FIELDS):
        return None
    if value.get("status") != "available":
        return None
    estimate = _finite_vector(value.get("estimate"))
    covariance = canonical_spd_covariance(value.get("modeled_covariance"))
    if estimate is None or covariance is None:
        return None
    indices = (
        value.get("source_fresh_frame"),
        value.get("propagated_to_frame"),
        value.get("age_frames"),
    )
    if any(
        isinstance(item, bool) or not isinstance(item, Integral) or item < 0
        for item in indices
    ):
        return None
    source, propagated, age = (int(item) for item in indices)
    if propagated - source != age:
        return None
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": source,
        "propagated_to_frame": propagated,
        "age_frames": age,
    }


def reset_private_state(candidate: object, *, frame_index: int) -> dict | None:
    if not isinstance(candidate, Mapping):
        return None
    estimate = _finite_vector(candidate.get("estimate"))
    covariance = canonical_spd_covariance(candidate.get("modeled_covariance"))
    if estimate is None or covariance is None:
        return None
    if isinstance(frame_index, bool) or not isinstance(frame_index, Integral):
        return None
    if frame_index < 0:
        return None
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": int(frame_index),
        "propagated_to_frame": int(frame_index),
        "age_frames": 0,
    }


def propagate_private_state(
    previous_state: object,
    held_velocity: object,
    *,
    next_frame_index: int,
    dt: float = FRAME_DT_SECONDS,
) -> dict | None:
    state = canonical_private_state(previous_state)
    velocity = _finite_vector(held_velocity)
    if state is None or velocity is None:
        return None
    if (
        isinstance(next_frame_index, bool)
        or not isinstance(next_frame_index, Integral)
        or isinstance(dt, bool)
        or not isinstance(dt, Real)
        or not np.isfinite(float(dt))
        or float(dt) <= 0.0
    ):
        return None
    if next_frame_index != state["propagated_to_frame"] + 1:
        return None
    estimate = np.asarray(state["estimate"]) + float(dt) * velocity
    covariance = np.asarray(state["modeled_covariance"]) + 0.25 * np.eye(2)
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": state["source_fresh_frame"],
        "propagated_to_frame": next_frame_index,
        "age_frames": next_frame_index - state["source_fresh_frame"],
    }


def advance_two_range_prior(
    previous_public: object,
    previous_private: object,
    held_velocity: object,
    *,
    next_frame_index: int,
) -> dict:
    unavailable = make_unavailable_output("no_live_public_prediction")
    if (
        isinstance(next_frame_index, bool)
        or not isinstance(next_frame_index, Integral)
        or next_frame_index < 0
    ):
        return {
            "public_prediction": unavailable,
            "branch_selection_prior": None,
        }
    if next_frame_index == 0:
        return {
            "public_prediction": unavailable,
            "branch_selection_prior": None,
        }
    velocity = _finite_vector(held_velocity)
    if velocity is None:
        return {
            "public_prediction": unavailable,
            "branch_selection_prior": None,
        }
    incoming = propagate_private_state(
        previous_private,
        velocity,
        next_frame_index=next_frame_index,
    )
    public_bundle = propagate_estimator_prior(
        dict(previous_public) if isinstance(previous_public, Mapping) else None,
        None,
        velocity,
    )
    return {
        "public_prediction": public_bundle["public_prediction"],
        "branch_selection_prior": incoming,
    }


def finalize_two_range_lifecycle(
    attempt: object,
    prior_bundle: object,
    *,
    frame_index: int,
) -> dict:
    if not isinstance(attempt, Mapping) or not isinstance(prior_bundle, Mapping):
        raise ValueError("attempt and prior_bundle must be mappings")
    public_output = finalize_attempt(
        dict(attempt),
        {"public_prediction": prior_bundle.get("public_prediction")},
        frame_index=frame_index,
    )
    if public_output["output_status"] == "fresh":
        next_private_state = reset_private_state(
            public_output,
            frame_index=frame_index,
        )
    else:
        next_private_state = canonical_private_state(
            prior_bundle.get("branch_selection_prior")
        )
    return {
        "public_output": public_output,
        "next_private_state": next_private_state,
    }
