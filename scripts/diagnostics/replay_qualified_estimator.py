"""Exact raw-row producer for the qualified global-mode estimator."""

from collections.abc import Mapping
from numbers import Integral, Real
import math

import numpy as np

from scripts.diagnostics.predictive_wnls import solve_qualified_multistart


RAW_SCHEMA_ID = "cbf2026-qualified-mode-hybrid-dcbf-estimator-raw-v1"
METHOD_ID = "qualified-mode-hybrid-dcbf-estimator-v1"
ROW_FIELDS = (
    "schema_id",
    "method_id",
    "frame_index",
    "robot_id",
    "squad_local_index",
    "schedule_id",
    "runtime_inputs",
    "local_candidate_count",
    "mode_count",
    "mode_members",
    "nonseparable_chain",
    "qualification_kind",
    "qualifications",
    "admissible_mode_count",
    "published_mode_id",
    "sensitivity_count_0_5_mm",
    "sensitivity_count_1_mm",
    "sensitivity_count_2_mm",
    "primary_tolerance_m",
    "public_status",
    "public_age",
    "private_age",
    "history_version",
    "audit_bundle",
)
AUDIT_FIELDS = (
    "runtime_inputs",
    "starts",
    "solver_attempts",
    "local_candidates",
    "clustering",
    "representatives",
    "transition_inputs",
    "prior_bundle",
    "qualifier_context",
    "qualifications",
    "sensitivity",
    "decision",
    "lifecycle",
)
SENSITIVITY_KEYS = {
    "sensitivity_count_0_5_mm": 0.0005.hex(),
    "sensitivity_count_1_mm": 0.001.hex(),
    "sensitivity_count_2_mm": 0.002.hex(),
}
SOLVER_RESULT_FIELDS = {
    "status", "estimate", "covariance", "epsilon", "phi_min_eigenvalue",
    "phi_condition", "fim_valid", "proposal_count", "iterations", "cost",
    "stationarity_norm", "failure_reason", "proposal_trace",
}
LOCAL_DIAGNOSTIC_FIELDS = {
    "active_reference_count", "base_anchor_provenance", "failure_reason",
    "geometry_failure_reason",
    "recomputed_covariance", "recomputed_estimate", "recomputed_fim",
    "recomputed_gauss_newton", "recomputed_objective",
    "recomputed_residual", "recomputed_residual_cost",
    "recomputed_stationarity_norm", "reduced_whitened_cost",
    "solver_geometry_consistent",
}


def build_qualified_replay_row(
    *,
    frame_index,
    robot_id,
    squad_local_index,
    schedule_id,
    references,
    solver_from_start,
    live_seed,
    private_seed,
    qualifier_kind,
    qualifier_payload,
    previous_public,
    previous_private,
    held_velocity,
    applied_command_frame,
    history_version,
    mission_horizon_frames,
    active_reference_count,
    base_anchor_provenance,
) -> dict:
    """Return one independently replayable scheduled estimator row."""
    _validate_schedule_identity(
        frame_index,
        robot_id,
        squad_local_index,
        schedule_id,
    )
    audit = solve_qualified_multistart(
        references=references,
        solver_from_start=solver_from_start,
        live_seed=live_seed,
        private_seed=private_seed,
        qualifier_kind=qualifier_kind,
        qualifier_payload=qualifier_payload,
        previous_public=previous_public,
        previous_private=previous_private,
        held_velocity=held_velocity,
        frame_index=frame_index,
        applied_command_frame=applied_command_frame,
        history_version=history_version,
        mission_horizon_frames=mission_horizon_frames,
        active_reference_count=active_reference_count,
        base_anchor_provenance=base_anchor_provenance,
    )
    clustering = audit["clustering"]
    lifecycle = audit["lifecycle"]
    public = lifecycle["public_output"]
    next_private = lifecycle["next_private_state"]
    qualifications = audit["qualifications"]
    sensitivity = audit["sensitivity"]
    row = {
        "schema_id": RAW_SCHEMA_ID,
        "method_id": METHOD_ID,
        "frame_index": int(frame_index),
        "robot_id": int(robot_id),
        "squad_local_index": int(squad_local_index),
        "schedule_id": schedule_id,
        "runtime_inputs": audit["runtime_inputs"],
        "local_candidate_count": len(audit["local_candidates"]),
        "mode_count": clustering["mode_count"],
        "mode_members": [
            list(mode["member_ids"]) for mode in clustering["modes"]
        ],
        "nonseparable_chain": (
            clustering["reason"] == "nonseparable_chain"
        ),
        "qualification_kind": qualifier_kind,
        "qualifications": qualifications,
        "admissible_mode_count": sum(
            qualification["admissible"] for qualification in qualifications
        ),
        "published_mode_id": audit["decision"]["mode_id"],
        "sensitivity_count_0_5_mm": sensitivity[
            SENSITIVITY_KEYS["sensitivity_count_0_5_mm"]
        ],
        "sensitivity_count_1_mm": sensitivity[
            SENSITIVITY_KEYS["sensitivity_count_1_mm"]
        ],
        "sensitivity_count_2_mm": sensitivity[
            SENSITIVITY_KEYS["sensitivity_count_2_mm"]
        ],
        "primary_tolerance_m": clustering["tolerance_m"],
        "public_status": public["output_status"],
        "public_age": public["prediction_age"],
        "private_age": (
            None if next_private is None else next_private["age_frames"]
        ),
        "history_version": lifecycle["history_version"],
        "audit_bundle": audit,
    }
    validate_qualified_replay_row(row)
    return row


def validate_qualified_replay_row(row: Mapping) -> None:
    """Validate the producer-side exact row protocol without recomputation."""
    if not isinstance(row, Mapping) or tuple(row) != ROW_FIELDS:
        raise ValueError("qualified row differs from exact field order")
    if row["schema_id"] != RAW_SCHEMA_ID:
        raise ValueError("qualified row schema identity is invalid")
    if row["method_id"] != METHOD_ID:
        raise ValueError("qualified row method identity is invalid")
    _validate_schedule_identity(
        row["frame_index"],
        row["robot_id"],
        row["squad_local_index"],
        row["schedule_id"],
    )
    audit = row["audit_bundle"]
    if not isinstance(audit, Mapping) or tuple(audit) != AUDIT_FIELDS:
        raise ValueError("qualified audit bundle differs from exact schema")
    _validate_exact_audit_schema(audit)
    if row["runtime_inputs"] != audit["runtime_inputs"]:
        raise ValueError("qualified runtime input projection is inconsistent")
    for field in (
        "local_candidate_count",
        "mode_count",
        "admissible_mode_count",
        "sensitivity_count_0_5_mm",
        "sensitivity_count_1_mm",
        "sensitivity_count_2_mm",
        "history_version",
    ):
        if not _nonnegative_int(row[field]):
            raise ValueError(f"{field} is invalid")
    if not isinstance(row["mode_members"], list) or any(
        not isinstance(members, list)
        or any(not isinstance(member, str) or not member for member in members)
        for members in row["mode_members"]
    ):
        raise ValueError("mode members are invalid")
    if not isinstance(row["nonseparable_chain"], bool):
        raise ValueError("nonseparable chain flag is invalid")
    if row["qualification_kind"] not in {
        "deployment",
        "history",
        "unavailable",
    }:
        raise ValueError("qualification kind is invalid")
    if not isinstance(row["qualifications"], list):
        raise ValueError("qualifications are invalid")
    for qualification in row["qualifications"]:
        if (
            not isinstance(qualification, Mapping)
            or tuple(qualification) != (
                "mode_id",
                "admissible",
                "reason",
                "score",
            )
            or not isinstance(qualification["mode_id"], str)
            or not qualification["mode_id"]
            or not isinstance(qualification["admissible"], bool)
            or not isinstance(qualification["reason"], str)
            or not qualification["reason"]
            or (
                qualification["score"] is not None
                and _finite_number(qualification["score"]) is None
            )
        ):
            raise ValueError("qualification record is invalid")
    if row["published_mode_id"] is not None and (
        not isinstance(row["published_mode_id"], str)
        or not row["published_mode_id"]
    ):
        raise ValueError("published mode ID is invalid")
    if row["primary_tolerance_m"] != 0.001:
        raise ValueError("primary tolerance is invalid")
    if row["public_status"] not in {"fresh", "predicted", "unavailable"}:
        raise ValueError("public status is invalid")
    if row["public_age"] is not None and not _nonnegative_int(row["public_age"]):
        raise ValueError("public age is invalid")
    if row["private_age"] is not None and not _nonnegative_int(row["private_age"]):
        raise ValueError("private age is invalid")
    _reject_forbidden_runtime_fields(row)


def _validate_exact_audit_schema(audit):
    runtime = audit["runtime_inputs"]
    if not isinstance(runtime, Mapping) or set(runtime) != {
        "references", "live_seed", "private_seed", "active_reference_count",
        "base_anchor_provenance",
    }:
        raise ValueError("runtime inputs differ from exact schema")
    references = runtime["references"]
    if not isinstance(references, list) or len(references) < 2 or any(
        not isinstance(reference, Mapping)
        or set(reference) != {
            "key", "position", "range", "covariance", "ranging_sigma",
            "base_anchor_provenance",
        }
        for reference in references
    ):
        raise ValueError("runtime references differ from exact schema")
    reference_keys = []
    for reference in references:
        key = reference["key"]
        provenance = reference["base_anchor_provenance"]
        if (
            not isinstance(key, list)
            or len(key) != 2
            or not isinstance(key[0], str)
            or not key[0]
            or not _nonnegative_int(key[1])
            or _finite_vec2(reference["position"]) is None
            or _finite_nonnegative(reference["range"]) is None
            or not _finite_psd_2x2(reference["covariance"])
            or _finite_positive(reference["ranging_sigma"]) is None
            or not _canonical_provenance(provenance)
            or (key[0] == "base" and provenance != [key[1]])
            or (key[0] != "base" and len(provenance) < 2)
        ):
            raise ValueError("runtime reference value is invalid")
        reference_keys.append(tuple(key))
    if (
        len(set(reference_keys)) != len(reference_keys)
        or reference_keys != sorted(reference_keys)
    ):
        raise ValueError("runtime reference order is invalid")
    if len({float(item["ranging_sigma"]).hex() for item in references}) != 1:
        raise ValueError("runtime ranging sigma is inconsistent")
    aggregate_provenance = runtime["base_anchor_provenance"]
    derived_provenance = sorted({
        anchor
        for reference in references
        for anchor in reference["base_anchor_provenance"]
    })
    if (
        not _canonical_provenance(aggregate_provenance)
        or len(aggregate_provenance) < 2
        or aggregate_provenance != derived_provenance
    ):
        raise ValueError("base anchor provenance is invalid")
    if (
        not _nonnegative_int(runtime["active_reference_count"])
        or runtime["active_reference_count"] != len(references)
    ):
        raise ValueError("active reference count is invalid")
    starts = audit["starts"]
    if not isinstance(starts, list) or any(
        not isinstance(start, Mapping)
        or set(start) != {
            "attempt_id", "kind", "estimate", "reference_keys", "branch",
        }
        for start in starts
    ):
        raise ValueError("start records differ from exact schema")
    attempts = audit["solver_attempts"]
    if not isinstance(attempts, list):
        raise ValueError("solver attempts differ from exact schema")
    for attempt in attempts:
        if not isinstance(attempt, Mapping) or set(attempt) != {
            "attempt_id", "start_record", "solver_result", "local_eligible",
            "local_eligibility_reason", "local_diagnostics",
        }:
            raise ValueError("solver attempt differs from exact schema")
        if (
            not isinstance(attempt["attempt_id"], str)
            or not attempt["attempt_id"]
            or not isinstance(attempt["local_eligible"], bool)
            or not isinstance(attempt["local_eligibility_reason"], str)
        ):
            raise ValueError("solver attempt scalar is invalid")
        if (
            not isinstance(attempt["solver_result"], Mapping)
            or set(attempt["solver_result"]) != SOLVER_RESULT_FIELDS
        ):
            raise ValueError("solver result differs from exact schema")
        trace = attempt["solver_result"]["proposal_trace"]
        if not isinstance(trace, list) or any(
            not isinstance(proposal, Mapping)
            or set(proposal) != {
                "proposal", "damping", "cost", "stationarity_norm",
                "raw_step_norm", "trial_cost", "invalid_trial_reason",
                "accepted",
            }
            for proposal in trace
        ):
            raise ValueError("proposal trace differs from exact schema")
        if (
            not isinstance(attempt["local_diagnostics"], Mapping)
            or set(attempt["local_diagnostics"]) != LOCAL_DIAGNOSTIC_FIELDS
        ):
            raise ValueError("local diagnostics differ from exact schema")
        local = attempt["local_diagnostics"]
        if (
            not _nonnegative_int(local["active_reference_count"])
            or not _canonical_provenance(local["base_anchor_provenance"])
            or local["base_anchor_provenance"] != aggregate_provenance
            or not isinstance(local["solver_geometry_consistent"], bool)
        ):
            raise ValueError("local diagnostics values are invalid")
    candidates = audit["local_candidates"]
    representatives = audit["representatives"]
    for name, records in (
        ("local candidates", candidates),
        ("representatives", representatives),
    ):
        if not isinstance(records, list) or any(
            not isinstance(record, Mapping)
            or set(record) != {
                "attempt_id", "estimate", "objective_cost", "payload",
            }
            or not isinstance(record["attempt_id"], str)
            or not record["attempt_id"]
            or _finite_vec2(record["estimate"]) is None
            or _finite_nonnegative(record["objective_cost"]) is None
            or not isinstance(record["payload"], Mapping)
            for record in records
        ):
            raise ValueError(f"{name} differ from exact schema")
    clustering = audit["clustering"]
    if not isinstance(clustering, Mapping) or set(clustering) != {
        "tolerance_m", "separable", "reason", "mode_count", "modes",
    }:
        raise ValueError("clustering differs from exact schema")
    if (
        _finite_nonnegative(clustering["tolerance_m"]) is None
        or not isinstance(clustering["separable"], bool)
        or not isinstance(clustering["reason"], str)
        or not _nonnegative_int(clustering["mode_count"])
    ):
        raise ValueError("clustering scalar fields are invalid")
    if not isinstance(clustering["modes"], list) or any(
        not isinstance(mode, Mapping)
        or set(mode) != {"mode_id", "member_ids", "diameter_m"}
        or not isinstance(mode["mode_id"], str)
        or not mode["mode_id"]
        or not isinstance(mode["member_ids"], list)
        or any(
            not isinstance(member, str) or not member
            for member in mode["member_ids"]
        )
        or _finite_nonnegative(mode["diameter_m"]) is None
        for mode in clustering["modes"]
    ):
        raise ValueError("cluster diameter or mode record is invalid")
    transition = audit["transition_inputs"]
    if not isinstance(transition, Mapping) or set(transition) != {
        "previous_public", "previous_private", "held_velocity", "frame_index",
        "applied_command_frame", "history_version", "mission_horizon_frames",
    }:
        raise ValueError("transition inputs differ from exact schema")
    prior = audit["prior_bundle"]
    if not isinstance(prior, Mapping) or set(prior) != {
        "public_prediction", "private_prior", "history_version",
    }:
        raise ValueError("prior bundle differs from exact schema")
    context = audit["qualifier_context"]
    if not isinstance(context, Mapping) or set(context) != {
        "qualifier_kind", "qualifier_payload", "propagated_private_prior",
    }:
        raise ValueError("qualifier context differs from exact schema")
    audit_qualifications = audit["qualifications"]
    if not isinstance(audit_qualifications, list) or any(
        not isinstance(item, Mapping)
        or set(item) != {"mode_id", "admissible", "reason", "score"}
        or not isinstance(item["mode_id"], str)
        or not item["mode_id"]
        or not isinstance(item["admissible"], bool)
        or not isinstance(item["reason"], str)
        or (
            item["score"] is not None
            and _finite_number(item["score"]) is None
        )
        for item in audit_qualifications
    ):
        raise ValueError("audit qualifications differ from exact schema")
    if not isinstance(audit["sensitivity"], Mapping) or set(
        audit["sensitivity"]
    ) != set(SENSITIVITY_KEYS.values()):
        raise ValueError("sensitivity record differs from exact schema")
    if any(
        not _nonnegative_int(count)
        for count in audit["sensitivity"].values()
    ):
        raise ValueError("sensitivity count is invalid")
    decision = audit["decision"]
    if not isinstance(decision, Mapping) or set(decision) != {
        "status", "reason", "mode_id", "representative_attempt_id",
    }:
        raise ValueError("decision differs from exact schema")
    lifecycle = audit["lifecycle"]
    if not isinstance(lifecycle, Mapping) or set(lifecycle) != {
        "public_output", "next_private_state", "history_version",
    }:
        raise ValueError("lifecycle differs from exact schema")


def _validate_schedule_identity(frame, robot, local, schedule_id):
    if not all(_nonnegative_int(value) for value in (frame, robot, local)):
        raise ValueError("qualified schedule indices are invalid")
    expected = f"frame-{int(frame)}:robot-{int(robot)}:squad-local-{int(local)}"
    if schedule_id != expected:
        raise ValueError("qualified schedule identity is invalid")


def _nonnegative_int(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, Integral)
        and value >= 0
    )


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _finite_nonnegative(value):
    number = _finite_number(value)
    return number if number is not None and number >= 0.0 else None


def _finite_positive(value):
    number = _finite_number(value)
    return number if number is not None and number > 0.0 else None


def _finite_vec2(value):
    if not isinstance(value, list) or len(value) != 2:
        return None
    result = [_finite_number(item) for item in value]
    return result if all(item is not None for item in result) else None


def _finite_psd_2x2(value):
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(covariance, covariance.T, rtol=1e-12, atol=1e-12)
    ):
        return False
    try:
        eigenvalues = np.linalg.eigvalsh(0.5 * (covariance + covariance.T))
    except np.linalg.LinAlgError:
        return False
    return bool(
        np.isfinite(eigenvalues).all()
        and eigenvalues[0] >= -1e-12 * max(1.0, eigenvalues[-1])
    )


def _canonical_provenance(value):
    return (
        isinstance(value, list)
        and all(_nonnegative_int(item) for item in value)
        and value == sorted(set(value))
    )


def _reject_forbidden_runtime_fields(value, active_ids=None):
    if active_ids is None:
        active_ids = set()
    if isinstance(value, Mapping):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified row payload")
        active_ids.add(identifier)
        try:
            for key, nested in value.items():
                if not isinstance(key, str):
                    raise ValueError("qualified row key is invalid")
                lowered = key.lower()
                if any(fragment in lowered for fragment in (
                    "truth",
                    "analyzer",
                    "future",
                    "realized",
                )):
                    raise ValueError("forbidden qualified runtime field")
                _reject_forbidden_runtime_fields(nested, active_ids)
        finally:
            active_ids.remove(identifier)
    elif isinstance(value, list):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified row payload")
        active_ids.add(identifier)
        try:
            for item in value:
                _reject_forbidden_runtime_fields(item, active_ids)
        finally:
            active_ids.remove(identifier)
