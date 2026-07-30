"""Tagged private-state lifecycle for two-range branch reacquisition."""

from collections.abc import Mapping
from numbers import Integral, Real

import numpy as np

from scripts.diagnostics.predictive_wnls import (
    FRAME_DT_SECONDS,
    canonical_spd_covariance,
    finalize_attempt,
    make_unavailable_output,
    propagate_estimator_prior,
)


METHOD_ID = "two_range_private_branch_reacquisition"
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
