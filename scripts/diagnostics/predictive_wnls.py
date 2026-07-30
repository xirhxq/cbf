"""Bounded public prediction lifecycle for diagnostic WNLS estimates."""

import math
from collections.abc import Mapping

import numpy as np


FRAME_DT_SECONDS = 0.5
MAX_PUBLIC_PREDICTION_AGE = 2
OUTPUT_STATUSES = ("fresh", "predicted", "unavailable")
ATTEMPT_STATUSES = (
    "accepted",
    "rejected",
    "failed",
    "invalid",
    "reference_unavailable",
)


def _finite_vector(value: object) -> np.ndarray | None:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if vector.shape != (2,) or not np.isfinite(vector).all():
        return None
    return vector


def _finite_spd_covariance(value: object) -> np.ndarray | None:
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.array_equal(covariance, covariance.T)
    ):
        return None
    try:
        eigenvalues = np.linalg.eigvalsh(covariance)
    except np.linalg.LinAlgError:
        return None
    if not np.isfinite(eigenvalues).all() or np.any(eigenvalues <= 0.0):
        return None
    return covariance


def _finite_estimate_and_covariance(source: object) -> tuple[np.ndarray, np.ndarray] | None:
    if not isinstance(source, Mapping):
        return None
    estimate = _finite_vector(source.get("estimate"))
    covariance = _finite_spd_covariance(source.get("modeled_covariance"))
    if estimate is None or covariance is None:
        return None
    return estimate, covariance


def make_unavailable_output(reason: str) -> dict:
    """Return a public unavailable output with no finite localization fields."""
    return {
        "output_status": "unavailable",
        "estimate": None,
        "modeled_covariance": None,
        "epsilon": None,
        "prediction_age": None,
        "aged_modeled_radius": None,
        "base_anchor_provenance": [],
        "reason": str(reason),
    }


def output_is_fresh(output: dict | None) -> bool:
    """Validate a current-frame fresh public output."""
    if not isinstance(output, Mapping) or output.get("output_status") != "fresh":
        return False
    if output.get("prediction_age") != 0:
        return False
    state = _finite_estimate_and_covariance(output)
    if state is None:
        return False
    try:
        epsilon = float(output.get("epsilon"))
    except (TypeError, ValueError, OverflowError):
        return False
    return math.isfinite(epsilon) and epsilon >= 0.0


def reference_is_eligible(output: dict | None) -> bool:
    """Accept only finite fresh outputs with at least two base roots."""
    if not output_is_fresh(output):
        return False
    provenance = output.get("base_anchor_provenance")
    if not isinstance(provenance, (list, tuple)):
        return False
    try:
        return len(set(provenance)) >= 2
    except TypeError:
        return False


def _private_seed(estimate: np.ndarray, covariance: np.ndarray) -> dict:
    return {
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
    }


def propagate_estimator_prior(
    previous_output: dict | None,
    previous_private_seed: dict | None,
    held_velocity: object,
    *,
    dt: float = FRAME_DT_SECONDS,
) -> dict:
    """Return public_prediction and private_reacquisition_seed for one transition."""
    velocity = _finite_vector(held_velocity)
    try:
        dt_value = float(dt)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError("dt must be finite") from error
    if velocity is None or not math.isfinite(dt_value):
        raise ValueError("held_velocity and dt must be finite two-dimensional values")

    previous_is_unavailable = (
        isinstance(previous_output, Mapping)
        and previous_output.get("output_status") == "unavailable"
    )
    source = previous_private_seed if previous_is_unavailable else previous_output
    state = _finite_estimate_and_covariance(source)
    if state is None:
        return {
            "public_prediction": make_unavailable_output("no_finite_prior"),
            "private_reacquisition_seed": None,
        }

    estimate, covariance = state
    next_estimate = estimate + dt_value * velocity
    next_covariance = covariance + 0.25 * np.eye(2)
    private_seed = _private_seed(next_estimate, next_covariance)

    if previous_is_unavailable:
        return {
            "public_prediction": make_unavailable_output("previously_unavailable"),
            "private_reacquisition_seed": private_seed,
        }

    prior_age = previous_output.get("prediction_age") if isinstance(previous_output, Mapping) else None
    if previous_output.get("output_status") == "fresh" and prior_age == 0:
        next_age = 1
    elif previous_output.get("output_status") == "predicted" and prior_age in (1, 2):
        next_age = prior_age + 1
    else:
        return {
            "public_prediction": make_unavailable_output("invalid_prior"),
            "private_reacquisition_seed": private_seed,
        }

    if next_age > MAX_PUBLIC_PREDICTION_AGE:
        return {
            "public_prediction": make_unavailable_output("prediction_expired"),
            "private_reacquisition_seed": private_seed,
        }

    radius = 3.0 * math.sqrt(float(np.max(np.linalg.eigvalsh(next_covariance))))
    return {
        "public_prediction": {
            "output_status": "predicted",
            "estimate": next_estimate.tolist(),
            "modeled_covariance": next_covariance.tolist(),
            "epsilon": None,
            "prediction_age": next_age,
            "aged_modeled_radius": radius,
            "base_anchor_provenance": [],
        },
        "private_reacquisition_seed": private_seed,
    }


def finalize_attempt(
    attempt: dict,
    prior_bundle: dict,
    *,
    frame_index: int,
) -> dict:
    """Publish fresh after acceptance, otherwise predicted or unavailable by age."""
    if not isinstance(attempt, Mapping) or not isinstance(prior_bundle, Mapping):
        raise ValueError("attempt and prior_bundle must be mappings")
    if not isinstance(frame_index, int) or isinstance(frame_index, bool) or frame_index < 0:
        raise ValueError("frame_index must be a non-negative integer")

    status = attempt.get("attempt_status", attempt.get("status"))
    if status == "accepted":
        candidate = attempt.get("candidate", attempt.get("output"))
        if not isinstance(candidate, Mapping):
            raise ValueError("accepted attempt requires a candidate")
        state = _finite_estimate_and_covariance(candidate)
        if state is None:
            raise ValueError("accepted candidate must be finite with SPD covariance")
        estimate, covariance = state
        try:
            epsilon = float(candidate.get("epsilon"))
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("accepted candidate requires finite epsilon") from error
        if not math.isfinite(epsilon) or epsilon < 0.0:
            raise ValueError("accepted candidate requires finite epsilon")
        provenance = candidate.get("base_anchor_provenance", [])
        if not isinstance(provenance, (list, tuple)):
            raise ValueError("candidate provenance must be a sequence")
        return {
            "output_status": "fresh",
            "estimate": estimate.tolist(),
            "modeled_covariance": covariance.tolist(),
            "epsilon": epsilon,
            "prediction_age": 0,
            "aged_modeled_radius": None,
            "base_anchor_provenance": list(provenance),
        }

    prediction = prior_bundle.get("public_prediction")
    if (
        isinstance(prediction, Mapping)
        and prediction.get("output_status") == "predicted"
        and prediction.get("prediction_age") in (1, 2)
        and _finite_estimate_and_covariance(prediction) is not None
    ):
        return dict(prediction)
    return make_unavailable_output("attempt_not_accepted")
