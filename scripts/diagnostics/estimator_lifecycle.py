"""Dependency-leaf public/private state lifecycle for qualified modes."""

from collections.abc import Mapping
from dataclasses import dataclass
import math
from numbers import Integral, Real

import numpy as np


FRAME_DT_SECONDS = 0.5
PROCESS_NOISE_VARIANCE = 0.25
MAX_PUBLIC_PREDICTION_AGE = 2
COVARIANCE_RTOL = 1e-12
COVARIANCE_ATOL = 1e-12
RELATIVE_SPECTRAL_THRESHOLD = 1e-12
PRIVATE_STATE_FIELDS = (
    "status",
    "estimate",
    "modeled_covariance",
    "source_fresh_frame",
    "propagated_to_frame",
    "age_frames",
    "last_command_frame",
    "last_held_velocity",
    "history_version",
)
FORBIDDEN_RUNTIME_KEYS = frozenset({
    "truth_position",
    "future_estimate",
    "analyzer_label",
    "realized_error",
})
FRESH_PUBLIC_FIELDS = frozenset({
    "output_status",
    "estimate",
    "modeled_covariance",
    "epsilon",
    "prediction_age",
    "aged_modeled_radius",
    "base_anchor_provenance",
    "mode_id",
    "reason",
})
PREDICTED_PUBLIC_FIELDS = frozenset({
    "output_status",
    "estimate",
    "modeled_covariance",
    "epsilon",
    "prediction_age",
    "aged_modeled_radius",
    "base_anchor_provenance",
})
UNAVAILABLE_PUBLIC_FIELDS = frozenset({
    "output_status",
    "estimate",
    "modeled_covariance",
    "epsilon",
    "prediction_age",
    "aged_modeled_radius",
    "base_anchor_provenance",
    "reason",
})


@dataclass(frozen=True)
class PriorBundle:
    public_prediction: Mapping | None
    private_prior: Mapping | None
    history_version: int


def reset_private_state(
    candidate: object,
    *,
    frame_index: int,
    history_version: int = 0,
    last_command_frame: int | None = None,
    last_held_velocity: object = None,
) -> dict | None:
    """Create a canonical age-zero private state from one fresh estimate."""
    if not isinstance(candidate, Mapping):
        return None
    estimate = _finite_vector(candidate.get("estimate"))
    covariance = _canonical_spd_covariance(
        candidate.get(
            "modeled_covariance",
            candidate.get("covariance"),
        )
    )
    frame = _nonnegative_integer(frame_index)
    version = _nonnegative_integer(history_version)
    if estimate is None or covariance is None or frame is None or version is None:
        return None
    if last_command_frame is None:
        if last_held_velocity is not None:
            return None
        command_frame = None
        velocity = None
    else:
        command_frame = _nonnegative_integer(last_command_frame)
        velocity_array = _finite_vector(last_held_velocity)
        if (
            command_frame is None
            or frame == 0
            or command_frame != frame - 1
            or velocity_array is None
        ):
            return None
        velocity = velocity_array.tolist()
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": frame,
        "propagated_to_frame": frame,
        "age_frames": 0,
        "last_command_frame": command_frame,
        "last_held_velocity": velocity,
        "history_version": version,
    }


def canonical_private_state(value: object) -> dict | None:
    """Validate and canonicalize the exact private-state schema."""
    if not isinstance(value, Mapping) or set(value) != set(PRIVATE_STATE_FIELDS):
        return None
    if value.get("status") != "available":
        return None
    estimate = _finite_vector(value.get("estimate"))
    covariance = _canonical_spd_covariance(value.get("modeled_covariance"))
    source = _nonnegative_integer(value.get("source_fresh_frame"))
    propagated = _nonnegative_integer(value.get("propagated_to_frame"))
    age = _nonnegative_integer(value.get("age_frames"))
    version = _nonnegative_integer(value.get("history_version"))
    if (
        estimate is None
        or covariance is None
        or source is None
        or propagated is None
        or age is None
        or version is None
        or propagated < source
        or propagated - source != age
    ):
        return None
    raw_command = value.get("last_command_frame")
    raw_velocity = value.get("last_held_velocity")
    if raw_command is None:
        if age != 0 or raw_velocity is not None:
            return None
        command_frame = None
        velocity = None
    else:
        command_frame = _nonnegative_integer(raw_command)
        velocity_array = _finite_vector(raw_velocity)
        if (
            command_frame is None
            or propagated == 0
            or command_frame != propagated - 1
            or velocity_array is None
        ):
            return None
        velocity = velocity_array.tolist()
    return {
        "status": "available",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "source_fresh_frame": source,
        "propagated_to_frame": propagated,
        "age_frames": age,
        "last_command_frame": command_frame,
        "last_held_velocity": velocity,
        "history_version": version,
    }


def _finite_vector(value: object) -> np.ndarray | None:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if vector.shape != (2,) or not np.isfinite(vector).all():
        return None
    return vector


def _canonical_spd_covariance(value: object) -> np.ndarray | None:
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(
            covariance,
            covariance.T,
            rtol=COVARIANCE_RTOL,
            atol=COVARIANCE_ATOL,
        )
    ):
        return None
    canonical = 0.5 * (covariance + covariance.T)
    try:
        eigenvalues = np.linalg.eigvalsh(canonical)
    except np.linalg.LinAlgError:
        return None
    if (
        not np.isfinite(eigenvalues).all()
        or eigenvalues[-1] <= 0.0
        or eigenvalues[0] <= RELATIVE_SPECTRAL_THRESHOLD * eigenvalues[-1]
    ):
        return None
    return canonical


def _nonnegative_integer(value: object) -> int | None:
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        return None
    return int(value)


def _positive_integer(value: object) -> int | None:
    integer = _nonnegative_integer(value)
    return integer if integer is not None and integer > 0 else None


def propagate_private_state(
    previous_state: object,
    held_velocity: object,
    *,
    next_frame_index: int,
    applied_command_frame: int,
    history_version: int,
    mission_horizon_frames: int,
) -> dict | None:
    """Propagate one exact held-command transition or make history absent."""
    state = canonical_private_state(previous_state)
    velocity = _finite_vector(held_velocity)
    next_frame = _nonnegative_integer(next_frame_index)
    command_frame = _nonnegative_integer(applied_command_frame)
    next_version = _nonnegative_integer(history_version)
    horizon = _positive_integer(mission_horizon_frames)
    if (
        state is None
        or velocity is None
        or next_frame is None
        or command_frame is None
        or next_version is None
        or horizon is None
        or state["source_fresh_frame"] >= horizon
        or next_frame >= horizon
        or next_frame != state["propagated_to_frame"] + 1
        or command_frame != next_frame - 1
        or (
            state["last_command_frame"] is not None
            and command_frame != state["last_command_frame"] + 1
        )
        or next_version <= state["history_version"]
    ):
        return None
    age = next_frame - state["source_fresh_frame"]
    if age >= horizon:
        return None
    estimate = np.asarray(state["estimate"], dtype=float)
    covariance = np.asarray(state["modeled_covariance"], dtype=float)
    next_estimate = estimate + FRAME_DT_SECONDS * velocity
    next_covariance = covariance + PROCESS_NOISE_VARIANCE * np.eye(2)
    return {
        "status": "available",
        "estimate": next_estimate.tolist(),
        "modeled_covariance": next_covariance.tolist(),
        "source_fresh_frame": state["source_fresh_frame"],
        "propagated_to_frame": next_frame,
        "age_frames": age,
        "last_command_frame": command_frame,
        "last_held_velocity": velocity.tolist(),
        "history_version": next_version,
    }


def advance_qualified_prior(
    previous_public: object,
    previous_private: object,
    held_velocity: object,
    *,
    next_frame_index: int,
    applied_command_frame: int | None,
    mission_horizon_frames: int,
    history_version: int,
) -> PriorBundle:
    """Advance independent public and private priors by one logged transition."""
    frame = _nonnegative_integer(next_frame_index)
    horizon = _positive_integer(mission_horizon_frames)
    version = _nonnegative_integer(history_version)
    safe_version = 0 if version is None else version
    if frame is None or horizon is None or version is None:
        return PriorBundle(None, None, safe_version)
    if frame == 0:
        return PriorBundle(None, None, version)
    velocity = _finite_vector(held_velocity)
    command_frame = _nonnegative_integer(applied_command_frame)
    if (
        frame >= horizon
        or velocity is None
        or command_frame is None
        or command_frame != frame - 1
    ):
        return PriorBundle(
            _make_unavailable_public("invalid_prior_transition"),
            None,
            version,
        )
    public_prediction = _propagate_public_state(previous_public, velocity)
    private_prior = (
        None
        if previous_private is None
        else propagate_private_state(
            previous_private,
            velocity,
            next_frame_index=frame,
            applied_command_frame=command_frame,
            history_version=version,
            mission_horizon_frames=horizon,
        )
    )
    return PriorBundle(public_prediction, private_prior, version)


def finalize_qualified_lifecycle(
    decision: object,
    prior_bundle: PriorBundle,
    *,
    frame_index: int,
    mission_horizon_frames: int,
) -> dict:
    """Publish a unique fresh mode or retain only already-propagated priors."""
    frame = _nonnegative_integer(frame_index)
    horizon = _positive_integer(mission_horizon_frames)
    if (
        not isinstance(prior_bundle, PriorBundle)
        or frame is None
        or horizon is None
        or frame >= horizon
    ):
        return {
            "public_output": _make_unavailable_public("invalid_lifecycle_input"),
            "next_private_state": None,
            "history_version": 0,
        }
    bundle_version = _nonnegative_integer(prior_bundle.history_version)
    if bundle_version is None:
        return {
            "public_output": _make_unavailable_public("invalid_history_version"),
            "next_private_state": None,
            "history_version": 0,
        }
    retained_private = _retained_private_state(
        prior_bundle.private_prior,
        history_version=bundle_version,
        frame_index=frame,
        mission_horizon_frames=horizon,
    )
    if prior_bundle.private_prior is not None and retained_private is None:
        return {
            "public_output": _make_unavailable_public("invalid_private_prior"),
            "next_private_state": None,
            "history_version": bundle_version,
        }
    status = _decision_field(decision, "status")
    representative = _decision_field(decision, "representative")
    mode_id = _decision_field(decision, "mode_id")
    decision_reason = _decision_field(decision, "reason")
    fresh_state = (
        _candidate_state(representative)
        if (
            status == "fresh"
            and isinstance(mode_id, str)
            and mode_id
            and isinstance(decision_reason, str)
            and decision_reason
        )
        else None
    )
    if fresh_state is not None:
        estimate, covariance, provenance = fresh_state
        inherited_command = (
            None
            if retained_private is None
            else retained_private["last_command_frame"]
        )
        inherited_velocity = (
            None
            if retained_private is None
            else retained_private["last_held_velocity"]
        )
        next_version = bundle_version + 1
        next_private = reset_private_state(
            {
                "estimate": estimate,
                "modeled_covariance": covariance,
            },
            frame_index=frame,
            history_version=next_version,
            last_command_frame=inherited_command,
            last_held_velocity=inherited_velocity,
        )
        if next_private is not None:
            epsilon = 3.0 * math.sqrt(
                float(np.linalg.eigvalsh(np.asarray(covariance))[-1])
            )
            return {
                "public_output": {
                    "output_status": "fresh",
                    "estimate": list(estimate),
                    "modeled_covariance": [list(row) for row in covariance],
                    "epsilon": epsilon,
                    "prediction_age": 0,
                    "aged_modeled_radius": None,
                    "base_anchor_provenance": provenance,
                    "mode_id": mode_id,
                    "reason": decision_reason,
                },
                "next_private_state": next_private,
                "history_version": next_version,
            }
    public_output = _retained_public_prediction(prior_bundle.public_prediction)
    return {
        "public_output": public_output,
        "next_private_state": retained_private,
        "history_version": bundle_version,
    }


def _canonical_public_state(
    value: object,
) -> tuple[str, np.ndarray, np.ndarray, int] | None:
    if (
        not isinstance(value, Mapping)
        or not _runtime_public_payload_is_safe(value)
    ):
        return None
    status = value.get("output_status")
    age = _nonnegative_integer(value.get("prediction_age"))
    if status == "fresh":
        if (
            set(value) != FRESH_PUBLIC_FIELDS
            or age != 0
            or value.get("aged_modeled_radius") is not None
            or not isinstance(value.get("mode_id"), str)
            or not value.get("mode_id")
            or not isinstance(value.get("reason"), str)
            or not value.get("reason")
            or _canonical_provenance(value.get("base_anchor_provenance")) is None
        ):
            return None
    elif status == "predicted":
        if (
            set(value) != PREDICTED_PUBLIC_FIELDS
            or age not in (1, 2)
            or value.get("epsilon") is not None
            or value.get("base_anchor_provenance") != []
        ):
            return None
    else:
        return None
    estimate = _finite_vector(value.get("estimate"))
    covariance = _canonical_spd_covariance(value.get("modeled_covariance"))
    if estimate is None or covariance is None:
        return None
    expected_radius = _modeled_radius(covariance)
    raw_radius = (
        value.get("epsilon")
        if status == "fresh"
        else value.get("aged_modeled_radius")
    )
    radius = _finite_real(raw_radius)
    if (
        radius is None
        or radius <= 0.0
        or abs(radius - expected_radius)
        > COVARIANCE_ATOL + COVARIANCE_RTOL * abs(expected_radius)
    ):
        return None
    return status, estimate, covariance, age


def _propagate_public_state(previous_public: object, velocity: np.ndarray) -> dict:
    state = _canonical_public_state(previous_public)
    if state is None:
        return _make_unavailable_public("no_valid_public_prior")
    status, estimate, covariance, age = state
    next_age = 1 if status == "fresh" else age + 1
    next_estimate = estimate + FRAME_DT_SECONDS * velocity
    next_covariance = covariance + PROCESS_NOISE_VARIANCE * np.eye(2)
    if next_age > MAX_PUBLIC_PREDICTION_AGE:
        return _make_unavailable_public("prediction_expired")
    radius = 3.0 * math.sqrt(float(np.linalg.eigvalsh(next_covariance)[-1]))
    return {
        "output_status": "predicted",
        "estimate": next_estimate.tolist(),
        "modeled_covariance": next_covariance.tolist(),
        "epsilon": None,
        "prediction_age": next_age,
        "aged_modeled_radius": radius,
        "base_anchor_provenance": [],
    }


def _retained_public_prediction(value: object) -> Mapping:
    state = _canonical_public_state(value)
    if state is None or state[0] != "predicted":
        unavailable_reason = _canonical_unavailable_reason(value)
        if unavailable_reason is not None:
            return _make_unavailable_public(unavailable_reason)
        return _make_unavailable_public("no_qualified_public_output")
    _, estimate, covariance, age = state
    radius = 3.0 * math.sqrt(float(np.linalg.eigvalsh(covariance)[-1]))
    return {
        "output_status": "predicted",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "epsilon": None,
        "prediction_age": age,
        "aged_modeled_radius": radius,
        "base_anchor_provenance": [],
    }


def _make_unavailable_public(reason: str) -> dict:
    return {
        "output_status": "unavailable",
        "estimate": None,
        "modeled_covariance": None,
        "epsilon": None,
        "prediction_age": None,
        "aged_modeled_radius": None,
        "base_anchor_provenance": [],
        "reason": reason,
    }


def _canonical_unavailable_reason(value: object) -> str | None:
    if (
        not isinstance(value, Mapping)
        or set(value) != UNAVAILABLE_PUBLIC_FIELDS
        or not _runtime_public_payload_is_safe(value)
        or value.get("output_status") != "unavailable"
        or any(
            value.get(field) is not None
            for field in (
                "estimate",
                "modeled_covariance",
                "epsilon",
                "prediction_age",
                "aged_modeled_radius",
            )
        )
        or value.get("base_anchor_provenance") != []
        or not isinstance(value.get("reason"), str)
        or not value.get("reason")
    ):
        return None
    return value["reason"]


def _runtime_public_payload_is_safe(value: object) -> bool:
    if isinstance(value, Mapping):
        return all(
            isinstance(key, str)
            and key not in FORBIDDEN_RUNTIME_KEYS
            and _runtime_public_payload_is_safe(nested)
            for key, nested in value.items()
        )
    if isinstance(value, (tuple, list)):
        return all(_runtime_public_payload_is_safe(item) for item in value)
    if value is None or isinstance(value, (str, bool)):
        return True
    return _finite_real(value) is not None


def _finite_real(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    scalar = float(value)
    return scalar if math.isfinite(scalar) else None


def _canonical_provenance(value: object) -> list[int] | None:
    if not isinstance(value, list):
        return None
    roots = []
    for item in value:
        root = _nonnegative_integer(item)
        if root is None:
            return None
        roots.append(root)
    return roots if roots == sorted(set(roots)) else None


def _modeled_radius(covariance: np.ndarray) -> float:
    return 3.0 * math.sqrt(float(np.linalg.eigvalsh(covariance)[-1]))


def _retained_private_state(
    value: object,
    *,
    history_version: int,
    frame_index: int,
    mission_horizon_frames: int,
) -> Mapping | None:
    canonical = canonical_private_state(value)
    if (
        canonical is None
        or canonical["history_version"] != history_version
        or canonical["propagated_to_frame"] != frame_index
        or canonical["source_fresh_frame"] >= mission_horizon_frames
        or canonical["age_frames"] >= mission_horizon_frames
    ):
        return None
    return value


def _candidate_state(
    representative: object,
) -> tuple[list[float], list[list[float]], list[int]] | None:
    estimate = _finite_vector(getattr(representative, "estimate", None))
    payload = getattr(representative, "payload", None)
    if not isinstance(payload, Mapping):
        return None
    has_covariance = "covariance" in payload
    has_modeled = "modeled_covariance" in payload
    if has_covariance == has_modeled:
        return None
    covariance = _canonical_spd_covariance(
        payload["covariance"] if has_covariance else payload["modeled_covariance"]
    )
    if estimate is None or covariance is None:
        return None
    raw_provenance = payload.get("base_anchor_provenance", ())
    if not isinstance(raw_provenance, (tuple, list)):
        return None
    provenance = []
    for root in raw_provenance:
        canonical_root = _nonnegative_integer(root)
        if canonical_root is None:
            return None
        provenance.append(canonical_root)
    return estimate.tolist(), covariance.tolist(), sorted(set(provenance))


def _decision_field(decision: object, field: str) -> object:
    if isinstance(decision, Mapping):
        return decision.get(field)
    return getattr(decision, field, None)
