"""Bounded public prediction lifecycle for diagnostic WNLS estimates."""

import math
from collections.abc import Mapping
from dataclasses import asdict
from numbers import Integral, Real

import numpy as np

from scripts.diagnostics.replay_localization_calibration import (
    _linearized_terms,
    _validated_inputs,
    fim_radius,
)
from scripts.diagnostics.qualified_modes import (
    MODE_TOLERANCE_M,
    DeploymentContract,
    LocalCandidate,
    ModeQualification,
    canonical_mode_id,
    cluster_candidates,
    enumerate_qualified_starts,
    project_local_candidate,
    publish_unique_mode,
    qualify_all,
    select_representative,
    sensitivity_cluster_counts,
    stable_attempt_id,
)
from scripts.diagnostics.estimator_lifecycle import (
    FRESH_PUBLIC_FIELDS,
    PREDICTED_PUBLIC_FIELDS,
    UNAVAILABLE_PUBLIC_FIELDS,
    PriorBundle,
    _canonical_public_state,
    _canonical_unavailable_reason,
    advance_qualified_prior,
    canonical_private_state,
    finalize_qualified_lifecycle,
)


FRAME_DT_SECONDS = 0.5
MAX_PUBLIC_PREDICTION_AGE = 2
CANDIDATE_DEDUP_M = 1e-9
RELATIVE_TIE_TOLERANCE = 1e-12
PUBLIC_COVARIANCE_RTOL = 1e-12
PUBLIC_COVARIANCE_ATOL = 1e-12
QUALIFICATION_VARIANTS = (
    "prediction_expiry",
    "fresh_reference_qualification",
    "predictive_multistart",
)
OUTPUT_STATUSES = ("fresh", "predicted", "unavailable")
ATTEMPT_STATUSES = (
    "accepted",
    "rejected",
    "failed",
    "invalid",
    "reference_unavailable",
)
MAX_WNLS_PROPOSALS = 50
INITIAL_WNLS_DAMPING = 1e-3
MIN_WNLS_DAMPING = 1e-15
MAX_WNLS_DAMPING = 1e15
WNLS_DAMPING_FACTOR = 10.0
STATIONARITY_SCALE = 1e-6
REPRESENTABLE_STEP_SCALE = 1e-12
INNOVATION_REFERENCE_QUANTILE = 11.829007011943707
RELATIVE_SPECTRAL_THRESHOLD = 1e-12
_CANDIDATE_SOURCE_ORDER = {
    "prediction": 0,
    "private_reacquisition_seed": 0,
    "algebraic": 1,
    "circle_negative": 2,
    "circle_positive": 3,
}
_PROPOSAL_TRACE_FIELDS = {
    "proposal",
    "damping",
    "cost",
    "stationarity_norm",
    "raw_step_norm",
    "trial_cost",
    "invalid_trial_reason",
    "accepted",
}
_QUALIFIED_PROPOSAL_TRACE_FIELDS = (
    "proposal",
    "damping",
    "cost",
    "stationarity_norm",
    "raw_step_norm",
    "trial_cost",
    "invalid_trial_reason",
    "accepted",
)
_CONVERGED_TRACE_INVALID_REASONS = {
    "invalid_trial_terms",
    "non-finite_trial_cost",
}


def _finite_vector(value: object) -> np.ndarray | None:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if vector.shape != (2,) or not np.isfinite(vector).all():
        return None
    return vector


def canonical_spd_covariance(value: object) -> np.ndarray | None:
    """Return the exact symmetric SPD representative of a finite covariance."""
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
            rtol=PUBLIC_COVARIANCE_RTOL,
            atol=PUBLIC_COVARIANCE_ATOL,
        )
    ):
        return None
    canonical = 0.5 * (covariance + covariance.T)
    try:
        eigenvalues = np.linalg.eigvalsh(canonical)
    except np.linalg.LinAlgError:
        return None
    if not np.isfinite(eigenvalues).all() or np.any(eigenvalues <= 0.0):
        return None
    return canonical


def _finite_spd_covariance(value: object) -> np.ndarray | None:
    return canonical_spd_covariance(value)


def _finite_estimate_and_covariance(source: object) -> tuple[np.ndarray, np.ndarray] | None:
    if not isinstance(source, Mapping):
        return None
    estimate = _finite_vector(source.get("estimate"))
    covariance = _finite_spd_covariance(source.get("modeled_covariance"))
    if estimate is None or covariance is None:
        return None
    return estimate, covariance


def _canonical_base_provenance(value: object) -> list[int] | None:
    if not isinstance(value, (list, tuple)):
        return None
    roots = []
    for root in value:
        if isinstance(root, bool) or not isinstance(root, Integral) or root < 0:
            return None
        roots.append(int(root))
    distinct_roots = sorted(set(roots))
    if len(distinct_roots) < 2:
        return None
    return distinct_roots


def _canonical_prediction(output: object) -> dict | None:
    if not isinstance(output, Mapping) or output.get("output_status") != "predicted":
        return None
    age = output.get("prediction_age")
    if (
        isinstance(age, bool)
        or not isinstance(age, Integral)
        or age not in (1, 2)
    ):
        return None
    state = _finite_estimate_and_covariance(output)
    if state is None:
        return None
    estimate, covariance = state
    radius = 3.0 * math.sqrt(float(np.max(np.linalg.eigvalsh(covariance))))
    return {
        "output_status": "predicted",
        "estimate": estimate.tolist(),
        "modeled_covariance": covariance.tolist(),
        "epsilon": None,
        "prediction_age": int(age),
        "aged_modeled_radius": radius,
        "base_anchor_provenance": [],
    }


def _prediction_is_canonical(output: object) -> bool:
    canonical = _canonical_prediction(output)
    if canonical is None or not isinstance(output, Mapping):
        return False
    try:
        supplied_radius = float(output.get("aged_modeled_radius"))
    except (TypeError, ValueError, OverflowError):
        return False
    return (
        output.get("epsilon") is None
        and output.get("base_anchor_provenance") == []
        and math.isfinite(supplied_radius)
        and math.isclose(
            supplied_radius,
            canonical["aged_modeled_radius"],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
    )


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
    prediction_age = output.get("prediction_age")
    if (
        isinstance(prediction_age, bool)
        or not isinstance(prediction_age, Integral)
        or prediction_age != 0
    ):
        return False
    state = _finite_estimate_and_covariance(output)
    if state is None:
        return False
    try:
        epsilon = float(output.get("epsilon"))
    except (TypeError, ValueError, OverflowError):
        return False
    return (
        math.isfinite(epsilon)
        and epsilon >= 0.0
        and _canonical_base_provenance(
            output.get("base_anchor_provenance")
        )
        is not None
    )


def reference_is_eligible(output: dict | None) -> bool:
    """Accept only finite fresh outputs with at least two base roots."""
    return output_is_fresh(output)


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
    previous_is_fresh = output_is_fresh(previous_output)
    previous_is_prediction = _prediction_is_canonical(previous_output)
    if not (
        previous_is_unavailable
        or previous_is_fresh
        or previous_is_prediction
    ):
        return {
            "public_prediction": make_unavailable_output("invalid_prior"),
            "private_reacquisition_seed": None,
        }

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

    prior_age = previous_output.get("prediction_age")
    if previous_is_fresh:
        next_age = 1
    elif previous_is_prediction:
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

    raw_status = attempt.get("attempt_status", attempt.get("status"))
    status = (
        raw_status
        if isinstance(raw_status, str) and raw_status in ATTEMPT_STATUSES
        else "invalid"
    )
    if status == "accepted":
        candidate = attempt.get("candidate", attempt.get("output"))
        state = _finite_estimate_and_covariance(candidate)
        provenance = (
            None
            if not isinstance(candidate, Mapping)
            else _canonical_base_provenance(
                candidate.get("base_anchor_provenance")
            )
        )
        if state is not None and provenance is not None:
            estimate, covariance = state
            epsilon = 3.0 * math.sqrt(
                float(np.linalg.eigvalsh(covariance)[-1])
            )
            return {
                "attempt_status": "accepted",
                "output_status": "fresh",
                "estimate": estimate.tolist(),
                "modeled_covariance": covariance.tolist(),
                "epsilon": epsilon,
                "prediction_age": 0,
                "aged_modeled_radius": None,
                "base_anchor_provenance": provenance,
            }
        status = "invalid"

    prediction = _canonical_prediction(prior_bundle.get("public_prediction"))
    if prediction is not None:
        prediction["attempt_status"] = status
        return prediction
    unavailable = make_unavailable_output("attempt_not_accepted")
    unavailable["attempt_status"] = status
    return unavailable


def _reference_key(value: object) -> tuple[str, int] | None:
    if not isinstance(value, tuple) or len(value) != 2:
        return None
    kind, identifier = value
    if (
        kind not in ("base", "uav")
        or isinstance(identifier, bool)
        or not isinstance(identifier, Integral)
        or identifier < 0
    ):
        return None
    return kind, int(identifier)


def _reference_key_order(key: tuple[str, int]) -> tuple[int, int]:
    return (0 if key[0] == "base" else 1, key[1])


def _present_scalar_record(record: object) -> dict | None:
    if not isinstance(record, Mapping) or record.get("present") is not True:
        return None
    raw_range = record.get("noisy_range")
    if isinstance(raw_range, bool) or not isinstance(raw_range, Real):
        return None
    noisy_range = float(raw_range)
    if not math.isfinite(noisy_range) or noisy_range < 0.0:
        return None
    # Copy only sensor-boundary fields so offline/truth fields cannot leak on.
    return {"present": True, "noisy_range": noisy_range}


def merge_base_anchor_provenance(
    direct_base_ids: object,
    uav_outputs: object,
) -> tuple[int, ...]:
    """Return sorted unique current-frame recursive base roots."""
    roots: set[int] = set()
    if isinstance(direct_base_ids, (list, tuple)):
        for identifier in direct_base_ids:
            if (
                not isinstance(identifier, bool)
                and isinstance(identifier, Integral)
                and identifier >= 0
            ):
                roots.add(int(identifier))
    if isinstance(uav_outputs, Mapping):
        for output in uav_outputs.values():
            if reference_is_eligible(output):
                provenance = _canonical_base_provenance(
                    output.get("base_anchor_provenance")
                )
                if provenance is not None:
                    roots.update(provenance)
    return tuple(sorted(roots))


def qualify_active_references(
    *,
    mandatory: dict[str, list[int]],
    optional_keys: object,
    measurement_records: dict[tuple[str, int], dict],
    uav_outputs: dict[int, dict],
    variant: str,
) -> dict:
    """Return active measurement/reference records or reference_unavailable."""
    invalid_result = {
        "status": "invalid",
        "active_keys": [],
        "active_records": [],
        "missing_mandatory": [],
        "excluded": [],
        "violations": [],
        "base_anchor_provenance": (),
    }
    if (
        not isinstance(mandatory, dict)
        or set(mandatory) != {"base_ids", "uav_ids"}
        or variant not in QUALIFICATION_VARIANTS
        or not isinstance(optional_keys, list)
        or not isinstance(measurement_records, Mapping)
        or not isinstance(uav_outputs, Mapping)
    ):
        return invalid_result
    base_ids = mandatory["base_ids"]
    uav_ids = mandatory["uav_ids"]
    if not isinstance(base_ids, list) or not isinstance(uav_ids, list):
        return invalid_result
    mandatory_keys: list[tuple[str, int]] = []
    for kind, identifiers in (("base", base_ids), ("uav", uav_ids)):
        for identifier in identifiers:
            if (
                isinstance(identifier, bool)
                or not isinstance(identifier, Integral)
                or identifier < 0
            ):
                return invalid_result
            mandatory_keys.append((kind, int(identifier)))
    if len(set(mandatory_keys)) != len(mandatory_keys):
        return invalid_result
    optional = [_reference_key(raw) for raw in optional_keys]
    if any(key is None for key in optional):
        return invalid_result
    optional = [key for key in optional if key is not None]
    if len(set(optional)) != len(optional):
        return invalid_result
    mandatory_keys = sorted(mandatory_keys, key=_reference_key_order)
    optional = sorted(optional, key=_reference_key_order)
    records = measurement_records
    outputs = uav_outputs
    active_keys: list[tuple[str, int]] = []
    active_records: list[dict] = []
    missing_mandatory: list[tuple[str, int]] = []
    excluded: list[dict] = []
    violations: list[dict] = []

    def add_active(key: tuple[str, int], record: dict) -> None:
        active_keys.append(key)
        active_records.append({"key": key, **record})

    for key in mandatory_keys:
        record = _present_scalar_record(records.get(key))
        if record is None:
            missing_mandatory.append(key)
            continue
        if key[0] == "uav" and variant != "prediction_expiry":
            if not reference_is_eligible(outputs.get(key[1])):
                missing_mandatory.append(key)
                continue
        add_active(key, record)
        if key[0] == "uav" and not reference_is_eligible(outputs.get(key[1])):
            violations.append({"key": key, "reason": "stale_or_predicted_anchor_used"})

    if missing_mandatory:
        return {
            "status": "reference_unavailable",
            "active_keys": active_keys,
            "active_records": active_records,
            "missing_mandatory": missing_mandatory,
            "excluded": excluded,
            "violations": violations,
            "base_anchor_provenance": merge_base_anchor_provenance(
                [key[1] for key in active_keys if key[0] == "base"],
                {key[1]: outputs[key[1]] for key in active_keys if key[0] == "uav" and key[1] in outputs},
            ),
        }

    for key in optional:
        if key in mandatory_keys:
            continue
        record = _present_scalar_record(records.get(key))
        if record is None:
            excluded.append({"key": key, "reason": "measurement_not_present"})
            continue
        if key[0] == "uav" and variant != "prediction_expiry":
            if not reference_is_eligible(outputs.get(key[1])):
                excluded.append({"key": key, "reason": "not_current_frame_fresh"})
                continue
        add_active(key, record)
        if key[0] == "uav" and not reference_is_eligible(outputs.get(key[1])):
            violations.append({"key": key, "reason": "stale_or_predicted_anchor_used"})

    active_pairs = sorted(zip(active_keys, active_records), key=lambda pair: _reference_key_order(pair[0]))
    active_keys = [key for key, _ in active_pairs]
    active_records = [record for _, record in active_pairs]
    active_uav_outputs = {
        key[1]: outputs[key[1]]
        for key in active_keys
        if key[0] == "uav" and key[1] in outputs
    }
    violations.sort(
        key=lambda record: (
            _reference_key_order(record["key"]),
            record["reason"],
        )
    )
    return {
        "status": "ok",
        "active_keys": active_keys,
        "active_records": active_records,
        "missing_mandatory": [],
        "excluded": excluded,
        "violations": violations,
        "base_anchor_provenance": merge_base_anchor_provenance(
            [key[1] for key in active_keys if key[0] == "base"],
            active_uav_outputs,
        ),
    }


def _geometry_inputs(
    reference_positions: object,
    measured_ranges: object,
    reference_keys: object | None = None,
) -> tuple[np.ndarray, np.ndarray, list[tuple[str, int]]] | None:
    try:
        positions = np.asarray(reference_positions, dtype=float)
        ranges = np.asarray(measured_ranges, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        positions.ndim != 2
        or positions.shape[1:] != (2,)
        or ranges.shape != (positions.shape[0],)
        or not np.isfinite(positions).all()
        or not np.isfinite(ranges).all()
        or np.any(ranges < 0.0)
    ):
        return None
    if reference_keys is None:
        keys = [("base", index) for index in range(len(positions))]
    elif isinstance(reference_keys, (list, tuple)) and len(reference_keys) == len(positions):
        keys = [_reference_key(value) for value in reference_keys]
        if any(key is None for key in keys):
            return None
    else:
        return None
    return positions, ranges, keys


def algebraic_multilateration_candidate(
    reference_positions: object,
    measured_ranges: object,
) -> np.ndarray | None:
    """Return one deterministic finite least-squares point."""
    geometry = _geometry_inputs(reference_positions, measured_ranges)
    if geometry is None:
        return None
    positions, ranges, _ = geometry
    if len(positions) < 3:
        return None
    origin = positions[0]
    coefficients = 2.0 * (positions[1:] - origin)
    values = (
        np.sum(positions[1:] ** 2, axis=1) - ranges[1:] ** 2
        - (float(np.dot(origin, origin)) - ranges[0] ** 2)
    )
    try:
        candidate, _, rank, _ = np.linalg.lstsq(coefficients, values, rcond=None)
    except np.linalg.LinAlgError:
        return None
    if rank < 2 or candidate.shape != (2,) or not np.isfinite(candidate).all():
        return None
    return candidate


def best_conditioned_pair(
    observer_center: object | None,
    reference_positions: object,
    measured_ranges: object,
    reference_keys: object,
) -> tuple[int, int] | None:
    """Use predicted directions or re-acquisition cosine-law geometry."""
    geometry = _geometry_inputs(reference_positions, measured_ranges, reference_keys)
    if geometry is None:
        return None
    positions, ranges, keys = geometry
    center = None if observer_center is None else _finite_vector(observer_center)
    if observer_center is not None and center is None:
        return None
    best: tuple[float, tuple[tuple[int, int], tuple[int, int]], tuple[int, int]] | None = None
    for first in range(len(positions)):
        for second in range(first + 1, len(positions)):
            delta = positions[second] - positions[first]
            baseline = float(np.linalg.norm(delta))
            if not math.isfinite(baseline) or baseline == 0.0:
                continue
            if center is not None:
                direction_a = positions[first] - center
                direction_b = positions[second] - center
                norm_a = float(np.linalg.norm(direction_a))
                norm_b = float(np.linalg.norm(direction_b))
                if norm_a == 0.0 or norm_b == 0.0:
                    continue
                score = abs(float(np.cross(direction_a / norm_a, direction_b / norm_b)))
            else:
                if ranges[first] == 0.0 or ranges[second] == 0.0:
                    continue
                cosine = (ranges[first] ** 2 + ranges[second] ** 2 - baseline ** 2) / (2.0 * ranges[first] * ranges[second])
                if not math.isfinite(cosine) or abs(cosine) > 1.0 + RELATIVE_TIE_TOLERANCE:
                    continue
                score = math.sqrt(max(0.0, 1.0 - float(cosine) ** 2))
            ordered_keys = tuple(sorted((_reference_key_order(keys[first]), _reference_key_order(keys[second]))))
            pair = tuple(
                sorted(
                    (first, second),
                    key=lambda index: _reference_key_order(keys[index]),
                )
            )
            if best is None:
                best = (score, ordered_keys, pair)
                continue
            tolerance = RELATIVE_TIE_TOLERANCE * max(1.0, abs(score), abs(best[0]))
            if score > best[0] + tolerance or (abs(score - best[0]) <= tolerance and ordered_keys < best[1]):
                best = (score, ordered_keys, pair)
    return None if best is None else best[2]


def two_circle_candidates(
    first_position: object,
    first_range: float,
    second_position: object,
    second_range: float,
) -> tuple[np.ndarray, ...]:
    """Return negative-oriented then positive-oriented branches."""
    first = _finite_vector(first_position)
    second = _finite_vector(second_position)
    try:
        radius_a, radius_b = float(first_range), float(second_range)
    except (TypeError, ValueError, OverflowError):
        return ()
    if (
        first is None or second is None or not math.isfinite(radius_a) or not math.isfinite(radius_b)
        or radius_a < 0.0 or radius_b < 0.0
    ):
        return ()
    offset = second - first
    baseline = float(np.linalg.norm(offset))
    if baseline == 0.0 or not math.isfinite(baseline) or baseline > radius_a + radius_b or baseline < abs(radius_a - radius_b):
        return ()
    along = (radius_a ** 2 - radius_b ** 2 + baseline ** 2) / (2.0 * baseline)
    height_sq = radius_a ** 2 - along ** 2
    if height_sq < -CANDIDATE_DEDUP_M:
        return ()
    height = math.sqrt(max(0.0, height_sq))
    direction = offset / baseline
    midpoint = first + along * direction
    perpendicular = np.array([-direction[1], direction[0]])
    negative = midpoint - height * perpendicular
    positive = midpoint + height * perpendicular
    if not np.isfinite(negative).all() or not np.isfinite(positive).all():
        return ()
    return (negative, positive)


def initial_candidates(
    *,
    live_prediction: dict | None,
    private_seed: dict | None,
    reference_positions: object,
    measured_ranges: object,
    reference_keys: object,
) -> tuple[dict, ...]:
    """Return at most four deterministic deduplicated candidates."""
    geometry = _geometry_inputs(reference_positions, measured_ranges, reference_keys)
    if geometry is None:
        return ()
    positions, ranges, keys = geometry
    candidates: list[dict] = []

    def append(source: str, estimate: object) -> None:
        point = _finite_vector(estimate)
        if point is None or len(candidates) >= 4:
            return
        if any(float(np.linalg.norm(point - np.asarray(candidate["estimate"], dtype=float))) <= CANDIDATE_DEDUP_M for candidate in candidates):
            return
        candidates.append({"source": source, "estimate": point.tolist()})

    prediction = _finite_estimate_and_covariance(live_prediction)
    if prediction is not None:
        append("prediction", prediction[0])
        center = prediction[0]
    else:
        private = _finite_estimate_and_covariance(private_seed)
        if private is not None:
            append("private_reacquisition_seed", private[0])
        center = None
    algebraic = algebraic_multilateration_candidate(positions, ranges)
    if algebraic is not None:
        append("algebraic", algebraic)
    pair = best_conditioned_pair(center, positions, ranges, keys)
    if pair is not None:
        branches = two_circle_candidates(positions[pair[0]], ranges[pair[0]], positions[pair[1]], ranges[pair[1]])
        if branches:
            append("circle_negative", branches[0])
        if len(branches) > 1:
            append("circle_positive", branches[1])
    return tuple(candidates)


def scale_aware_stationary(gradient: object, residual: object) -> bool:
    """Return whether the frozen, scale-aware stationarity rule is met."""
    try:
        gradient_array = np.asarray(gradient, dtype=float)
        residual_array = np.asarray(residual, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    if (
        gradient_array.shape != (2,)
        or residual_array.ndim != 1
        or residual_array.size == 0
        or not np.isfinite(gradient_array).all()
        or not np.isfinite(residual_array).all()
    ):
        return False
    return bool(
        np.linalg.norm(gradient_array, ord=np.inf)
        <= STATIONARITY_SCALE * (1.0 + np.linalg.norm(residual_array))
    )


def _finite_relative_spd(value: object) -> np.ndarray | None:
    """Symmetrize a finite 2-D covariance and apply the frozen spectral rule."""
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if matrix.shape != (2, 2) or not np.isfinite(matrix).all():
        return None
    symmetric = 0.5 * (matrix + matrix.T)
    try:
        eigenvalues = np.linalg.eigvalsh(symmetric)
    except np.linalg.LinAlgError:
        return None
    if (
        not np.isfinite(eigenvalues).all()
        or eigenvalues[-1] <= 0.0
        or eigenvalues[0] <= RELATIVE_SPECTRAL_THRESHOLD * eigenvalues[-1]
    ):
        return None
    return symmetric


def _solver_result(
    status: str,
    *,
    estimate: np.ndarray | None,
    cost: float | None,
    stationarity_norm: float | None,
    proposal_trace: list[dict],
    failure_reason: str | None = None,
    fim: dict | None = None,
) -> dict:
    """Construct a self-contained finite-budget WNLS result."""
    result = {
        "status": status,
        "estimate": None if estimate is None else estimate.tolist(),
        "covariance": None,
        "epsilon": None,
        "phi_min_eigenvalue": None,
        "phi_condition": None,
        "fim_valid": False,
        "proposal_count": len(proposal_trace),
        "iterations": len(proposal_trace),
        "cost": cost,
        "stationarity_norm": stationarity_norm,
        "failure_reason": failure_reason,
        "proposal_trace": proposal_trace,
    }
    if fim is not None:
        for key in (
            "covariance",
            "epsilon",
            "phi_min_eigenvalue",
            "phi_condition",
        ):
            value = fim.get(key)
            if key == "covariance":
                try:
                    covariance = np.asarray(value, dtype=float)
                except (TypeError, ValueError, OverflowError):
                    covariance = np.asarray([], dtype=float)
                if covariance.shape == (2, 2) and np.isfinite(covariance).all():
                    value = (0.5 * (covariance + covariance.T)).tolist()
            result[key] = value
        result["fim_valid"] = fim.get("status") == "converged"
    return result


def solve_finite_budget_wnls(
    reference_positions: object,
    reference_covariances: object,
    measurements: object,
    initial_estimate: object,
    ranging_sigma: float,
) -> dict:
    """Run the unchanged residual/Jacobian with the frozen finite LM policy."""
    validated = _validated_inputs(
        reference_positions,
        reference_covariances,
        measurements,
        initial_estimate,
        ranging_sigma,
    )
    if validated is None:
        return _solver_result(
            "invalid",
            estimate=None,
            cost=None,
            stationarity_norm=None,
            proposal_trace=[],
            failure_reason="non-finite or malformed WNLS input",
        )
    references, covariances, ranges, estimate, sigma = validated
    if ranges is None:
        return _solver_result(
            "invalid",
            estimate=None,
            cost=None,
            stationarity_norm=None,
            proposal_trace=[],
            failure_reason="measurements are required",
        )

    current = estimate.copy()
    damping = INITIAL_WNLS_DAMPING
    proposal_trace: list[dict] = []
    for proposal in range(MAX_WNLS_PROPOSALS):
        terms = _linearized_terms(current, references, covariances, ranges, sigma)
        if terms is None:
            return _solver_result(
                "invalid",
                estimate=current,
                cost=None,
                stationarity_norm=None,
                proposal_trace=proposal_trace,
                failure_reason="invalid linearized WNLS terms",
            )
        _, _, residual, gauss_newton, gradient, cost = terms
        stationarity_norm = float(np.linalg.norm(gradient, ord=np.inf))
        if scale_aware_stationary(gradient, residual):
            fim = fim_radius(current, references, covariances, sigma)
            if fim.get("status") != "converged":
                return _solver_result(
                    "invalid",
                    estimate=current,
                    cost=cost,
                    stationarity_norm=stationarity_norm,
                    proposal_trace=proposal_trace,
                    failure_reason=fim.get("failure_reason"),
                    fim=fim,
                )
            return _solver_result(
                "converged",
                estimate=current,
                cost=cost,
                stationarity_norm=stationarity_norm,
                proposal_trace=proposal_trace,
                fim=fim,
            )

        row = {
            "proposal": proposal,
            "damping": damping,
            "cost": cost,
            "stationarity_norm": stationarity_norm,
            "raw_step_norm": None,
            "trial_cost": None,
            "invalid_trial_reason": None,
            "accepted": False,
        }
        try:
            delta = -np.linalg.solve(gauss_newton + damping * np.eye(2), gradient)
        except np.linalg.LinAlgError:
            row["invalid_trial_reason"] = "damped_normal_equations_unsolvable"
            proposal_trace.append(row)
            return _solver_result(
                "invalid",
                estimate=current,
                cost=cost,
                stationarity_norm=stationarity_norm,
                proposal_trace=proposal_trace,
                failure_reason=row["invalid_trial_reason"],
            )
        if delta.shape != (2,) or not np.isfinite(delta).all():
            row["invalid_trial_reason"] = "non-finite damped step"
            proposal_trace.append(row)
            return _solver_result(
                "invalid",
                estimate=current,
                cost=cost,
                stationarity_norm=stationarity_norm,
                proposal_trace=proposal_trace,
                failure_reason=row["invalid_trial_reason"],
            )
        raw_step_norm = float(np.linalg.norm(delta))
        row["raw_step_norm"] = raw_step_norm
        if raw_step_norm <= REPRESENTABLE_STEP_SCALE * (1.0 + np.linalg.norm(current)):
            row["invalid_trial_reason"] = "no_representable_improving_step"
            proposal_trace.append(row)
            return _solver_result(
                "failed",
                estimate=current,
                cost=cost,
                stationarity_norm=stationarity_norm,
                proposal_trace=proposal_trace,
                failure_reason="no_representable_improving_step",
            )

        trial = current + delta
        trial_terms = (
            None
            if not np.isfinite(trial).all()
            else _linearized_terms(trial, references, covariances, ranges, sigma)
        )
        if trial_terms is None:
            row["invalid_trial_reason"] = "invalid_trial_terms"
        else:
            trial_cost = float(trial_terms[-1])
            if math.isfinite(trial_cost):
                row["trial_cost"] = trial_cost
                if trial_cost < cost:
                    row["accepted"] = True
                    proposal_trace.append(row)
                    current = trial
                    trial_residual = trial_terms[2]
                    trial_gradient = trial_terms[4]
                    trial_stationarity_norm = float(
                        np.linalg.norm(trial_gradient, ord=np.inf)
                    )
                    if scale_aware_stationary(trial_gradient, trial_residual):
                        fim = fim_radius(current, references, covariances, sigma)
                        if fim.get("status") != "converged":
                            return _solver_result(
                                "invalid",
                                estimate=current,
                                cost=trial_cost,
                                stationarity_norm=trial_stationarity_norm,
                                proposal_trace=proposal_trace,
                                failure_reason=fim.get("failure_reason"),
                                fim=fim,
                            )
                        return _solver_result(
                            "converged",
                            estimate=current,
                            cost=trial_cost,
                            stationarity_norm=trial_stationarity_norm,
                            proposal_trace=proposal_trace,
                            fim=fim,
                        )
                    damping = max(MIN_WNLS_DAMPING, damping / WNLS_DAMPING_FACTOR)
                    continue
            else:
                row["invalid_trial_reason"] = "non-finite_trial_cost"
        proposal_trace.append(row)
        if damping > MAX_WNLS_DAMPING / WNLS_DAMPING_FACTOR:
            return _solver_result(
                "failed",
                estimate=current,
                cost=cost,
                stationarity_norm=stationarity_norm,
                proposal_trace=proposal_trace,
                failure_reason="maximum_damping_exceeded",
            )
        damping *= WNLS_DAMPING_FACTOR

    terms = _linearized_terms(current, references, covariances, ranges, sigma)
    cost = None if terms is None else float(terms[-1])
    stationarity_norm = (
        None if terms is None else float(np.linalg.norm(terms[-2], ord=np.inf))
    )
    return _solver_result(
        "failed",
        estimate=current,
        cost=cost,
        stationarity_norm=stationarity_norm,
        proposal_trace=proposal_trace,
        failure_reason="maximum_proposals_exhausted",
    )


def normalized_innovation(
    candidate_estimate: object,
    candidate_covariance: object,
    live_prediction: dict,
) -> dict:
    """Return the finite normalized innovation using the summed covariance."""
    candidate = _finite_vector(candidate_estimate)
    candidate_covariance_array = _finite_relative_spd(candidate_covariance)
    prediction = (
        _finite_vector(live_prediction.get("estimate"))
        if isinstance(live_prediction, Mapping)
        else None
    )
    prediction_covariance = (
        _finite_relative_spd(live_prediction.get("modeled_covariance"))
        if isinstance(live_prediction, Mapping)
        else None
    )
    if (
        candidate is None
        or candidate_covariance_array is None
        or prediction is None
        or prediction_covariance is None
    ):
        return {
            "valid": False,
            "q_innov": None,
            "failure_reason": "invalid_innovation_input",
        }
    covariance_sum = 0.5 * (
        candidate_covariance_array
        + prediction_covariance
        + (candidate_covariance_array + prediction_covariance).T
    )
    covariance_sum = _finite_relative_spd(covariance_sum)
    if covariance_sum is None:
        return {
            "valid": False,
            "q_innov": None,
            "failure_reason": "innovation_covariance_not_positive_definite",
        }
    innovation = candidate - prediction
    try:
        solved = np.linalg.solve(covariance_sum, innovation)
        q_innov = float(innovation @ solved)
    except np.linalg.LinAlgError:
        return {
            "valid": False,
            "q_innov": None,
            "failure_reason": "innovation_covariance_solve_failed",
        }
    if not math.isfinite(q_innov):
        return {
            "valid": False,
            "q_innov": None,
            "failure_reason": "non-finite_normalized_innovation",
        }
    return {
        "valid": True,
        "q_innov": q_innov,
        "failure_reason": None,
    }


def _finite_nonnegative_scalar(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    scalar = float(value)
    if not math.isfinite(scalar) or scalar < 0.0:
        return None
    return scalar


def _same_recomputed_cost(first: float, second: float) -> bool:
    tolerance = RELATIVE_TIE_TOLERANCE * max(1.0, abs(first), abs(second))
    return abs(first - second) <= tolerance


def _complete_converged_solver_result(result: object) -> bool:
    """Validate the complete, internally consistent converged-result schema."""
    if not isinstance(result, Mapping):
        return False
    required = {
        "status",
        "estimate",
        "covariance",
        "epsilon",
        "phi_min_eigenvalue",
        "phi_condition",
        "fim_valid",
        "proposal_count",
        "iterations",
        "cost",
        "stationarity_norm",
        "failure_reason",
        "proposal_trace",
    }
    if not required.issubset(result):
        return False
    if (
        result.get("status") != "converged"
        or result.get("failure_reason") is not None
        or result.get("fim_valid") is not True
    ):
        return False
    estimate = _finite_vector(result.get("estimate"))
    covariance = _finite_spd_covariance(result.get("covariance"))
    epsilon = _finite_nonnegative_scalar(result.get("epsilon"))
    phi_minimum = _finite_nonnegative_scalar(result.get("phi_min_eigenvalue"))
    phi_condition = _finite_nonnegative_scalar(result.get("phi_condition"))
    cost = _finite_nonnegative_scalar(result.get("cost"))
    stationarity_norm = _finite_nonnegative_scalar(result.get("stationarity_norm"))
    if (
        estimate is None
        or covariance is None
        or epsilon is None
        or epsilon <= 0.0
        or phi_minimum is None
        or phi_minimum <= 0.0
        or phi_condition is None
        or phi_condition < 1.0
        or cost is None
        or stationarity_norm is None
    ):
        return False
    covariance_eigenvalues = np.linalg.eigvalsh(covariance)
    if (
        covariance_eigenvalues[0]
        <= RELATIVE_SPECTRAL_THRESHOLD * covariance_eigenvalues[-1]
    ):
        return False
    expected_epsilon = 3.0 * math.sqrt(float(covariance_eigenvalues[-1]))
    expected_phi_minimum = 1.0 / float(covariance_eigenvalues[-1])
    expected_phi_condition = float(
        covariance_eigenvalues[-1] / covariance_eigenvalues[0]
    )
    if not (
        math.isclose(epsilon, expected_epsilon, rel_tol=1e-9, abs_tol=1e-12)
        and math.isclose(
            phi_minimum,
            expected_phi_minimum,
            rel_tol=1e-9,
            abs_tol=1e-12,
        )
        and math.isclose(
            phi_condition,
            expected_phi_condition,
            rel_tol=1e-9,
            abs_tol=1e-12,
        )
    ):
        return False
    proposal_count = result.get("proposal_count")
    iterations = result.get("iterations")
    trace = result.get("proposal_trace")
    if (
        isinstance(proposal_count, bool)
        or not isinstance(proposal_count, Integral)
        or isinstance(iterations, bool)
        or not isinstance(iterations, Integral)
        or not isinstance(trace, list)
        or proposal_count != iterations
        or proposal_count != len(trace)
        or proposal_count < 0
        or proposal_count > MAX_WNLS_PROPOSALS
    ):
        return False
    previous_damping: float | None = None
    previous_cost: float | None = None
    previous_trial_cost: float | None = None
    previous_accepted: bool | None = None
    for index, row in enumerate(trace):
        if not isinstance(row, Mapping) or set(row) != _PROPOSAL_TRACE_FIELDS:
            return False
        proposal = row.get("proposal")
        if (
            isinstance(proposal, bool)
            or not isinstance(proposal, Integral)
            or proposal != index
        ):
            return False
        damping = _finite_nonnegative_scalar(row.get("damping"))
        row_cost = _finite_nonnegative_scalar(row.get("cost"))
        row_stationarity = _finite_nonnegative_scalar(row.get("stationarity_norm"))
        raw_step_norm = _finite_nonnegative_scalar(row.get("raw_step_norm"))
        if (
            damping is None
            or damping < MIN_WNLS_DAMPING
            or damping > MAX_WNLS_DAMPING
            or row_cost is None
            or row_stationarity is None
            or raw_step_norm is None
            or not isinstance(row.get("accepted"), bool)
        ):
            return False
        if index == 0:
            if damping != INITIAL_WNLS_DAMPING:
                return False
        else:
            if (
                previous_damping is None
                or previous_cost is None
                or previous_accepted is None
            ):
                return False
            expected_damping = (
                max(
                    MIN_WNLS_DAMPING,
                    previous_damping / WNLS_DAMPING_FACTOR,
                )
                if previous_accepted
                else previous_damping * WNLS_DAMPING_FACTOR
            )
            expected_cost = (
                previous_trial_cost if previous_accepted else previous_cost
            )
            if (
                damping != expected_damping
                or expected_cost is None
                or not _same_recomputed_cost(row_cost, expected_cost)
            ):
                return False
        trial_cost_value = row.get("trial_cost")
        trial_cost = (
            None
            if trial_cost_value is None
            else _finite_nonnegative_scalar(trial_cost_value)
        )
        invalid_reason = row.get("invalid_trial_reason")
        if trial_cost_value is not None and trial_cost is None:
            return False
        if invalid_reason is not None and invalid_reason not in _CONVERGED_TRACE_INVALID_REASONS:
            return False
        if row["accepted"]:
            if invalid_reason is not None or trial_cost is None or not trial_cost < row_cost:
                return False
        elif trial_cost is None and invalid_reason is None:
            return False
        elif trial_cost is not None and invalid_reason is not None:
            return False
        elif trial_cost is not None and trial_cost < row_cost:
            return False
        previous_damping = damping
        previous_cost = row_cost
        previous_trial_cost = trial_cost
        previous_accepted = row["accepted"]
    if trace:
        terminal = trace[-1]
        if terminal["accepted"] is not True or terminal["trial_cost"] != cost:
            return False
    return True


def recompute_local_candidate_diagnostics(
    result: object,
    *,
    active_reference_count: int,
    base_anchor_provenance: object,
) -> dict:
    """Validate local solver facts without a cross-mode innovation decision."""
    diagnostics: dict = {
        "failure_reason": None,
        "recomputed_result": None,
        "reduced_whitened_cost": None,
    }
    if not isinstance(result, Mapping):
        diagnostics["failure_reason"] = "invalid_candidate_result"
        return diagnostics
    if result.get("status") != "converged":
        diagnostics["failure_reason"] = "candidate_not_numerically_converged"
        return diagnostics
    if not _complete_converged_solver_result(result):
        diagnostics["failure_reason"] = "invalid_candidate_output"
        return diagnostics
    estimate = _finite_vector(result.get("estimate"))
    covariance = _finite_spd_covariance(result.get("covariance"))
    cost = _finite_nonnegative_scalar(result.get("cost"))
    if estimate is None or covariance is None or cost is None:
        diagnostics["failure_reason"] = "invalid_candidate_output"
        return diagnostics
    if result.get("fim_valid") is not True:
        diagnostics["failure_reason"] = "final_fim_not_positive_definite"
        return diagnostics
    if _canonical_base_provenance(base_anchor_provenance) is None:
        diagnostics["failure_reason"] = "insufficient_base_anchor_provenance"
        return diagnostics
    if isinstance(active_reference_count, bool) or not isinstance(
        active_reference_count,
        Integral,
    ):
        diagnostics["failure_reason"] = "invalid_active_reference_count"
        return diagnostics
    active_count = int(active_reference_count)
    if active_count < 2:
        diagnostics["failure_reason"] = "insufficient_active_references"
        return diagnostics
    reduced_cost = cost / max(1, active_count - 2)
    diagnostics["reduced_whitened_cost"] = reduced_cost
    if reduced_cost > 9.0:
        diagnostics["failure_reason"] = "reacquisition_reduced_cost_exceeds_nine"
        return diagnostics
    diagnostics["recomputed_result"] = dict(result)
    return diagnostics


def candidate_local_eligibility(
    result: object,
    *,
    active_reference_count: int,
    base_anchor_provenance: object,
) -> tuple[bool, str, dict]:
    """Validate one local solver result without global-mode qualification."""
    diagnostics = recompute_local_candidate_diagnostics(
        result,
        active_reference_count=active_reference_count,
        base_anchor_provenance=base_anchor_provenance,
    )
    if diagnostics["failure_reason"] is not None:
        return False, diagnostics["failure_reason"], diagnostics
    return True, "locally_eligible", diagnostics


def locally_eligible_candidate(
    result: object,
    *,
    attempt_id: str,
    active_reference_count: int,
    base_anchor_provenance: object,
) -> LocalCandidate | None:
    """Project an eligible result for post-solver global-mode clustering."""
    eligible, _, diagnostics = candidate_local_eligibility(
        result,
        active_reference_count=active_reference_count,
        base_anchor_provenance=base_anchor_provenance,
    )
    if not eligible:
        return None
    return project_local_candidate(attempt_id, diagnostics["recomputed_result"])


def candidate_acceptance(
    candidate_result: dict,
    *,
    live_prediction: dict | None,
    active_reference_count: int,
    base_anchor_provenance: object,
    allow_two_reference_reacquisition: bool = False,
) -> tuple[bool, str, dict]:
    """Apply the frozen numerical, FIM, provenance, and online gates."""
    diagnostics: dict = {
        "innovation_gate": None,
        "q_innov": None,
        "gate_outcome": "invalid",
    }
    if not isinstance(allow_two_reference_reacquisition, bool):
        return (
            False,
            "invalid_two_reference_reacquisition_option",
            diagnostics,
        )
    if not isinstance(candidate_result, Mapping):
        return False, "invalid_candidate_result", diagnostics
    if candidate_result.get("status") != "converged":
        return False, "candidate_not_numerically_converged", diagnostics
    if not _complete_converged_solver_result(candidate_result):
        return False, "invalid_candidate_output", diagnostics
    estimate = _finite_vector(candidate_result.get("estimate"))
    covariance = _finite_spd_covariance(candidate_result.get("covariance"))
    try:
        cost = float(candidate_result.get("cost"))
    except (TypeError, ValueError, OverflowError):
        cost = math.nan
    if estimate is None or covariance is None or not math.isfinite(cost):
        return False, "invalid_candidate_output", diagnostics
    if candidate_result.get("fim_valid") is not True:
        return False, "final_fim_not_positive_definite", diagnostics
    provenance = _canonical_base_provenance(base_anchor_provenance)
    if provenance is None:
        return False, "insufficient_base_anchor_provenance", diagnostics
    if isinstance(active_reference_count, bool) or not isinstance(active_reference_count, Integral):
        return False, "invalid_active_reference_count", diagnostics
    active_count = int(active_reference_count)
    if live_prediction is not None:
        diagnostics["innovation_gate"] = "applied"
        innovation = normalized_innovation(estimate, covariance, live_prediction)
        diagnostics.update(innovation)
        if not innovation["valid"]:
            return False, innovation["failure_reason"], diagnostics
        if innovation["q_innov"] > INNOVATION_REFERENCE_QUANTILE:
            diagnostics["gate_outcome"] = "rejected"
            return False, "innovation_q_exceeds_reference_quantile", diagnostics
        diagnostics["gate_outcome"] = "accepted"
        return True, "accepted", diagnostics

    diagnostics["innovation_gate"] = "not_applicable_reacquisition"
    if active_count < 3 and not (
        allow_two_reference_reacquisition and active_count == 2
    ):
        diagnostics["gate_outcome"] = "rejected"
        return False, "reacquisition_requires_three_active_references", diagnostics
    reduced_cost = cost / max(1, active_count - 2)
    diagnostics["reduced_whitened_cost"] = reduced_cost
    if reduced_cost > 9.0:
        diagnostics["gate_outcome"] = "rejected"
        return False, "reacquisition_reduced_cost_exceeds_nine", diagnostics
    diagnostics["gate_outcome"] = "accepted"
    return True, "accepted", diagnostics


def _candidate_source_rank(source: object) -> int:
    return _CANDIDATE_SOURCE_ORDER.get(source, len(_CANDIDATE_SOURCE_ORDER))


def select_candidate_result(
    candidate_records: object,
    *,
    has_live_prediction: bool,
) -> dict | None:
    """Select accepted candidates by cost, innovation, then fixed source order."""
    if not isinstance(candidate_records, (list, tuple)):
        return None
    selected: dict | None = None
    for record in candidate_records:
        if not isinstance(record, Mapping) or record.get("accepted") is not True:
            continue
        try:
            cost = float(record.get("cost"))
        except (TypeError, ValueError, OverflowError):
            continue
        if not math.isfinite(cost):
            continue
        if has_live_prediction:
            try:
                q_innov = float(record.get("q_innov"))
            except (TypeError, ValueError, OverflowError):
                continue
            if not math.isfinite(q_innov):
                continue
        if selected is None:
            selected = dict(record)
            continue
        selected_cost = float(selected["cost"])
        tolerance = RELATIVE_TIE_TOLERANCE * max(1.0, abs(cost), abs(selected_cost))
        if cost < selected_cost - tolerance:
            selected = dict(record)
            continue
        if abs(cost - selected_cost) > tolerance:
            continue
        if has_live_prediction:
            selected_q = float(selected["q_innov"])
            if q_innov < selected_q:
                selected = dict(record)
                continue
            if q_innov > selected_q:
                continue
        if _candidate_source_rank(record.get("source")) < _candidate_source_rank(selected.get("source")):
            selected = dict(record)
    return selected


def solve_predictive_multistart(
    *,
    reference_positions: object,
    reference_covariances: object,
    measurements: object,
    reference_keys: object,
    live_prediction: dict | None,
    private_seed: dict | None,
    ranging_sigma: float,
    base_anchor_provenance: object,
) -> dict:
    """Solve and retain deterministic starts, candidate traces, and gate outcomes."""
    canonical_live_prediction = None
    if live_prediction is not None:
        if not _prediction_is_canonical(live_prediction):
            return {
                "attempt_status": "invalid",
                "status": "invalid",
                "candidates": [],
                "selected_candidate": None,
                "candidate": None,
                "failure_reason": "invalid_live_prediction",
            }
        canonical_live_prediction = _canonical_prediction(live_prediction)
        if canonical_live_prediction is None:
            return {
                "attempt_status": "invalid",
                "status": "invalid",
                "candidates": [],
                "selected_candidate": None,
                "candidate": None,
                "failure_reason": "invalid_live_prediction",
            }
    candidates = initial_candidates(
        live_prediction=canonical_live_prediction,
        private_seed=private_seed,
        reference_positions=reference_positions,
        measured_ranges=measurements,
        reference_keys=reference_keys,
    )
    records: list[dict] = []
    for candidate in candidates:
        final = solve_finite_budget_wnls(
            reference_positions,
            reference_covariances,
            measurements,
            candidate["estimate"],
            ranging_sigma,
        )
        accepted, reason, diagnostics = candidate_acceptance(
            final,
            live_prediction=canonical_live_prediction,
            active_reference_count=len(reference_positions) if hasattr(reference_positions, "__len__") else -1,
            base_anchor_provenance=base_anchor_provenance,
        )
        record = {
            "source": candidate["source"],
            "initial_estimate": list(candidate["estimate"]),
            "result": final,
            "accepted": accepted,
            "rejection_reason": None if accepted else reason,
            "gate_diagnostics": diagnostics,
            "status": final["status"],
            "estimate": final["estimate"],
            "covariance": final["covariance"],
            "cost": final["cost"],
            "q_innov": diagnostics.get("q_innov"),
        }
        records.append(record)
    selected = select_candidate_result(
        records,
        has_live_prediction=canonical_live_prediction is not None,
    )
    if selected is not None:
        final = selected["result"]
        candidate = {
            "estimate": final["estimate"],
            "modeled_covariance": final["covariance"],
            "epsilon": final["epsilon"],
            "base_anchor_provenance": list(_canonical_base_provenance(base_anchor_provenance) or ()),
        }
        return {
            "attempt_status": "accepted",
            "status": "accepted",
            "candidates": records,
            "selected_candidate": selected,
            "candidate": candidate,
            "failure_reason": None,
        }
    if any(
        record["gate_diagnostics"].get("gate_outcome") == "rejected"
        for record in records
    ):
        attempt_status = "rejected"
    elif any(record["status"] == "failed" for record in records):
        attempt_status = "failed"
    else:
        attempt_status = "invalid"
    return {
        "attempt_status": attempt_status,
        "status": attempt_status,
        "candidates": records,
        "selected_candidate": None,
        "candidate": None,
        "failure_reason": None if records else "no_valid_initial_candidates",
    }


def solve_qualified_multistart(
    *,
    references,
    solver_from_start,
    live_seed,
    private_seed,
    qualifier_kind,
    qualifier_payload,
    previous_public,
    previous_private,
    held_velocity,
    frame_index,
    applied_command_frame,
    history_version,
    mission_horizon_frames,
    active_reference_count,
    base_anchor_provenance,
) -> dict:
    """Solve every stable start and publish only one qualified global mode."""
    validated = _validate_qualified_runtime_inputs(
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
    prior_bundle = advance_qualified_prior(
        previous_public,
        previous_private,
        held_velocity,
        next_frame_index=frame_index,
        applied_command_frame=applied_command_frame,
        history_version=history_version,
        mission_horizon_frames=mission_horizon_frames,
    )
    starts = enumerate_qualified_starts(
        validated["references"],
        live_seed=live_seed,
        private_seed=private_seed,
    )
    bound_solver = solver_from_start.bind_canonical_references(
        validated["references"],
    )
    if not callable(bound_solver):
        raise ValueError("bound qualified solver must be callable")
    raw_solver_results = [bound_solver(start) for start in starts]
    solver_results = [
        _canonical_qualified_solver_result(raw_result)
        for raw_result in raw_solver_results
    ]

    attempts = []
    local_candidates = []
    for start, result in zip(starts, solver_results, strict=True):
        attempt_id = stable_attempt_id(start)
        locally_valid, reason, _ = candidate_local_eligibility(
            result,
            active_reference_count=active_reference_count,
            base_anchor_provenance=base_anchor_provenance,
        )
        if _is_invalid_qualified_solver_result_evidence(result):
            locally_valid = False
            reason = "invalid_solver_result_evidence"
        geometry = _qualified_recompute_solver_geometry(
            result,
            validated["references"],
        )
        eligible = locally_valid and geometry["solver_geometry_consistent"]
        if locally_valid and not geometry["solver_geometry_consistent"]:
            reason = "solver_geometry_mismatch"
        candidate = (
            _qualified_local_candidate(
                attempt_id,
                geometry["verified_result"],
                base_anchor_provenance,
            )
            if eligible
            else None
        )
        if candidate is not None:
            local_candidates.append(candidate)
        attempts.append(
            _serialize_qualified_solver_attempt(
                start,
                result,
                eligible=eligible,
                eligibility_reason=reason,
                geometry=geometry,
                active_reference_count=active_reference_count,
                base_anchor_provenance=base_anchor_provenance,
            )
        )

    clustering = cluster_candidates(local_candidates, MODE_TOLERANCE_M)
    representatives = tuple(
        select_representative(mode) for mode in clustering.modes
    )
    qualifications = _qualify_integrated_modes(
        clustering.modes,
        representatives,
        qualifier_kind=qualifier_kind,
        qualifier_payload=qualifier_payload,
        propagated_private_prior=prior_bundle.private_prior,
    )
    decision = publish_unique_mode(clustering, qualifications)
    lifecycle = finalize_qualified_lifecycle(
        decision,
        prior_bundle,
        frame_index=frame_index,
        mission_horizon_frames=mission_horizon_frames,
    )
    return {
        "runtime_inputs": _qualified_json_value({
            "references": validated["references"],
            "live_seed": live_seed,
            "private_seed": private_seed,
            "active_reference_count": active_reference_count,
            "base_anchor_provenance": base_anchor_provenance,
        }),
        "starts": [_serialize_qualified_start(start) for start in starts],
        "solver_attempts": attempts,
        "local_candidates": [
            _serialize_qualified_candidate(candidate)
            for candidate in local_candidates
        ],
        "clustering": _serialize_qualified_clustering(clustering),
        "representatives": [
            _serialize_qualified_candidate(candidate)
            for candidate in representatives
        ],
        "transition_inputs": _serialize_qualified_transition_inputs(
            previous_public=previous_public,
            previous_private=previous_private,
            held_velocity=held_velocity,
            frame_index=frame_index,
            applied_command_frame=applied_command_frame,
            history_version=history_version,
            mission_horizon_frames=mission_horizon_frames,
        ),
        "prior_bundle": _serialize_qualified_prior_bundle(prior_bundle),
        "qualifier_context": _serialize_qualified_qualifier_context(
            qualifier_kind=qualifier_kind,
            qualifier_payload=qualifier_payload,
            propagated_private_prior=prior_bundle.private_prior,
        ),
        "qualifications": [
            _serialize_mode_qualification(item) for item in qualifications
        ],
        "sensitivity": sensitivity_cluster_counts(local_candidates),
        "decision": _serialize_qualified_publication_decision(decision),
        "lifecycle": _qualified_json_value(lifecycle),
    }


_QUALIFIED_SOLVER_RESULT_FIELDS = (
    "status",
    "estimate",
    "covariance",
    "epsilon",
    "phi_min_eigenvalue",
    "phi_condition",
    "fim_valid",
    "proposal_count",
    "iterations",
    "cost",
    "stationarity_norm",
    "failure_reason",
    "proposal_trace",
)
_QUALIFIED_EVIDENCE_MAX_DEPTH = 8
_QUALIFIED_EVIDENCE_MAX_NODES = 4096
_QUALIFIED_FORBIDDEN_FRAGMENTS = (
    "truth",
    "analyzer",
    "future",
    "realized",
)


class _CanonicalQualifiedSolver:
    def __init__(self, implementation, references=None):
        if not callable(implementation):
            raise ValueError("qualified solver implementation must be callable")
        self._implementation = implementation
        self._references = references

    def bind_canonical_references(self, references):
        canonical = tuple(
            _qualified_json_value(reference) for reference in references
        )
        return _CanonicalQualifiedSolver(self._implementation, canonical)

    def __call__(self, start):
        if self._references is None:
            raise ValueError("qualified solver references are not bound")
        return self._implementation(start, self._references)


def canonical_qualified_solver(implementation):
    """Bind a one-start solver implementation to producer-canonical references."""
    return _CanonicalQualifiedSolver(implementation)


def _validate_qualified_runtime_inputs(**values) -> dict:
    references = values["references"]
    if not isinstance(references, (tuple, list)) or len(references) < 2:
        raise ValueError("qualified references must contain at least two records")
    canonical_references = []
    for reference in references:
        if not isinstance(reference, Mapping) or set(reference) != {
            "key",
            "position",
            "range",
            "covariance",
            "ranging_sigma",
            "base_anchor_provenance",
        }:
            raise ValueError("qualified reference schema is invalid")
        key = reference["key"]
        if (
            not isinstance(key, (tuple, list))
            or len(key) != 2
            or not isinstance(key[0], str)
            or not key[0]
            or isinstance(key[1], bool)
            or not isinstance(key[1], Integral)
            or key[1] < 0
        ):
            raise ValueError("qualified reference key is invalid")
        position = _qualified_finite_vec2(reference["position"])
        radius = _qualified_finite_nonnegative(reference["range"])
        covariance = _qualified_reference_covariance(reference["covariance"])
        ranging_sigma = _qualified_finite_number(reference["ranging_sigma"])
        reference_provenance = _qualified_base_provenance(
            reference["base_anchor_provenance"],
        )
        if (
            position is None
            or radius is None
            or covariance is None
            or ranging_sigma is None
            or ranging_sigma <= 0.0
            or reference_provenance is None
            or (
                key[0] == "base"
                and reference_provenance != (int(key[1]),)
            )
            or (key[0] != "base" and len(reference_provenance) < 2)
        ):
            raise ValueError("qualified reference geometry is invalid")
        canonical_references.append({
            "key": (key[0], int(key[1])),
            "position": position,
            "range": radius,
            "covariance": covariance,
            "ranging_sigma": ranging_sigma,
            "base_anchor_provenance": list(reference_provenance),
        })
    keys = [reference["key"] for reference in canonical_references]
    if len(set(keys)) != len(keys):
        raise ValueError("qualified reference keys must be unique")
    sigmas = {
        reference["ranging_sigma"].hex()
        for reference in canonical_references
    }
    if len(sigmas) != 1:
        raise ValueError("qualified references must share one ranging sigma")
    solver = values["solver_from_start"]
    if (
        not callable(solver)
        or not callable(getattr(solver, "bind_canonical_references", None))
    ):
        raise ValueError("solver_from_start must bind canonical references")
    for name in ("live_seed", "private_seed"):
        seed = values[name]
        if seed is None:
            continue
        if not isinstance(seed, Mapping) or set(seed) != {
            "estimate",
            "modeled_covariance",
        }:
            raise ValueError(f"{name} schema is invalid")
        if (
            _qualified_finite_vec2(seed["estimate"]) is None
            or canonical_spd_covariance(seed["modeled_covariance"]) is None
            or _qualified_contains_forbidden(seed)
        ):
            raise ValueError(f"{name} is invalid")
    previous_public = values["previous_public"]
    if previous_public is not None:
        if not isinstance(previous_public, Mapping):
            raise ValueError("previous public state is invalid")
        status = previous_public.get("output_status")
        expected = {
            "fresh": FRESH_PUBLIC_FIELDS,
            "predicted": PREDICTED_PUBLIC_FIELDS,
            "unavailable": UNAVAILABLE_PUBLIC_FIELDS,
        }.get(status)
        if expected is None or set(previous_public) != set(expected):
            raise ValueError("previous public state schema is invalid")
        if (
            status in {"fresh", "predicted"}
            and _canonical_public_state(previous_public) is None
        ) or (
            status == "unavailable"
            and _canonical_unavailable_reason(previous_public) is None
        ):
            raise ValueError("previous public state is invalid")
        _qualified_json_value(previous_public)
    previous_private = values["previous_private"]
    if previous_private is not None:
        if canonical_private_state(previous_private) is None:
            raise ValueError("previous private state is invalid")
        _qualified_json_value(previous_private)
    held_velocity = _qualified_finite_vec2(values["held_velocity"])
    if held_velocity is None:
        raise ValueError("held velocity is invalid")
    integer_fields = (
        "frame_index",
        "history_version",
        "mission_horizon_frames",
        "active_reference_count",
    )
    for field in integer_fields:
        value = values[field]
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise ValueError(f"{field} must be an integer")
    frame_index = int(values["frame_index"])
    history_version = int(values["history_version"])
    horizon = int(values["mission_horizon_frames"])
    active_count = int(values["active_reference_count"])
    if (
        frame_index < 0
        or history_version < 0
        or horizon <= 0
        or frame_index >= horizon
        or active_count != len(canonical_references)
    ):
        raise ValueError("qualified frame/count contract is invalid")
    command_frame = values["applied_command_frame"]
    if frame_index == 0:
        if (
            command_frame is not None
            or previous_public is not None
            or previous_private is not None
            or values["qualifier_kind"] != "deployment"
        ):
            raise ValueError("frame-zero qualified chronology is invalid")
    elif (
        isinstance(command_frame, bool)
        or not isinstance(command_frame, Integral)
        or int(command_frame) != frame_index - 1
    ):
        raise ValueError("applied command frame is invalid")
    provenance = values["base_anchor_provenance"]
    canonical_provenance = _qualified_base_provenance(provenance)
    derived_provenance = tuple(sorted({
        anchor
        for reference in canonical_references
        for anchor in reference["base_anchor_provenance"]
    }))
    if (
        canonical_provenance is None
        or len(canonical_provenance) < 2
        or canonical_provenance != derived_provenance
    ):
        raise ValueError("base anchor provenance is invalid")
    qualifier_kind = values["qualifier_kind"]
    payload = values["qualifier_payload"]
    if not isinstance(payload, Mapping) or _qualified_contains_forbidden(payload):
        raise ValueError("qualifier payload is invalid")
    if qualifier_kind == "deployment":
        if set(payload) != {"domain"}:
            raise ValueError("deployment qualifier payload schema is invalid")
        qualify_all((), (), "deployment", payload)
    elif qualifier_kind == "history":
        if set(payload) != {"innovation_limit"}:
            raise ValueError("history qualifier payload schema is invalid")
        limit = _qualified_finite_nonnegative(payload["innovation_limit"])
        if limit != INNOVATION_REFERENCE_QUANTILE:
            raise ValueError("history innovation limit is invalid")
    elif qualifier_kind == "unavailable":
        if (
            set(payload) != {"reason"}
            or not isinstance(payload["reason"], str)
            or not payload["reason"]
        ):
            raise ValueError("unavailable qualifier payload schema is invalid")
    else:
        raise ValueError("unsupported qualifier kind")
    return {
        "references": tuple(sorted(
            canonical_references,
            key=lambda reference: reference["key"],
        )),
    }


def _qualified_finite_vec2(value):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        return None
    result = []
    for item in value:
        number = _qualified_finite_number(item)
        if number is None:
            return None
        result.append(number)
    return tuple(result)


def _qualified_finite_number(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def _qualified_finite_nonnegative(value):
    number = _qualified_finite_number(value)
    return number if number is not None and number >= 0.0 else None


def _qualified_reference_covariance(value):
    try:
        covariance = np.asarray(value, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        covariance.shape != (2, 2)
        or not np.isfinite(covariance).all()
        or not np.allclose(covariance, covariance.T, rtol=1e-12, atol=1e-12)
    ):
        return None
    canonical = 0.5 * (covariance + covariance.T)
    try:
        eigenvalues = np.linalg.eigvalsh(canonical)
    except np.linalg.LinAlgError:
        return None
    if (
        not np.isfinite(eigenvalues).all()
        or eigenvalues[0] < -1e-12 * max(1.0, eigenvalues[-1])
    ):
        return None
    return canonical.tolist()


def _qualified_base_provenance(value):
    if not isinstance(value, (tuple, list)) or any(
        isinstance(item, bool)
        or not isinstance(item, Integral)
        or item < 0
        for item in value
    ):
        return None
    canonical = tuple(int(item) for item in value)
    if canonical != tuple(sorted(set(canonical))):
        return None
    return canonical


def _qualified_contains_forbidden(value, active_ids=None):
    if active_ids is None:
        active_ids = set()
    if isinstance(value, Mapping):
        identifier = id(value)
        if identifier in active_ids:
            return True
        active_ids.add(identifier)
        try:
            for key, nested in value.items():
                if not isinstance(key, str):
                    return True
                lowered = key.lower()
                if any(fragment in lowered for fragment in _QUALIFIED_FORBIDDEN_FRAGMENTS):
                    return True
                if _qualified_contains_forbidden(nested, active_ids):
                    return True
            return False
        finally:
            active_ids.remove(identifier)
    if isinstance(value, (tuple, list, np.ndarray)):
        identifier = id(value)
        if identifier in active_ids:
            return True
        active_ids.add(identifier)
        try:
            iterable = value.tolist() if isinstance(value, np.ndarray) else value
            return any(
                _qualified_contains_forbidden(item, active_ids)
                for item in iterable
            )
        finally:
            active_ids.remove(identifier)
    return False


def _qualified_bounded_solver_evidence(value):
    active_ids = set()
    nodes = 0

    def visit(item, depth):
        nonlocal nodes
        nodes += 1
        if (
            nodes > _QUALIFIED_EVIDENCE_MAX_NODES
            or depth > _QUALIFIED_EVIDENCE_MAX_DEPTH
        ):
            return False
        if isinstance(item, Mapping):
            identifier = id(item)
            if identifier in active_ids:
                return False
            active_ids.add(identifier)
            try:
                for key, nested in item.items():
                    if not isinstance(key, str) or any(
                        fragment in key.lower()
                        for fragment in _QUALIFIED_FORBIDDEN_FRAGMENTS
                    ):
                        return False
                    if not visit(nested, depth + 1):
                        return False
                return True
            finally:
                active_ids.remove(identifier)
        if isinstance(item, (tuple, list)):
            identifier = id(item)
            if identifier in active_ids:
                return False
            active_ids.add(identifier)
            try:
                return all(visit(nested, depth + 1) for nested in item)
            finally:
                active_ids.remove(identifier)
        return item is None or isinstance(item, (str, bool, Real))

    return visit(value, 0)


def _qualified_nonempty_string(value):
    return isinstance(value, str) and bool(value)


def _qualified_proposal_trace_valid(trace):
    if (
        not isinstance(trace, (tuple, list))
        or len(trace) > MAX_WNLS_PROPOSALS
    ):
        return False
    previous_damping = None
    previous_cost = None
    previous_trial_cost = None
    previous_accepted = None
    for index, row in enumerate(trace):
        if (
            not isinstance(row, Mapping)
            or tuple(row) != _QUALIFIED_PROPOSAL_TRACE_FIELDS
        ):
            return False
        if not _nonnegative_qualified_int(row["proposal"]):
            return False
        if int(row["proposal"]) != index:
            return False
        damping = _qualified_finite_number(row["damping"])
        cost = _qualified_finite_nonnegative(row["cost"])
        stationarity = _qualified_finite_nonnegative(row["stationarity_norm"])
        raw_step = (
            None
            if row["raw_step_norm"] is None
            else _qualified_finite_nonnegative(row["raw_step_norm"])
        )
        trial = (
            None
            if row["trial_cost"] is None
            else _qualified_finite_nonnegative(row["trial_cost"])
        )
        invalid_reason = row["invalid_trial_reason"]
        accepted = row["accepted"]
        if (
            damping is None
            or damping < MIN_WNLS_DAMPING
            or damping > MAX_WNLS_DAMPING
            or cost is None
            or stationarity is None
            or (row["raw_step_norm"] is not None and raw_step is None)
            or (row["trial_cost"] is not None and trial is None)
            or (
                invalid_reason is not None
                and not _qualified_nonempty_string(invalid_reason)
            )
            or not isinstance(accepted, bool)
        ):
            return False
        if index == 0:
            if damping != INITIAL_WNLS_DAMPING:
                return False
        else:
            expected_damping = (
                max(MIN_WNLS_DAMPING, previous_damping / WNLS_DAMPING_FACTOR)
                if previous_accepted
                else previous_damping * WNLS_DAMPING_FACTOR
            )
            expected_cost = (
                previous_trial_cost if previous_accepted else previous_cost
            )
            if (
                damping != expected_damping
                or expected_cost is None
                or not _same_recomputed_cost(cost, expected_cost)
            ):
                return False
        if accepted:
            if invalid_reason is not None or trial is None or not trial < cost:
                return False
        elif trial is None and invalid_reason is None:
            return False
        elif trial is not None and invalid_reason is not None:
            return False
        elif trial is not None and trial < cost:
            return False
        previous_damping = damping
        previous_cost = cost
        previous_trial_cost = trial
        previous_accepted = accepted
    return True


def _nonnegative_qualified_int(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, Integral)
        and value >= 0
    )


def _qualified_nonconverged_solver_result_valid(value):
    status = value["status"]
    estimate = value["estimate"]
    phi_minimum = value["phi_min_eigenvalue"]
    cost = value["cost"]
    stationarity = value["stationarity_norm"]
    if (
        status not in {"failed", "invalid"}
        or value["covariance"] is not None
        or value["epsilon"] is not None
        or value["phi_condition"] is not None
        or value["fim_valid"] is not False
        or not _qualified_nonempty_string(value["failure_reason"])
        or (
            estimate is not None
            and _qualified_finite_vec2(estimate) is None
        )
        or (
            cost is not None
            and _qualified_finite_nonnegative(cost) is None
        )
        or (
            stationarity is not None
            and _qualified_finite_nonnegative(stationarity) is None
        )
    ):
        return False
    if status == "failed" and phi_minimum is not None:
        return False
    if (
        status == "invalid"
        and phi_minimum is not None
        and _qualified_finite_number(phi_minimum) is None
    ):
        return False
    return True


def _complete_qualified_converged_solver_result(value):
    trace = value.get("proposal_trace") if isinstance(value, Mapping) else None
    if not isinstance(trace, (tuple, list)):
        return False
    if isinstance(trace, list):
        return _complete_converged_solver_result(value)
    legacy_compatible = dict(value)
    legacy_compatible["proposal_trace"] = list(trace)
    return _complete_converged_solver_result(legacy_compatible)


def _qualified_solver_result_evidence_valid(value):
    if (
        not isinstance(value, Mapping)
        or tuple(value) != _QUALIFIED_SOLVER_RESULT_FIELDS
        or not _qualified_bounded_solver_evidence(value)
    ):
        return False
    status = value["status"]
    if status not in {"converged", "failed", "invalid"}:
        return False
    count = value["proposal_count"]
    iterations = value["iterations"]
    trace = value["proposal_trace"]
    if (
        not _nonnegative_qualified_int(count)
        or not _nonnegative_qualified_int(iterations)
        or not isinstance(trace, (tuple, list))
        or int(count) != int(iterations)
        or int(count) != len(trace)
        or not _qualified_proposal_trace_valid(trace)
    ):
        return False
    if status == "converged":
        return _complete_qualified_converged_solver_result(value)
    return _qualified_nonconverged_solver_result_valid(value)


def _invalid_qualified_solver_result_evidence():
    return {
        "status": "invalid",
        "estimate": None,
        "covariance": None,
        "epsilon": None,
        "phi_min_eigenvalue": None,
        "phi_condition": None,
        "fim_valid": False,
        "proposal_count": 0,
        "iterations": 0,
        "cost": None,
        "stationarity_norm": None,
        "failure_reason": "invalid_solver_result_evidence",
        "proposal_trace": [],
    }


def _is_invalid_qualified_solver_result_evidence(value):
    return (
        isinstance(value, Mapping)
        and tuple(value) == _QUALIFIED_SOLVER_RESULT_FIELDS
        and value["status"] == "invalid"
        and value["estimate"] is None
        and value["covariance"] is None
        and value["epsilon"] is None
        and value["phi_min_eigenvalue"] is None
        and value["phi_condition"] is None
        and value["fim_valid"] is False
        and type(value["proposal_count"]) is int
        and value["proposal_count"] == 0
        and type(value["iterations"]) is int
        and value["iterations"] == 0
        and value["cost"] is None
        and value["stationarity_norm"] is None
        and value["failure_reason"] == "invalid_solver_result_evidence"
        and isinstance(value["proposal_trace"], list)
        and not value["proposal_trace"]
    )


def _qualified_json_value(value, active_ids=None):
    if active_ids is None:
        active_ids = set()
    if _qualified_contains_forbidden(value):
        raise ValueError("forbidden qualified runtime field")
    if isinstance(value, DeploymentContract):
        return _qualified_json_value(asdict(value), active_ids)
    if isinstance(value, np.generic):
        return _qualified_json_value(value.item(), active_ids)
    if isinstance(value, np.ndarray):
        return _qualified_json_value(value.tolist(), active_ids)
    if isinstance(value, Mapping):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified runtime payload")
        active_ids.add(identifier)
        try:
            return {
                str(key): _qualified_json_value(value[key], active_ids)
                for key in sorted(value)
            }
        finally:
            active_ids.remove(identifier)
    if isinstance(value, (tuple, list)):
        identifier = id(value)
        if identifier in active_ids:
            raise ValueError("cyclic qualified runtime payload")
        active_ids.add(identifier)
        try:
            return [_qualified_json_value(item, active_ids) for item in value]
        finally:
            active_ids.remove(identifier)
    if value is None or isinstance(value, (str, bool)):
        return value
    number = _qualified_finite_number(value)
    if number is None:
        raise ValueError("qualified payload is not canonical JSON")
    if isinstance(value, Integral):
        return int(value)
    return number


def _canonical_qualified_solver_result(value):
    if not _qualified_solver_result_evidence_valid(value):
        return _invalid_qualified_solver_result_evidence()
    canonical = {
        field: _qualified_json_value(value[field])
        for field in _QUALIFIED_SOLVER_RESULT_FIELDS
        if field != "proposal_trace"
    }
    canonical["proposal_trace"] = [
        {
            field: _qualified_json_value(row[field])
            for field in _QUALIFIED_PROPOSAL_TRACE_FIELDS
        }
        for row in value["proposal_trace"]
    ]
    return {
        field: canonical[field]
        for field in _QUALIFIED_SOLVER_RESULT_FIELDS
    }


def _qualified_recompute_solver_geometry(result, references):
    geometry = {
        "geometry_failure_reason": None,
        "recomputed_covariance": None,
        "recomputed_estimate": None,
        "recomputed_fim": None,
        "recomputed_gauss_newton": None,
        "recomputed_objective": None,
        "recomputed_residual": None,
        "recomputed_stationarity_norm": None,
        "solver_geometry_consistent": False,
        "verified_result": None,
    }
    if not _complete_converged_solver_result(result):
        geometry["geometry_failure_reason"] = "solver_result_not_converged"
        return geometry
    estimate = _finite_vector(result["estimate"])
    if estimate is None:
        geometry["geometry_failure_reason"] = "invalid_solver_estimate"
        return geometry
    positions = np.asarray(
        [reference["position"] for reference in references],
        dtype=float,
    )
    covariances = np.asarray(
        [reference["covariance"] for reference in references],
        dtype=float,
    )
    ranges = np.asarray(
        [reference["range"] for reference in references],
        dtype=float,
    )
    sigma = float(references[0]["ranging_sigma"])
    terms = _linearized_terms(
        estimate,
        positions,
        covariances,
        ranges,
        sigma,
    )
    fim = fim_radius(estimate, positions, covariances, sigma)
    if terms is None or fim.get("status") != "converged":
        geometry["geometry_failure_reason"] = "geometry_recomputation_failed"
        return geometry
    _, _, residual, gauss_newton, gradient, cost = terms
    covariance = np.asarray(fim["covariance"], dtype=float)
    try:
        phi = np.linalg.inv(covariance)
    except np.linalg.LinAlgError:
        geometry["geometry_failure_reason"] = "geometry_fim_inversion_failed"
        return geometry
    stationarity = float(np.linalg.norm(gradient, ord=np.inf))
    geometry.update({
        "recomputed_covariance": covariance.tolist(),
        "recomputed_estimate": estimate.tolist(),
        "recomputed_fim": phi.tolist(),
        "recomputed_gauss_newton": gauss_newton.tolist(),
        "recomputed_objective": float(cost),
        "recomputed_residual": residual.tolist(),
        "recomputed_stationarity_norm": stationarity,
    })
    consistent = (
        _qualified_same_scalar(result["cost"], cost)
        and _qualified_same_matrix(result["covariance"], covariance)
        and _qualified_same_scalar(result["epsilon"], fim["epsilon"])
        and _qualified_same_scalar(
            result["phi_min_eigenvalue"],
            fim["phi_min_eigenvalue"],
        )
        and _qualified_same_scalar(
            result["phi_condition"],
            fim["phi_condition"],
        )
        and _qualified_same_scalar(
            result["stationarity_norm"],
            stationarity,
        )
    )
    geometry["solver_geometry_consistent"] = consistent
    if not consistent:
        geometry["geometry_failure_reason"] = "solver_geometry_mismatch"
        return geometry
    verified = dict(result)
    verified.update({
        "estimate": estimate.tolist(),
        "covariance": covariance.tolist(),
        "epsilon": float(fim["epsilon"]),
        "phi_min_eigenvalue": float(fim["phi_min_eigenvalue"]),
        "phi_condition": float(fim["phi_condition"]),
        "cost": float(cost),
        "stationarity_norm": stationarity,
    })
    geometry["verified_result"] = verified
    return geometry


def _qualified_same_scalar(first, second):
    first_number = _qualified_finite_number(first)
    second_number = _qualified_finite_number(second)
    return (
        first_number is not None
        and second_number is not None
        and math.isclose(
            first_number,
            second_number,
            rel_tol=1e-9,
            abs_tol=1e-12,
        )
    )


def _qualified_same_matrix(first, second):
    try:
        first_array = np.asarray(first, dtype=float)
        second_array = np.asarray(second, dtype=float)
    except (TypeError, ValueError, OverflowError):
        return False
    return bool(
        first_array.shape == second_array.shape
        and np.isfinite(first_array).all()
        and np.isfinite(second_array).all()
        and np.allclose(
            first_array,
            second_array,
            rtol=1e-9,
            atol=1e-12,
        )
    )


def _serialize_qualified_start(start):
    return {
        "attempt_id": stable_attempt_id(start),
        "kind": start.kind,
        "estimate": list(start.estimate),
        "reference_keys": [list(key) for key in start.reference_keys],
        "branch": start.branch,
    }


def _serialize_qualified_solver_attempt(
    start,
    result,
    *,
    eligible,
    eligibility_reason,
    geometry,
    active_reference_count,
    base_anchor_provenance,
):
    objective = geometry["recomputed_objective"]
    reduced_cost = (
        None
        if objective is None
        else objective / max(1, int(active_reference_count) - 2)
    )
    return {
        "attempt_id": stable_attempt_id(start),
        "start_record": _serialize_qualified_start(start),
        "solver_result": _qualified_json_value(result),
        "local_eligible": bool(eligible),
        "local_eligibility_reason": eligibility_reason,
        "local_diagnostics": _qualified_json_value({
            "failure_reason": (
                None if eligible else eligibility_reason
            ),
            "geometry_failure_reason": geometry["geometry_failure_reason"],
            "reduced_whitened_cost": reduced_cost,
            "recomputed_estimate": geometry["recomputed_estimate"],
            "recomputed_covariance": geometry["recomputed_covariance"],
            "recomputed_objective": objective,
            "recomputed_residual_cost": objective,
            "recomputed_residual": geometry["recomputed_residual"],
            "recomputed_fim": geometry["recomputed_fim"],
            "recomputed_gauss_newton": geometry["recomputed_gauss_newton"],
            "recomputed_stationarity_norm": geometry[
                "recomputed_stationarity_norm"
            ],
            "solver_geometry_consistent": geometry[
                "solver_geometry_consistent"
            ],
            "active_reference_count": int(active_reference_count),
            "base_anchor_provenance": list(base_anchor_provenance),
        }),
    }


def _qualified_local_candidate(
    attempt_id,
    recomputed_result,
    base_anchor_provenance,
):
    projected = project_local_candidate(attempt_id, recomputed_result)
    if projected is None:
        return None
    return LocalCandidate(
        projected.attempt_id,
        projected.estimate,
        projected.objective_cost,
        {
            **dict(projected.payload),
            "base_anchor_provenance": list(base_anchor_provenance),
        },
    )


def _serialize_qualified_candidate(candidate):
    return {
        "attempt_id": candidate.attempt_id,
        "estimate": list(candidate.estimate),
        "objective_cost": candidate.objective_cost,
        "payload": _qualified_json_value(candidate.payload),
    }


def _serialize_qualified_clustering(clustering):
    return {
        "tolerance_m": clustering.tolerance_m,
        "separable": clustering.separable,
        "reason": clustering.reason,
        "mode_count": len(clustering.modes),
        "modes": [
            {
                "mode_id": canonical_mode_id(mode),
                "member_ids": list(mode.member_ids),
                "diameter_m": mode.diameter_m,
            }
            for mode in clustering.modes
        ],
    }


def _serialize_qualified_transition_inputs(**values):
    return _qualified_json_value({
        "previous_public": values["previous_public"],
        "previous_private": values["previous_private"],
        "held_velocity": values["held_velocity"],
        "frame_index": values["frame_index"],
        "applied_command_frame": values["applied_command_frame"],
        "history_version": values["history_version"],
        "mission_horizon_frames": values["mission_horizon_frames"],
    })


def _serialize_qualified_prior_bundle(prior_bundle):
    if not isinstance(prior_bundle, PriorBundle):
        raise ValueError("prior bundle is invalid")
    return _qualified_json_value({
        "public_prediction": prior_bundle.public_prediction,
        "private_prior": prior_bundle.private_prior,
        "history_version": prior_bundle.history_version,
    })


def _serialize_qualified_qualifier_context(
    *,
    qualifier_kind,
    qualifier_payload,
    propagated_private_prior,
):
    return _qualified_json_value({
        "qualifier_kind": qualifier_kind,
        "qualifier_payload": qualifier_payload,
        "propagated_private_prior": (
            propagated_private_prior if qualifier_kind == "history" else None
        ),
    })


def _qualify_integrated_modes(
    modes,
    representatives,
    *,
    qualifier_kind,
    qualifier_payload,
    propagated_private_prior,
):
    if qualifier_kind == "unavailable":
        return tuple(
            ModeQualification(
                canonical_mode_id(mode),
                False,
                qualifier_payload["reason"],
                None,
            )
            for mode in modes
        )
    if qualifier_kind == "history":
        if propagated_private_prior is None:
            return tuple(
                ModeQualification(
                    canonical_mode_id(mode),
                    False,
                    "propagated_private_prior_unavailable",
                    None,
                )
                for mode in modes
            )
        payload = {
            "propagated_private_prior": propagated_private_prior,
            "innovation_limit": qualifier_payload["innovation_limit"],
        }
    else:
        payload = qualifier_payload
    return qualify_all(
        modes,
        representatives,
        qualifier_kind,
        payload,
    )


def _serialize_mode_qualification(item):
    return {
        "mode_id": item.mode_id,
        "admissible": item.admissible,
        "reason": item.reason,
        "score": item.score,
    }


def _serialize_qualified_publication_decision(decision):
    return {
        "status": decision.status,
        "reason": decision.reason,
        "mode_id": decision.mode_id,
        "representative_attempt_id": (
            None
            if decision.representative is None
            else decision.representative.attempt_id
        ),
    }
