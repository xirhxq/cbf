"""Bounded public prediction lifecycle for diagnostic WNLS estimates."""

import math
from collections.abc import Mapping
from numbers import Integral, Real

import numpy as np


FRAME_DT_SECONDS = 0.5
MAX_PUBLIC_PREDICTION_AGE = 2
CANDIDATE_DEDUP_M = 1e-9
RELATIVE_TIE_TOLERANCE = 1e-12
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
        try:
            epsilon = float(candidate.get("epsilon"))
        except (AttributeError, TypeError, ValueError, OverflowError):
            epsilon = math.nan
        if (
            state is not None
            and provenance is not None
            and math.isfinite(epsilon)
            and epsilon >= 0.0
        ):
            estimate, covariance = state
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
