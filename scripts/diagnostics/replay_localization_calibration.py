"""Pure-NumPy primitives and evidence bundles for localization calibration."""

import argparse
import gzip
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from scripts.diagnostics.run_diagnostic import (
    HARD_FLOOR_BYTES,
    OUTPUT_ROOT_CAP_BYTES,
    RUN_CAP_BYTES,
    DiskSpaceError,
    _allocate_run_root,
    _nearest_existing_ancestor,
    _sha256,
    _validate_output_root,
    _write_manifest,
    allocated_bytes,
    available_bytes,
    require_start_space,
)


MAX_ITERATIONS = 50
INITIAL_DAMPING = 1e-3
STEP_TOLERANCE = 1e-9
COST_TOLERANCE = 1e-12
RANGE_EPSILON = 1e-12
RELATIVE_SPECTRAL_THRESHOLD = 1e-12
STRICT_PREVIOUS_POLICY = "strict_previous_v1"
RESTART_BEFORE_FIRST_FINITE_POLICY = (
    "deployment_restart_before_first_finite_v1"
)
INITIALIZATION_POLICIES = (
    STRICT_PREVIOUS_POLICY,
    RESTART_BEFORE_FIRST_FINITE_POLICY,
)


def stable_measurement_seed(
    run_seed: int,
    frame_index: int,
    observer_id: int,
    reference_kind: str,
    reference_id: int,
) -> int:
    """Return a process-independent seed for one directed range measurement."""
    payload = (
        f"cbf2026-range-v1|{run_seed}|{frame_index}|{observer_id}|"
        f"{reference_kind}|{reference_id}"
    ).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


def _squad_indices(config: dict, robot_id: int) -> tuple[int, int]:
    squad_size = math.ceil(config["num"] / config["formation"]["parts"])
    part_index = (robot_id - 1) // squad_size
    local_index = (robot_id - 1) % squad_size + 1
    return part_index, local_index


def fixed_references(config: dict, observer_id: int) -> dict[str, list[int]]:
    """Reconstruct the fixed production localization references for one UAV."""
    part_index, local_index = _squad_indices(config, observer_id)
    comm = config["cbfs"]["without-slack"]["comm-fixed"]
    min_offset = int(comm.get("min-neighbour-id-offset", -2))
    max_offset = int(comm.get("max-neighbour-id-offset", 0))
    assigned_bases = config["formation"]["bases-id"][part_index]
    maximum_base_index = -local_index - min_offset

    base_ids = [
        int(base_id)
        for index, base_id in enumerate(assigned_bases)
        if index <= maximum_base_index
    ]
    uav_ids = [
        candidate
        for candidate in range(1, int(config["num"]) + 1)
        if candidate != observer_id
        and observer_id + min_offset <= candidate <= observer_id + max_offset
        and _squad_indices(config, candidate)[0] == part_index
        and _squad_indices(config, candidate)[1] < local_index
    ]
    return {"base_ids": sorted(set(base_ids)), "uav_ids": sorted(set(uav_ids))}


def active_references(
    config: dict,
    observer_id: int,
    uav_positions: dict[int, np.ndarray],
) -> dict[str, list[int]]:
    """Return fixed references plus all visible eligible localization references."""
    references = fixed_references(config, observer_id)
    base_ids = set(references["base_ids"])
    uav_ids = set(references["uav_ids"])
    observer = np.asarray(uav_positions[observer_id], dtype=float)
    max_range = float(config["cbfs"]["without-slack"]["comm-fixed"]["max-range"])

    for base_id, base_position in enumerate(config["bases"]):
        if np.linalg.norm(observer - np.asarray(base_position, dtype=float)) <= max_range:
            base_ids.add(base_id)

    observer_part, observer_local = _squad_indices(config, observer_id)
    for candidate, candidate_position in uav_positions.items():
        candidate_part, candidate_local = _squad_indices(config, int(candidate))
        if (
            candidate_part == observer_part
            and candidate_local < observer_local
            and np.linalg.norm(observer - np.asarray(candidate_position, dtype=float))
            <= max_range
        ):
            uav_ids.add(int(candidate))

    return {"base_ids": sorted(base_ids), "uav_ids": sorted(uav_ids)}


def _result(
    status: str,
    *,
    estimate: np.ndarray | None = None,
    covariance: np.ndarray | None = None,
    epsilon: float | None = None,
    phi_min_eigenvalue: float | None = None,
    phi_condition: float | None = None,
    iterations: int = 0,
    cost: float | None = None,
    stationarity_norm: float | None = None,
    failure_reason: str | None = None,
) -> dict:
    return {
        "status": status,
        "estimate": None if estimate is None else np.asarray(estimate, dtype=float).tolist(),
        "covariance": (
            None if covariance is None else np.asarray(covariance, dtype=float).tolist()
        ),
        "epsilon": epsilon,
        "phi_min_eigenvalue": phi_min_eigenvalue,
        "phi_condition": phi_condition,
        "iterations": iterations,
        "cost": cost,
        "stationarity_norm": stationarity_norm,
        "failure_reason": failure_reason,
    }


def _validated_inputs(
    reference_positions: np.ndarray,
    reference_covariances: np.ndarray,
    measurements: np.ndarray | None,
    initial_estimate: np.ndarray,
    ranging_sigma: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray, float] | None:
    try:
        references = np.asarray(reference_positions, dtype=float)
        covariances = np.asarray(reference_covariances, dtype=float)
        ranges = (
            None if measurements is None else np.asarray(measurements, dtype=float)
        )
        estimate = np.asarray(initial_estimate, dtype=float)
        sigma_value = np.asarray(ranging_sigma)
        if sigma_value.ndim != 0:
            return None
        sigma = float(sigma_value)
    except (TypeError, ValueError, OverflowError):
        return None
    if (
        references.ndim != 2
        or references.shape[1:] != (2,)
        or covariances.shape != (references.shape[0], 2, 2)
        or estimate.shape != (2,)
        or references.shape[0] < 2
        or not np.isfinite(references).all()
        or not np.isfinite(covariances).all()
        or not np.isfinite(estimate).all()
        or not np.isfinite(sigma)
        or sigma <= 0.0
        or (
            ranges is not None
            and (
                ranges.shape != (references.shape[0],)
                or not np.isfinite(ranges).all()
            )
        )
    ):
        return None
    covariance_scale = np.maximum(
        1.0, np.max(np.abs(covariances), axis=(1, 2))
    )
    symmetry_error = np.max(
        np.abs(covariances - np.swapaxes(covariances, 1, 2)), axis=(1, 2)
    )
    if np.any(symmetry_error > RELATIVE_SPECTRAL_THRESHOLD * covariance_scale):
        return None
    symmetric_covariances = 0.5 * (
        covariances + np.swapaxes(covariances, 1, 2)
    )
    try:
        covariance_eigenvalues = np.linalg.eigvalsh(symmetric_covariances)
    except np.linalg.LinAlgError:
        return None
    if (
        not np.isfinite(covariance_eigenvalues).all()
        or np.any(
            covariance_eigenvalues[:, 0]
            < -RELATIVE_SPECTRAL_THRESHOLD * covariance_scale
        )
    ):
        return None
    return references, symmetric_covariances, ranges, estimate, sigma


def _linearized_terms(
    estimate: np.ndarray,
    reference_positions: np.ndarray,
    reference_covariances: np.ndarray,
    measurements: np.ndarray,
    ranging_sigma: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float] | None:
    diff = estimate[None, :] - reference_positions
    distance = np.linalg.norm(diff, axis=1)
    if not np.isfinite(distance).all() or np.any(distance <= RANGE_EPSILON):
        return None
    direction = diff / distance[:, None]
    projected_variance = np.einsum(
        "ni,nij,nj->n", direction, reference_covariances, direction
    )
    total_variance = ranging_sigma**2 + projected_variance
    if not np.isfinite(total_variance).all() or np.any(total_variance <= 0.0):
        return None
    weight = 1.0 / total_variance
    range_residual = distance - measurements
    residual = range_residual / np.sqrt(total_variance)
    covariance_direction = np.einsum(
        "nij,nj->ni", reference_covariances, direction
    )
    tangent_covariance_direction = covariance_direction - direction * (
        projected_variance[:, None]
    )
    residual_jacobian = direction / np.sqrt(total_variance[:, None]) - (
        range_residual[:, None]
        * tangent_covariance_direction
        / (
            distance[:, None]
            * total_variance[:, None] ** 1.5
        )
    )
    if (
        not np.isfinite(weight).all()
        or not np.isfinite(residual).all()
        or not np.isfinite(residual_jacobian).all()
    ):
        return None
    gauss_newton = residual_jacobian.T @ residual_jacobian
    gradient = residual_jacobian.T @ residual
    cost = float(residual @ residual)
    if (
        not np.isfinite(gauss_newton).all()
        or not np.isfinite(gradient).all()
        or not np.isfinite(cost)
    ):
        return None
    return direction, weight, residual, gauss_newton, gradient, cost


def _fim_terms(
    estimate: np.ndarray,
    reference_positions: np.ndarray,
    reference_covariances: np.ndarray,
    ranging_sigma: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    diff = estimate[None, :] - reference_positions
    distance = np.linalg.norm(diff, axis=1)
    if not np.isfinite(distance).all() or np.any(distance <= RANGE_EPSILON):
        return None
    direction = diff / distance[:, None]
    projected_variance = np.einsum(
        "ni,nij,nj->n", direction, reference_covariances, direction
    )
    total_variance = ranging_sigma**2 + projected_variance
    if not np.isfinite(total_variance).all() or np.any(total_variance <= 0.0):
        return None
    weight = 1.0 / total_variance
    phi = direction.T @ (weight[:, None] * direction)
    if not np.isfinite(weight).all() or not np.isfinite(phi).all():
        return None
    return direction, weight, phi


def fim_radius(
    estimate: np.ndarray,
    reference_positions: np.ndarray,
    reference_covariances: np.ndarray,
    ranging_sigma: float,
) -> dict:
    """Recompute the final FIM, covariance approximation, and three-sigma radius."""
    validated = _validated_inputs(
        reference_positions,
        reference_covariances,
        None,
        estimate,
        ranging_sigma,
    )
    if validated is None:
        return _result("invalid", failure_reason="non-finite or malformed FIM input")
    references, covariances, _, final_estimate, sigma = validated
    terms = _fim_terms(final_estimate, references, covariances, sigma)
    if terms is None:
        return _result(
            "invalid",
            failure_reason="undefined zero-range direction or invalid final FIM weights",
        )
    _, _, phi = terms
    eigenvalues = np.linalg.eigvalsh(phi)
    if not np.isfinite(eigenvalues).all():
        return _result("invalid", failure_reason="non-finite final FIM eigenvalues")
    minimum = float(eigenvalues[0])
    maximum = float(eigenvalues[-1])
    if maximum <= 0.0 or minimum <= RELATIVE_SPECTRAL_THRESHOLD * maximum:
        return _result(
            "invalid",
            failure_reason="singular or ill-conditioned final FIM",
            phi_min_eigenvalue=minimum,
        )
    try:
        covariance = np.linalg.inv(phi)
    except np.linalg.LinAlgError:
        return _result(
            "invalid",
            failure_reason="final FIM inversion failed",
            phi_min_eigenvalue=minimum,
        )
    if not np.isfinite(covariance).all():
        return _result(
            "invalid",
            failure_reason="non-finite final covariance",
            phi_min_eigenvalue=minimum,
        )
    epsilon = 3 * np.sqrt(np.max(np.linalg.eigvalsh(covariance)))
    if not np.isfinite(epsilon):
        return _result(
            "invalid",
            failure_reason="non-finite final radius",
            phi_min_eigenvalue=minimum,
        )
    return _result(
        "converged",
        estimate=final_estimate,
        covariance=covariance,
        epsilon=float(epsilon),
        phi_min_eigenvalue=minimum,
        phi_condition=maximum / minimum,
    )


def solve_wnls(
    reference_positions: np.ndarray,
    reference_covariances: np.ndarray,
    measurements: np.ndarray,
    initial_estimate: np.ndarray,
    ranging_sigma: float,
) -> dict:
    """Estimate one 2-D UAV position with damped weighted nonlinear least squares."""
    validated = _validated_inputs(
        reference_positions,
        reference_covariances,
        measurements,
        initial_estimate,
        ranging_sigma,
    )
    if validated is None:
        return _result("invalid", failure_reason="non-finite or malformed WNLS input")
    references, covariances, ranges, estimate, sigma = validated
    damping = INITIAL_DAMPING
    cost = None
    stationarity_norm = None

    for iteration in range(1, MAX_ITERATIONS + 1):
        distance = np.linalg.norm(estimate[None, :] - references, axis=1)
        if np.any(distance <= RANGE_EPSILON):
            return _result(
                "invalid",
                iterations=iteration - 1,
                cost=cost,
                stationarity_norm=stationarity_norm,
                failure_reason="undefined zero-range direction",
            )
        terms = _linearized_terms(estimate, references, covariances, ranges, sigma)
        if terms is None:
            return _result(
                "invalid",
                iterations=iteration - 1,
                cost=cost,
                stationarity_norm=stationarity_norm,
                failure_reason="invalid WNLS terms",
            )
        _, _, _, gauss_newton, gradient, cost = terms
        stationarity_norm = float(np.linalg.norm(gradient))
        if stationarity_norm <= STEP_TOLERANCE:
            final = fim_radius(estimate, references, covariances, sigma)
            final["iterations"] = iteration
            final["cost"] = cost
            final["stationarity_norm"] = stationarity_norm
            return final
        try:
            delta = np.linalg.solve(
                gauss_newton + damping * np.eye(2), -gradient
            )
        except np.linalg.LinAlgError:
            return _result(
                "invalid",
                iterations=iteration - 1,
                cost=cost,
                stationarity_norm=stationarity_norm,
                failure_reason="damped solve failed",
            )
        if not np.isfinite(delta).all():
            return _result(
                "invalid",
                iterations=iteration - 1,
                cost=cost,
                stationarity_norm=stationarity_norm,
                failure_reason="non-finite WNLS step",
            )
        trial_estimate = estimate + delta
        trial_terms = _linearized_terms(
            trial_estimate, references, covariances, ranges, sigma
        )
        if trial_terms is not None and trial_terms[-1] < cost:
            trial_gradient = trial_terms[4]
            trial_cost = trial_terms[5]
            trial_stationarity_norm = float(np.linalg.norm(trial_gradient))
            estimate = trial_estimate
            damping /= 10.0
            cost = trial_cost
            stationarity_norm = trial_stationarity_norm
            if trial_stationarity_norm <= STEP_TOLERANCE:
                final = fim_radius(estimate, references, covariances, sigma)
                final["iterations"] = iteration
                final["cost"] = trial_cost
                final["stationarity_norm"] = trial_stationarity_norm
                return final
        else:
            damping *= 10.0

    return _result(
        "failed",
        iterations=MAX_ITERATIONS,
        cost=cost,
        stationarity_norm=stationarity_norm,
        failure_reason="maximum WNLS iterations exceeded",
    )


def _valid_prior_result(
    previous_result: dict | None,
) -> tuple[np.ndarray, np.ndarray, float, float, float] | None:
    if (
        not isinstance(previous_result, dict)
        or previous_result.get("status") not in {"converged", "stale"}
    ):
        return None
    try:
        estimate = np.asarray(previous_result["estimate"], dtype=float)
        covariance = np.asarray(previous_result["covariance"], dtype=float)
        epsilon = float(previous_result["epsilon"])
        phi_minimum = float(previous_result["phi_min_eigenvalue"])
        phi_condition = float(previous_result["phi_condition"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if (
        estimate.shape != (2,)
        or covariance.shape != (2, 2)
        or not np.isfinite(estimate).all()
        or not np.isfinite(covariance).all()
        or not np.isfinite(epsilon)
        or epsilon <= 0.0
        or not np.isfinite(phi_minimum)
        or phi_minimum <= 0.0
        or not np.isfinite(phi_condition)
        or phi_condition < 1.0
    ):
        return None
    covariance_scale = max(1.0, float(np.max(np.abs(covariance))))
    if np.max(np.abs(covariance - covariance.T)) > (
        RELATIVE_SPECTRAL_THRESHOLD * covariance_scale
    ):
        return None
    symmetric_covariance = 0.5 * (covariance + covariance.T)
    try:
        covariance_eigenvalues = np.linalg.eigvalsh(symmetric_covariance)
    except np.linalg.LinAlgError:
        return None
    if (
        not np.isfinite(covariance_eigenvalues).all()
        or covariance_eigenvalues[0] <= 0.0
    ):
        return None
    return (
        estimate,
        symmetric_covariance,
        epsilon,
        phi_minimum,
        phi_condition,
    )


def select_initial_estimate(
    policy: str,
    *,
    frame_index: int,
    deployment: np.ndarray,
    previous_result: dict | None,
    ever_acquired_finite: bool,
) -> tuple[np.ndarray, str]:
    """Return the initial estimate and its auditable source label."""
    if policy not in INITIALIZATION_POLICIES:
        raise ValueError(f"unknown initialization policy: {policy}")
    if frame_index == 0:
        return np.asarray(deployment, dtype=float), "deployment_frame_zero"
    valid_previous = _valid_prior_result(previous_result)
    if valid_previous is not None:
        return valid_previous[0], "previous_finite"
    if (
        policy == RESTART_BEFORE_FIRST_FINITE_POLICY
        and not ever_acquired_finite
    ):
        return (
            np.asarray(deployment, dtype=float),
            "deployment_restart_before_first_finite",
        )
    raw = (
        previous_result.get("estimate", deployment)
        if isinstance(previous_result, dict)
        else deployment
    )
    return np.asarray(raw, dtype=float), "strict_previous_missing"


def solve_later_frame(
    previous_result: dict | None,
    reference_positions: np.ndarray,
    reference_covariances: np.ndarray,
    measurements: np.ndarray,
    initial_estimate: np.ndarray,
    ranging_sigma: float,
) -> dict:
    """Retain a previous finite result as stale after a later-frame failure."""
    result = solve_wnls(
        reference_positions,
        reference_covariances,
        measurements,
        initial_estimate,
        ranging_sigma,
    )
    if result["status"] == "converged":
        return result
    valid_prior = _valid_prior_result(previous_result)
    if valid_prior is not None:
        estimate, covariance, epsilon, phi_minimum, phi_condition = valid_prior
        stale = _result(
            "stale",
            estimate=estimate,
            covariance=covariance,
            epsilon=epsilon,
            phi_min_eigenvalue=phi_minimum,
            phi_condition=phi_condition,
            iterations=result["iterations"],
            cost=result["cost"],
            stationarity_norm=result.get("stationarity_norm"),
            failure_reason=result["failure_reason"],
        )
        stale["failure"] = result
        return stale
    return result


GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")
PREREGISTERED_RANGING_SIGMA = 0.5
IMPLEMENTATION_ID = "cbf2026-localization-calibration-v2"
ESTIMATOR_CONTRACT_ID = "variable_weight_nls_full_residual_jacobian_v1"
BOOTSTRAP_RESAMPLES = 10_000
BOOTSTRAP_SEED = 20260728
_STATUS_CODES = {"converged": 0, "stale": 1, "invalid": 2, "failed": 3}
_CODE_STATUSES = {code: status for status, code in _STATUS_CODES.items()}


def _strict_load(path: Path) -> dict:
    def reject_constant(value: str):
        raise ValueError(f"non-finite JSON constant {value}")

    with path.open() as source:
        loaded = json.load(source, parse_constant=reject_constant)
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return loaded


def _json_value(value):
    """Convert NumPy values while rejecting non-finite output evidence."""
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def _strict_json_bytes(value, *, indent=None) -> bytes:
    return json.dumps(
        _json_value(value),
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":") if indent is None else None,
        indent=indent,
    ).encode("utf-8")


def _frames(data: dict) -> list[dict]:
    frames = data.get("state", data.get("frames"))
    if not isinstance(frames, list) or not frames:
        raise ValueError("truth replay must contain a non-empty state or frames array")
    if not all(isinstance(frame, dict) and isinstance(frame.get("robots"), list) for frame in frames):
        raise ValueError("every truth frame must contain a robots array")
    return frames


def _truth_positions(frame: dict, expected_ids: set[int]) -> dict[int, np.ndarray]:
    positions = {}
    for robot in frame["robots"]:
        try:
            robot_id = int(robot["id"])
            state = robot["state"]
            position = np.asarray([state["x"], state["y"]], dtype=float)
        except (KeyError, TypeError, ValueError, OverflowError):
            raise ValueError("robot truth state must provide finite id, x, and y") from None
        if robot_id in positions or position.shape != (2,) or not np.isfinite(position).all():
            raise ValueError("robot truth IDs must be unique with finite planar positions")
        positions[robot_id] = position
    if set(positions) != expected_ids:
        raise ValueError("every frame must contain exactly the configured UAV IDs")
    return positions


def _initial_positions(config: dict, expected_ids: set[int]) -> dict[int, np.ndarray]:
    try:
        positions = config["initial"]["position"]["positions"]
    except (KeyError, TypeError):
        raise ValueError("config must provide known deployment positions") from None
    if not isinstance(positions, list) or len(positions) != len(expected_ids):
        raise ValueError("deployment positions must match configured UAV count")
    result = {index: np.asarray(position, dtype=float) for index, position in enumerate(positions, 1)}
    if any(position.shape != (2,) or not np.isfinite(position).all() for position in result.values()):
        raise ValueError("deployment positions must be finite planar coordinates")
    return result


def _invalid_result(reason: str) -> dict:
    return _result("invalid", failure_reason=reason)


def _reference_inputs(
    references: dict[str, list[int]],
    config: dict,
    truth: dict[int, np.ndarray],
    current: dict[int, dict],
    seed: int,
    frame_index: int,
    observer_id: int,
    ranging_sigma: float,
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, list[dict], str | None]:
    """Record every physical edge before validating estimated DAG inputs."""
    positions = []
    covariances = []
    measurements = []
    records = []
    errors = []
    observer_truth = truth[observer_id]
    for kind, identifiers in (("base", references["base_ids"]), ("uav", references["uav_ids"])):
        for reference_id in identifiers:
            if kind == "base":
                try:
                    reference_truth = np.asarray(config["bases"][reference_id], dtype=float)
                except (IndexError, KeyError, TypeError, ValueError):
                    reference_truth = None
            else:
                reference_truth = truth.get(reference_id)
            truth_available = (
                reference_truth is not None
                and np.asarray(reference_truth).shape == (2,)
                and np.isfinite(reference_truth).all()
            )
            true_range = (
                float(np.linalg.norm(observer_truth - reference_truth))
                if truth_available
                else None
            )
            noise = float(np.random.default_rng(stable_measurement_seed(
                seed, frame_index, observer_id, kind, reference_id
            )).normal(0.0, ranging_sigma))
            noisy_range = None if true_range is None else true_range + noise

            estimated_available = False
            if kind == "base" and truth_available:
                reference_position = np.asarray(reference_truth, dtype=float)
                covariance = np.zeros((2, 2), dtype=float)
                estimated_available = True
            elif kind == "uav":
                valid = _valid_prior_result(current.get(reference_id))
                if valid is not None:
                    reference_position, covariance, _, _, _ = valid
                    estimated_available = True
            if truth_available and estimated_available:
                positions.append(reference_position)
                covariances.append(covariance)
                measurements.append(noisy_range)
            elif not truth_available:
                errors.append("invalid reference truth")
            else:
                errors.append("invalid upstream UAV reference")
            records.append({
                "kind": kind,
                "id": int(reference_id),
                "true_range": true_range,
                "noise": noise,
                "noisy_range": noisy_range,
                "estimated_reference_available": estimated_available,
            })
    if errors:
        return None, None, None, records, errors[0]
    if len(records) < 2:
        return None, None, None, records, "fewer than two active references"
    return np.asarray(positions), np.asarray(covariances), np.asarray(measurements), records, None


def _transition(previous: dict[str, list[int]] | None, active: dict[str, list[int]], epsilon: float | None, previous_epsilon: float | None) -> dict:
    return {
        "changed": previous is not None and previous != active,
        "previous_active_references": previous,
        "epsilon_change": None if epsilon is None or previous_epsilon is None else epsilon - previous_epsilon,
    }


def _quantiles(values, percents):
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    return {
        str(percent): (
            None if not finite.size else float(np.percentile(finite, percent))
        )
        for percent in percents
    }


class _SummarySamples:
    """Fixed-size compact scalar storage; nested process rows are never retained."""

    DTYPE = np.dtype(
        [
            ("seed", "<i8"),
            ("case", "u1"),
            ("depth", "<u2"),
            ("primary", "?"),
            ("retained_status", "u1"),
            ("attempt_status", "u1"),
            ("containment", "?"),
            ("ratio", "<f8"),
            ("phi_min", "<f8"),
            ("condition", "<f8"),
            ("epsilon", "<f8"),
            ("transition", "?"),
            ("epsilon_change", "<f8"),
        ]
    )

    def __init__(self, capacity: int, seeds: list[int]):
        self._data = np.empty(capacity, dtype=self.DTYPE)
        self._count = 0
        self.seeds = tuple(seeds)

    @property
    def nbytes(self) -> int:
        return int(self._data.nbytes)

    @property
    def count(self) -> int:
        return self._count

    @property
    def data(self) -> np.ndarray:
        return self._data[: self._count]

    def add(self, row: dict) -> None:
        if self._count >= len(self._data):
            raise ValueError("more process rows than the registered capacity")
        transition = row["transition"]
        self._data[self._count] = (
            int(row["seed"]),
            GRAPH_CASES.index(row["graph_case"]),
            int(row["squad_local_index"]),
            bool(row["primary_statistics"]),
            _STATUS_CODES[row["status"]],
            _STATUS_CODES[row["attempt_status"]],
            bool(row["containment"]),
            _sample_float(row.get("error_to_epsilon_ratio")),
            _sample_float(row.get("phi_min_eigenvalue")),
            _sample_float(row.get("phi_condition")),
            _sample_float(row.get("epsilon")),
            bool(transition["changed"]),
            _sample_float(transition.get("epsilon_change")),
        )
        self._count += 1


def _sample_float(value) -> float:
    if value is None:
        return math.nan
    converted = float(value)
    return converted if math.isfinite(converted) else math.nan


def _stats(data: np.ndarray) -> dict:
    retained_statuses = {
        status: int(np.count_nonzero(data["retained_status"] == code))
        for code, status in _CODE_STATUSES.items()
    }
    attempt_statuses = {
        status: int(np.count_nonzero(data["attempt_status"] == code))
        for code, status in _CODE_STATUSES.items()
    }
    denominator = len(data)
    ratios = data["ratio"]
    conditions = data["condition"]
    epsilons = data["epsilon"]
    epsilon_changes = data["epsilon_change"][data["transition"]]
    finite_changes = epsilon_changes[np.isfinite(epsilon_changes)]
    return {
        "robot_frame_count": denominator,
        "status_counts": retained_statuses,
        "attempt_status_counts": attempt_statuses,
        "containment_count": int(np.count_nonzero(data["containment"])),
        "containment_denominator": denominator,
        "containment_rate": (
            None
            if denominator == 0
            else float(np.count_nonzero(data["containment"]) / denominator)
        ),
        "error_to_epsilon_ratio": {
            **_quantiles(ratios, (50, 95, 99)),
            "max": _finite_max(ratios),
        },
        "phi_min_eigenvalue": _quantiles(data["phi_min"], (1, 5, 50)),
        "phi_condition": {
            **_quantiles(conditions, (50, 95, 99)),
            "max": _finite_max(conditions),
        },
        "epsilon": {
            **_quantiles(epsilons, (50, 95, 99)),
            "max": _finite_max(epsilons),
        },
        "transition_count": int(np.count_nonzero(data["transition"])),
        "epsilon_changes": {
            "finite_count": int(finite_changes.size),
            "minimum": _finite_min(finite_changes),
            "maximum": _finite_max(finite_changes),
            "mean": (
                None if not finite_changes.size else float(np.mean(finite_changes))
            ),
            **_quantiles(finite_changes, (5, 50, 95)),
        },
    }


def _finite_min(values) -> float | None:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return None if not values.size else float(np.min(values))


def _finite_max(values) -> float | None:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return None if not values.size else float(np.max(values))


def _bootstrap(seed_rates: list[float | None]) -> dict:
    usable = np.asarray([rate for rate in seed_rates if rate is not None], dtype=float)
    if not len(usable):
        return {
            "resamples": BOOTSTRAP_RESAMPLES,
            "rng_seed": BOOTSTRAP_SEED,
            "lower": None,
            "upper": None,
        }
    samples = np.random.default_rng(BOOTSTRAP_SEED).choice(
        usable,
        size=(BOOTSTRAP_RESAMPLES, len(usable)),
        replace=True,
    ).mean(axis=1)
    return {
        "resamples": BOOTSTRAP_RESAMPLES,
        "rng_seed": BOOTSTRAP_SEED,
        "lower": float(np.percentile(samples, 2.5)),
        "upper": float(np.percentile(samples, 97.5)),
    }


def _summary(
    samples: _SummarySamples,
    expected_count: int,
    settings: dict,
) -> dict:
    data = samples.data
    cases = {}
    for case_index, case in enumerate(GRAPH_CASES):
        case_data = data[data["case"] == case_index]
        primary = case_data[case_data["primary"]]
        initialization = case_data[~case_data["primary"]]
        by_seed = {}
        by_depth = {}
        for seed in samples.seeds:
            by_seed[str(seed)] = _stats(primary[primary["seed"] == seed])
        for depth in sorted(np.unique(case_data["depth"]).tolist()):
            by_depth[str(depth)] = _stats(primary[primary["depth"] == depth])
        cases[case] = {
            "overall": _stats(primary),
            "initialization_frame": _stats(initialization),
            "by_seed": by_seed,
            "by_squad_local_depth": by_depth,
            "containment_bootstrap_95": _bootstrap(
                [item["containment_rate"] for item in by_seed.values()]
            ),
        }
    dynamic = cases["dynamic_dag_wnls"]["overall"]
    fixed = cases["fixed_refs_wnls"]["overall"]
    depth_rates = [item["containment_rate"] for item in cases["dynamic_dag_wnls"]["by_squad_local_depth"].values() if item["containment_rate"] is not None]
    dynamic_invalid = dynamic["attempt_status_counts"]["invalid"] / max(1, dynamic["robot_frame_count"])
    fixed_invalid = fixed["attempt_status_counts"]["invalid"] / max(1, fixed["robot_frame_count"])
    dynamic_failed = dynamic["attempt_status_counts"]["failed"] / max(1, dynamic["robot_frame_count"])
    fixed_failed = fixed["attempt_status_counts"]["failed"] / max(1, fixed["robot_frame_count"])
    adequacy = {
        "aggregate_containment_at_least_0_98": dynamic["containment_rate"] is not None and dynamic["containment_rate"] >= 0.98,
        "every_depth_containment_at_least_0_95": bool(depth_rates) and min(depth_rates) >= 0.95,
        "zero_silently_discarded_failures": samples.count == expected_count,
        "dynamic_invalid_rate_no_worse": dynamic_invalid <= fixed_invalid,
        "dynamic_failure_rate_no_worse": dynamic_failed <= fixed_failed,
    }
    adequacy["passed"] = all(adequacy.values())
    return {
        "estimator_contract": ESTIMATOR_CONTRACT_ID,
        "settings": settings,
        "graph_cases": cases,
        "expected_process_rows": expected_count,
        "process_rows": samples.count,
        "summary_scalar_storage_bytes": samples.nbytes,
        "adequacy": adequacy,
    }


def _summary_markdown(summary: dict) -> str:
    adequacy = summary["adequacy"]
    return "# Localization calibration replay\n\n" + "\n".join(
        f"- {key}: {value}" for key, value in adequacy.items()
    ) + "\n"


class _ReplayLimitError(DiskSpaceError):
    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def _limit_reason(output_root: Path, run_root: Path) -> str | None:
    if available_bytes(output_root) < HARD_FLOOR_BYTES:
        return "disk_hard_floor"
    if allocated_bytes(output_root) > OUTPUT_ROOT_CAP_BYTES:
        return "cache_root_cap"
    if allocated_bytes(run_root) > RUN_CAP_BYTES:
        return "cache_run_cap"
    return None


def _enforce_limits(output_root: Path, run_root: Path) -> None:
    reason = _limit_reason(output_root, run_root)
    if reason is not None:
        raise _ReplayLimitError(reason)


def _parse_ranging_sigma(config: dict) -> float:
    try:
        raw = np.asarray(config["position_covariance"]["ranging_sigma"])
        if raw.ndim != 0:
            raise ValueError
        sigma = float(raw)
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError(
            "config.position_covariance.ranging_sigma must be a finite scalar"
        ) from None
    if not math.isfinite(sigma) or sigma <= 0.0:
        raise ValueError(
            "config.position_covariance.ranging_sigma must be positive and finite"
        )
    if sigma != PREREGISTERED_RANGING_SIGMA:
        raise ValueError(
            "ranging sigma differs from preregistered 0.5 m setting"
        )
    return sigma


def _settings(
    seeds: list[int],
    max_frames: int | None,
    effective_frames: int,
    ranging_sigma: float,
) -> dict:
    implementation_path = Path(__file__).resolve()
    return {
        "estimator_contract": ESTIMATOR_CONTRACT_ID,
        "run_seeds": seeds,
        "graph_cases": list(GRAPH_CASES),
        "max_frames": max_frames,
        "effective_frame_count": effective_frames,
        "ranging_sigma": ranging_sigma,
        "lm_fim": {
            "max_iterations": MAX_ITERATIONS,
            "initial_damping": INITIAL_DAMPING,
            "step_tolerance": STEP_TOLERANCE,
            "cost_tolerance": COST_TOLERANCE,
            "range_epsilon": RANGE_EPSILON,
            "relative_spectral_threshold": RELATIVE_SPECTRAL_THRESHOLD,
            "epsilon_sigma_multiplier": 3.0,
            "bootstrap_resamples": BOOTSTRAP_RESAMPLES,
            "bootstrap_seed": BOOTSTRAP_SEED,
        },
        "implementation": {
            "identity": IMPLEMENTATION_ID,
            "path": str(implementation_path),
            "sha256": _sha256(implementation_path),
        },
    }


def _retained_after_attempt(previous_result: dict | None, attempt: dict) -> dict:
    valid_prior = _valid_prior_result(previous_result)
    if valid_prior is None:
        return attempt
    estimate, covariance, epsilon, phi_minimum, phi_condition = valid_prior
    stale = _result(
        "stale",
        estimate=estimate,
        covariance=covariance,
        epsilon=epsilon,
        phi_min_eigenvalue=phi_minimum,
        phi_condition=phi_condition,
        iterations=attempt["iterations"],
        cost=attempt["cost"],
        stationarity_norm=attempt.get("stationarity_norm"),
        failure_reason=attempt["failure_reason"],
    )
    stale["failure"] = attempt
    return stale


def replay_calibration(data_path, manifest_path, output_root, run_seeds, project_root, max_frames=None) -> dict:
    """Replay truth trajectories into a disk-guarded, paired calibration bundle."""
    data_path, manifest_path, output_root, project_root = map(Path, (data_path, manifest_path, output_root, project_root))
    _validate_output_root(project_root, output_root)
    free_before = require_start_space(_nearest_existing_ancestor(output_root))
    data = _strict_load(data_path)
    input_manifest = _strict_load(manifest_path)
    config = data.get("config")
    if not isinstance(config, dict):
        raise ValueError("truth replay must contain config")
    try:
        number = int(config["num"])
        parts = int(config["formation"]["parts"])
    except (KeyError, TypeError, ValueError):
        raise ValueError("config must provide positive num and formation parts") from None
    if number <= 0 or parts <= 0:
        raise ValueError("num and formation parts must be positive")
    frames = _frames(data)
    input_frame_count = len(frames)
    if max_frames is not None:
        if not isinstance(max_frames, int) or max_frames <= 0:
            raise ValueError("max_frames must be a positive integer")
        frames = frames[:max_frames]
    seeds = list(run_seeds) if run_seeds else [int(config.get("execute", {}).get("random-seed", 0))]
    if not seeds:
        raise ValueError("at least one run seed is required")
    seeds = [int(seed) for seed in seeds]
    if len(seeds) != len(set(seeds)):
        raise ValueError("duplicate run seeds are not allowed")
    ranging_sigma = _parse_ranging_sigma(config)
    expected_ids = set(range(1, number + 1))
    deployment = _initial_positions(config, expected_ids)
    settings = _settings(seeds, max_frames, len(frames), ranging_sigma)
    run_root = _allocate_run_root(output_root / "localization-calibration")
    started_at = datetime.now(timezone.utc).isoformat()
    process_path = run_root / "calibration.jsonl.gz"
    summary_path = run_root / "summary.json"
    markdown_path = run_root / "summary.md"
    expected_count = len(seeds) * len(GRAPH_CASES) * number * len(frames)
    samples = _SummarySamples(expected_count, seeds)
    termination_reason = "completed"
    error = None
    states = {}
    decompressed_digest = hashlib.sha256()

    def terminal_manifest():
        nonlocal termination_reason, error
        manifest = {
            "termination_reason": termination_reason,
            "estimator_contract": ESTIMATOR_CONTRACT_ID,
            "output_dir": str(run_root),
            "started_at": started_at,
            "ended_at": datetime.now(timezone.utc).isoformat(),
            "input_data": {"path": str(data_path.resolve()), "sha256": _sha256(data_path)},
            "input_manifest": {"path": str(manifest_path.resolve()), "sha256": _sha256(manifest_path), "source_commit": input_manifest.get("base_commit"), "materialized_config_sha256": input_manifest.get("config_sha256")},
            "free_bytes_before": free_before,
            "free_bytes_after": _safe_disk(output_root),
            "output_root_allocated_bytes": _safe_allocated(output_root),
            "bundle_allocated_bytes": _safe_allocated(run_root),
            "output_root_cap_bytes": OUTPUT_ROOT_CAP_BYTES,
            "bundle_cap_bytes": RUN_CAP_BYTES,
            "settings": settings,
            "input_frame_count": input_frame_count,
            "compressed_process_sha256": _sha256(process_path) if process_path.exists() else None,
            "decompressed_process_sha256": (
                decompressed_digest.hexdigest() if process_path.exists() else None
            ),
            "summary_json_sha256": _sha256(summary_path) if summary_path.exists() else None,
            "summary_markdown_sha256": _sha256(markdown_path) if markdown_path.exists() else None,
        }
        if error is not None:
            manifest["error"] = {"type": type(error).__name__, "message": str(error)}
        _write_manifest(run_root, _json_value(manifest))
        try:
            final_reason = _limit_reason(output_root, run_root)
        except Exception as probe_error:
            if termination_reason == "completed":
                termination_reason = "runner_setup_error"
                error = probe_error
                manifest["termination_reason"] = termination_reason
                manifest["error"] = {
                    "type": type(probe_error).__name__,
                    "message": str(probe_error),
                }
                _write_manifest(run_root, _json_value(manifest))
            return manifest
        if termination_reason == "completed" and final_reason is not None:
            termination_reason = final_reason
            manifest["termination_reason"] = final_reason
            manifest["free_bytes_after"] = _safe_disk(output_root)
            manifest["output_root_allocated_bytes"] = _safe_allocated(output_root)
            manifest["bundle_allocated_bytes"] = _safe_allocated(run_root)
            _write_manifest(run_root, _json_value(manifest))
        return manifest

    try:
        _enforce_limits(output_root, run_root)
        with process_path.open("wb") as raw, gzip.GzipFile(fileobj=raw, mode="wb", mtime=0) as compressed:
            for frame_index, frame in enumerate(frames):
                truth = _truth_positions(frame, expected_ids)
                for seed in seeds:
                    for graph_case in GRAPH_CASES:
                        state_key = (seed, graph_case)
                        case_state = states.setdefault(state_key, ({}, {}, {}))
                        previous, previous_active, previous_epsilon = case_state
                        squad_size = math.ceil(number / parts)
                        for robot_id in range(1, number + 1):
                            local_index = (robot_id - 1) % squad_size + 1
                            refs = active_references(config, robot_id, truth) if graph_case == "dynamic_dag_wnls" else fixed_references(config, robot_id)
                            inputs = _reference_inputs(
                                refs,
                                config,
                                truth,
                                previous,
                                seed,
                                frame_index,
                                robot_id,
                                ranging_sigma,
                            )
                            reference_positions, reference_covariances, measurements, records, input_error = inputs
                            initial = deployment[robot_id] if frame_index == 0 else np.asarray(previous.get(robot_id, {}).get("estimate", deployment[robot_id]), dtype=float)
                            if input_error is not None:
                                attempt = _invalid_result(input_error)
                                result = (
                                    attempt
                                    if frame_index == 0
                                    else _retained_after_attempt(
                                        previous.get(robot_id), attempt
                                    )
                                )
                            elif frame_index == 0:
                                attempt = solve_wnls(reference_positions, reference_covariances, measurements, initial, ranging_sigma)
                                result = attempt
                            else:
                                result = solve_later_frame(
                                    previous.get(robot_id),
                                    reference_positions,
                                    reference_covariances,
                                    measurements,
                                    initial,
                                    ranging_sigma,
                                )
                                attempt = (
                                    result["failure"]
                                    if result["status"] == "stale"
                                    else result
                                )
                            estimate = result.get("estimate")
                            error_vector = None if estimate is None else (truth[robot_id] - np.asarray(estimate, dtype=float)).tolist()
                            error_norm = None if error_vector is None else float(np.linalg.norm(error_vector))
                            epsilon = result.get("epsilon")
                            valid_result = _valid_prior_result(result)
                            state_containment = (
                                None
                                if error_norm is None or epsilon is None
                                else error_norm <= epsilon
                            )
                            containment = (
                                attempt["status"] == "converged"
                                and state_containment is True
                            )
                            row = {
                                "seed": seed, "graph_case": graph_case, "frame_index": frame_index,
                                "robot_id": robot_id, "squad_local_index": local_index,
                                "primary_statistics": frame_index != 0, "active_references": refs,
                                "measurements": records, "truth_position": truth[robot_id].tolist(),
                                "status": result["status"], "estimate": estimate,
                                "attempt_status": attempt["status"],
                                "attempt_failure_reason": attempt.get("failure_reason"),
                                "attempt_stationarity_norm": attempt.get(
                                    "stationarity_norm"
                                ),
                                "covariance": result.get("covariance"), "epsilon": epsilon,
                                "finite": valid_result is not None,
                                "covariance_spd": valid_result is not None,
                                "error_vector": error_vector, "error_norm": error_norm,
                                "error_to_epsilon_ratio": None if error_norm is None or epsilon is None or epsilon <= 0 else error_norm / epsilon,
                                "state_containment": state_containment,
                                "containment": containment,
                                "phi_min_eigenvalue": result.get("phi_min_eigenvalue"), "phi_condition": result.get("phi_condition"),
                                "iterations": result.get("iterations"), "cost": result.get("cost"),
                                "stationarity_norm": result.get(
                                    "stationarity_norm"
                                ),
                                "failure_reason": result.get("failure_reason"),
                                "failure": (
                                    attempt if attempt["status"] != "converged" else None
                                ),
                                "transition": _transition(previous_active.get(robot_id), refs, epsilon, previous_epsilon.get(robot_id)),
                            }
                            row = _json_value(row)
                            line = _strict_json_bytes(row) + b"\n"
                            compressed.write(line)
                            decompressed_digest.update(line)
                            samples.add(row)
                            previous[robot_id] = result
                            previous_active[robot_id] = refs
                            previous_epsilon[robot_id] = epsilon
                        states[state_key] = (previous, previous_active, previous_epsilon)
                compressed.flush()
                _enforce_limits(output_root, run_root)
        summary = _summary(samples, expected_count, settings)
        summary_path.write_bytes(_strict_json_bytes(summary, indent=2) + b"\n")
        markdown_path.write_text(_summary_markdown(summary))
        _enforce_limits(output_root, run_root)
    except _ReplayLimitError as caught:
        error = caught
        termination_reason = caught.reason
    except Exception as caught:
        error = caught
        termination_reason = "runner_setup_error"
    return terminal_manifest()


def _safe_disk(path: Path) -> int | None:
    try:
        return available_bytes(path)
    except Exception:
        return None


def _safe_allocated(path: Path) -> int | None:
    try:
        return allocated_bytes(path)
    except Exception:
        return None


def main(argv: list[str] | None = None) -> int:
    """Run the offline calibration evidence replay command."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument("--seed", action="append", type=int, default=[])
    parser.add_argument("--max-frames", type=int)
    arguments = parser.parse_args(argv)
    project_root = Path(__file__).resolve().parents[2]
    try:
        manifest = replay_calibration(arguments.data, arguments.manifest, arguments.output_root, arguments.seed, project_root, arguments.max_frames)
    except (DiskSpaceError, ValueError, OSError) as error:
        parser.error(str(error))
    print(json.dumps(manifest, indent=2, allow_nan=False))
    return 0 if manifest["termination_reason"] == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
