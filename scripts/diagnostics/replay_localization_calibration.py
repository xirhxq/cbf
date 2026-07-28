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
    direction = diff / np.maximum(distance[:, None], RANGE_EPSILON)
    projected_variance = np.einsum(
        "ni,nij,nj->n", direction, reference_covariances, direction
    )
    total_variance = ranging_sigma**2 + projected_variance
    if not np.isfinite(total_variance).all() or np.any(total_variance <= 0.0):
        return None
    weight = 1.0 / total_variance
    residual = distance - measurements
    if not np.isfinite(weight).all() or not np.isfinite(residual).all():
        return None
    phi = direction.T @ (weight[:, None] * direction)
    gradient = direction.T @ (weight * residual)
    cost = float(np.sum(weight * residual**2))
    if (
        not np.isfinite(phi).all()
        or not np.isfinite(gradient).all()
        or not np.isfinite(cost)
    ):
        return None
    return direction, weight, residual, phi, gradient, cost


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
    zero_measurements = np.zeros(references.shape[0], dtype=float)
    terms = _linearized_terms(
        final_estimate, references, covariances, zero_measurements, sigma
    )
    if terms is None:
        return _result("invalid", failure_reason="invalid final FIM weights")
    _, _, _, phi, _, _ = terms
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

    for iteration in range(1, MAX_ITERATIONS + 1):
        terms = _linearized_terms(estimate, references, covariances, ranges, sigma)
        if terms is None:
            return _result(
                "invalid", iterations=iteration - 1, failure_reason="invalid WNLS terms"
            )
        _, _, _, phi, gradient, cost = terms
        if np.linalg.norm(gradient) <= STEP_TOLERANCE:
            final = fim_radius(estimate, references, covariances, sigma)
            final["iterations"] = iteration
            final["cost"] = cost
            return final
        try:
            delta = np.linalg.solve(phi + damping * np.eye(2), -gradient)
        except np.linalg.LinAlgError:
            return _result(
                "invalid",
                iterations=iteration - 1,
                cost=cost,
                failure_reason="damped solve failed",
            )
        if not np.isfinite(delta).all():
            return _result(
                "invalid",
                iterations=iteration - 1,
                cost=cost,
                failure_reason="non-finite WNLS step",
            )
        trial_estimate = estimate + delta
        trial_terms = _linearized_terms(
            trial_estimate, references, covariances, ranges, sigma
        )
        if trial_terms is not None and trial_terms[-1] < cost:
            trial_gradient = trial_terms[4]
            trial_cost = trial_terms[5]
            estimate = trial_estimate
            damping /= 10.0
            if (
                np.linalg.norm(delta) <= STEP_TOLERANCE
                or cost - trial_cost <= COST_TOLERANCE
            ) and np.linalg.norm(trial_gradient) <= STEP_TOLERANCE:
                final = fim_radius(estimate, references, covariances, sigma)
                final["iterations"] = iteration
                final["cost"] = trial_cost
                return final
        else:
            damping *= 10.0

    return _result(
        "failed",
        iterations=MAX_ITERATIONS,
        cost=cost,
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
            failure_reason=result["failure_reason"],
        )
        stale["failure"] = result
        return stale
    return result


GRAPH_CASES = ("dynamic_dag_wnls", "fixed_refs_wnls")
RANGING_SIGMA = 0.5


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
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, list[dict], str | None]:
    positions = []
    covariances = []
    measurements = []
    records = []
    observer_truth = truth[observer_id]
    for kind, identifiers in (("base", references["base_ids"]), ("uav", references["uav_ids"])):
        for reference_id in identifiers:
            if kind == "base":
                try:
                    reference_truth = np.asarray(config["bases"][reference_id], dtype=float)
                except (IndexError, KeyError, TypeError, ValueError):
                    return None, None, None, records, "invalid base reference"
                reference_position = reference_truth
                covariance = np.zeros((2, 2), dtype=float)
            else:
                reference_truth = truth[reference_id]
                upstream = current.get(reference_id)
                valid = _valid_prior_result(upstream)
                if valid is None:
                    return None, None, None, records, "invalid upstream UAV reference"
                reference_position, covariance, _, _, _ = valid
            true_range = float(np.linalg.norm(observer_truth - reference_truth))
            noise = float(np.random.default_rng(stable_measurement_seed(
                seed, frame_index, observer_id, kind, reference_id
            )).normal(0.0, RANGING_SIGMA))
            noisy_range = true_range + noise
            positions.append(reference_position)
            covariances.append(covariance)
            measurements.append(noisy_range)
            records.append({
                "kind": kind,
                "id": int(reference_id),
                "true_range": true_range,
                "noise": noise,
                "noisy_range": noisy_range,
            })
    if len(positions) < 2:
        return None, None, None, records, "fewer than two active references"
    return np.asarray(positions), np.asarray(covariances), np.asarray(measurements), records, None


def _transition(previous: dict[str, list[int]] | None, active: dict[str, list[int]], epsilon: float | None, previous_epsilon: float | None) -> dict:
    return {
        "changed": previous is not None and previous != active,
        "previous_active_references": previous,
        "epsilon_change": None if epsilon is None or previous_epsilon is None else epsilon - previous_epsilon,
    }


def _finite(values):
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


def _quantiles(values, percents):
    values = _finite(values)
    return {str(percent): (None if not values else float(np.percentile(values, percent))) for percent in percents}


def _stats(rows: list[dict]) -> dict:
    primary = [row for row in rows if row["primary_statistics"]]
    statuses = {status: sum(row["status"] == status for row in primary) for status in ("converged", "stale", "invalid", "failed")}
    containments = [row["containment"] for row in primary if row["containment"] is not None]
    ratios = [row["error_to_epsilon_ratio"] for row in primary]
    transitions = [row for row in rows if row["transition"]["changed"]]
    epsilon_changes = [row["transition"]["epsilon_change"] for row in transitions]
    return {
        "robot_frame_count": len(primary),
        "status_counts": statuses,
        "containment_count": sum(containments),
        "containment_denominator": len(containments),
        "containment_rate": None if not containments else sum(containments) / len(containments),
        "error_to_epsilon_ratio": {**_quantiles(ratios, (50, 95, 99)), "max": None if not _finite(ratios) else max(_finite(ratios))},
        "phi_min_eigenvalue": _quantiles([row["phi_min_eigenvalue"] for row in primary], (1, 5, 50)),
        "phi_condition": {**_quantiles([row["phi_condition"] for row in primary], (50, 95, 99)), "max": None if not _finite([row["phi_condition"] for row in primary]) else max(_finite([row["phi_condition"] for row in primary]))},
        "epsilon": {**_quantiles([row["epsilon"] for row in primary], (50, 95, 99)), "max": None if not _finite([row["epsilon"] for row in primary]) else max(_finite([row["epsilon"] for row in primary]))},
        "transition_count": len(transitions),
        "epsilon_changes": epsilon_changes,
    }


def _bootstrap(seed_rates: list[float | None]) -> dict:
    usable = np.asarray([rate for rate in seed_rates if rate is not None], dtype=float)
    if not len(usable):
        return {"resamples": 10000, "rng_seed": 20260728, "lower": None, "upper": None}
    samples = np.random.default_rng(20260728).choice(usable, size=(10000, len(usable)), replace=True).mean(axis=1)
    return {"resamples": 10000, "rng_seed": 20260728, "lower": float(np.percentile(samples, 2.5)), "upper": float(np.percentile(samples, 97.5))}


def _summary(rows: list[dict], expected_count: int) -> dict:
    cases = {}
    for case in GRAPH_CASES:
        case_rows = [row for row in rows if row["graph_case"] == case]
        by_seed = {}
        by_depth = {}
        for seed in sorted({row["seed"] for row in case_rows}):
            by_seed[str(seed)] = _stats([row for row in case_rows if row["seed"] == seed])
        for depth in sorted({row["squad_local_index"] for row in case_rows}):
            by_depth[str(depth)] = _stats([row for row in case_rows if row["squad_local_index"] == depth])
        cases[case] = {"overall": _stats(case_rows), "by_seed": by_seed, "by_squad_local_depth": by_depth, "containment_bootstrap_95": _bootstrap([item["containment_rate"] for item in by_seed.values()])}
    dynamic = cases["dynamic_dag_wnls"]["overall"]
    fixed = cases["fixed_refs_wnls"]["overall"]
    depth_rates = [item["containment_rate"] for item in cases["dynamic_dag_wnls"]["by_squad_local_depth"].values() if item["containment_rate"] is not None]
    dynamic_invalid = dynamic["status_counts"]["invalid"] / max(1, dynamic["robot_frame_count"])
    fixed_invalid = fixed["status_counts"]["invalid"] / max(1, fixed["robot_frame_count"])
    dynamic_failed = dynamic["status_counts"]["failed"] / max(1, dynamic["robot_frame_count"])
    fixed_failed = fixed["status_counts"]["failed"] / max(1, fixed["robot_frame_count"])
    adequacy = {
        "aggregate_containment_at_least_0_98": dynamic["containment_rate"] is not None and dynamic["containment_rate"] >= 0.98,
        "every_depth_containment_at_least_0_95": bool(depth_rates) and min(depth_rates) >= 0.95,
        "zero_silently_discarded_failures": len(rows) == expected_count,
        "dynamic_invalid_rate_no_worse": dynamic_invalid <= fixed_invalid,
        "dynamic_failure_rate_no_worse": dynamic_failed <= fixed_failed,
    }
    adequacy["passed"] = all(adequacy.values())
    return {"graph_cases": cases, "expected_process_rows": expected_count, "process_rows": len(rows), "adequacy": adequacy}


def _summary_markdown(summary: dict) -> str:
    adequacy = summary["adequacy"]
    return "# Localization calibration replay\n\n" + "\n".join(
        f"- {key}: {value}" for key, value in adequacy.items()
    ) + "\n"


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
    if max_frames is not None:
        if not isinstance(max_frames, int) or max_frames <= 0:
            raise ValueError("max_frames must be a positive integer")
        frames = frames[:max_frames]
    seeds = list(run_seeds) if run_seeds else [int(config.get("execute", {}).get("random-seed", 0))]
    if not seeds:
        raise ValueError("at least one run seed is required")
    seeds = [int(seed) for seed in seeds]
    expected_ids = set(range(1, number + 1))
    deployment = _initial_positions(config, expected_ids)
    run_root = _allocate_run_root(output_root / "localization-calibration")
    started_at = datetime.now(timezone.utc).isoformat()
    process_path = run_root / "calibration.jsonl.gz"
    summary_path = run_root / "summary.json"
    markdown_path = run_root / "summary.md"
    rows = []
    termination_reason = "completed"
    error = None
    states = {}

    def terminal_manifest():
        manifest = {
            "termination_reason": termination_reason,
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
            "process_sha256": _sha256(process_path) if process_path.exists() else None,
            "summary_json_sha256": _sha256(summary_path) if summary_path.exists() else None,
            "summary_markdown_sha256": _sha256(markdown_path) if markdown_path.exists() else None,
        }
        if error is not None:
            manifest["error"] = {"type": type(error).__name__, "message": str(error)}
        _write_manifest(run_root, _json_value(manifest))
        return manifest

    try:
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
                            inputs = _reference_inputs(refs, config, truth, previous, seed, frame_index, robot_id)
                            reference_positions, reference_covariances, measurements, records, input_error = inputs
                            initial = deployment[robot_id] if frame_index == 0 else np.asarray(previous.get(robot_id, {}).get("estimate", deployment[robot_id]), dtype=float)
                            if input_error is not None:
                                result = _invalid_result(input_error)
                                if frame_index > 0:
                                    valid = _valid_prior_result(previous.get(robot_id))
                                    if valid is not None:
                                        estimate, covariance, epsilon, phi_minimum, phi_condition = valid
                                        result = _result("stale", estimate=estimate, covariance=covariance, epsilon=epsilon, phi_min_eigenvalue=phi_minimum, phi_condition=phi_condition, failure_reason=input_error)
                            elif frame_index == 0:
                                result = solve_wnls(reference_positions, reference_covariances, measurements, initial, RANGING_SIGMA)
                            else:
                                result = solve_later_frame(previous.get(robot_id), reference_positions, reference_covariances, measurements, initial, RANGING_SIGMA)
                            estimate = result.get("estimate")
                            error_vector = None if estimate is None else (truth[robot_id] - np.asarray(estimate, dtype=float)).tolist()
                            error_norm = None if error_vector is None else float(np.linalg.norm(error_vector))
                            epsilon = result.get("epsilon")
                            valid_result = _valid_prior_result(result)
                            row = {
                                "seed": seed, "graph_case": graph_case, "frame_index": frame_index,
                                "robot_id": robot_id, "squad_local_index": local_index,
                                "primary_statistics": frame_index != 0, "active_references": refs,
                                "measurements": records, "truth_position": truth[robot_id].tolist(),
                                "status": result["status"], "estimate": estimate,
                                "covariance": result.get("covariance"), "epsilon": epsilon,
                                "finite": valid_result is not None,
                                "covariance_spd": valid_result is not None,
                                "error_vector": error_vector, "error_norm": error_norm,
                                "error_to_epsilon_ratio": None if error_norm is None or epsilon is None or epsilon <= 0 else error_norm / epsilon,
                                "containment": None if error_norm is None or epsilon is None else error_norm <= epsilon,
                                "phi_min_eigenvalue": result.get("phi_min_eigenvalue"), "phi_condition": result.get("phi_condition"),
                                "iterations": result.get("iterations"), "cost": result.get("cost"),
                                "failure_reason": result.get("failure_reason"), "failure": result.get("failure"),
                                "transition": _transition(previous_active.get(robot_id), refs, epsilon, previous_epsilon.get(robot_id)),
                            }
                            row = _json_value(row)
                            compressed.write(_strict_json_bytes(row) + b"\n")
                            rows.append(row)
                            previous[robot_id] = result
                            previous_active[robot_id] = refs
                            previous_epsilon[robot_id] = epsilon
                        states[state_key] = (previous, previous_active, previous_epsilon)
                compressed.flush()
                if available_bytes(output_root) < HARD_FLOOR_BYTES:
                    raise DiskSpaceError("live disk floor reached")
                if allocated_bytes(output_root) > OUTPUT_ROOT_CAP_BYTES:
                    termination_reason = "cache_root_cap"
                    raise DiskSpaceError("output root cap reached")
                if allocated_bytes(run_root) > RUN_CAP_BYTES:
                    termination_reason = "cache_run_cap"
                    raise DiskSpaceError("bundle cap reached")
        summary = _summary(rows, len(seeds) * len(GRAPH_CASES) * number * len(frames))
        summary_path.write_bytes(_strict_json_bytes(summary, indent=2) + b"\n")
        markdown_path.write_text(_summary_markdown(summary))
    except DiskSpaceError as caught:
        error = caught
        if termination_reason == "completed":
            termination_reason = "disk_hard_floor"
    except BaseException as caught:
        error = caught
        termination_reason = "runner_setup_error"
    return terminal_manifest()


def _safe_disk(path: Path) -> int | None:
    try:
        return available_bytes(path)
    except BaseException:
        return None


def _safe_allocated(path: Path) -> int | None:
    try:
        return allocated_bytes(path)
    except BaseException:
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
