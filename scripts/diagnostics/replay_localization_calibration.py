"""Pure-NumPy primitives for offline localization calibration replay."""

import hashlib
import math

import numpy as np


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
    references = np.asarray(reference_positions, dtype=float)
    covariances = np.asarray(reference_covariances, dtype=float)
    ranges = None if measurements is None else np.asarray(measurements, dtype=float)
    estimate = np.asarray(initial_estimate, dtype=float)
    sigma = float(ranging_sigma)
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
    return references, covariances, ranges, estimate, sigma


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
        if np.linalg.norm(delta) <= STEP_TOLERANCE:
            final = fim_radius(estimate, references, covariances, sigma)
            final["iterations"] = iteration
            final["cost"] = cost
            return final

        trial_estimate = estimate + delta
        trial_terms = _linearized_terms(
            trial_estimate, references, covariances, ranges, sigma
        )
        if trial_terms is not None and trial_terms[-1] < cost:
            trial_cost = trial_terms[-1]
            estimate = trial_estimate
            damping /= 10.0
            if cost - trial_cost <= COST_TOLERANCE:
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
    if (
        previous_result is not None
        and previous_result.get("estimate") is not None
        and previous_result.get("covariance") is not None
        and np.isfinite(np.asarray(previous_result["estimate"], dtype=float)).all()
        and np.isfinite(np.asarray(previous_result["covariance"], dtype=float)).all()
    ):
        stale = _result(
            "stale",
            estimate=np.asarray(previous_result["estimate"], dtype=float),
            covariance=np.asarray(previous_result["covariance"], dtype=float),
            epsilon=previous_result.get("epsilon"),
            phi_min_eigenvalue=previous_result.get("phi_min_eigenvalue"),
            phi_condition=previous_result.get("phi_condition"),
            iterations=result["iterations"],
            cost=result["cost"],
            failure_reason=result["failure_reason"],
        )
        stale["failure"] = result
        return stale
    return result
