"""Extract conservative, reproducible metrics from one diagnostic run.

The simulator writes the linear hard-CBF condition as ``a^T u + const >= 0``.
This module keeps that representation explicit: the feasibility calculation uses
``rhs = -const`` and residual statistics are recomputed from the recorded
coefficient, constant, and applied control instead of trusting presentation-only
fields.
"""

import argparse
import itertools
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np


RESIDUAL_TOLERANCE = -1e-7
INPUT_LIMIT_TOLERANCE = 1e-7
PRACTICAL_NEGATIVE_TOLERANCE_M = -0.5
UNAVAILABLE_METRIC = "unavailable_no_distinct_truth_estimate_or_input_bounds"


def _finite_number(value: Any) -> float | None:
    """Return a finite numeric value, or ``None`` for missing/invalid input."""
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _summary_statistics(values: list[float]) -> dict[str, float | int | None]:
    """Return JSON-safe distribution statistics without inventing empty values."""
    if not values:
        return {
            "count": 0,
            "min": None,
            "max": None,
            "mean": None,
            "p05": None,
            "p50": None,
            "p95": None,
        }
    array = np.asarray(values, dtype=float)
    return {
        "count": int(array.size),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
        "mean": float(np.mean(array)),
        "p05": float(np.quantile(array, 0.05)),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
    }


def minimum_linf_bound(constraints: list[dict], control_names: list[str]) -> float:
    """Return the least ``t`` for a control with ``||u||_inf <= t``.

    Every input constraint is interpreted as ``coe dot u >= rhs``.  The compact
    diagnostic QPs have a small control dimension, so enumerating all vertices
    of the epigraph LP avoids introducing a second solver dependency.
    """
    if not constraints:
        return 0.0
    if not control_names:
        return math.inf

    dimension = len(control_names)
    rows: list[np.ndarray] = []
    rhs_values: list[float] = []
    for constraint in constraints:
        coe = constraint.get("coe")
        rhs = _finite_number(constraint.get("rhs"))
        if not isinstance(coe, dict) or rhs is None:
            return math.inf
        coefficient = []
        for name in control_names:
            value = _finite_number(coe.get(name, 0.0))
            if value is None:
                return math.inf
            coefficient.append(value)
        rows.append(np.asarray([*coefficient, 0.0], dtype=float))
        rhs_values.append(rhs)

    # u_j + t >= 0, -u_j + t >= 0, and t >= 0.
    for index in range(dimension):
        lower = np.zeros(dimension + 1, dtype=float)
        lower[index] = 1.0
        lower[-1] = 1.0
        rows.append(lower)
        rhs_values.append(0.0)

        upper = np.zeros(dimension + 1, dtype=float)
        upper[index] = -1.0
        upper[-1] = 1.0
        rows.append(upper)
        rhs_values.append(0.0)

    nonnegative_t = np.zeros(dimension + 1, dtype=float)
    nonnegative_t[-1] = 1.0
    rows.append(nonnegative_t)
    rhs_values.append(0.0)

    matrix = np.asarray(rows, dtype=float)
    rhs_vector = np.asarray(rhs_values, dtype=float)
    vertex_size = dimension + 1
    best = math.inf
    feasibility_tolerance = 1e-9

    for active_rows in itertools.combinations(range(len(rows)), vertex_size):
        active_matrix = matrix[list(active_rows)]
        if np.linalg.matrix_rank(active_matrix) != vertex_size:
            continue
        try:
            candidate = np.linalg.solve(active_matrix, rhs_vector[list(active_rows)])
        except np.linalg.LinAlgError:
            continue
        if not np.all(np.isfinite(candidate)):
            continue
        if np.all(matrix @ candidate >= rhs_vector - feasibility_tolerance):
            best = min(best, float(candidate[-1]))

    return best if math.isfinite(best) else math.inf


def _load_json(path: Path, label: str) -> dict:
    try:
        payload = json.loads(path.read_text())
    except FileNotFoundError as error:
        raise ValueError(f"{label} does not exist: {path}") from error
    except OSError as error:
        raise ValueError(f"cannot read {label}: {path}: {error}") from error
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid {label} JSON: {path}: {error.msg}") from error
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be a JSON object: {path}")
    return payload


def _record_nonfinite(failures: list[str], value: Any, path: str) -> float | None:
    number = _finite_number(value)
    if number is None:
        failures.append(path)
    return number


def _point_from_state(robot: dict, failures: list[str], path: str) -> tuple[float, float] | None:
    state = robot.get("state")
    if not isinstance(state, dict):
        failures.append(f"{path}.state")
        return None
    x = _record_nonfinite(failures, state.get("x"), f"{path}.state.x")
    y = _record_nonfinite(failures, state.get("y"), f"{path}.state.y")
    return (x, y) if x is not None and y is not None else None


def _base_point(bases: Any, index: Any) -> tuple[float, float] | None:
    if not isinstance(index, int) or not isinstance(bases, list) or index < 0 or index >= len(bases):
        return None
    base = bases[index]
    if isinstance(base, list) and len(base) >= 2:
        x, y = _finite_number(base[0]), _finite_number(base[1])
        return (x, y) if x is not None and y is not None else None
    if isinstance(base, dict):
        x, y = _finite_number(base.get("x")), _finite_number(base.get("y"))
        return (x, y) if x is not None and y is not None else None
    return None


def _metric_minimum(values: list[float]) -> float | None:
    return float(min(values)) if values else None


def _metric_maximum(values: list[float]) -> float | None:
    return float(max(values)) if values else None


def _neighbor_constraint_source(name: Any) -> tuple[int, str] | None:
    """Extract neighbor and source from stable hard neighbor-row names."""
    if not isinstance(name, str) or not name.endswith(")"):
        return None
    prefixes = (
        ("safetyCBF(#", "safety"),
        ("fixedCommCBF(#", "fixed_communication"),
    )
    for prefix, source in prefixes:
        if not name.startswith(prefix):
            continue
        try:
            return int(name[len(prefix):-1]), source
        except ValueError:
            return None
    return None


def _record_physical_margin(
    records: dict[str, dict[int, tuple[float, float]]],
    key: str,
    frame_index: int,
    nominal: float,
    tightened: float,
) -> None:
    """Keep the strictest observation for one physical key in one frame."""
    frame_records = records.setdefault(key, {})
    previous = frame_records.get(frame_index)
    if previous is None:
        frame_records[frame_index] = (nominal, tightened)
        return
    frame_records[frame_index] = (
        min(previous[0], nominal),
        min(previous[1], tightened),
    )


def _practical_tolerance_summary(
    records: dict[str, dict[int, tuple[float, float]]],
) -> dict:
    nominal_values: list[float] = []
    tightened_values: list[float] = []
    below_records: list[dict] = []
    negative_observation_count = 0
    isolated_tolerance_event_count = 0
    consecutive_negative_keys: list[str] = []

    for key in sorted(records):
        by_frame = records[key]
        negative_frames = {
            frame_index
            for frame_index, (_, tightened) in by_frame.items()
            if tightened < 0.0
        }
        negative_observation_count += len(negative_frames)
        if any(frame_index + 1 in negative_frames for frame_index in negative_frames):
            consecutive_negative_keys.append(key)
        for frame_index in negative_frames:
            if (
                frame_index - 1 not in negative_frames
                and frame_index + 1 not in negative_frames
            ):
                isolated_tolerance_event_count += 1
        for frame_index, (nominal, tightened) in sorted(by_frame.items()):
            nominal_values.append(nominal)
            tightened_values.append(tightened)
            if tightened < PRACTICAL_NEGATIVE_TOLERANCE_M:
                below_records.append(
                    {
                        "key": key,
                        "frame_index": frame_index,
                        "tightened_margin": tightened,
                    }
                )

    if below_records:
        classification = "failed_below_negative_tolerance"
    elif consecutive_negative_keys:
        classification = "failed_consecutive_negative"
    elif negative_observation_count:
        classification = "practical_tolerance_event"
    elif tightened_values:
        classification = "satisfied"
    else:
        classification = "unavailable"

    return {
        "strict_nominal_min": _metric_minimum(nominal_values),
        "strict_tightened_min": _metric_minimum(tightened_values),
        "below_negative_tolerance_count": len(below_records),
        "below_negative_tolerance_records": below_records,
        "negative_observation_count": negative_observation_count,
        "isolated_tolerance_event_count": isolated_tolerance_event_count,
        "has_consecutive_negative_frames": bool(consecutive_negative_keys),
        "consecutive_negative_keys": consecutive_negative_keys,
        "classification": classification,
    }


def _configured_class_k(config: dict, constraint_name: Any) -> dict | None:
    if not isinstance(constraint_name, str):
        return None
    without_slack = config.get("cbfs", {}).get("without-slack", {})
    if constraint_name.startswith("fixedCommCBF"):
        config_key = "comm-fixed"
        cbf_config = without_slack.get("comm-fixed", {})
    elif constraint_name.startswith("safetyCBF"):
        config_key = "safety"
        cbf_config = without_slack.get("safety", {})
    elif constraint_name.startswith("energy"):
        config_key = "energy"
        cbf_config = without_slack.get("energy", {})
    elif constraint_name == "commCBF" or constraint_name.startswith("commCBF("):
        config_key = "comm-auto"
        cbf_config = without_slack.get("comm-auto", {})
    else:
        return None
    alpha = cbf_config.get("alpha", {}) if isinstance(cbf_config, dict) else {}
    coefficient = _finite_number(alpha.get("coe")) if isinstance(alpha, dict) else None
    power = alpha.get("pow") if isinstance(alpha, dict) else None
    configured = (
        {"coefficient": coefficient, "power": power}
        if coefficient is not None and isinstance(power, int)
        else None
    )
    return {
        "name": constraint_name,
        "config_source": f"config.cbfs.without-slack.{config_key}.alpha",
        "configured": configured,
    }


def _aggregate_constraint_names(name: Any) -> list[str] | None:
    """Return top-level names from ``min(a,b,...)``, preserving nested names."""
    if not isinstance(name, str) or not name.startswith("min(") or not name.endswith(")"):
        return None
    body = name[4:-1]
    if not body:
        return []
    names: list[str] = []
    start = 0
    depth = 0
    for index, character in enumerate(body):
        if character == "(":
            depth += 1
        elif character == ")":
            depth -= 1
        elif character == "," and depth == 0:
            names.append(body[start:index])
            start = index + 1
    names.append(body[start:])
    return [item for item in names if item]


def _derive_uncertainty_from_covariance(
    covariance: dict[str, float],
    uncertainty_type: str,
) -> tuple[float | None, str | None]:
    """Mirror ``Robot``'s configured covariance-to-radius conversion."""
    cov_xx = covariance["cov_xx"]
    cov_xy = covariance["cov_xy"]
    cov_yx = covariance["cov_yx"]
    cov_yy = covariance["cov_yy"]
    if uncertainty_type == "max_eigenvalue":
        matrix = np.asarray([[cov_xx, cov_xy], [cov_yx, cov_yy]], dtype=float)
        symmetric = (matrix + matrix.T) / 2.0
        lambda_max = float(np.max(np.linalg.eigvalsh(symmetric)))
        if lambda_max < -1e-12:
            return None, "negative_maximum_eigenvalue"
        return 3.0 * math.sqrt(max(lambda_max, 0.0)), None
    if uncertainty_type == "std_avg":
        if cov_xx < 0.0 or cov_yy < 0.0:
            return None, "negative_covariance_diagonal"
        return (math.sqrt(cov_xx) + math.sqrt(cov_yy)) / 2.0, None
    return None, "unsupported_uncertainty_type"


def _write_markdown(summary: dict, path: Path) -> None:
    """Write a compact, human-readable counterpart of the JSON evidence."""
    hard = summary["hard_constraints"]
    coverage = summary["coverage"]
    lines = [
        "# Diagnostic Summary",
        "",
        f"- Case: `{summary['manifest'].get('case', 'unknown')}`",
        f"- Frames: {summary['frame_count']}",
        f"- Solver statuses: `{json.dumps(summary['solver_status_counts'], sort_keys=True)}`",
        f"- Hard residual minimum: {hard['residual_min']}",
        f"- Hard residuals below {hard['negative_tolerance']}: {hard['negative_count']}",
        f"- Coverage: {coverage['fraction']} ({coverage['unique_cells']}/{coverage['total_cells']})",
        f"- Maximum uncertainty: {summary['uncertainty']['max']}",
        f"- Maximum positive uncertainty rate: {summary['uncertainty']['max_positive_one_step_rate']}",
        "",
        "## Metrics intentionally unavailable",
        "",
    ]
    lines.extend(f"- {name}: `{reason}`" for name, reason in summary["unavailable_metrics"].items())
    lines.extend(
        [
            "",
            "## Machine-readable detail",
            "",
            "```json",
            json.dumps(summary, indent=2, sort_keys=True),
            "```",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def analyze_run(data_path: Path, manifest_path: Path) -> dict:
    """Analyze a simulator ``data.json`` and write summaries beside it.

    Missing or malformed files are errors.  Individual unavailable measurements
    remain explicit ``null`` values rather than being silently replaced with
    optimistic zeros.
    """
    data = _load_json(data_path, "data")
    manifest = _load_json(manifest_path, "manifest")
    frames = data.get("state")
    config = data.get("config")
    if not isinstance(frames, list):
        raise ValueError("data JSON must contain a state array")
    if not isinstance(config, dict):
        raise ValueError("data JSON must contain a config object")

    para = data.get("para") if isinstance(data.get("para"), dict) else {}
    grid_world = para.get("gridWorld") if isinstance(para.get("gridWorld"), dict) else {}
    x_num = _finite_number(grid_world.get("xNum"))
    y_num = _finite_number(grid_world.get("yNum"))
    total_cells = int(x_num * y_num) if x_num is not None and y_num is not None and x_num >= 0 and y_num >= 0 else 0

    status_counts: Counter[str] = Counter()
    solve_times: list[float] = []
    finite_failures: list[str] = []
    hard_residuals: list[float] = []
    logged_residual_missing_count = 0
    logged_residual_mismatch_count = 0
    nominal_link_margins: list[float] = []
    tightened_link_margins: list[float] = []
    nominal_collision_margins: list[float] = []
    tightened_collision_margins: list[float] = []
    l2_norms: list[float] = []
    linf_norms: list[float] = []
    per_frame_linf: list[float | None] = []
    infeasible_frame_indices: list[int] = []
    unavailable_frame_indices: list[int] = []
    unapplied_per_frame_linf: list[float | None] = []
    unapplied_infeasible_frame_indices: list[int] = []
    applied_solver_record_count = 0
    unapplied_solver_record_count = 0
    failed_solver_record_count = 0
    unconfirmed_solver_record_count = 0
    unapplied_status_counts: Counter[str] = Counter()
    unapplied_hard_constraint_count = 0
    unapplied_result_placeholder_count = 0
    unapplied_invalid_result_placeholder_count = 0
    unapplied_unexpected_logged_residual_count = 0
    uncertainties: list[float] = []
    uncertainty_rates: list[float] = []
    positive_uncertainty_jumps: list[float] = []
    logged_uncertainty_rates: list[float] = []
    logged_uncertainty_rate_missing_count = 0
    logged_uncertainty_rate_nonfinite_count = 0
    previous_uncertainty: dict[int, tuple[int, float, float]] = {}
    previous_frame_index: int | None = None
    previous_frame_applied_planar_controls: dict[int, tuple[float, float]] = {}
    control_mismatch_records: list[dict] = []
    pairwise_expected_rows: list[dict] = []
    pairwise_observed_rows: list[dict] = []
    pairwise_missing_rows: list[dict] = []
    pairwise_unexpected_rows: list[dict] = []
    pairwise_coverage_per_frame: list[dict] = []
    observed_saturation_counts: Counter[str] = Counter()
    logged_saturation_counts: Counter[str] = Counter()
    input_violation_counts: Counter[str] = Counter()
    input_limit_evidence_records: list[dict] = []
    localization_margin_records: dict[
        str, dict[int, tuple[float, float]]
    ] = {}
    collision_margin_records: dict[
        str, dict[int, tuple[float, float]]
    ] = {}
    updated_cells: set[tuple[int, int]] = set()
    class_k_record_count = 0
    class_k_missing_count = 0
    class_k_comparison_count = 0
    class_k_mismatch_count = 0
    class_k_not_directly_comparable_count = 0
    class_k_unmapped_count = 0
    class_k_records: list[dict] = []
    covariance_consistency_records: list[dict] = []
    covariance_consistency_unavailable: Counter[str] = Counter()

    without_slack = config.get("cbfs", {}).get("without-slack", {})
    comm_config = without_slack.get("comm-fixed", {}) if isinstance(without_slack, dict) else {}
    safety_config = without_slack.get("safety", {}) if isinstance(without_slack, dict) else {}
    max_range = _finite_number(comm_config.get("max-range")) if isinstance(comm_config, dict) else None
    safe_distance = _finite_number(safety_config.get("safe-distance")) if isinstance(safety_config, dict) else None
    safety_mode = (
        safety_config.get("mode")
        if isinstance(safety_config, dict)
        and isinstance(safety_config.get("mode"), str)
        else None
    )
    input_limits_config = (
        config.get("cbfs", {}).get("input-limits", {})
        if isinstance(config.get("cbfs"), dict)
        else {}
    )
    input_limits_enabled = (
        input_limits_config.get("on")
        if isinstance(input_limits_config, dict)
        and isinstance(input_limits_config.get("on"), bool)
        else None
    )
    planar_component_max = (
        _finite_number(input_limits_config.get("planar-component-max"))
        if isinstance(input_limits_config, dict)
        else None
    )
    yaw_rate_max = (
        _finite_number(input_limits_config.get("yaw-rate-max"))
        if isinstance(input_limits_config, dict)
        else None
    )
    input_limits_available = (
        input_limits_enabled is not None
        and planar_component_max is not None
        and planar_component_max >= 0.0
        and yaw_rate_max is not None
        and yaw_rate_max >= 0.0
    )
    bases = config.get("bases", [])
    position_covariance_config = (
        config.get("position_covariance")
        if isinstance(config.get("position_covariance"), dict)
        else {}
    )
    configured_uncertainty_type = position_covariance_config.get("uncertainty-type")
    supported_uncertainty_types = {"max_eigenvalue", "std_avg"}
    covariance_absolute_tolerance = 1e-6
    covariance_relative_tolerance = 1e-6

    for frame_index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            finite_failures.append(f"state[{frame_index}]")
            per_frame_linf.append(None)
            unapplied_per_frame_linf.append(None)
            continue
        runtime = _record_nonfinite(finite_failures, frame.get("runtime"), f"state[{frame_index}].runtime")
        robots = frame.get("robots")
        if not isinstance(robots, list):
            finite_failures.append(f"state[{frame_index}].robots")
            per_frame_linf.append(None)
            unapplied_per_frame_linf.append(None)
            continue

        formation_by_id: dict[int, dict] = {}
        formations = frame.get("formation")
        if isinstance(formations, list):
            for formation in formations:
                if isinstance(formation, dict) and isinstance(formation.get("id"), int):
                    formation_by_id[formation["id"]] = formation

        frame_robot_ids: set[int] = set()
        frame_applied_planar_controls: dict[int, tuple[float, float]] = {}
        frame_applied_neighbor_sources: dict[
            tuple[int, int], dict[str, set[str]]
        ] = {}
        frame_safety_rows: set[tuple[int, str]] = set()
        robot_info: dict[int, tuple[tuple[float, float], float | None]] = {}
        frame_bounds: list[float] = []
        unapplied_frame_bounds: list[float] = []
        for robot_index, robot in enumerate(robots):
            robot_path = f"state[{frame_index}].robots[{robot_index}]"
            if not isinstance(robot, dict) or not isinstance(robot.get("id"), int):
                finite_failures.append(robot_path)
                continue
            robot_id = robot["id"]
            frame_robot_ids.add(robot_id)
            position = _point_from_state(robot, finite_failures, robot_path)
            uncertainty = _record_nonfinite(finite_failures, robot.get("uncertainty"), f"{robot_path}.uncertainty")
            if uncertainty is not None:
                uncertainties.append(uncertainty)
                if runtime is not None and robot_id in previous_uncertainty:
                    (
                        previous_uncertainty_frame,
                        previous_time,
                        previous_value,
                    ) = previous_uncertainty[robot_id]
                    elapsed = runtime - previous_time
                    if (
                        previous_uncertainty_frame == frame_index - 1
                        and elapsed > 0.0
                    ):
                        jump = uncertainty - previous_value
                        if jump > 0.0:
                            positive_uncertainty_jumps.append(jump)
                            uncertainty_rates.append(jump / elapsed)
                if runtime is not None:
                    previous_uncertainty[robot_id] = (
                        frame_index,
                        runtime,
                        uncertainty,
                    )
            if "uncertainty_rate" not in robot:
                logged_uncertainty_rate_missing_count += 1
            else:
                logged_rate = _record_nonfinite(
                    finite_failures,
                    robot.get("uncertainty_rate"),
                    f"{robot_path}.uncertainty_rate",
                )
                if logged_rate is None:
                    logged_uncertainty_rate_nonfinite_count += 1
                else:
                    logged_uncertainty_rates.append(logged_rate)
            if position is not None:
                robot_info[robot_id] = (position, uncertainty)

            covariance = robot.get("position_covariance")
            finite_covariance: dict[str, float] = {}
            if isinstance(covariance, dict):
                for covariance_name in ("cov_xx", "cov_xy", "cov_yx", "cov_yy"):
                    covariance_value = _record_nonfinite(
                        finite_failures,
                        covariance.get(covariance_name),
                        f"{robot_path}.position_covariance.{covariance_name}",
                    )
                    if covariance_value is not None:
                        finite_covariance[covariance_name] = covariance_value
                if len(finite_covariance) != 4:
                    covariance_consistency_unavailable["incomplete_or_nonfinite_covariance"] += 1
                elif not isinstance(configured_uncertainty_type, str):
                    covariance_consistency_unavailable["missing_configured_uncertainty_type"] += 1
                elif configured_uncertainty_type not in supported_uncertainty_types:
                    covariance_consistency_unavailable["unsupported_uncertainty_type"] += 1
                elif uncertainty is None:
                    covariance_consistency_unavailable["missing_or_nonfinite_logged_uncertainty"] += 1
                else:
                    derived_uncertainty, unavailable_reason = _derive_uncertainty_from_covariance(
                        finite_covariance,
                        configured_uncertainty_type,
                    )
                    if derived_uncertainty is None:
                        covariance_consistency_unavailable[
                            unavailable_reason or "covariance_conversion_failed"
                        ] += 1
                    else:
                        absolute_error = abs(derived_uncertainty - uncertainty)
                        matches = math.isclose(
                            derived_uncertainty,
                            uncertainty,
                            rel_tol=covariance_relative_tolerance,
                            abs_tol=covariance_absolute_tolerance,
                        )
                        covariance_consistency_records.append(
                            {
                                "frame_index": frame_index,
                                "runtime": runtime,
                                "robot_id": robot_id,
                                "logged_uncertainty": uncertainty,
                                "derived_uncertainty": derived_uncertainty,
                                "absolute_error": absolute_error,
                                "matches": matches,
                            }
                        )
            elif (
                configured_uncertainty_type in supported_uncertainty_types
                and position_covariance_config.get("enable", False)
            ):
                covariance_consistency_unavailable["missing_position_covariance"] += 1

            opt = robot.get("opt") if isinstance(robot.get("opt"), dict) else {}
            solver_info = opt.get("solver_info") if isinstance(opt.get("solver_info"), dict) else {}
            solver_status = solver_info.get("status", opt.get("status", "missing"))
            status_counts[str(solver_status)] += 1
            solve_time = _finite_number(solver_info.get("solve_time_ms"))
            if solve_time is not None:
                solve_times.append(solve_time)
            confirmed_applied = (
                opt.get("status") == "success"
                and solver_info.get("status") == "optimal"
            )
            if confirmed_applied:
                applied_solver_record_count += 1
            else:
                unapplied_solver_record_count += 1
                unapplied_status_counts[str(solver_status)] += 1
                if (
                    opt.get("status") == "failed"
                    or (
                        "status" in solver_info
                        and solver_info.get("status") != "optimal"
                    )
                ):
                    failed_solver_record_count += 1
                else:
                    unconfirmed_solver_record_count += 1

            result = opt.get("result") if isinstance(opt.get("result"), dict) else {}
            finite_result: dict[str, float] = {}
            observed_input_saturation: dict[str, bool] | None = None
            raw_logged_input_limits = opt.get("input_limits")
            logged_input_limits = (
                raw_logged_input_limits
                if isinstance(raw_logged_input_limits, dict)
                else {}
            )
            logged_saturation = (
                logged_input_limits.get("saturated")
                if isinstance(logged_input_limits.get("saturated"), dict)
                else {}
            )
            if confirmed_applied:
                controls: list[float] = []
                for name, value in result.items():
                    control = _record_nonfinite(
                        finite_failures,
                        value,
                        f"{robot_path}.opt.result.{name}",
                    )
                    if control is not None:
                        controls.append(control)
                        finite_result[name] = control
                if result and len(controls) == len(result):
                    control_array = np.asarray(controls, dtype=float)
                    l2_norms.append(float(np.linalg.norm(control_array, ord=2)))
                    linf_norms.append(float(np.linalg.norm(control_array, ord=np.inf)))
                    vx = finite_result.get("vx")
                    vy = finite_result.get("vy")
                    if vx is not None and vy is not None:
                        frame_applied_planar_controls[robot_id] = (vx, vy)

                    if input_limits_available and input_limits_enabled:
                        yaw_rate = finite_result.get("yawRateRad")
                        if (
                            vx is not None
                            and vy is not None
                            and yaw_rate is not None
                        ):
                            vx_saturated = (
                                abs(vx)
                                >= planar_component_max - INPUT_LIMIT_TOLERANCE
                            )
                            vy_saturated = (
                                abs(vy)
                                >= planar_component_max - INPUT_LIMIT_TOLERANCE
                            )
                            yaw_saturated = (
                                abs(yaw_rate)
                                >= yaw_rate_max - INPUT_LIMIT_TOLERANCE
                            )
                            observed_input_saturation = {
                                "vx": vx_saturated,
                                "vy": vy_saturated,
                                "yawRateRad": yaw_saturated,
                                "any": (
                                    vx_saturated
                                    or vy_saturated
                                    or yaw_saturated
                                ),
                            }
                            observed_saturation_counts["vx"] += int(vx_saturated)
                            observed_saturation_counts["vy"] += int(vy_saturated)
                            observed_saturation_counts["yawRateRad"] += int(
                                yaw_saturated
                            )
                            observed_saturation_counts["planar_any"] += int(
                                vx_saturated or vy_saturated
                            )
                            observed_saturation_counts["any"] += int(
                                vx_saturated or vy_saturated or yaw_saturated
                            )
                            input_violation_counts["vx"] += int(
                                abs(vx)
                                > planar_component_max + INPUT_LIMIT_TOLERANCE
                            )
                            input_violation_counts["vy"] += int(
                                abs(vy)
                                > planar_component_max + INPUT_LIMIT_TOLERANCE
                            )
                            input_violation_counts["yawRateRad"] += int(
                                abs(yaw_rate)
                                > yaw_rate_max + INPUT_LIMIT_TOLERANCE
                            )

                    for saturation_name in ("vx", "vy", "yawRateRad", "any"):
                        if logged_saturation.get(saturation_name) is True:
                            logged_saturation_counts[saturation_name] += 1
                if input_limits_available and input_limits_enabled:
                    evidence_failures: list[str] = []
                    if not isinstance(raw_logged_input_limits, dict):
                        evidence_failures.append("missing_opt_input_limits")
                    if logged_input_limits.get("enabled") is not True:
                        evidence_failures.append("enabled_not_true")
                    bound_row_count = logged_input_limits.get(
                        "bound_row_count"
                    )
                    if (
                        isinstance(bound_row_count, bool)
                        or bound_row_count != 6
                    ):
                        evidence_failures.append("bound_row_count_not_six")
                    logged_planar_max = _finite_number(
                        logged_input_limits.get("planar_component_max")
                    )
                    if (
                        logged_planar_max is None
                        or not math.isclose(
                            logged_planar_max,
                            planar_component_max,
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        )
                    ):
                        evidence_failures.append(
                            "planar_component_max_mismatch"
                        )
                    logged_yaw_max = _finite_number(
                        logged_input_limits.get("yaw_rate_max")
                    )
                    if (
                        logged_yaw_max is None
                        or not math.isclose(
                            logged_yaw_max,
                            yaw_rate_max,
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        )
                    ):
                        evidence_failures.append("yaw_rate_max_mismatch")
                    logged_tolerance = _finite_number(
                        logged_input_limits.get("saturation_tolerance")
                    )
                    if (
                        logged_tolerance is None
                        or not math.isclose(
                            logged_tolerance,
                            INPUT_LIMIT_TOLERANCE,
                            rel_tol=0.0,
                            abs_tol=1e-15,
                        )
                    ):
                        evidence_failures.append(
                            "saturation_tolerance_mismatch"
                        )
                    saturation_names = (
                        "vx",
                        "vy",
                        "yawRateRad",
                        "any",
                    )
                    if not all(
                        isinstance(logged_saturation.get(name), bool)
                        for name in saturation_names
                    ):
                        evidence_failures.append(
                            "missing_logged_saturation_flags"
                        )
                    if observed_input_saturation is None:
                        evidence_failures.append(
                            "missing_finite_applied_control"
                        )
                    elif all(
                        isinstance(logged_saturation.get(name), bool)
                        for name in saturation_names
                    ) and any(
                        logged_saturation[name]
                        != observed_input_saturation[name]
                        for name in saturation_names
                    ):
                        evidence_failures.append(
                            "logged_saturation_mismatch"
                        )
                    input_limit_evidence_records.append(
                        {
                            "frame_index": frame_index,
                            "robot_id": robot_id,
                            "valid": not evidence_failures,
                            "failures": evidence_failures,
                        }
                    )
            elif result:
                unapplied_result_placeholder_count += 1
                if any(_finite_number(value) is None for value in result.values()):
                    unapplied_invalid_result_placeholder_count += 1

            raw_constraints = opt.get("cbfNoSlack")
            constraints = raw_constraints if isinstance(raw_constraints, list) else []
            feasibility_constraints: list[dict] = []
            unapplied_feasibility_constraints: list[dict] = []
            control_names = sorted(result) if result else []
            for constraint_index, constraint in enumerate(constraints):
                constraint_path = f"{robot_path}.opt.cbfNoSlack[{constraint_index}]"
                if not isinstance(constraint, dict):
                    finite_failures.append(constraint_path)
                    continue
                constraint_name = constraint.get("name")
                neighbor_source = _neighbor_constraint_source(
                    constraint_name
                )
                if neighbor_source is not None:
                    neighbor_id, source = neighbor_source
                    if source == "safety":
                        frame_safety_rows.add((robot_id, constraint_name))
                    if confirmed_applied:
                        source_record = (
                            frame_applied_neighbor_sources.setdefault(
                                (robot_id, neighbor_id),
                                {"sources": set(), "source_rows": set()},
                            )
                        )
                        source_record["sources"].add(source)
                        source_record["source_rows"].add(constraint_name)
                coe = constraint.get("coe")
                constant = _finite_number(constraint.get("const"))
                if not isinstance(coe, dict) or constant is None:
                    finite_failures.append(constraint_path)
                    continue
                if not control_names:
                    control_names = sorted(coe)
                if confirmed_applied:
                    residual = constant
                    valid_residual = True
                    for name, coefficient_value in coe.items():
                        coefficient = _finite_number(coefficient_value)
                        control = _finite_number(result.get(name))
                        if coefficient is None or control is None:
                            valid_residual = False
                            finite_failures.append(f"{constraint_path}.coe_or_control.{name}")
                            break
                        residual += coefficient * control
                    if valid_residual:
                        hard_residuals.append(residual)
                        logged_residual = _finite_number(constraint.get("residual"))
                        if logged_residual is None:
                            logged_residual_missing_count += 1
                        elif not math.isclose(
                            logged_residual,
                            residual,
                            rel_tol=1e-9,
                            abs_tol=1e-9,
                        ):
                            logged_residual_mismatch_count += 1
                    feasibility_constraints.append({"coe": coe, "rhs": -constant})
                else:
                    unapplied_hard_constraint_count += 1
                    if _finite_number(constraint.get("residual")) is not None:
                        unapplied_unexpected_logged_residual_count += 1
                    unapplied_feasibility_constraints.append(
                        {"coe": coe, "rhs": -constant}
                    )

                class_k_record_count += 1
                alpha_coe = _finite_number(constraint.get("alpha_coe"))
                alpha_pow = constraint.get("alpha_pow")
                class_k_record: dict[str, Any] = {"name": constraint_name}
                if alpha_coe is None or not isinstance(alpha_pow, int):
                    class_k_missing_count += 1
                    class_k_record.update(
                        {
                            "source": "unavailable_missing_instantiated_parameters",
                            "instantiated": None,
                            "comparison": "unavailable_missing_instantiated_parameters",
                        }
                    )
                else:
                    class_k_record["instantiated"] = {
                        "coefficient": alpha_coe,
                        "power": alpha_pow,
                    }
                    aggregate_names = _aggregate_constraint_names(constraint_name)
                    if aggregate_names is not None:
                        class_k_not_directly_comparable_count += 1
                        contained_parameters: list[dict] = []
                        for contained_name in aggregate_names:
                            configured = _configured_class_k(config, contained_name)
                            if configured is None:
                                class_k_unmapped_count += 1
                                contained_parameters.append(
                                    {
                                        "name": contained_name,
                                        "config_source": None,
                                        "configured": None,
                                        "availability": "unmapped_constraint_name",
                                    }
                                )
                            else:
                                contained_parameters.append(
                                    {
                                        **configured,
                                        "availability": (
                                            "available"
                                            if configured["configured"] is not None
                                            else "missing_configured_alpha"
                                        ),
                                    }
                                )
                        class_k_record.update(
                            {
                                "source": "MultiCBF.identity_alpha",
                                "comparison": (
                                    "intentional_semantic_difference_not_directly_comparable"
                                    if math.isclose(
                                        alpha_coe,
                                        1.0,
                                        rel_tol=1e-12,
                                        abs_tol=1e-12,
                                    )
                                    and alpha_pow == 1
                                    else "logged_identity_parameters_mismatch"
                                ),
                                "contained_configured_parameters": contained_parameters,
                            }
                        )
                        if class_k_record["comparison"] == "logged_identity_parameters_mismatch":
                            class_k_mismatch_count += 1
                    else:
                        class_k_record["source"] = "CBF.instantiated_alpha"
                        configured = _configured_class_k(config, constraint_name)
                        if configured is None:
                            class_k_unmapped_count += 1
                            class_k_record.update(
                                {
                                    "comparison": "unmapped_constraint_name",
                                    "config_source": None,
                                    "configured": None,
                                }
                            )
                        elif configured["configured"] is None:
                            class_k_record.update(
                                {
                                    **configured,
                                    "comparison": "unavailable_missing_configured_alpha",
                                }
                            )
                        else:
                            class_k_comparison_count += 1
                            matches = (
                                math.isclose(
                                    alpha_coe,
                                    configured["configured"]["coefficient"],
                                    rel_tol=1e-12,
                                    abs_tol=1e-12,
                                )
                                and alpha_pow == configured["configured"]["power"]
                            )
                            if not matches:
                                class_k_mismatch_count += 1
                            class_k_record.update(
                                {
                                    **configured,
                                    "comparison": "match" if matches else "mismatch",
                                }
                            )
                class_k_records.append(class_k_record)

            if confirmed_applied:
                frame_bounds.append(
                    minimum_linf_bound(feasibility_constraints, control_names)
                )
            else:
                unapplied_frame_bounds.append(
                    minimum_linf_bound(
                        unapplied_feasibility_constraints,
                        control_names,
                    )
                )

        if previous_frame_index == frame_index - 1:
            for (
                owner_id,
                neighbor_id,
            ), source_record in sorted(
                frame_applied_neighbor_sources.items()
            ):
                owner_info = robot_info.get(owner_id)
                if owner_info is None:
                    continue
                owner_position = owner_info[0]
                neighbor_info = robot_info.get(neighbor_id)
                current_control = frame_applied_planar_controls.get(
                    neighbor_id
                )
                previous_control = (
                    previous_frame_applied_planar_controls.get(neighbor_id)
                )
                if (
                    neighbor_info is None
                    or current_control is None
                    or previous_control is None
                ):
                    continue
                neighbor_position = neighbor_info[0]
                dx = owner_position[0] - neighbor_position[0]
                dy = owner_position[1] - neighbor_position[1]
                distance = math.hypot(dx, dy)
                if distance <= 0.0:
                    continue
                delta_vx = current_control[0] - previous_control[0]
                delta_vy = current_control[1] - previous_control[1]
                projected = abs(
                    dx / distance * delta_vx
                    + dy / distance * delta_vy
                )
                control_mismatch_records.append(
                    {
                        "frame_index": frame_index,
                        "owner_id": owner_id,
                        "neighbor_id": neighbor_id,
                        "sources": sorted(source_record["sources"]),
                        "source_rows": sorted(
                            source_record["source_rows"]
                        ),
                        "projected_mismatch": projected,
                    }
                )
        previous_frame_index = frame_index
        previous_frame_applied_planar_controls = (
            frame_applied_planar_controls
        )

        if safety_mode == "pairwise":
            expected_rows = {
                (owner_id, f"safetyCBF(#{neighbor_id})")
                for owner_id in frame_robot_ids
                for neighbor_id in frame_robot_ids
                if owner_id != neighbor_id
            }
            observed_rows = set(frame_safety_rows)
            missing_rows = expected_rows - observed_rows
            unexpected_rows = observed_rows - expected_rows

            def coverage_records(rows):
                return [
                    {
                        "frame_index": frame_index,
                        "owner_id": owner_id,
                        "name": name,
                    }
                    for owner_id, name in sorted(rows)
                ]

            expected_records = coverage_records(expected_rows)
            observed_records = coverage_records(observed_rows)
            missing_records = coverage_records(missing_rows)
            unexpected_records = coverage_records(unexpected_rows)
            pairwise_expected_rows.extend(expected_records)
            pairwise_observed_rows.extend(observed_records)
            pairwise_missing_rows.extend(missing_records)
            pairwise_unexpected_rows.extend(unexpected_records)
            pairwise_coverage_per_frame.append(
                {
                    "frame_index": frame_index,
                    "expected_count": len(expected_records),
                    "observed_count": len(observed_records),
                    "missing_count": len(missing_records),
                    "unexpected_count": len(unexpected_records),
                    "complete": not missing_records and not unexpected_records,
                }
            )

        # A genuinely empty simulator frame needs no bound.  A nonempty frame
        # with no confirmed applied solve has no usable bound evidence.
        frame_bound = max(frame_bounds) if frame_bounds else None
        if frame_bound is None and robots:
            per_frame_linf.append(None)
            unavailable_frame_indices.append(frame_index)
        elif frame_bound is None:
            per_frame_linf.append(0.0)
        elif math.isinf(frame_bound):
            # JSON has no portable infinity literal.  Preserve the diagnosis
            # explicitly instead of emitting non-standard NaN/Infinity JSON.
            per_frame_linf.append(None)
            infeasible_frame_indices.append(frame_index)
        else:
            per_frame_linf.append(float(frame_bound))

        unapplied_frame_bound = (
            max(unapplied_frame_bounds) if unapplied_frame_bounds else 0.0
        )
        if math.isinf(unapplied_frame_bound):
            unapplied_per_frame_linf.append(None)
            unapplied_infeasible_frame_indices.append(frame_index)
        else:
            unapplied_per_frame_linf.append(float(unapplied_frame_bound))

        if max_range is not None:
            for robot_id, (position, uncertainty) in robot_info.items():
                formation = formation_by_id.get(robot_id, {})
                base_ids = formation.get("baseIds", []) if isinstance(formation, dict) else []
                anchor_ids = formation.get("anchorIds", []) if isinstance(formation, dict) else []
                references: list[
                    tuple[tuple[float, float], float, str]
                ] = []
                if isinstance(base_ids, list):
                    for base_id in base_ids:
                        point = _base_point(bases, base_id)
                        if point is not None:
                            references.append(
                                (
                                    point,
                                    0.0,
                                    f"base:{robot_id}-{base_id}",
                                )
                            )
                if isinstance(anchor_ids, list):
                    for anchor_id in anchor_ids:
                        if anchor_id in robot_info:
                            anchor_point, anchor_uncertainty = robot_info[anchor_id]
                            references.append(
                                (
                                    anchor_point,
                                    (
                                        anchor_uncertainty
                                        if anchor_uncertainty is not None
                                        else math.nan
                                    ),
                                    "robots:"
                                    + "-".join(
                                        map(str, sorted((robot_id, anchor_id)))
                                    ),
                                )
                            )
                for reference_point, reference_uncertainty, reference_key in references:
                    distance = math.dist(position, reference_point)
                    nominal_margin = max_range - distance
                    nominal_link_margins.append(nominal_margin)
                    if uncertainty is not None and math.isfinite(reference_uncertainty):
                        tightened_margin = (
                            max_range
                            - distance
                            - uncertainty
                            - reference_uncertainty
                        )
                        tightened_link_margins.append(tightened_margin)
                        _record_physical_margin(
                            localization_margin_records,
                            reference_key,
                            frame_index,
                            nominal_margin,
                            tightened_margin,
                        )

        if safe_distance is not None:
            for first_id, second_id in itertools.combinations(sorted(robot_info), 2):
                first_position, first_uncertainty = robot_info[first_id]
                second_position, second_uncertainty = robot_info[second_id]
                distance = math.dist(first_position, second_position)
                nominal_margin = distance - safe_distance
                nominal_collision_margins.append(nominal_margin)
                if first_uncertainty is not None and second_uncertainty is not None:
                    tightened_margin = (
                        distance
                        - safe_distance
                        - first_uncertainty
                        - second_uncertainty
                    )
                    tightened_collision_margins.append(tightened_margin)
                    _record_physical_margin(
                        collision_margin_records,
                        f"robots:{first_id}-{second_id}",
                        frame_index,
                        nominal_margin,
                        tightened_margin,
                    )

        updates = frame.get("update")
        if isinstance(updates, list):
            for cell in updates:
                if isinstance(cell, list) and len(cell) >= 2 and isinstance(cell[0], int) and isinstance(cell[1], int):
                    updated_cells.add((cell[0], cell[1]))

    if logged_uncertainty_rate_nonfinite_count:
        logged_uncertainty_rate_status = "available_with_nonfinite_values"
    elif logged_uncertainty_rates and logged_uncertainty_rate_missing_count:
        logged_uncertainty_rate_status = "available_with_missing_values"
    elif logged_uncertainty_rates:
        logged_uncertainty_rate_status = "available"
    else:
        logged_uncertainty_rate_status = "unavailable_missing_values"

    applied_linf_values = [
        value for value in per_frame_linf if value is not None
    ]
    minimum_required_linf_candidate = (
        float(max(applied_linf_values))
        if (
            applied_linf_values
            and not infeasible_frame_indices
            and not unavailable_frame_indices
        )
        else None
    )
    applicable_input_limit_record_count = len(
        input_limit_evidence_records
    )
    validated_input_limit_record_count = sum(
        record["valid"] for record in input_limit_evidence_records
    )
    invalid_input_limit_record_count = (
        applicable_input_limit_record_count
        - validated_input_limit_record_count
    )
    if not input_limits_available:
        input_limits_status = "unavailable"
        input_limits_unavailable_reason = (
            "missing_or_invalid_materialized_input_limit_config"
        )
    elif not input_limits_enabled:
        input_limits_status = "disabled"
        input_limits_unavailable_reason = "input_limits_disabled"
    elif not applicable_input_limit_record_count:
        input_limits_status = "unavailable"
        input_limits_unavailable_reason = (
            "no_applicable_optimal_input_limit_records"
        )
    elif invalid_input_limit_record_count:
        input_limits_status = "invalid"
        input_limits_unavailable_reason = (
            "invalid_or_inconsistent_opt_input_limit_records"
        )
    else:
        input_limits_status = "enabled"
        input_limits_unavailable_reason = None
    minimum_required_linf = (
        minimum_required_linf_candidate
        if input_limits_status == "enabled"
        else None
    )
    bounded_feasibility_headroom = (
        planar_component_max - minimum_required_linf
        if input_limits_status == "enabled"
        and minimum_required_linf is not None
        else None
    )
    observed_saturation_summary = {
        name: observed_saturation_counts[name]
        for name in ("vx", "vy", "yawRateRad", "planar_any", "any")
    }
    logged_saturation_summary = {
        name: logged_saturation_counts[name]
        for name in ("vx", "vy", "yawRateRad", "any")
    }
    violation_summary = {
        name: input_violation_counts[name]
        for name in ("vx", "vy", "yawRateRad")
    }
    violation_summary["total"] = sum(violation_summary.values())
    if input_limits_status != "enabled":
        observed_saturation_summary = None
        logged_saturation_summary = None
        violation_summary = None

    if safety_mode == "pairwise":
        pairwise_row_coverage = {
            "status": "available",
            "mode": "pairwise",
            "expected_count": len(pairwise_expected_rows),
            "observed_count": len(pairwise_observed_rows),
            "missing_count": len(pairwise_missing_rows),
            "unexpected_count": len(pairwise_unexpected_rows),
            "expected_rows": pairwise_expected_rows,
            "observed_rows": pairwise_observed_rows,
            "missing_rows": pairwise_missing_rows,
            "unexpected_rows": pairwise_unexpected_rows,
            "complete": (
                not pairwise_missing_rows
                and not pairwise_unexpected_rows
            ),
            "per_frame": pairwise_coverage_per_frame,
        }
    else:
        pairwise_row_coverage = {
            "status": "not_applicable",
            "mode": safety_mode,
            "expected_count": None,
            "observed_count": None,
            "missing_count": None,
            "unexpected_count": None,
            "expected_rows": [],
            "observed_rows": [],
            "missing_rows": [],
            "unexpected_rows": [],
            "complete": None,
            "per_frame": [],
        }

    unavailable_metrics = {
        "RMSE": UNAVAILABLE_METRIC,
        "NEES": UNAVAILABLE_METRIC,
        "ANEES": UNAVAILABLE_METRIC,
        "tracking_error": UNAVAILABLE_METRIC,
    }
    if input_limits_status != "enabled":
        unavailable_metrics["saturation"] = (
            UNAVAILABLE_METRIC
            if not input_limits_available
            else f"unavailable_{input_limits_unavailable_reason}"
        )
    if covariance_consistency_records:
        covariance_consistency_status = "available"
    elif not isinstance(configured_uncertainty_type, str):
        covariance_consistency_status = (
            "unavailable_missing_position_covariance_uncertainty_type"
        )
    elif configured_uncertainty_type not in supported_uncertainty_types:
        covariance_consistency_status = "unavailable_unsupported_uncertainty_type"
    else:
        covariance_consistency_status = (
            "unavailable_no_complete_covariance_uncertainty_pairs"
        )
    covariance_errors = [
        record["absolute_error"] for record in covariance_consistency_records
    ]
    summary = {
        "manifest": manifest,
        "frame_count": len(frames),
        "solver_status_counts": dict(sorted(status_counts.items())),
        "applied_solver_records": {
            "count": applied_solver_record_count,
            "confirmation": "opt.status=success_and_solver_info.status=optimal",
        },
        "unapplied_solver_records": {
            "count": unapplied_solver_record_count,
            "failed_solver_record_count": failed_solver_record_count,
            "unconfirmed_record_count": unconfirmed_solver_record_count,
            "status_counts": dict(sorted(unapplied_status_counts.items())),
            "hard_constraint_count": unapplied_hard_constraint_count,
            "result_placeholder_count": unapplied_result_placeholder_count,
            "invalid_result_placeholder_count": unapplied_invalid_result_placeholder_count,
            "unexpected_logged_residual_count": unapplied_unexpected_logged_residual_count,
            "minimum_linf_bound": {
                "per_frame": unapplied_per_frame_linf,
                "maximum": float(
                    max(
                        value
                        for value in unapplied_per_frame_linf
                        if value is not None
                    )
                )
                if any(value is not None for value in unapplied_per_frame_linf)
                else None,
                "infeasible_frame_count": len(
                    unapplied_infeasible_frame_indices
                ),
                "infeasible_frame_indices": unapplied_infeasible_frame_indices,
            },
            "interpretation": (
                "unconfirmed records are excluded from applied evidence; "
                "solver failures are caught and logged without a state step, "
                "so their result placeholders were not applied"
            ),
        },
        "solve_time_ms": _summary_statistics(solve_times),
        "finite_value_failures": {"count": len(finite_failures), "paths": finite_failures},
        "hard_constraints": {
            "count": len(hard_residuals),
            "residual_min": _metric_minimum(hard_residuals),
            "negative_tolerance": RESIDUAL_TOLERANCE,
            "negative_count": sum(value < RESIDUAL_TOLERANCE for value in hard_residuals),
            "logged_residual_missing_count": logged_residual_missing_count,
            "logged_residual_mismatch_count": logged_residual_mismatch_count,
            "residual_source": "const_plus_coe_dot_result",
            "scope": "confirmed_applied_optimal_controls_only",
        },
        "localization_margins": {
            "required_reference": {
                "nominal_count": len(nominal_link_margins),
                "nominal_min": _metric_minimum(nominal_link_margins),
                "tightened_count": len(tightened_link_margins),
                "tightened_min": _metric_minimum(tightened_link_margins),
                "uncertainty_source": (
                    "logged_scalar_cross_checked_against_position_covariance_when_available"
                ),
            }
        },
        "collision_margins": {
            "all_pairs": {
                "nominal_count": len(nominal_collision_margins),
                "nominal_min": _metric_minimum(nominal_collision_margins),
                "tightened_count": len(tightened_collision_margins),
                "tightened_min": _metric_minimum(tightened_collision_margins),
            }
        },
        "practical_tolerance": {
            "negative_tolerance_m": PRACTICAL_NEGATIVE_TOLERANCE_M,
            "localization": _practical_tolerance_summary(
                localization_margin_records
            ),
            "collision": _practical_tolerance_summary(
                collision_margin_records
            ),
        },
        "control_norms": {
            "scope": "confirmed_applied_optimal_controls_only",
            "l2": _summary_statistics(l2_norms),
            "linf": _summary_statistics(linf_norms),
        },
        "input_limits": {
            "status": input_limits_status,
            "enabled": input_limits_enabled,
            "planar_component_max": planar_component_max,
            "yaw_rate_max": yaw_rate_max,
            "violation_tolerance": INPUT_LIMIT_TOLERANCE,
            "observed_saturation_counts": observed_saturation_summary,
            "logged_saturation_counts": logged_saturation_summary,
            "violation_counts": violation_summary,
            "minimum_required_linf": minimum_required_linf,
            "bounded_feasibility_headroom": bounded_feasibility_headroom,
            "applicable_optimal_record_count": (
                applicable_input_limit_record_count
            ),
            "validated_record_count": validated_input_limit_record_count,
            "invalid_record_count": invalid_input_limit_record_count,
            "evidence_records": input_limit_evidence_records,
            "unavailable_reason": input_limits_unavailable_reason,
            "headroom_definition": (
                "planar_component_max_minus_minimum_required_linf"
            ),
            "scope": "confirmed_applied_optimal_controls_only",
        },
        "minimum_linf_bound": {
            "per_frame": per_frame_linf,
            "maximum": (
                float(max(value for value in per_frame_linf if value is not None))
                if any(value is not None for value in per_frame_linf)
                else None
            ),
            "infeasible_frame_count": len(infeasible_frame_indices),
            "infeasible_frame_indices": infeasible_frame_indices,
            "unavailable_frame_count": len(unavailable_frame_indices),
            "unavailable_frame_indices": unavailable_frame_indices,
            "scope": "confirmed_applied_optimal_constraint_sets_only",
        },
        "uncertainty": {
            "count": len(uncertainties),
            "max": _metric_maximum(uncertainties),
            "max_positive_one_step_rate": _metric_maximum(uncertainty_rates),
            "logged_rate": {
                "status": logged_uncertainty_rate_status,
                "count": len(logged_uncertainty_rates),
                "max": _metric_maximum(logged_uncertainty_rates),
                "missing_count": logged_uncertainty_rate_missing_count,
                "nonfinite_count": logged_uncertainty_rate_nonfinite_count,
                "source": "robot.uncertainty_rate",
            },
        },
        "uncertainty_jumps": {
            "positive_count": len(positive_uncertainty_jumps),
            "maximum": _metric_maximum(positive_uncertainty_jumps),
            "source": "consecutive_logged_scalar_uncertainty",
        },
        "control_mismatch": {
            "status": (
                "available"
                if control_mismatch_records
                else "unavailable_no_consecutive_applied_neighbor_controls"
            ),
            "count": len(control_mismatch_records),
            "maximum": _metric_maximum(
                [
                    record["projected_mismatch"]
                    for record in control_mismatch_records
                ]
            ),
            "records": control_mismatch_records,
            "definition": "abs(n_ij_dot_(u_j_current_minus_u_j_previous))",
        },
        "pairwise_row_coverage": pairwise_row_coverage,
        "coverage": {
            "unique_cells": len(updated_cells),
            "total_cells": total_cells,
            "fraction": len(updated_cells) / total_cells if total_cells else None,
        },
        "class_k_parameters": {
            "record_count": class_k_record_count,
            "missing_instantiated_count": class_k_missing_count,
            "configured_comparison_count": class_k_comparison_count,
            "configured_mismatch_count": class_k_mismatch_count,
            "not_directly_comparable_count": class_k_not_directly_comparable_count,
            "unmapped_count": class_k_unmapped_count,
            "records": class_k_records,
        },
        "covariance_uncertainty_consistency": {
            "status": covariance_consistency_status,
            "configured_type": configured_uncertainty_type,
            "checked_count": len(covariance_consistency_records),
            "match_count": sum(
                record["matches"] for record in covariance_consistency_records
            ),
            "mismatch_count": sum(
                not record["matches"] for record in covariance_consistency_records
            ),
            "max_absolute_error": _metric_maximum(covariance_errors),
            "absolute_tolerance": covariance_absolute_tolerance,
            "relative_tolerance": covariance_relative_tolerance,
            "unavailable_count": sum(
                covariance_consistency_unavailable.values()
            ),
            "unavailable_reasons": dict(
                sorted(covariance_consistency_unavailable.items())
            ),
            "records": covariance_consistency_records,
        },
        "unavailable_metrics": unavailable_metrics,
    }

    output_json = data_path.parent / "diagnostic-summary.json"
    output_markdown = data_path.parent / "diagnostic-summary.md"
    output_json.write_text(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n")
    _write_markdown(summary, output_markdown)
    return summary


def main(argv: list[str] | None = None) -> int:
    """Run the analyzer CLI, returning nonzero for missing or invalid JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_path", type=Path)
    parser.add_argument("manifest_path", type=Path)
    arguments = parser.parse_args(argv)
    try:
        summary = analyze_run(arguments.data_path, arguments.manifest_path)
    except (TypeError, ValueError, OSError) as error:
        print(f"diagnostic analysis failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
