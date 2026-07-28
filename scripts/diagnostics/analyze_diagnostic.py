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
    previous_uncertainty: dict[int, tuple[float, float]] = {}
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

        robot_info: dict[int, tuple[tuple[float, float], float | None]] = {}
        frame_bounds: list[float] = []
        unapplied_frame_bounds: list[float] = []
        for robot_index, robot in enumerate(robots):
            robot_path = f"state[{frame_index}].robots[{robot_index}]"
            if not isinstance(robot, dict) or not isinstance(robot.get("id"), int):
                finite_failures.append(robot_path)
                continue
            robot_id = robot["id"]
            position = _point_from_state(robot, finite_failures, robot_path)
            uncertainty = _record_nonfinite(finite_failures, robot.get("uncertainty"), f"{robot_path}.uncertainty")
            if uncertainty is not None:
                uncertainties.append(uncertainty)
                if runtime is not None and robot_id in previous_uncertainty:
                    previous_time, previous_value = previous_uncertainty[robot_id]
                    elapsed = runtime - previous_time
                    if elapsed > 0.0:
                        rate = (uncertainty - previous_value) / elapsed
                        if rate > 0.0:
                            uncertainty_rates.append(rate)
                if runtime is not None:
                    previous_uncertainty[robot_id] = (runtime, uncertainty)
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
                if result and len(controls) == len(result):
                    control_array = np.asarray(controls, dtype=float)
                    l2_norms.append(float(np.linalg.norm(control_array, ord=2)))
                    linf_norms.append(float(np.linalg.norm(control_array, ord=np.inf)))
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
                constraint_name = constraint.get("name")
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

        # The frame requirement is the largest per-robot bound; zero makes an
        # empty frame and a no-hard-constraint frame unambiguous.
        frame_bound = max(frame_bounds) if frame_bounds else 0.0
        if math.isinf(frame_bound):
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
                references: list[tuple[tuple[float, float], float]] = []
                if isinstance(base_ids, list):
                    for base_id in base_ids:
                        point = _base_point(bases, base_id)
                        if point is not None:
                            references.append((point, 0.0))
                if isinstance(anchor_ids, list):
                    for anchor_id in anchor_ids:
                        if anchor_id in robot_info:
                            anchor_point, anchor_uncertainty = robot_info[anchor_id]
                            references.append((anchor_point, anchor_uncertainty if anchor_uncertainty is not None else math.nan))
                for reference_point, reference_uncertainty in references:
                    distance = math.dist(position, reference_point)
                    nominal_link_margins.append(max_range - distance)
                    if uncertainty is not None and math.isfinite(reference_uncertainty):
                        tightened_link_margins.append(max_range - distance - uncertainty - reference_uncertainty)

        if safe_distance is not None:
            for first_id, second_id in itertools.combinations(sorted(robot_info), 2):
                first_position, first_uncertainty = robot_info[first_id]
                second_position, second_uncertainty = robot_info[second_id]
                distance = math.dist(first_position, second_position)
                nominal_collision_margins.append(distance - safe_distance)
                if first_uncertainty is not None and second_uncertainty is not None:
                    tightened_collision_margins.append(distance - safe_distance - first_uncertainty - second_uncertainty)

        updates = frame.get("update")
        if isinstance(updates, list):
            for cell in updates:
                if isinstance(cell, list) and len(cell) >= 2 and isinstance(cell[0], int) and isinstance(cell[1], int):
                    updated_cells.add((cell[0], cell[1]))

    unavailable_metrics = {
        "RMSE": UNAVAILABLE_METRIC,
        "NEES": UNAVAILABLE_METRIC,
        "ANEES": UNAVAILABLE_METRIC,
        "tracking_error": UNAVAILABLE_METRIC,
        "saturation": UNAVAILABLE_METRIC,
    }
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
        "control_norms": {
            "scope": "confirmed_applied_optimal_controls_only",
            "l2": _summary_statistics(l2_norms),
            "linf": _summary_statistics(linf_norms),
        },
        "minimum_linf_bound": {
            "per_frame": per_frame_linf,
            "maximum": float(max(value for value in per_frame_linf if value is not None)) if any(
                value is not None for value in per_frame_linf
            ) else None,
            "infeasible_frame_count": len(infeasible_frame_indices),
            "infeasible_frame_indices": infeasible_frame_indices,
            "scope": "confirmed_applied_optimal_constraint_sets_only",
        },
        "uncertainty": {
            "count": len(uncertainties),
            "max": _metric_maximum(uncertainties),
            "max_positive_one_step_rate": _metric_maximum(uncertainty_rates),
        },
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
