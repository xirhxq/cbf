#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
from typing import Any


def _finite_min(values: list[float], default: float = math.nan) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return min(finite) if finite else default


def _finite_max(values: list[float], default: float = math.nan) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return max(finite) if finite else default


def _float_or_nan(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def _slack_values(opt: dict[str, Any]) -> list[float]:
    return [abs(float(value)) for value in opt.get("slacks", [])]


def _hocbf_acceleration_constraint(item: dict[str, Any]) -> tuple[float, float, float] | None:
    if "coe" not in item or "const" not in item:
        return None
    coe = item.get("coe", {})
    try:
        ax = float(coe.get("ax", 0.0))
        ay = float(coe.get("ay", 0.0))
        rhs = -float(item.get("const", math.nan))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(ax) and math.isfinite(ay) and math.isfinite(rhs)):
        return None
    return ax, ay, rhs


def _hocbf_control_authority_margin(item: dict[str, Any], acceleration_bound: float) -> float:
    constraint = _hocbf_acceleration_constraint(item)
    if constraint is None or not math.isfinite(acceleration_bound):
        return math.nan
    ax, ay, rhs = constraint
    return acceleration_bound * (abs(ax) + abs(ay)) - rhs


def _point_satisfies_halfspaces(
    point: tuple[float, float],
    constraints: list[tuple[float, float, float]],
    tolerance: float = 1.0e-8,
) -> bool:
    x, y = point
    return all(ax * x + ay * y >= rhs - tolerance for ax, ay, rhs in constraints)


def _hocbf_joint_acceleration_feasible(
    items: list[dict[str, Any]],
    acceleration_bound: float,
) -> bool | None:
    if not math.isfinite(acceleration_bound) or acceleration_bound < 0.0:
        return None
    constraints = [
        constraint
        for item in items
        if (constraint := _hocbf_acceleration_constraint(item)) is not None
    ]
    if not constraints:
        return None

    bound = acceleration_bound
    candidates: list[tuple[float, float]] = [
        (-bound, -bound),
        (-bound, bound),
        (bound, -bound),
        (bound, bound),
    ]

    for ax, ay, rhs in constraints:
        if abs(ay) > 1.0e-12:
            candidates.append((-bound, (rhs + ax * bound) / ay))
            candidates.append((bound, (rhs - ax * bound) / ay))
        if abs(ax) > 1.0e-12:
            candidates.append(((rhs + ay * bound) / ax, -bound))
            candidates.append(((rhs - ay * bound) / ax, bound))

    for i, (ax1, ay1, rhs1) in enumerate(constraints):
        for ax2, ay2, rhs2 in constraints[i + 1:]:
            determinant = ax1 * ay2 - ax2 * ay1
            if abs(determinant) <= 1.0e-12:
                continue
            x = (rhs1 * ay2 - rhs2 * ay1) / determinant
            y = (ax1 * rhs2 - ax2 * rhs1) / determinant
            candidates.append((x, y))

    for x, y in candidates:
        if x < -bound - 1.0e-8 or x > bound + 1.0e-8:
            continue
        if y < -bound - 1.0e-8 or y > bound + 1.0e-8:
            continue
        if _point_satisfies_halfspaces((x, y), constraints):
            return True
    return False


def _xy(robot: dict[str, Any]) -> tuple[float, float]:
    state = robot.get("state", {})
    return float(state.get("x", math.nan)), float(state.get("y", math.nan))


def _uncertainty(robot: dict[str, Any]) -> float:
    return float(robot.get("uncertainty", 0.0))


def _physical_max_comm_excess(data: dict[str, Any]) -> float:
    min_margin = _physical_min_comm_margin(data)
    if not math.isfinite(min_margin):
        return 0.0
    return max(0.0, -min_margin)


def _physical_min_comm_margin(data: dict[str, Any]) -> float:
    config = data.get("config", {})
    comm_config = config.get("cbfs", {}).get("without-slack", {}).get("comm-fixed", {})
    if not comm_config.get("on", False):
        return 0.0
    max_range = float(comm_config.get("max-range", math.inf))
    bases = config.get("bases", [])
    min_margin = math.inf

    for frame in data.get("state", []):
        robots = {int(robot.get("id")): robot for robot in frame.get("robots", []) if "id" in robot}
        formations = {int(item.get("id")): item for item in frame.get("formation", []) if "id" in item}
        for robot_id, formation in formations.items():
            if robot_id not in robots:
                continue
            x1, y1 = _xy(robots[robot_id])
            uncertainty1 = _uncertainty(robots[robot_id])
            for anchor_id in formation.get("anchorIds", []):
                other_id = int(anchor_id)
                if other_id not in robots:
                    continue
                x2, y2 = _xy(robots[other_id])
                distance = math.hypot(x1 - x2, y1 - y2)
                margin = max_range - distance - uncertainty1 - _uncertainty(robots[other_id])
                min_margin = min(min_margin, margin)
            for base_id in formation.get("baseIds", []):
                base_index = int(base_id)
                if base_index < 0 or base_index >= len(bases):
                    continue
                bx, by = bases[base_index]
                distance = math.hypot(x1 - float(bx), y1 - float(by))
                margin = max_range - distance - uncertainty1
                min_margin = min(min_margin, margin)
    return min_margin if math.isfinite(min_margin) else 0.0


def _solver_failure_records(robot_opts: list[dict[str, Any]]) -> int:
    failures = 0
    allowed_solver_statuses = {"optimal", "optimal_inaccurate"}
    for opt in robot_opts:
        status = opt.get("status")
        solver_status = opt.get("solver_info", {}).get("status")
        opt_failed = status is not None and status != "success"
        solver_failed = solver_status is not None and solver_status not in allowed_solver_statuses
        if opt_failed or solver_failed:
            failures += 1
    return failures


def _topology_certified_flag(topology: dict[str, Any]) -> bool | None:
    if "certified" in topology:
        return bool(topology.get("certified", False))
    robust_margin = _float_or_nan(topology.get("min_robust_margin", math.nan))
    if math.isfinite(robust_margin):
        return robust_margin >= 0.0
    return None


def extract_bridge_metrics(data: dict[str, Any]) -> dict[str, float | str]:
    metadata = data.get("bridge", {}).get("metadata", {})
    high_order_config = data.get("config", {}).get("cbfs", {}).get("high-order", {})
    acceleration_bound = float(high_order_config.get("acceleration-bound", math.nan))
    frames = data.get("state", [])
    horizon_s = float(metadata.get("horizon_s", math.nan))
    control_sample_time_s = float(metadata.get("control_sample_time_s", math.nan))
    final_runtime_s = float(frames[-1].get("runtime", math.nan)) if frames else math.nan
    if math.isfinite(horizon_s) and math.isfinite(control_sample_time_s) and control_sample_time_s > 0.0:
        expected_frames = max(1, int(round(horizon_s / control_sample_time_s)))
        completed_horizon = 1.0 if len(frames) >= expected_frames else 0.0
    else:
        completed_horizon = math.nan
    robot_opts: list[dict[str, Any]] = [
        robot.get("opt", {})
        for frame in frames
        for robot in frame.get("robots", [])
    ]
    qp_successes = sum(1 for opt in robot_opts if opt.get("status") == "success")
    qp_total = len(robot_opts)

    search_frames = [frame.get("bridge", {}).get("search", {}) for frame in frames]
    topology_frames = [frame.get("bridge", {}).get("topology", {}) for frame in frames]
    coverage_threshold = float(
        data.get("config", {})
        .get("bridge", {})
        .get("search", {})
        .get("coverage-completion-threshold", 1.0)
    )
    relay_support_guard_frames = [
        frame.get("bridge", {}).get("nominal", {}).get("relay_support_guard", {})
        for frame in frames
    ]
    support_chain_guard_frames = [
        frame.get("bridge", {}).get("nominal", {}).get("support_chain_guard", {})
        for frame in frames
    ]
    predictive_gate_frames = [
        frame.get("bridge", {}).get("nominal", {}).get("predictive_active_gate", {})
        for frame in frames
    ]
    final_search = search_frames[-1] if search_frames else {}
    coverage_completion_times = [
        float(frame.get("runtime", math.nan))
        for frame, search in zip(frames, search_frames)
        if _float_or_nan(search.get("coverage_ratio", math.nan)) >= coverage_threshold - 1.0e-9
        and math.isfinite(float(frame.get("runtime", math.nan)))
    ]
    detected_times = [
        float(search.get("detection_time_s", -1.0))
        for search in search_frames
        if bool(search.get("detected", False)) and float(search.get("detection_time_s", -1.0)) >= 0.0
    ]

    cbf_values: list[float] = []
    safety_values: list[float] = []
    comm_values: list[float] = []
    hocbf_values: list[float] = []
    hocbf_h_values: list[float] = []
    hocbf_hdot_values: list[float] = []
    psi1_values: list[float] = []
    control_authority_margins: list[float] = []
    joint_hocbf_feasible_flags: list[bool] = []
    terminal_control_authority_margins: list[float] = []
    terminal_joint_hocbf_feasible_flags: list[bool] = []
    nominal_guard_active_flags: list[bool] = []
    nominal_guard_feasible_flags: list[bool] = []
    terminal_nominal_guard_feasible_flags: list[bool] = []
    nominal_guard_projection_norms: list[float] = []
    nominal_guard_margin_before_values: list[float] = []
    nominal_guard_margin_after_values: list[float] = []
    acceleration_values: list[float] = []
    yaw_values: list[float] = []
    slack_values: list[float] = []

    for frame_index, frame in enumerate(frames):
        is_terminal_frame = frame_index == len(frames) - 1
        for robot in frame.get("robots", []):
            for name, value in robot.get("cbfNoSlack", {}).items():
                numeric = float(value)
                cbf_values.append(numeric)
                lowered = name.lower()
                if "safety" in lowered:
                    safety_values.append(numeric)
                if "comm" in lowered or "anchor" in lowered:
                    comm_values.append(numeric)
            opt = robot.get("opt", {})
            slack_values.extend(_slack_values(opt))
            guard = opt.get("nominalGuard", {})
            if guard.get("enabled", False):
                nominal_guard_active_flags.append(bool(guard.get("active", False)))
                nominal_guard_feasible_flags.append(bool(guard.get("feasible", False)))
                if is_terminal_frame:
                    terminal_nominal_guard_feasible_flags.append(bool(guard.get("feasible", False)))
                nominal_guard_projection_norms.append(_float_or_nan(guard.get("projection_norm", math.nan)))
                nominal_guard_margin_before_values.append(_float_or_nan(guard.get("margin_before", math.nan)))
                nominal_guard_margin_after_values.append(_float_or_nan(guard.get("margin_after", math.nan)))
            result = opt.get("result", {})
            if "ax" in result and "ay" in result:
                acceleration_values.append(math.hypot(float(result["ax"]), float(result["ay"])))
            if "yawRateRad" in result:
                yaw_values.append(abs(float(result["yawRateRad"])))
            for item in opt.get("hocbfNoSlack", []):
                hocbf_values.append(float(item.get("hocbf", math.nan)))
                hocbf_h_values.append(float(item.get("h", math.nan)))
                hocbf_hdot_values.append(float(item.get("hdot", math.nan)))
                psi1_values.append(float(item.get("psi1", math.nan)))
                control_authority_margins.append(_hocbf_control_authority_margin(item, acceleration_bound))
                if is_terminal_frame:
                    terminal_control_authority_margins.append(_hocbf_control_authority_margin(item, acceleration_bound))
            joint_feasible = _hocbf_joint_acceleration_feasible(opt.get("hocbfNoSlack", []), acceleration_bound)
            if joint_feasible is not None:
                joint_hocbf_feasible_flags.append(joint_feasible)
                if is_terminal_frame:
                    terminal_joint_hocbf_feasible_flags.append(joint_feasible)

    relay_active_count = sum(1 for topology in topology_frames if bool(topology.get("relay_active", False)))
    certified_flags = [
        flag
        for topology in topology_frames
        if (flag := _topology_certified_flag(topology)) is not None
    ]
    fail_safe_flags = [
        bool(topology.get("fail_safe", False))
        for topology in topology_frames
    ]
    accepted_switches = max((int(topology.get("accepted_switches", 0)) for topology in topology_frames), default=0)
    rejected_candidates = max((int(topology.get("rejected_candidates", 0)) for topology in topology_frames), default=0)
    relay_support_guard_enabled_frames = [
        frame for frame in relay_support_guard_frames
        if bool(frame.get("enabled", False))
    ]
    relay_support_margins = [
        _float_or_nan(frame.get("robust_margin", math.nan))
        for frame in relay_support_guard_enabled_frames
    ]
    support_chain_guard_enabled_frames = [
        frame for frame in support_chain_guard_frames
        if bool(frame.get("enabled", False))
    ]
    support_chain_active_counts = [
        _float_or_nan(frame.get("active_count", math.nan))
        for frame in support_chain_guard_enabled_frames
    ]
    support_chain_margins_before = [
        _float_or_nan(frame.get("min_margin_before", math.nan))
        for frame in support_chain_guard_enabled_frames
    ]
    support_chain_margins_after = [
        _float_or_nan(frame.get("min_margin_after", math.nan))
        for frame in support_chain_guard_enabled_frames
    ]
    predictive_gate_enabled_frames = [
        frame for frame in predictive_gate_frames
        if bool(frame.get("enabled", False))
    ]
    predictive_gate_active_counts = [
        _float_or_nan(frame.get("active_count", math.nan))
        for frame in predictive_gate_enabled_frames
    ]
    predictive_gate_penalties = [
        _float_or_nan(frame.get("max_penalty", math.nan))
        for frame in predictive_gate_enabled_frames
    ]
    predictive_gate_selected_robust_margins = [
        _float_or_nan(frame.get("min_selected_robust_margin", math.nan))
        for frame in predictive_gate_enabled_frames
    ]
    predictive_gate_exposure_active_counts = [
        float(sum(1 for link in frame.get("links", []) if bool(link.get("exposure_active", False))))
        for frame in predictive_gate_enabled_frames
    ]
    predictive_gate_selected_exposure_utilities = [
        _float_or_nan(link.get("selected_exposure_utility", math.nan))
        for frame in predictive_gate_enabled_frames
        for link in frame.get("links", [])
    ]
    predictive_gate_max_exposure_utilities = [
        _float_or_nan(link.get("max_exposure_utility", math.nan))
        for frame in predictive_gate_enabled_frames
        for link in frame.get("links", [])
    ]
    predictive_gate_service_rejected_counts = [
        float(sum(_float_or_nan(link.get("service_rejected_candidates", 0.0)) for link in frame.get("links", [])))
        for frame in predictive_gate_enabled_frames
    ]
    predictive_gate_selected_service_utilities = [
        _float_or_nan(link.get("selected_service_utility", math.nan))
        for frame in predictive_gate_enabled_frames
        for link in frame.get("links", [])
    ]
    predictive_gate_max_service_utilities = [
        _float_or_nan(link.get("max_service_utility", math.nan))
        for frame in predictive_gate_enabled_frames
        for link in frame.get("links", [])
    ]
    predictive_gate_required_service_utilities = [
        _float_or_nan(link.get("required_service_utility", math.nan))
        for frame in predictive_gate_enabled_frames
        for link in frame.get("links", [])
    ]
    predictive_gate_service_schedule_due_counts = [
        float(sum(1 for link in frame.get("links", []) if bool(link.get("service_schedule_due", False))))
        for frame in predictive_gate_enabled_frames
    ]
    predictive_gate_service_schedule_deficits = [
        _float_or_nan(link.get("service_schedule_deficit", math.nan))
        for frame in predictive_gate_enabled_frames
        for link in frame.get("links", [])
    ]
    predictive_gate_required_searched_cells = [
        _float_or_nan(link.get("required_searched_cells", math.nan))
        for frame in predictive_gate_enabled_frames
        for link in frame.get("links", [])
    ]

    return {
        "row": str(metadata.get("row", "")),
        "area_width_m": float(metadata.get("area_width_m", math.nan)),
        "area_height_m": float(metadata.get("area_height_m", math.nan)),
        "horizon_s": horizon_s,
        "control_sample_time_s": control_sample_time_s,
        "sampled_data_reserve": float(high_order_config.get("sampled-data-reserve", 0.0)),
        "report_cadence_s": float(metadata.get("report_cadence_s", math.nan)),
        "frames": float(len(frames)),
        "final_runtime_s": final_runtime_s,
        "completed_horizon": completed_horizon,
        "final_coverage": float(final_search.get("coverage_ratio", math.nan)),
        "coverage_completed": float(1.0 if coverage_completion_times else 0.0),
        "coverage_completion_time_s": min(coverage_completion_times) if coverage_completion_times else math.nan,
        "final_belief_entropy": float(final_search.get("belief_entropy", math.nan)),
        "belief_at_target_final": float(final_search.get("belief_at_target", math.nan)),
        "detected": float(1.0 if detected_times else 0.0),
        "detection_time_s": min(detected_times) if detected_times else math.nan,
        "qp_success_ratio": float(qp_successes / qp_total) if qp_total else math.nan,
        "solver_failures": float(_solver_failure_records(robot_opts)),
        "max_comm_excess": _physical_max_comm_excess(data),
        "min_physical_comm_margin": _physical_min_comm_margin(data),
        "max_slack": _finite_max(slack_values, 0.0),
        "min_cbf": _finite_min(cbf_values),
        "min_collision_margin": _finite_min(safety_values),
        "min_localization_margin": _finite_min(comm_values),
        "min_hocbf": _finite_min(hocbf_values),
        "min_hocbf_h": _finite_min(hocbf_h_values),
        "min_hocbf_hdot": _finite_min(hocbf_hdot_values),
        "min_psi1": _finite_min(psi1_values),
        "min_control_authority_margin": _finite_min(control_authority_margins),
        "joint_hocbf_feasible_ratio": (
            float(sum(1 for flag in joint_hocbf_feasible_flags if flag) / len(joint_hocbf_feasible_flags))
            if joint_hocbf_feasible_flags else math.nan
        ),
        "terminal_min_control_authority_margin": _finite_min(terminal_control_authority_margins),
        "terminal_joint_hocbf_feasible_ratio": (
            float(sum(1 for flag in terminal_joint_hocbf_feasible_flags if flag) / len(terminal_joint_hocbf_feasible_flags))
            if terminal_joint_hocbf_feasible_flags else math.nan
        ),
        "terminal_hocbf_infeasible_robot_count": (
            float(sum(1 for flag in terminal_joint_hocbf_feasible_flags if not flag))
            if terminal_joint_hocbf_feasible_flags else math.nan
        ),
        "nominal_guard_active_ratio": (
            float(sum(1 for flag in nominal_guard_active_flags if flag) / len(nominal_guard_active_flags))
            if nominal_guard_active_flags else math.nan
        ),
        "nominal_guard_feasible_ratio": (
            float(sum(1 for flag in nominal_guard_feasible_flags if flag) / len(nominal_guard_feasible_flags))
            if nominal_guard_feasible_flags else math.nan
        ),
        "terminal_nominal_guard_feasible_ratio": (
            float(sum(1 for flag in terminal_nominal_guard_feasible_flags if flag) / len(terminal_nominal_guard_feasible_flags))
            if terminal_nominal_guard_feasible_flags else math.nan
        ),
        "max_nominal_guard_projection_norm": _finite_max(nominal_guard_projection_norms, 0.0),
        "min_nominal_guard_margin_before": _finite_min(nominal_guard_margin_before_values),
        "min_nominal_guard_margin_after": _finite_min(nominal_guard_margin_after_values),
        "max_acceleration": _finite_max(acceleration_values, 0.0),
        "max_yaw_rate": _finite_max(yaw_values, 0.0),
        "relay_active_ratio": float(relay_active_count / len(topology_frames)) if topology_frames else 0.0,
        "certified_graph_ratio": (
            float(sum(1 for flag in certified_flags if flag) / len(certified_flags))
            if certified_flags else math.nan
        ),
        "certified_graph_steps": float(sum(1 for flag in certified_flags if flag)) if certified_flags else math.nan,
        "fail_safe_ratio": (
            float(sum(1 for flag in fail_safe_flags if flag) / len(fail_safe_flags))
            if fail_safe_flags else 0.0
        ),
        "fail_safe_steps": float(sum(1 for flag in fail_safe_flags if flag)) if fail_safe_flags else 0.0,
        "relay_support_guard_active_ratio": (
            float(sum(1 for frame in relay_support_guard_enabled_frames if bool(frame.get("active", False))) / len(relay_support_guard_enabled_frames))
            if relay_support_guard_enabled_frames else math.nan
        ),
        "min_relay_support_margin": _finite_min(relay_support_margins),
        "terminal_relay_support_margin": (
            _float_or_nan(relay_support_guard_enabled_frames[-1].get("robust_margin", math.nan))
            if relay_support_guard_enabled_frames else math.nan
        ),
        "support_chain_guard_active_ratio": (
            float(sum(1 for count in support_chain_active_counts if math.isfinite(count) and count > 0.0) / len(support_chain_active_counts))
            if support_chain_active_counts else math.nan
        ),
        "max_support_chain_guard_active_count": _finite_max(support_chain_active_counts, 0.0),
        "min_support_chain_margin_before": _finite_min(support_chain_margins_before),
        "min_support_chain_margin_after": _finite_min(support_chain_margins_after),
        "terminal_support_chain_margin_before": (
            _float_or_nan(support_chain_guard_enabled_frames[-1].get("min_margin_before", math.nan))
            if support_chain_guard_enabled_frames else math.nan
        ),
        "terminal_support_chain_margin_after": (
            _float_or_nan(support_chain_guard_enabled_frames[-1].get("min_margin_after", math.nan))
            if support_chain_guard_enabled_frames else math.nan
        ),
        "predictive_gate_active_ratio": (
            float(sum(1 for count in predictive_gate_active_counts if math.isfinite(count) and count > 0.0) / len(predictive_gate_active_counts))
            if predictive_gate_active_counts else math.nan
        ),
        "max_predictive_gate_active_count": _finite_max(predictive_gate_active_counts, 0.0),
        "max_predictive_gate_penalty": _finite_max(predictive_gate_penalties, 0.0),
        "min_predictive_gate_selected_robust_margin": _finite_min(predictive_gate_selected_robust_margins),
        "terminal_predictive_gate_active_count": (
            _float_or_nan(predictive_gate_enabled_frames[-1].get("active_count", math.nan))
            if predictive_gate_enabled_frames else math.nan
        ),
        "terminal_predictive_gate_selected_robust_margin": (
            _float_or_nan(predictive_gate_enabled_frames[-1].get("min_selected_robust_margin", math.nan))
            if predictive_gate_enabled_frames else math.nan
        ),
        "exposure_gate_active_ratio": (
            float(sum(1 for count in predictive_gate_exposure_active_counts if math.isfinite(count) and count > 0.0) / len(predictive_gate_exposure_active_counts))
            if predictive_gate_exposure_active_counts else math.nan
        ),
        "max_exposure_gate_active_count": _finite_max(predictive_gate_exposure_active_counts, 0.0),
        "max_selected_exposure_utility": _finite_max(predictive_gate_selected_exposure_utilities, 0.0),
        "max_exposure_utility": _finite_max(predictive_gate_max_exposure_utilities, 0.0),
        "service_gate_rejection_ratio": (
            float(sum(1 for count in predictive_gate_service_rejected_counts if math.isfinite(count) and count > 0.0) / len(predictive_gate_service_rejected_counts))
            if predictive_gate_service_rejected_counts else math.nan
        ),
        "max_service_rejected_candidates": _finite_max(predictive_gate_service_rejected_counts, 0.0),
        "max_selected_service_utility": _finite_max(predictive_gate_selected_service_utilities, 0.0),
        "max_service_utility": _finite_max(predictive_gate_max_service_utilities, 0.0),
        "max_required_service_utility": _finite_max(predictive_gate_required_service_utilities, 0.0),
        "service_schedule_due_ratio": (
            float(sum(1 for count in predictive_gate_service_schedule_due_counts if math.isfinite(count) and count > 0.0) / len(predictive_gate_service_schedule_due_counts))
            if predictive_gate_service_schedule_due_counts else math.nan
        ),
        "max_service_schedule_due_count": _finite_max(predictive_gate_service_schedule_due_counts, 0.0),
        "max_service_schedule_deficit": _finite_max(predictive_gate_service_schedule_deficits, 0.0),
        "max_required_searched_cells": _finite_max(predictive_gate_required_searched_cells, 0.0),
        "accepted_switches": float(accepted_switches),
        "rejected_candidates": float(rejected_candidates),
        "min_robust_margin": _finite_min([float(t.get("min_robust_margin", math.nan)) for t in topology_frames]),
        "min_fim_eigenvalue": _finite_min([float(t.get("min_fim_eigenvalue", math.nan)) for t in topology_frames]),
    }


def write_summary(rows: list[dict[str, float | str]], output_csv: pathlib.Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "row",
        "area_width_m",
        "area_height_m",
        "horizon_s",
        "control_sample_time_s",
        "sampled_data_reserve",
        "report_cadence_s",
        "frames",
        "final_runtime_s",
        "completed_horizon",
        "final_coverage",
        "coverage_completed",
        "coverage_completion_time_s",
        "final_belief_entropy",
        "belief_at_target_final",
        "detected",
        "detection_time_s",
        "qp_success_ratio",
        "solver_failures",
        "max_comm_excess",
        "min_physical_comm_margin",
        "max_slack",
        "min_cbf",
        "min_collision_margin",
        "min_localization_margin",
        "min_hocbf",
        "min_hocbf_h",
        "min_hocbf_hdot",
        "min_psi1",
        "min_control_authority_margin",
        "joint_hocbf_feasible_ratio",
        "terminal_min_control_authority_margin",
        "terminal_joint_hocbf_feasible_ratio",
        "terminal_hocbf_infeasible_robot_count",
        "nominal_guard_active_ratio",
        "nominal_guard_feasible_ratio",
        "terminal_nominal_guard_feasible_ratio",
        "max_nominal_guard_projection_norm",
        "min_nominal_guard_margin_before",
        "min_nominal_guard_margin_after",
        "max_acceleration",
        "max_yaw_rate",
        "relay_active_ratio",
        "certified_graph_ratio",
        "certified_graph_steps",
        "fail_safe_ratio",
        "fail_safe_steps",
        "relay_support_guard_active_ratio",
        "min_relay_support_margin",
        "terminal_relay_support_margin",
        "support_chain_guard_active_ratio",
        "max_support_chain_guard_active_count",
        "min_support_chain_margin_before",
        "min_support_chain_margin_after",
        "terminal_support_chain_margin_before",
        "terminal_support_chain_margin_after",
        "predictive_gate_active_ratio",
        "max_predictive_gate_active_count",
        "max_predictive_gate_penalty",
        "min_predictive_gate_selected_robust_margin",
        "terminal_predictive_gate_active_count",
        "terminal_predictive_gate_selected_robust_margin",
        "exposure_gate_active_ratio",
        "max_exposure_gate_active_count",
        "max_selected_exposure_utility",
        "max_exposure_utility",
        "service_gate_rejection_ratio",
        "max_service_rejected_candidates",
        "max_selected_service_utility",
        "max_service_utility",
        "max_required_service_utility",
        "service_schedule_due_ratio",
        "max_service_schedule_due_count",
        "max_service_schedule_deficit",
        "max_required_searched_cells",
        "accepted_switches",
        "rejected_candidates",
        "min_robust_margin",
        "min_fim_eigenvalue",
    ]
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract full-simulator bridge evidence")
    parser.add_argument("data_json", nargs="+", type=pathlib.Path)
    parser.add_argument("--summary-csv", type=pathlib.Path, default=pathlib.Path("papers/bridge_full_simulator_summary.csv"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = [extract_bridge_metrics(json.loads(path.read_text(encoding="utf-8"))) for path in args.data_json]
    write_summary(rows, args.summary_csv)
    print(args.summary_csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
