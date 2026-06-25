#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import pathlib
import random
import re
import subprocess
import sys
import tempfile
from collections import defaultdict
from typing import Any, Iterable

SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

os.environ.setdefault("MPLCONFIGDIR", str(pathlib.Path(tempfile.gettempdir()) / "cbf-matplotlib"))
pathlib.Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from bridge_experiments.extract_full_simulator_bridge import extract_bridge_metrics
from bridge_experiments.run_full_simulator_bridge import build_row_config


DEFAULT_ROWS = ("R2", "R3", "R4")
TRIAL_FIELDS = [
    "row",
    "trial",
    "seed",
    "source_data",
    "target_x",
    "target_y",
    "prior_x",
    "prior_y",
    "denial_center_x",
    "denial_center_y",
    "denial_half_x",
    "denial_half_y",
    "area_width_m",
    "area_height_m",
    "horizon_s",
    "control_sample_time_s",
    "sampled_data_reserve",
    "frames",
    "final_runtime_s",
    "completed_horizon",
    "final_coverage",
    "coverage_completed",
    "coverage_completion_time_s",
    "detected",
    "detection_time_s",
    "qp_success_ratio",
    "solver_failures",
    "max_comm_excess",
    "min_physical_comm_margin",
    "certified_graph_ratio",
    "certified_graph_steps",
    "fail_safe_ratio",
    "fail_safe_steps",
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
AGGREGATE_FIELDS = [
    "row",
    "trial_count",
    "completion_count",
    "completion_rate",
    "detection_count",
    "detection_rate",
    "mean_detection_time_s",
    "coverage_completion_count",
    "coverage_completion_rate",
    "mean_coverage_completion_time_s",
    "coverage_completion_time_s_max",
    "mean_final_coverage",
    "std_final_coverage",
    "mean_qp_success_ratio",
    "solver_failures_max",
    "max_comm_excess_max",
    "min_physical_comm_margin_min",
    "max_slack_max",
    "min_cbf_min",
    "min_collision_margin_min",
    "min_localization_margin_min",
    "min_hocbf_min",
    "min_hocbf_h_min",
    "min_hocbf_hdot_min",
    "min_psi1_min",
    "min_control_authority_margin_min",
    "joint_hocbf_feasible_ratio_min",
    "terminal_min_control_authority_margin_min",
    "terminal_joint_hocbf_feasible_ratio_min",
    "terminal_hocbf_infeasible_robot_count_max",
    "nominal_guard_active_ratio_max",
    "nominal_guard_feasible_ratio_min",
    "terminal_nominal_guard_feasible_ratio_min",
    "max_nominal_guard_projection_norm_max",
    "min_nominal_guard_margin_before_min",
    "min_nominal_guard_margin_after_min",
    "max_acceleration_max",
    "max_yaw_rate_max",
    "relay_active_ratio_mean",
    "certified_graph_ratio_mean",
    "certified_graph_ratio_min",
    "certified_graph_steps_min",
    "fail_safe_ratio_mean",
    "fail_safe_ratio_max",
    "fail_safe_steps_max",
    "relay_support_guard_active_ratio_max",
    "min_relay_support_margin_min",
    "terminal_relay_support_margin_min",
    "support_chain_guard_active_ratio_max",
    "max_support_chain_guard_active_count_max",
    "min_support_chain_margin_before_min",
    "min_support_chain_margin_after_min",
    "terminal_support_chain_margin_before_min",
    "terminal_support_chain_margin_after_min",
    "predictive_gate_active_ratio_mean",
    "predictive_gate_active_ratio_max",
    "max_predictive_gate_active_count_max",
    "max_predictive_gate_penalty_max",
    "min_predictive_gate_selected_robust_margin_min",
    "terminal_predictive_gate_active_count_max",
    "terminal_predictive_gate_selected_robust_margin_min",
    "exposure_gate_active_ratio_mean",
    "exposure_gate_active_ratio_max",
    "max_exposure_gate_active_count_max",
    "max_selected_exposure_utility_max",
    "max_exposure_utility_max",
    "service_gate_rejection_ratio_mean",
    "service_gate_rejection_ratio_max",
    "max_service_rejected_candidates_max",
    "max_selected_service_utility_max",
    "max_service_utility_max",
    "max_required_service_utility_max",
    "service_schedule_due_ratio_mean",
    "service_schedule_due_ratio_max",
    "max_service_schedule_due_count_max",
    "max_service_schedule_deficit_max",
    "max_required_searched_cells_max",
    "min_robust_margin_min",
    "min_fim_eigenvalue_min",
]
VALIDATION_FIELDS = [
    "row",
    "trial",
    "seed",
    "source_data",
    "passed",
    "error_count",
    "warning_count",
    "frames",
    "bridge_completed_horizon",
    "bridge_final_runtime_s",
    "bridge_coverage_completed",
    "bridge_coverage_completion_time_s",
    "bridge_max_comm_excess",
    "bridge_min_physical_comm_margin",
    "bridge_certified_graph_ratio",
    "bridge_certified_graph_steps",
    "bridge_fail_safe_ratio",
    "bridge_fail_safe_steps",
    "bridge_qp_success_ratio",
    "bridge_solver_failures",
    "bridge_max_slack",
    "bridge_min_collision_margin",
    "bridge_min_localization_margin",
    "bridge_min_hocbf",
    "bridge_min_hocbf_h",
    "bridge_min_hocbf_hdot",
    "bridge_min_psi1",
    "bridge_min_control_authority_margin",
    "bridge_joint_hocbf_feasible_ratio",
    "bridge_terminal_min_control_authority_margin",
    "bridge_terminal_joint_hocbf_feasible_ratio",
    "bridge_terminal_hocbf_infeasible_robot_count",
    "bridge_nominal_guard_active_ratio",
    "bridge_nominal_guard_feasible_ratio",
    "bridge_terminal_nominal_guard_feasible_ratio",
    "bridge_max_nominal_guard_projection_norm",
    "bridge_min_nominal_guard_margin_before",
    "bridge_min_nominal_guard_margin_after",
    "bridge_predictive_gate_active_ratio",
    "bridge_max_predictive_gate_active_count",
    "bridge_max_predictive_gate_penalty",
    "bridge_min_predictive_gate_selected_robust_margin",
    "bridge_terminal_predictive_gate_active_count",
    "bridge_terminal_predictive_gate_selected_robust_margin",
]


def _world_bounds(config: dict[str, Any]) -> tuple[float, float, float, float]:
    boundary = config["world"]["boundary"]
    xs = [float(point[0]) for point in boundary]
    ys = [float(point[1]) for point in boundary]
    return min(xs), max(xs), min(ys), max(ys)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _safe_label(label: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", label).strip("_").lower()


def _finite_values(rows: Iterable[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        try:
            value = float(row.get(key, math.nan))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else math.nan


def _sample_std(values: list[float]) -> float:
    if len(values) <= 1:
        return 0.0 if values else math.nan
    mean = _mean(values)
    variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
    return math.sqrt(variance)


def _row_sort_key(label: str) -> tuple[int, str]:
    order = {"R1": 1, "R2": 2, "R3": 3, "R4": 4}
    return order.get(label, 99), label


def _scenario_seed(base_seed: int, trial: int) -> int:
    return int(base_seed) + int(trial)


def _sample_target_and_prior(config: dict[str, Any], rng: random.Random) -> tuple[dict[str, float], dict[str, float]]:
    min_x, max_x, min_y, max_y = _world_bounds(config)
    width = max_x - min_x
    height = max_y - min_y
    bridge = config["bridge"]
    radius = float(bridge["target"].get("radius", 180.0))
    search = bridge.get("search", {})
    margin = max(radius + 50.0, float(search.get("goal-boundary-margin", 250.0)))

    target_x = rng.uniform(min_x + 0.62 * width, min_x + 0.86 * width)
    target_y = rng.uniform(min_y + 0.30 * height, min_y + 0.72 * height)
    target_x = _clamp(target_x, min_x + margin, max_x - margin)
    target_y = _clamp(target_y, min_y + margin, max_y - margin)

    offset = rng.uniform(120.0, 360.0)
    angle = rng.uniform(0.0, 2.0 * math.pi)
    prior_x = _clamp(target_x + offset * math.cos(angle), min_x + margin, max_x - margin)
    prior_y = _clamp(target_y + offset * math.sin(angle), min_y + margin, max_y - margin)

    return (
        {"x": target_x, "y": target_y, "radius": radius},
        {"x": prior_x, "y": prior_y},
    )


def _sample_denial_zone(config: dict[str, Any], rng: random.Random) -> tuple[list[float], list[float]]:
    min_x, max_x, min_y, max_y = _world_bounds(config)
    width = max_x - min_x
    height = max_y - min_y
    center = [
        rng.uniform(min_x + 0.45 * width, min_x + 0.62 * width),
        rng.uniform(min_y + 0.40 * height, min_y + 0.60 * height),
    ]
    half_size = [
        rng.uniform(0.12 * width, 0.18 * width),
        rng.uniform(0.18 * height, 0.26 * height),
    ]
    return center, half_size


def build_trial_config(
    base: dict[str, Any],
    row: str,
    trial: int,
    seed: int,
    row_label: str | None = None,
    acceleration_bound: float | None = None,
    enable_nominal_guard: bool = False,
) -> dict[str, Any]:
    label = row_label or row
    config = build_row_config(base, row)
    rng = random.Random(seed)

    target, prior_center = _sample_target_and_prior(config, rng)
    denial_center, denial_half_size = _sample_denial_zone(config, rng)

    config["bridge"]["row"] = label
    config["bridge"]["target"] = target
    config["bridge"]["search"]["target-prior-center"] = prior_center
    config["bridge"]["topology"]["denial-center"] = denial_center
    config["bridge"]["topology"]["denial-half-size"] = denial_half_size
    config["execute"]["random-seed"] = int(seed)
    guard = config.setdefault("bridge", {}).setdefault("nominal", {}).setdefault("guard", {})
    guard.setdefault("mode", "hocbf-feasible-projection")
    guard.setdefault("tolerance", 1.0e-9)
    guard["enabled"] = bool(enable_nominal_guard)
    if acceleration_bound is not None:
        config["cbfs"]["high-order"]["acceleration-bound"] = float(acceleration_bound)

    config["run_suffix"] = f"_bridge_mc_seed{int(seed)}_trial{int(trial):03d}_{_safe_label(label)}"
    return config


def enrich_trial_metrics(
    data: dict[str, Any],
    metrics: dict[str, float | str],
    data_path: pathlib.Path,
    trial: int,
    seed: int,
) -> dict[str, float | str]:
    config = data.get("config", {})
    bridge = config.get("bridge", {})
    target = bridge.get("target", {})
    search = bridge.get("search", {})
    prior = search.get("target-prior-center", target)
    topology = bridge.get("topology", {})
    denial_center = topology.get("denial-center", [math.nan, math.nan])
    denial_half = topology.get("denial-half-size", [math.nan, math.nan])

    row = dict(metrics)
    row.update({
        "trial": float(trial),
        "seed": float(seed),
        "source_data": str(data_path),
        "target_x": float(target.get("x", math.nan)),
        "target_y": float(target.get("y", math.nan)),
        "prior_x": float(prior.get("x", math.nan)),
        "prior_y": float(prior.get("y", math.nan)),
        "denial_center_x": float(denial_center[0]) if len(denial_center) > 0 else math.nan,
        "denial_center_y": float(denial_center[1]) if len(denial_center) > 1 else math.nan,
        "denial_half_x": float(denial_half[0]) if len(denial_half) > 0 else math.nan,
        "denial_half_y": float(denial_half[1]) if len(denial_half) > 1 else math.nan,
    })
    return row


def aggregate_bridge_mc_metrics(rows: list[dict[str, Any]]) -> list[dict[str, float | str]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["row"])].append(row)

    aggregate: list[dict[str, float | str]] = []
    for row_label in sorted(grouped, key=_row_sort_key):
        items = grouped[row_label]
        detections = [float(item.get("detected", 0.0)) >= 0.5 for item in items]
        detection_times = [
            float(item.get("detection_time_s", math.nan))
            for item in items
            if float(item.get("detected", 0.0)) >= 0.5
            and math.isfinite(float(item.get("detection_time_s", math.nan)))
        ]
        coverage_completions = [float(item.get("coverage_completed", 0.0)) >= 0.5 for item in items]
        coverage_completion_times = [
            float(item.get("coverage_completion_time_s", math.nan))
            for item in items
            if float(item.get("coverage_completed", 0.0)) >= 0.5
            and math.isfinite(float(item.get("coverage_completion_time_s", math.nan)))
        ]
        coverage = _finite_values(items, "final_coverage")
        aggregate.append({
            "row": row_label,
            "trial_count": float(len(items)),
            "completion_count": float(sum(1 for item in items if float(item.get("completed_horizon", 0.0)) >= 0.5)),
            "completion_rate": (
                float(sum(1 for item in items if float(item.get("completed_horizon", 0.0)) >= 0.5) / len(items))
                if items else math.nan
            ),
            "detection_count": float(sum(1 for detected in detections if detected)),
            "detection_rate": float(sum(1 for detected in detections if detected) / len(items)) if items else math.nan,
            "mean_detection_time_s": _mean(detection_times),
            "coverage_completion_count": float(sum(1 for completed in coverage_completions if completed)),
            "coverage_completion_rate": (
                float(sum(1 for completed in coverage_completions if completed) / len(items))
                if items else math.nan
            ),
            "mean_coverage_completion_time_s": _mean(coverage_completion_times),
            "coverage_completion_time_s_max": max(coverage_completion_times, default=math.nan),
            "mean_final_coverage": _mean(coverage),
            "std_final_coverage": _sample_std(coverage),
            "mean_qp_success_ratio": _mean(_finite_values(items, "qp_success_ratio")),
            "solver_failures_max": max(_finite_values(items, "solver_failures"), default=math.nan),
            "max_comm_excess_max": max(_finite_values(items, "max_comm_excess"), default=math.nan),
            "min_physical_comm_margin_min": min(_finite_values(items, "min_physical_comm_margin"), default=math.nan),
            "max_slack_max": max(_finite_values(items, "max_slack"), default=math.nan),
            "min_cbf_min": min(_finite_values(items, "min_cbf"), default=math.nan),
            "min_collision_margin_min": min(_finite_values(items, "min_collision_margin"), default=math.nan),
            "min_localization_margin_min": min(_finite_values(items, "min_localization_margin"), default=math.nan),
            "min_hocbf_min": min(_finite_values(items, "min_hocbf"), default=math.nan),
            "min_hocbf_h_min": min(_finite_values(items, "min_hocbf_h"), default=math.nan),
            "min_hocbf_hdot_min": min(_finite_values(items, "min_hocbf_hdot"), default=math.nan),
            "min_psi1_min": min(_finite_values(items, "min_psi1"), default=math.nan),
            "min_control_authority_margin_min": min(_finite_values(items, "min_control_authority_margin"), default=math.nan),
            "joint_hocbf_feasible_ratio_min": min(_finite_values(items, "joint_hocbf_feasible_ratio"), default=math.nan),
            "terminal_min_control_authority_margin_min": min(_finite_values(items, "terminal_min_control_authority_margin"), default=math.nan),
            "terminal_joint_hocbf_feasible_ratio_min": min(_finite_values(items, "terminal_joint_hocbf_feasible_ratio"), default=math.nan),
            "terminal_hocbf_infeasible_robot_count_max": max(_finite_values(items, "terminal_hocbf_infeasible_robot_count"), default=math.nan),
            "nominal_guard_active_ratio_max": max(_finite_values(items, "nominal_guard_active_ratio"), default=math.nan),
            "nominal_guard_feasible_ratio_min": min(_finite_values(items, "nominal_guard_feasible_ratio"), default=math.nan),
            "terminal_nominal_guard_feasible_ratio_min": min(_finite_values(items, "terminal_nominal_guard_feasible_ratio"), default=math.nan),
            "max_nominal_guard_projection_norm_max": max(_finite_values(items, "max_nominal_guard_projection_norm"), default=math.nan),
            "min_nominal_guard_margin_before_min": min(_finite_values(items, "min_nominal_guard_margin_before"), default=math.nan),
            "min_nominal_guard_margin_after_min": min(_finite_values(items, "min_nominal_guard_margin_after"), default=math.nan),
            "max_acceleration_max": max(_finite_values(items, "max_acceleration"), default=math.nan),
            "max_yaw_rate_max": max(_finite_values(items, "max_yaw_rate"), default=math.nan),
            "relay_active_ratio_mean": _mean(_finite_values(items, "relay_active_ratio")),
            "certified_graph_ratio_mean": _mean(_finite_values(items, "certified_graph_ratio")),
            "certified_graph_ratio_min": min(_finite_values(items, "certified_graph_ratio"), default=math.nan),
            "certified_graph_steps_min": min(_finite_values(items, "certified_graph_steps"), default=math.nan),
            "fail_safe_ratio_mean": _mean(_finite_values(items, "fail_safe_ratio")),
            "fail_safe_ratio_max": max(_finite_values(items, "fail_safe_ratio"), default=math.nan),
            "fail_safe_steps_max": max(_finite_values(items, "fail_safe_steps"), default=math.nan),
            "relay_support_guard_active_ratio_max": max(_finite_values(items, "relay_support_guard_active_ratio"), default=math.nan),
            "min_relay_support_margin_min": min(_finite_values(items, "min_relay_support_margin"), default=math.nan),
            "terminal_relay_support_margin_min": min(_finite_values(items, "terminal_relay_support_margin"), default=math.nan),
            "support_chain_guard_active_ratio_max": max(_finite_values(items, "support_chain_guard_active_ratio"), default=math.nan),
            "max_support_chain_guard_active_count_max": max(_finite_values(items, "max_support_chain_guard_active_count"), default=math.nan),
            "min_support_chain_margin_before_min": min(_finite_values(items, "min_support_chain_margin_before"), default=math.nan),
            "min_support_chain_margin_after_min": min(_finite_values(items, "min_support_chain_margin_after"), default=math.nan),
            "terminal_support_chain_margin_before_min": min(_finite_values(items, "terminal_support_chain_margin_before"), default=math.nan),
            "terminal_support_chain_margin_after_min": min(_finite_values(items, "terminal_support_chain_margin_after"), default=math.nan),
            "predictive_gate_active_ratio_mean": _mean(_finite_values(items, "predictive_gate_active_ratio")),
            "predictive_gate_active_ratio_max": max(_finite_values(items, "predictive_gate_active_ratio"), default=math.nan),
            "max_predictive_gate_active_count_max": max(_finite_values(items, "max_predictive_gate_active_count"), default=math.nan),
            "max_predictive_gate_penalty_max": max(_finite_values(items, "max_predictive_gate_penalty"), default=math.nan),
            "min_predictive_gate_selected_robust_margin_min": min(_finite_values(items, "min_predictive_gate_selected_robust_margin"), default=math.nan),
            "terminal_predictive_gate_active_count_max": max(_finite_values(items, "terminal_predictive_gate_active_count"), default=math.nan),
            "terminal_predictive_gate_selected_robust_margin_min": min(_finite_values(items, "terminal_predictive_gate_selected_robust_margin"), default=math.nan),
            "exposure_gate_active_ratio_mean": _mean(_finite_values(items, "exposure_gate_active_ratio")),
            "exposure_gate_active_ratio_max": max(_finite_values(items, "exposure_gate_active_ratio"), default=math.nan),
            "max_exposure_gate_active_count_max": max(_finite_values(items, "max_exposure_gate_active_count"), default=math.nan),
            "max_selected_exposure_utility_max": max(_finite_values(items, "max_selected_exposure_utility"), default=math.nan),
            "max_exposure_utility_max": max(_finite_values(items, "max_exposure_utility"), default=math.nan),
            "service_gate_rejection_ratio_mean": _mean(_finite_values(items, "service_gate_rejection_ratio")),
            "service_gate_rejection_ratio_max": max(_finite_values(items, "service_gate_rejection_ratio"), default=math.nan),
            "max_service_rejected_candidates_max": max(_finite_values(items, "max_service_rejected_candidates"), default=math.nan),
            "max_selected_service_utility_max": max(_finite_values(items, "max_selected_service_utility"), default=math.nan),
            "max_service_utility_max": max(_finite_values(items, "max_service_utility"), default=math.nan),
            "max_required_service_utility_max": max(_finite_values(items, "max_required_service_utility"), default=math.nan),
            "service_schedule_due_ratio_mean": _mean(_finite_values(items, "service_schedule_due_ratio")),
            "service_schedule_due_ratio_max": max(_finite_values(items, "service_schedule_due_ratio"), default=math.nan),
            "max_service_schedule_due_count_max": max(_finite_values(items, "max_service_schedule_due_count"), default=math.nan),
            "max_service_schedule_deficit_max": max(_finite_values(items, "max_service_schedule_deficit"), default=math.nan),
            "max_required_searched_cells_max": max(_finite_values(items, "max_required_searched_cells"), default=math.nan),
            "min_robust_margin_min": min(_finite_values(items, "min_robust_margin"), default=math.nan),
            "min_fim_eigenvalue_min": min(_finite_values(items, "min_fim_eigenvalue"), default=math.nan),
        })
    return aggregate


def _write_csv(rows: list[dict[str, Any]], output_csv: pathlib.Path, fieldnames: list[str]) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name, math.nan) for name in fieldnames})


def _plot_bridge_mc_summary(aggregate: list[dict[str, Any]], output_prefix: pathlib.Path) -> dict[str, pathlib.Path]:
    if not aggregate:
        raise ValueError("At least one aggregate Monte Carlo row is required")

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 7,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
    })

    labels = [str(row["row"]) for row in aggregate]
    completion = [
        float(row.get("coverage_completion_rate", row.get("completion_rate", math.nan)))
        for row in aggregate
    ]
    detection = [float(row.get("detection_rate", math.nan)) for row in aggregate]
    coverage = [float(row.get("mean_final_coverage", math.nan)) for row in aggregate]
    comm = [float(row.get("max_comm_excess_max", math.nan)) for row in aggregate]

    fig, axes = plt.subplots(1, 4, figsize=(7.4, 1.95), constrained_layout=True)
    colors = ["#4f6d7a", "#5b8c5a", "#2f6f9f", "#b23a48"]
    axes[0].bar(labels, completion, color=colors[0], width=0.62)
    axes[0].set_ylim(0.0, 1.05)
    axes[0].set_ylabel("Search completion rate")

    axes[1].bar(labels, detection, color=colors[1], width=0.62)
    axes[1].set_ylim(0.0, 1.05)
    axes[1].set_ylabel("Detection rate")

    axes[2].bar(labels, coverage, color=colors[2], width=0.62)
    axes[2].set_ylim(0.0, max(0.6, max(coverage) * 1.15 if coverage else 0.6))
    axes[2].set_ylabel("Mean searched")

    axes[3].bar(labels, comm, color=colors[3], width=0.62)
    axes[3].set_ylabel("Max comm. excess (m)")
    axes[3].axhline(0.0, color="#222222", linewidth=0.8)

    for ax in axes:
        ax.tick_params(labelsize=7, length=2)
        ax.set_xlabel("Row")

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    outputs = {
        "figure_png": output_prefix.with_suffix(".png"),
        "figure_svg": output_prefix.with_suffix(".svg"),
        "figure_pdf": output_prefix.with_suffix(".pdf"),
    }
    fig.savefig(outputs["figure_png"], dpi=450, bbox_inches="tight")
    fig.savefig(outputs["figure_svg"], bbox_inches="tight")
    fig.savefig(outputs["figure_pdf"], bbox_inches="tight")
    plt.close(fig)
    return outputs


def write_bridge_mc_artifacts(rows: list[dict[str, Any]], output_dir: pathlib.Path, stem: str = "bridge_mc_summary") -> dict[str, pathlib.Path]:
    aggregate = aggregate_bridge_mc_metrics(rows)
    paths = {
        "trials_csv": output_dir / "bridge_mc_trials.csv",
        "aggregate_csv": output_dir / "bridge_mc_aggregate.csv",
    }
    _write_csv(rows, paths["trials_csv"], TRIAL_FIELDS)
    _write_csv(aggregate, paths["aggregate_csv"], AGGREGATE_FIELDS)
    paths.update(_plot_bridge_mc_summary(aggregate, output_dir / stem))
    return paths


def write_bridge_validation_artifact(
    rows: list[dict[str, Any]],
    output_csv: pathlib.Path,
    require_coverage_completion: bool = False,
    require_search_completion: bool = False,
) -> pathlib.Path:
    analysis_dir = SCRIPT_ROOT / "analysis"
    if str(analysis_dir) not in sys.path:
        sys.path.insert(0, str(analysis_dir))
    from validate_rationality import validate_data

    validation_rows: list[dict[str, Any]] = []
    for row in rows:
        source_data = pathlib.Path(str(row.get("source_data", "")))
        record: dict[str, Any] = {
            "row": row.get("row", ""),
            "trial": row.get("trial", math.nan),
            "seed": row.get("seed", math.nan),
            "source_data": str(source_data),
        }
        if not source_data.exists():
            record.update({
                "passed": 0.0,
                "error_count": 1.0,
                "warning_count": 0.0,
            })
            validation_rows.append(record)
            continue

        data = json.loads(source_data.read_text(encoding="utf-8"))
        result = validate_data(data)
        coverage_value = float(row.get("coverage_completed", math.nan))
        coverage_failed = (
            require_coverage_completion
            and (not math.isfinite(coverage_value) or coverage_value < 0.5)
        )
        detected_value = float(row.get("detected", math.nan))
        search_completed = (
            (math.isfinite(coverage_value) and coverage_value >= 0.5)
            or (math.isfinite(detected_value) and detected_value >= 0.5)
        )
        search_failed = require_search_completion and not search_completed
        record.update({
            "passed": 1.0 if result.passed and not coverage_failed and not search_failed else 0.0,
            "error_count": float(result.error_count + (1 if coverage_failed else 0) + (1 if search_failed else 0)),
            "warning_count": float(len(result.warnings)),
        })
        for field in VALIDATION_FIELDS:
            if field in record:
                continue
            if field.startswith("bridge_"):
                trial_field = field.removeprefix("bridge_")
                if trial_field in row:
                    record[field] = row.get(trial_field, math.nan)
                    continue
            record[field] = result.metrics.get(field, math.nan)
        validation_rows.append(record)

    _write_csv(validation_rows, output_csv, VALIDATION_FIELDS)
    return output_csv


def _data_path_for_suffix(data_dir: pathlib.Path, suffix: str, previous: set[pathlib.Path]) -> pathlib.Path:
    candidates = sorted(data_dir.glob(f"*{suffix}/data.json"), key=lambda path: path.stat().st_mtime)
    fresh = [path for path in candidates if path not in previous]
    if fresh:
        return fresh[-1]
    if candidates:
        return candidates[-1]
    raise FileNotFoundError(f"No data.json found for run suffix {suffix!r} under {data_dir}")


def _run_config(binary: pathlib.Path, config_path: pathlib.Path, data_dir: pathlib.Path, suffix: str) -> pathlib.Path:
    previous = set(data_dir.glob(f"*{suffix}/data.json"))
    subprocess.run([str(binary), str(config_path)], check=True)
    return _data_path_for_suffix(data_dir, suffix, previous)


def _trial_specs(rows: list[str], trials: int, base_seed: int, r4_accel_ablation: list[float]) -> Iterable[tuple[str, str, int, int, float | None]]:
    for trial in range(trials):
        seed = _scenario_seed(base_seed, trial)
        for row in rows:
            yield row, row, trial, seed, None
        for bound in r4_accel_ablation:
            label = f"R4_a{bound:g}"
            yield "R4", label, trial, seed, float(bound)


def run_bridge_mc(
    base: dict[str, Any],
    config_dir: pathlib.Path,
    binary: pathlib.Path,
    output_dir: pathlib.Path,
    rows: list[str],
    trials: int,
    base_seed: int,
    data_dir: pathlib.Path,
    r4_accel_ablation: list[float],
    dry_run: bool = False,
) -> list[dict[str, Any]]:
    config_dir.mkdir(parents=True, exist_ok=True)
    trial_rows: list[dict[str, Any]] = []

    for row, label, trial, seed, acceleration_bound in _trial_specs(rows, trials, base_seed, r4_accel_ablation):
        config = build_trial_config(
            base,
            row=row,
            row_label=label,
            trial=trial,
            seed=seed,
            acceleration_bound=acceleration_bound,
        )
        config_path = config_dir / f"{_safe_label(label)}_seed{seed}_trial{trial:03d}.json"
        config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
        if dry_run:
            print(config_path)
            continue

        data_path = _run_config(binary, config_path, data_dir, str(config["run_suffix"]))
        data = json.loads(data_path.read_text(encoding="utf-8"))
        metrics = extract_bridge_metrics(data)
        trial_rows.append(enrich_trial_metrics(data, metrics, data_path, trial=trial, seed=seed))

    if not dry_run:
        paths = write_bridge_mc_artifacts(trial_rows, output_dir)
        for path in paths.values():
            print(path)
    return trial_rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Monte Carlo full-simulator bridge scenarios")
    parser.add_argument("--base", type=pathlib.Path, default=pathlib.Path("config/bridge_full_simulator_base.json"))
    parser.add_argument("--config-dir", type=pathlib.Path, default=pathlib.Path("config/generated_bridge_mc"))
    parser.add_argument("--binary", type=pathlib.Path, default=pathlib.Path("build-codex/Swarm"))
    parser.add_argument("--output-dir", type=pathlib.Path, default=pathlib.Path("papers"))
    parser.add_argument("--data-dir", type=pathlib.Path, default=pathlib.Path("data"))
    parser.add_argument("--rows", nargs="+", default=list(DEFAULT_ROWS))
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--base-seed", type=int, default=20260617)
    parser.add_argument("--r4-accel-ablation", nargs="*", type=float, default=[])
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.trials <= 0:
        raise ValueError("--trials must be positive")
    base = json.loads(args.base.read_text(encoding="utf-8"))
    run_bridge_mc(
        base=base,
        config_dir=args.config_dir,
        binary=args.binary,
        output_dir=args.output_dir,
        rows=args.rows,
        trials=args.trials,
        base_seed=args.base_seed,
        data_dir=args.data_dir,
        r4_accel_ablation=args.r4_accel_ablation,
        dry_run=args.dry_run,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
