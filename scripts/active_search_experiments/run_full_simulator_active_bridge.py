#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import pathlib
import sys
from dataclasses import dataclass
from typing import Any

SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

from bridge_experiments.extract_full_simulator_bridge import extract_bridge_metrics
from bridge_experiments.run_full_simulator_bridge_mc import (
    _run_config,
    _safe_label,
    _scenario_seed,
    build_trial_config,
    enrich_trial_metrics,
    write_bridge_validation_artifact,
    write_bridge_mc_artifacts,
)


@dataclass(frozen=True)
class SuiteSpec:
    source_row: str
    label: str
    enable_nominal_guard: bool = False
    search_policy: str | None = None
    enable_predictive_gate: bool = False
    enable_exposure_gate: bool = False
    topology_policy: str | None = None
    robust_switch_margin: float = 0.0
    enable_state_dependent_edge_reserve: bool = False
    enable_task_aware_reserve: bool = False
    enable_goal_diversion: bool = False
    support_chain_guard_scope: str = "first-anchor"


def default_active_search_suite_specs(
    include_hocbf: bool = False,
    include_guard: bool = False,
    include_predictive_gate: bool = False,
    include_predictive_all_edge_gate: bool = False,
    include_exposure_all_edge_gate: bool = False,
    include_horizon_exposure_all_edge_gate: bool = False,
    include_service_exposure_all_edge_gate: bool = False,
    include_scheduled_service_exposure_all_edge_gate: bool = False,
    include_scheduled_service_exposure_adaptive_all_edge_gate: bool = False,
    include_belief_concentration_all_edge_gate: bool = False,
    include_task_aware_reserve: bool = False,
    include_task_aware_state_reserve: bool = False,
    include_goal_diversion: bool = False,
    predictive_reserve_margin: float = 25.0,
    state_reserve_distance: float = 120.0,
    state_reserve_radial: float = 4.0,
    state_reserve_velocity_gain: float = 1.0,
    state_reserve_max: float = 120.0,
) -> list[SuiteSpec]:
    specs = [
        SuiteSpec(source_row="R1", label="AS_COVERAGE"),
        SuiteSpec(source_row="R2", label="AS_ACTIVE"),
        SuiteSpec(source_row="R3", label="AS_RELAY"),
    ]
    if include_hocbf:
        specs.append(SuiteSpec(source_row="R4", label="AS_HOCBF"))
        if include_guard:
            specs.append(SuiteSpec(source_row="R4", label="AS_HOCBF_GUARD", enable_nominal_guard=True))
        if include_predictive_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED",
                    enable_nominal_guard=True,
                    search_policy="active-predictive",
                    enable_predictive_gate=True,
                )
            )
        if include_task_aware_reserve:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_TASK_RESERVE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive",
                    enable_predictive_gate=True,
                    topology_policy="adaptive-chain",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_task_aware_reserve=True,
                    support_chain_guard_scope="first-anchor",
                )
            )
        if include_task_aware_state_reserve:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_TASK_RESERVE_SD",
                    enable_nominal_guard=True,
                    search_policy="active-predictive",
                    enable_predictive_gate=True,
                    topology_policy="adaptive-chain",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_task_aware_reserve=True,
                    support_chain_guard_scope="first-anchor",
                )
            )
        if include_goal_diversion:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_TASK_RESERVE_DIVERT",
                    enable_nominal_guard=True,
                    search_policy="active-predictive",
                    enable_predictive_gate=True,
                    topology_policy="adaptive-chain",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_task_aware_reserve=True,
                    enable_goal_diversion=True,
                    support_chain_guard_scope="first-anchor",
                )
            )
        if include_predictive_all_edge_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED_AE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive",
                    enable_predictive_gate=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
        if include_exposure_all_edge_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED_EXPOSE_AE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive-exposure",
                    enable_predictive_gate=True,
                    enable_exposure_gate=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
        if include_horizon_exposure_all_edge_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED_EXPOSE_HORIZON_AE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive-exposure",
                    enable_predictive_gate=True,
                    enable_exposure_gate=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
        if include_service_exposure_all_edge_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED_EXPOSE_SERVICE_AE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive-exposure",
                    enable_predictive_gate=True,
                    enable_exposure_gate=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
        if include_scheduled_service_exposure_all_edge_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED_EXPOSE_SCHED_AE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive-exposure",
                    enable_predictive_gate=True,
                    enable_exposure_gate=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
        if include_scheduled_service_exposure_adaptive_all_edge_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_AE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive-exposure",
                    enable_predictive_gate=True,
                    enable_exposure_gate=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
        if include_belief_concentration_all_edge_gate:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_CONC_AE",
                    enable_nominal_guard=True,
                    search_policy="active-predictive-exposure",
                    enable_predictive_gate=True,
                    enable_exposure_gate=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
    return specs


def default_active_completion_suite_specs() -> list[SuiteSpec]:
    return [
        SuiteSpec(source_row="R2", label="AS_COMPLETION_ACTIVE", search_policy="active"),
        SuiteSpec(source_row="R2", label="AS_COMPLETION_FALLBACK", search_policy="coverage"),
    ]


def _validate_positive_finite(name: str, value: float) -> float:
    numeric = float(value)
    if numeric <= 0.0 or not (numeric < float("inf")):
        raise ValueError(f"{name} must be positive and finite")
    return numeric


def apply_predictive_active_gate(
    config: dict[str, Any],
    weight: float = 0.03,
    robust_margin: float = 25.0,
    horizon: int = 8,
    step_m: float = 60.0,
) -> None:
    if horizon <= 0:
        raise ValueError("predictive gate horizon must be positive")
    search = config.setdefault("bridge", {}).setdefault("search", {})
    search["predictive-feasibility-weight"] = _validate_positive_finite("predictive gate weight", weight)
    search["predictive-robust-margin"] = _validate_positive_finite("predictive gate robust margin", robust_margin)
    search["predictive-horizon"] = int(horizon)
    search["predictive-step-m"] = _validate_positive_finite("predictive gate step", step_m)


def apply_exposure_active_gate(
    config: dict[str, Any],
    weight: float,
    radius_m: float | None = None,
    half_angle_deg: float | None = None,
    unsearched_only: bool = True,
    lookahead_steps: int = 1,
    lookahead_step_m: float | None = None,
    lookahead_discount: float = 1.0,
    service_gate_min_cells: float = 0.0,
    service_gate_ratio: float = 0.0,
    service_schedule_rate_cells_per_s: float = 0.0,
    service_schedule_slack_cells: float = 0.0,
    service_schedule_adaptive: bool = False,
    service_schedule_rate_min_cells_per_s: float = 0.0,
    service_schedule_saturation_window: int = 4,
    service_schedule_cut_factor: float = 0.5,
    service_schedule_stall_factor: float = 0.3,
) -> None:
    if lookahead_steps <= 0:
        raise ValueError("exposure lookahead steps must be positive")
    if lookahead_step_m is not None:
        lookahead_step_m = _validate_positive_finite("exposure lookahead step", lookahead_step_m)
    lookahead_discount = _validate_positive_finite("exposure lookahead discount", lookahead_discount)
    if service_gate_min_cells < 0.0:
        raise ValueError("exposure service gate min cells must be nonnegative")
    if service_gate_ratio < 0.0:
        raise ValueError("exposure service gate ratio must be nonnegative")
    if service_schedule_rate_cells_per_s < 0.0:
        raise ValueError("exposure service schedule rate must be nonnegative")
    if service_schedule_slack_cells < 0.0:
        raise ValueError("exposure service schedule slack must be nonnegative")
    if service_schedule_rate_min_cells_per_s < 0.0:
        raise ValueError("exposure service schedule rate min must be nonnegative")
    if service_schedule_saturation_window < 1:
        raise ValueError("exposure service schedule saturation window must be positive")
    if not (0.0 <= service_schedule_cut_factor <= 1.0):
        raise ValueError("exposure service schedule cut factor must be in [0,1]")
    if not (0.0 <= service_schedule_stall_factor <= 1.0):
        raise ValueError("exposure service schedule stall factor must be in [0,1]")
    front_sector = config.get("searching", {}).get("front-sector", {})
    if radius_m is None:
        radius_m = float(front_sector.get("outer-radius", 260.0))
    if half_angle_deg is None:
        half_angle_deg = float(front_sector.get("half-angle-deg", 35.0))
    search = config.setdefault("bridge", {}).setdefault("search", {})
    search["exposure-weight"] = _validate_positive_finite("exposure weight", weight)
    search["exposure-radius-m"] = _validate_positive_finite("exposure radius", radius_m)
    search["exposure-half-angle-deg"] = _validate_positive_finite("exposure half angle", half_angle_deg)
    search["exposure-unsearched-only"] = bool(unsearched_only)
    search["exposure-lookahead-steps"] = int(lookahead_steps)
    if lookahead_step_m is not None:
        search["exposure-lookahead-step-m"] = lookahead_step_m
    search["exposure-lookahead-discount"] = lookahead_discount
    search["exposure-service-gate-min-cells"] = float(service_gate_min_cells)
    search["exposure-service-gate-ratio"] = float(service_gate_ratio)
    search["exposure-service-schedule-rate-cells-per-s"] = float(service_schedule_rate_cells_per_s)
    search["exposure-service-schedule-slack-cells"] = float(service_schedule_slack_cells)
    search["exposure-service-schedule-adaptive"] = bool(service_schedule_adaptive)
    search["exposure-service-schedule-rate-min-cells-per-s"] = float(service_schedule_rate_min_cells_per_s)
    search["exposure-service-schedule-saturation-window"] = int(service_schedule_saturation_window)
    search["exposure-service-schedule-cut-factor"] = float(service_schedule_cut_factor)
    search["exposure-service-schedule-stall-factor"] = float(service_schedule_stall_factor)


def apply_belief_concentration_gate(
    config: dict[str, Any],
    weight: float = 8.0,
    radius_m: float | None = None,
    sigma_m: float | None = None,
    mode: str = "mass",
) -> None:
    if weight < 0.0:
        raise ValueError("belief concentration weight must be nonnegative")
    if mode not in ("mass", "ridge", "gradient", "information_gain", "explore_mass", "verify", "hybrid"):
        raise ValueError("belief concentration mode must be one of mass/ridge/gradient/information_gain/explore_mass/verify/hybrid")
    search = config.setdefault("bridge", {}).setdefault("search", {})
    if radius_m is None:
        radius_m = float(search.get("exposure-radius-m", 260.0)) * 1.5
    if sigma_m is None:
        sigma_m = radius_m * 0.5
    search["belief-concentration-weight"] = float(weight)
    search["belief-concentration-radius-m"] = _validate_positive_finite(
        "belief concentration radius", radius_m
    )
    search["belief-concentration-sigma-m"] = _validate_positive_finite(
        "belief concentration sigma", sigma_m
    )
    search["belief-concentration-mode"] = str(mode)


def apply_predictive_state_dependent_edge_reserve(config: dict[str, Any]) -> None:
    comm_fixed = config.setdefault("cbfs", {}).setdefault("without-slack", {}).setdefault("comm-fixed", {})
    high_order = config.setdefault("cbfs", {}).setdefault("high-order", {})
    execute = config.setdefault("execute", {})
    comm_fixed["state-dependent-reserve"] = {
        "enabled": True,
        "velocity-gain": 1.0,
        "sample-time": float(execute.get("time-step", 0.5)),
        "acceleration-gain": 1.0,
        "neighbor-acceleration-bound": float(high_order.get("acceleration-bound", 8.0)),
        "max-reserve": 120.0,
    }


def apply_pair_state_safety_reserve(
    config: dict[str, Any],
    distance_threshold: float = 120.0,
    radial_threshold: float = 4.0,
    velocity_gain: float = 1.0,
    max_reserve: float = 120.0,
) -> None:
    safety = config.setdefault("cbfs", {}).setdefault("without-slack", {}).setdefault("safety", {})
    execute = config.setdefault("execute", {})
    safety["pair-state-reserve"] = {
        "enabled": True,
        "distance-threshold": float(distance_threshold),
        "radial-threshold": float(radial_threshold),
        "velocity-gain": float(velocity_gain),
        "sample-time": float(execute.get("time-step", 0.5)),
        "max-reserve": float(max_reserve),
    }


def apply_goal_diversion(
    config: dict[str, Any],
    distance_threshold: float = 120.0,
    radial_threshold: float = 4.0,
    separation_scale: float = 4.0,
    max_offset: float = 200.0,
    pair_scope: str = "all",
    pair_id_a: int = 3,
    pair_id_b: int = 4,
) -> None:
    nominal = config.setdefault("bridge", {}).setdefault("nominal", {})
    nominal["goal-diversion"] = {
        "enabled": True,
        "distance-threshold": float(distance_threshold),
        "radial-threshold": float(radial_threshold),
        "separation-scale": float(separation_scale),
        "max-offset": float(max_offset),
        "pair-scope": str(pair_scope),
        "pair-id-a": int(pair_id_a),
        "pair-id-b": int(pair_id_b),
    }


def apply_completion_stress_config(config: dict[str, Any]) -> None:
    config["model"] = "SingleIntegrate2D"
    config["dim"] = 4
    config.setdefault("initial", {}).pop("velocity", None)
    execute = config.setdefault("execute", {})
    execute["time-total"] = 400.0
    execute["time-step"] = 1.0

    high_order = config.setdefault("cbfs", {}).setdefault("high-order", {})
    high_order["enabled"] = False
    high_order["sampled-data-reserve"] = 0.0

    front_sector = config.setdefault("searching", {}).setdefault("front-sector", {})
    front_sector["outer-radius"] = 450.0
    front_sector["half-angle-deg"] = 60.0

    bridge = config.setdefault("bridge", {})
    bridge["safety-filter"] = "first-order-cbf"
    nominal = bridge.setdefault("nominal", {})
    nominal["max-speed"] = max(float(nominal.get("max-speed", 0.0)), 12.0)
    search = bridge.setdefault("search", {})
    search["coverage-completion-threshold"] = 1.0
    search["goal-boundary-margin"] = 0.0
    search["clarity-weight"] = 1.0
    search["belief-weight"] = max(float(search.get("belief-weight", 0.0)), 4.0)
    search["travel-weight"] = 0.0005
    search["target-prior-strength"] = max(float(search.get("target-prior-strength", 0.0)), 1000.0)


def write_active_search_bridge_artifacts(
    rows: list[dict[str, Any]],
    output_dir: pathlib.Path,
    require_coverage_completion: bool = False,
    require_search_completion: bool = False,
) -> dict[str, pathlib.Path]:
    paths = write_bridge_mc_artifacts(rows, output_dir, stem="active_search_full_bridge")
    renamed = {
        "trials_csv": output_dir / "active_search_full_bridge_trials.csv",
        "aggregate_csv": output_dir / "active_search_full_bridge_aggregate.csv",
    }
    for key, target in renamed.items():
        paths[key].replace(target)
        paths[key] = target
    paths["validation_csv"] = write_bridge_validation_artifact(
        rows,
        output_dir / "active_search_full_bridge_validation.csv",
        require_coverage_completion=require_coverage_completion,
        require_search_completion=require_search_completion,
    )
    return paths


def run_active_search_bridge_suite(
    base: dict[str, Any],
    config_dir: pathlib.Path,
    binary: pathlib.Path,
    output_dir: pathlib.Path,
    trials: int,
    base_seed: int,
    data_dir: pathlib.Path,
    include_hocbf: bool = False,
    include_guard: bool = False,
    include_predictive_gate: bool = False,
    include_predictive_all_edge_gate: bool = False,
    include_exposure_all_edge_gate: bool = False,
    include_horizon_exposure_all_edge_gate: bool = False,
    include_service_exposure_all_edge_gate: bool = False,
    include_scheduled_service_exposure_all_edge_gate: bool = False,
    include_scheduled_service_exposure_adaptive_all_edge_gate: bool = False,
    include_belief_concentration_all_edge_gate: bool = False,
    include_task_aware_reserve: bool = False,
    include_task_aware_state_reserve: bool = False,
    include_goal_diversion: bool = False,
    predictive_reserve_margin: float = 25.0,
    state_reserve_distance: float = 120.0,
    state_reserve_radial: float = 4.0,
    state_reserve_velocity_gain: float = 1.0,
    state_reserve_max: float = 120.0,
    predictive_gate_weight: float = 0.03,
    predictive_gate_robust_margin: float = 25.0,
    predictive_gate_horizon: int = 8,
    predictive_gate_step_m: float = 60.0,
    exposure_weight: float = 160.0,
    exposure_radius_m: float | None = None,
    exposure_half_angle_deg: float | None = None,
    exposure_unsearched_only: bool = True,
    exposure_lookahead_steps: int = 3,
    exposure_lookahead_step_m: float | None = None,
    exposure_lookahead_discount: float = 0.8,
    exposure_service_gate_min_cells: float = 4.0,
    exposure_service_gate_ratio: float = 0.6,
    exposure_service_schedule_rate_cells_per_s: float = 0.25,
    exposure_service_schedule_slack_cells: float = 12.0,
    exposure_service_schedule_adaptive: bool = False,
    exposure_service_schedule_rate_min_cells_per_s: float = 0.3,
    exposure_service_schedule_saturation_window: int = 4,
    exposure_service_schedule_cut_factor: float = 0.5,
    exposure_service_schedule_stall_factor: float = 0.3,
    belief_concentration_weight: float = 8.0,
    belief_concentration_radius_m: float | None = None,
    belief_concentration_sigma_m: float | None = None,
    belief_concentration_mode: str = "mass",
    goal_diversion_distance: float = 120.0,
    goal_diversion_radial: float = 4.0,
    goal_diversion_separation_scale: float = 4.0,
    goal_diversion_max_offset: float = 200.0,
    goal_diversion_pair_scope: str = "all",
    goal_diversion_pair_id_a: int = 3,
    goal_diversion_pair_id_b: int = 4,
    completion_stress: bool = False,
    dry_run: bool = False,
) -> list[dict[str, Any]]:
    config_dir.mkdir(parents=True, exist_ok=True)
    trial_rows: list[dict[str, Any]] = []
    if completion_stress:
        specs = default_active_completion_suite_specs()
    else:
        specs = default_active_search_suite_specs(
            include_hocbf=include_hocbf,
            include_guard=include_guard,
            include_predictive_gate=include_predictive_gate,
            include_predictive_all_edge_gate=include_predictive_all_edge_gate,
            include_exposure_all_edge_gate=include_exposure_all_edge_gate,
            include_horizon_exposure_all_edge_gate=include_horizon_exposure_all_edge_gate,
            include_service_exposure_all_edge_gate=include_service_exposure_all_edge_gate,
            include_scheduled_service_exposure_all_edge_gate=include_scheduled_service_exposure_all_edge_gate,
            include_scheduled_service_exposure_adaptive_all_edge_gate=include_scheduled_service_exposure_adaptive_all_edge_gate,
            include_belief_concentration_all_edge_gate=include_belief_concentration_all_edge_gate,
            include_task_aware_reserve=include_task_aware_reserve,
            include_task_aware_state_reserve=include_task_aware_state_reserve,
            include_goal_diversion=include_goal_diversion,
            predictive_reserve_margin=predictive_reserve_margin,
            state_reserve_distance=state_reserve_distance,
            state_reserve_radial=state_reserve_radial,
            state_reserve_velocity_gain=state_reserve_velocity_gain,
            state_reserve_max=state_reserve_max,
        )

    for trial in range(trials):
        seed = _scenario_seed(base_seed, trial)
        for spec in specs:
            config = build_trial_config(
                base,
                row=spec.source_row,
                row_label=spec.label,
                trial=trial,
                seed=seed,
                enable_nominal_guard=spec.enable_nominal_guard,
            )
            if completion_stress:
                apply_completion_stress_config(config)
            if spec.search_policy is not None:
                config["bridge"]["search-policy"] = spec.search_policy
            if completion_stress and spec.label == "AS_COMPLETION_FALLBACK":
                config["bridge"].setdefault("search", {})["fallback-from-policy"] = "active"
            if spec.enable_predictive_gate:
                apply_predictive_active_gate(
                    config,
                    weight=predictive_gate_weight,
                    robust_margin=predictive_gate_robust_margin,
                    horizon=predictive_gate_horizon,
                    step_m=predictive_gate_step_m,
                )
            if spec.enable_exposure_gate:
                use_horizon_exposure = spec.label == "AS_HOCBF_PRED_EXPOSE_HORIZON_AE"
                use_service_exposure = spec.label == "AS_HOCBF_PRED_EXPOSE_SERVICE_AE"
                use_scheduled_service_exposure = spec.label == "AS_HOCBF_PRED_EXPOSE_SCHED_AE"
                use_adaptive_scheduled_service_exposure = spec.label in (
                    "AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_AE",
                    "AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_CONC_AE",
                )
                use_belief_concentration = spec.label == "AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_CONC_AE"
                use_any_scheduled_service = use_scheduled_service_exposure or use_adaptive_scheduled_service_exposure
                use_any_lookahead = use_horizon_exposure or use_service_exposure or use_any_scheduled_service
                apply_exposure_active_gate(
                    config,
                    weight=exposure_weight,
                    radius_m=exposure_radius_m,
                    half_angle_deg=exposure_half_angle_deg,
                    unsearched_only=exposure_unsearched_only,
                    lookahead_steps=exposure_lookahead_steps if use_any_lookahead else 1,
                    lookahead_step_m=exposure_lookahead_step_m if use_any_lookahead else None,
                    lookahead_discount=exposure_lookahead_discount if use_any_lookahead else 1.0,
                    service_gate_min_cells=exposure_service_gate_min_cells if use_service_exposure else 0.0,
                    service_gate_ratio=exposure_service_gate_ratio if use_service_exposure else 0.0,
                    service_schedule_rate_cells_per_s=exposure_service_schedule_rate_cells_per_s if use_any_scheduled_service else 0.0,
                    service_schedule_slack_cells=exposure_service_schedule_slack_cells if use_any_scheduled_service else 0.0,
                    service_schedule_adaptive=use_adaptive_scheduled_service_exposure,
                    service_schedule_rate_min_cells_per_s=exposure_service_schedule_rate_min_cells_per_s,
                    service_schedule_saturation_window=exposure_service_schedule_saturation_window,
                    service_schedule_cut_factor=exposure_service_schedule_cut_factor,
                    service_schedule_stall_factor=exposure_service_schedule_stall_factor,
                )
                if use_belief_concentration:
                    apply_belief_concentration_gate(
                        config,
                        weight=belief_concentration_weight,
                        radius_m=belief_concentration_radius_m,
                        sigma_m=belief_concentration_sigma_m,
                        mode=belief_concentration_mode,
                    )
            if spec.topology_policy is not None:
                config["bridge"]["topology-policy"] = spec.topology_policy
            if spec.robust_switch_margin > 0.0:
                config["bridge"].setdefault("topology", {})["robust-switch-margin"] = float(spec.robust_switch_margin)
            if spec.topology_policy == "adaptive-relay-reserve":
                relay_guard = config["bridge"].setdefault("nominal", {}).setdefault("relay-support-guard", {})
                relay_guard["enabled"] = True
                relay_guard["robust-margin"] = float(spec.robust_switch_margin)
                chain_guard = config["bridge"].setdefault("nominal", {}).setdefault("support-chain-guard", {})
                chain_guard["enabled"] = True
                chain_guard["robust-margin"] = float(spec.robust_switch_margin)
                chain_guard["scope"] = spec.support_chain_guard_scope
            if spec.enable_task_aware_reserve:
                config["bridge"].setdefault("topology", {})["robust-switch-margin"] = float(spec.robust_switch_margin)
                chain_guard = config["bridge"].setdefault("nominal", {}).setdefault("support-chain-guard", {})
                chain_guard["enabled"] = True
                chain_guard["robust-margin"] = float(spec.robust_switch_margin)
                chain_guard["scope"] = spec.support_chain_guard_scope
            if spec.label == "AS_HOCBF_TASK_RESERVE_SD":
                apply_pair_state_safety_reserve(
                    config,
                    distance_threshold=state_reserve_distance,
                    radial_threshold=state_reserve_radial,
                    velocity_gain=state_reserve_velocity_gain,
                    max_reserve=state_reserve_max,
                )
            if spec.enable_goal_diversion:
                apply_goal_diversion(
                    config,
                    distance_threshold=goal_diversion_distance,
                    radial_threshold=goal_diversion_radial,
                    separation_scale=goal_diversion_separation_scale,
                    max_offset=goal_diversion_max_offset,
                    pair_scope=goal_diversion_pair_scope,
                    pair_id_a=goal_diversion_pair_id_a,
                    pair_id_b=goal_diversion_pair_id_b,
                )
            if spec.enable_state_dependent_edge_reserve:
                apply_predictive_state_dependent_edge_reserve(config)
            config["output_path"] = str(data_dir)
            config_path = config_dir / f"{_safe_label(spec.label)}_seed{seed}_trial{trial:03d}.json"
            config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")

            if dry_run:
                print(config_path)
                continue

            data_path = _run_config(binary, config_path, data_dir, str(config["run_suffix"]))
            data = json.loads(data_path.read_text(encoding="utf-8"))
            metrics = extract_bridge_metrics(data)
            trial_rows.append(enrich_trial_metrics(data, metrics, data_path, trial=trial, seed=seed))

    if not dry_run:
        paths = write_active_search_bridge_artifacts(
            trial_rows,
            output_dir,
            require_search_completion=completion_stress,
        )
        for path in paths.values():
            print(path)
    return trial_rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run ActiveSearch2026 full-simulator bridge suite")
    parser.add_argument("--base", type=pathlib.Path, default=pathlib.Path("config/bridge_full_simulator_base.json"))
    parser.add_argument("--config-dir", type=pathlib.Path, default=pathlib.Path("config/generated_active_search_full_bridge"))
    parser.add_argument("--binary", type=pathlib.Path, default=pathlib.Path("build-codex/Swarm"))
    parser.add_argument("--output-dir", type=pathlib.Path, default=pathlib.Path("papers/ActiveSearch2026/assets"))
    parser.add_argument("--data-dir", type=pathlib.Path, default=pathlib.Path("data"))
    parser.add_argument("--trials", type=int, default=12)
    parser.add_argument("--base-seed", type=int, default=20260617)
    parser.add_argument("--include-hocbf", action="store_true")
    parser.add_argument("--include-guard", action="store_true")
    parser.add_argument("--include-predictive-gate", action="store_true")
    parser.add_argument("--include-predictive-all-edge-gate", action="store_true")
    parser.add_argument("--include-exposure-all-edge-gate", action="store_true")
    parser.add_argument("--include-horizon-exposure-all-edge-gate", action="store_true")
    parser.add_argument("--include-service-exposure-all-edge-gate", action="store_true")
    parser.add_argument("--include-scheduled-service-exposure-all-edge-gate", action="store_true")
    parser.add_argument("--include-scheduled-service-exposure-adaptive-all-edge-gate", action="store_true")
    parser.add_argument("--include-belief-concentration-all-edge-gate", action="store_true")
    parser.add_argument("--include-task-aware-reserve", action="store_true")
    parser.add_argument("--include-task-aware-state-reserve", action="store_true")
    parser.add_argument("--predictive-reserve-margin", type=float, default=25.0)
    parser.add_argument("--state-reserve-distance", type=float, default=120.0)
    parser.add_argument("--state-reserve-radial", type=float, default=4.0)
    parser.add_argument("--state-reserve-velocity-gain", type=float, default=1.0)
    parser.add_argument("--state-reserve-max", type=float, default=120.0)
    parser.add_argument("--predictive-gate-weight", type=float, default=0.03)
    parser.add_argument("--predictive-gate-robust-margin", type=float, default=25.0)
    parser.add_argument("--predictive-gate-horizon", type=int, default=8)
    parser.add_argument("--predictive-gate-step-m", type=float, default=60.0)
    parser.add_argument("--exposure-weight", type=float, default=160.0)
    parser.add_argument("--exposure-radius-m", type=float, default=None)
    parser.add_argument("--exposure-half-angle-deg", type=float, default=None)
    parser.add_argument("--exposure-include-searched", action="store_true")
    parser.add_argument("--exposure-lookahead-steps", type=int, default=3)
    parser.add_argument("--exposure-lookahead-step-m", type=float, default=None)
    parser.add_argument("--exposure-lookahead-discount", type=float, default=0.8)
    parser.add_argument("--exposure-service-gate-min-cells", type=float, default=4.0)
    parser.add_argument("--exposure-service-gate-ratio", type=float, default=0.6)
    parser.add_argument("--exposure-service-schedule-rate-cells-per-s", type=float, default=0.25)
    parser.add_argument("--exposure-service-schedule-slack-cells", type=float, default=12.0)
    parser.add_argument("--exposure-service-schedule-rate-min-cells-per-s", type=float, default=0.3)
    parser.add_argument("--exposure-service-schedule-saturation-window", type=int, default=4)
    parser.add_argument("--exposure-service-schedule-cut-factor", type=float, default=0.5)
    parser.add_argument("--exposure-service-schedule-stall-factor", type=float, default=0.3)
    parser.add_argument("--belief-concentration-weight", type=float, default=8.0)
    parser.add_argument("--belief-concentration-radius-m", type=float, default=None)
    parser.add_argument("--belief-concentration-sigma-m", type=float, default=None)
    parser.add_argument("--belief-concentration-mode", type=str, default="mass", choices=["mass", "ridge", "gradient", "information_gain", "explore_mass", "verify", "hybrid"])
    parser.add_argument("--include-goal-diversion", action="store_true")
    parser.add_argument("--goal-diversion-distance", type=float, default=120.0)
    parser.add_argument("--goal-diversion-radial", type=float, default=4.0)
    parser.add_argument("--goal-diversion-separation-scale", type=float, default=4.0)
    parser.add_argument("--goal-diversion-max-offset", type=float, default=200.0)
    parser.add_argument("--goal-diversion-pair-scope", type=str, default="all")
    parser.add_argument("--goal-diversion-pair-id-a", type=int, default=3)
    parser.add_argument("--goal-diversion-pair-id-b", type=int, default=4)
    parser.add_argument("--completion-stress", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.trials <= 0:
        raise ValueError("--trials must be positive")
    base = json.loads(args.base.read_text(encoding="utf-8"))
    run_active_search_bridge_suite(
        base=base,
        config_dir=args.config_dir,
        binary=args.binary,
        output_dir=args.output_dir,
        trials=args.trials,
        base_seed=args.base_seed,
        data_dir=args.data_dir,
        include_hocbf=args.include_hocbf,
        include_guard=args.include_guard,
        include_predictive_gate=args.include_predictive_gate,
        include_predictive_all_edge_gate=args.include_predictive_all_edge_gate,
        include_exposure_all_edge_gate=args.include_exposure_all_edge_gate,
        include_horizon_exposure_all_edge_gate=args.include_horizon_exposure_all_edge_gate,
        include_service_exposure_all_edge_gate=args.include_service_exposure_all_edge_gate,
        include_scheduled_service_exposure_all_edge_gate=args.include_scheduled_service_exposure_all_edge_gate,
        include_scheduled_service_exposure_adaptive_all_edge_gate=args.include_scheduled_service_exposure_adaptive_all_edge_gate,
        include_belief_concentration_all_edge_gate=args.include_belief_concentration_all_edge_gate,
        include_task_aware_reserve=args.include_task_aware_reserve,
        include_task_aware_state_reserve=args.include_task_aware_state_reserve,
        include_goal_diversion=args.include_goal_diversion,
        predictive_reserve_margin=args.predictive_reserve_margin,
        state_reserve_distance=args.state_reserve_distance,
        state_reserve_radial=args.state_reserve_radial,
        state_reserve_velocity_gain=args.state_reserve_velocity_gain,
        state_reserve_max=args.state_reserve_max,
        predictive_gate_weight=args.predictive_gate_weight,
        predictive_gate_robust_margin=args.predictive_gate_robust_margin,
        predictive_gate_horizon=args.predictive_gate_horizon,
        predictive_gate_step_m=args.predictive_gate_step_m,
        exposure_weight=args.exposure_weight,
        exposure_radius_m=args.exposure_radius_m,
        exposure_half_angle_deg=args.exposure_half_angle_deg,
        exposure_unsearched_only=not args.exposure_include_searched,
        exposure_lookahead_steps=args.exposure_lookahead_steps,
        exposure_lookahead_step_m=args.exposure_lookahead_step_m,
        exposure_lookahead_discount=args.exposure_lookahead_discount,
        exposure_service_gate_min_cells=args.exposure_service_gate_min_cells,
        exposure_service_gate_ratio=args.exposure_service_gate_ratio,
        exposure_service_schedule_rate_cells_per_s=args.exposure_service_schedule_rate_cells_per_s,
        exposure_service_schedule_slack_cells=args.exposure_service_schedule_slack_cells,
        exposure_service_schedule_rate_min_cells_per_s=args.exposure_service_schedule_rate_min_cells_per_s,
        exposure_service_schedule_saturation_window=args.exposure_service_schedule_saturation_window,
        exposure_service_schedule_cut_factor=args.exposure_service_schedule_cut_factor,
        exposure_service_schedule_stall_factor=args.exposure_service_schedule_stall_factor,
        belief_concentration_weight=args.belief_concentration_weight,
        belief_concentration_radius_m=args.belief_concentration_radius_m,
        belief_concentration_sigma_m=args.belief_concentration_sigma_m,
        belief_concentration_mode=args.belief_concentration_mode,
        goal_diversion_distance=args.goal_diversion_distance,
        goal_diversion_radial=args.goal_diversion_radial,
        goal_diversion_separation_scale=args.goal_diversion_separation_scale,
        goal_diversion_max_offset=args.goal_diversion_max_offset,
        goal_diversion_pair_scope=args.goal_diversion_pair_scope,
        goal_diversion_pair_id_a=args.goal_diversion_pair_id_a,
        goal_diversion_pair_id_b=args.goal_diversion_pair_id_b,
        completion_stress=args.completion_stress,
        dry_run=args.dry_run,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
