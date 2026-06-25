#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
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
    topology_policy: str | None = None
    robust_switch_margin: float = 0.0
    enable_edge_reserve_tightening: bool = False
    enable_state_dependent_edge_reserve: bool = False
    enable_task_aware_reserve: bool = False
    enable_task_aware_state_reserve: bool = False
    support_chain_guard_scope: str = "first-anchor"


def default_link_denied_suite_specs(
    include_hocbf: bool = False,
    include_guard: bool = False,
    include_predictive_reserve: bool = False,
    include_predictive_edge_reserve: bool = False,
    include_predictive_state_reserve: bool = False,
    include_predictive_all_edge_reserve: bool = False,
    predictive_reserve_margin: float = 25.0,
) -> list[SuiteSpec]:
    specs = [
        SuiteSpec(source_row="R2", label="LD_FIXED"),
        SuiteSpec(source_row="R3", label="LD_RELAY"),
    ]
    if include_hocbf:
        specs.append(SuiteSpec(source_row="R4", label="LD_HOCBF"))
        if include_guard:
            specs.append(SuiteSpec(source_row="R4", label="LD_HOCBF_GUARD", enable_nominal_guard=True))
        if include_predictive_reserve:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="LD_HOCBF_PRED",
                    enable_nominal_guard=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                )
            )
        if include_predictive_edge_reserve:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="LD_HOCBF_PRED_EDGE",
                    enable_nominal_guard=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_edge_reserve_tightening=True,
                )
            )
        if include_predictive_state_reserve:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="LD_HOCBF_PRED_SD",
                    enable_nominal_guard=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                )
            )
        if include_predictive_all_edge_reserve:
            specs.append(
                SuiteSpec(
                    source_row="R4",
                    label="LD_HOCBF_PRED_AE",
                    enable_nominal_guard=True,
                    topology_policy="adaptive-relay-reserve",
                    robust_switch_margin=predictive_reserve_margin,
                    enable_state_dependent_edge_reserve=True,
                    support_chain_guard_scope="all-active-edges",
                )
            )
    return specs


def _reserve_margin_label_suffix(margin: float) -> str:
    text = f"{float(margin):g}".replace(".", "P")
    return text.replace("-", "NEG")


def default_link_denied_completion_suite_specs(
    include_predictive_reserve: bool = False,
    include_task_aware_reserve: bool = False,
    include_task_aware_state_reserve: bool = False,
    predictive_reserve_margin: float = 25.0,
    predictive_reserve_margins: list[float] | None = None,
    state_reserve_base_margin: float = 25.0,
    state_reserve_headroom: float = 60.0,
    state_reserve_tighten_margin: float = 80.0,
    state_reserve_closing_rate_gain: float = 4.0,
    state_reserve_max_margin: float = 120.0,
) -> list[SuiteSpec]:
    specs = [
        SuiteSpec(source_row="R1", label="LD_COMPLETION_FIXED", topology_policy="fixed"),
        SuiteSpec(source_row="R3", label="LD_COMPLETION_RELAY", topology_policy="adaptive-chain"),
    ]
    if predictive_reserve_margins is not None:
        reserve_margins = [float(margin) for margin in predictive_reserve_margins]
    elif include_predictive_reserve:
        reserve_margins = [float(predictive_reserve_margin)]
    else:
        reserve_margins = []
    for margin in reserve_margins:
        if margin < 0.0:
            raise ValueError("predictive reserve margins must be nonnegative")
        label = "LD_COMPLETION_RELAY_RESERVE"
        if len(reserve_margins) > 1:
            label = f"LD_COMPLETION_RELAY_RESERVE_M{_reserve_margin_label_suffix(margin)}"
        specs.append(
            SuiteSpec(
                source_row="R3",
                label=label,
                topology_policy="adaptive-relay-reserve",
                robust_switch_margin=margin,
            )
        )
    if include_task_aware_reserve:
        if predictive_reserve_margin < 0.0:
            raise ValueError("task-aware reserve margin must be nonnegative")
        specs.append(
            SuiteSpec(
                source_row="R3",
                label="LD_COMPLETION_CHAIN_TASK_RESERVE",
                topology_policy="adaptive-chain",
                robust_switch_margin=float(predictive_reserve_margin),
                enable_task_aware_reserve=True,
                support_chain_guard_scope="first-anchor",
            )
        )
    if include_task_aware_state_reserve:
        if state_reserve_base_margin < 0.0:
            raise ValueError("state reserve base margin must be nonnegative")
        specs.append(
            SuiteSpec(
                source_row="R3",
                label="LD_COMPLETION_CHAIN_TASK_RESERVE_SD",
                topology_policy="adaptive-chain",
                robust_switch_margin=float(state_reserve_base_margin),
                enable_task_aware_state_reserve=True,
                support_chain_guard_scope="first-anchor",
            )
        )
    return specs


def worst_topology_range_uncertainty(config: dict[str, Any]) -> float:
    topology = config.get("bridge", {}).get("topology", {})
    denied_quality = float(topology.get("denied-quality", 1.0))
    if denied_quality <= 0.0:
        raise ValueError("bridge.topology.denied-quality must be positive for edge reserve tightening")
    sigma0 = float(topology.get("sigma0", 0.0))
    reference_variance = float(topology.get("reference-variance", 0.0))
    multiplier = float(topology.get("uncertainty-multiplier", 0.0))
    return multiplier * math.sqrt(sigma0 * sigma0 / denied_quality + reference_variance)


def apply_predictive_edge_reserve_tightening(config: dict[str, Any], robust_switch_margin: float) -> None:
    comm_fixed = config.setdefault("cbfs", {}).setdefault("without-slack", {}).setdefault("comm-fixed", {})
    existing = float(comm_fixed.get("range-tightening-margin", 0.0))
    max_range = float(comm_fixed.get("max-range", 0.0))
    requested = float(robust_switch_margin) + worst_topology_range_uncertainty(config)
    if max_range > 0.0:
        requested = min(requested, max_range - 1e-6)
    comm_fixed["range-tightening-margin"] = max(existing, requested)


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


def apply_completion_stress_config(config: dict[str, Any]) -> None:
    config["model"] = "SingleIntegrate2D"
    config.setdefault("initial", {}).pop("velocity", None)
    execute = config.setdefault("execute", {})
    execute["time-total"] = 400.0
    execute["time-step"] = 1.0

    high_order = config.setdefault("cbfs", {}).setdefault("high-order", {})
    high_order["enabled"] = False
    high_order["sampled-data-reserve"] = 0.0

    front_sector = config.setdefault("searching", {}).setdefault("front-sector", {})
    front_sector["outer-radius"] = max(float(front_sector.get("outer-radius", 0.0)), 650.0)
    front_sector["half-angle-deg"] = max(float(front_sector.get("half-angle-deg", 0.0)), 60.0)

    bridge = config.setdefault("bridge", {})
    bridge["search-policy"] = "coverage"
    bridge["safety-filter"] = "first-order-cbf"
    nominal = bridge.setdefault("nominal", {})
    nominal["max-speed"] = max(float(nominal.get("max-speed", 0.0)), 12.0)
    search = bridge.setdefault("search", {})
    search["target-prior-strength"] = 0.0
    search["belief-weight"] = 0.0
    search["clarity-weight"] = 1.0
    search["travel-weight"] = 0.0005
    search["goal-boundary-margin"] = 0.0

    support_guard = nominal.setdefault("support-chain-guard", {})
    support_guard["enabled"] = True
    support_guard["robust-margin"] = 0.0
    support_guard["scope"] = "all-active-edges"

    topology = bridge.setdefault("topology", {})
    comm_fixed = config.setdefault("cbfs", {}).setdefault("without-slack", {}).setdefault("comm-fixed", {})
    if "max-range" in comm_fixed:
        topology["max-range"] = float(comm_fixed["max-range"])
    robust_tightening = worst_topology_range_uncertainty(config)
    max_range = float(comm_fixed.get("max-range", topology.get("max-range", 0.0)))
    if max_range > 0.0:
        robust_tightening = min(robust_tightening, max_range - 1.0e-6)
    comm_fixed["range-tightening-margin"] = max(
        float(comm_fixed.get("range-tightening-margin", 0.0)),
        robust_tightening,
    )
    topology["certified-only"] = True
    topology["fail-safe-hold"] = True
    topology["certified-margin"] = 0.0


def write_link_denied_bridge_artifacts(
    rows: list[dict[str, Any]],
    output_dir: pathlib.Path,
    require_coverage_completion: bool = False,
) -> dict[str, pathlib.Path]:
    paths = write_bridge_mc_artifacts(rows, output_dir, stem="link_denied_full_bridge")
    renamed = {
        "trials_csv": output_dir / "link_denied_full_bridge_trials.csv",
        "aggregate_csv": output_dir / "link_denied_full_bridge_aggregate.csv",
    }
    for key, target in renamed.items():
        paths[key].replace(target)
        paths[key] = target
    paths["validation_csv"] = write_bridge_validation_artifact(
        rows,
        output_dir / "link_denied_full_bridge_validation.csv",
        require_coverage_completion=require_coverage_completion,
    )
    return paths


def run_link_denied_bridge_suite(
    base: dict[str, Any],
    config_dir: pathlib.Path,
    binary: pathlib.Path,
    output_dir: pathlib.Path,
    trials: int,
    base_seed: int,
    data_dir: pathlib.Path,
    include_hocbf: bool = False,
    include_guard: bool = False,
    include_predictive_reserve: bool = False,
    include_predictive_edge_reserve: bool = False,
    include_predictive_state_reserve: bool = False,
    include_predictive_all_edge_reserve: bool = False,
    include_task_aware_reserve: bool = False,
    include_task_aware_state_reserve: bool = False,
    predictive_reserve_margin: float = 25.0,
    completion_reserve_margins: list[float] | None = None,
    completion_stress: bool = False,
    state_reserve_base_margin: float = 25.0,
    state_reserve_headroom: float = 60.0,
    state_reserve_tighten_margin: float = 80.0,
    state_reserve_closing_rate_gain: float = 4.0,
    state_reserve_max_margin: float = 120.0,
    dry_run: bool = False,
) -> list[dict[str, Any]]:
    config_dir.mkdir(parents=True, exist_ok=True)
    trial_rows: list[dict[str, Any]] = []
    if completion_stress:
        specs = default_link_denied_completion_suite_specs(
            include_predictive_reserve=include_predictive_reserve,
            include_task_aware_reserve=include_task_aware_reserve,
            include_task_aware_state_reserve=include_task_aware_state_reserve,
            predictive_reserve_margin=predictive_reserve_margin,
            predictive_reserve_margins=completion_reserve_margins,
            state_reserve_base_margin=state_reserve_base_margin,
            state_reserve_headroom=state_reserve_headroom,
            state_reserve_tighten_margin=state_reserve_tighten_margin,
            state_reserve_closing_rate_gain=state_reserve_closing_rate_gain,
            state_reserve_max_margin=state_reserve_max_margin,
        )
    else:
        specs = default_link_denied_suite_specs(
            include_hocbf=include_hocbf,
            include_guard=include_guard,
            include_predictive_reserve=include_predictive_reserve,
            include_predictive_edge_reserve=include_predictive_edge_reserve,
            include_predictive_state_reserve=include_predictive_state_reserve,
            include_predictive_all_edge_reserve=include_predictive_all_edge_reserve,
            predictive_reserve_margin=predictive_reserve_margin,
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
            if spec.enable_task_aware_state_reserve:
                config["bridge"].setdefault("topology", {})["robust-switch-margin"] = float(state_reserve_base_margin)
                chain_guard = config["bridge"].setdefault("nominal", {}).setdefault("support-chain-guard", {})
                chain_guard["enabled"] = True
                chain_guard["robust-margin"] = float(state_reserve_base_margin)
                chain_guard["scope"] = spec.support_chain_guard_scope
                chain_guard["state-dependent-reserve"] = {
                    "enabled": True,
                    "base-margin": float(state_reserve_base_margin),
                    "headroom-margin": float(state_reserve_headroom),
                    "tighten-margin": float(state_reserve_tighten_margin),
                    "closing-rate-gain": float(state_reserve_closing_rate_gain),
                    "max-margin": float(state_reserve_max_margin),
                }
            if spec.enable_edge_reserve_tightening:
                apply_predictive_edge_reserve_tightening(config, spec.robust_switch_margin)
            if spec.enable_state_dependent_edge_reserve:
                apply_predictive_state_dependent_edge_reserve(config)
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
        paths = write_link_denied_bridge_artifacts(
            trial_rows,
            output_dir,
            require_coverage_completion=completion_stress,
        )
        for path in paths.values():
            print(path)
    return trial_rows


def parse_margin_list(value: str | None) -> list[float]:
    if value is None or not value.strip():
        return []
    margins: list[float] = []
    for chunk in value.split(","):
        text = chunk.strip()
        if not text:
            continue
        margin = float(text)
        if margin < 0.0:
            raise ValueError("--completion-reserve-margins values must be nonnegative")
        margins.append(margin)
    if not margins:
        raise ValueError("--completion-reserve-margins must contain at least one value")
    return margins


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run LinkDenied2026 full-simulator bridge suite")
    parser.add_argument("--base", type=pathlib.Path, default=pathlib.Path("config/bridge_full_simulator_base.json"))
    parser.add_argument("--config-dir", type=pathlib.Path, default=pathlib.Path("config/generated_link_denied_full_bridge"))
    parser.add_argument("--binary", type=pathlib.Path, default=pathlib.Path("build-codex/Swarm"))
    parser.add_argument("--output-dir", type=pathlib.Path, default=pathlib.Path("papers/LinkDenied2026/figures"))
    parser.add_argument("--data-dir", type=pathlib.Path, default=pathlib.Path("data"))
    parser.add_argument("--trials", type=int, default=12)
    parser.add_argument("--base-seed", type=int, default=20260617)
    parser.add_argument("--include-hocbf", action="store_true")
    parser.add_argument("--include-guard", action="store_true")
    parser.add_argument("--include-predictive-reserve", action="store_true")
    parser.add_argument("--include-predictive-edge-reserve", action="store_true")
    parser.add_argument("--include-predictive-state-reserve", action="store_true")
    parser.add_argument("--include-predictive-all-edge-reserve", action="store_true")
    parser.add_argument("--include-task-aware-reserve", action="store_true")
    parser.add_argument("--include-task-aware-state-reserve", action="store_true")
    parser.add_argument("--predictive-reserve-margin", type=float, default=25.0)
    parser.add_argument("--state-reserve-base-margin", type=float, default=25.0)
    parser.add_argument("--state-reserve-headroom", type=float, default=60.0)
    parser.add_argument("--state-reserve-tighten-margin", type=float, default=80.0)
    parser.add_argument("--state-reserve-closing-rate-gain", type=float, default=4.0)
    parser.add_argument("--state-reserve-max-margin", type=float, default=120.0)
    parser.add_argument(
        "--completion-reserve-margins",
        type=str,
        default="",
        help="Comma-separated predictive reserve margins for completion-stress frontier rows.",
    )
    parser.add_argument("--completion-stress", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.trials <= 0:
        raise ValueError("--trials must be positive")
    completion_reserve_margins = parse_margin_list(args.completion_reserve_margins)
    base = json.loads(args.base.read_text(encoding="utf-8"))
    run_link_denied_bridge_suite(
        base=base,
        config_dir=args.config_dir,
        binary=args.binary,
        output_dir=args.output_dir,
        trials=args.trials,
        base_seed=args.base_seed,
        data_dir=args.data_dir,
        include_hocbf=args.include_hocbf,
        include_guard=args.include_guard,
        include_predictive_reserve=args.include_predictive_reserve,
        include_predictive_edge_reserve=args.include_predictive_edge_reserve,
        include_predictive_state_reserve=args.include_predictive_state_reserve,
        include_predictive_all_edge_reserve=args.include_predictive_all_edge_reserve,
        include_task_aware_reserve=args.include_task_aware_reserve,
        include_task_aware_state_reserve=args.include_task_aware_state_reserve,
        predictive_reserve_margin=args.predictive_reserve_margin,
        completion_reserve_margins=completion_reserve_margins if completion_reserve_margins else None,
        completion_stress=args.completion_stress,
        state_reserve_base_margin=args.state_reserve_base_margin,
        state_reserve_headroom=args.state_reserve_headroom,
        state_reserve_tighten_margin=args.state_reserve_tighten_margin,
        state_reserve_closing_rate_gain=args.state_reserve_closing_rate_gain,
        state_reserve_max_margin=args.state_reserve_max_margin,
        dry_run=args.dry_run,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
