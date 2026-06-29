import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts"))

from active_search_experiments.run_full_simulator_active_bridge import (
    apply_belief_concentration_gate,
    apply_exposure_active_gate,
    apply_predictive_active_gate,
    apply_predictive_state_dependent_edge_reserve,
)
from bridge_experiments.run_full_simulator_bridge_mc import (
    _run_config,
    _safe_label,
    _scenario_seed,
    build_trial_config,
    enrich_trial_metrics,
)
from bridge_experiments.extract_full_simulator_bridge import extract_bridge_metrics

BINARY = REPO / "build-codex" / "Swarm"
BASE_CONFIG = REPO / "config" / "bridge_full_simulator_base.json"


def build_concentration_config(trial: int, base_seed: int, conc_weight: float,
                                conc_radius: float | None, conc_sigma: float | None,
                                conc_mode: str) -> dict:
    seed = _scenario_seed(base_seed, trial)
    base = json.loads(BASE_CONFIG.read_text(encoding="utf-8"))
    config = build_trial_config(
        base, row="R4", row_label="AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_CONC_AE",
        trial=trial, seed=seed, enable_nominal_guard=True,
    )
    config["bridge"]["search-policy"] = "active-predictive-exposure"
    apply_predictive_active_gate(config, weight=0.03, robust_margin=25.0, horizon=8, step_m=60.0)
    apply_exposure_active_gate(
        config, weight=160.0, radius_m=None, half_angle_deg=None, unsearched_only=True,
        lookahead_steps=4, lookahead_step_m=120.0, lookahead_discount=0.7,
        service_gate_min_cells=0.0, service_gate_ratio=0.0,
        service_schedule_rate_cells_per_s=1.0, service_schedule_slack_cells=20.0,
        service_schedule_adaptive=True, service_schedule_rate_min_cells_per_s=0.4,
        service_schedule_saturation_window=4, service_schedule_cut_factor=0.65,
        service_schedule_stall_factor=0.3,
    )
    apply_belief_concentration_gate(
        config, weight=conc_weight, radius_m=conc_radius, sigma_m=conc_sigma, mode=conc_mode,
    )
    config["bridge"]["topology-policy"] = "adaptive-relay-reserve"
    config["bridge"].setdefault("topology", {})["robust-switch-margin"] = 25.0
    relay_guard = config["bridge"].setdefault("nominal", {}).setdefault("relay-support-guard", {})
    relay_guard["enabled"] = True
    relay_guard["robust-margin"] = 25.0
    chain_guard = config["bridge"].setdefault("nominal", {}).setdefault("support-chain-guard", {})
    chain_guard["enabled"] = True
    chain_guard["robust-margin"] = 25.0
    chain_guard["scope"] = "all-active-edges"
    apply_predictive_state_dependent_edge_reserve(config)
    return config


def build_adaptive_config(trial: int, base_seed: int) -> dict:
    seed = _scenario_seed(base_seed, trial)
    base = json.loads(BASE_CONFIG.read_text(encoding="utf-8"))
    config = build_trial_config(
        base, row="R4", row_label="AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_AE",
        trial=trial, seed=seed, enable_nominal_guard=True,
    )
    config["bridge"]["search-policy"] = "active-predictive-exposure"
    apply_predictive_active_gate(config, weight=0.03, robust_margin=25.0, horizon=8, step_m=60.0)
    apply_exposure_active_gate(
        config, weight=160.0, radius_m=None, half_angle_deg=None, unsearched_only=True,
        lookahead_steps=4, lookahead_step_m=120.0, lookahead_discount=0.7,
        service_gate_min_cells=0.0, service_gate_ratio=0.0,
        service_schedule_rate_cells_per_s=1.0, service_schedule_slack_cells=20.0,
        service_schedule_adaptive=True, service_schedule_rate_min_cells_per_s=0.4,
        service_schedule_saturation_window=4, service_schedule_cut_factor=0.65,
        service_schedule_stall_factor=0.3,
    )
    config["bridge"]["topology-policy"] = "adaptive-relay-reserve"
    config["bridge"].setdefault("topology", {})["robust-switch-margin"] = 25.0
    relay_guard = config["bridge"].setdefault("nominal", {}).setdefault("relay-support-guard", {})
    relay_guard["enabled"] = True
    relay_guard["robust-margin"] = 25.0
    chain_guard = config["bridge"].setdefault("nominal", {}).setdefault("support-chain-guard", {})
    chain_guard["enabled"] = True
    chain_guard["robust-margin"] = 25.0
    chain_guard["scope"] = "all-active-edges"
    apply_predictive_state_dependent_edge_reserve(config)
    return config


def run_pilot(trials: list[int], base_seed: int, data_subdir: str, config_dir: str,
              conc_weight: float, conc_radius: float | None, conc_sigma: float | None,
              conc_mode: str, include_baseline: bool) -> list[dict]:
    data_dir = REPO / "data" / data_subdir
    if data_dir.exists():
        shutil.rmtree(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    cfg_dir = REPO / config_dir
    cfg_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    for trial in trials:
        seed = _scenario_seed(base_seed, trial)
        if include_baseline:
            config = build_adaptive_config(trial, base_seed)
            config["output_path"] = str(data_dir)
            cfg_path = cfg_dir / f"adapt_seed{seed}_trial{trial:03d}.json"
            cfg_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
            data_path = _run_config(BINARY, cfg_path, data_dir, str(config["run_suffix"]))
            data = json.loads(data_path.read_text(encoding="utf-8"))
            metrics = extract_bridge_metrics(data)
            row = enrich_trial_metrics(data, metrics, data_path, trial=trial, seed=seed)
            row["row"] = "AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_AE"
            rows.append(row)
        config = build_concentration_config(
            trial, base_seed, conc_weight, conc_radius, conc_sigma, conc_mode
        )
        config["output_path"] = str(data_dir)
        cfg_path = cfg_dir / f"conc_seed{seed}_trial{trial:03d}.json"
        cfg_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
        data_path = _run_config(BINARY, cfg_path, data_dir, str(config["run_suffix"]))
        data = json.loads(data_path.read_text(encoding="utf-8"))
        metrics = extract_bridge_metrics(data)
        row = enrich_trial_metrics(data, metrics, data_path, trial=trial, seed=seed)
        row["row"] = "AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_CONC_AE"
        rows.append(row)
    return rows


def belief_concentration_evidence(data_path: Path) -> dict:
    data = json.loads(data_path.read_text(encoding="utf-8"))
    steps = data.get("steps", [])
    if not steps:
        return {}
    first_search = steps[0].get("bridge", {}).get("search", {})
    last_search = steps[-1].get("bridge", {}).get("search", {})
    first_entropy = first_search.get("belief_entropy")
    last_entropy = last_search.get("belief_entropy")
    first_target = first_search.get("belief_at_target")
    last_target = last_search.get("belief_at_target")
    entropy_drop = None
    target_rise = None
    if first_entropy is not None and last_entropy is not None:
        entropy_drop = float(first_entropy) - float(last_entropy)
    if first_target is not None and last_target is not None:
        target_rise = float(last_target) - float(first_target)
    conc_active = 0
    conc_steps = 0
    for step in steps:
        gate = step.get("bridge", {}).get("nominal", {}).get("predictive_active_gate", {})
        links = gate.get("links", [])
        for link in links:
            if link.get("belief_concentration_active"):
                conc_active += 1
                break
        if links:
            conc_steps += 1
    return {
        "entropy_drop": entropy_drop,
        "target_rise": target_rise,
        "final_entropy": last_entropy,
        "final_target": last_target,
        "conc_active_ratio": conc_active / max(1, conc_steps),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, nargs="+", default=[0, 1, 5, 8, 9, 22])
    parser.add_argument("--base-seed", type=int, default=20261217)
    parser.add_argument("--data-subdir", type=str, default="iter6_concentration_pilot")
    parser.add_argument("--config-dir", type=str, default="config/generated_iter6_concentration_pilot")
    parser.add_argument("--conc-weight", type=float, default=8.0)
    parser.add_argument("--conc-radius", type=float, default=None)
    parser.add_argument("--conc-sigma", type=float, default=None)
    parser.add_argument("--conc-mode", type=str, default="mass", choices=["mass", "ridge", "gradient", "information_gain", "explore_mass", "verify", "hybrid"])
    parser.add_argument("--no-baseline", action="store_true")
    parser.add_argument("--label", type=str, default="mass_w8")
    args = parser.parse_args()

    rows = run_pilot(
        args.trials, args.base_seed, args.data_subdir, args.config_dir,
        args.conc_weight, args.conc_radius, args.conc_sigma, args.conc_mode,
        include_baseline=not args.no_baseline,
    )
    data_dir = REPO / "data" / args.data_subdir
    evidence = []
    for row in rows:
        data_path = Path(row["source_data"])
        ev = belief_concentration_evidence(data_path)
        ev["row"] = row["row"]
        ev["trial"] = int(row["trial"])
        ev["seed"] = int(row["seed"])
        ev["detected"] = int(float(row.get("detected", 0)))
        ev["final_coverage"] = float(row.get("final_coverage", 0.0))
        ev["detection_time_s"] = row.get("detection_time_s")
        ev["source_data"] = row["source_data"]
        evidence.append(ev)
    out = {
        "label": args.label,
        "conc_weight": args.conc_weight,
        "conc_mode": args.conc_mode,
        "conc_radius": args.conc_radius,
        "conc_sigma": args.conc_sigma,
        "evidence": evidence,
    }
    summary_path = data_dir / "pilot_summary.json"
    summary_path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
