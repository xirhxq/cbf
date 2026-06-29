#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import pathlib
import sys

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
)
from active_search_experiments.run_full_simulator_active_bridge import (
    apply_goal_diversion,
    apply_predictive_active_gate,
)


BASE = pathlib.Path("config/bridge_full_simulator_base.json")
BINARY = pathlib.Path("build-codex/Swarm")
CONFIG_DIR = pathlib.Path("config/generated_iter7_anticipatory")
DATA_DIR = pathlib.Path("data/iter7_anticipatory")
OUT_DIR = pathlib.Path("data/iter7_anticipatory")
TRIALS = [(7, 20261224), (12, 20261229)]
BASE_SEED = 20261217
PREDICTIVE_RESERVE_MARGIN = 25.0

LOOKAHEAD_STEPS = 4
LOOKAHEAD_DISTANCE = 150.0
LOOKAHEAD_RADIAL = 2.0


def build_spec_config(base, trial, seed, label, mode):
    config = build_trial_config(
        base,
        row="R4",
        row_label=label,
        trial=trial,
        seed=seed,
        enable_nominal_guard=True,
    )
    config["bridge"]["search-policy"] = "active-predictive"
    apply_predictive_active_gate(
        config,
        weight=0.03,
        robust_margin=PREDICTIVE_RESERVE_MARGIN,
        horizon=8,
        step_m=60.0,
    )
    config["bridge"]["topology-policy"] = "adaptive-chain"
    config["bridge"].setdefault("topology", {})["robust-switch-margin"] = float(PREDICTIVE_RESERVE_MARGIN)
    chain_guard = config["bridge"].setdefault("nominal", {}).setdefault("support-chain-guard", {})
    chain_guard["enabled"] = True
    chain_guard["robust-margin"] = float(PREDICTIVE_RESERVE_MARGIN)
    chain_guard["scope"] = "first-anchor"
    if mode == "divert":
        apply_goal_diversion(
            config,
            distance_threshold=120.0,
            radial_threshold=4.0,
            separation_scale=4.0,
            max_offset=200.0,
            pair_scope="all",
            pair_id_a=3,
            pair_id_b=4,
        )
    elif mode == "divert_pred":
        apply_goal_diversion(
            config,
            distance_threshold=120.0,
            radial_threshold=4.0,
            separation_scale=4.0,
            max_offset=200.0,
            pair_scope="all",
            pair_id_a=3,
            pair_id_b=4,
            lookahead_steps=LOOKAHEAD_STEPS,
            lookahead_distance_threshold=LOOKAHEAD_DISTANCE,
            lookahead_radial_threshold=LOOKAHEAD_RADIAL,
        )
    return config


def main() -> int:
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    base = json.loads(BASE.read_text(encoding="utf-8"))
    rows = []
    for trial, seed in TRIALS:
        actual_seed = _scenario_seed(BASE_SEED, trial)
        if actual_seed != seed:
            raise RuntimeError(f"seed mismatch trial {trial}: {actual_seed} != {seed}")
        for label, mode in [
            ("AS_HOCBF_TASK_RESERVE_DIVERT", "divert"),
            ("AS_HOCBF_TASK_RESERVE_DIVERT_PRED", "divert_pred"),
        ]:
            config = build_spec_config(base, trial, seed, label, mode)
            config["output_path"] = str(DATA_DIR)
            config_path = CONFIG_DIR / f"{_safe_label(label)}_seed{seed}_trial{trial:03d}.json"
            config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
            data_path = _run_config(BINARY, config_path, DATA_DIR, str(config["run_suffix"]))
            data = json.loads(data_path.read_text(encoding="utf-8"))
            metrics = extract_bridge_metrics(data)
            row = enrich_trial_metrics(data, metrics, data_path, trial=trial, seed=seed)
            row["label"] = label
            row["config_path"] = str(config_path)
            row["data_path"] = str(data_path)
            rows.append(row)
            print(
                f"seed={seed} trial={trial} label={label} "
                f"completed_horizon={metrics.get('completed_horizon')} "
                f"min_control_authority_margin={metrics.get('min_control_authority_margin')} "
                f"terminal_joint_hocbf_feasible_ratio={metrics.get('terminal_joint_hocbf_feasible_ratio')}"
            )
    out_csv = OUT_DIR / "iter7_seed_results.csv"
    fields = ["label", "trial", "seed", "completed_horizon", "detected",
              "min_control_authority_margin", "terminal_joint_hocbf_feasible_ratio",
              "min_hocbf", "min_robust_margin", "config_path", "data_path"]
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print(f"WROTE {out_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
