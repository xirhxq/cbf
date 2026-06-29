import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "scripts" / "active_search_experiments" / "run_full_simulator_active_bridge.py"
BINARY = REPO / "build-codex" / "Swarm"


def parse_aggregate(csv_path: Path, focus_row: str):
    lines = csv_path.read_text().splitlines()
    header = lines[0].split(",")
    wanted = {
        "row",
        "completion_count",
        "detection_count",
        "mean_final_coverage",
        "service_schedule_due_ratio_mean",
        "service_schedule_due_ratio_max",
        "exposure_gate_active_ratio_mean",
    }
    idx = {name: header.index(name) for name in wanted if name in header}
    out = {}
    for line in lines[1:]:
        cells = line.split(",")
        if not cells:
            continue
        row_name = cells[0]
        rec = {}
        for name, i in idx.items():
            if i < len(cells):
                rec[name] = cells[i]
        out[row_name] = rec
    return out.get(focus_row, {})


def run_one(label: str, trials: int, base_seed: int, cut_factor: float,
            stall_factor: float, floor: float, data_subdir: str,
            config_dir: str, paper_dir: str) -> dict:
    data_dir = REPO / "data" / data_subdir
    if data_dir.exists():
        shutil.rmtree(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        "conda", "run", "-n", "cbf_env",
        "env", "MPLCONFIGDIR=/private/tmp/mpl-cache",
        "python", str(RUNNER),
        "--base", str(REPO / "config" / "bridge_full_simulator_base.json"),
        "--binary", str(BINARY),
        "--config-dir", str(REPO / config_dir),
        "--output-dir", str(REPO / paper_dir),
        "--data-dir", str(REPO / "data"),
        "--trials", str(trials),
        "--base-seed", str(base_seed),
        "--include-hocbf",
        "--include-predictive-gate",
        "--include-predictive-all-edge-gate",
        "--include-scheduled-service-exposure-all-edge-gate",
        "--include-scheduled-service-exposure-adaptive-all-edge-gate",
        "--predictive-reserve-margin", "25",
        "--exposure-weight", "160",
        "--exposure-lookahead-steps", "4",
        "--exposure-lookahead-step-m", "120",
        "--exposure-lookahead-discount", "0.7",
        "--exposure-service-schedule-rate-cells-per-s", "1.0",
        "--exposure-service-schedule-slack-cells", "20",
        "--exposure-service-schedule-rate-min-cells-per-s", str(floor),
        "--exposure-service-schedule-cut-factor", str(cut_factor),
        "--exposure-service-schedule-stall-factor", str(stall_factor),
    ]
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True)
    agg_csv = data_dir / "active_search_full_bridge_aggregate.csv"
    rec = parse_aggregate(agg_csv, "AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_AE") if agg_csv.exists() else {}
    rec["label"] = label
    rec["cut_factor"] = cut_factor
    rec["stall_factor"] = stall_factor
    rec["floor"] = floor
    rec["returncode"] = proc.returncode
    rec["stderr_tail"] = proc.stderr[-800:] if proc.stderr else ""
    return rec


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=3)
    parser.add_argument("--base-seed", type=int, default=20261217)
    parser.add_argument("--mode", choices=["smoke", "full"], default="smoke")
    parser.add_argument("--cut-factor", type=float, default=None)
    parser.add_argument("--stall-factor", type=float, default=None)
    parser.add_argument("--floor", type=float, default=None)
    parser.add_argument("--data-subdir", type=str, default="adaptive_schedule_v2_sweep_smoke")
    parser.add_argument("--config-dir", type=str, default="config/generated_adaptive_schedule_v2_sweep_smoke")
    parser.add_argument("--paper-dir", type=str, default="papers/ActiveSearch2026/assets/adaptive_schedule_v2_sweep_smoke")
    parser.add_argument("--label", type=str, default="single")
    args = parser.parse_args()

    if args.mode == "smoke":
        variants = [
            ("baseline_cut050_floor030", 0.5, 0.3, 0.3),
            ("shallow_cut065_floor030", 0.65, 0.3, 0.3),
            ("floor040_cut050", 0.5, 0.3, 0.4),
            ("shallow_cut065_floor040", 0.65, 0.3, 0.4),
        ]
        results = []
        for label, cut, stall, floor in variants:
            sub_data = f"{args.data_subdir}_{label}"
            cfg_dir = f"config/generated_{sub_data}"
            paper_dir = f"papers/ActiveSearch2026/assets/{sub_data}"
            rec = run_one(label, args.trials, args.base_seed, cut, stall, floor,
                          sub_data, cfg_dir, paper_dir)
            results.append(rec)
        print(json.dumps(results, indent=2))
        return 0

    if args.cut_factor is None or args.stall_factor is None or args.floor is None:
        raise ValueError("full mode requires --cut-factor --stall-factor --floor")
    rec = run_one(args.label, args.trials, args.base_seed, args.cut_factor,
                  args.stall_factor, args.floor, args.data_subdir,
                  args.config_dir, args.paper_dir)
    print(json.dumps(rec, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
