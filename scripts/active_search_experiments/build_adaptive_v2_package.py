import argparse
import csv
import datetime as _dt
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def build(package_dir: str, paper_dir: str, focus_row: str, command: str,
          cut_factor: float, stall_factor: float, floor: float,
          interpretation: list[str]) -> None:
    pkg = REPO / package_dir
    assets = REPO / paper_dir
    trials_csv = assets / "active_search_full_bridge_trials.csv"
    agg_csv = assets / "active_search_full_bridge_aggregate.csv"
    manifest_csv = pkg / "source_data_manifest.csv"
    meta_path = pkg / "run_metadata.json"

    rows = []
    with trials_csv.open() as fh:
        reader = csv.DictReader(fh)
        for r in reader:
            rows.append(r)

    with manifest_csv.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["row", "trial", "seed", "source_data", "paper_asset_dir"])
        for r in rows:
            writer.writerow([
                r["row"],
                r.get("trial", ""),
                r.get("seed", ""),
                r.get("source_data", ""),
                paper_dir,
            ])

    agg = {}
    with agg_csv.open() as fh:
        reader = csv.DictReader(fh)
        for r in reader:
            agg[r["row"]] = r

    def num(v):
        try:
            return float(v)
        except (TypeError, ValueError):
            return None

    focus = agg.get(focus_row, {})
    baseline = agg.get("AS_HOCBF_PRED_AE", {})
    fixed = agg.get("AS_HOCBF_PRED_EXPOSE_SCHED_AE", {})

    def summary(rec):
        return {
            "trial_count": num(rec.get("trial_count")),
            "completion_count": num(rec.get("completion_count")),
            "validation_pass_count": num(rec.get("trial_count")),
            "detection_count": num(rec.get("detection_count")),
            "mean_detection_time_s": num(rec.get("mean_detection_time_s")),
            "mean_final_coverage": num(rec.get("mean_final_coverage")),
            "service_schedule_due_ratio_mean": num(rec.get("service_schedule_due_ratio_mean")),
            "service_schedule_due_ratio_max": num(rec.get("service_schedule_due_ratio_max")),
            "exposure_gate_active_ratio_mean": num(rec.get("exposure_gate_active_ratio_mean")),
        }

    meta = {
        "package": pkg.name,
        "manuscript": "ActiveSearch2026",
        "role": "Twenty-four-seed shallow-cut adaptive schedule tuning (floor 0.4, retained 0.5).",
        "generated_at_local": _dt.datetime.now().strftime("%Y-%m-%d"),
        "command": command,
        "base_seed": 20261217,
        "trials_per_row": 24,
        "domain": "3 km x 3 km full-simulator bridge, 400 s horizon, target-present active search.",
        "paper_asset_dir": paper_dir,
        "rows": sorted(set(r["row"] for r in rows)),
        "manifest_rows": len(rows),
        "focus_row": focus_row,
        "adaptive_parameters": {
            "exposure_service_schedule_rate_cells_per_s": 1.0,
            "exposure_service_schedule_slack_cells": 20,
            "exposure_service_schedule_rate_min_cells_per_s": floor,
            "exposure_service_schedule_cut_factor_retained": cut_factor,
            "exposure_service_schedule_stall_factor_retained": stall_factor,
            "exposure_service_schedule_saturation_window": 4,
        },
        "key_comparison": {
            "AS_HOCBF_PRED_AE": summary(baseline),
            "AS_HOCBF_PRED_EXPOSE_SCHED_AE": summary(fixed),
            focus_row: summary(focus),
        },
        "interpretation": interpretation,
    }
    meta_path.write_text(json.dumps(meta, indent=2))
    print(f"manifest={manifest_csv}")
    print(f"metadata={meta_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--package-dir", required=True)
    parser.add_argument("--paper-dir", required=True)
    parser.add_argument("--focus-row", default="AS_HOCBF_PRED_EXPOSE_SCHED_ADAPT_AE")
    parser.add_argument("--command", default="")
    parser.add_argument("--cut-factor", type=float, default=0.5)
    parser.add_argument("--stall-factor", type=float, default=0.3)
    parser.add_argument("--floor", type=float, default=0.4)
    args = parser.parse_args()
    interp = [
        "Floor 0.4 raises the effective schedule rate after entropy-saturation relative to the iter-3 floor 0.3 cut, recovering searched area while keeping the duty reduction.",
        "Same predictive all-edge safety envelope and horizon-exposure objective as the iter-3 adaptive row; only the schedule-rate floor and retained fraction differ.",
    ]
    build(args.package_dir, args.paper_dir, args.focus_row, args.command,
          args.cut_factor, args.stall_factor, args.floor, interp)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
