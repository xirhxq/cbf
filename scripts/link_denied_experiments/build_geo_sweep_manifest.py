#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import pathlib

DATA_ROOT = pathlib.Path("data/link_denied_completion_geo_sweep_24")
GEOMETRIES = ["mr1200", "mr1125", "mr1100", "mr1075", "mr1050"]
MANIFEST_FIELDS = [
    "max_range",
    "row",
    "trial",
    "seed",
    "source_data",
    "completed_horizon",
    "coverage_completed",
    "coverage_completion_time_s",
    "detected",
    "detection_time_s",
    "final_coverage",
    "min_physical_comm_margin",
    "certified_graph_ratio",
    "fail_safe_ratio",
    "min_support_chain_margin_after",
    "terminal_support_chain_margin_after",
    "support_chain_guard_active_ratio",
    "qp_success_ratio",
    "solver_failures",
]


def main() -> None:
    out_path = DATA_ROOT / "source_data_manifest.csv"
    rows_out = []
    for geo in GEOMETRIES:
        max_range = float(geo.replace("mr", ""))
        trials_csv = DATA_ROOT / geo / "link_denied_full_bridge_trials.csv"
        if not trials_csv.exists():
            continue
        with trials_csv.open() as fh:
            for rec in csv.DictReader(fh):
                src = rec.get("source_data", "")
                if src:
                    src_path = pathlib.Path(src)
                    if not src_path.is_absolute():
                        src = str(src_path)
                out = {"max_range": f"{max_range:.0f}", "row": rec.get("row", "")}
                for f in MANIFEST_FIELDS[2:]:
                    out[f] = rec.get(f, "")
                out["source_data"] = src
                rows_out.append(out)
    with out_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        for r in rows_out:
            writer.writerow(r)
    print(f"wrote {out_path} ({len(rows_out)} rows)")

    metadata = {
        "experiment": "LinkDenied2026 geometric sweep: max-range knob to separate fixed-25m from state-dependent reserve",
        "command": ".venv/bin/python scripts/link_denied_experiments/run_geo_sweep.py --base config/bridge_full_simulator_base_alt_geo_mid.json --max-ranges 1200,1125,1100,1075,1050 --trials 24",
        "base_seed": 20260617,
        "trials": 24,
        "geometries": GEOMETRIES,
        "max_range_values": [1200, 1125, 1100, 1075, 1050],
        "rows": [
            "LD_COMPLETION_FIXED",
            "LD_COMPLETION_RELAY",
            "LD_COMPLETION_CHAIN_TASK_RESERVE",
            "LD_COMPLETION_CHAIN_TASK_RESERVE_SD",
        ],
        "verdict": "fixed-25m-robust: no geometry in [1050,1200] separates fixed-25m from state-dependent on coverage completion; SD advantage is terminal margin floor only",
        "manifest_rows": len(rows_out),
    }
    (DATA_ROOT / "run_metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"wrote {DATA_ROOT / 'run_metadata.json'}")


if __name__ == "__main__":
    main()
