#!/usr/bin/env python3
from __future__ import annotations

import csv
import pathlib

DATA_ROOT = pathlib.Path("data/link_denied_completion_geo_sweep_24")
GEOMETRIES = ["mr1200", "mr1125", "mr1100", "mr1075", "mr1050"]
TRIALS_EXPECTED = 24
ROWS = [
    "LD_COMPLETION_FIXED",
    "LD_COMPLETION_RELAY",
    "LD_COMPLETION_CHAIN_TASK_RESERVE",
    "LD_COMPLETION_CHAIN_TASK_RESERVE_SD",
]
ROW_SHORT = {
    "LD_COMPLETION_FIXED": "fixed",
    "LD_COMPLETION_RELAY": "relay",
    "LD_COMPLETION_CHAIN_TASK_RESERVE": "task-reserve-25m",
    "LD_COMPLETION_CHAIN_TASK_RESERVE_SD": "task-reserve-sd",
}


def read_agg(path: pathlib.Path) -> dict[str, dict[str, str]]:
    if not path.exists():
        return {}
    out: dict[str, dict[str, str]] = {}
    with path.open() as fh:
        reader = csv.DictReader(fh)
        for rec in reader:
            out[rec["row"]] = rec
    return out


def main() -> None:
    summary_path = DATA_ROOT / "geo_sweep_completion_table.csv"
    cols = [
        "max_range",
        "row",
        "trial_count",
        "completion_count",
        "completion_rate",
        "coverage_completion_count",
        "coverage_completion_rate",
        "certified_graph_ratio_mean",
        "certified_graph_ratio_min",
        "fail_safe_ratio_max",
        "fail_safe_steps_max",
        "solver_failures_max",
        "mean_qp_success_ratio",
    ]
    rows_out: list[dict[str, str]] = []
    for geo in GEOMETRIES:
        agg = read_agg(DATA_ROOT / geo / "link_denied_full_bridge_aggregate.csv")
        max_range = float(geo.replace("mr", ""))
        for row in ROWS:
            rec = agg.get(row, {})
            if not rec:
                continue
            out = {"max_range": f"{max_range:.0f}", "row": ROW_SHORT[row]}
            for c in cols[2:]:
                out[c] = rec.get(c, "")
            rows_out.append(out)
    with summary_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=cols)
        writer.writeheader()
        for r in rows_out:
            writer.writerow(r)
    print(f"wrote {summary_path}")
    print()
    print(f"{'geo':<8} {'row':<20} {'cmp_rate':>9} {'cov_rate':>9} {'cert_mean':>10} {'cert_min':>9} {'fs_max':>7} {'sf_max':>7}")
    for r in rows_out:
        print(
            f"{r['max_range']:<8} {r['row']:<20} "
            f"{float(r['completion_rate']):>9.3f} "
            f"{float(r['coverage_completion_rate']):>9.3f} "
            f"{float(r['certified_graph_ratio_mean']):>10.4f} "
            f"{float(r['certified_graph_ratio_min']):>9.3f} "
            f"{float(r['fail_safe_ratio_max']):>7.3f} "
            f"{float(r['solver_failures_max']):>7.1f}"
        )


if __name__ == "__main__":
    main()
