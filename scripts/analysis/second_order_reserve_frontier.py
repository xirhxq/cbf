#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import pathlib
from typing import Any


MODE_BY_ROW = {
    "LD_HOCBF_PRED": "topology_reserve",
    "LD_HOCBF_PRED_EDGE": "constant_edge_reserve",
    "LD_HOCBF_PRED_SD": "state_dependent_edge_reserve",
    "LD_HOCBF_PRED_AE": "all_active_edge_reserve",
}

MODE_ORDER = [
    "topology_reserve",
    "constant_edge_reserve",
    "state_dependent_edge_reserve",
    "all_active_edge_reserve",
]

ROW_BY_MODE = {mode: row for row, mode in MODE_BY_ROW.items()}

SOURCE_DEFAULTS = [
    pathlib.Path("papers/LinkDenied2026/figures/predictive_hocbf_chain_step_bridge_6/link_denied_full_bridge_aggregate.csv"),
    pathlib.Path("papers/LinkDenied2026/figures/predictive_hocbf_edge_bridge_6/link_denied_full_bridge_aggregate.csv"),
    pathlib.Path("papers/LinkDenied2026/figures/predictive_hocbf_state_edge_bridge_6/link_denied_full_bridge_aggregate.csv"),
    pathlib.Path("papers/LinkDenied2026/figures/predictive_hocbf_all_edge_bridge_6/link_denied_full_bridge_aggregate.csv"),
]

SUMMARY_FIELDS = [
    "mode",
    "row",
    "trial_count",
    "completion_count",
    "completion_rate",
    "completion_delta_vs_topology",
    "detection_count",
    "detection_rate",
    "detection_delta_vs_topology",
    "mean_final_coverage",
    "coverage_delta_vs_topology",
    "min_physical_comm_margin_m",
    "physical_margin_delta_m",
    "min_robust_margin_m",
    "robust_margin_delta_m",
    "min_support_chain_margin_after_m",
    "support_chain_margin_delta_m",
    "terminal_support_chain_margin_after_m",
    "terminal_support_chain_margin_delta_m",
    "min_hocbf",
    "hocbf_delta",
    "min_control_authority_margin",
    "control_authority_delta",
    "terminal_joint_hocbf_feasible_ratio",
    "terminal_hocbf_infeasible_robot_count",
    "min_fim_eigenvalue",
]


def _float_or_nan(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result


def _metric(row: dict[str, Any], name: str) -> float:
    return _float_or_nan(row.get(name, math.nan))


def _delta(value: float, baseline: float) -> float:
    if not math.isfinite(value) or not math.isfinite(baseline):
        return math.nan
    return value - baseline


def _read_mode_rows(paths: list[pathlib.Path]) -> dict[str, dict[str, Any]]:
    rows_by_mode: dict[str, dict[str, Any]] = {}
    for path in paths:
        with path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                mode = MODE_BY_ROW.get(str(row.get("row", "")))
                if mode is None:
                    continue
                rows_by_mode[mode] = row
    missing = [mode for mode in MODE_ORDER if mode not in rows_by_mode]
    if missing:
        names = ", ".join(missing)
        raise ValueError(f"missing reserve-frontier rows: {names}")
    return rows_by_mode


def summarize_reserve_frontier(paths: list[pathlib.Path]) -> list[dict[str, float | str]]:
    rows_by_mode = _read_mode_rows(paths)
    baseline = rows_by_mode["topology_reserve"]

    baseline_completion = _metric(baseline, "completion_count")
    baseline_detection = _metric(baseline, "detection_count")
    baseline_coverage = _metric(baseline, "mean_final_coverage")
    baseline_physical = _metric(baseline, "min_physical_comm_margin_min")
    baseline_robust = _metric(baseline, "min_robust_margin_min")
    baseline_support = _metric(baseline, "min_support_chain_margin_after_min")
    baseline_terminal_support = _metric(baseline, "terminal_support_chain_margin_after_min")
    baseline_hocbf = _metric(baseline, "min_hocbf_min")
    baseline_authority = _metric(baseline, "min_control_authority_margin_min")

    summary: list[dict[str, float | str]] = []
    for mode in MODE_ORDER:
        row = rows_by_mode[mode]
        completion = _metric(row, "completion_count")
        detection = _metric(row, "detection_count")
        coverage = _metric(row, "mean_final_coverage")
        physical = _metric(row, "min_physical_comm_margin_min")
        robust = _metric(row, "min_robust_margin_min")
        support = _metric(row, "min_support_chain_margin_after_min")
        terminal_support = _metric(row, "terminal_support_chain_margin_after_min")
        hocbf = _metric(row, "min_hocbf_min")
        authority = _metric(row, "min_control_authority_margin_min")

        summary.append(
            {
                "mode": mode,
                "row": str(row.get("row", ROW_BY_MODE[mode])),
                "trial_count": _metric(row, "trial_count"),
                "completion_count": completion,
                "completion_rate": _metric(row, "completion_rate"),
                "completion_delta_vs_topology": _delta(completion, baseline_completion),
                "detection_count": detection,
                "detection_rate": _metric(row, "detection_rate"),
                "detection_delta_vs_topology": _delta(detection, baseline_detection),
                "mean_final_coverage": coverage,
                "coverage_delta_vs_topology": _delta(coverage, baseline_coverage),
                "min_physical_comm_margin_m": physical,
                "physical_margin_delta_m": _delta(physical, baseline_physical),
                "min_robust_margin_m": robust,
                "robust_margin_delta_m": _delta(robust, baseline_robust),
                "min_support_chain_margin_after_m": support,
                "support_chain_margin_delta_m": _delta(support, baseline_support),
                "terminal_support_chain_margin_after_m": terminal_support,
                "terminal_support_chain_margin_delta_m": _delta(terminal_support, baseline_terminal_support),
                "min_hocbf": hocbf,
                "hocbf_delta": _delta(hocbf, baseline_hocbf),
                "min_control_authority_margin": authority,
                "control_authority_delta": _delta(authority, baseline_authority),
                "terminal_joint_hocbf_feasible_ratio": _metric(row, "terminal_joint_hocbf_feasible_ratio_min"),
                "terminal_hocbf_infeasible_robot_count": _metric(row, "terminal_hocbf_infeasible_robot_count_max"),
                "min_fim_eigenvalue": _metric(row, "min_fim_eigenvalue_min"),
            }
        )
    return summary


def write_summary_csv(rows: list[dict[str, float | str]], path: pathlib.Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def write_frontier_plot(rows: list[dict[str, float | str]], path: pathlib.Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path.parent.mkdir(parents=True, exist_ok=True)
    labels = ["Topology", "Edge", "State", "All-edge"]
    completion = [_float_or_nan(row["completion_rate"]) for row in rows]
    robust = [_float_or_nan(row["min_robust_margin_m"]) for row in rows]
    support = [_float_or_nan(row["min_support_chain_margin_after_m"]) for row in rows]
    authority = [_float_or_nan(row["min_control_authority_margin"]) for row in rows]

    fig, axes = plt.subplots(1, 3, figsize=(10.6, 3.6), constrained_layout=False)
    panels = [
        (robust, "Worst robust topology margin (m)"),
        (support, "Worst support-chain margin (m)"),
        (authority, "Worst control-authority margin"),
    ]
    colors = ["#4c78a8", "#f58518", "#54a24b", "#b279a2"]

    legend_handles = []
    for axis, (values, ylabel) in zip(axes, panels):
        axis.axhline(0.0, color="#333333", linewidth=0.8, linestyle="--")
        for label, x_value, y_value, color in zip(labels, completion, values, colors):
            point = axis.scatter(
                [x_value],
                [y_value],
                s=72,
                color=color,
                edgecolor="white",
                linewidth=0.8,
                zorder=3,
            )
            if axis is axes[0]:
                legend_handles.append(point)
        axis.set_xlabel("Completion rate")
        axis.set_ylabel(ylabel)
        axis.set_xlim(-0.02, 0.72)
        axis.grid(True, alpha=0.25)

    fig.suptitle("Second-order reserve frontier on the six-seed link-denied bridge", y=0.98)
    fig.legend(legend_handles, labels, loc="upper center", ncol=4, frameon=False, bbox_to_anchor=(0.5, 0.90))
    fig.subplots_adjust(left=0.07, right=0.99, bottom=0.16, top=0.76, wspace=0.30)
    fig.savefig(path, dpi=220)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sources",
        nargs="*",
        type=pathlib.Path,
        default=SOURCE_DEFAULTS,
        help="Aggregate CSV files containing LD_HOCBF_PRED reserve variants.",
    )
    parser.add_argument(
        "--summary-csv",
        type=pathlib.Path,
        default=pathlib.Path("papers/SecondOrderCBF2026/assets/reserve_frontier_summary.csv"),
    )
    parser.add_argument(
        "--figure",
        type=pathlib.Path,
        default=pathlib.Path("papers/SecondOrderCBF2026/assets/reserve_frontier.png"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rows = summarize_reserve_frontier(args.sources)
    write_summary_csv(rows, args.summary_csv)
    write_frontier_plot(rows, args.figure)
    print(f"wrote {args.summary_csv}")
    print(f"wrote {args.figure}")


if __name__ == "__main__":
    main()
