#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from pathlib import Path
from statistics import mean, median
from typing import Any


DEFAULT_TRIALS = Path("data/second_order_active_task_reserve_24/active_search_full_bridge_trials.csv")
DEFAULT_VALIDATION = Path("data/second_order_active_task_reserve_24/active_search_full_bridge_validation.csv")
DEFAULT_TRIAL_OUT = Path("data/second_order_active_task_reserve_24/task_interface_boundary_trials.csv")
DEFAULT_SUMMARY_OUT = Path("data/second_order_active_task_reserve_24/task_interface_boundary_summary.csv")

HOCBF_ROWS = {
    "AS_HOCBF",
    "AS_HOCBF_PRED",
    "AS_HOCBF_PRED_AE",
    "AS_HOCBF_TASK_RESERVE",
}

TRIAL_FIELDS = [
    "row",
    "trial",
    "seed",
    "completed_horizon",
    "validation_passed",
    "detected",
    "final_runtime_s",
    "final_coverage",
    "min_hocbf",
    "min_control_authority_margin",
    "terminal_min_control_authority_margin",
    "joint_hocbf_feasible_ratio",
    "terminal_joint_hocbf_feasible_ratio",
    "terminal_hocbf_infeasible_robot_count",
    "min_support_chain_margin_after_m",
    "terminal_support_chain_margin_after_m",
    "min_robust_margin_m",
    "failure_mode",
    "interpretation",
]

SUMMARY_FIELDS = [
    "row",
    "completed_horizon",
    "trial_count",
    "validation_pass_count",
    "detection_count",
    "mean_final_coverage",
    "min_hocbf_min",
    "min_hocbf_median",
    "min_control_authority_margin_min",
    "min_control_authority_margin_median",
    "terminal_min_control_authority_margin_min",
    "terminal_joint_hocbf_feasible_ratio_min",
    "terminal_hocbf_infeasible_robot_count_max",
    "min_support_chain_margin_after_m_min",
    "terminal_support_chain_margin_after_m_min",
    "min_robust_margin_m_min",
    "failure_mode_counts",
]


def _float_or_nan(value: Any) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result


def _finite(values: list[float]) -> list[float]:
    return [value for value in values if math.isfinite(value)]


def _metric(row: dict[str, Any], name: str) -> float:
    return _float_or_nan(row.get(name, "nan"))


def _flag(value: Any) -> int:
    number = _float_or_nan(value)
    if not math.isfinite(number):
        return 0
    return int(number > 0.5)


def _read_validation(path: Path) -> dict[tuple[str, int], int]:
    result: dict[tuple[str, int], int] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            key = (str(row.get("row", "")), int(_float_or_nan(row.get("trial", "nan"))))
            result[key] = _flag(row.get("passed", 0))
    return result


def _classify(row: dict[str, Any]) -> tuple[str, str]:
    if _flag(row.get("completed_horizon", 0)):
        return "completed", "The horizon completed under the recorded HOCBF task interface."

    terminal_infeasible = _metric(row, "terminal_hocbf_infeasible_robot_count")
    terminal_authority = _metric(row, "terminal_min_control_authority_margin")
    min_authority = _metric(row, "min_control_authority_margin")
    if terminal_infeasible > 0 or terminal_authority < 0 or min_authority < 0:
        return (
            "acceleration_admissibility_loss",
            "The run stops with negative HOCBF/control-authority diagnostics, so the task interface has reached the acceleration-admissibility boundary.",
        )

    support_after = _metric(row, "min_support_chain_margin_after")
    terminal_support = _metric(row, "terminal_support_chain_margin_after")
    if support_after < 0 or terminal_support < 0:
        return (
            "support_chain_margin_loss",
            "The run does not complete and the support-chain reserve margin is negative.",
        )

    robust = _metric(row, "min_robust_margin")
    if robust < 0:
        return (
            "robust_topology_margin_loss",
            "The run does not complete and the robust-topology margin is negative.",
        )

    return (
        "mission_not_completed_without_recorded_boundary_loss",
        "The run does not complete, but the available aggregate diagnostics do not isolate a single boundary loss.",
    )


def build_trial_rows(trials_path: Path, validation_path: Path) -> list[dict[str, Any]]:
    validation = _read_validation(validation_path)
    rows: list[dict[str, Any]] = []
    with trials_path.open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            row_name = str(raw.get("row", ""))
            if row_name not in HOCBF_ROWS:
                continue
            trial = int(_float_or_nan(raw.get("trial", "nan")))
            failure_mode, interpretation = _classify(raw)
            rows.append(
                {
                    "row": row_name,
                    "trial": trial,
                    "seed": int(_float_or_nan(raw.get("seed", "nan"))),
                    "completed_horizon": _flag(raw.get("completed_horizon", 0)),
                    "validation_passed": validation.get((row_name, trial), 0),
                    "detected": _flag(raw.get("detected", 0)),
                    "final_runtime_s": _metric(raw, "final_runtime_s"),
                    "final_coverage": _metric(raw, "final_coverage"),
                    "min_hocbf": _metric(raw, "min_hocbf"),
                    "min_control_authority_margin": _metric(raw, "min_control_authority_margin"),
                    "terminal_min_control_authority_margin": _metric(raw, "terminal_min_control_authority_margin"),
                    "joint_hocbf_feasible_ratio": _metric(raw, "joint_hocbf_feasible_ratio"),
                    "terminal_joint_hocbf_feasible_ratio": _metric(raw, "terminal_joint_hocbf_feasible_ratio"),
                    "terminal_hocbf_infeasible_robot_count": _metric(raw, "terminal_hocbf_infeasible_robot_count"),
                    "min_support_chain_margin_after_m": _metric(raw, "min_support_chain_margin_after"),
                    "terminal_support_chain_margin_after_m": _metric(raw, "terminal_support_chain_margin_after"),
                    "min_robust_margin_m": _metric(raw, "min_robust_margin"),
                    "failure_mode": failure_mode,
                    "interpretation": interpretation,
                }
            )
    return rows


def _min_or_nan(values: list[float]) -> float:
    finite = _finite(values)
    return min(finite) if finite else math.nan


def _median_or_nan(values: list[float]) -> float:
    finite = _finite(values)
    return median(finite) if finite else math.nan


def _mean_or_nan(values: list[float]) -> float:
    finite = _finite(values)
    return mean(finite) if finite else math.nan


def _max_or_nan(values: list[float]) -> float:
    finite = _finite(values)
    return max(finite) if finite else math.nan


def build_summary_rows(trial_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    summary: list[dict[str, Any]] = []
    keys = sorted({(row["row"], row["completed_horizon"]) for row in trial_rows})
    for row_name, completed in keys:
        group = [row for row in trial_rows if row["row"] == row_name and row["completed_horizon"] == completed]
        failure_counts = Counter(str(row["failure_mode"]) for row in group)
        summary.append(
            {
                "row": row_name,
                "completed_horizon": completed,
                "trial_count": len(group),
                "validation_pass_count": sum(int(row["validation_passed"]) for row in group),
                "detection_count": sum(int(row["detected"]) for row in group),
                "mean_final_coverage": _mean_or_nan([_float_or_nan(row["final_coverage"]) for row in group]),
                "min_hocbf_min": _min_or_nan([_float_or_nan(row["min_hocbf"]) for row in group]),
                "min_hocbf_median": _median_or_nan([_float_or_nan(row["min_hocbf"]) for row in group]),
                "min_control_authority_margin_min": _min_or_nan(
                    [_float_or_nan(row["min_control_authority_margin"]) for row in group]
                ),
                "min_control_authority_margin_median": _median_or_nan(
                    [_float_or_nan(row["min_control_authority_margin"]) for row in group]
                ),
                "terminal_min_control_authority_margin_min": _min_or_nan(
                    [_float_or_nan(row["terminal_min_control_authority_margin"]) for row in group]
                ),
                "terminal_joint_hocbf_feasible_ratio_min": _min_or_nan(
                    [_float_or_nan(row["terminal_joint_hocbf_feasible_ratio"]) for row in group]
                ),
                "terminal_hocbf_infeasible_robot_count_max": _max_or_nan(
                    [_float_or_nan(row["terminal_hocbf_infeasible_robot_count"]) for row in group]
                ),
                "min_support_chain_margin_after_m_min": _min_or_nan(
                    [_float_or_nan(row["min_support_chain_margin_after_m"]) for row in group]
                ),
                "terminal_support_chain_margin_after_m_min": _min_or_nan(
                    [_float_or_nan(row["terminal_support_chain_margin_after_m"]) for row in group]
                ),
                "min_robust_margin_m_min": _min_or_nan([_float_or_nan(row["min_robust_margin_m"]) for row in group]),
                "failure_mode_counts": ";".join(f"{name}:{count}" for name, count in sorted(failure_counts.items())),
            }
        )
    return summary


def write_csv(rows: list[dict[str, Any]], fields: list[str], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit SecondOrder task-interface boundary diagnostics.")
    parser.add_argument("--trials", type=Path, default=DEFAULT_TRIALS)
    parser.add_argument("--validation", type=Path, default=DEFAULT_VALIDATION)
    parser.add_argument("--trial-output", type=Path, default=DEFAULT_TRIAL_OUT)
    parser.add_argument("--summary-output", type=Path, default=DEFAULT_SUMMARY_OUT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trial_rows = build_trial_rows(args.trials, args.validation)
    summary_rows = build_summary_rows(trial_rows)
    write_csv(trial_rows, TRIAL_FIELDS, args.trial_output)
    write_csv(summary_rows, SUMMARY_FIELDS, args.summary_output)
    print(f"wrote {len(trial_rows)} trial rows to {args.trial_output}")
    print(f"wrote {len(summary_rows)} summary rows to {args.summary_output}")


if __name__ == "__main__":
    main()
