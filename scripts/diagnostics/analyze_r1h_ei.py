"""Analyze an estimator-in-the-loop R1H run against the truth-in-loop R1H run.

Consumes the simulator ``data.json`` plus the per-frame estimates log written
by ``run_r1h_ei.py`` and reports:

- search coverage time series and the first frame/time with 100% coverage;
- estimator tier counts (fresh / predicted / hold) and epsilon statistics;
- solver status counts, worst stacked hard-row residual, and route-1 chain
  latency;
- side-by-side comparison with a truth-in-loop R1H ``data.json``.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


TOTAL_CELLS = 90_000


def _coverage_series(frames: list[dict]) -> dict:
    covered: set[tuple[int, int]] = set()
    series: list[dict] = []
    first_full: dict | None = None
    for frame_index, frame in enumerate(frames):
        for cell in frame.get("update", []):
            if (
                isinstance(cell, list)
                and len(cell) >= 2
                and isinstance(cell[0], int)
                and isinstance(cell[1], int)
            ):
                covered.add((cell[0], cell[1]))
        fraction = len(covered) / TOTAL_CELLS
        series.append(
            {
                "frame_index": frame_index,
                "t": frame.get("runtime"),
                "unique_cells": len(covered),
                "fraction": round(fraction, 6),
            }
        )
        if first_full is None and len(covered) >= TOTAL_CELLS:
            first_full = series[-1]
    return {
        "series": series,
        "first_full": first_full,
        "final_fraction": series[-1]["fraction"] if series else None,
    }


def _feasibility(frames: list[dict]) -> dict:
    status_counts: dict[str, int] = {}
    worst_stacked = math.inf
    worst_stacked_at = None
    max_chain_latency = 0.0
    max_chain_latency_at = None
    applied_count = 0
    total_rows = 0
    for frame_index, frame in enumerate(frames):
        latency = frame.get("route1", {}).get("chain_latency_s", 0.0)
        if latency > max_chain_latency:
            max_chain_latency = latency
            max_chain_latency_at = frame_index
        for robot in frame.get("robots", []):
            opt = robot.get("opt", {})
            status = opt.get("status")
            status_counts[status if status is not None else "missing"] = (
                status_counts.get(status if status is not None else "missing", 0)
                + 1
            )
            hard_rows = opt.get("cbfNoSlack", [])
            total_rows += len(hard_rows)
            if hard_rows:
                stacked = sum(row.get("residual", 0.0) for row in hard_rows)
                if stacked < worst_stacked:
                    worst_stacked = stacked
                    worst_stacked_at = {
                        "frame_index": frame_index,
                        "robot": robot.get("id"),
                    }
            applied_count += 1
    return {
        "applied_records": applied_count,
        "hard_rows": total_rows,
        "solver_status_counts": status_counts,
        "worst_stacked_residual": worst_stacked,
        "worst_stacked_at": worst_stacked_at,
        "max_chain_latency_s": max_chain_latency,
        "max_chain_latency_at": max_chain_latency_at,
    }


def _estimates(path: Path | None) -> dict | None:
    if path is None or not path.exists():
        return None
    tier_counts: dict[str, int] = {}
    epsilons: dict[str, list[float]] = {}
    max_epsilon = 0.0
    max_epsilon_at = None
    per_frame: list[dict] = []
    for line in path.read_text().splitlines():
        frame = json.loads(line)
        tiers: dict[str, int] = {}
        frame_eps = []
        for robot in frame["robots"]:
            tier = robot["tier"]
            tier_counts[tier] = tier_counts.get(tier, 0) + 1
            tiers[tier] = tiers.get(tier, 0) + 1
            epsilon = robot["epsilon"]
            frame_eps.append(epsilon)
            epsilons.setdefault(tier, []).append(epsilon)
            if epsilon > max_epsilon:
                max_epsilon = epsilon
                max_epsilon_at = frame["frame_index"]
        per_frame.append(
            {
                "frame_index": frame["frame_index"],
                "tiers": tiers,
                "max_epsilon": max(frame_eps) if frame_eps else None,
            }
        )
    stats = {}
    for tier, values in epsilons.items():
        values.sort()
        stats[tier] = {
            "n": len(values),
            "mean": round(sum(values) / len(values), 4),
            "p50": round(values[len(values) // 2], 4),
            "p95": round(values[int(0.95 * len(values))], 4),
            "max": round(values[-1], 4),
        }
    total = sum(tier_counts.values())
    return {
        "tier_counts": tier_counts,
        "tier_fractions": {
            tier: round(count / total, 6) for tier, count in tier_counts.items()
        },
        "epsilon_stats": stats,
        "max_epsilon": max_epsilon,
        "max_epsilon_at_frame": max_epsilon_at,
        "per_frame": per_frame,
    }


def _load_frames(path: Path) -> list[dict]:
    data = json.loads(path.read_text())
    return data["state"]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data_path", type=Path)
    parser.add_argument(
        "--estimates-log", type=Path, default=None,
        help="estimates-log.jsonl written by run_r1h_ei.py",
    )
    parser.add_argument(
        "--truth-in-loop-data", type=Path, default=None,
        help="R1H truth-in-loop data.json for side-by-side comparison",
    )
    arguments = parser.parse_args(argv)
    frames = _load_frames(arguments.data_path)
    report = {
        "frames": len(frames),
        "coverage": _coverage_series(frames),
        "feasibility": _feasibility(frames),
        "estimates": _estimates(arguments.estimates_log),
    }
    if arguments.truth_in_loop_data is not None:
        truth_frames = _load_frames(arguments.truth_in_loop_data)
        report["truth_in_loop"] = {
            "coverage": _coverage_series(truth_frames),
            "feasibility": _feasibility(truth_frames),
        }
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
