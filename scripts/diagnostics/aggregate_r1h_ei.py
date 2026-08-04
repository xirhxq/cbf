"""Aggregate R1H-EI multi-seed runs against truth-in-loop R1H."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


TOTAL_CELLS = 90_000


def _frames(data_path: Path) -> list[dict]:
    return json.loads(data_path.read_text())["state"]


def _coverage_series(frames: list[dict]) -> tuple[list[float], dict | None]:
    covered: set[tuple[int, int]] = set()
    series = []
    first_full = None
    for frame_index, frame in enumerate(frames):
        for cell in frame.get("update", []):
            if (
                isinstance(cell, list)
                and len(cell) >= 2
                and isinstance(cell[0], int)
                and isinstance(cell[1], int)
            ):
                covered.add((cell[0], cell[1]))
        series.append(len(covered) / TOTAL_CELLS)
        if first_full is None and len(covered) >= TOTAL_CELLS:
            first_full = {
                "frame_index": frame_index,
                "t": frame.get("runtime"),
            }
    return series, first_full


def _estimates(estimates_log: Path) -> dict:
    tier_counts: dict[str, int] = {}
    epsilons: dict[str, list[float]] = {}
    per_frame = []
    for line in estimates_log.read_text().splitlines():
        frame = json.loads(line)
        frame_eps = []
        for robot in frame["robots"]:
            tier_counts[robot["tier"]] = tier_counts.get(robot["tier"], 0) + 1
            frame_eps.append(robot["epsilon"])
            epsilons.setdefault(robot["tier"], []).append(robot["epsilon"])
        per_frame.append(
            {
                "frame_index": frame["frame_index"],
                "max_epsilon": max(frame_eps),
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
        "per_frame": per_frame,
    }


def _errors(frames: list[dict], estimates_log: Path) -> dict:
    truth_by_frame = {
        int(frame["runtime"] / 0.5): {
            robot["id"]: (robot["state"]["x"], robot["state"]["y"])
            for robot in frame["robots"]
        }
        for frame in frames
    }
    errors = []
    epsilons = []
    contained = []
    for line in estimates_log.read_text().splitlines():
        frame = json.loads(line)
        truth = truth_by_frame[frame["frame_index"]]
        for robot in frame["robots"]:
            error = float(
                np.linalg.norm(
                    np.asarray(robot["estimate"])
                    - np.asarray(truth[robot["id"]])
                )
            )
            errors.append(error)
            epsilons.append(robot["epsilon"])
            contained.append(error <= robot["epsilon"])
    errors = np.asarray(errors)
    epsilons = np.asarray(epsilons)
    contained = np.asarray(contained)
    return {
        "n": int(len(errors)),
        "error_p50": round(float(np.percentile(errors, 50)), 4),
        "error_p95": round(float(np.percentile(errors, 95)), 4),
        "error_max": round(float(errors.max()), 4),
        "error_mean": round(float(errors.mean()), 4),
        "containment": round(float(contained.mean()), 6),
        "containment_3sigma": round(
            float((errors <= 3.0 * epsilons).mean()), 6
        ),
        "correlation": round(float(np.corrcoef(errors, epsilons)[0, 1]), 4),
    }


def _feasibility(frames: list[dict]) -> dict:
    status_counts: dict[str, int] = {}
    worst = math.inf
    worst_at = None
    max_latency = 0.0
    for frame_index, frame in enumerate(frames):
        latency = frame.get("route1", {}).get("chain_latency_s", 0.0)
        max_latency = max(max_latency, latency)
        for robot in frame.get("robots", []):
            opt = robot.get("opt", {})
            status = opt.get("status", "missing")
            status_counts[status] = status_counts.get(status, 0) + 1
            rows = opt.get("cbfNoSlack", [])
            if rows:
                stacked = sum(row.get("residual", 0.0) for row in rows)
                if stacked < worst:
                    worst = stacked
                    worst_at = {
                        "frame_index": frame_index,
                        "robot": robot.get("id"),
                    }
    return {
        "solver_status_counts": status_counts,
        "worst_stacked_residual": worst,
        "worst_stacked_at": worst_at,
        "max_chain_latency_s": max_latency,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ei-run", action="append", required=True, metavar="ROOT",
        help="EI run root containing estimates-log.jsonl and a */data.json",
    )
    parser.add_argument(
        "--truth-data", action="append", default=[], metavar="PATH",
        help="truth-in-loop R1H data.json for the same seed",
    )
    parser.add_argument(
        "--out-dir", type=Path, default=Path("/private/tmp/r1h-figs"),
    )
    arguments = parser.parse_args(argv)

    runs = []
    for run_root in map(Path, arguments.ei_run):
        data_path = sorted(run_root.glob("*/data.json"))
        if not data_path:
            raise SystemExit(f"no data.json under {run_root}")
        frames = _frames(data_path[-1])
        estimates_log = run_root / "estimates-log.jsonl"
        coverage, first_full = _coverage_series(frames)
        runs.append(
            {
                "root": str(run_root),
                "seed": run_root.name,
                "frames": len(frames),
                "coverage_first_full": first_full,
                "coverage_final": coverage[-1],
                "coverage_series": coverage,
                "estimates": _estimates(estimates_log),
                "errors": _errors(frames, estimates_log),
                "feasibility": _feasibility(frames),
            }
        )

    truths = []
    for truth_path in map(Path, arguments.truth_data):
        frames = _frames(truth_path)
        coverage, first_full = _coverage_series(frames)
        truths.append(
            {
                "path": str(truth_path),
                "seed": truth_path.parent.name.split("_seed_")[-1].split("_")[0],
                "coverage_first_full": first_full,
                "coverage_final": coverage[-1],
                "coverage_series": coverage,
                "feasibility": _feasibility(frames),
            }
        )

    report = {
        "total_cells": TOTAL_CELLS,
        "ei_runs": [
            {key: value for key, value in run.items() if key != "coverage_series"}
            for run in runs
        ],
        "truth_runs": [
            {key: value for key, value in truth.items() if key != "coverage_series"}
            for truth in truths
        ],
    }
    out = arguments.out_dir
    out.mkdir(parents=True, exist_ok=True)
    (out / "ei-multiseed-summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    )

    t_truth = np.arange(len(truths[0]["coverage_series"])) * 0.5 if truths else np.array([])
    fig, axes = plt.subplots(1, 3, figsize=(17, 4.6))
    for truth in truths:
        axes[0].plot(
            t_truth, np.asarray(truth["coverage_series"]) * 100,
            lw=1.1, alpha=0.65,
            color="tab:blue", label=f"R1H {truth['seed']}",
        )
    for run in runs:
        t_run = np.arange(len(run["coverage_series"])) * 0.5
        axes[0].plot(
            t_run, np.asarray(run["coverage_series"]) * 100, lw=1.4,
            label=f"R1H-EI {run['seed']}",
        )
    axes[0].axhline(100, color="gray", ls=":", lw=1)
    axes[0].set_xlabel("t (s)")
    axes[0].set_ylabel("coverage (%)")
    axes[0].set_title("Coverage: EI vs truth-in-loop")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    for run in runs:
        axes[1].plot(
            np.arange(len(run["estimates"]["per_frame"])) * 0.5,
            [entry["max_epsilon"] for entry in run["estimates"]["per_frame"]],
            lw=1.1, label=f"{run['seed']}",
        )
    axes[1].axhline(30, color="gray", ls=":", lw=1)
    axes[1].set_xlabel("t (s)")
    axes[1].set_ylabel("max epsilon (m)")
    axes[1].set_title("Per-frame max estimator epsilon")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(runs), 1)))
    for run, color in zip(runs, colors):
        frames = _frames(sorted(Path(run["root"]).glob("*/data.json"))[-1])
        truth_by_frame = {
            int(frame["runtime"] / 0.5): {
                robot["id"]: (robot["state"]["x"], robot["state"]["y"])
                for robot in frame["robots"]
            }
            for frame in frames
        }
        errors = []
        epsilons = []
        for line in (Path(run["root"]) / "estimates-log.jsonl").read_text().splitlines():
            frame = json.loads(line)
            truth = truth_by_frame[frame["frame_index"]]
            for robot in frame["robots"]:
                errors.append(
                    float(
                        np.linalg.norm(
                            np.asarray(robot["estimate"])
                            - np.asarray(truth[robot["id"]])
                        )
                    )
                )
                epsilons.append(robot["epsilon"])
        axes[2].scatter(
            errors, epsilons, s=4, alpha=0.22, color=color,
            label=run["seed"],
        )
    lim = axes[2].get_xlim()[1]
    axes[2].plot([0, lim], [0, lim], color="gray", ls=":", lw=1)
    axes[2].set_xlabel("|estimate - truth| (m)")
    axes[2].set_ylabel("epsilon (m)")
    axes[2].set_title("Error vs epsilon (containment = below line)")
    axes[2].legend(fontsize=8, markerscale=4)
    axes[2].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "ei-multiseed-summary.png", dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
