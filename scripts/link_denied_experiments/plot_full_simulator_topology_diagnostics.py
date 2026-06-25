#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import pathlib
import tempfile
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(pathlib.Path(tempfile.gettempdir()) / "cbf-matplotlib"))
pathlib.Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def _load_trial_rows(trials_csv: pathlib.Path, trial: int, rows: list[str]) -> dict[str, dict[str, Any]]:
    with trials_csv.open(newline="") as f:
        trial_rows = list(csv.DictReader(f))
    selected: dict[str, dict[str, Any]] = {}
    for row_name in rows:
        for row in trial_rows:
            if row.get("row") == row_name and int(float(row.get("trial", -1))) == trial:
                source = pathlib.Path(row["source_data"])
                selected[row_name] = json.loads(source.read_text(encoding="utf-8"))
                break
        if row_name not in selected:
            raise ValueError(f"Could not find row {row_name} trial {trial} in {trials_csv}")
    return selected


def _world_extent(data: dict[str, Any]) -> tuple[float, float, float, float]:
    boundary = data.get("config", {}).get("world", {}).get("boundary", [])
    xs = [float(point[0]) for point in boundary]
    ys = [float(point[1]) for point in boundary]
    return min(xs), max(xs), min(ys), max(ys)


def _denial_rectangle(data: dict[str, Any]) -> tuple[float, float, float, float] | None:
    topology = data.get("config", {}).get("bridge", {}).get("topology", {})
    center = topology.get("denial-center")
    half = topology.get("denial-half-size")
    if not center or not half:
        return None
    return (
        float(center[0]) - float(half[0]),
        float(center[1]) - float(half[1]),
        2.0 * float(half[0]),
        2.0 * float(half[1]),
    )


def _trajectories(data: dict[str, Any]) -> dict[int, list[tuple[float, float]]]:
    trajectories: dict[int, list[tuple[float, float]]] = {}
    for frame in data.get("state", []):
        for robot in frame.get("robots", []):
            state = robot.get("state", {})
            if "x" not in state or "y" not in state:
                continue
            robot_id = int(robot.get("id", len(trajectories) + 1))
            trajectories.setdefault(robot_id, []).append((float(state["x"]), float(state["y"])))
    return trajectories


def _series(data: dict[str, Any], key: str) -> tuple[list[float], list[float]]:
    times: list[float] = []
    values: list[float] = []
    for frame in data.get("state", []):
        topology = frame.get("bridge", {}).get("topology", {})
        value = topology.get(key)
        if value is None:
            continue
        times.append(float(frame.get("runtime", 0.0)))
        if isinstance(value, bool):
            values.append(1.0 if value else 0.0)
        else:
            values.append(float(value))
    return times, values


def _draw_map(ax: plt.Axes, data: dict[str, Any], title: str) -> None:
    extent = _world_extent(data)
    denial = _denial_rectangle(data)
    if denial is not None:
        ax.add_patch(
            Rectangle(
                (denial[0], denial[1]),
                denial[2],
                denial[3],
                facecolor="#d95f02",
                edgecolor="#d95f02",
                linewidth=1.0,
                alpha=0.16,
            )
        )
    metadata = data.get("bridge", {}).get("metadata", {})
    target = metadata.get("target", {})
    if target:
        ax.add_patch(
            Circle(
                (float(target.get("x", 0.0)), float(target.get("y", 0.0))),
                float(target.get("radius", 0.0)),
                fill=False,
                linewidth=1.1,
                edgecolor="#b2182b",
            )
        )
    bases = data.get("config", {}).get("bases", [])
    if bases:
        ax.scatter(
            [float(base[0]) for base in bases],
            [float(base[1]) for base in bases],
            marker="s",
            s=26,
            c="#111111",
            linewidths=0.0,
            label="base",
            zorder=5,
        )
    colors = ["#1b9e77", "#7570b3", "#e7298a", "#66a61e", "#e6ab02", "#a6761d"]
    for index, (robot_id, trajectory) in enumerate(sorted(_trajectories(data).items())):
        if len(trajectory) < 2:
            continue
        xs = [point[0] for point in trajectory]
        ys = [point[1] for point in trajectory]
        ax.plot(xs, ys, color=colors[index % len(colors)], linewidth=1.0, label=f"UAV {robot_id}")
        ax.scatter(xs[0], ys[0], marker="o", s=12, color=colors[index % len(colors)], linewidths=0.0)
        ax.scatter(xs[-1], ys[-1], marker="x", s=22, color=colors[index % len(colors)], linewidths=0.9)
    ax.set_title(title, fontsize=8)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", fontsize=7)
    ax.set_ylabel("y (m)", fontsize=7)
    ax.tick_params(labelsize=6, length=2)


def plot_diagnostics(
    data_by_row: dict[str, dict[str, Any]],
    output_prefix: pathlib.Path,
    map_title: str,
) -> list[pathlib.Path]:
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
    })
    colors = {
        "LD_FIXED": "#4d4d4d",
        "LD_RELAY": "#1b9e77",
        "LD_HOCBF": "#7570b3",
        "LD_HOCBF_GUARD": "#d95f02",
    }
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 4.7), constrained_layout=True)
    map_data = data_by_row.get("LD_RELAY", next(iter(data_by_row.values())))
    _draw_map(axes[0, 0], map_data, map_title)

    for row_name, data in data_by_row.items():
        times, values = _series(data, "min_robust_margin")
        axes[0, 1].plot(times, values, linewidth=1.2, color=colors.get(row_name), label=row_name.replace("LD_", ""))
    axes[0, 1].axhline(0.0, color="#b2182b", linewidth=0.8, linestyle="--")
    axes[0, 1].set_title("Robust topology margin", fontsize=8)
    axes[0, 1].set_ylabel("margin (m)", fontsize=7)

    for row_name, data in data_by_row.items():
        times, values = _series(data, "min_fim_eigenvalue")
        axes[1, 0].plot(times, values, linewidth=1.2, color=colors.get(row_name), label=row_name.replace("LD_", ""))
    axes[1, 0].set_title("FIM diagnostic", fontsize=8)
    axes[1, 0].set_xlabel("time (s)", fontsize=7)
    axes[1, 0].set_ylabel("min eigenvalue", fontsize=7)

    for row_name in [name for name in data_by_row if name != "LD_FIXED"]:
        data = data_by_row[row_name]
        times, relay_active = _series(data, "relay_active")
        axes[1, 1].step(times, relay_active, where="post", linewidth=1.1, color=colors.get(row_name), label=f"{row_name.replace('LD_', '')} relay")
    times, accepted = _series(map_data, "accepted_switches")
    _, rejected = _series(map_data, "rejected_candidates")
    axes[1, 1].plot(times, accepted, color="#1b9e77", linewidth=0.8, alpha=0.55, label="accepted candidates")
    axes[1, 1].plot(times, rejected, color="#b2182b", linewidth=0.8, alpha=0.55, label="rejected candidates")
    axes[1, 1].set_title("Relay selection and candidate counts", fontsize=8)
    axes[1, 1].set_xlabel("time (s)", fontsize=7)
    axes[1, 1].set_ylabel("state/count", fontsize=7)
    axes[1, 1].set_ylim(-0.08, 2.15)

    for ax in axes.flat:
        ax.grid(True, color="#e6e6e6", linewidth=0.45)
        ax.tick_params(labelsize=6, length=2)
    axes[0, 1].legend(fontsize=6, loc="best", frameon=False)
    axes[1, 0].legend(fontsize=6, loc="best", frameon=False)
    axes[1, 1].legend(fontsize=5.8, loc="best", frameon=False)

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    paths: list[pathlib.Path] = []
    for suffix in [".png", ".svg", ".pdf"]:
        path = output_prefix.with_suffix(suffix)
        fig.savefig(path, dpi=300)
        paths.append(path)
    plt.close(fig)
    return paths


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot trial-level LinkDenied full-simulator topology diagnostics")
    parser.add_argument("--trials-csv", type=pathlib.Path, default=pathlib.Path("papers/LinkDenied2026/figures/link_denied_full_bridge_trials.csv"))
    parser.add_argument("--trial", type=int, default=0)
    parser.add_argument("--rows", nargs="+", default=["LD_FIXED", "LD_RELAY", "LD_HOCBF"])
    parser.add_argument("--output-prefix", type=pathlib.Path, default=pathlib.Path("papers/LinkDenied2026/figures/link_denied_topology_diagnostics"))
    parser.add_argument("--map-title", default="Representative relay trajectory")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data_by_row = _load_trial_rows(args.trials_csv, args.trial, args.rows)
    for path in plot_diagnostics(data_by_row, args.output_prefix, args.map_title):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
