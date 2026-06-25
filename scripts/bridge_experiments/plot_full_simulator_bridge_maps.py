#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
from matplotlib.colors import ListedColormap
from matplotlib.patches import Circle, Rectangle


def _world_extent(data: dict[str, Any]) -> tuple[float, float, float, float]:
    boundary = data.get("config", {}).get("world", {}).get("boundary", [])
    xs = [float(point[0]) for point in boundary]
    ys = [float(point[1]) for point in boundary]
    return min(xs), max(xs), min(ys), max(ys)


def _denial_rectangle_bounds(data: dict[str, Any]) -> tuple[float, float, float, float] | None:
    topology = data.get("config", {}).get("bridge", {}).get("topology", {})
    center = topology.get("denial-center")
    half_size = topology.get("denial-half-size")
    if not center or not half_size or len(center) < 2 or len(half_size) < 2:
        return None
    left = float(center[0]) - float(half_size[0])
    bottom = float(center[1]) - float(half_size[1])
    return left, bottom, 2.0 * float(half_size[0]), 2.0 * float(half_size[1])


def _coverage_matrix(search_map: list[list[dict[str, Any]]]) -> list[list[float]]:
    return [[1.0 if bool(cell.get("searched", False)) else 0.0 for cell in row] for row in search_map]


def _belief_matrix(search_map: list[list[dict[str, Any]]]) -> list[list[float]]:
    return [[float(cell.get("belief", 0.0)) for cell in row] for row in search_map]


def _robot_trajectories(data: dict[str, Any]) -> dict[int, list[tuple[float, float]]]:
    trajectories: dict[int, list[tuple[float, float]]] = {}
    for frame in data.get("state", []):
        for robot in frame.get("robots", []):
            state = robot.get("state", {})
            if "x" not in state or "y" not in state:
                continue
            robot_id = int(robot.get("id", len(trajectories) + 1))
            trajectories.setdefault(robot_id, []).append((float(state["x"]), float(state["y"])))
    return trajectories


def _final_robot_positions(data: dict[str, Any]) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for trajectory in _robot_trajectories(data).values():
        if trajectory:
            xs.append(trajectory[-1][0])
            ys.append(trajectory[-1][1])
    return xs, ys


def _coverage_ratio(search_map: list[list[dict[str, Any]]]) -> float:
    cells = [cell for row in search_map for cell in row]
    if not cells:
        return 0.0
    return sum(1 for cell in cells if bool(cell.get("searched", False))) / len(cells)


def _row_display_label(row: str) -> str:
    labels = {
        "LD_COMPLETION_FIXED": "Fixed topology",
        "LD_COMPLETION_RELAY": "Certified relay",
        "AS_COMPLETION_ACTIVE": "Active search",
        "AS_COMPLETION_FALLBACK": "Coverage fallback",
    }
    return labels.get(row, row.replace("_", " "))


def _draw_row_map(ax: plt.Axes, data: dict[str, Any]) -> None:
    metadata = data.get("bridge", {}).get("metadata", {})
    search_map = data.get("bridge", {}).get("final_search_map", [])
    if not search_map:
        raise ValueError("bridge.final_search_map is missing")

    extent = _world_extent(data)
    coverage = _coverage_matrix(search_map)
    belief = _belief_matrix(search_map)

    ax.imshow(
        coverage,
        origin="lower",
        extent=extent,
        interpolation="nearest",
        cmap=ListedColormap(["#f2f2f2", "#2f6f9f"]),
        vmin=0.0,
        vmax=1.0,
    )
    ax.imshow(
        belief,
        origin="lower",
        extent=extent,
        interpolation="nearest",
        cmap="YlOrRd",
        alpha=0.42,
    )

    target = metadata.get("target", {})
    if target:
        ax.add_patch(
            Circle(
                (float(target.get("x", 0.0)), float(target.get("y", 0.0))),
                float(target.get("radius", 0.0)),
                fill=False,
                linewidth=1.1,
                edgecolor="#b23a48",
            )
        )

    denial_bounds = _denial_rectangle_bounds(data)
    if denial_bounds is not None:
        ax.add_patch(
            Rectangle(
                (denial_bounds[0], denial_bounds[1]),
                denial_bounds[2],
                denial_bounds[3],
                fill=True,
                facecolor="#b23a48",
                edgecolor="#b23a48",
                linewidth=1.0,
                alpha=0.14,
                zorder=2,
            )
        )

    bases = data.get("config", {}).get("bases", [])
    if bases:
        ax.scatter(
            [float(base[0]) for base in bases],
            [float(base[1]) for base in bases],
            marker="s",
            s=18,
            c="#111111",
            linewidths=0.0,
        )

    colors = ["#1b4f72", "#d35400", "#117a65", "#7d3c98", "#6c3483", "#935116"]
    for index, (robot_id, trajectory) in enumerate(sorted(_robot_trajectories(data).items())):
        if len(trajectory) < 2:
            continue
        xs_traj = [point[0] for point in trajectory]
        ys_traj = [point[1] for point in trajectory]
        ax.plot(
            xs_traj,
            ys_traj,
            color=colors[index % len(colors)],
            linewidth=1.0,
            alpha=0.88,
            zorder=3,
        )

    xs, ys = _final_robot_positions(data)
    if xs:
        ax.scatter(xs, ys, s=16, c="#111111", linewidths=0.0)

    row = str(metadata.get("row", "row"))
    coverage_ratio = _coverage_ratio(search_map)
    ax.set_title(f"{_row_display_label(row)}: {coverage_ratio:.1%} searched", fontsize=7)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")
    ax.tick_params(labelsize=6, length=2)


def write_bridge_search_map_figure(data_rows: list[dict[str, Any]], output_prefix: pathlib.Path) -> list[pathlib.Path]:
    if not data_rows:
        raise ValueError("At least one bridge data row is required")

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 7,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
    })

    fig_width = max(3.2, 3.2 * len(data_rows))
    fig, axes = plt.subplots(1, len(data_rows), figsize=(fig_width, 2.6), constrained_layout=True)
    if len(data_rows) == 1:
        axes = [axes]

    for index, (ax, data) in enumerate(zip(axes, data_rows)):
        _draw_row_map(ax, data)
        ax.set_xlabel("x (m)")
        if index == 0:
            ax.set_ylabel("y (m)")
        else:
            ax.set_ylabel("")

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    outputs = [
        output_prefix.with_suffix(".png"),
        output_prefix.with_suffix(".svg"),
        output_prefix.with_suffix(".pdf"),
    ]
    fig.savefig(outputs[0], dpi=450, bbox_inches="tight")
    fig.savefig(outputs[1], bbox_inches="tight")
    fig.savefig(outputs[2], bbox_inches="tight")
    plt.close(fig)
    return outputs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot final full-simulator bridge search maps")
    parser.add_argument("data_json", nargs="+", type=pathlib.Path)
    parser.add_argument("--output-prefix", type=pathlib.Path, default=pathlib.Path("papers/bridge_search_maps"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = [json.loads(path.read_text(encoding="utf-8")) for path in args.data_json]
    outputs = write_bridge_search_map_figure(rows, args.output_prefix)
    for output in outputs:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
