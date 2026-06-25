#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import pathlib
import sys
import tempfile
from dataclasses import dataclass
from typing import Any

SCRIPT_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPT_ROOT))

os.environ.setdefault("MPLCONFIGDIR", str(pathlib.Path(tempfile.gettempdir()) / "cbf-matplotlib"))
pathlib.Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

from bridge_experiments.extract_full_simulator_bridge import (
    _hocbf_acceleration_constraint,
    _hocbf_control_authority_margin,
)


@dataclass(frozen=True)
class HOCBFFeasibilityPanel:
    label: str
    row: str
    trial: int | None
    runtime_s: float
    robot_id: int
    feasible: bool
    constraints: list[tuple[float, float, float]]
    polygon: list[tuple[float, float]]
    acceleration_bound: float
    min_control_authority_margin: float


def _inside_halfspace(point: tuple[float, float], constraint: tuple[float, float, float]) -> bool:
    x, y = point
    ax, ay, rhs = constraint
    return ax * x + ay * y >= rhs - 1.0e-9


def _line_intersection(
    start: tuple[float, float],
    end: tuple[float, float],
    constraint: tuple[float, float, float],
) -> tuple[float, float]:
    sx, sy = start
    ex, ey = end
    ax, ay, rhs = constraint
    denominator = ax * (ex - sx) + ay * (ey - sy)
    if abs(denominator) <= 1.0e-12:
        return end
    t = (rhs - ax * sx - ay * sy) / denominator
    t = min(1.0, max(0.0, t))
    return sx + t * (ex - sx), sy + t * (ey - sy)


def _deduplicate_polygon(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    cleaned: list[tuple[float, float]] = []
    for point in points:
        if not cleaned or math.hypot(point[0] - cleaned[-1][0], point[1] - cleaned[-1][1]) > 1.0e-8:
            cleaned.append(point)
    if len(cleaned) > 1 and math.hypot(cleaned[0][0] - cleaned[-1][0], cleaned[0][1] - cleaned[-1][1]) <= 1.0e-8:
        cleaned.pop()
    return cleaned


def feasible_acceleration_polygon(
    constraints: list[tuple[float, float, float]],
    acceleration_bound: float,
) -> list[tuple[float, float]]:
    if not math.isfinite(acceleration_bound) or acceleration_bound < 0.0:
        return []
    bound = float(acceleration_bound)
    polygon = [(-bound, -bound), (bound, -bound), (bound, bound), (-bound, bound)]
    for constraint in constraints:
        if not all(math.isfinite(value) for value in constraint):
            continue
        clipped: list[tuple[float, float]] = []
        if not polygon:
            return []
        previous = polygon[-1]
        previous_inside = _inside_halfspace(previous, constraint)
        for current in polygon:
            current_inside = _inside_halfspace(current, constraint)
            if current_inside:
                if not previous_inside:
                    clipped.append(_line_intersection(previous, current, constraint))
                clipped.append(current)
            elif previous_inside:
                clipped.append(_line_intersection(previous, current, constraint))
            previous = current
            previous_inside = current_inside
        polygon = _deduplicate_polygon(clipped)
    return polygon if len(polygon) >= 3 else []


def _polygon_area(points: list[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    twice_area = 0.0
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        twice_area += x1 * y2 - x2 * y1
    return abs(twice_area) * 0.5


def _acceleration_bound(data: dict[str, Any]) -> float:
    high_order = data.get("config", {}).get("cbfs", {}).get("high-order", {})
    return float(high_order.get("acceleration-bound", math.nan))


def _trial_from_metadata(data: dict[str, Any]) -> int | None:
    metadata = data.get("bridge", {}).get("metadata", {})
    for key in ("trial", "trial_id"):
        if key in metadata:
            return int(float(metadata[key]))
    return None


def _robot_panel_candidates(data: dict[str, Any], label: str) -> list[HOCBFFeasibilityPanel]:
    frames = data.get("state", [])
    if not frames:
        return []
    terminal = frames[-1]
    metadata = data.get("bridge", {}).get("metadata", {})
    row = str(metadata.get("row", "HOCBF"))
    bound = _acceleration_bound(data)
    runtime_s = float(terminal.get("runtime", math.nan))
    candidates: list[HOCBFFeasibilityPanel] = []
    for robot in terminal.get("robots", []):
        opt = robot.get("opt", {}) or {}
        items = opt.get("hocbfNoSlack", [])
        constraints = [
            constraint
            for item in items
            if (constraint := _hocbf_acceleration_constraint(item)) is not None
        ]
        if not constraints:
            continue
        polygon = feasible_acceleration_polygon(constraints, bound)
        margins = [
            _hocbf_control_authority_margin(item, bound)
            for item in items
        ]
        finite_margins = [value for value in margins if math.isfinite(value)]
        candidates.append(
            HOCBFFeasibilityPanel(
                label=label,
                row=row,
                trial=_trial_from_metadata(data),
                runtime_s=runtime_s,
                robot_id=int(robot.get("id", len(candidates) + 1)),
                feasible=bool(polygon),
                constraints=constraints,
                polygon=polygon,
                acceleration_bound=bound,
                min_control_authority_margin=min(finite_margins) if finite_margins else math.nan,
            )
        )
    return candidates


def terminal_hocbf_panels(
    completed_data: dict[str, Any],
    early_data: dict[str, Any],
) -> list[HOCBFFeasibilityPanel]:
    completed_candidates = [panel for panel in _robot_panel_candidates(completed_data, "completed") if panel.feasible]
    if not completed_candidates:
        raise ValueError("No terminal feasible HOCBF robot found in completed_data")
    completed_panel = min(
        completed_candidates,
        key=lambda panel: (_polygon_area(panel.polygon), panel.min_control_authority_margin, panel.robot_id),
    )

    early_candidates = _robot_panel_candidates(early_data, "early termination")
    infeasible_candidates = [panel for panel in early_candidates if not panel.feasible]
    if not infeasible_candidates:
        raise ValueError("No terminal infeasible HOCBF robot found in early_data")
    early_panel = min(
        infeasible_candidates,
        key=lambda panel: (panel.min_control_authority_margin, panel.robot_id),
    )
    return [completed_panel, early_panel]


def _terminal_ratio_rows(trials_csv: pathlib.Path, row_label: str) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with trials_csv.open(newline="", encoding="utf-8") as handle:
        for record in csv.DictReader(handle):
            if str(record.get("row", "")) != row_label:
                continue
            try:
                rows.append({
                    "trial": float(record.get("trial", math.nan)),
                    "completed": float(record.get("completed_horizon", math.nan)),
                    "ratio": float(record.get("terminal_joint_hocbf_feasible_ratio", math.nan)),
                    "infeasible_count": float(record.get("terminal_hocbf_infeasible_robot_count", math.nan)),
                })
            except (TypeError, ValueError):
                continue
    return rows


def _draw_constraint_lines(ax: plt.Axes, panel: HOCBFFeasibilityPanel) -> None:
    bound = panel.acceleration_bound
    xs = [-bound, bound]
    for cx, cy, rhs in panel.constraints:
        color = "#59656f" if panel.feasible else "#8c4b4b"
        if abs(cy) > 1.0e-12:
            ys = [(rhs - cx * x) / cy for x in xs]
            ax.plot(xs, ys, color=color, linewidth=0.9, alpha=0.76)
        elif abs(cx) > 1.0e-12:
            x = rhs / cx
            ax.plot([x, x], [-bound, bound], color=color, linewidth=0.9, alpha=0.76)


def _draw_feasibility_panel(ax: plt.Axes, panel: HOCBFFeasibilityPanel) -> None:
    bound = panel.acceleration_bound
    ax.add_patch(
        Rectangle(
            (-bound, -bound),
            2.0 * bound,
            2.0 * bound,
            fill=False,
            edgecolor="#202020",
            linewidth=1.0,
        )
    )
    _draw_constraint_lines(ax, panel)
    if panel.polygon:
        ax.add_patch(
            Polygon(
                panel.polygon,
                closed=True,
                facecolor="#2f6f9f",
                edgecolor="#1b4f72",
                alpha=0.26,
                linewidth=1.0,
            )
        )
        centroid_x = sum(x for x, _ in panel.polygon) / len(panel.polygon)
        centroid_y = sum(y for _, y in panel.polygon) / len(panel.polygon)
        ax.text(
            centroid_x,
            centroid_y,
            "feasible\nset",
            ha="center",
            va="center",
            fontsize=7,
            color="#1b4f72",
        )
    else:
        ax.text(
            0.5,
            0.5,
            "empty\nintersection",
            transform=ax.transAxes,
            ha="center",
            va="center",
            fontsize=8,
            color="#b23a48",
        )
    status = "nonempty" if panel.feasible else "empty"
    ax.set_title(
        f"{panel.label}\nrobot {panel.robot_id}, t={panel.runtime_s:.1f}s, {status}",
        fontsize=8,
    )
    ax.set_xlim(-bound * 1.08, bound * 1.08)
    ax.set_ylim(-bound * 1.08, bound * 1.08)
    ax.set_aspect("equal")
    ax.set_xlabel("$a_x$ (m/s$^2$)")
    ax.set_ylabel("$a_y$ (m/s$^2$)")
    ax.tick_params(labelsize=6, length=2)
    ax.axhline(0.0, color="#dddddd", linewidth=0.6, zorder=0)
    ax.axvline(0.0, color="#dddddd", linewidth=0.6, zorder=0)


def _draw_terminal_ratio_panel(ax: plt.Axes, rows: list[dict[str, float]]) -> None:
    completed = [row for row in rows if row["completed"] >= 0.5 and math.isfinite(row["ratio"])]
    early = [row for row in rows if row["completed"] < 0.5 and math.isfinite(row["ratio"])]
    ax.scatter(
        [row["trial"] for row in completed],
        [row["ratio"] for row in completed],
        s=24,
        color="#2f6f9f",
        label="completed",
    )
    ax.scatter(
        [row["trial"] for row in early],
        [row["ratio"] for row in early],
        s=28,
        marker="x",
        color="#b23a48",
        label="early stop",
    )
    ax.set_ylim(0.68, 1.03)
    ax.set_xlabel("trial")
    ax.set_ylabel("terminal feasible ratio")
    ax.set_title("12-trial terminal check", fontsize=8)
    ax.tick_params(labelsize=6, length=2)
    ax.grid(True, axis="y", color="#e6e6e6", linewidth=0.6)
    ax.legend(loc="lower left", fontsize=6)


def write_hocbf_feasibility_figure(
    completed_data: dict[str, Any],
    early_data: dict[str, Any],
    trials_csv: pathlib.Path,
    output_prefix: pathlib.Path,
) -> list[pathlib.Path]:
    panels = terminal_hocbf_panels(completed_data, early_data)
    ratio_rows = _terminal_ratio_rows(trials_csv, panels[0].row)
    if not ratio_rows:
        raise ValueError(f"No rows for {panels[0].row!r} in {trials_csv}")

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 7,
        "axes.spines.right": False,
        "axes.spines.top": False,
        "axes.linewidth": 0.8,
        "legend.frameon": False,
    })

    fig, axes = plt.subplots(1, 3, figsize=(7.35, 2.35), constrained_layout=True)
    _draw_feasibility_panel(axes[0], panels[0])
    _draw_feasibility_panel(axes[1], panels[1])
    _draw_terminal_ratio_panel(axes[2], ratio_rows)

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
    parser = argparse.ArgumentParser(description="Plot HOCBF acceleration feasibility mechanism")
    parser.add_argument("completed_data_json", type=pathlib.Path)
    parser.add_argument("early_data_json", type=pathlib.Path)
    parser.add_argument("--trials-csv", type=pathlib.Path, required=True)
    parser.add_argument("--output-prefix", type=pathlib.Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    completed_data = json.loads(args.completed_data_json.read_text(encoding="utf-8"))
    early_data = json.loads(args.early_data_json.read_text(encoding="utf-8"))
    outputs = write_hocbf_feasibility_figure(
        completed_data,
        early_data,
        args.trials_csv,
        args.output_prefix,
    )
    for output in outputs:
        print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
