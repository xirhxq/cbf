#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:
    plt = None


DEFAULT_MANIFEST = Path("data/second_order_active_task_reserve_24/source_data_manifest.csv")
DEFAULT_OUT_DIR = Path("data/second_order_active_task_reserve_24")
FIG_DIR_DEFAULT = Path("papers/SecondOrderCBF2026/assets")
DEFAULT_FAILING = "20261224,20261229"
DEFAULT_CONTROL = "20261230"
ACCEL_HALF_BOX = 8.0
EPSILON = 1e-9


def _parse_seeds(raw: str) -> list[int]:
    tokens = [token.strip() for token in raw.split(",") if token.strip()]
    return [int(token) for token in tokens]


def _manifest_rows(manifest: Path) -> list[dict[str, str]]:
    with manifest.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _select_trial(rows: list[dict[str, str]], seed: int) -> dict[str, str]:
    for row in rows:
        if row.get("row") == "AS_HOCBF_TASK_RESERVE" and int(float(row.get("seed", "nan"))) == seed:
            return row
    raise KeyError(f"AS_HOCBF_TASK_RESERVE row for seed {seed} not found in manifest")


def _load_samples(source_field: str) -> list[dict[str, Any]]:
    data_path = Path(source_field)
    if not data_path.exists():
        raise FileNotFoundError(data_path)
    with data_path.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    return payload["state"]


def _half_spaces(robot: dict[str, Any]) -> list[dict[str, Any]]:
    opt = robot.get("opt") or {}
    edges = []
    for entry in opt.get("hocbfNoSlack", []):
        coe = entry.get("coe", {})
        ax = float(coe.get("ax", 0.0))
        ay = float(coe.get("ay", 0.0))
        const = float(entry.get("const", 0.0))
        reserve = float(entry.get("sampledDataReserve", 0.0))
        h_value = float(entry.get("h", math.nan))
        hdot = float(entry.get("hdot", math.nan))
        hocbf_val = float(entry.get("hocbf", math.nan))
        name = str(entry.get("name", ""))
        edges.append(
            {
                "name": name,
                "coe": np.array([ax, ay], dtype=float),
                "const": const,
                "reserve": reserve,
                "h": h_value,
                "hdot": hdot,
                "hocbf": hocbf_val,
            }
        )
    return edges


def _box_vertices(half: float) -> np.ndarray:
    return np.array(
        [
            [-half, -half],
            [half, -half],
            [-half, half],
            [half, half],
        ],
        dtype=float,
    )


def _residual(edge: dict[str, Any], accel: np.ndarray) -> float:
    return float(edge["const"] - np.dot(edge["coe"], accel))


def gamma_star_value(edges: list[dict[str, Any]]) -> float:
    if not edges:
        return math.inf
    vertices = _box_vertices(ACCEL_HALF_BOX)
    best = -math.inf
    for vertex in vertices:
        per_edge = [_residual(edge, vertex) for edge in edges]
        candidate = min(per_edge)
        if candidate > best:
            best = candidate
    best_vertex_value = best
    improved = _refine_gamma_star(edges)
    return max(best_vertex_value, improved)


def _refine_gamma_star(edges: list[dict[str, Any]]) -> float:
    if plt is None or len(edges) <= 1:
        return -math.inf
    coes = np.array([edge["coe"] for edge in edges])
    consts = np.array([edge["const"] for edge in edges])
    step = ACCEL_HALF_BOX / 200.0
    grid_axis = np.arange(-ACCEL_HALF_BOX, ACCEL_HALF_BOX + step, step)
    gx, gy = np.meshgrid(grid_axis, grid_axis, indexing="ij")
    flat = np.stack([gx.ravel(), gy.ravel()], axis=1)
    slacks = consts[None, :] - flat @ coes.T
    row_min = slacks.min(axis=1)
    return float(row_min.max())


def dominant_edge(edges: list[dict[str, Any]], accel: np.ndarray) -> str:
    if not edges:
        return ""
    worst = None
    worst_value = math.inf
    for edge in edges:
        value = _residual(edge, accel)
        if value < worst_value:
            worst_value = value
            worst = edge
    return worst["name"] if worst else ""


def gamma_star_accelerations(edges: list[dict[str, Any]]) -> dict[str, Any]:
    if not edges:
        return {"gamma_star": math.inf, "accel": (0.0, 0.0), "dominant": ""}
    vertices = _box_vertices(ACCEL_HALF_BOX)
    best_gamma = -math.inf
    best_vertex = vertices[0]
    for vertex in vertices:
        candidate = min(_residual(edge, vertex) for edge in edges)
        if candidate > best_gamma:
            best_gamma = candidate
            best_vertex = vertex
    refined_gamma = _refine_gamma_star(edges)
    if refined_gamma > best_gamma:
        best_gamma = refined_gamma
        grid_axis = np.arange(-ACCEL_HALF_BOX, ACCEL_HALF_BOX + ACCEL_HALF_BOX / 200.0, ACCEL_HALF_BOX / 200.0)
        gx, gy = np.meshgrid(grid_axis, grid_axis, indexing="ij")
        flat = np.stack([gx.ravel(), gy.ravel()], axis=1)
        coes = np.array([edge["coe"] for edge in edges])
        consts = np.array([edge["const"] for edge in edges])
        slacks = consts[None, :] - flat @ coes.T
        idx = int(slacks.min(axis=1).argmax())
        best_vertex = flat[idx]
    dom = dominant_edge(edges, best_vertex)
    return {"gamma_star": best_gamma, "accel": (float(best_vertex[0]), float(best_vertex[1])), "dominant": dom}


def _control_bounds(robot: dict[str, Any]) -> float:
    opt = robot.get("opt") or {}
    bounds = opt.get("controlBounds", [])
    if not bounds:
        return ACCEL_HALF_BOX
    upper_values = [
        float(b.get("rhs", ACCEL_HALF_BOX))
        for b in bounds
        if "upper" in str(b.get("name", "")).lower()
    ]
    return min(upper_values) if upper_values else ACCEL_HALF_BOX


def _relative_geometry(robot_i: dict[str, Any], robot_j: dict[str, Any]) -> dict[str, float]:
    si = robot_i.get("state", {})
    sj = robot_j.get("state", {})
    xi, yi = float(si.get("x", 0.0)), float(si.get("y", 0.0))
    xj, yj = float(sj.get("x", 0.0)), float(sj.get("y", 0.0))
    vxi, vyi = float(si.get("vx", 0.0)), float(si.get("vy", 0.0))
    vxj, vyj = float(sj.get("vx", 0.0)), float(sj.get("vy", 0.0))
    dx = xi - xj
    dy = yi - yj
    dist = math.hypot(dx, dy) or math.nan
    if dist == dist and dist > EPSILON:
        n_x, n_y = dx / dist, dy / dist
    else:
        n_x, n_y = math.nan, math.nan
    rvx, rvy = vxi - vxj, vyi - vyj
    radial = (rvx * n_x + rvy * n_y) if math.isfinite(n_x) else math.nan
    speed = math.hypot(rvx, rvy)
    return {
        "distance_m": dist,
        "normal_x": n_x,
        "normal_y": n_y,
        "rel_vx_ms": rvx,
        "rel_vy_ms": rvy,
        "radial_speed_ms": radial,
        "rel_speed_ms": speed,
    }


def _neighbour_id_from_edge_name(name: str, robot_id: int) -> str:
    inside = ""
    if "#" in name:
        inside = name.split("#", 1)[1].split(")", 1)[0]
    elif "base" in name:
        inside = name.split("base", 1)[1].strip("- ")
    return inside if inside else "?"


def _robot_by_id(sample: dict[str, Any], label: str) -> dict[str, Any] | None:
    for robot in sample.get("robots", []):
        if str(robot.get("id")) == str(label):
            return robot
    return None


def analyse_trial(trial_row: dict[str, str], seed: int, label: str) -> dict[str, Any]:
    samples = _load_samples(trial_row["source_data"])
    per_sample_rows: list[dict[str, Any]] = []
    first_crossings: dict[int, dict[str, Any]] = {}

    for index, sample in enumerate(samples):
        runtime = float(sample.get("runtime", math.nan))
        for robot in sample.get("robots", []):
            robot_id = int(robot.get("id", -1))
            edges = _half_spaces(robot)
            if not edges:
                continue
            result = gamma_star_accelerations(edges)
            gamma_val = result["gamma_star"]
            half_box = _control_bounds(robot)
            dom_name = result["dominant"]
            neighbour_label = _neighbour_id_from_edge_name(dom_name, robot_id)
            neighbour = _robot_by_id(sample, neighbour_label)
            geom = _relative_geometry(robot, neighbour) if neighbour else {
                "distance_m": math.nan,
                "normal_x": math.nan,
                "normal_y": math.nan,
                "rel_vx_ms": math.nan,
                "rel_vy_ms": math.nan,
                "radial_speed_ms": math.nan,
                "rel_speed_ms": math.nan,
            }
            crossing_flag = 1 if gamma_val < -EPSILON else 0
            if crossing_flag and robot_id not in first_crossings:
                first_crossings[robot_id] = {
                    "runtime_s": runtime,
                    "sample_index": index,
                    "gamma_star": gamma_val,
                    "dominant_edge": dom_name,
                    "neighbour_id": neighbour_label,
                    **geom,
                }
            per_sample_rows.append(
                {
                    "seed": seed,
                    "trial": label,
                    "sample_index": index,
                    "runtime_s": runtime,
                    "robot_id": robot_id,
                    "gamma_star_m_per_s2": gamma_val,
                    "crossing_flag": crossing_flag,
                    "active_edge_count": len(edges),
                    "dominant_edge": dom_name,
                    "dominant_neighbour_id": neighbour_label,
                    "dominant_edge_const": next(
                        (e["const"] for e in edges if e["name"] == dom_name), math.nan
                    ),
                    "dominant_edge_h": next(
                        (e["h"] for e in edges if e["name"] == dom_name), math.nan
                    ),
                    "dominant_edge_hdot": next(
                        (e["hdot"] for e in edges if e["name"] == dom_name), math.nan
                    ),
                    "dominant_edge_reserve": next(
                        (e["reserve"] for e in edges if e["name"] == dom_name), math.nan
                    ),
                    "accel_half_box_m_per_s2": half_box,
                    "robot_x_m": float(robot.get("state", {}).get("x", math.nan)),
                    "robot_y_m": float(robot.get("state", {}).get("y", math.nan)),
                    "robot_vx_ms": float(robot.get("state", {}).get("vx", math.nan)),
                    "robot_vy_ms": float(robot.get("state", {}).get("vy", math.nan)),
                    "pair_distance_m": geom["distance_m"],
                    "pair_radial_speed_ms": geom["radial_speed_ms"],
                    "pair_rel_speed_ms": geom["rel_speed_ms"],
                    "pair_normal_x": geom["normal_x"],
                    "pair_normal_y": geom["normal_y"],
                }
            )

    terminal_gamma: dict[int, float] = {}
    if per_sample_rows:
        last_index = per_sample_rows[-1]["sample_index"]
        for row in per_sample_rows:
            if row["sample_index"] == last_index:
                terminal_gamma[row["robot_id"]] = row["gamma_star_m_per_s2"]

    return {
        "seed": seed,
        "label": label,
        "completed_horizon": int(float(trial_row.get("completed_horizon", "0"))),
        "n_samples": len(samples),
        "per_sample_rows": per_sample_rows,
        "first_crossings": first_crossings,
        "terminal_gamma": terminal_gamma,
        "min_gamma_overall": (
            min((row["gamma_star_m_per_s2"] for row in per_sample_rows), default=math.nan)
        ),
    }


def write_per_sample_csv(rows: list[dict[str, Any]], path: Path) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_crossing_summary(results: list[dict[str, Any]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "seed",
        "trial_label",
        "completed_horizon",
        "n_samples",
        "min_gamma_overall",
        "robot_id",
        "crossed_gamma_star",
        "crossing_runtime_s",
        "crossing_sample_index",
        "dominant_edge",
        "dominant_neighbour_id",
        "pair_distance_m",
        "pair_radial_speed_ms",
        "pair_rel_speed_ms",
        "terminal_gamma_star",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for result in results:
            robots_here = set(result["terminal_gamma"].keys()) | set(result["first_crossings"].keys())
            if not robots_here and result["per_sample_rows"]:
                robots_here = {row["robot_id"] for row in result["per_sample_rows"]}
            for robot_id in sorted(robots_here):
                crossing = result["first_crossings"].get(robot_id, {})
                writer.writerow(
                    {
                        "seed": result["seed"],
                        "trial_label": result["label"],
                        "completed_horizon": result["completed_horizon"],
                        "n_samples": result["n_samples"],
                        "min_gamma_overall": result["min_gamma_overall"],
                        "robot_id": robot_id,
                        "crossed_gamma_star": crossing.get("gamma_star", math.nan),
                        "crossing_runtime_s": crossing.get("runtime_s", math.nan),
                        "crossing_sample_index": crossing.get("sample_index", math.nan),
                        "dominant_edge": crossing.get("dominant_edge", ""),
                        "dominant_neighbour_id": crossing.get("neighbour_id", ""),
                        "pair_distance_m": crossing.get("distance_m", math.nan),
                        "pair_radial_speed_ms": crossing.get("radial_speed_ms", math.nan),
                        "pair_rel_speed_ms": crossing.get("rel_speed_ms", math.nan),
                        "terminal_gamma_star": result["terminal_gamma"].get(robot_id, math.nan),
                    }
                )


def render_figure(results: list[dict[str, Any]], fig_path: Path) -> None:
    if plt is None:
        return
    fig_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    style_by_label = {
        "failing-20261224": {"color": "tab:red", "linestyle": "-"},
        "failing-20261229": {"color": "tab:orange", "linestyle": "--"},
        "control": {"color": "tab:blue", "linestyle": "-."},
    }
    for result in results:
        by_robot: dict[int, list[tuple[float, float]]] = {}
        for row in result["per_sample_rows"]:
            by_robot.setdefault(row["robot_id"], []).append(
                (row["runtime_s"], row["gamma_star_m_per_s2"])
            )
        style = style_by_label.get(
            result["label"], {"color": "tab:gray", "linestyle": ":"}
        )
        for robot_id, series in sorted(by_robot.items()):
            series.sort(key=lambda item: item[0])
            times = [item[0] for item in series]
            values = [item[1] for item in series]
            ax.plot(
                times,
                values,
                label=f"{result['label']} robot {robot_id} (seed {result['seed']})",
                color=style["color"],
                linestyle=style["linestyle"],
                alpha=0.85,
            )
    ax.axhline(0.0, color="black", linewidth=0.8, linestyle=":")
    ax.set_xlabel("runtime (s)")
    ax.set_ylabel(r"$\gamma_i^\star(x_k;\rho)$  (m/s$^2$)")
    ax.set_title(r"Reserve-feasibility budget $\gamma_i^\star$")
    ax.legend(fontsize=7, ncol=1, loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_path, dpi=180)
    fig.savefig(fig_path.with_suffix(".svg"))
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recompute the reserve-feasibility budget gamma* per sample on logged AS_HOCBF_TASK_RESERVE trials."
    )
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--failing-seeds", type=str, default=DEFAULT_FAILING)
    parser.add_argument("--control-seed", type=str, default=DEFAULT_CONTROL)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--fig-dir", type=Path, default=FIG_DIR_DEFAULT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows = _manifest_rows(args.manifest)

    failing_seeds = _parse_seeds(args.failing_seeds)
    control_seeds = _parse_seeds(args.control_seed)

    targets: list[tuple[int, str]] = []
    for seed in failing_seeds:
        targets.append((seed, f"failing-{seed}"))
    for seed in control_seeds:
        targets.append((seed, "control"))

    results: list[dict[str, Any]] = []
    for seed, label in targets:
        trial_row = _select_trial(rows, seed)
        result = analyse_trial(trial_row, seed, label)
        results.append(result)
        n_crossed = len(result["first_crossings"])
        min_gamma = result["min_gamma_overall"]
        print(
            f"seed={seed} label={label} completed={result['completed_horizon']} "
            f"samples={result['n_samples']} robots_crossed={n_crossed} "
            f"min_gamma={min_gamma:.6g}"
        )
        for robot_id, crossing in sorted(result["first_crossings"].items()):
            print(
                f"  robot {robot_id} crossing @ t={crossing['runtime_s']:.2f}s "
                f"(sample {crossing['sample_index']}) gamma={crossing['gamma_star']:.4f} "
                f"edge={crossing['dominant_edge']} neighbour={crossing['neighbour_id']} "
                f"dist={crossing['distance_m']:.2f} radial={crossing['radial_speed_ms']:.3f}"
            )

    all_rows: list[dict[str, Any]] = []
    for result in results:
        all_rows.extend(result["per_sample_rows"])

    per_sample_path = args.out_dir / "reserve_budget_trajectory.csv"
    crossing_path = args.out_dir / "reserve_budget_crossings.csv"
    write_per_sample_csv(all_rows, per_sample_path)
    write_crossing_summary(results, crossing_path)
    print(f"wrote {len(all_rows)} per-sample rows to {per_sample_path}")
    print(f"wrote crossing summary to {crossing_path}")

    fig_path = args.fig_dir / "reserve_budget_trajectory.png"
    render_figure(results, fig_path)
    if fig_path.exists():
        print(f"wrote figure to {fig_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
