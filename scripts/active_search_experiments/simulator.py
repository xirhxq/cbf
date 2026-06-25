from __future__ import annotations

from dataclasses import dataclass, replace
import csv
import math
from pathlib import Path
import random
from typing import Iterable


Point = tuple[float, float]


@dataclass(frozen=True)
class ActiveSearchConfig:
    width: int = 30
    height: int = 16
    robots: int = 3
    steps: int = 30
    seed: int = 1
    max_step: float = 2.0
    d_loc: float = 7.0
    sensor_radius: float = 2.4
    clarity_weight: float = 1.0
    belief_weight: float = 3.0
    travel_weight: float = 0.05
    overlap_weight: float = 0.2
    feasibility_weight: float = 0.05
    feasibility_reserve: float = 0.0
    predictive_feasibility_weight: float = 0.0
    predictive_feasibility_reserve: float = 0.0
    predictive_horizon: int = 1
    fallback_period: int = 8
    boundary_weight: float = 0.0
    boundary_reserve: float = 0.0
    cell_size_m: float = 100.0
    dt_s: float = 1.0
    target_cell: tuple[int, int] | None = None
    prior_center: Point | None = None


def maritime_scale_metadata(config: ActiveSearchConfig) -> dict[str, float]:
    return {
        "width_m": config.width * config.cell_size_m,
        "height_m": config.height * config.cell_size_m,
        "duration_s": config.steps * config.dt_s,
        "cell_size_m": config.cell_size_m,
        "dt_s": config.dt_s,
    }


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def choose_active_target(
    position: Point,
    clarity: list[list[float]],
    belief: list[list[float]],
    config: ActiveSearchConfig,
) -> tuple[int, int]:
    best_cell = (0, 0)
    best_score = -math.inf
    for y in range(config.height):
        for x in range(config.width):
            completion_need = 1.0 - clarity[y][x]
            if completion_need <= 1e-12 and belief[y][x] <= 1e-12:
                continue
            score = (
                config.clarity_weight * completion_need
                + config.belief_weight * belief[y][x]
                - config.travel_weight * distance(position, (x, y))
            )
            if score > best_score:
                best_score = score
                best_cell = (x, y)
    return best_cell


def choose_feasibility_aware_target(
    position: Point,
    anchor: Point,
    clarity: list[list[float]],
    belief: list[list[float]],
    config: ActiveSearchConfig,
) -> tuple[int, int]:
    best_cell = (0, 0)
    best_score = -math.inf
    for y in range(config.height):
        for x in range(config.width):
            completion_need = 1.0 - clarity[y][x]
            if completion_need <= 1e-12 and belief[y][x] <= 1e-12:
                continue
            score = (
                config.clarity_weight * completion_need
                + config.belief_weight * belief[y][x]
                - config.travel_weight * distance(position, (x, y))
                - _feasibility_penalty(position, anchor, (x, y), config)
            )
            if score > best_score:
                best_score = score
                best_cell = (x, y)
    return best_cell


def choose_predictive_feasibility_target(
    position: Point,
    anchor: Point,
    clarity: list[list[float]],
    belief: list[list[float]],
    config: ActiveSearchConfig,
) -> tuple[int, int]:
    best_cell = (0, 0)
    best_score = -math.inf
    for y in range(config.height):
        for x in range(config.width):
            completion_need = 1.0 - clarity[y][x]
            if completion_need <= 1e-12 and belief[y][x] <= 1e-12:
                continue
            score = (
                config.clarity_weight * completion_need
                + config.belief_weight * belief[y][x]
                - config.travel_weight * distance(position, (x, y))
                - _feasibility_penalty(position, anchor, (x, y), config)
                - _predictive_feasibility_penalty(position, anchor, (x, y), config)
            )
            if score > best_score:
                best_score = score
                best_cell = (x, y)
    return best_cell


def choose_active_fallback_target(
    position: Point,
    clarity: list[list[float]],
    belief: list[list[float]],
    config: ActiveSearchConfig,
    *,
    lane: int,
) -> tuple[int, int]:
    active_cell = choose_active_target(position, clarity, belief, config)
    if clarity[active_cell[1]][active_cell[0]] < 1.0:
        return active_cell
    return _nearest_uncleared_cell(position, clarity, config, lane=lane)


def filter_step_by_localization(
    current: Point,
    desired: Point,
    anchor: Point,
    config: ActiveSearchConfig,
) -> tuple[Point, float]:
    proposed = limit_step(current, desired, config.max_step)
    from_anchor = (proposed[0] - anchor[0], proposed[1] - anchor[1])
    dist = math.hypot(from_anchor[0], from_anchor[1])
    if dist > config.d_loc:
        scale = config.d_loc / dist
        proposed = (anchor[0] + from_anchor[0] * scale, anchor[1] + from_anchor[1] * scale)
    margin = config.d_loc - distance(anchor, proposed)
    return proposed, max(0.0, margin)


def limit_step(current: Point, target: Point, max_step: float) -> Point:
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    norm = math.hypot(dx, dy)
    if norm <= max_step or norm == 0.0:
        return (target[0], target[1])
    return (current[0] + dx / norm * max_step, current[1] + dy / norm * max_step)


def run_active_search_suite(config: ActiveSearchConfig) -> list[dict[str, float | str]]:
    return [
        _simulate_method("coverage_cvt", config),
        _simulate_method("active_unfiltered", config),
        _simulate_method("active_cbf", config),
        _simulate_method("active_fallback_cbf", config),
        _simulate_method("active_feasible_cbf", config),
        _simulate_method("active_predictive_cbf", config),
    ]


def write_active_search_artifacts(
    output_dir: Path,
    config: ActiveSearchConfig,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    methods = [
        "coverage_cvt",
        "active_unfiltered",
        "active_cbf",
        "active_fallback_cbf",
        "active_feasible_cbf",
        "active_predictive_cbf",
    ]
    simulations = [simulate_active_search(method, config) for method in methods]
    summary_csv = output_dir / "active_search_summary.csv"
    progress_png = output_dir / "active_search_progress.png"
    map_png = output_dir / "active_search_map.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "final_coverage",
                "belief_reduction",
                "min_loc_margin",
                "min_pair_margin",
            ],
        )
        writer.writeheader()
        for sim in simulations:
            writer.writerow(
                {
                    "method": sim["method"],
                    "final_coverage": f"{float(sim['final_coverage']):.6f}",
                    "belief_reduction": f"{float(sim['belief_reduction']):.6f}",
                    "min_loc_margin": f"{float(sim['min_loc_margin']):.6f}",
                    "min_pair_margin": f"{float(sim['min_pair_margin']):.6f}",
                }
            )

    _plot_active_progress(simulations, progress_png)
    _plot_active_map(simulations[-1], map_png)
    return {
        "summary_csv": summary_csv,
        "progress_png": progress_png,
        "map_png": map_png,
    }


def run_active_fallback_stress(config: ActiveSearchConfig) -> dict[str, object]:
    methods = ["active_cbf", "active_fallback_cbf"]
    simulations = [simulate_active_search(method, config) for method in methods]
    summary: list[dict[str, float | int | str]] = []
    for sim in simulations:
        summary.append(
            {
                "method": str(sim["method"]),
                "final_coverage": float(sim["final_coverage"]),
                "belief_reduction": float(sim["belief_reduction"]),
                "min_loc_margin": float(sim["min_loc_margin"]),
                "min_pair_margin": float(sim["min_pair_margin"]),
                "fallback_switches": int(sim["fallback_switches"]),
            }
        )
    return {
        "metadata": maritime_scale_metadata(config),
        "summary": summary,
        "simulations": simulations,
    }


def write_active_fallback_stress_artifacts(
    output_dir: Path,
    config: ActiveSearchConfig,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_active_fallback_stress(config)
    summary_csv = output_dir / "active_fallback_stress_summary.csv"
    progress_png = output_dir / "active_fallback_stress_progress.png"
    map_png = output_dir / "active_fallback_stress_map.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "final_coverage",
                "belief_reduction",
                "min_loc_margin",
                "min_pair_margin",
                "fallback_switches",
            ],
        )
        writer.writeheader()
        for row in result["summary"]:  # type: ignore[assignment]
            writer.writerow(
                {
                    "method": row["method"],
                    "final_coverage": f"{float(row['final_coverage']):.6f}",
                    "belief_reduction": f"{float(row['belief_reduction']):.6f}",
                    "min_loc_margin": f"{float(row['min_loc_margin']):.6f}",
                    "min_pair_margin": f"{float(row['min_pair_margin']):.6f}",
                    "fallback_switches": int(row["fallback_switches"]),
                }
            )

    simulations = result["simulations"]  # type: ignore[assignment]
    _plot_active_progress(simulations, progress_png)
    _plot_active_map(simulations[-1], map_png)
    return {"summary_csv": summary_csv, "progress_png": progress_png, "map_png": map_png}


def run_active_completion_stress(config: ActiveSearchConfig) -> dict[str, object]:
    simulation = simulate_active_search("coverage_cbf", config)
    completion_step = _completion_step(simulation)
    summary = {
        "method": str(simulation["method"]),
        "final_coverage": float(simulation["final_coverage"]),
        "completion_step": completion_step,
        "completion_time_s": completion_step * config.dt_s,
        "min_loc_margin": float(simulation["min_loc_margin"]),
        "min_pair_margin": float(simulation["min_pair_margin"]),
    }
    return {
        "metadata": maritime_scale_metadata(config),
        "summary": summary,
        "simulation": simulation,
    }


def write_active_completion_stress_artifacts(
    output_dir: Path,
    config: ActiveSearchConfig,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_active_completion_stress(config)
    summary_csv = output_dir / "active_completion_stress_summary.csv"
    progress_png = output_dir / "active_completion_stress_progress.png"
    map_png = output_dir / "active_completion_stress_map.png"
    row = result["summary"]  # type: ignore[assignment]

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "final_coverage",
                "completion_step",
                "completion_time_s",
                "min_loc_margin",
                "min_pair_margin",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "method": row["method"],
                "final_coverage": f"{float(row['final_coverage']):.6f}",
                "completion_step": int(row["completion_step"]),
                "completion_time_s": f"{float(row['completion_time_s']):.2f}",
                "min_loc_margin": f"{float(row['min_loc_margin']):.6f}",
                "min_pair_margin": f"{float(row['min_pair_margin']):.6f}",
            }
        )

    simulation = result["simulation"]  # type: ignore[assignment]
    _plot_active_progress([simulation], progress_png)
    _plot_active_map(simulation, map_png)
    return {"summary_csv": summary_csv, "progress_png": progress_png, "map_png": map_png}


def write_active_detection_artifacts(
    output_dir: Path,
    config: ActiveSearchConfig,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    methods = ["coverage_cvt", "coverage_cbf", "active_unfiltered", "active_cbf", "active_feasible_cbf"]
    simulations = [simulate_active_detection(method, config) for method in methods]
    summary_csv = output_dir / "active_detection_summary.csv"
    progress_png = output_dir / "active_detection_progress.png"
    map_png = output_dir / "active_detection_map.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "detection_step",
                "belief_at_target_final",
                "belief_entropy_final",
                "min_loc_margin",
                "min_pair_margin",
            ],
        )
        writer.writeheader()
        for sim in simulations:
            detection_step = sim["detection_step"]
            writer.writerow(
                {
                    "method": sim["method"],
                    "detection_step": "" if detection_step is None else f"{int(detection_step)}",
                    "belief_at_target_final": f"{float(sim['belief_at_target_final']):.6f}",
                    "belief_entropy_final": f"{float(sim['belief_entropy_final']):.6f}",
                    "min_loc_margin": f"{float(sim['min_loc_margin']):.6f}",
                    "min_pair_margin": f"{float(sim['min_pair_margin']):.6f}",
                }
            )

    _plot_detection_progress(simulations, progress_png, config)
    _plot_detection_map(simulations[-1], map_png, config)
    return {
        "summary_csv": summary_csv,
        "progress_png": progress_png,
        "map_png": map_png,
    }


def run_active_detection_sensitivity(
    config: ActiveSearchConfig,
    d_locs: Iterable[float],
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for d_loc in d_locs:
        scenario = replace(config, d_loc=float(d_loc))
        coverage = simulate_active_detection("coverage_cbf", scenario)
        active = simulate_active_detection("active_cbf", scenario)
        coverage_step = float(coverage["detection_step"])
        active_step = float(active["detection_step"])
        rows.append(
            {
                "d_loc": float(d_loc),
                "coverage_detection_step": coverage_step,
                "active_detection_step": active_step,
                "detection_gain_s": (coverage_step - active_step) * scenario.dt_s,
                "coverage_min_loc_margin": float(coverage["min_loc_margin"]),
                "active_min_loc_margin": float(active["min_loc_margin"]),
                "active_min_pair_margin": float(active["min_pair_margin"]),
            }
        )
    return rows


def write_active_detection_sensitivity_artifacts(
    output_dir: Path,
    config: ActiveSearchConfig,
    d_locs: Iterable[float],
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = run_active_detection_sensitivity(config, d_locs)
    summary_csv = output_dir / "active_detection_sensitivity.csv"
    progress_png = output_dir / "active_detection_sensitivity.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "d_loc",
                "coverage_detection_step",
                "active_detection_step",
                "detection_gain_s",
                "coverage_min_loc_margin",
                "active_min_loc_margin",
                "active_min_pair_margin",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "d_loc": f"{row['d_loc']:.2f}",
                    "coverage_detection_step": f"{row['coverage_detection_step']:.0f}",
                    "active_detection_step": f"{row['active_detection_step']:.0f}",
                    "detection_gain_s": f"{row['detection_gain_s']:.2f}",
                    "coverage_min_loc_margin": f"{row['coverage_min_loc_margin']:.6f}",
                    "active_min_loc_margin": f"{row['active_min_loc_margin']:.6f}",
                    "active_min_pair_margin": f"{row['active_min_pair_margin']:.6f}",
                }
            )

    _plot_detection_sensitivity(rows, progress_png, config)
    return {"summary_csv": summary_csv, "progress_png": progress_png}


def run_active_detection_sensor_sensitivity(
    config: ActiveSearchConfig,
    sensor_radii: Iterable[float],
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for sensor_radius in sensor_radii:
        scenario = replace(config, sensor_radius=float(sensor_radius))
        coverage = simulate_active_detection("coverage_cbf", scenario)
        active = simulate_active_detection("active_cbf", scenario)
        coverage_step = float(coverage["detection_step"])
        active_step = float(active["detection_step"])
        rows.append(
            {
                "sensor_radius": float(sensor_radius),
                "sensor_radius_m": float(sensor_radius) * scenario.cell_size_m,
                "coverage_detection_step": coverage_step,
                "active_detection_step": active_step,
                "detection_gain_s": (coverage_step - active_step) * scenario.dt_s,
                "coverage_min_loc_margin": float(coverage["min_loc_margin"]),
                "active_min_loc_margin": float(active["min_loc_margin"]),
                "active_min_pair_margin": float(active["min_pair_margin"]),
            }
        )
    return rows


def write_active_detection_sensor_sensitivity_artifacts(
    output_dir: Path,
    config: ActiveSearchConfig,
    sensor_radii: Iterable[float],
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = run_active_detection_sensor_sensitivity(config, sensor_radii)
    summary_csv = output_dir / "active_detection_sensor_sensitivity.csv"
    progress_png = output_dir / "active_detection_sensor_sensitivity.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "sensor_radius",
                "sensor_radius_m",
                "coverage_detection_step",
                "active_detection_step",
                "detection_gain_s",
                "coverage_min_loc_margin",
                "active_min_loc_margin",
                "active_min_pair_margin",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "sensor_radius": f"{row['sensor_radius']:.2f}",
                    "sensor_radius_m": f"{row['sensor_radius_m']:.2f}",
                    "coverage_detection_step": f"{row['coverage_detection_step']:.0f}",
                    "active_detection_step": f"{row['active_detection_step']:.0f}",
                    "detection_gain_s": f"{row['detection_gain_s']:.2f}",
                    "coverage_min_loc_margin": f"{row['coverage_min_loc_margin']:.6f}",
                    "active_min_loc_margin": f"{row['active_min_loc_margin']:.6f}",
                    "active_min_pair_margin": f"{row['active_min_pair_margin']:.6f}",
                }
            )

    _plot_detection_sensor_sensitivity(rows, progress_png, config)
    return {"summary_csv": summary_csv, "progress_png": progress_png}


def run_active_detection_monte_carlo(
    config: ActiveSearchConfig,
    trials: int,
    methods: tuple[str, ...] = ("coverage_cbf", "active_cbf"),
) -> dict[str, object]:
    trial_rows: list[dict[str, float | int | str]] = []
    rng = random.Random(config.seed + 1009)
    for trial in range(trials):
        target = _sample_target_cell(rng, config)
        prior_center = _sample_prior_center(rng, target, config)
        scenario = replace(
            config,
            seed=config.seed + trial,
            target_cell=target,
            prior_center=prior_center,
        )
        for method in methods:
            sim = simulate_active_detection(method, scenario)
            detection_step = int(sim["detection_step"])
            detected = detection_step <= scenario.steps
            trial_rows.append(
                {
                    "trial": trial,
                    "method": method,
                    "target_x": target[0],
                    "target_y": target[1],
                    "prior_x": prior_center[0],
                    "prior_y": prior_center[1],
                    "detected": int(detected),
                    "detection_step": detection_step,
                    "detection_time_s": detection_step * scenario.dt_s,
                    "min_loc_margin": float(sim["min_loc_margin"]),
                    "min_pair_margin": float(sim["min_pair_margin"]),
                }
            )
    return {
        "metadata": maritime_scale_metadata(config),
        "summary": _summarize_detection_trials(trial_rows, config, methods),
        "trials": trial_rows,
    }


def write_active_detection_monte_carlo_artifacts(
    output_dir: Path,
    config: ActiveSearchConfig,
    trials: int,
    methods: tuple[str, ...] = ("coverage_cbf", "active_cbf"),
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_active_detection_monte_carlo(config, trials, methods=methods)
    summary_rows = result["summary"]
    trial_rows = result["trials"]
    summary_csv = output_dir / "active_detection_monte_carlo_summary.csv"
    trials_csv = output_dir / "active_detection_monte_carlo_trials.csv"
    progress_png = output_dir / "active_detection_monte_carlo.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "trial_count",
                "detections",
                "success_rate",
                "success_rate_ci_low",
                "success_rate_ci_high",
                "mean_detection_time_s",
                "mean_detection_time_ci_half_width_s",
                "median_detection_time_s",
                "mean_censored_detection_time_s",
                "mean_censored_detection_time_ci_half_width_s",
                "mean_min_loc_margin",
                "mean_min_pair_margin",
            ],
        )
        writer.writeheader()
        for row in summary_rows:  # type: ignore[assignment]
            writer.writerow(
                {
                    "method": row["method"],
                    "trial_count": int(row["trial_count"]),
                    "detections": int(row["detections"]),
                    "success_rate": f"{float(row['success_rate']):.6f}",
                    "success_rate_ci_low": f"{float(row['success_rate_ci_low']):.6f}",
                    "success_rate_ci_high": f"{float(row['success_rate_ci_high']):.6f}",
                    "mean_detection_time_s": f"{float(row['mean_detection_time_s']):.2f}",
                    "mean_detection_time_ci_half_width_s": f"{float(row['mean_detection_time_ci_half_width_s']):.2f}",
                    "median_detection_time_s": f"{float(row['median_detection_time_s']):.2f}",
                    "mean_censored_detection_time_s": f"{float(row['mean_censored_detection_time_s']):.2f}",
                    "mean_censored_detection_time_ci_half_width_s": f"{float(row['mean_censored_detection_time_ci_half_width_s']):.2f}",
                    "mean_min_loc_margin": f"{float(row['mean_min_loc_margin']):.6f}",
                    "mean_min_pair_margin": f"{float(row['mean_min_pair_margin']):.6f}",
                }
            )

    with trials_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "trial",
                "method",
                "target_x",
                "target_y",
                "prior_x",
                "prior_y",
                "detected",
                "detection_step",
                "detection_time_s",
                "min_loc_margin",
                "min_pair_margin",
            ],
        )
        writer.writeheader()
        for row in trial_rows:  # type: ignore[assignment]
            writer.writerow(
                {
                    "trial": int(row["trial"]),
                    "method": row["method"],
                    "target_x": int(row["target_x"]),
                    "target_y": int(row["target_y"]),
                    "prior_x": f"{float(row['prior_x']):.3f}",
                    "prior_y": f"{float(row['prior_y']):.3f}",
                    "detected": int(row["detected"]),
                    "detection_step": int(row["detection_step"]),
                    "detection_time_s": f"{float(row['detection_time_s']):.2f}",
                    "min_loc_margin": f"{float(row['min_loc_margin']):.6f}",
                    "min_pair_margin": f"{float(row['min_pair_margin']):.6f}",
                }
            )

    _plot_detection_monte_carlo(summary_rows, progress_png)  # type: ignore[arg-type]
    return {"summary_csv": summary_csv, "trials_csv": trials_csv, "progress_png": progress_png}


def _sample_target_cell(rng: random.Random, config: ActiveSearchConfig) -> tuple[int, int]:
    min_x = min(config.width - 2, max(1, int(config.width * 0.58)))
    max_x = max(min_x, config.width - 2)
    min_y = min(config.height - 2, max(1, int(config.height * 0.52)))
    max_y = max(min_y, config.height - 2)
    return (rng.randint(min_x, max_x), rng.randint(min_y, max_y))


def _sample_prior_center(rng: random.Random, target: tuple[int, int], config: ActiveSearchConfig) -> Point:
    jitter_x = rng.uniform(-2.0, 2.0)
    jitter_y = rng.uniform(-2.0, 2.0)
    return (
        min(max(target[0] + jitter_x, 0.0), config.width - 1.0),
        min(max(target[1] + jitter_y, 0.0), config.height - 1.0),
    )


def _summarize_detection_trials(
    trial_rows: list[dict[str, float | int | str]],
    config: ActiveSearchConfig,
    methods: tuple[str, ...],
) -> list[dict[str, float | int | str]]:
    summary: list[dict[str, float | int | str]] = []
    censored_time = (config.steps + 1) * config.dt_s
    for method in methods:
        rows = [row for row in trial_rows if row["method"] == method]
        detected_rows = [row for row in rows if int(row["detected"]) == 1]
        detection_times = [float(row["detection_time_s"]) for row in detected_rows]
        all_detection_times = [min(float(row["detection_time_s"]), censored_time) for row in rows]
        ci_low, ci_high = _wilson_interval(len(detected_rows), len(rows))
        summary.append(
            {
                "method": method,
                "trial_count": len(rows),
                "detections": len(detected_rows),
                "success_rate": len(detected_rows) / len(rows) if rows else 0.0,
                "success_rate_ci_low": ci_low,
                "success_rate_ci_high": ci_high,
                "mean_detection_time_s": _mean(detection_times, default=censored_time),
                "mean_detection_time_ci_half_width_s": _mean_ci_half_width(detection_times),
                "median_detection_time_s": _median(detection_times, default=censored_time),
                "mean_censored_detection_time_s": _mean(all_detection_times, default=censored_time),
                "mean_censored_detection_time_ci_half_width_s": _mean_ci_half_width(all_detection_times),
                "mean_min_loc_margin": _mean([float(row["min_loc_margin"]) for row in rows], default=0.0),
                "mean_min_pair_margin": _mean([float(row["min_pair_margin"]) for row in rows], default=0.0),
            }
        )
    return summary


def _mean(values: list[float], *, default: float) -> float:
    if not values:
        return default
    return sum(values) / len(values)


def _mean_ci_half_width(values: list[float], *, z: float = 1.96) -> float:
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    variance = sum((value - mean) ** 2 for value in values) / (len(values) - 1)
    return z * math.sqrt(variance / len(values))


def _wilson_interval(successes: int, total: int, *, z: float = 1.96) -> tuple[float, float]:
    if total <= 0:
        return 0.0, 0.0
    phat = successes / total
    z2 = z * z
    denominator = 1.0 + z2 / total
    center = (phat + z2 / (2.0 * total)) / denominator
    half_width = z * math.sqrt(phat * (1.0 - phat) / total + z2 / (4.0 * total * total)) / denominator
    return max(0.0, center - half_width), min(1.0, center + half_width)


def _median(values: list[float], *, default: float) -> float:
    if not values:
        return default
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def simulate_active_search(
    method: str,
    config: ActiveSearchConfig,
) -> dict[str, object]:
    random.seed(config.seed)
    clarity = [[0.0 for _ in range(config.width)] for _ in range(config.height)]
    initial_belief = _make_belief(config)
    belief = [row[:] for row in initial_belief]
    robots = _initial_robot_positions(config)
    anchor = (0.0, config.height / 2.0)
    history: list[dict[str, object]] = []
    min_loc_margin = math.inf
    min_pair_margin = math.inf
    initial_belief_sum = _sum_grid(initial_belief)
    fallback_switches = 0

    for step in range(config.steps + 1):
        _observe(robots, clarity, belief, config)
        history.append(
            {
                "step": step,
                "robots": robots[:],
                "coverage": _coverage(clarity),
                "belief_sum": _sum_grid(belief),
                "min_loc_margin": min_loc_margin if min_loc_margin < math.inf else config.d_loc,
            }
        )
        if step == config.steps:
            break

        next_robots: list[Point] = []
        for idx, pos in enumerate(robots):
            parent = anchor if idx == 0 else next_robots[idx - 1]
            if method == "active_fallback_cbf":
                active_target = choose_active_target(pos, clarity, belief, config)
                if _coverage_fallback_due(step, idx, config) and _coverage(clarity) < 1.0:
                    target = _lane_sweep_uncleared_cell(clarity, config, lane=idx)
                else:
                    target = choose_active_fallback_target(pos, clarity, belief, config, lane=idx)
                if target != active_target:
                    fallback_switches += 1
            else:
                target = _choose_target_for_method(method, idx, pos, parent, clarity, belief, config)
            desired = (float(target[0]), float(target[1]))
            if method in {
                "active_cbf",
                "coverage_cbf",
                "active_fallback_cbf",
                "active_feasible_cbf",
                "active_predictive_cbf",
            }:
                desired = _lane_offset_desired(desired, idx, config)
                new_pos, margin = filter_step_by_localization(pos, desired, parent, config)
                new_pos, margin = _separate_from_previous(new_pos, next_robots, parent, config)
                min_loc_margin = min(min_loc_margin, margin)
            else:
                new_pos = limit_step(pos, desired, config.max_step)
                min_loc_margin = min(min_loc_margin, config.d_loc - distance(parent, new_pos))
            next_robots.append(_clamp_to_world(new_pos, config))
        robots = next_robots
        min_pair_margin = min(min_pair_margin, _min_pair_margin(robots))

    final_belief_sum = _sum_grid(belief)
    return {
        "method": method,
        "final_coverage": _coverage(clarity),
        "belief_reduction": max(0.0, initial_belief_sum - final_belief_sum),
        "min_loc_margin": min_loc_margin,
        "min_pair_margin": min_pair_margin if min_pair_margin < math.inf else 0.0,
        "fallback_switches": fallback_switches,
        "history": history,
        "clarity": clarity,
        "belief": belief,
    }


def simulate_active_detection(
    method: str,
    config: ActiveSearchConfig,
) -> dict[str, object]:
    target = _target_cell(config)
    belief = _normalize_grid(_make_belief(config))
    clarity = [[0.0 for _ in range(config.width)] for _ in range(config.height)]
    robots = _initial_robot_positions(config)
    anchor = (0.0, config.height / 2.0)
    history: list[dict[str, object]] = []
    min_loc_margin = math.inf
    min_pair_margin = math.inf
    detection_step: int | None = None

    for step in range(config.steps + 1):
        if detection_step is None:
            for pos in robots:
                detected = distance(pos, target) <= config.sensor_radius * 0.72
                belief = update_belief_with_detection(belief, pos, target, detected, config)
                if detected:
                    detection_step = step
                    break
        _observe_clarity_only(robots, clarity, config)
        history.append(
            {
                "step": step,
                "robots": robots[:],
                "belief_at_target": belief[target[1]][target[0]],
                "belief_entropy": _belief_entropy(belief),
                "coverage": _coverage(clarity),
                "detected": detection_step is not None,
            }
        )
        if step == config.steps:
            break

        next_robots: list[Point] = []
        for idx, pos in enumerate(robots):
            parent = anchor if idx == 0 else next_robots[idx - 1]
            target_cell = (
                _nearest_uncleared_cell(pos, clarity, config, lane=idx)
                if method in {"coverage_cvt", "coverage_cbf"}
                else _choose_detection_target(
                    pos,
                    clarity,
                    belief,
                    config,
                    anchor=parent if method in {"active_feasible_cbf", "active_predictive_cbf"} else None,
                    predictive=method == "active_predictive_cbf",
                )
            )
            desired = (float(target_cell[0]), float(target_cell[1]))
            if method in {"active_cbf", "coverage_cbf", "active_feasible_cbf", "active_predictive_cbf"}:
                desired = _lane_offset_desired(desired, idx, config)
                new_pos, margin = filter_step_by_localization(pos, desired, parent, config)
                new_pos, margin = _separate_from_previous(new_pos, next_robots, parent, config)
                min_loc_margin = min(min_loc_margin, margin)
            else:
                new_pos = limit_step(pos, desired, config.max_step)
                min_loc_margin = min(min_loc_margin, config.d_loc - distance(parent, new_pos))
            next_robots.append(_clamp_to_world(new_pos, config))
        robots = next_robots
        min_pair_margin = min(min_pair_margin, _min_pair_margin(robots))

    return {
        "method": method,
        "target": target,
        "detection_step": detection_step if detection_step is not None else config.steps + 1,
        "belief_at_target_final": belief[target[1]][target[0]],
        "belief_entropy_final": _belief_entropy(belief),
        "min_loc_margin": min_loc_margin,
        "min_pair_margin": min_pair_margin if min_pair_margin < math.inf else 0.0,
        "history": history,
        "belief": belief,
        "clarity": clarity,
    }


def update_belief_with_detection(
    belief: list[list[float]],
    observer: Point,
    target: tuple[int, int],
    detected: bool,
    config: ActiveSearchConfig,
) -> list[list[float]]:
    updated = [[0.0 for _ in range(config.width)] for _ in range(config.height)]
    for y in range(config.height):
        for x in range(config.width):
            likelihood = _detection_probability(observer, (x, y), config)
            if detected:
                factor = max(1e-4, likelihood)
            else:
                factor = max(1e-4, 1.0 - likelihood)
            updated[y][x] = belief[y][x] * factor
    return _normalize_grid(updated)


def _simulate_method(method: str, config: ActiveSearchConfig) -> dict[str, float | str]:
    result = simulate_active_search(method, config)
    return {
        "method": str(result["method"]),
        "final_coverage": float(result["final_coverage"]),
        "belief_reduction": float(result["belief_reduction"]),
        "min_loc_margin": float(result["min_loc_margin"]),
        "min_pair_margin": float(result["min_pair_margin"]),
    }


def _completion_step(simulation: dict[str, object]) -> int:
    history = simulation["history"]
    for frame in history:  # type: ignore[assignment]
        if float(frame["coverage"]) >= 1.0:
            return int(frame["step"])
    return int(history[-1]["step"]) + 1  # type: ignore[index]


def _choose_target_for_method(
    method: str,
    idx: int,
    position: Point,
    anchor: Point,
    clarity: list[list[float]],
    belief: list[list[float]],
    config: ActiveSearchConfig,
) -> tuple[int, int]:
    if method == "coverage_cvt":
        return _nearest_uncleared_cell(position, clarity, config, lane=idx)
    if method == "coverage_cbf":
        return _lane_sweep_uncleared_cell(clarity, config, lane=idx)
    if method == "active_fallback_cbf":
        return choose_active_fallback_target(position, clarity, belief, config, lane=idx)
    if method == "active_feasible_cbf":
        return choose_feasibility_aware_target(position, anchor, clarity, belief, config)
    if method == "active_predictive_cbf":
        return choose_predictive_feasibility_target(position, anchor, clarity, belief, config)
    return choose_active_target(position, clarity, belief, config)


def _coverage_fallback_due(step: int, idx: int, config: ActiveSearchConfig) -> bool:
    period = max(1, int(config.fallback_period))
    return (step + idx) % period == 0


def _lane_sweep_uncleared_cell(
    clarity: list[list[float]],
    config: ActiveSearchConfig,
    *,
    lane: int,
) -> tuple[int, int]:
    band_min = max(0, int(round(lane * config.height / config.robots)))
    band_max = min(config.height - 1, int(round((lane + 1) * config.height / config.robots)) - 1)
    lane_center = 0.5 * (band_min + band_max)
    best = (0, int(round(lane_center)))
    best_score = -math.inf
    for x in range(config.width):
        for y in range(band_min, band_max + 1):
            if clarity[y][x] >= 1.0:
                continue
            score = x * config.height - abs(y - lane_center)
            if score > best_score:
                best_score = score
                best = (x, y)
    if best_score > -math.inf:
        return best
    return _nearest_uncleared_cell((float(best[0]), float(best[1])), clarity, config, lane=lane)


def _choose_detection_target(
    position: Point,
    clarity: list[list[float]],
    belief: list[list[float]],
    config: ActiveSearchConfig,
    *,
    anchor: Point | None = None,
    predictive: bool = False,
) -> tuple[int, int]:
    best = (0, 0)
    best_score = -math.inf
    for y in range(config.height):
        for x in range(config.width):
            expected_detection = belief[y][x] * _detection_probability(position, (x, y), config)
            posterior_priority = 42.0 * belief[y][x]
            completion_need = 0.15 * (1.0 - clarity[y][x])
            score = posterior_priority + expected_detection + completion_need - config.travel_weight * distance(position, (x, y))
            if anchor is not None:
                score -= _feasibility_penalty(position, anchor, (x, y), config)
                if predictive:
                    score -= _predictive_feasibility_penalty(position, anchor, (x, y), config)
            if score > best_score:
                best_score = score
                best = (x, y)
    return best


def _feasibility_penalty(
    position: Point,
    anchor: Point,
    candidate: tuple[int, int],
    config: ActiveSearchConfig,
) -> float:
    predicted = limit_step(position, (float(candidate[0]), float(candidate[1])), config.max_step)
    loc_margin = config.d_loc - distance(anchor, predicted)
    loc_deficit = max(0.0, config.feasibility_reserve - loc_margin)
    boundary_margin = min(
        predicted[0],
        config.width - 1.0 - predicted[0],
        predicted[1],
        config.height - 1.0 - predicted[1],
    )
    boundary_deficit = max(0.0, config.boundary_reserve - boundary_margin)
    return config.feasibility_weight * loc_deficit + config.boundary_weight * boundary_deficit


def _predictive_feasibility_penalty(
    position: Point,
    anchor: Point,
    candidate: tuple[int, int],
    config: ActiveSearchConfig,
) -> float:
    if config.predictive_feasibility_weight <= 0.0 or config.predictive_horizon <= 1:
        return 0.0
    simulated = position
    target = (float(candidate[0]), float(candidate[1]))
    min_margin = math.inf
    projection_deficit = 0.0
    boundary_deficit = 0.0
    for _ in range(config.predictive_horizon):
        desired = limit_step(simulated, target, config.max_step)
        filtered, margin = filter_step_by_localization(simulated, desired, anchor, config)
        min_margin = min(min_margin, margin)
        projection_deficit += max(0.0, distance(simulated, desired) - distance(simulated, filtered))
        boundary_margin = min(
            filtered[0],
            config.width - 1.0 - filtered[0],
            filtered[1],
            config.height - 1.0 - filtered[1],
        )
        boundary_deficit += max(0.0, config.boundary_reserve - boundary_margin)
        simulated = filtered
        if distance(simulated, target) <= 1e-9:
            break
    loc_deficit = max(0.0, config.predictive_feasibility_reserve - min_margin)
    return config.predictive_feasibility_weight * (loc_deficit + projection_deficit + boundary_deficit)


def _nearest_uncleared_cell(
    position: Point,
    clarity: list[list[float]],
    config: ActiveSearchConfig,
    *,
    lane: int,
) -> tuple[int, int]:
    best = (0, 0)
    best_score = math.inf
    lane_center = (lane + 1) * config.height / (config.robots + 1)
    for y in range(config.height):
        for x in range(config.width):
            if clarity[y][x] >= 1.0:
                continue
            lane_penalty = abs(y - lane_center) * 0.4
            score = distance(position, (x, y)) + lane_penalty
            if score < best_score:
                best_score = score
                best = (x, y)
    return best


def _make_belief(config: ActiveSearchConfig) -> list[list[float]]:
    if config.prior_center is None:
        cx = config.width * 0.78
        cy = config.height * 0.65
    else:
        cx, cy = config.prior_center
    sigma_x = max(3.0, config.width * 0.25)
    sigma_y = max(2.0, config.height * 0.25)
    belief: list[list[float]] = []
    for y in range(config.height):
        row = []
        for x in range(config.width):
            exponent = -(((x - cx) / sigma_x) ** 2 + ((y - cy) / sigma_y) ** 2) / 2.0
            row.append(0.05 + 0.95 * math.exp(exponent))
        belief.append(row)
    return belief


def _target_cell(config: ActiveSearchConfig) -> tuple[int, int]:
    if config.target_cell is not None:
        return (
            min(max(int(config.target_cell[0]), 0), config.width - 1),
            min(max(int(config.target_cell[1]), 0), config.height - 1),
        )
    return (min(config.width - 2, int(config.width * 0.72)), min(config.height - 2, int(config.height * 0.76)))


def _normalize_grid(grid: list[list[float]]) -> list[list[float]]:
    total = _sum_grid(grid)
    if total <= 0.0:
        height = len(grid)
        width = len(grid[0])
        value = 1.0 / (width * height)
        return [[value for _ in range(width)] for _ in range(height)]
    return [[value / total for value in row] for row in grid]


def _detection_probability(
    observer: Point,
    cell: tuple[int, int],
    config: ActiveSearchConfig,
) -> float:
    dist = distance(observer, cell)
    if dist > config.sensor_radius:
        return 0.0
    range_term = max(0.0, 1.0 - dist / config.sensor_radius)
    return min(0.92, 0.18 + 0.74 * range_term)


def _belief_entropy(belief: list[list[float]]) -> float:
    entropy = 0.0
    for row in belief:
        for value in row:
            if value > 0.0:
                entropy -= value * math.log(value)
    return entropy


def _initial_robot_positions(config: ActiveSearchConfig) -> list[Point]:
    return [
        (2.0 + 0.4 * i, (i + 1) * config.height / (config.robots + 1))
        for i in range(config.robots)
    ]


def _observe(
    robots: Iterable[Point],
    clarity: list[list[float]],
    belief: list[list[float]],
    config: ActiveSearchConfig,
) -> None:
    for pos in robots:
        for y in range(config.height):
            for x in range(config.width):
                if distance(pos, (x, y)) <= config.sensor_radius:
                    clarity[y][x] = 1.0
                    belief[y][x] *= 0.35


def _observe_clarity_only(
    robots: Iterable[Point],
    clarity: list[list[float]],
    config: ActiveSearchConfig,
) -> None:
    for pos in robots:
        for y in range(config.height):
            for x in range(config.width):
                if distance(pos, (x, y)) <= config.sensor_radius:
                    clarity[y][x] = 1.0


def _coverage(clarity: list[list[float]]) -> float:
    total = len(clarity) * len(clarity[0])
    searched = sum(1 for row in clarity for value in row if value >= 1.0)
    return searched / total


def _sum_grid(grid: list[list[float]]) -> float:
    return sum(sum(row) for row in grid)


def _clamp_to_world(point: Point, config: ActiveSearchConfig) -> Point:
    return (
        min(max(point[0], 0.0), config.width - 1.0),
        min(max(point[1], 0.0), config.height - 1.0),
    )


def _lane_offset_desired(desired: Point, idx: int, config: ActiveSearchConfig) -> Point:
    center = (config.robots - 1) / 2.0
    y = desired[1] + (idx - center) * 1.1
    return (desired[0], min(max(y, 0.0), config.height - 1.0))


def _separate_from_previous(
    proposed: Point,
    previous: list[Point],
    parent: Point,
    config: ActiveSearchConfig,
    safe_distance: float = 0.8,
) -> tuple[Point, float]:
    adjusted = proposed
    for idx, other in enumerate(previous):
        if distance(adjusted, other) >= safe_distance:
            continue
        sign = 1.0 if idx % 2 == 0 else -1.0
        if adjusted[1] >= other[1]:
            sign = 1.0
        candidate = (adjusted[0], other[1] + sign * safe_distance)
        candidate = _clamp_to_world(candidate, config)
        adjusted, _ = filter_step_by_localization(adjusted, candidate, parent, config)
    return adjusted, max(0.0, config.d_loc - distance(parent, adjusted))


def _min_pair_margin(robots: list[Point], safe_distance: float = 0.8) -> float:
    margin = math.inf
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            margin = min(margin, distance(robots[i], robots[j]) - safe_distance)
    return margin


def _plot_active_progress(simulations: list[dict[str, object]], output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(6.2, 5.0), sharex=True)
    for sim in simulations:
        history = sim["history"]
        steps = [frame["step"] for frame in history]  # type: ignore[index]
        coverage = [100.0 * frame["coverage"] for frame in history]  # type: ignore[index]
        belief = [frame["belief_sum"] for frame in history]  # type: ignore[index]
        axes[0].plot(steps, coverage, marker="o", markersize=2.5, label=str(sim["method"]))
        axes[1].plot(steps, belief, marker="s", markersize=2.2, label=str(sim["method"]))
    axes[0].set_ylabel("Coverage / %")
    axes[1].set_ylabel("Belief mass")
    axes[1].set_xlabel("Step")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_active_map(simulation: dict[str, object], output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    clarity = simulation["clarity"]
    history = simulation["history"]
    fig, ax = plt.subplots(figsize=(6.2, 3.6))
    ax.imshow(clarity, origin="lower", cmap="viridis", vmin=0.0, vmax=1.0)  # type: ignore[arg-type]
    robot_count = len(history[0]["robots"])  # type: ignore[index]
    for robot_idx in range(robot_count):
        xs = [frame["robots"][robot_idx][0] for frame in history]  # type: ignore[index]
        ys = [frame["robots"][robot_idx][1] for frame in history]  # type: ignore[index]
        ax.plot(xs, ys, linewidth=1.8, marker="o", markersize=2.2, label=f"UAV {robot_idx + 1}")
    ax.set_title(str(simulation["method"]).replace("_", " "))
    ax.set_xlabel("x cell")
    ax.set_ylabel("y cell")
    ax.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_detection_progress(
    simulations: list[dict[str, object]],
    output_path: Path,
    config: ActiveSearchConfig,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True)
    for sim in simulations:
        history = sim["history"]
        times = [frame["step"] * config.dt_s for frame in history]  # type: ignore[index]
        target_belief = [frame["belief_at_target"] for frame in history]  # type: ignore[index]
        entropy = [frame["belief_entropy"] for frame in history]  # type: ignore[index]
        label = str(sim["method"])
        axes[0].plot(times, target_belief, marker="o", markersize=2.0, label=label)
        axes[1].plot(times, entropy, marker="s", markersize=1.8, label=label)
        detection_step = sim["detection_step"]
        if detection_step is not None:
            axes[0].axvline(float(detection_step) * config.dt_s, color="gray", alpha=0.15, linewidth=0.8)
    axes[0].set_ylabel("Target posterior")
    axes[1].set_ylabel("Belief entropy")
    axes[1].set_xlabel("Time / s")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_detection_map(
    simulation: dict[str, object],
    output_path: Path,
    config: ActiveSearchConfig,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    belief = simulation["belief"]
    history = simulation["history"]
    target = simulation["target"]
    metadata = maritime_scale_metadata(config)
    fig, ax = plt.subplots(figsize=(6.2, 5.2))
    ax.imshow(belief, origin="lower", cmap="magma")  # type: ignore[arg-type]
    robot_count = len(history[0]["robots"])  # type: ignore[index]
    for robot_idx in range(robot_count):
        xs = [frame["robots"][robot_idx][0] for frame in history]  # type: ignore[index]
        ys = [frame["robots"][robot_idx][1] for frame in history]  # type: ignore[index]
        ax.plot(xs, ys, linewidth=1.4, marker="o", markersize=1.8, label=f"UAV {robot_idx + 1}")
    ax.scatter([target[0]], [target[1]], color="#00e5ff", marker="*", s=95, edgecolor="black", linewidth=0.5, label="target")
    ax.set_title(f"Detection posterior, {metadata['width_m'] / 1000:.1f} km x {metadata['height_m'] / 1000:.1f} km")
    ax.set_xlabel("x cell")
    ax.set_ylabel("y cell")
    ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_detection_sensitivity(
    rows: list[dict[str, float]],
    output_path: Path,
    config: ActiveSearchConfig,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d_locs = [row["d_loc"] * config.cell_size_m / 1000.0 for row in rows]
    coverage_times = [row["coverage_detection_step"] * config.dt_s for row in rows]
    active_times = [row["active_detection_step"] * config.dt_s for row in rows]
    gains = [row["detection_gain_s"] for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True)
    axes[0].plot(d_locs, coverage_times, marker="o", label="coverage CBF")
    axes[0].plot(d_locs, active_times, marker="s", label="active CBF")
    axes[1].bar(d_locs, gains, width=0.035, color="#118ab2", alpha=0.8)
    axes[0].set_ylabel("Detection time / s")
    axes[1].set_ylabel("Active gain / s")
    axes[1].set_xlabel("Localization range / km")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_detection_sensor_sensitivity(
    rows: list[dict[str, float]],
    output_path: Path,
    config: ActiveSearchConfig,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    radii_m = [row["sensor_radius_m"] for row in rows]
    coverage_times = [row["coverage_detection_step"] * config.dt_s for row in rows]
    active_times = [row["active_detection_step"] * config.dt_s for row in rows]
    gains = [row["detection_gain_s"] for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True)
    axes[0].plot(radii_m, coverage_times, marker="o", label="coverage CBF")
    axes[0].plot(radii_m, active_times, marker="s", label="active CBF")
    axes[1].bar(radii_m, gains, width=16.0, color="#5c677d", alpha=0.82)
    axes[0].set_ylabel("Detection time / s")
    axes[1].set_ylabel("Active gain / s")
    axes[1].set_xlabel("Sensor radius / m")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_detection_monte_carlo(
    rows: list[dict[str, float | int | str]],
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = [str(row["method"]) for row in rows]
    success = [100.0 * float(row["success_rate"]) for row in rows]
    success_yerr = [
        [100.0 * (float(row["success_rate"]) - float(row["success_rate_ci_low"])) for row in rows],
        [100.0 * (float(row["success_rate_ci_high"]) - float(row["success_rate"])) for row in rows],
    ]
    censored_time = [float(row["mean_censored_detection_time_s"]) for row in rows]
    censored_time_yerr = [float(row["mean_censored_detection_time_ci_half_width_s"]) for row in rows]

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 3.2))
    axes[0].bar(labels, success, yerr=success_yerr, capsize=4.0, color="#2a9d8f", alpha=0.86)
    axes[1].bar(labels, censored_time, yerr=censored_time_yerr, capsize=4.0, color="#457b9d", alpha=0.86)
    axes[0].set_ylabel("Detection success / %")
    axes[1].set_ylabel("Mean censored time / s")
    axes[0].set_ylim(0.0, 105.0)
    for axis in axes:
        axis.grid(True, axis="y", alpha=0.3)
        axis.tick_params(axis="x", labelrotation=20)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)
