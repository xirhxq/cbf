from __future__ import annotations

from dataclasses import dataclass, replace
import csv
import math
from pathlib import Path
import random
from typing import Iterable


Point = tuple[float, float]


@dataclass(frozen=True)
class LinkState:
    available: bool
    quality: float
    dropout_age: float
    delay: float
    reference_variance: float


@dataclass(frozen=True)
class LinkDeniedConfig:
    width: int = 34
    height: int = 14
    steps: int = 24
    seed: int = 1
    d_loc: float = 9.0
    max_step: float = 1.0
    sensor_radius: float = 2.2
    sigma0: float = 0.2
    q_min: float = 0.05
    dropout_weight: float = 0.08
    delay_weight: float = 0.04
    kappa: float = 0.5
    cell_size_m: float = 100.0
    dt_s: float = 1.0
    healthy_quality: float = 0.94
    denied_quality: float = 0.18
    denied_dropout_scale: float = 0.25
    denied_delay: float = 0.8
    denied_reference_variance: float = 0.08
    predictive_margin_reserve: float = 0.0
    denial_center: Point | None = None
    denial_size: Point | None = None


@dataclass(frozen=True)
class Topology:
    parent: dict[str, str]
    roles: dict[str, str]
    min_loc_margin: float
    certified: bool


def link_denied_scale_metadata(config: LinkDeniedConfig) -> dict[str, float]:
    return {
        "width_m": config.width * config.cell_size_m,
        "height_m": config.height * config.cell_size_m,
        "duration_s": config.steps * config.dt_s,
        "cell_size_m": config.cell_size_m,
        "dt_s": config.dt_s,
    }


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def effective_range_variance(link: LinkState, config: LinkDeniedConfig | None = None) -> float:
    config = config or LinkDeniedConfig()
    if not link.available:
        return math.inf
    quality = max(config.q_min, link.quality)
    return (
        config.sigma0**2 / quality
        + config.dropout_weight * link.dropout_age
        + config.delay_weight * link.delay
        + link.reference_variance
    )


def select_adaptive_topology(
    positions: dict[str, Point],
    links: dict[tuple[str, str], LinkState],
    config: LinkDeniedConfig,
) -> Topology:
    direct_margin = _edge_margin("beacon", "searcher", positions, links, config)
    if direct_margin >= 0.0:
        return Topology(
            parent={"searcher": "beacon"},
            roles={"relay": "search", "searcher": "search"},
            min_loc_margin=direct_margin,
            certified=True,
        )

    relay_margin_1 = _edge_margin("beacon", "relay", positions, links, config)
    relay_margin_2 = _edge_margin("relay", "searcher", positions, links, config)
    relay_margin = min(relay_margin_1, relay_margin_2)
    if relay_margin >= 0.0:
        return Topology(
            parent={"relay": "beacon", "searcher": "relay"},
            roles={"relay": "relay", "searcher": "search"},
            min_loc_margin=relay_margin,
            certified=True,
        )

    return Topology(
        parent={},
        roles={"relay": "relay", "searcher": "hold"},
        min_loc_margin=max(direct_margin, relay_margin),
        certified=False,
    )


def run_link_denied_suite(config: LinkDeniedConfig) -> list[dict[str, float | str]]:
    methods = ["no_denial_fixed", "fixed_denied", "adaptive", "adaptive_relay"]
    return [_summarize(method, simulate_link_denied(method, config)) for method in methods]


def write_link_denied_artifacts(
    output_dir: Path,
    config: LinkDeniedConfig,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    methods = ["no_denial_fixed", "fixed_denied", "adaptive", "adaptive_relay"]
    simulations = [simulate_link_denied(method, config) for method in methods]
    summary_csv = output_dir / "link_denied_summary.csv"
    progress_png = output_dir / "link_denied_progress.png"
    map_png = output_dir / "link_denied_map.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "final_coverage",
                "min_loc_margin",
                "certified_ratio",
                "fail_safe_ratio",
                "relay_steps",
            ],
        )
        writer.writeheader()
        for sim in simulations:
            writer.writerow(
                {
                    "method": sim["method"],
                    "final_coverage": f"{float(sim['final_coverage']):.6f}",
                    "min_loc_margin": f"{float(sim['min_loc_margin']):.6f}",
                    "certified_ratio": f"{float(sim['certified_ratio']):.6f}",
                    "fail_safe_ratio": f"{float(sim['fail_safe_ratio']):.6f}",
                    "relay_steps": f"{float(sim['relay_steps']):.0f}",
                }
            )

    _plot_link_progress(simulations, progress_png)
    _plot_link_map(simulations, map_png)
    return {
        "summary_csv": summary_csv,
        "progress_png": progress_png,
        "map_png": map_png,
    }


def write_link_denied_2d_artifacts(
    output_dir: Path,
    config: LinkDeniedConfig,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    methods = ["fixed_denied", "adaptive", "adaptive_relay", "adaptive_relay_penalty", "adaptive_relay_predictive"]
    simulations = [simulate_link_denied_2d(method, config) for method in methods]
    summary_csv = output_dir / "link_denied_2d_summary.csv"
    switch_log_csv = output_dir / "link_denied_2d_switch_log.csv"
    progress_png = output_dir / "link_denied_2d_progress.png"
    map_png = output_dir / "link_denied_2d_map.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "final_coverage",
                "min_loc_margin",
                "certified_ratio",
                "fail_safe_ratio",
                "accepted_switches",
                "rejected_switches",
                "relay_steps",
            ],
        )
        writer.writeheader()
        for sim in simulations:
            writer.writerow(
                {
                    "method": sim["method"],
                    "final_coverage": f"{float(sim['final_coverage']):.6f}",
                    "min_loc_margin": f"{float(sim['min_loc_margin']):.6f}",
                    "certified_ratio": f"{float(sim['certified_ratio']):.6f}",
                    "fail_safe_ratio": f"{float(sim['fail_safe_ratio']):.6f}",
                    "accepted_switches": f"{float(sim['accepted_switches']):.0f}",
                    "rejected_switches": f"{float(sim['rejected_switches']):.0f}",
                    "relay_steps": f"{float(sim['relay_steps']):.0f}",
                }
            )

    with switch_log_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["method", "step", "event", "candidate", "accepted", "margin"],
        )
        writer.writeheader()
        for sim in simulations:
            for row in sim["switch_log"]:  # type: ignore[index]
                writer.writerow(row)

    _plot_link_2d_progress(simulations, progress_png)
    _plot_link_2d_map(simulations, map_png, config)
    return {
        "summary_csv": summary_csv,
        "switch_log_csv": switch_log_csv,
        "progress_png": progress_png,
        "map_png": map_png,
    }


def run_link_denied_completion_stress(config: LinkDeniedConfig) -> dict[str, object]:
    methods = ("fixed_denied", "adaptive_relay")
    simulations = [_simulate_link_denied_completion_stress(method, config) for method in methods]
    summary: list[dict[str, float | int | str]] = []
    for sim in simulations:
        completion_step = _link_completion_step(sim)
        summary.append(
            {
                "method": str(sim["method"]),
                "final_coverage": float(sim["final_coverage"]),
                "completion_step": completion_step,
                "completion_time_s": completion_step * config.dt_s,
                "min_loc_margin": float(sim["min_loc_margin"]),
                "certified_ratio": float(sim["certified_ratio"]),
                "fail_safe_ratio": float(sim["fail_safe_ratio"]),
                "relay_steps": int(sim["relay_steps"]),
            }
        )
    return {
        "metadata": link_denied_scale_metadata(config),
        "summary": summary,
        "simulations": simulations,
    }


def write_link_denied_completion_stress_artifacts(
    output_dir: Path,
    config: LinkDeniedConfig,
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_link_denied_completion_stress(config)
    summary_csv = output_dir / "link_denied_completion_stress_summary.csv"
    progress_png = output_dir / "link_denied_completion_stress_progress.png"
    map_png = output_dir / "link_denied_completion_stress_map.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "final_coverage",
                "completion_step",
                "completion_time_s",
                "min_loc_margin",
                "certified_ratio",
                "fail_safe_ratio",
                "relay_steps",
            ],
        )
        writer.writeheader()
        for row in result["summary"]:  # type: ignore[assignment]
            writer.writerow(
                {
                    "method": row["method"],
                    "final_coverage": f"{float(row['final_coverage']):.6f}",
                    "completion_step": int(row["completion_step"]),
                    "completion_time_s": f"{float(row['completion_time_s']):.2f}",
                    "min_loc_margin": f"{float(row['min_loc_margin']):.6f}",
                    "certified_ratio": f"{float(row['certified_ratio']):.6f}",
                    "fail_safe_ratio": f"{float(row['fail_safe_ratio']):.6f}",
                    "relay_steps": int(row["relay_steps"]),
                }
            )

    simulations = result["simulations"]  # type: ignore[assignment]
    _plot_link_2d_progress(simulations, progress_png)
    _plot_link_2d_map(simulations, map_png, config)
    return {"summary_csv": summary_csv, "progress_png": progress_png, "map_png": map_png}


def run_link_denied_range_sensitivity(
    config: LinkDeniedConfig,
    d_locs: Iterable[float],
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for d_loc in d_locs:
        scenario = replace(config, d_loc=float(d_loc))
        fixed = simulate_link_denied_2d("fixed_denied", scenario)
        relay = simulate_link_denied_2d("adaptive_relay", scenario)
        rows.append(
            {
                "d_loc": float(d_loc),
                "fixed_final_coverage": float(fixed["final_coverage"]),
                "relay_final_coverage": float(relay["final_coverage"]),
                "coverage_gain": float(relay["final_coverage"]) - float(fixed["final_coverage"]),
                "fixed_certified_ratio": float(fixed["certified_ratio"]),
                "relay_certified_ratio": float(relay["certified_ratio"]),
                "fixed_min_loc_margin": float(fixed["min_loc_margin"]),
                "relay_min_loc_margin": float(relay["min_loc_margin"]),
            }
        )
    return rows


def write_link_denied_range_sensitivity_artifacts(
    output_dir: Path,
    config: LinkDeniedConfig,
    d_locs: Iterable[float],
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = run_link_denied_range_sensitivity(config, d_locs)
    summary_csv = output_dir / "link_denied_range_sensitivity.csv"
    progress_png = output_dir / "link_denied_range_sensitivity.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "d_loc",
                "fixed_final_coverage",
                "relay_final_coverage",
                "coverage_gain",
                "fixed_certified_ratio",
                "relay_certified_ratio",
                "fixed_min_loc_margin",
                "relay_min_loc_margin",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "d_loc": f"{row['d_loc']:.2f}",
                    "fixed_final_coverage": f"{row['fixed_final_coverage']:.6f}",
                    "relay_final_coverage": f"{row['relay_final_coverage']:.6f}",
                    "coverage_gain": f"{row['coverage_gain']:.6f}",
                    "fixed_certified_ratio": f"{row['fixed_certified_ratio']:.6f}",
                    "relay_certified_ratio": f"{row['relay_certified_ratio']:.6f}",
                    "fixed_min_loc_margin": f"{row['fixed_min_loc_margin']:.6f}",
                    "relay_min_loc_margin": f"{row['relay_min_loc_margin']:.6f}",
                }
            )

    _plot_link_range_sensitivity(rows, progress_png, config)
    return {"summary_csv": summary_csv, "progress_png": progress_png}


def run_link_denied_severity_sensitivity(
    config: LinkDeniedConfig,
    denied_qualities: Iterable[float],
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for quality in denied_qualities:
        scenario = replace(config, denied_quality=float(quality))
        fixed = simulate_link_denied_2d("fixed_denied", scenario)
        relay = simulate_link_denied_2d("adaptive_relay", scenario)
        rows.append(
            {
                "denied_quality": float(quality),
                "fixed_final_coverage": float(fixed["final_coverage"]),
                "relay_final_coverage": float(relay["final_coverage"]),
                "coverage_gain": float(relay["final_coverage"]) - float(fixed["final_coverage"]),
                "fixed_certified_ratio": float(fixed["certified_ratio"]),
                "relay_certified_ratio": float(relay["certified_ratio"]),
                "fixed_min_loc_margin": float(fixed["min_loc_margin"]),
                "relay_min_loc_margin": float(relay["min_loc_margin"]),
            }
        )
    return rows


def write_link_denied_severity_sensitivity_artifacts(
    output_dir: Path,
    config: LinkDeniedConfig,
    denied_qualities: Iterable[float],
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = run_link_denied_severity_sensitivity(config, denied_qualities)
    summary_csv = output_dir / "link_denied_severity_sensitivity.csv"
    progress_png = output_dir / "link_denied_severity_sensitivity.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "denied_quality",
                "fixed_final_coverage",
                "relay_final_coverage",
                "coverage_gain",
                "fixed_certified_ratio",
                "relay_certified_ratio",
                "fixed_min_loc_margin",
                "relay_min_loc_margin",
            ],
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "denied_quality": f"{row['denied_quality']:.2f}",
                    "fixed_final_coverage": f"{row['fixed_final_coverage']:.6f}",
                    "relay_final_coverage": f"{row['relay_final_coverage']:.6f}",
                    "coverage_gain": f"{row['coverage_gain']:.6f}",
                    "fixed_certified_ratio": f"{row['fixed_certified_ratio']:.6f}",
                    "relay_certified_ratio": f"{row['relay_certified_ratio']:.6f}",
                    "fixed_min_loc_margin": f"{row['fixed_min_loc_margin']:.6f}",
                    "relay_min_loc_margin": f"{row['relay_min_loc_margin']:.6f}",
                }
            )

    _plot_link_severity_sensitivity(rows, progress_png)
    return {"summary_csv": summary_csv, "progress_png": progress_png}


def run_link_denied_monte_carlo(
    config: LinkDeniedConfig,
    trials: int,
    methods: tuple[str, ...] = ("fixed_denied", "adaptive_relay"),
) -> dict[str, object]:
    rng = random.Random(config.seed + 2027)
    trial_rows: list[dict[str, float | int | str]] = []
    for trial in range(trials):
        center, size = _sample_denial_zone(rng, config)
        denied_quality = rng.uniform(0.12, 0.24)
        dropout_scale = rng.uniform(0.20, 0.32)
        scenario = replace(
            config,
            seed=config.seed + trial,
            denial_center=center,
            denial_size=size,
            denied_quality=denied_quality,
            denied_dropout_scale=dropout_scale,
        )
        for method in methods:
            sim = simulate_link_denied_2d(method, scenario)
            certified_ratio = float(sim["certified_ratio"])
            final_coverage = float(sim["final_coverage"])
            trial_rows.append(
                {
                    "trial": trial,
                    "method": method,
                    "denial_center_x": center[0],
                    "denial_center_y": center[1],
                    "denial_width": size[0],
                    "denial_height": size[1],
                    "denied_quality": denied_quality,
                    "denied_dropout_scale": dropout_scale,
                    "final_coverage": final_coverage,
                    "certified_ratio": certified_ratio,
                    "success": int(certified_ratio >= 0.99 and final_coverage > 0.0),
                    "min_loc_margin": float(sim["min_loc_margin"]),
                    "accepted_switches": int(sim["accepted_switches"]),
                    "rejected_switches": int(sim["rejected_switches"]),
                    "relay_steps": int(sim["relay_steps"]),
                }
            )
    return {
        "metadata": link_denied_scale_metadata(config),
        "summary": _summarize_link_monte_carlo(trial_rows, methods),
        "trials": trial_rows,
    }


def write_link_denied_monte_carlo_artifacts(
    output_dir: Path,
    config: LinkDeniedConfig,
    trials: int,
    methods: tuple[str, ...] = ("fixed_denied", "adaptive_relay"),
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_link_denied_monte_carlo(config, trials, methods=methods)
    summary_rows = result["summary"]
    trial_rows = result["trials"]
    summary_csv = output_dir / "link_denied_monte_carlo_summary.csv"
    trials_csv = output_dir / "link_denied_monte_carlo_trials.csv"
    progress_png = output_dir / "link_denied_monte_carlo.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "trial_count",
                "successes",
                "success_rate",
                "success_rate_ci_low",
                "success_rate_ci_high",
                "mean_final_coverage",
                "mean_final_coverage_ci_half_width",
                "mean_certified_ratio",
                "mean_certified_ratio_ci_half_width",
                "mean_min_loc_margin",
                "mean_min_loc_margin_ci_half_width",
                "mean_accepted_switches",
                "mean_rejected_switches",
                "mean_relay_steps",
            ],
        )
        writer.writeheader()
        for row in summary_rows:  # type: ignore[assignment]
            writer.writerow(
                {
                    "method": row["method"],
                    "trial_count": int(row["trial_count"]),
                    "successes": int(row["successes"]),
                    "success_rate": f"{float(row['success_rate']):.6f}",
                    "success_rate_ci_low": f"{float(row['success_rate_ci_low']):.6f}",
                    "success_rate_ci_high": f"{float(row['success_rate_ci_high']):.6f}",
                    "mean_final_coverage": f"{float(row['mean_final_coverage']):.6f}",
                    "mean_final_coverage_ci_half_width": f"{float(row['mean_final_coverage_ci_half_width']):.6f}",
                    "mean_certified_ratio": f"{float(row['mean_certified_ratio']):.6f}",
                    "mean_certified_ratio_ci_half_width": f"{float(row['mean_certified_ratio_ci_half_width']):.6f}",
                    "mean_min_loc_margin": f"{float(row['mean_min_loc_margin']):.6f}",
                    "mean_min_loc_margin_ci_half_width": f"{float(row['mean_min_loc_margin_ci_half_width']):.6f}",
                    "mean_accepted_switches": f"{float(row['mean_accepted_switches']):.2f}",
                    "mean_rejected_switches": f"{float(row['mean_rejected_switches']):.2f}",
                    "mean_relay_steps": f"{float(row['mean_relay_steps']):.2f}",
                }
            )

    with trials_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "trial",
                "method",
                "denial_center_x",
                "denial_center_y",
                "denial_width",
                "denial_height",
                "denied_quality",
                "denied_dropout_scale",
                "final_coverage",
                "certified_ratio",
                "success",
                "min_loc_margin",
                "accepted_switches",
                "rejected_switches",
                "relay_steps",
            ],
        )
        writer.writeheader()
        for row in trial_rows:  # type: ignore[assignment]
            writer.writerow(
                {
                    "trial": int(row["trial"]),
                    "method": row["method"],
                    "denial_center_x": f"{float(row['denial_center_x']):.3f}",
                    "denial_center_y": f"{float(row['denial_center_y']):.3f}",
                    "denial_width": f"{float(row['denial_width']):.3f}",
                    "denial_height": f"{float(row['denial_height']):.3f}",
                    "denied_quality": f"{float(row['denied_quality']):.3f}",
                    "denied_dropout_scale": f"{float(row['denied_dropout_scale']):.3f}",
                    "final_coverage": f"{float(row['final_coverage']):.6f}",
                    "certified_ratio": f"{float(row['certified_ratio']):.6f}",
                    "success": int(row["success"]),
                    "min_loc_margin": f"{float(row['min_loc_margin']):.6f}",
                    "accepted_switches": int(row["accepted_switches"]),
                    "rejected_switches": int(row["rejected_switches"]),
                    "relay_steps": int(row["relay_steps"]),
                }
            )

    _plot_link_monte_carlo(summary_rows, progress_png)  # type: ignore[arg-type]
    return {"summary_csv": summary_csv, "trials_csv": trials_csv, "progress_png": progress_png}


def run_link_denied_topology_ablation(
    config: LinkDeniedConfig,
    trials: int,
    methods: tuple[str, ...] = ("fixed_direct", "relay_candidate", "adaptive_topology"),
) -> dict[str, object]:
    rng = random.Random(config.seed + 3031)
    trial_rows: list[dict[str, float | int | str]] = []
    for trial in range(trials):
        center, size = _sample_denial_zone(rng, config)
        denied_quality = rng.uniform(0.12, 0.24)
        dropout_scale = rng.uniform(0.20, 0.32)
        scenario = replace(
            config,
            seed=config.seed + trial,
            denial_center=center,
            denial_size=size,
            denied_quality=denied_quality,
            denied_dropout_scale=dropout_scale,
        )
        metrics = _simulate_topology_only_trial(scenario, methods)
        for row in metrics:
            trial_rows.append(
                {
                    "trial": trial,
                    "method": row["method"],
                    "denial_center_x": center[0],
                    "denial_center_y": center[1],
                    "denial_width": size[0],
                    "denial_height": size[1],
                    "denied_quality": denied_quality,
                    "denied_dropout_scale": dropout_scale,
                    "certified_ratio": row["certified_ratio"],
                    "full_certified": row["full_certified"],
                    "min_margin": row["min_margin"],
                    "relay_active_ratio": row["relay_active_ratio"],
                    "recovery_ratio": row["recovery_ratio"],
                    "direct_certified_ratio": row["direct_certified_ratio"],
                    "relay_certified_ratio": row["relay_certified_ratio"],
                }
            )
    return {
        "metadata": link_denied_scale_metadata(config),
        "summary": _summarize_topology_ablation(trial_rows, methods),
        "trials": trial_rows,
    }


def write_link_denied_topology_ablation_artifacts(
    output_dir: Path,
    config: LinkDeniedConfig,
    trials: int,
    methods: tuple[str, ...] = ("fixed_direct", "relay_candidate", "adaptive_topology"),
) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    result = run_link_denied_topology_ablation(config, trials, methods=methods)
    summary_rows = result["summary"]
    trial_rows = result["trials"]
    summary_csv = output_dir / "link_denied_topology_ablation_summary.csv"
    trials_csv = output_dir / "link_denied_topology_ablation_trials.csv"
    progress_png = output_dir / "link_denied_topology_ablation.png"

    with summary_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "method",
                "trial_count",
                "fully_certified_trials",
                "full_certification_rate",
                "full_certification_ci_low",
                "full_certification_ci_high",
                "mean_certified_ratio",
                "mean_certified_ratio_ci_half_width",
                "mean_min_margin",
                "mean_min_margin_ci_half_width",
                "mean_relay_active_ratio",
                "mean_recovery_ratio",
                "mean_direct_certified_ratio",
                "mean_relay_certified_ratio",
            ],
        )
        writer.writeheader()
        for row in summary_rows:  # type: ignore[assignment]
            writer.writerow(
                {
                    "method": row["method"],
                    "trial_count": int(row["trial_count"]),
                    "fully_certified_trials": int(row["fully_certified_trials"]),
                    "full_certification_rate": f"{float(row['full_certification_rate']):.6f}",
                    "full_certification_ci_low": f"{float(row['full_certification_ci_low']):.6f}",
                    "full_certification_ci_high": f"{float(row['full_certification_ci_high']):.6f}",
                    "mean_certified_ratio": f"{float(row['mean_certified_ratio']):.6f}",
                    "mean_certified_ratio_ci_half_width": f"{float(row['mean_certified_ratio_ci_half_width']):.6f}",
                    "mean_min_margin": f"{float(row['mean_min_margin']):.6f}",
                    "mean_min_margin_ci_half_width": f"{float(row['mean_min_margin_ci_half_width']):.6f}",
                    "mean_relay_active_ratio": f"{float(row['mean_relay_active_ratio']):.6f}",
                    "mean_recovery_ratio": f"{float(row['mean_recovery_ratio']):.6f}",
                    "mean_direct_certified_ratio": f"{float(row['mean_direct_certified_ratio']):.6f}",
                    "mean_relay_certified_ratio": f"{float(row['mean_relay_certified_ratio']):.6f}",
                }
            )

    with trials_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "trial",
                "method",
                "denial_center_x",
                "denial_center_y",
                "denial_width",
                "denial_height",
                "denied_quality",
                "denied_dropout_scale",
                "certified_ratio",
                "full_certified",
                "min_margin",
                "relay_active_ratio",
                "recovery_ratio",
                "direct_certified_ratio",
                "relay_certified_ratio",
            ],
        )
        writer.writeheader()
        for row in trial_rows:  # type: ignore[assignment]
            writer.writerow(
                {
                    "trial": int(row["trial"]),
                    "method": row["method"],
                    "denial_center_x": f"{float(row['denial_center_x']):.3f}",
                    "denial_center_y": f"{float(row['denial_center_y']):.3f}",
                    "denial_width": f"{float(row['denial_width']):.3f}",
                    "denial_height": f"{float(row['denial_height']):.3f}",
                    "denied_quality": f"{float(row['denied_quality']):.3f}",
                    "denied_dropout_scale": f"{float(row['denied_dropout_scale']):.3f}",
                    "certified_ratio": f"{float(row['certified_ratio']):.6f}",
                    "full_certified": int(row["full_certified"]),
                    "min_margin": f"{float(row['min_margin']):.6f}",
                    "relay_active_ratio": f"{float(row['relay_active_ratio']):.6f}",
                    "recovery_ratio": f"{float(row['recovery_ratio']):.6f}",
                    "direct_certified_ratio": f"{float(row['direct_certified_ratio']):.6f}",
                    "relay_certified_ratio": f"{float(row['relay_certified_ratio']):.6f}",
                }
            )

    _plot_topology_ablation(summary_rows, progress_png)  # type: ignore[arg-type]
    return {"summary_csv": summary_csv, "trials_csv": trials_csv, "progress_png": progress_png}


def _sample_denial_zone(rng: random.Random, config: LinkDeniedConfig) -> tuple[Point, Point]:
    center = (
        rng.uniform(config.width * 0.46, config.width * 0.62),
        rng.uniform(config.height * 0.43, config.height * 0.57),
    )
    size = (
        rng.uniform(config.width * 0.30, config.width * 0.42),
        rng.uniform(config.height * 0.46, config.height * 0.62),
    )
    return center, size


def _summarize_link_monte_carlo(
    trial_rows: list[dict[str, float | int | str]],
    methods: tuple[str, ...],
) -> list[dict[str, float | int | str]]:
    summary: list[dict[str, float | int | str]] = []
    for method in methods:
        rows = [row for row in trial_rows if row["method"] == method]
        successes = sum(int(row["success"]) for row in rows)
        coverage_values = [float(row["final_coverage"]) for row in rows]
        certified_values = [float(row["certified_ratio"]) for row in rows]
        min_loc_values = [float(row["min_loc_margin"]) for row in rows]
        ci_low, ci_high = _wilson_interval(successes, len(rows))
        summary.append(
            {
                "method": method,
                "trial_count": len(rows),
                "successes": successes,
                "success_rate": successes / len(rows) if rows else 0.0,
                "success_rate_ci_low": ci_low,
                "success_rate_ci_high": ci_high,
                "mean_final_coverage": _mean(coverage_values, default=0.0),
                "mean_final_coverage_ci_half_width": _mean_ci_half_width(coverage_values),
                "mean_certified_ratio": _mean(certified_values, default=0.0),
                "mean_certified_ratio_ci_half_width": _mean_ci_half_width(certified_values),
                "mean_min_loc_margin": _mean(min_loc_values, default=0.0),
                "mean_min_loc_margin_ci_half_width": _mean_ci_half_width(min_loc_values),
                "mean_accepted_switches": _mean([float(row["accepted_switches"]) for row in rows], default=0.0),
                "mean_rejected_switches": _mean([float(row["rejected_switches"]) for row in rows], default=0.0),
                "mean_relay_steps": _mean([float(row["relay_steps"]) for row in rows], default=0.0),
            }
        )
    return summary


def _simulate_topology_only_trial(
    config: LinkDeniedConfig,
    methods: tuple[str, ...],
) -> list[dict[str, float | int | str]]:
    total_frames = config.steps + 1
    direct_certified_steps = 0
    relay_certified_steps = 0
    direct_failed_steps = 0
    relay_recovery_steps = 0
    stats = {
        method: {
            "certified_steps": 0,
            "relay_active_steps": 0,
            "recovery_steps": 0,
            "min_margin": math.inf,
        }
        for method in methods
    }

    for step in range(total_frames):
        beacon, relay, searchers = _topology_only_positions(step, config)
        direct_margin = _direct_2d_margin(beacon, searchers, config)
        relay_margin = _relay_2d_margin(beacon, relay, searchers, config)
        direct_certified = direct_margin >= 0.0
        relay_certified = relay_margin >= 0.0
        direct_certified_steps += int(direct_certified)
        relay_certified_steps += int(relay_certified)
        direct_failed = not direct_certified
        direct_failed_steps += int(direct_failed)
        relay_recovery = direct_failed and relay_certified
        relay_recovery_steps += int(relay_recovery)

        for method in methods:
            certified, selected_margin, relay_active, recovered = _topology_method_decision(
                method,
                direct_certified=direct_certified,
                relay_certified=relay_certified,
                direct_margin=direct_margin,
                relay_margin=relay_margin,
            )
            stats[method]["certified_steps"] += int(certified)
            stats[method]["relay_active_steps"] += int(relay_active)
            stats[method]["recovery_steps"] += int(recovered)
            stats[method]["min_margin"] = min(float(stats[method]["min_margin"]), selected_margin)

    direct_certified_ratio = direct_certified_steps / total_frames
    relay_certified_ratio = relay_certified_steps / total_frames
    rows: list[dict[str, float | int | str]] = []
    for method in methods:
        certified_steps = int(stats[method]["certified_steps"])
        rows.append(
            {
                "method": method,
                "certified_ratio": certified_steps / total_frames,
                "full_certified": int(certified_steps == total_frames),
                "min_margin": float(stats[method]["min_margin"]),
                "relay_active_ratio": float(stats[method]["relay_active_steps"]) / total_frames,
                "recovery_ratio": (
                    float(stats[method]["recovery_steps"]) / direct_failed_steps
                    if direct_failed_steps > 0
                    else 0.0
                ),
                "direct_certified_ratio": direct_certified_ratio,
                "relay_certified_ratio": relay_certified_ratio,
                "available_recovery_ratio": (
                    relay_recovery_steps / direct_failed_steps
                    if direct_failed_steps > 0
                    else 0.0
                ),
            }
        )
    return rows


def _topology_only_positions(
    step: int,
    config: LinkDeniedConfig,
) -> tuple[Point, Point, dict[str, Point]]:
    beacon = (2.0, config.height / 2.0)
    relay_x = min(config.d_loc - 0.45, 4.2 + step * config.max_step * 0.45)
    searcher_x_nominal = 5.2 + step * config.max_step * 0.95
    searcher_x = min(searcher_x_nominal, relay_x + config.d_loc - 1.10, config.width - 1.0)
    relay = (relay_x, config.height / 2.0)
    searchers = {
        "searcher_1": (searcher_x, config.height * 0.45),
        "searcher_2": (searcher_x, config.height * 0.55),
    }
    return beacon, relay, searchers


def _topology_method_decision(
    method: str,
    *,
    direct_certified: bool,
    relay_certified: bool,
    direct_margin: float,
    relay_margin: float,
) -> tuple[bool, float, bool, bool]:
    if method == "fixed_direct":
        return direct_certified, direct_margin, False, False
    if method == "relay_candidate":
        return relay_certified, relay_margin, True, (not direct_certified and relay_certified)
    if method == "adaptive_topology":
        if direct_certified:
            return True, direct_margin, False, False
        if relay_certified:
            return True, relay_margin, True, True
        return False, max(direct_margin, relay_margin), False, False
    raise ValueError(f"Unknown topology-only method: {method}")


def _summarize_topology_ablation(
    trial_rows: list[dict[str, float | int | str]],
    methods: tuple[str, ...],
) -> list[dict[str, float | int | str]]:
    summary: list[dict[str, float | int | str]] = []
    for method in methods:
        rows = [row for row in trial_rows if row["method"] == method]
        fully_certified = sum(int(row["full_certified"]) for row in rows)
        ci_low, ci_high = _wilson_interval(fully_certified, len(rows))
        certified_values = [float(row["certified_ratio"]) for row in rows]
        min_margin_values = [float(row["min_margin"]) for row in rows]
        summary.append(
            {
                "method": method,
                "trial_count": len(rows),
                "fully_certified_trials": fully_certified,
                "full_certification_rate": fully_certified / len(rows) if rows else 0.0,
                "full_certification_ci_low": ci_low,
                "full_certification_ci_high": ci_high,
                "mean_certified_ratio": _mean(certified_values, default=0.0),
                "mean_certified_ratio_ci_half_width": _mean_ci_half_width(certified_values),
                "mean_min_margin": _mean(min_margin_values, default=0.0),
                "mean_min_margin_ci_half_width": _mean_ci_half_width(min_margin_values),
                "mean_relay_active_ratio": _mean([float(row["relay_active_ratio"]) for row in rows], default=0.0),
                "mean_recovery_ratio": _mean([float(row["recovery_ratio"]) for row in rows], default=0.0),
                "mean_direct_certified_ratio": _mean([float(row["direct_certified_ratio"]) for row in rows], default=0.0),
                "mean_relay_certified_ratio": _mean([float(row["relay_certified_ratio"]) for row in rows], default=0.0),
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



def simulate_link_denied(method: str, config: LinkDeniedConfig) -> dict[str, object]:
    beacon = (0.0, config.height / 2.0)
    relay = (config.d_loc * 0.42, config.height / 2.0)
    searcher = (config.d_loc * 0.74, config.height / 2.0)
    searched: set[tuple[int, int]] = set()
    history: list[dict[str, object]] = []
    min_margin = math.inf
    certified_steps = 0
    fail_safe_steps = 0
    relay_steps = 0

    for step in range(config.steps + 1):
        positions = {"beacon": beacon, "relay": relay, "searcher": searcher}
        links = _links_for_state(method, positions, config)
        topology = (
            _fixed_topology(positions, links, config)
            if method in {"no_denial_fixed", "fixed_denied", "adaptive"}
            else select_adaptive_topology(positions, links, config)
        )
        min_margin = min(min_margin, topology.min_loc_margin)
        certified_steps += int(topology.certified)
        fail_safe_steps += int(not topology.certified)
        relay_steps += int(topology.roles.get("relay") == "relay")
        _observe(searcher, searched, config)
        history.append(
            {
                "step": step,
                "beacon": beacon,
                "relay": relay,
                "searcher": searcher,
                "topology": topology,
                "coverage": len(searched) / (config.width * config.height),
                "max_uncertainty": _max_uncertainty(links, config),
            }
        )
        if step == config.steps:
            break
        if topology.certified:
            relay, searcher = _advance(method, relay, searcher, config)
        elif method == "fixed_denied":
            # Fixed topology enters a conservative hold mode once certification fails.
            relay = relay
            searcher = searcher

    total_frames = config.steps + 1
    return {
        "method": method,
        "final_coverage": len(searched) / (config.width * config.height),
        "min_loc_margin": min_margin,
        "certified_ratio": certified_steps / total_frames,
        "fail_safe_ratio": fail_safe_steps / total_frames,
        "relay_steps": relay_steps,
        "history": history,
    }


def simulate_link_denied_2d(method: str, config: LinkDeniedConfig) -> dict[str, object]:
    beacon = (2.0, config.height / 2.0)
    relay = (4.2, config.height / 2.0)
    searchers = {
        "searcher_1": (5.2, config.height * 0.45),
        "searcher_2": (5.2, config.height * 0.55),
    }
    searched: set[tuple[int, int]] = set()
    history: list[dict[str, object]] = []
    switch_log: list[dict[str, object]] = []
    fail_safe_commands: list[str] = []
    active_graph = "direct"
    min_margin = math.inf
    certified_steps = 0
    fail_safe_steps = 0
    relay_steps = 0
    accepted_switches = 0
    rejected_switches = 0

    for step in range(config.steps + 1):
        direct_margin = _direct_2d_margin(beacon, searchers, config)
        relay_margin = _relay_2d_margin(beacon, relay, searchers, config)
        direct_certified = direct_margin >= 0.0
        relay_certified = relay_margin >= 0.0
        selected_graph = "direct"
        certified = direct_certified
        selected_margin = direct_margin

        predictive_direct_risk = (
            method == "adaptive_relay_predictive"
            and direct_margin < config.predictive_margin_reserve
        )
        if method in {"adaptive_relay", "adaptive_relay_penalty", "adaptive_relay_predictive"} and (
            not direct_certified or predictive_direct_risk
        ):
            rejected_switches += 1
            switch_log.append(
                {
                    "method": method,
                    "step": step,
                    "event": "rejected",
                    "candidate": "direct",
                    "accepted": 0,
                    "margin": f"{direct_margin:.6f}",
                }
            )
            relay_has_reserve = relay_margin >= config.predictive_margin_reserve
            if relay_certified and (method != "adaptive_relay_predictive" or relay_has_reserve or not direct_certified):
                selected_graph = "relay"
                selected_margin = relay_margin
                certified = True
                if active_graph != "relay":
                    accepted_switches += 1
                    switch_log.append(
                        {
                            "method": method,
                            "step": step,
                            "event": "accepted",
                            "candidate": "relay",
                            "accepted": 1,
                            "margin": f"{relay_margin:.6f}",
                        }
                    )
                active_graph = "relay"
            elif direct_certified:
                selected_graph = "direct"
                selected_margin = direct_margin
                certified = True
            else:
                selected_graph = "fail_safe"
                selected_margin = max(direct_margin, relay_margin)
                certified = False
                fail_safe_commands.append("hold")
        elif method == "adaptive" and not direct_certified:
            rejected_switches += 1
            switch_log.append(
                {
                    "method": method,
                    "step": step,
                    "event": "rejected",
                    "candidate": "direct",
                    "accepted": 0,
                    "margin": f"{direct_margin:.6f}",
                }
            )
            selected_graph = "fail_safe"
            selected_margin = direct_margin
            certified = False
            fail_safe_commands.append("hold")
        elif method == "fixed_denied" and not direct_certified:
            selected_graph = "fail_safe"
            certified = False
            selected_margin = direct_margin
            fail_safe_commands.append("hold")

        min_margin = min(min_margin, selected_margin)
        certified_steps += int(certified)
        fail_safe_steps += int(not certified)
        relay_steps += int(selected_graph == "relay")
        for position in searchers.values():
            _observe(position, searched, config)

        history.append(
            {
                "step": step,
                "beacon": beacon,
                "relay": relay,
                "searchers": dict(searchers),
                "topology": selected_graph,
                "coverage": len(searched) / (config.width * config.height),
                "min_margin": selected_margin,
                "direct_margin": direct_margin,
                "relay_margin": relay_margin,
            }
        )
        if step == config.steps:
            break

        if certified:
            relay, searchers = _advance_2d(method, relay, searchers, selected_graph, config)

    total_frames = config.steps + 1
    return {
        "method": method,
        "searcher_names": sorted(searchers),
        "final_coverage": len(searched) / (config.width * config.height),
        "min_loc_margin": max(0.0, min_margin) if min_margin > -1e-9 else min_margin,
        "certified_ratio": certified_steps / total_frames,
        "fail_safe_ratio": fail_safe_steps / total_frames,
        "relay_steps": relay_steps,
        "accepted_switches": accepted_switches,
        "rejected_switches": rejected_switches,
        "fail_safe_commands": fail_safe_commands,
        "searched_cells": sorted(searched),
        "history": history,
        "switch_log": switch_log,
    }


def _simulate_link_denied_completion_stress(method: str, config: LinkDeniedConfig) -> dict[str, object]:
    beacon = (2.0, config.height / 2.0)
    relay = (min(config.d_loc - 0.45, config.width * 0.50), config.height / 2.0)
    searchers = {
        "searcher_1": (5.2, 2.0),
        "searcher_2": (5.2, config.height - 2.0),
    }
    waypoint_indices = {name: 0 for name in searchers}
    waypoints = {
        "searcher_1": _completion_waypoints(config, upper=False),
        "searcher_2": _completion_waypoints(config, upper=True),
    }
    searched: set[tuple[int, int]] = set()
    history: list[dict[str, object]] = []
    switch_log: list[dict[str, object]] = []
    fail_safe_commands: list[str] = []
    min_margin = math.inf
    certified_steps = 0
    fail_safe_steps = 0
    relay_steps = 0
    accepted_switches = 0

    for step in range(config.steps + 1):
        direct_margin = _direct_2d_margin(beacon, searchers, config)
        relay_margin = _relay_2d_margin(beacon, relay, searchers, config)
        if method == "adaptive_relay":
            selected_graph = "relay" if relay_margin >= 0.0 else "fail_safe"
            selected_margin = relay_margin
            certified = relay_margin >= 0.0
            if step == 0 and certified:
                accepted_switches = 1
                switch_log.append(
                    {
                        "method": method,
                        "step": step,
                        "event": "accepted",
                        "candidate": "relay",
                        "accepted": 1,
                        "margin": f"{relay_margin:.6f}",
                    }
                )
        else:
            selected_graph = "direct" if direct_margin >= 0.0 else "fail_safe"
            selected_margin = direct_margin
            certified = direct_margin >= 0.0

        if not certified:
            fail_safe_commands.append("hold")
        min_margin = min(min_margin, selected_margin)
        certified_steps += int(certified)
        fail_safe_steps += int(not certified)
        relay_steps += int(selected_graph == "relay")

        for position in searchers.values():
            _observe(position, searched, config)
        history.append(
            {
                "step": step,
                "beacon": beacon,
                "relay": relay,
                "searchers": dict(searchers),
                "topology": selected_graph,
                "coverage": len(searched) / (config.width * config.height),
                "min_margin": selected_margin,
                "direct_margin": direct_margin,
                "relay_margin": relay_margin,
            }
        )
        if step == config.steps:
            break
        if certified:
            searchers = _advance_completion_searchers(searchers, waypoint_indices, waypoints, config)

    total_frames = config.steps + 1
    return {
        "method": method,
        "searcher_names": sorted(searchers),
        "final_coverage": len(searched) / (config.width * config.height),
        "min_loc_margin": max(0.0, min_margin) if min_margin > -1e-9 else min_margin,
        "certified_ratio": certified_steps / total_frames,
        "fail_safe_ratio": fail_safe_steps / total_frames,
        "relay_steps": relay_steps,
        "accepted_switches": accepted_switches,
        "rejected_switches": 0,
        "fail_safe_commands": fail_safe_commands,
        "searched_cells": sorted(searched),
        "history": history,
        "switch_log": switch_log,
    }


def _edge_margin(
    parent: str,
    child: str,
    positions: dict[str, Point],
    links: dict[tuple[str, str], LinkState],
    config: LinkDeniedConfig,
) -> float:
    link = links.get((parent, child))
    if link is None:
        return -math.inf
    variance = effective_range_variance(link, config)
    if not math.isfinite(variance):
        return -math.inf
    uncertainty = config.kappa * math.sqrt(variance)
    return config.d_loc - distance(positions[parent], positions[child]) - uncertainty


def _fixed_topology(
    positions: dict[str, Point],
    links: dict[tuple[str, str], LinkState],
    config: LinkDeniedConfig,
) -> Topology:
    margin = _edge_margin("beacon", "searcher", positions, links, config)
    return Topology(
        parent={"searcher": "beacon"},
        roles={"relay": "search", "searcher": "search"},
        min_loc_margin=margin,
        certified=margin >= 0.0,
    )


def _links_for_state(
    method: str,
    positions: dict[str, Point],
    config: LinkDeniedConfig,
) -> dict[tuple[str, str], LinkState]:
    denied = method != "no_denial_fixed" and positions["searcher"][0] > config.d_loc * 0.95
    direct_quality = 1.0 if not denied else 0.12
    direct_dropout = 0.0 if not denied else max(0.0, positions["searcher"][0] - config.d_loc)
    return {
        ("beacon", "searcher"): LinkState(True, direct_quality, direct_dropout, 0.4 if denied else 0.0, 0.0),
        ("beacon", "relay"): LinkState(True, 0.92, 0.0, 0.0, 0.0),
        ("relay", "searcher"): LinkState(True, 0.86, 0.0, 0.0, 0.12),
    }


def _advance(
    method: str,
    relay: Point,
    searcher: Point,
    config: LinkDeniedConfig,
) -> tuple[Point, Point]:
    if method == "adaptive_relay":
        desired_search_x = min(config.width - 1.0, searcher[0] + config.max_step)
        max_search_x = relay[0] + config.d_loc - 0.35
        new_searcher = (min(desired_search_x, max_search_x), searcher[1])
        desired_relay_x = min(config.d_loc - 0.35, new_searcher[0] - config.d_loc * 0.72)
        new_relay = (max(relay[0], desired_relay_x), relay[1])
        return new_relay, new_searcher
    if method == "adaptive":
        max_search_x = config.d_loc * 1.45
        return relay, (min(max_search_x, searcher[0] + config.max_step * 0.55), searcher[1])
    if method == "no_denial_fixed":
        max_search_x = config.d_loc - 0.35
        return relay, (min(max_search_x, searcher[0] + config.max_step * 0.45), searcher[1])
    return relay, (min(config.width - 1.0, searcher[0] + config.max_step), searcher[1])


def _observe(position: Point, searched: set[tuple[int, int]], config: LinkDeniedConfig) -> None:
    for y in range(config.height):
        for x in range(config.width):
            if distance(position, (x, y)) <= config.sensor_radius:
                searched.add((x, y))


def _completion_waypoints(config: LinkDeniedConfig, *, upper: bool) -> list[Point]:
    if upper:
        y_values = [config.height - 2.0, config.height - 6.5, config.height - 11.0, config.height - 14.5]
    else:
        y_values = [2.0, 6.5, 11.0, 14.5]
    waypoints: list[Point] = []
    right = config.width - 2.0
    left = 1.0
    for idx, y in enumerate(y_values):
        if idx % 2 == 0:
            waypoints.append((right, y))
            waypoints.append((left, y))
        else:
            waypoints.append((left, y))
            waypoints.append((right, y))
    return waypoints


def _advance_completion_searchers(
    searchers: dict[str, Point],
    waypoint_indices: dict[str, int],
    waypoints: dict[str, list[Point]],
    config: LinkDeniedConfig,
) -> dict[str, Point]:
    moved: dict[str, Point] = {}
    for name, position in searchers.items():
        route = waypoints[name]
        idx = min(waypoint_indices[name], len(route) - 1)
        target = route[idx]
        if distance(position, target) <= config.max_step:
            position = target
            waypoint_indices[name] = min(idx + 1, len(route) - 1)
            target = route[waypoint_indices[name]]
        moved[name] = _move_toward(position, target, config.max_step)
    return moved


def _move_toward(current: Point, target: Point, max_step: float) -> Point:
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    norm = math.hypot(dx, dy)
    if norm <= max_step or norm == 0.0:
        return target
    return (current[0] + dx / norm * max_step, current[1] + dy / norm * max_step)


def _link_completion_step(simulation: dict[str, object]) -> int:
    history = simulation["history"]
    for frame in history:  # type: ignore[assignment]
        if float(frame["coverage"]) >= 1.0:
            return int(frame["step"])
    return int(history[-1]["step"]) + 1  # type: ignore[index]


def _max_uncertainty(
    links: dict[tuple[str, str], LinkState],
    config: LinkDeniedConfig,
) -> float:
    finite = [
        config.kappa * math.sqrt(effective_range_variance(link, config))
        for link in links.values()
        if math.isfinite(effective_range_variance(link, config))
    ]
    return max(finite) if finite else math.inf


def _denial_rect(config: LinkDeniedConfig) -> tuple[float, float, float, float]:
    if config.denial_center is None:
        center_x = config.width * 0.525
        center_y = config.height * 0.50
    else:
        center_x, center_y = config.denial_center
    if config.denial_size is None:
        width = config.width * 0.35
        height = config.height * 0.56
    else:
        width, height = config.denial_size
    x0 = min(max(center_x - width / 2.0, 0.0), config.width)
    x1 = min(max(center_x + width / 2.0, 0.0), config.width)
    y0 = min(max(center_y - height / 2.0, 0.0), config.height)
    y1 = min(max(center_y + height / 2.0, 0.0), config.height)
    return x0, x1, y0, y1


def _inside_denial_zone(position: Point, config: LinkDeniedConfig) -> bool:
    x0, x1, y0, y1 = _denial_rect(config)
    return x0 <= position[0] <= x1 and y0 <= position[1] <= y1


def _link_state_2d(parent: Point, child: Point, config: LinkDeniedConfig) -> LinkState:
    denied = _inside_denial_zone(child, config)
    quality = config.denied_quality if denied else config.healthy_quality
    dropout = max(0.0, child[0] - _denial_rect(config)[0]) * config.denied_dropout_scale if denied else 0.0
    delay = config.denied_delay if denied else 0.0
    reference_variance = config.denied_reference_variance if _inside_denial_zone(parent, config) else 0.0
    return LinkState(True, quality, dropout, delay, reference_variance)


def _margin_between(parent: Point, child: Point, config: LinkDeniedConfig) -> float:
    link = _link_state_2d(parent, child, config)
    variance = effective_range_variance(link, config)
    uncertainty = config.kappa * math.sqrt(variance)
    return config.d_loc - distance(parent, child) - uncertainty


def _direct_2d_margin(
    beacon: Point,
    searchers: dict[str, Point],
    config: LinkDeniedConfig,
) -> float:
    return min(_margin_between(beacon, position, config) for position in searchers.values())


def _relay_2d_margin(
    beacon: Point,
    relay: Point,
    searchers: dict[str, Point],
    config: LinkDeniedConfig,
) -> float:
    margins = [_margin_between(beacon, relay, config)]
    margins.extend(_margin_between(relay, position, config) for position in searchers.values())
    return min(margins)


def _advance_2d(
    method: str,
    relay: Point,
    searchers: dict[str, Point],
    topology: str,
    config: LinkDeniedConfig,
) -> tuple[Point, dict[str, Point]]:
    step_scale = 0.65 if method in {"adaptive_relay_penalty", "adaptive_relay_predictive"} else 1.0
    if topology == "relay":
        desired_relay_x = min(config.d_loc - 0.45, relay[0] + config.max_step * 0.45 * step_scale)
        relay = (desired_relay_x, relay[1])
        supported_x = relay[0] + config.d_loc - 1.10
        step = config.max_step * 0.95 * step_scale
    else:
        relay = (min(config.d_loc - 0.45, relay[0] + config.max_step * 0.24), relay[1])
        supported_x = config.width - 1.0
        step = config.max_step * (0.50 if method == "fixed_denied" else 0.70)

    moved: dict[str, Point] = {}
    for name, position in searchers.items():
        target_x = min(position[0] + step, supported_x, config.width - 1.0)
        target_y = min(max(position[1], config.height * 0.40), config.height * 0.60)
        moved[name] = (target_x, target_y)
    return relay, moved


def _summarize(method: str, result: dict[str, object]) -> dict[str, float | str]:
    return {
        "method": method,
        "final_coverage": float(result["final_coverage"]),
        "min_loc_margin": float(result["min_loc_margin"]),
        "certified_ratio": float(result["certified_ratio"]),
        "fail_safe_ratio": float(result["fail_safe_ratio"]),
        "relay_steps": float(result["relay_steps"]),
    }


def _plot_link_progress(simulations: list[dict[str, object]], output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(6.2, 5.0), sharex=True)
    for sim in simulations:
        history = sim["history"]
        steps = [frame["step"] for frame in history]  # type: ignore[index]
        coverage = [100.0 * frame["coverage"] for frame in history]  # type: ignore[index]
        uncertainty = [frame["max_uncertainty"] for frame in history]  # type: ignore[index]
        axes[0].plot(steps, coverage, marker="o", markersize=2.4, label=str(sim["method"]))
        axes[1].plot(steps, uncertainty, marker="s", markersize=2.2, label=str(sim["method"]))
    axes[0].set_ylabel("Coverage / %")
    axes[1].set_ylabel("Max uncertainty")
    axes[1].set_xlabel("Step")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_link_map(simulations: list[dict[str, object]], output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, axes = plt.subplots(1, 2, figsize=(8.0, 3.4), sharey=True)
    selected = [simulations[1], simulations[-1]]
    titles = ["Fixed graph under denial", "Adaptive relay topology"]
    for ax, sim, title in zip(axes, selected, titles):
        history = sim["history"]
        xs = [frame["searcher"][0] for frame in history]  # type: ignore[index]
        ys = [frame["searcher"][1] for frame in history]  # type: ignore[index]
        relay_xs = [frame["relay"][0] for frame in history]  # type: ignore[index]
        relay_ys = [frame["relay"][1] for frame in history]  # type: ignore[index]
        ax.add_patch(Rectangle((8.0, 0.0), 8.0, 20.0, color="#ffcccc", alpha=0.35, label="denial zone"))
        ax.plot(xs, ys, color="#1f77b4", marker="o", markersize=2.2, label="searcher")
        ax.plot(relay_xs, relay_ys, color="#2ca02c", marker="s", markersize=2.2, label="relay")
        ax.scatter([0.0], [ys[0]], color="black", marker="D", label="beacon")
        ax.set_title(title)
        ax.set_xlabel("x cell")
        ax.grid(True, alpha=0.25)
    axes[0].set_ylabel("y cell")
    axes[0].legend(fontsize=7, loc="lower right")
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_link_2d_progress(simulations: list[dict[str, object]], output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True)
    for sim in simulations:
        history = sim["history"]
        steps = [frame["step"] for frame in history]  # type: ignore[index]
        coverage = [100.0 * frame["coverage"] for frame in history]  # type: ignore[index]
        margins = [frame["min_margin"] for frame in history]  # type: ignore[index]
        axes[0].plot(steps, coverage, marker="o", markersize=2.2, label=str(sim["method"]))
        axes[1].plot(steps, margins, marker="s", markersize=2.0, label=str(sim["method"]))
    axes[0].set_ylabel("Coverage / %")
    axes[1].set_ylabel("Min loc. margin")
    axes[1].set_xlabel("Step")
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_link_2d_map(
    simulations: list[dict[str, object]],
    output_path: Path,
    config: LinkDeniedConfig,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    selected = [sim for sim in simulations if sim["method"] in {"fixed_denied", "adaptive_relay"}]
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 3.8), sharex=True, sharey=True)
    x0, x1, y0, y1 = _denial_rect(config)
    for ax, sim in zip(axes, selected):
        history = sim["history"]
        final = history[-1]  # type: ignore[index]
        searched_cells = sim["searched_cells"]
        if searched_cells:
            xs = [cell[0] for cell in searched_cells]  # type: ignore[index]
            ys = [cell[1] for cell in searched_cells]  # type: ignore[index]
            ax.scatter(xs, ys, s=8, color="#f2c14e", alpha=0.45, marker="s", label="searched cells")
        ax.add_patch(
            Rectangle(
                (x0, y0),
                x1 - x0,
                y1 - y0,
                color="#ef476f",
                alpha=0.18,
                label="denial zone",
            )
        )
        relay_xs = [frame["relay"][0] for frame in history]  # type: ignore[index]
        relay_ys = [frame["relay"][1] for frame in history]  # type: ignore[index]
        ax.plot(relay_xs, relay_ys, color="#118ab2", marker="s", markersize=2.2, label="relay")
        searcher_names = sim["searcher_names"]
        for name in searcher_names:  # type: ignore[assignment]
            xs = [frame["searchers"][name][0] for frame in history]  # type: ignore[index]
            ys = [frame["searchers"][name][1] for frame in history]  # type: ignore[index]
            ax.plot(xs, ys, marker="o", markersize=2.0, linewidth=1.2, label=str(name))
        beacon = final["beacon"]  # type: ignore[index]
        relay = final["relay"]  # type: ignore[index]
        ax.scatter([beacon[0]], [beacon[1]], color="black", marker="D", s=32, label="beacon")
        if final["topology"] == "relay":  # type: ignore[index]
            ax.plot([beacon[0], relay[0]], [beacon[1], relay[1]], color="black", linewidth=0.8, alpha=0.6)
            for position in final["searchers"].values():  # type: ignore[index]
                ax.plot([relay[0], position[0]], [relay[1], position[1]], color="black", linewidth=0.8, alpha=0.6)
        ax.set_title(str(sim["method"]).replace("_", " "))
        ax.set_xlabel("x cell")
        ax.grid(True, alpha=0.22)
        ax.set_xlim(0, config.width)
        ax.set_ylim(0, config.height)
    axes[0].set_ylabel("y cell")
    axes[0].legend(fontsize=6, loc="upper left")
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_link_range_sensitivity(
    rows: list[dict[str, float]],
    output_path: Path,
    config: LinkDeniedConfig,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d_locs = [row["d_loc"] * config.cell_size_m / 1000.0 for row in rows]
    fixed_cert = [row["fixed_certified_ratio"] for row in rows]
    relay_cert = [row["relay_certified_ratio"] for row in rows]
    fixed_cov = [100.0 * row["fixed_final_coverage"] for row in rows]
    relay_cov = [100.0 * row["relay_final_coverage"] for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True)
    axes[0].plot(d_locs, fixed_cert, marker="o", label="fixed denied")
    axes[0].plot(d_locs, relay_cert, marker="s", label="adaptive relay")
    axes[1].plot(d_locs, fixed_cov, marker="o", label="fixed denied")
    axes[1].plot(d_locs, relay_cov, marker="s", label="adaptive relay")
    axes[0].set_ylabel("Certified ratio")
    axes[1].set_ylabel("Coverage / %")
    axes[1].set_xlabel("Localization range / km")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_link_severity_sensitivity(
    rows: list[dict[str, float]],
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    qualities = [row["denied_quality"] for row in rows]
    fixed_cert = [row["fixed_certified_ratio"] for row in rows]
    relay_cert = [row["relay_certified_ratio"] for row in rows]
    fixed_cov = [100.0 * row["fixed_final_coverage"] for row in rows]
    relay_cov = [100.0 * row["relay_final_coverage"] for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(6.4, 5.0), sharex=True)
    axes[0].plot(qualities, fixed_cert, marker="o", label="fixed denied")
    axes[0].plot(qualities, relay_cert, marker="s", label="adaptive relay")
    axes[1].plot(qualities, fixed_cov, marker="o", label="fixed denied")
    axes[1].plot(qualities, relay_cov, marker="s", label="adaptive relay")
    axes[0].set_ylabel("Certified ratio")
    axes[1].set_ylabel("Coverage / %")
    axes[1].set_xlabel("Denied-link quality")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_link_monte_carlo(
    rows: list[dict[str, float | int | str]],
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = [str(row["method"]).replace("_", " ") for row in rows]
    certified = [float(row["mean_certified_ratio"]) for row in rows]
    certified_yerr = [float(row["mean_certified_ratio_ci_half_width"]) for row in rows]
    coverage = [100.0 * float(row["mean_final_coverage"]) for row in rows]
    coverage_yerr = [100.0 * float(row["mean_final_coverage_ci_half_width"]) for row in rows]
    success = [100.0 * float(row["success_rate"]) for row in rows]
    success_yerr = [
        [100.0 * max(0.0, float(row["success_rate"]) - float(row["success_rate_ci_low"])) for row in rows],
        [100.0 * max(0.0, float(row["success_rate_ci_high"]) - float(row["success_rate"])) for row in rows],
    ]

    fig, axes = plt.subplots(1, 3, figsize=(9.0, 3.2))
    axes[0].bar(labels, certified, yerr=certified_yerr, capsize=4.0, color="#118ab2", alpha=0.86)
    axes[1].bar(labels, coverage, yerr=coverage_yerr, capsize=4.0, color="#2a9d8f", alpha=0.86)
    axes[2].bar(labels, success, yerr=success_yerr, capsize=4.0, color="#5c677d", alpha=0.86)
    axes[0].set_ylabel("Mean certified ratio")
    axes[1].set_ylabel("Mean coverage / %")
    axes[2].set_ylabel("Full-cert. success / %")
    axes[0].set_ylim(0.0, 1.05)
    axes[2].set_ylim(0.0, 105.0)
    for axis in axes:
        axis.grid(True, axis="y", alpha=0.3)
        axis.tick_params(axis="x", labelrotation=20)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)


def _plot_topology_ablation(
    rows: list[dict[str, float | int | str]],
    output_path: Path,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = [str(row["method"]).replace("_", " ") for row in rows]
    certified = [float(row["mean_certified_ratio"]) for row in rows]
    certified_yerr = [float(row["mean_certified_ratio_ci_half_width"]) for row in rows]
    min_margin = [float(row["mean_min_margin"]) for row in rows]
    min_margin_yerr = [float(row["mean_min_margin_ci_half_width"]) for row in rows]
    recovery = [100.0 * float(row["mean_recovery_ratio"]) for row in rows]

    fig, axes = plt.subplots(1, 3, figsize=(9.0, 3.2))
    axes[0].bar(labels, certified, yerr=certified_yerr, capsize=4.0, color="#118ab2", alpha=0.86)
    axes[1].bar(labels, min_margin, yerr=min_margin_yerr, capsize=4.0, color="#2a9d8f", alpha=0.86)
    axes[2].bar(labels, recovery, color="#5c677d", alpha=0.86)
    axes[0].set_ylabel("Mean certified ratio")
    axes[1].set_ylabel("Mean min margin")
    axes[2].set_ylabel("Direct-failure recovery / %")
    axes[0].set_ylim(0.0, 1.05)
    axes[2].set_ylim(0.0, 105.0)
    for axis in axes:
        axis.grid(True, axis="y", alpha=0.3)
        axis.tick_params(axis="x", labelrotation=20)
    fig.tight_layout()
    fig.savefig(output_path, dpi=220)
    plt.close(fig)
