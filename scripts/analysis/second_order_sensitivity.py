#!/usr/bin/env python3
"""Run and plot second-order CBF sensitivity experiments."""

import argparse
import csv
import json
import math
import re
import subprocess
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt


VARIANTS = [
    {"name": "dt=0.25", "kind": "sample time", "dt": 0.25, "amax": 5.0},
    {"name": "dt=0.50", "kind": "sample time", "dt": 0.50, "amax": 5.0},
    {"name": "dt=1.00", "kind": "sample time", "dt": 1.00, "amax": 5.0},
    {"name": "amax=0.25", "kind": "acceleration bound", "dt": 0.50, "amax": 0.25},
    {"name": "amax=0.40", "kind": "acceleration bound", "dt": 0.50, "amax": 0.40},
    {"name": "amax=0.55", "kind": "acceleration bound", "dt": 0.50, "amax": 0.55},
    {"name": "amax=1.00", "kind": "acceleration bound", "dt": 0.50, "amax": 1.00},
]


def robot_map(frame):
    return {int(robot["id"]): robot for robot in frame.get("robots", [])}


def xy(robot):
    state = robot["state"]
    return float(state["x"]), float(state["y"])


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def summarize_data(data_path, variant, stdout):
    with data_path.open() as handle:
        data = json.load(handle)

    config = data["config"]
    frames = data["state"]
    base = tuple(float(v) for v in config["bases"][0])
    dloc = float(config["cbfs"]["without-slack"]["comm-fixed"]["max-range"])
    time_total = float(config["execute"]["time-total"])
    dt = float(config["execute"]["time-step"])

    max_chain_edge = -math.inf
    min_pair_distance = math.inf
    max_abs_ax = 0.0
    min_hocbf = math.inf
    min_psi1 = math.inf
    opt_failures = 0

    for frame in frames:
        robots = robot_map(frame)
        dbase = dist(xy(robots[1]), base)
        d12 = dist(xy(robots[1]), xy(robots[2]))
        d23 = dist(xy(robots[2]), xy(robots[3]))
        max_chain_edge = max(max_chain_edge, dbase, d12, d23)
        min_pair_distance = min(min_pair_distance, d12, d23)

        for robot in robots.values():
            opt = robot.get("opt", {})
            status = opt.get("status")
            solver_status = opt.get("solver_info", {}).get("status")
            if status is not None and status != "success":
                opt_failures += 1
            if solver_status is not None and solver_status not in ("optimal", "optimal_inaccurate"):
                opt_failures += 1

            control = opt.get("result", {})
            max_abs_ax = max(max_abs_ax, abs(float(control.get("ax", 0.0))))
            for item in opt.get("hocbfNoSlack", []):
                if "FixedComm" not in item.get("name", ""):
                    continue
                min_hocbf = min(min_hocbf, float(item.get("hocbf", math.inf)))
                min_psi1 = min(min_psi1, float(item.get("psi1", math.inf)))

    warning_count = stdout.count("OSQP solver did not find optimal solution")
    runtime_last = float(frames[-1]["runtime"])
    full_horizon = runtime_last >= time_total - dt - 1.0e-9
    min_margin = dloc - max_chain_edge

    return {
        "variant": variant["name"],
        "kind": variant["kind"],
        "dt": variant["dt"],
        "amax": variant["amax"],
        "frames": len(frames),
        "runtime_last": runtime_last,
        "full_horizon": int(full_horizon),
        "max_chain_edge": max_chain_edge,
        "min_chain_margin": min_margin,
        "min_pair_distance": min_pair_distance,
        "max_abs_ax": max_abs_ax,
        "min_hocbf": min_hocbf if math.isfinite(min_hocbf) else math.nan,
        "min_psi1": min_psi1 if math.isfinite(min_psi1) else math.nan,
        "solver_warnings": warning_count,
        "opt_failures": opt_failures,
        "data_path": str(data_path),
    }


def write_summary(rows, csv_path):
    fieldnames = [
        "variant",
        "kind",
        "dt",
        "amax",
        "frames",
        "runtime_last",
        "full_horizon",
        "max_chain_edge",
        "min_chain_margin",
        "min_pair_distance",
        "max_abs_ax",
        "min_hocbf",
        "min_psi1",
        "solver_warnings",
        "opt_failures",
        "data_path",
    ]
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(rows, output_png):
    sample_rows = [row for row in rows if row["kind"] == "sample time"]
    accel_rows = [row for row in rows if row["kind"] == "acceleration bound"]

    plt.rcParams.update({
        "font.size": 9,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "lines.linewidth": 1.8,
    })
    fig, axes = plt.subplots(2, 1, figsize=(5.2, 6.0))

    x_dt = [row["dt"] for row in sample_rows]
    axes[0].plot(x_dt, [row["min_chain_margin"] for row in sample_rows], marker="o", label="range margin")
    axes[0].plot(x_dt, [row["min_psi1"] for row in sample_rows], marker="s", label="min $\\psi_1$")
    axes[0].axhline(0.0, color="black", linewidth=0.8)
    axes[0].set_xlabel("Sample time (s)")
    axes[0].set_ylabel("Margin / HOCBF value")
    axes[0].legend(frameon=True)

    x_acc = [row["amax"] for row in accel_rows]
    axes[1].plot(x_acc, [row["runtime_last"] for row in accel_rows], marker="o", label="logged horizon")
    axes[1].plot(x_acc, [4.0 * row["full_horizon"] for row in accel_rows], marker="s", label="full-horizon flag")
    for row in accel_rows:
        if row["solver_warnings"] > 0:
            axes[1].annotate(
                "warn",
                (row["amax"], row["runtime_last"]),
                textcoords="offset points",
                xytext=(0, 7),
                ha="center",
            )
    axes[1].set_xlabel("$a_{max}$ (m/s$^2$)")
    axes[1].set_ylabel("Time (s)")
    axes[1].set_ylim(-0.1, 4.3)
    axes[1].legend(frameon=True)

    fig.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=220)


def run_variants(args):
    repo = args.repo.resolve()
    template = json.loads(args.template.read_text())
    rows = []

    with tempfile.TemporaryDirectory(prefix="cbf-second-order-sensitivity-") as tmp:
        tmp_dir = Path(tmp)
        for index, variant in enumerate(VARIANTS):
            config = json.loads(json.dumps(template))
            config["execute"]["time-step"] = variant["dt"]
            config["cbfs"]["high-order"]["acceleration-bound"] = variant["amax"]
            suffix = re.sub(r"[^a-zA-Z0-9]+", "_", variant["name"]).strip("_").lower()
            config["run_suffix"] = "_second_order_sensitivity_" + suffix

            config_path = tmp_dir / f"variant_{index}.json"
            config_path.write_text(json.dumps(config, indent=2))

            completed = subprocess.run(
                [str(repo / "build-codex" / "Swarm"), str(config_path)],
                cwd=repo,
                check=False,
                capture_output=True,
                text=True,
            )
            stdout = completed.stdout + completed.stderr
            match = re.search(r"\[OUTPUT_DIR\]\s+(.+)", stdout)
            if not match:
                raise RuntimeError(f"Swarm output did not contain OUTPUT_DIR for {variant['name']}\n{stdout}")

            data_path = Path(match.group(1).strip()) / "data.json"
            rows.append(summarize_data(data_path, variant, stdout))

    write_summary(rows, args.csv)
    plot_summary(rows, args.figure)
    for row in rows:
        print(
            f"{row['variant']}: full={row['full_horizon']} "
            f"runtime={row['runtime_last']:.3g} "
            f"margin={row['min_chain_margin']:.6g} "
            f"psi1={row['min_psi1']:.6g} "
            f"warnings={row['solver_warnings']}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path("."))
    parser.add_argument("--template", type=Path, default=Path("config/config_second_order_chain.json"))
    parser.add_argument("--csv", type=Path, default=Path("papers/SecondOrderCBF2026/assets/sensitivity_summary.csv"))
    parser.add_argument("--figure", type=Path, default=Path("papers/SecondOrderCBF2026/assets/sensitivity_summary.png"))
    args = parser.parse_args()
    run_variants(args)


if __name__ == "__main__":
    main()
