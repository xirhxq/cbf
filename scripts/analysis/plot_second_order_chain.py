#!/usr/bin/env python3
"""Plot metrics for the second-order chain-localization experiment."""

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt


def robot_map(frame):
    return {int(robot["id"]): robot for robot in frame.get("robots", [])}


def xy(robot):
    state = robot["state"]
    return float(state["x"]), float(state["y"])


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_json", type=Path)
    parser.add_argument("output_png", type=Path)
    args = parser.parse_args()

    with args.data_json.open() as handle:
        data = json.load(handle)

    times = []
    d12 = []
    d23 = []
    dbase = []
    ax1 = []
    ax2 = []
    ax3 = []
    min_hocbf = []
    min_psi1 = []

    base = tuple(float(v) for v in data["config"]["bases"][0])

    for frame in data["state"]:
        robots = robot_map(frame)
        times.append(float(frame["runtime"]))
        d12.append(dist(xy(robots[1]), xy(robots[2])))
        d23.append(dist(xy(robots[2]), xy(robots[3])))
        dbase.append(dist(xy(robots[1]), base))

        controls = []
        hocbf_values = []
        psi_values = []
        for robot_id in (1, 2, 3):
            opt = robots[robot_id].get("opt", {})
            controls.append(float(opt.get("result", {}).get("ax", 0.0)))
            for item in opt.get("hocbfNoSlack", []):
                if "FixedComm" in item.get("name", ""):
                    hocbf_values.append(float(item.get("hocbf", math.nan)))
                    psi_values.append(float(item.get("psi1", math.nan)))

        ax1.append(controls[0])
        ax2.append(controls[1])
        ax3.append(controls[2])
        min_hocbf.append(min(hocbf_values) if hocbf_values else math.nan)
        min_psi1.append(min(psi_values) if psi_values else math.nan)

    plt.rcParams.update({
        "font.size": 9,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "lines.linewidth": 1.8,
    })
    fig, axes = plt.subplots(3, 1, figsize=(5.2, 6.4), sharex=True)

    axes[0].plot(times, dbase, marker="o", label="base-1")
    axes[0].plot(times, d12, marker="s", label="1-2")
    axes[0].plot(times, d23, marker="^", label="2-3")
    axes[0].axhline(10.0, color="tab:red", linestyle="--", label="$d_{loc}$")
    axes[0].set_ylabel("Distance (m)")
    axes[0].legend(loc="best", ncols=2, frameon=True)

    axes[1].plot(times, ax1, marker="o", label="robot 1")
    axes[1].plot(times, ax2, marker="s", label="robot 2")
    axes[1].plot(times, ax3, marker="^", label="robot 3")
    axes[1].set_ylabel("$a_x$ (m/s$^2$)")
    axes[1].legend(loc="best", ncols=3, frameon=True)

    axes[2].plot(times, min_psi1, marker="s", label="min $\\psi_1$")
    axes[2].plot(times, min_hocbf, marker="o", label="min residual")
    axes[2].axhline(0.0, color="black", linewidth=0.8)
    axes[2].set_ylabel("HOCBF value")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="best", frameon=True)

    fig.tight_layout()
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output_png, dpi=220)

    metrics = {
        "frames": len(times),
        "runtime_last": times[-1],
        "max_chain_distance": max(max(dbase), max(d12), max(d23)),
        "min_chain_margin": 10.0 - max(max(dbase), max(d12), max(d23)),
        "min_pair_distance": min(min(d12), min(d23)),
        "max_abs_ax": max(abs(value) for values in (ax1, ax2, ax3) for value in values),
        "min_hocbf": min(value for value in min_hocbf if math.isfinite(value)),
        "min_psi1": min(value for value in min_psi1 if math.isfinite(value)),
    }
    for key, value in metrics.items():
        print(f"{key}: {value:.12g}")


if __name__ == "__main__":
    main()
