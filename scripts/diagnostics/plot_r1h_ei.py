"""Plot estimator-in-the-loop R1H-EI results against truth-in-loop R1H."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _frames(path: Path) -> list[dict]:
    return json.loads(path.read_text())["state"]


def _coverage_series(frames: list[dict], total: int) -> list[float]:
    covered: set[tuple[int, int]] = set()
    series = []
    for frame in frames:
        for cell in frame.get("update", []):
            if (
                isinstance(cell, list)
                and len(cell) >= 2
                and isinstance(cell[0], int)
                and isinstance(cell[1], int)
            ):
                covered.add((cell[0], cell[1]))
        series.append(len(covered) / total)
    return series


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("ei_data", type=Path)
    parser.add_argument("ei_estimates", type=Path)
    parser.add_argument("truth_data", type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/private/tmp/r1h-figs"))
    arguments = parser.parse_args(argv)

    ei_frames = _frames(arguments.ei_data)
    truth_frames = _frames(arguments.truth_data)
    total = 90_000
    t = np.arange(len(ei_frames)) * 0.5
    ei_cov = np.array(_coverage_series(ei_frames, total))
    truth_cov = np.array(_coverage_series(truth_frames, total))

    estimates = [
        json.loads(line)
        for line in arguments.ei_estimates.read_text().splitlines()
    ]
    epsilon = np.array(
        [
            max(robot["epsilon"] for robot in frame["robots"])
            for frame in estimates
        ]
    )
    tiers = [
        sorted(
            {robot["tier"] for robot in frame["robots"]},
            key=lambda name: ("fresh", "predicted", "hold").index(name),
        )
        for frame in estimates
    ]

    out = arguments.out_dir
    out.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 3, figsize=(17, 4.6))
    axes[0].plot(t, truth_cov * 100, label="R1H truth-in-loop", lw=1.6)
    axes[0].plot(t, ei_cov * 100, label="R1H-EI estimator-in-loop", lw=1.6)
    axes[0].axhline(100, color="gray", ls=":", lw=1)
    axes[0].set_xlabel("t (s)")
    axes[0].set_ylabel("coverage (%)")
    axes[0].set_title("Search coverage")
    axes[0].legend()
    axes[0].grid(alpha=0.3)

    axes[1].plot(t, epsilon, lw=1.2, color="tab:red")
    axes[1].axhline(30, color="gray", ls=":", lw=1, label="30 m alarm")
    axes[1].set_xlabel("t (s)")
    axes[1].set_ylabel("max epsilon (m)")
    axes[1].set_title("Per-frame max estimator epsilon")
    axes[1].legend()
    axes[1].grid(alpha=0.3)

    tier_colors = {"fresh": "tab:green", "predicted": "tab:blue", "hold": "tab:red"}
    frame_tier = [
        max(frame["robots"], key=lambda r: tier_colors.get(r["tier"], "")).get(
            "tier"
        )
        for frame in estimates
    ]
    for tier, color in tier_colors.items():
        mask = np.array([ft == tier for ft in frame_tier])
        axes[2].fill_between(
            t, 0, 1, where=mask, step="post", color=color, alpha=0.55,
            label=tier,
        )
    axes[2].set_xlabel("t (s)")
    axes[2].set_ylim(0, 1)
    axes[2].set_yticks([])
    axes[2].set_title("Per-frame worst estimator tier")
    axes[2].legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out / "ei-summary.png", dpi=160)
    plt.close(fig)

    # error distribution and containment against epsilon
    truth_by_frame = {
        int(frame["runtime"] / 0.5): {
            robot["id"]: (robot["state"]["x"], robot["state"]["y"])
            for robot in frame["robots"]
        }
        for frame in ei_frames
    }
    errors = []
    contained = []
    eps_used = []
    for frame in estimates:
        truth = truth_by_frame[frame["frame_index"]]
        for robot in frame["robots"]:
            est = np.asarray(robot["estimate"])
            tpos = np.asarray(truth[robot["id"]])
            err = float(np.linalg.norm(est - tpos))
            errors.append(err)
            contained.append(err <= robot["epsilon"])
            eps_used.append(robot["epsilon"])
    errors = np.asarray(errors)
    eps_used = np.asarray(eps_used)
    contained = np.asarray(contained)

    fig, axes = plt.subplots(1, 3, figsize=(17, 4.6))
    axes[0].hist(errors, bins=60, color="tab:blue", alpha=0.8)
    axes[0].set_xlabel("|estimate - truth| (m)")
    axes[0].set_ylabel("count")
    axes[0].set_title(
        f"Localization error  n={len(errors)}  p50={np.median(errors):.2f}  "
        f"p95={np.percentile(errors, 95):.2f}  max={errors.max():.2f}"
    )
    axes[0].grid(alpha=0.3)

    axes[1].scatter(errors, eps_used, s=5, alpha=0.25, color="tab:green")
    lim = max(errors.max(), eps_used.max()) * 1.05
    axes[1].plot([0, lim], [0, lim], color="gray", ls=":", lw=1)
    axes[1].set_xlabel("|estimate - truth| (m)")
    axes[1].set_ylabel("epsilon (m)")
    axes[1].set_title(f"Containment |err| <= eps: {contained.mean() * 100:.2f}%")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, epsilon, color="tab:red", lw=1.2)
    axes[2].plot(
        t,
        [
            max(np.linalg.norm(
                np.asarray(robot["estimate"])
                - np.asarray(truth_by_frame[frame["frame_index"]][robot["id"]])
            ) for robot in frame["robots"])
            for frame in estimates
        ],
        color="tab:blue", lw=1.2,
    )
    axes[2].set_xlabel("t (s)")
    axes[2].set_ylabel("m")
    axes[2].set_title("max epsilon vs max error")
    axes[2].legend(["max epsilon", "max error"], loc="upper right")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out / "ei-error-containment.png", dpi=160)
    plt.close(fig)
    print("figures written to", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
