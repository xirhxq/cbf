"""Paper-quality estimator error figure for the single-run narrative.

Generates a single 5x3 figure (matching the draw-framework ``sp``
figsize): scatter of localization error versus uncertainty radius with
marginal histograms. Samples lying below the diagonal visualize the
containment claim while the marginals show the error distribution.
The coverage-progress figure is produced by the draw framework ``sp``
component to match the paper's figure style.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path,
        default=Path("/private/tmp/r1h-ekf-full/G1/2026081303"),
        help="EI run root with estimates-log.jsonl and a */data.json",
    )
    parser.add_argument(
        "--out-dir", type=Path,
        default=Path("/private/tmp/r1h-figs"),
    )
    arguments = parser.parse_args(argv)
    root = arguments.root
    data = json.loads((sorted(root.glob("*/data.json"))[-1]).read_text())
    frames = data["state"]
    truth_by_frame = {
        int(frame["runtime"] / 0.5): {
            robot["id"]: (robot["state"]["x"], robot["state"]["y"])
            for robot in frame["robots"]
        }
        for frame in frames
    }
    errors = []
    epsilons = []
    for line in (root / "estimates-log.jsonl").read_text().splitlines():
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
    errors = np.asarray(errors)
    epsilons = np.asarray(epsilons)
    contained = errors <= epsilons
    containment = float(contained.mean())
    median = float(np.median(errors))
    p95 = float(np.percentile(errors, 95))

    out = arguments.out_dir
    out.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(5, 3))
    grid = fig.add_gridspec(
        2, 2, width_ratios=(4, 1), height_ratios=(1, 4),
        wspace=0.06, hspace=0.06,
    )
    ax = fig.add_subplot(grid[1, 0])
    ax_top = fig.add_subplot(grid[0, 0], sharex=ax)
    ax_right = fig.add_subplot(grid[1, 1], sharey=ax)

    ax.scatter(errors, epsilons, s=4, alpha=0.25, color="tab:green")
    limit = max(errors.max(), epsilons.max()) * 1.05
    ax.plot([0, limit], [0, limit], color="gray", ls=":", lw=1.2)
    ax.set_xlabel("Localization error (m)")
    ax.set_ylabel(r"Uncertainty radius $\varepsilon$ (m)")
    ax.set_title(
        f"Containment {containment * 100:.1f}% "
        f"(median {median:.1f} m, p95 {p95:.1f} m)"
    )
    ax.grid(alpha=0.3)

    ax_top.hist(errors, bins=50, color="tab:blue", alpha=0.85)
    ax_top.tick_params(labelbottom=False)
    ax_top.set_ylabel("Count")

    ax_right.hist(
        epsilons, bins=50, orientation="horizontal",
        color="tab:orange", alpha=0.85,
    )
    ax_right.tick_params(labelleft=False)
    ax_right.set_xlabel("Count")

    fig.savefig(out / "estimator-error.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    print(
        "containment %.4f median %.3f p95 %.3f max %.1f"
        % (containment, median, p95, errors.max())
    )
    return 0


if __name__ == "__main__":
    main()
