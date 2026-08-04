"""Paper-quality estimator error panels for the single-run narrative.

Generates the two localization-error panels (histogram and
epsilon-containment scatter) consumed by the estimator-in-the-loop
section of the paper. The coverage-progress figure is produced by the
draw framework ``sp`` component to match the paper's figure style.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# Use matplotlib defaults so rendered text matches the draw-framework
# paper figures (no custom font inflation).


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path,
        default=Path("/private/tmp/r1h-ekf-avail/2026081303"),
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

    fig, ax = plt.subplots(figsize=(7, 3))
    ax.hist(errors, bins=60, color="tab:blue", alpha=0.85)
    ax.set_xlabel("Localization error (m)")
    ax.set_ylabel("Count")
    ax.set_title("Error Distribution")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "estimator-error-histogram.png", dpi=300)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 3))
    ax.scatter(errors, epsilons, s=6, alpha=0.25, color="tab:green")
    limit = max(errors.max(), epsilons.max()) * 1.05
    ax.plot([0, limit], [0, limit], color="gray", ls=":", lw=1.2)
    ax.set_xlabel("Localization error (m)")
    ax.set_ylabel(r"Uncertainty radius $\varepsilon$ (m)")
    ax.set_title(f"Containment {containment * 100:.1f}%")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "estimator-error-scatter.png", dpi=300)
    plt.close(fig)

    print(
        "containment %.4f median %.3f p95 %.3f max %.1f"
        % (containment, median, p95, errors.max())
    )
    return 0


if __name__ == "__main__":
    main()
