import json
from pathlib import Path

import numpy as np

from .base import BaseComponent


class EstimatorErrorComponent(BaseComponent):
    """Scatter of localization error versus epsilon with marginals.

    Reads the per-frame estimator log (estimates-log.jsonl) and the truth
    state from data.json. Single 5x3 figure matching the draw-framework
    ``sp`` figsize; containment / median / p95 are reported by the caller
    (or in the caption), not drawn inside the figure.
    """

    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data
        params = kwargs.get("params", {})
        estimates_log = params.get("estimates_log")
        if not estimates_log:
            raise ValueError("EstimatorErrorComponent requires params['estimates_log']")
        self._draw(Path(estimates_log))

    def _draw(self, estimates_log: Path) -> None:
        import matplotlib.pyplot as plt

        truth_by_frame = {
            int(frame["runtime"] / 0.5): {
                robot["id"]: (robot["state"]["x"], robot["state"]["y"])
                for robot in frame["robots"]
            }
            for frame in self.data["state"]
        }
        errors = []
        epsilons = []
        for line in estimates_log.read_text().splitlines():
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
        containment = float((errors <= epsilons).mean())

        ax = self.ax
        ax_top = ax.inset_axes([0, 1.04, 1, 0.3], sharex=ax)
        ax_right = ax.inset_axes([1.04, 0, 0.3, 1], sharey=ax)

        ax.scatter(errors, epsilons, s=4, alpha=0.25, color="tab:green")
        limit = max(errors.max(), epsilons.max()) * 1.05
        ax.plot([0, limit], [0, limit], color="gray", ls=":", lw=1.2)
        ax.set_xlabel(r"$|p-\hat p|$ (m)")
        ax.set_ylabel(
            r"$\varepsilon_i = 3\sqrt{\lambda_{\max}(\mathbf{\Sigma}_i)}$ (m)"
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

        print(
            "EstimatorError: containment %.4f median %.3f p95 %.3f max %.1f"
            % (
                containment,
                float(np.median(errors)),
                float(np.percentile(errors, 95)),
                float(errors.max()),
            )
        )
