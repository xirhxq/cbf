import numpy as np

from .base import BaseComponent


class ConstraintMarginComponent(BaseComponent):
    """Per-frame minimum hard-row margin for the estimator-in-loop run.

    Reads ``cbfNoSlack`` barrier values from data.json and plots the
    minimum over all hard rows, plus the minima restricted to safety and
    fixed-communication rows separately. Positive margins throughout show
    that the sequential QP satisfies every hard constraint.
    """

    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data["state"]
        self._draw()

    def _draw(self) -> None:
        import matplotlib.pyplot as plt

        time = []
        min_all = []
        min_safe = []
        min_comm = []
        for frame in self.data:
            time.append(frame["runtime"])
            vals_all = []
            vals_safe = []
            vals_comm = []
            for robot in frame["robots"]:
                cbf = robot.get("cbfNoSlack", {})
                for name, value in cbf.items():
                    if not isinstance(value, (int, float)) or np.isnan(value):
                        continue
                    vals_all.append(value)
                    if "safety" in name.lower():
                        vals_safe.append(value)
                    elif "comm" in name.lower():
                        vals_comm.append(value)
            min_all.append(min(vals_all) if vals_all else np.nan)
            min_safe.append(min(vals_safe) if vals_safe else np.nan)
            min_comm.append(min(vals_comm) if vals_comm else np.nan)

        self.ax.plot(time, min_all, color="tab:blue", lw=1.4,
                     label="all hard rows")
        self.ax.plot(time, min_safe, color="tab:red", lw=1.1, alpha=0.8,
                     label="safety")
        self.ax.plot(time, min_comm, color="tab:orange", lw=1.1, alpha=0.8,
                     label="fixed comm")
        self.ax.axhline(0, color="black", ls="--", lw=1.0)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Min hard-row margin")
        self.ax.set_title("Hard-constraint margins")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="lower right", fontsize=8)
        print(
            "ConstraintMargin: min all %.3f / safety %.3f / comm %.3f"
            % (np.nanmin(min_all), np.nanmin(min_safe), np.nanmin(min_comm))
        )
