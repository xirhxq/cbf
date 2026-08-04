import json
from pathlib import Path

import numpy as np

from .base import BaseComponent


def _load_epsilons(estimates_log: Path) -> dict[int, dict]:
    """Return per-robot epsilon time series from the estimator log."""
    data: dict[int, dict] = {}
    for line in estimates_log.read_text().splitlines():
        frame = json.loads(line)
        t = frame["frame_index"] * 0.5
        for robot in frame["robots"]:
            rid = robot["id"]
            entry = data.setdefault(rid, {"t": [], "eps": []})
            entry["t"].append(t)
            entry["eps"].append(robot["epsilon"])
    return data


class EKFEpsilonRepresentative(BaseComponent):
    """EKF uncertainty radius time series for representative UAVs."""

    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        params = kwargs.get("params", {})
        estimates_log = params.get("estimates_log")
        if not estimates_log:
            raise ValueError("EKFEpsilonRepresentative requires params['estimates_log']")
        ids = params.get("id_list", [0, 2, 4, 6])
        self._draw(Path(estimates_log), ids)

    def _draw(self, estimates_log: Path, ids: list[int]) -> None:
        data = _load_epsilons(estimates_log)
        for robot_idx in ids:
            rid = robot_idx + 1
            entry = data.get(rid)
            if entry is None:
                continue
            self.ax.plot(entry["t"], entry["eps"], lw=2.0, label=f"UAV {rid}")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel(
            r"EKF $\varepsilon_i = 3\sqrt{\lambda_{\max}(\mathbf{\Sigma}_i)}$ (m)"
        )
        self.ax.set_title("EKF uncertainty radius (UAV 1, 3, 5, 7)")
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper left", fontsize=8)


class EKFEpsilonBoxplot(BaseComponent):
    """EKF uncertainty radius per robot at a given time point."""

    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        params = kwargs.get("params", {})
        estimates_log = params.get("estimates_log")
        if not estimates_log:
            raise ValueError("EKFEpsilonBoxplot requires params['estimates_log']")
        time_point = params.get("time_point", 150.0)
        uav_range = params.get("uav_range", list(range(7)))
        self._draw(Path(estimates_log), time_point, uav_range)

    def _draw(
        self, estimates_log: Path, time_point: float, uav_range: list[int]
    ) -> None:
        data = _load_epsilons(estimates_log)
        values = {}
        for rid, entry in data.items():
            idx = int(np.argmin(np.abs(np.asarray(entry["t"]) - time_point)))
            values[rid] = entry["eps"][idx]
        colors = [
            "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
            "#9467bd", "#8c564b", "#e377c2",
        ]
        positions = []
        labels = []
        for i, robot_idx in enumerate(uav_range):
            rid = robot_idx + 1
            if rid not in values:
                continue
            pos = len(positions) + 1
            positions.append(pos)
            labels.append(f"UAV {rid}")
            color = colors[i % len(colors)]
            self.ax.plot(pos, values[rid], "o", color=color, markersize=10,
                         markeredgecolor="white", markeredgewidth=1.5,
                         label=f"UAV {rid}")
            self.ax.text(pos, values[rid] + 0.15, f"{values[rid]:.1f}",
                         ha="center", va="bottom", color=color, fontsize=8)
        self.ax.set_ylabel(
            r"EKF $\varepsilon_i = 3\sqrt{\lambda_{\max}(\mathbf{\Sigma}_i)}$ (m)"
        )
        self.ax.set_xlabel("UAV (Squad 1)")
        self.ax.set_xticks(positions)
        self.ax.set_xticklabels(labels, fontsize=8)
        self.ax.set_title(f"EKF uncertainty at t={time_point:.0f}s")
        self.ax.grid(True, alpha=0.3, axis="y", linestyle="--")
        self.ax.set_xlim(0.5, len(positions) + 0.5)
