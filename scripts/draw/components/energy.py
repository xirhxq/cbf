from utils import *

from .base import BaseComponent


class EnergyComponent(BaseComponent):
    def __init__(self, ax, data, robot_id=None, mode='global', **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.mode = mode

        self.runtime = [frame["runtime"] for frame in self.data]
        self.robot_num = data["config"]["num"]

        self.battery_data = [
            [frame["robots"][i]["state"]["battery"] for frame in self.data]
            for i in range(self.robot_num)
        ]

        self._initialize_plot()

    def _plot_single(self, ax, robot_id):
        ax.plot(self.runtime, self.battery_data[robot_id], label=f'UAV #{robot_id + 1}')
        ax.set_title(f'Energy Level, Robot #{robot_id + 1}')
        ax.set_xlabel('Time / s')
        ax.set_ylabel('Battery Level')

    def _plot_all_in_one(self, ax):
        for i in range(self.robot_num):
            ax.plot(self.runtime, self.battery_data[i], label=f'UAV #{i + 1}')
        ax.set_title('Energy Levels of All Robots')
        ax.set_xlabel('Time / s')
        ax.set_ylabel('Battery Level')
        ax.legend(bbox_to_anchor=(1.0, -0.15), ncol=5)

    def _plot_separate_subplots(self, fig, axes):
        for i, ax in enumerate(axes):
            if i < self.robot_num:
                self._plot_single(ax, i)
        fig.tight_layout()

    def _initialize_plot(self):
        if self.mode == 'global':
            self._plot_all_in_one(self.ax)
        elif self.mode == 'group':
            self._plot_separate_subplots(self.ax.figure, self.ax.figure.axes)
        elif self.mode == 'separate':
            self._plot_single(self.ax, self.robot_id)

    def update(self, num):
        pass
