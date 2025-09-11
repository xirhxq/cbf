from utils import *

from .base import BaseComponent


class ControlInputComponent(BaseComponent):
    def __init__(self, ax, data, robot_id=None, title=None, mode='global', **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.title = (title or f"Control Input")
        self.mode = mode

        self.runtime = [frame["runtime"] for frame in self.data]
        self.robot_num = data["config"]["num"]

        first_frame_result = self.data[0]["robots"][0]["opt"]["result"]
        control_fields = list(first_frame_result.keys())

        self.data_list = []
        for i in range(self.robot_num):
            robot_data = {}
            for field in control_fields:
                robot_data[field] = [frame["robots"][i]["opt"]["result"][field] for frame in self.data]

            if "vx" in control_fields and "vy" in control_fields:
                robot_data["speed"] = [np.linalg.norm([
                    frame["robots"][i]["opt"]["result"]["vx"],
                    frame["robots"][i]["opt"]["result"]["vy"]
                ]) for frame in self.data]
            
            self.data_list.append(robot_data)

        self.legend_list = {}
        for field in control_fields:
            if field == "vx":
                self.legend_list[field] = "Vel X"
            elif field == "vy":
                self.legend_list[field] = "Vel Y"
            elif field == "ax":
                self.legend_list[field] = "Accel X"
            elif field == "ay":
                self.legend_list[field] = "Accel Y"
            elif field == "yawRateRad":
                self.legend_list[field] = "Yaw Rate"
            else:
                self.legend_list[field] = field.capitalize()

        if "vx" in control_fields and "vy" in control_fields:
            self.legend_list["speed"] = "Speed"

        self.xLabel = "Time / s"
        if "vx" in control_fields:
            self.yLabel = "Value / m/s"
        elif "ax" in control_fields:
            self.yLabel = "Value / m/s²"
        else:
            self.yLabel = "Value"

        self._initialize_plot()

    def _plot_single(self, ax):
        for key, label in self.legend_list.items():
            ax.plot(self.runtime, self.data_list[self.robot_id][key], label=f'{label}, UAV #{self.robot_id + 1}')
        ax.set_title(self.title + f', Robot #{self.robot_id + 1}')
        ax.set_xlabel(self.xLabel)
        ax.set_ylabel(self.yLabel)
        ax.legend(loc='best')

    def _plot_all_in_one(self, ax):
        for i in range(self.robot_num):
            for key, label in self.legend_list.items():
                ax.plot(self.runtime, self.data_list[i][key], label=f'{label}, UAV #{i + 1}')
        ax.set_title(self.title + ', All Robots')
        ax.set_xlabel(self.xLabel)
        ax.set_ylabel(self.yLabel)
        ax.legend(loc='best')

    def _initialize_plot(self):
        if self.mode == "global":
            self._plot_all_in_one(self.ax)
        elif self.mode == "group":
            self._plot_single(self.ax)
        elif self.mode == "separate":
            self._plot_single(self.ax)
        elif self.mode == "animation":
            self._plot_single(self.ax)
            self._setup_timeline(self.ax)

    def _setup_timeline(self, ax):
        self.vline = self.ax.plot([0, 0], [0, 1], 'r--', alpha=0.3)[0]
        self.y_limits = self.ax.get_ylim()

    def update(self, num):
        self.vline.set_data([self.runtime[num], self.runtime[num]], self.y_limits)

