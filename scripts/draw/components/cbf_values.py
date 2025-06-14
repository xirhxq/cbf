from utils import *

from .base import BaseComponent


class CBFValuesComponent(BaseComponent):
    def __init__(self, ax, data, robot_id, title=None, mode='separate', **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.title = title or f"CBF Values, Robot #{robot_id + 1}"
        self.mode = mode

        self.keys = ("cbfNoSlack", "cbfSlack")

        self.runtime = [frame["runtime"] for frame in self.data]
        self.values = {}

        for key in self.keys:
            if key in self.data[0]["robots"][robot_id]:
                cbf_names = set()
                for frame in self.data:
                    if key in frame["robots"][robot_id]:
                        for name, value in frame["robots"][robot_id][key].items():
                            cbf_names.add(name)
                for cbf_name in sorted(cbf_names):
                    times = []
                    values = []
                    for idx, frame in enumerate(self.data):
                        val = None
                        if key in frame["robots"][robot_id]:
                            for name, value in frame["robots"][robot_id][key].items():
                                if name == cbf_name:
                                    val = value
                                    break
                        times.append(frame["runtime"])
                        values.append(val if val is not None else np.nan)

                    self.values[f"{key}:{cbf_name}"] = {"time": times, "value": values}

        self.lines = {}
        self._initialize_plot()

    def get_abbv(self, cbf_name):
        if 'min(' in cbf_name:
            return 'min'
        else:
            return cbf_name.split('CBF')[0]

    def _initialize_plot(self):
        self.ax.set_title(self.title)
        self.ax.set_xlabel('Time / s')
        self.ax.set_ylabel('CBF Value')

        self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.3)

        for label in self.values:
            time_data = self.values[label]["time"]
            value_data = self.values[label]["value"]
            label = label.split(":")[1]
            label = self.get_abbv(label) if self.mode != 'separate' else label
            line, = self.ax.plot(time_data, value_data, label=label)
            self.lines[label] = line

        self.ax.legend(loc='best')

        if self.mode == 'animation':
            self.setup()

    def setup(self):
        self.vline = self.ax.plot([0, 0], [0, 1], 'r--', alpha=0.3)[0]
        self.y_limits = self.ax.get_ylim()

    def update(self, num, dataNow=None):
        self.vline.set_data([self.runtime[num], self.runtime[num]], self.y_limits)
