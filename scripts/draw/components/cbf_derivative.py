from utils import *

from .base import BaseComponent


class CBFDerivativeComponent(BaseComponent):
    def __init__(self, ax, data, robot_id, title=None, mode='separate', **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.title = (title or f"CBF Values") + f", Robot #{robot_id + 1}"
        self.mode = mode

        self.key = "cbfNoSlack"

        self.runtime = [frame["runtime"] for frame in self.data]
        self.values = {}


        frame =  self.data[0]

        cbf_names = set()
        if self.key in frame["robots"][robot_id] and self.data[0]["robots"][robot_id][self.key] is not None:
            for name, value in frame["robots"][robot_id][self.key].items():
                cbf_names.add(name)


        for cbf_name in sorted(cbf_names):
            times = []
            dh_list = []
            alphah_list = []
            for idx, frame in enumerate(self.data):
                if idx == len(self.data) - 1:
                    continue
                next_frame = self.data[idx + 1]
                dh, alphah = None, None
                if self.key in frame["robots"][robot_id]:
                    nowh = frame["robots"][robot_id][self.key][cbf_name]
                    nexth = next_frame["robots"][robot_id][self.key][cbf_name]
                    dh = (nexth - nowh) / (next_frame["runtime"] - frame["runtime"])
                    alphah = -nowh
                times.append(frame["runtime"])
                dh_list.append(dh if dh is not None else np.nan)
                alphah_list.append(alphah if alphah is not None else np.nan)

            self.values[f"{cbf_name}"] = {"time": times, "dh": np.array(dh_list), "alphah": np.array(alphah_list)}

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
        self.ax.set_ylabel('CBF Derivative')

        self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.3)

        for label in self.values:
            time_data = self.values[label]["time"]
            line, = self.ax.plot(time_data, self.values[label]["dh"] - self.values[label]["alphah"], label=label)
            self.lines[label] = line
            # for key in self.values[label].keys():
            #     if key == 'time':
            #         continue
            #     label_str = self.get_abbv(label) + '-' + key
            #     line, = self.ax.plot(time_data, self.values[label][key], label=label_str)
            #     self.lines[label_str] = line

        self.ax.legend(loc='best')

        if self.mode == 'animation':
            self.setup()

    def setup(self):
        self.vline = self.ax.plot([0, 0], [0, 1], 'r--', alpha=0.3)[0]
        self.y_limits = self.ax.get_ylim()

    def update(self, num, dataNow=None):
        self.vline.set_data([self.runtime[num], self.runtime[num]], self.y_limits)
