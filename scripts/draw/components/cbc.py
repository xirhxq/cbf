from utils import *
from .base import BaseComponent


class CBCComponent(BaseComponent):
    def __init__(self, ax, data, robot_id, title=None, mode='separate', **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.title = (title or f"CBF Values") + f", Robot #{robot_id + 1}"
        self.mode = mode

        self.key = "cbfNoSlack"

        self.runtime = [frame["runtime"] for frame in self.data]
        self.values = {}

        frame = self.data[0]

        cbf_names = set()
        if self.key in frame["robots"][robot_id] and self.data[0]["robots"][robot_id][self.key] is not None:
            for name, value in frame["robots"][robot_id][self.key].items():
                cbf_names.add(name)

        if kwargs.get('params', {}).get('cbf_filter', None) is not None:
            cbf_names = set(filter(lambda name: kwargs['params']['cbf_filter'] in name, cbf_names))

        self.index_range = kwargs.get('index_range', (0, len(self.data)))

        for cbf_name in sorted(cbf_names):
            times = []
            dh_list = []
            alphah_list = []
            for idx, frame in enumerate(self.data[slice(*self.index_range)]):
                if self.index_range[0] + idx == len(self.data) - 1:
                    continue
                next_frame = self.data[self.index_range[0] + idx + 1]
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
        self.markers = {}
        self.value_texts = {}

        marker_style = dict(marker='*', color='red', alpha=0.7, markersize=10)
        text_style = dict(color='red', alpha=0.8, fontsize=9, bbox=dict(facecolor='white', alpha=0.3, edgecolor='none'))

        for label in self.values:
            marker_line, = self.ax.plot([np.nan], [np.nan], **marker_style)
            self.markers[label] = marker_line

            text = self.ax.text(np.nan, np.nan, "", **text_style,
                                verticalalignment='center',
                                horizontalalignment='left')
            self.value_texts[label] = text
        self._initialize_plot()

    def get_abbv(self, cbf_name):
        if 'min(' in cbf_name:
            return 'min'
        else:
            return cbf_name.split('CBF')[0]

    def _initialize_plot(self):
        self.ax.set_title(self.title)
        self.ax.set_xlabel('Time / s')
        self.ax.set_ylabel('CBC LHS-RHS')

        self.ax.axhline(y=0, color='black', linestyle='--', alpha=0.3)

        for label in self.values:
            time_data = self.values[label]["time"]
            line, = self.ax.plot(time_data, self.values[label]["dh"] - self.values[label]["alphah"], label=label, marker='o', markersize=5, alpha=0.4)
            self.lines[label] = line

        self.ax.legend(loc='best')

        if self.mode == 'animation':
            self.setup()

    def setup(self):
        self.y_limits = self.ax.get_ylim()
        self.vline = self.ax.plot(
            [self.runtime[self.index_range[0]], self.runtime[self.index_range[0]]],
            [self.y_limits[0], self.y_limits[1]],
            'r--', alpha=0.3
        )[0]

    def update(self, num, dataNow=None):
        self.vline.set_data([self.runtime[num], self.runtime[num]], self.y_limits)

        time_span = self.runtime[self.index_range[1] - 1] - self.runtime[self.index_range[0]]
        time_offset = time_span * 0.015

        x_limits = self.ax.get_xlim()

        for label, line in self.lines.items():
            idx = num - self.index_range[0]
            cbc = self.values[label]['dh'][idx] - self.values[label]['alphah'][idx]

            current_time = self.runtime[num]

            self.markers[label].set_data(current_time, cbc)

            if current_time < (x_limits[0] + x_limits[1]) / 2:
                self.value_texts[label].set_horizontalalignment('left')
                self.value_texts[label].set_position((current_time + time_offset, cbc))
            else:
                self.value_texts[label].set_horizontalalignment('right')
                self.value_texts[label].set_position((current_time - time_offset, cbc))

            # 设置文本内容
            self.value_texts[label].set_text(f"{self.get_abbv(label)}: {cbc:.4f}")

        self.ax.legend(loc='best')