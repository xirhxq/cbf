from utils import *

from .base import BaseComponent


def filter_all(name):
    return True


def filter_cvt(name):
    return name == 'cvtCBF'


def filter_min(name):
    return 'min' in name


CBF_FILTERS = {
    'all': filter_all,
    'cvt': filter_cvt,
    'min': filter_min
}


class CBFValuesComponent(BaseComponent):
    def __init__(self, ax, data, robot_id, title=None, mode='separate', **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.title = (title or f"CBF Values") + f", Robot #{robot_id + 1}"
        self.mode = mode
        self.cbf_filter = kwargs.get('params', {}).get('cbf_filter', 'all')

        if isinstance(self.cbf_filter, str):
            assert self.cbf_filter in CBF_FILTERS, f"Mode '{self.cbf_filter}' not supported."
            self.filter_func = CBF_FILTERS[self.cbf_filter]
        elif callable(self.cbf_filter):
            self.filter_func = self.cbf_filter
        else:
            raise ValueError("mode must be a string or a callable function.")

        self.keys = ("cbfNoSlack", "cbfSlack")

        self.runtime = [frame["runtime"] for frame in self.data]
        self.values = {}

        self.index_range = kwargs.get('index_range', (0, len(self.data)))

        for key in self.keys:
            if key in self.data[0]["robots"][robot_id] and self.data[0]["robots"][robot_id][key] is not None:
                cbf_names = set()
                for frame in self.data[slice(*self.index_range)]:
                    if key in frame["robots"][robot_id] and frame["robots"][robot_id][key] is not None:
                        for name, value in frame["robots"][robot_id][key].items():
                            if self.filter_func(name):
                                cbf_names.add(name)

                for cbf_name in sorted(cbf_names):
                    times = []
                    values = []
                    for idx, frame in enumerate(self.data[slice(*self.index_range)]):
                        val = None
                        if key in frame["robots"][robot_id]:
                            for name, value in frame["robots"][robot_id][key].items():
                                if name == cbf_name and self.filter_func(name):
                                    val = value
                                    break
                        times.append(frame["runtime"])
                        values.append(val if val is not None else np.nan)

                    self.values[f"{key}:{cbf_name}"] = {"time": times, "value": values}
                    print(f"CBF {key}:{cbf_name} for Robot {robot_id + 1} has {len(times)} data points.")

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
        self.y_limits = self.ax.get_ylim()
        self.vline = self.ax.plot(
            [self.runtime[self.index_range[0]], self.runtime[self.index_range[0]]],
            [self.y_limits[0], self.y_limits[1]],
            'r--', alpha=0.3
        )[0]

    def update(self, num, dataNow=None):
        self.vline.set_data([self.runtime[num], self.runtime[num]], self.y_limits)
