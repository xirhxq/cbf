from utils import *
from .base import BaseComponent


class Lines(BaseComponent):
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data

        self.title = kwargs.get('title', 'Line Plot')
        self.xlabel = kwargs.get('xlabel', 'Time / s')
        self.ylabel = kwargs.get('ylabel', 'Values')
        self.mode = kwargs.get('mode', 'separate')

        id_list = kwargs.get('id_list', [])
        if id_list:
            self.robot_ids = id_list
        elif 'robot_ids' in kwargs:
            self.robot_ids = kwargs['robot_ids']
        elif 'config' in data and 'num' in data['config']:
            self.robot_ids = list(range(data['config']['num']))
        else:
            self.robot_ids = [0]

        self.id_list = self.robot_ids
        self.time_range = kwargs.get('time_range', None)
        self.index_range = kwargs.get('index_range', None)

        self.line_style = kwargs.get('line_style', {})
        self.marker_style = kwargs.get('marker_style', {
            'marker': '*', 'color': 'red', 'alpha': 0.7, 'markersize': 10
        })
        self.text_style = kwargs.get('text_style', {
            'color': 'red', 'alpha': 0.8, 'fontsize': 9,
            'bbox': {'facecolor': 'white', 'alpha': 0.7, 'edgecolor': 'none'}
        })

        self.show_zero_line = kwargs.get('show_zero_line', True)
        self.show_legend = kwargs.get('show_legend', True)
        self.show_markers = kwargs.get('show_markers', True)
        self.show_value_text = kwargs.get('show_value_text', True)

        self.show_bounds = kwargs.get('show_bounds', False)
        self.bounds_lower = kwargs.get('bounds_lower', None)
        self.bounds_upper = kwargs.get('bounds_upper', None)
        self.bounds_color = kwargs.get('bounds_color', 'red')
        self.bounds_style = kwargs.get('bounds_style', '--')
        self.bounds_alpha = kwargs.get('bounds_alpha', 0.5)

        self.data_processor = kwargs.get('data_processor', None)
        self.data_interpreter = kwargs.get('data_interpreter', None)

        self.lines = {}
        self.markers = {}
        self.value_texts = {}
        self.runtime = []
        self.processed_data = {}

        self.show_robot_ids = kwargs.get('show_robot_ids', len(self.robot_ids) > 1)
        self.robot_prefix = kwargs.get('robot_prefix', 'Robot #')

        self._process_data()
        self._initialize_plot()
        self._apply_bounds_limits()

    def _process_data(self):
        if self.data_interpreter:
            self.processed_data = self.data_interpreter(self.data)
        elif self.data_processor:
            self.processed_data = self.data_processor(self.data)
        else:
            self.processed_data = self._default_data_processor()

        self._extract_runtime()

        if self.index_range and not self.time_range:
            start_idx, end_idx = self.index_range
            if start_idx < len(self.runtime) and end_idx <= len(self.runtime):
                start_time = self.runtime[start_idx]
                end_time = self.runtime[end_idx - 1] if end_idx > 0 else self.runtime[-1]
                self.time_range = (start_time, end_time)

        if self.time_range:
            self._apply_time_filter()

    def _default_data_processor(self):
        raise NotImplementedError("Subclasses must implement data processing")

    def _extract_runtime(self):
        if isinstance(self.processed_data, dict) and 'runtime' in self.processed_data:
            self.runtime = self.processed_data['runtime']
        elif isinstance(self.processed_data, list) and len(self.processed_data) > 0:
            if isinstance(self.processed_data[0], dict) and 'runtime' in self.processed_data[0]:
                self.runtime = [frame['runtime'] for frame in self.processed_data]
            else:
                self.runtime = []
                for item in self.processed_data:
                    if 'timestamp' in item:
                        self.runtime = item['timestamp']
                        break
                    elif 'time' in item:
                        self.runtime = item['time']
                        break
        elif isinstance(self.processed_data, dict):
            if len(self.processed_data) > 0:
                first_key = list(self.processed_data.keys())[0]
                first_value = self.processed_data[first_key]
                if isinstance(first_value, dict) and 'runtime' in first_value:
                    self.runtime = first_value['runtime']
                elif isinstance(first_value, dict) and 'timestamp' in first_value:
                    self.runtime = first_value['timestamp']

        if not self.runtime:
            raise ValueError("Cannot extract runtime from processed data")

    def _apply_time_filter(self):
        if not self.time_range or len(self.time_range) != 2:
            return

        start_time, end_time = self.time_range
        start_idx = np.searchsorted(self.runtime, start_time)
        end_idx = np.searchsorted(self.runtime, end_time)

        self._filter_time_range_data(start_idx, end_idx)

        self.runtime = self.runtime[start_idx:end_idx]

    def _filter_time_range_data(self, start_idx, end_idx):
        if isinstance(self.processed_data, dict):
            for key, value in self.processed_data.items():
                if isinstance(value, dict) and 'runtime' in value and 'value' in value:
                    self.processed_data[key]['runtime'] = value['runtime'][start_idx:end_idx]
                    if isinstance(value['value'], list):
                        self.processed_data[key]['value'] = value['value'][start_idx:end_idx]
                elif isinstance(value, list) and value and isinstance(value[0], (int, float, np.number)):
                    self.processed_data[key] = value[start_idx:end_idx]
        elif isinstance(self.processed_data, list):
            if self.processed_data and isinstance(self.processed_data[0], (int, float, np.number)):
                self.processed_data = self.processed_data[start_idx:end_idx]

    def _filter_data_by_time(self, data, start_time, end_time):
        return data

    def _initialize_plot(self):
        self.ax.set_title(self.title)
        self.ax.set_xlabel(self.xlabel)
        self.ax.set_ylabel(self.ylabel)

        if self.show_zero_line:
            self.ax.axhline(0, color='black', linestyle='--', alpha=0.3)

        self._plot_lines()

        if self.show_legend:
            self.ax.legend(loc='best')

        if self.mode == 'animation':
            self._setup_animation()

    def _plot_lines(self):
        line_data_list = self._get_line_data()

        if len(line_data_list) == 1:
            line_data = line_data_list[0]
            label = line_data['label']
            x_data = line_data['x']
            y_data = line_data['y']
            style = line_data.get('style', {})

            line, = self.ax.plot(x_data, y_data, label=label, **{**self.line_style, **style})
            self.lines[line_data['label']] = line

            if len(self.robot_ids) == 1:
                single_robot_id = self.robot_ids[0]
                self.ax.set_title(f"{self.title} - Robot #{single_robot_id + 1}")
        else:
            for line_data in line_data_list:
                label = line_data['label']
                x_data = line_data['x']
                y_data = line_data['y']
                style = line_data.get('style', {})

                line, = self.ax.plot(x_data, y_data, label=label, **{**self.line_style, **style})
                self.lines[label] = line

        self._apply_custom_styling()

    def _get_line_data(self):
        raise NotImplementedError("Subclasses must implement _get_line_data")

    def _apply_custom_styling(self):
        pass

    def _apply_bounds_limits(self):
        if self.show_bounds and (self.bounds_lower is not None or self.bounds_upper is not None):
            self.ax.axhline(self.bounds_lower, color=self.bounds_color,
                           linestyle=self.bounds_style, alpha=self.bounds_alpha)
            self.ax.axhline(self.bounds_upper, color=self.bounds_color,
                           linestyle=self.bounds_style, alpha=self.bounds_alpha)

            if self.bounds_lower is not None and self.bounds_upper is not None:
                range_size = self.bounds_upper - self.bounds_lower
                expansion = range_size * 0.05
                y_min = self.bounds_lower - expansion
                y_max = self.bounds_upper + expansion
                self.ax.set_ylim([y_min, y_max])
            elif self.bounds_lower is not None:
                y_min = self.bounds_lower * 0.95
                self.ax.set_ylim(bottom=y_min)
            elif self.bounds_upper is not None:
                y_max = self.bounds_upper * 1.05
                self.ax.set_ylim(top=y_max)

    def _setup_animation(self):
        if hasattr(self, 'runtime') and len(self.runtime) > 0:
            self.y_limits = self.ax.get_ylim()
            self.vline = self.ax.plot(
                [self.runtime[0], self.runtime[0]],
                [self.y_limits[0], self.y_limits[1]],
                'r--', alpha=0.3
            )[0]

        if self.show_markers or self.show_value_text:
            self._setup_animation_markers()

    def _setup_animation_markers(self):
        for label in self.lines:
            if self.show_markers:
                marker, = self.ax.plot([np.nan], [np.nan], **self.marker_style)
                self.markers[label] = marker

            if self.show_value_text:
                text = self.ax.text(
                    np.nan, np.nan, '', **self.text_style,
                    verticalalignment='center',
                    horizontalalignment='left'
                )
                self.value_texts[label] = text

    def update(self, frame):
        if self.mode == 'animation' and hasattr(self, 'vline'):
            filtered_frame = frame
            if hasattr(self, 'index_range') and self.index_range:
                filtered_frame = frame - self.index_range[0]

            if filtered_frame < len(self.runtime):
                self.vline.set_data(
                    [self.runtime[filtered_frame], self.runtime[filtered_frame]],
                    self.y_limits
                )
                self._update_animation_markers(filtered_frame)

    
    def _update_animation_markers(self, frame):
        if not hasattr(self, 'runtime'):
            return

        max_frame = len(self.runtime)

        if frame >= max_frame:
            return

        current_time = self.runtime[frame]
        time_span = self.runtime[-1] - self.runtime[0] if len(self.runtime) > 1 else 1
        time_offset = time_span * 0.015

        x_limits = self.ax.get_xlim()

        for label, line in self.lines.items():
            current_value = self._get_current_value(label, frame)

            if current_value is not None and not np.isnan(current_value):
                if label in self.markers:
                    self.markers[label].set_data(current_time, current_value)

                if label in self.value_texts:
                    formatted_text = self._format_marker_text(label, current_value)
                    self.value_texts[label].set_text(formatted_text)

                    if current_time < (x_limits[0] + x_limits[1]) / 2:
                        self.value_texts[label].set_horizontalalignment('left')
                        self.value_texts[label].set_position((current_time + time_offset, current_value))
                    else:
                        self.value_texts[label].set_horizontalalignment('right')
                        self.value_texts[label].set_position((current_time - time_offset, current_value))
            else:
                if label in self.markers:
                    self.markers[label].set_data([], [])
                if label in self.value_texts:
                    self.value_texts[label].set_text("")

    def _format_marker_text(self, label, value):
        return f"{label}: {value:.4f}"

    def _get_current_value(self, label, frame):
        if not self.lines:
            return None

        if len(self.lines) == 1:
            original_label = list(self.lines.keys())[0]

            return self._get_current_value_by_label(original_label, frame)

        return self._get_current_value_by_label(label, frame)

    def _get_current_value_by_label(self, label, frame):
        return None

    def _get_line_label(self, base_label, robot_id=None):
        label = base_label
        if self.show_robot_ids and robot_id is not None:
            label += f", {self.robot_prefix}{robot_id + 1}"
        return label

    def _get_marker_label(self, robot_id, data_name):
        return f"{robot_id}-{data_name}"


