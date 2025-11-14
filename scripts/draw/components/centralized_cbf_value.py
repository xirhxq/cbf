from utils import *
from .lines import Lines


def centralized_cbf_interpreter(data):
    processed_data = {}

    runtime = [frame["runtime"] for frame in data["state"]]

    all_cbf_names = set()
    for frame in data["state"]:
        if 'centralized' in frame and 'cbfs' in frame['centralized']:
            all_cbf_names.update(frame['centralized']['cbfs'].keys())

    filtered_cbf_names = list(all_cbf_names)

    if not filtered_cbf_names:
        return {}

    for cbf_name in filtered_cbf_names:
        cbf_values = []
        for frame in data["state"]:
            if 'centralized' in frame and 'cbfs' in frame['centralized']:
                cbfs = frame['centralized']['cbfs']
                value = cbfs.get(cbf_name, np.nan)
            else:
                value = np.nan
            cbf_values.append(value)

        processed_data[cbf_name] = {
            'runtime': runtime,
            'value': cbf_values
        }

    return processed_data


class CentralizedCBFValueComponent(Lines):
    def __init__(self, ax, data, **kwargs):
        kwargs.setdefault('title', 'Centralized CBF Values')
        kwargs.setdefault('ylabel', 'CBF Value')
        kwargs.setdefault('show_zero_line', True)

        params = kwargs.get('params', {})

        internal_params_defaults = {
            'cbf_filter': 'all'
        }

        for param, default_value in internal_params_defaults.items():
            if param in params:
                setattr(self, param, params[param])
            else:
                setattr(self, param, default_value)

        for key, value in params.items():
            if key not in internal_params_defaults:
                kwargs[key] = value

        kwargs['data_interpreter'] = centralized_cbf_interpreter
        Lines.__init__(self, ax, data, **kwargs)

        if not self._has_centralized_data():
            raise ValueError("Data does not contain centralized optimization data")

    def _has_centralized_data(self):
        if not isinstance(self.data, dict) or 'state' not in self.data:
            return False

        if len(self.data['state']) == 0:
            return False

        first_state = self.data['state'][0]
        return 'centralized' in first_state and 'cbfs' in first_state['centralized']

    def _get_line_data(self):
        line_data_list = []

        filtered_cbfs = self._filter_cbf_names(list(self.processed_data.keys()))

        for cbf_name in filtered_cbfs:
            if cbf_name not in self.processed_data:
                continue

            cbf_data = self.processed_data[cbf_name]
            if all(np.isnan(cbf_data['value'])):
                continue

            label = self._generate_cbf_label(cbf_name)

            line_data_list.append({
                'label': label,
                'x': cbf_data['runtime'],
                'y': cbf_data['value'],
                'style': {}
            })

        return line_data_list

    def _filter_cbf_names(self, cbf_names):
        if self.cbf_filter == 'all':
            return cbf_names
        elif self.cbf_filter == 'comm':
            return [name for name in cbf_names if 'comm' in name.lower() or 'anchor' in name.lower()]
        elif self.cbf_filter == 'communication':
            return [name for name in cbf_names if 'comm' in name.lower() or 'anchor' in name.lower()]
        elif self.cbf_filter == 'cvt':
            return [name for name in cbf_names if 'cvt' in name.lower()]
        elif callable(self.cbf_filter):
            return [name for name in cbf_names if self.cbf_filter(name)]
        else:
            return cbf_names

    def _generate_cbf_label(self, cbf_name):
        if 'anchorCBF' in cbf_name:
            parts = cbf_name.split('_')
            robot_id = parts[1]
            anchor_idx = parts[2]
            return f"UAV {robot_id}-Anchor {anchor_idx}"
        elif 'commCBF' in cbf_name:
            robot_ids = cbf_name.split('_')[1:]
            return f"Comm {'-'.join(robot_ids)}"
        elif 'cvtCBF' in cbf_name:
            return 'CVT'
        else:
            return cbf_name

    def _get_current_value_by_label(self, label, frame):
        cbf_name = None
        for original_cbf_name in self.processed_data.keys():
            display_label = self._generate_cbf_label(original_cbf_name)
            if display_label == label:
                cbf_name = original_cbf_name
                break

        if cbf_name is None:
            return None

        if cbf_name in self.processed_data and frame < len(self.processed_data[cbf_name]['value']):
            return self.processed_data[cbf_name]['value'][frame]

        return None


