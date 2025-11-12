from utils import *
from .lines import Lines


def energy_interpreter(data):
    processed_data = {}

    runtime = [frame["runtime"] for frame in data["state"]]
    robot_num = data["config"]["num"]

    for i in range(robot_num):
        battery_values = [frame["robots"][i]["state"]["battery"] for frame in data["state"]]
        processed_data[i] = {
            'runtime': runtime,
            'value': battery_values
        }

    return processed_data


class EnergyComponent(Lines):
    def __init__(self, ax, data, robot_id=None, **kwargs):
        if robot_id is not None:
            kwargs['robot_ids'] = [robot_id]

        kwargs.setdefault('ylabel', 'Battery Level')
        kwargs.setdefault('show_bounds', True)
        kwargs.setdefault('bounds_lower', 3700)
        kwargs.setdefault('bounds_upper', 4200)
        kwargs.setdefault('bounds_color', 'red')
        kwargs.setdefault('bounds_style', '--')

        kwargs['data_interpreter'] = energy_interpreter

        Lines.__init__(self, ax, data, **kwargs)

        num_robots = len(self.robot_ids)
        if num_robots == 1:
            single_robot_id = self.robot_ids[0]
            self.ax.set_title(f'Energy Level, Robot #{single_robot_id + 1}')
        else:
            self.ax.set_title('Energy Levels, All Robots')

    def _get_line_data(self):
        line_data_list = []

        for robot_id in self.robot_ids:
            if robot_id in self.processed_data:
                robot_data = self.processed_data[robot_id]
                label = f'UAV #{robot_id + 1}'

                line_data_list.append({
                    'label': label,
                    'x': robot_data['runtime'],
                    'y': robot_data['value'],
                    'style': {}
                })

        return line_data_list

    def _format_marker_text(self, label, value):
        return f"{label}: {value:.0f}"

    def _get_current_value_by_label(self, label, frame):
        robot_id = int(label.split('#')[1].strip()) - 1
        if robot_id in self.processed_data:
            values = self.processed_data[robot_id]['value']
            if frame < len(values):
                return values[frame]
        return None


EnergyComponentNew = EnergyComponent


