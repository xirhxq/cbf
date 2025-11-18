from utils import *
from .lines import Lines


def control_input_lines_data_interpreter(data):
    state = data["state"]
    robot_num = data["config"]["num"]

    runtime = [frame["runtime"] for frame in state]

    first_frame_result = state[0]["robots"][0]["opt"]["result"]
    control_fields = list(first_frame_result.keys())

    processed_data = {}

    for i in range(robot_num):
        robot_id = i
        for field in control_fields:
            label_base = field
            if field == "vx":
                label_base = "Vel X"
            elif field == "vy":
                label_base = "Vel Y"
            elif field == "ax":
                label_base = "Accel X"
            elif field == "ay":
                label_base = "Accel Y"
            elif field == "yawRateRad":
                label_base = "Yaw Rate"
            else:
                label_base = field.capitalize()

            label = f"{label_base}, Robot #{robot_id + 1}"

            field_data = [frame["robots"][robot_id]["opt"]["result"][field] for frame in state]
            processed_data[label] = field_data

        if "vx" in control_fields and "vy" in control_fields:
            speed_data = []
            for frame in state:
                vx = frame["robots"][robot_id]["opt"]["result"]["vx"]
                vy = frame["robots"][robot_id]["opt"]["result"]["vy"]
                speed = np.linalg.norm([vx, vy])
                speed_data.append(speed)

            speed_label = f"Speed, Robot #{robot_id + 1}"
            processed_data[speed_label] = speed_data

    processed_data['runtime'] = runtime

    return processed_data


class ControlInputComponent(Lines):
    def __init__(self, ax, data, **kwargs):
        kwargs['data_interpreter'] = control_input_lines_data_interpreter

        if 'title' not in kwargs:
            kwargs['title'] = 'Control Input'
        if 'xlabel' not in kwargs:
            kwargs['xlabel'] = 'Time / s'
        if 'ylabel' not in kwargs:
            state = data.get("state", [])
            if state:
                first_frame_result = state[0]["robots"][0]["opt"]["result"]
                if "vx" in first_frame_result or "vy" in first_frame_result:
                    kwargs['ylabel'] = 'Velocity / m/s'
                elif "ax" in first_frame_result or "ay" in first_frame_result:
                    kwargs['ylabel'] = 'Acceleration / m/s²'
                else:
                    kwargs['ylabel'] = 'Value'
            else:
                kwargs['ylabel'] = 'Value'

        super().__init__(ax, data, **kwargs)

    def _default_data_processor(self):
        return {}

    def _get_line_data(self):
        line_data_list = []

        for label, values in self.processed_data.items():
            if label == 'runtime':
                continue

            import re
            robot_match = re.search(r'Robot #(\d+)', label)

            if robot_match:
                robot_id_from_label = int(robot_match.group(1)) - 1

                if robot_id_from_label in self.robot_ids:
                    line_data = {
                        'label': label,
                        'x': self.runtime,
                        'y': values,
                        'style': {}
                    }
                    line_data_list.append(line_data)
            elif len(self.robot_ids) == 0 or len(self.robot_ids) == data.get('config', {}).get('num', 0):
                line_data = {
                    'label': label,
                    'x': self.runtime,
                    'y': values,
                    'style': {}
                }
                line_data_list.append(line_data)

        return line_data_list