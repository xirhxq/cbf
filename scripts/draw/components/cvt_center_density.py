from utils import *
from .lines import Lines


def cvt_center_density_data_interpreter(data):
    processed_data = {}
    state = data["state"]
    robot_num = data["config"]["num"]

    runtime = [frame["runtime"] for frame in state]

    # Extract grid world parameters
    x_lim = data["para"]["gridWorld"]["xLim"]
    y_lim = data["para"]["gridWorld"]["yLim"]
    x_num = data["para"]["gridWorld"]["xNum"]
    y_num = data["para"]["gridWorld"]["yNum"]

    for i in range(robot_num):
        robot_id = i
        density_values = []

        for frame_idx, frame in enumerate(state):
            if "cvt" in frame["robots"][robot_id]:
                cvt_center = frame["robots"][robot_id]["cvt"]["center"]
                cvt_x = cvt_center[0]
                cvt_y = cvt_center[1]

                # Convert world coordinates to grid indices
                x_ratio = (cvt_x - x_lim[0]) / (x_lim[1] - x_lim[0])
                y_ratio = (cvt_y - y_lim[0]) / (y_lim[1] - y_lim[0])

                x_idx = int(np.clip(x_ratio * x_num, 0, x_num - 1))
                y_idx = int(np.clip(y_ratio * y_num, 0, y_num - 1))

                # Check if this grid cell has been explored
                # We need to infer this from the update data
                # The update data contains coordinates of explored cells
                explored_cells = set()
                if "update" in frame:
                    for update in frame["update"]:
                        explored_cells.add((update[0], update[1]))

                # Calculate density as inverse of exploration status
                # 1.0 = unexplored (high density), 0.0 = explored (low density)
                is_explored = (x_idx, y_idx) in explored_cells
                density = 0.0 if is_explored else 1.0

                density_values.append(density)
            else:
                density_values.append(1.0)  # Default to unexplored if no CVT data

        processed_data[f'Robot #{robot_id + 1}'] = {
            'runtime': runtime,
            'value': density_values
        }

    processed_data['runtime'] = runtime
    return processed_data


class CVTCenterDensityComponent(Lines):
    def __init__(self, ax, data, **kwargs):
        kwargs['data_interpreter'] = cvt_center_density_data_interpreter

        if 'title' not in kwargs:
            kwargs['title'] = 'CVT Center Density Over Time'
        if 'xlabel' not in kwargs:
            kwargs['xlabel'] = 'Time / s'
        if 'ylabel' not in kwargs:
            kwargs['ylabel'] = 'Density (1.0=Unexplored, 0.0=Explored)'

        kwargs.setdefault('show_bounds', True)
        kwargs.setdefault('bounds_lower', 0.0)
        kwargs.setdefault('bounds_upper', 1.0)
        kwargs.setdefault('bounds_color', 'gray')
        kwargs.setdefault('bounds_style', '--')

        # For global mode, show all robots in one plot
        if kwargs.get('mode') == 'global':
            kwargs['robot_ids'] = []  # Empty list means show all robots
        # For group and separate modes, each plot shows only its corresponding robot
        elif kwargs.get('mode') in ['group', 'separate']:
            # Keep the robot_ids as set by the drawer for individual plots
            pass

        super().__init__(ax, data, **kwargs)

    def _get_line_data(self):
        line_data_list = []

        for label, values in self.processed_data.items():
            if label == 'runtime':
                continue

            # In group or separate mode, filter by robot_ids
            if self.mode in ['group', 'separate'] and len(self.robot_ids) == 1:
                # Extract robot ID from label like "Robot #1"
                import re
                robot_match = re.search(r'Robot #(\d+)', label)
                if robot_match:
                    robot_id_from_label = int(robot_match.group(1)) - 1  # Convert to 0-based
                    if robot_id_from_label not in self.robot_ids:
                        continue  # Skip if this robot is not assigned to this plot
                else:
                    continue  # Skip if label doesn't match expected format

            if isinstance(values, dict) and 'value' in values:
                line_data = {
                    'label': label,
                    'x': values['runtime'],
                    'y': values['value'],
                    'style': {}
                }
                line_data_list.append(line_data)

        return line_data_list