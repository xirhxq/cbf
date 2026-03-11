from utils import *
from .base import BaseComponent


def valid_links_interpreter(data):
    """
    Extract valid link data - distances for links defined in fixedCommCBF constraints
    that are within the safe range (between d_safe and d_loc).

    - d_loc: maximum communication/localization distance (from fixedCommCBF config)
    - d_safe: minimum safety distance (from safety CBF config)

    Only shows links that are defined in the CBF constraints (same as h_loc).
    """
    processed_data = {
        'runtime': [frame["runtime"] for frame in data["state"]],
        'links': {}
    }

    # Get d_loc (max range) from config
    if 'config' in data and 'cbfs' in data['config']:
        comm_fixed_config = data['config']['cbfs'].get('without-slack', {}).get('comm-fixed', {})
        d_loc = comm_fixed_config.get('max-range', 850) if comm_fixed_config.get('on', False) else 850
    else:
        d_loc = 850

    # Get d_safe from config
    if 'config' in data and 'cbfs' in data['config']:
        safety_config = data['config']['cbfs'].get('without-slack', {}).get('safety', {})
        d_safe = safety_config.get('safe-distance', 10) if safety_config.get('on', False) else 10
    else:
        d_safe = 10

    # Collect ALL unique constraints from first frame (same logic as h_loc_interpreter)
    first_frame = data["state"][0]
    all_constraints = []

    for robot in first_frame['robots']:
        robot_id = robot['id']
        if 'cbfNoSlack' in robot:
            for cbf_name in robot['cbfNoSlack'].keys():
                if 'fixedCommCBF' in cbf_name:
                    all_constraints.append((robot_id, cbf_name))

    # Determine color for each constraint (same as h_loc)
    for robot_id, cbf_name in all_constraints:
        # Determine category and color
        if '(base-' in cbf_name:
            category = 'base'
            color = 'red'
        elif '(#' in cbf_name:
            other_id = int(cbf_name.split('(#')[1].split(')')[0])
            # Determine squad based on both robots
            if robot_id <= 7 and other_id <= 7:
                category = 'squad1'
                color = 'blue'
            elif robot_id >= 8 and other_id >= 8:
                category = 'squad2'
                color = 'green'
            else:
                category = 'cross'
                color = 'gray'
        else:
            continue

        # Skip if we already processed this pair (for robot-robot)
        # Same logic as h_loc: only skip if robot_id < other_id
        if '(#' in cbf_name and robot_id < int(cbf_name.split('(#')[1].split(')')[0]):
            continue

        # Extract constraint info
        if '(base-' in cbf_name:
            anchor_idx = int(cbf_name.split('(base-')[1].split(')')[0])
            anchor_j = f'base-{anchor_idx}'
        else:
            other_id = int(cbf_name.split('(#')[1].split(')')[0])
            anchor_j = other_id

        # Process this constraint
        distances = []

        for frame in data["state"]:
            robots = frame["robots"]
            if robot_id <= len(robots):
                r1 = robots[robot_id - 1]

                # Get position of anchor
                if isinstance(anchor_j, str) and anchor_j.startswith('base-'):
                    # Base station
                    base_idx = int(anchor_j.split('-')[1])
                    base_pos = data['config']['bases'][base_idx]
                    x2, y2 = base_pos[0], base_pos[1]
                else:
                    # Other robot
                    if anchor_j <= len(robots):
                        r2 = robots[anchor_j - 1]
                        x2, y2 = r2["state"]["x"], r2["state"]["y"]
                    else:
                        continue

                x1, y1 = r1["state"]["x"], r1["state"]["y"]
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                distances.append(distance)

        key = f'{robot_id}_{anchor_j}'
        processed_data['links'][key] = {
            'id1': robot_id,
            'id2': anchor_j,
            'category': category,
            'color': color,
            'distances': distances,
            'valid': [d_safe <= d <= d_loc for d in distances]
        }

    processed_data['d_loc'] = d_loc
    processed_data['d_safe'] = d_safe

    return processed_data


class ValidLinksComponent(BaseComponent):
    """
    Component for visualizing valid links between UAVs.
    A link is "valid" when the distance is in the safe range [d_safe, d_loc].

    Only shows links defined in fixedCommCBF constraints (same as h_loc).
    """
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data
        self.processed_data = valid_links_interpreter(data)
        self._initialize_plot()

    def _initialize_plot(self):
        runtime = self.processed_data['runtime']
        d_loc = self.processed_data['d_loc']
        d_safe = self.processed_data['d_safe']

        # Plot each link
        for key, link_data in self.processed_data['links'].items():
            distances = np.array(link_data['distances'])
            valid = np.array(link_data['valid'])
            color = link_data['color']

            # Plot distance line
            self.ax.plot(runtime, distances, color=color, linewidth=1, alpha=0.7)

        # Add reference lines
        self.ax.axhline(y=d_loc, color='red', linestyle='--',
                       linewidth=2, alpha=0.8, label=f'$d_{{loc}}$ ({d_loc}m)')
        self.ax.axhline(y=d_safe, color='orange', linestyle='--',
                       linewidth=2, alpha=0.8, label=f'$d_{{safe}}$ ({d_safe}m)')

        # Fill valid region
        self.ax.axhspan(d_safe, d_loc, alpha=0.15, color='green', label='Valid range')

        # Labels and styling
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_title('Loc. & Safety Constraint Satisfaction')
        self.ax.grid(True, alpha=0.3)

        # Create custom legend
        from matplotlib.patches import Patch
        from matplotlib.lines import Line2D
        legend_elements = [
            Patch(facecolor='green', alpha=0.3, label='Valid range'),
            Line2D([0], [0], color='blue', linewidth=1.5, label='Squad 1 (1-7)'),
            Line2D([0], [0], color='green', linewidth=1.5, label='Squad 2 (8-14)'),
            Line2D([0], [0], color='red', linewidth=1.5, label='Base links'),
            Line2D([0], [0], color='red', linestyle='--', linewidth=2, label=f'$d_{{loc}}$ ({d_loc}m)'),
            Line2D([0], [0], color='orange', linestyle='--', linewidth=2, label=f'$d_{{safe}}$ ({d_safe}m)'),
        ]
        self.ax.legend(handles=legend_elements, loc='best', fontsize=8)

        # Set y-axis limit
        max_dist = max([max(link['distances']) for link in self.processed_data['links'].values()]) if self.processed_data['links'] else d_loc
        self.ax.set_ylim(bottom=0, top=max(max_dist * 1.05, d_loc * 1.1))

    def update(self, frame):
        pass
