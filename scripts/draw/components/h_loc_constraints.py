from utils import *
from .base import BaseComponent


def h_loc_interpreter(data):
    """
    Extract ALL h_loc constraint data (fixedCommCBF constraints).
    Shows distance + combined uncertainty for ALL constrained pairs.
    """
    processed_data = {
        'runtime': [frame["runtime"] for frame in data["state"]],
        'constraints': {}
    }

    # Get max_range from config
    if 'config' in data and 'cbfs' in data['config']:
        comm_fixed_config = data['config']['cbfs'].get('without-slack', {}).get('comm-fixed', {})
        max_range = comm_fixed_config.get('max-range', 850) if comm_fixed_config.get('on', False) else 850
    else:
        max_range = 850

    # Collect ALL unique constraints from first frame
    first_frame = data["state"][0]
    all_constraints = []

    for robot in first_frame['robots']:
        robot_id = robot['id']
        if 'cbfNoSlack' in robot:
            for cbf_name in robot['cbfNoSlack'].keys():
                if 'fixedCommCBF' in cbf_name:
                    all_constraints.append((robot_id, cbf_name))

    # Determine color for each constraint
    # Squad 1: robots 1-7, Squad 2: robots 8-14
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
                # Cross-squad (shouldn't happen based on data)
                category = 'cross'
                color = 'gray'
        else:
            continue

        # Skip if we already processed this pair (for robot-robot)
        # Each constraint is directional: robot i uses robot j as anchor
        # Only skip if we've already seen this pair in the opposite direction
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
        combined_uncertainties = []

        for frame in data["state"]:
            robots = frame["robots"]
            if robot_id <= len(robots):
                r1 = robots[robot_id - 1]

                # Get position of r2
                if isinstance(anchor_j, str) and anchor_j.startswith('base-'):
                    # Base station
                    base_idx = int(anchor_j.split('-')[1])
                    base_pos = data['config']['bases'][base_idx]
                    x2, y2 = base_pos[0], base_pos[1]
                    u2 = 0  # Base has no uncertainty
                else:
                    # Other robot
                    if anchor_j <= len(robots):
                        r2 = robots[anchor_j - 1]
                        x2, y2 = r2["state"]["x"], r2["state"]["y"]
                        u2 = r2.get("uncertainty", 0)
                    else:
                        continue

                x1, y1 = r1["state"]["x"], r1["state"]["y"]
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                u1 = r1.get("uncertainty", 0)
                combined_uncertainty = u1 + u2

                distances.append(distance)
                combined_uncertainties.append(combined_uncertainty)

        key = f'{robot_id}_{anchor_j}'
        processed_data['constraints'][key] = {
            'category': category,
            'color': color,
            'distance': distances,
            'uncertainty': combined_uncertainties,
            'total': [d + u for d, u in zip(distances, combined_uncertainties)]
        }

    processed_data['max_range'] = max_range
    return processed_data


class HLocComponent(BaseComponent):
    """
    Component for visualizing h_loc (localization) constraints.
    Shows ALL distance + combined uncertainty for constrained pairs, with d_loc reference.
    Colors: Blue = Squad 1 (robots 1-7), Green = Squad 2 (robots 8-14), Red = Base links.
    """
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data
        self.processed_data = h_loc_interpreter(data)
        self._initialize_plot()

    def _initialize_plot(self):
        runtime = self.processed_data['runtime']
        max_range = self.processed_data['max_range']

        # Group constraints by category for layered plotting
        categories = {
            'squad1': [],
            'squad2': [],
            'base': []
        }

        for key, constraint in self.processed_data['constraints'].items():
            category = constraint['category']
            if category in categories:
                categories[category].append(constraint)

        # Plot each category
        for category, constraints in categories.items():
            for constraint in constraints:
                distance = np.array(constraint['distance'])
                uncertainty = np.array(constraint['uncertainty'])
                total = np.array(constraint['total'])
                color = constraint['color']

                # Fill uncertainty buffer (lighter, more transparent)
                self.ax.fill_between(runtime, distance, total,
                                   alpha=0.15, color=color, linewidth=0)

                # Plot distance line (thinner for many lines)
                self.ax.plot(runtime, distance, color=color, linewidth=0.8, alpha=0.7)

        # Add reference line for max_range
        self.ax.axhline(y=max_range, color='red', linestyle='--',
                       linewidth=2, alpha=0.8, label=f'$d_{{loc}}$ ({max_range}m)')

        # Create custom legend with just category colors
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='blue', alpha=0.3, label='Squad 1 (UAVs 1-7)'),
            Patch(facecolor='green', alpha=0.3, label='Squad 2 (UAVs 8-14)'),
            Patch(facecolor='red', alpha=0.3, label='Base links'),
        ]
        legend_elements.append(
            self.ax.axhline(y=max_range, color='red', linestyle='--',
                          linewidth=2, alpha=0.8, label=f'$d_{{loc}}$ ({max_range}m)')
        )

        # Labels and styling
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_title('Localization Constraints ($h_{loc}$)')
        self.ax.grid(True, alpha=0.3)

        # Custom legend
        self.ax.legend(handles=legend_elements[:-1], loc='best', fontsize=9)

        # Set y-axis limit to show d_loc clearly
        self.ax.set_ylim(bottom=0, top=max_range)  # Show full range including d_loc

    def update(self, frame):
        pass
