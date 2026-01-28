from utils import *
from .base import BaseComponent


def formation_distance_interpreter(data):
    """
    Extract formation distance data with uncertainty for representative pairs.
    Returns processed data showing distance + combined uncertainty over time.
    """
    processed_data = {
        'runtime': [frame["runtime"] for frame in data["state"]],
        'pairs': {}
    }

    # Select representative pairs to show
    # Squad 1: robots 1-7, Squad 2: robots 8-14
    # Format: (robot_id_1, robot_id_2, label, color)
    representative_pairs = [
        (2, 3, 'Squad 1 (intra-squad)', 'blue'),
        (9, 10, 'Squad 2 (intra-squad)', 'green'),
        (7, 8, 'Cross-squad link', 'red'),
    ]

    for robot_i, robot_j, label, color in representative_pairs:
        distances = []
        combined_uncertainties = []

        for frame in data["state"]:
            robots = frame["robots"]
            if robot_i <= len(robots) and robot_j <= len(robots):
                r1 = robots[robot_i - 1]
                r2 = robots[robot_j - 1]

                # Calculate distance
                x1, y1 = r1["state"]["x"], r1["state"]["y"]
                x2, y2 = r2["state"]["x"], r2["state"]["y"]
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                # Get uncertainties
                u1 = r1.get("uncertainty", 0)
                u2 = r2.get("uncertainty", 0)
                combined_uncertainty = u1 + u2

                distances.append(distance)
                combined_uncertainties.append(combined_uncertainty)

        pair_key = f'pair_{robot_i}_{robot_j}'
        processed_data['pairs'][pair_key] = {
            'label': label,
            'color': color,
            'robot_i': robot_i,
            'robot_j': robot_j,
            'distance': distances,
            'uncertainty': combined_uncertainties,
            'total': [d + u for d, u in zip(distances, combined_uncertainties)]
        }

    return processed_data


class FormationDistanceComponent(BaseComponent):
    """
    Component for visualizing formation distances with uncertainty buffers.
    Shows distance + combined uncertainty for representative UAV pairs.
    """
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data

        # Get max_range from config for reference line
        if 'config' in data and 'cbfs' in data['config']:
            comm_fixed_config = data['config']['cbfs'].get('without-slack', {}).get('comm-fixed', {})
            if comm_fixed_config.get('on', False) and 'max-range' in comm_fixed_config:
                self.max_range = comm_fixed_config['max-range']
            else:
                self.max_range = 850  # Default value
        else:
            self.max_range = 850

        # Process data
        self.processed_data = formation_distance_interpreter(data)

        self._initialize_plot()

    def _initialize_plot(self):
        runtime = self.processed_data['runtime']

        # Plot each pair
        for pair_key, pair_data in self.processed_data['pairs'].items():
            distance = np.array(pair_data['distance'])
            uncertainty = np.array(pair_data['uncertainty'])
            total = np.array(pair_data['total'])
            label = pair_data['label']
            color = pair_data['color']

            # Fill uncertainty buffer (from distance to distance+uncertainty)
            self.ax.fill_between(runtime, distance, total,
                               alpha=0.2, color=color, label=f'{label} (uncertainty)')

            # Plot distance line
            self.ax.plot(runtime, distance, color=color, linewidth=2, label=f'{label} (distance)')

        # Add reference line for max_range
        self.ax.axhline(y=self.max_range, color='black', linestyle='--',
                       linewidth=1.5, alpha=0.7, label=f'$d_{{loc}}$ ({self.max_range}m)')

        # Labels and styling
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_title('Formation Distance with Uncertainty Buffer')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='best', fontsize=9)

    def update(self, frame):
        """Animation support - optional"""
        pass
