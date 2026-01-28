from utils import *
from .base import BaseComponent


def h_safe_interpreter(data):
    """
    Extract h_safe constraint data (safety CBF).
    Shows minimum distance between all UAV pairs over time.
    """
    processed_data = {
        'runtime': [frame["runtime"] for frame in data["state"]],
        'min_distances': [],
        'min_pairs': []  # Track which pair had minimum distance
    }

    # Get safe distance from config
    if 'config' in data and 'cbfs' in data['config']:
        safety_config = data['config']['cbfs'].get('without-slack', {}).get('safety', {})
        d_safe = safety_config.get('safe-distance', 5) if safety_config.get('on', False) else 5
    else:
        d_safe = 5

    for frame in data["state"]:
        robots = frame["robots"]
        min_dist = float('inf')
        min_pair = None

        # Check all pairs
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                r1, r2 = robots[i], robots[j]
                x1, y1 = r1["state"]["x"], r1["state"]["y"]
                x2, y2 = r2["state"]["x"], r2["state"]["y"]
                dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                if dist < min_dist:
                    min_dist = dist
                    min_pair = (r1['id'], r2['id'])

        processed_data['min_distances'].append(min_dist)
        processed_data['min_pairs'].append(min_pair)

    processed_data['d_safe'] = d_safe
    return processed_data


class HSafeComponent(BaseComponent):
    """
    Component for visualizing h_safe (safety) constraints.
    Shows minimum distance between all UAV pairs, with d_safe reference.
    """
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data
        self.processed_data = h_safe_interpreter(data)
        self._initialize_plot()

    def _initialize_plot(self):
        runtime = self.processed_data['runtime']
        min_distances = np.array(self.processed_data['min_distances'])
        d_safe = self.processed_data['d_safe']

        # Plot minimum distance
        self.ax.plot(runtime, min_distances, color='blue', linewidth=2, label='Minimum inter-UAV distance')

        # Fill safe region (above d_safe)
        self.ax.fill_between(runtime, d_safe, min_distances,
                           where=(min_distances >= d_safe),
                           alpha=0.3, color='green', label='Safe region')

        # Fill unsafe region (below d_safe) - if any
        if np.any(min_distances < d_safe):
            self.ax.fill_between(runtime, 0, d_safe,
                               alpha=0.3, color='red', label='Unsafe region')

        # Add reference line for d_safe
        self.ax.axhline(y=d_safe, color='red', linestyle='--',
                       linewidth=1.5, alpha=0.7, label=f'$d_{{safe}}$ ({d_safe}m)')

        # Labels and styling
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_title('Safety Constraints ($h_{safe}$)')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='best', fontsize=9)

        # Set y-axis limit
        max_dist = np.max(min_distances)
        self.ax.set_ylim(bottom=0, top=max_dist * 1.1)

    def update(self, frame):
        pass
