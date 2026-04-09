import numpy as np
import matplotlib.pyplot as plt
from .base import BaseComponent


class PositionUncertaintyRepresentative(BaseComponent):
    """
    Position uncertainty time series for representative UAVs.
    Uses id_list parameter to select UAVs (0-indexed).
    """

    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data

        params = kwargs.get('params', {})

        # Priority: params['id_list'] > kwargs['id_list'] > default
        # drawer passes id_list=range(0,14) automatically, but we want params['id_list']
        if 'id_list' in params and params['id_list'] is not None:
            self.id_list = params['id_list']
        elif 'id_list' in kwargs:
            self.id_list = kwargs['id_list']
        else:
            self.id_list = [0, 2, 4, 6]  # Default: UAV 1, 3, 5, 7

        # Generate labels and colors dynamically
        self.labels = [f'UAV {i+1}' for i in self.id_list]
        # Use default matplotlib color cycle
        self.colors = params.get('colors', None)  # None = use default cycle

        self.uncertainty_data = self._extract_uncertainty_data()

        self.draw()

    def _extract_uncertainty_data(self):
        """Extract uncertainty time series for all UAVs"""
        n_robots = len(self.data['state'][0]['robots'])
        uncertainty_data = {}

        for robot_idx in range(n_robots):
            uncertainties = []
            timestamps = []

            for frame in self.data['state']:
                if robot_idx < len(frame['robots']) and 'uncertainty' in frame['robots'][robot_idx]:
                    uncertainties.append(frame['robots'][robot_idx]['uncertainty'])
                    timestamps.append(frame['runtime'])

            if uncertainties:
                uncertainty_data[robot_idx] = {
                    'timestamps': np.array(timestamps),
                    'uncertainties': np.array(uncertainties),
                    'final': uncertainties[-1],
                    'mean': np.mean(uncertainties),
                    'max': np.max(uncertainties)
                }

        return uncertainty_data

    def draw(self):
        """Draw time series for representative UAVs"""
        # Get default color cycle if colors not specified
        if self.colors is None:
            prop_cycle = plt.rcParams['axes.prop_cycle']
            colors = prop_cycle.by_key()['color']
        else:
            colors = self.colors

        for i, robot_idx in enumerate(self.id_list):
            if robot_idx not in self.uncertainty_data:
                continue
            data = self.uncertainty_data[robot_idx]
            color = colors[i % len(colors)]
            label = self.labels[i]

            self.ax.plot(data['timestamps'], data['uncertainties'],
                        label=label, color=color, linewidth=2, alpha=0.9)

        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position Uncertainty\n$\\epsilon_i = 3\\sqrt{\\lambda_{\\max}(\\mathbf{\\Sigma}_i)}$ (m)')
        self.ax.set_title('Position Uncertainty (UAV 1, 3, 5, 7)')
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.legend(loc='upper left', framealpha=0.9)


class PositionUncertaintyBoxplot(BaseComponent):
    """
    Box plot showing uncertainty distribution at a specific time.
    Shows all UAVs with individual markers.
    """

    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data

        params = kwargs.get('params', {})

        # Time point for box plot
        self.time_point = params.get('time_point', 150.0)

        # UAVs to show (Squad 1: UAVs 1-7)
        self.uav_range = params.get('uav_range', list(range(7)))  # 0-indexed: 0-6

        self.uncertainty_data = self._extract_uncertainty_data()

        self.draw()

    def _extract_uncertainty_data(self):
        """Extract uncertainty at specific time point"""
        n_robots = len(self.data['state'][0]['robots'])
        uncertainty_at_time = {}

        # Find frame closest to time_point
        target_frame = None
        min_diff = float('inf')

        for frame in self.data['state']:
            diff = abs(frame['runtime'] - self.time_point)
            if diff < min_diff:
                min_diff = diff
                target_frame = frame

        if target_frame:
            for robot_idx in range(n_robots):
                if robot_idx < len(target_frame['robots']) and 'uncertainty' in target_frame['robots'][robot_idx]:
                    uncertainty_at_time[robot_idx] = target_frame['robots'][robot_idx]['uncertainty']

        return uncertainty_at_time

    def draw(self):
        """Draw box plot with individual UAV markers"""
        # Get values for UAVs 1-7
        values = [self.uncertainty_data[i] for i in self.uav_range if i in self.uncertainty_data]
        uav_labels = [f'UAV {i+1}' for i in self.uav_range if i in self.uncertainty_data]

        if not values:
            return

        # Create positions for each UAV
        positions = list(range(1, len(values) + 1))

        # Calculate y-axis range for smart label positioning
        y_min, y_max = min(values), max(values)
        y_range = y_max - y_min
        threshold = y_min + 0.8 * y_range  # Put labels below if above 80% of range

        # Use better colors: tab10 palette (blue, orange, green, red, purple, brown, pink)
        tab10_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']

        for i, (pos, val, label) in enumerate(zip(positions, values, uav_labels)):
            color = tab10_colors[i % len(tab10_colors)]
            self.ax.plot(pos, val, 'o', color=color, markersize=10,
                        markeredgecolor='white', markeredgewidth=1.5, label=label)

            # Smart label positioning: above for low values, below for high values
            if val > threshold:
                # Place text below the point
                self.ax.text(pos, val - 0.8, f'{val:.1f}', ha='center', va='top',
                            color=color, fontweight='bold')
            else:
                # Place text above the point
                self.ax.text(pos, val + 0.5, f'{val:.1f}', ha='center', va='bottom',
                            color=color, fontweight='bold')

        # Set labels
        self.ax.set_ylabel('Position Uncertainty\n$\\epsilon_i = 3\\sqrt{\\lambda_{\\max}(\\mathbf{\\Sigma}_i)}$ (m)')
        self.ax.set_xlabel('UAV (Squad 1)')
        self.ax.set_xticks(positions)
        self.ax.set_xticklabels(uav_labels)
        self.ax.set_title(f'Position Uncertainty at t={self.time_point:.0f}s')
        self.ax.grid(True, alpha=0.3, axis='y', linestyle='--')

        # Set x-axis limits
        self.ax.set_xlim(0.5, len(positions) + 0.5)
