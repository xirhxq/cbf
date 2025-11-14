from utils import *
from .base import BaseComponent
import matplotlib.pyplot as plt
import math


def commcbf_uncertainty_interpreter(data):
    """
    Extract commCBF relationship data and calculate distances + uncertainties
    Returns processed data for visualization with stacked area plots
    """
    processed_data = {}
    runtime = [frame["runtime"] for frame in data["state"]]

    # Extract anchor points from config
    anchor_points = {}
    if 'config' in data and 'cbfs' in data['config']:
        comm_fixed_config = data['config']['cbfs']['without-slack']['comm-fixed']
        if comm_fixed_config.get('on', False) and 'bases' in comm_fixed_config:
            for i, base in enumerate(comm_fixed_config['bases']):
                anchor_points[f"anchor_{i}"] = base

    # Infer commCBF relationships from formation data (works for both centralized and distributed)
    commcbf_relationships = set()

    # First, try to find CBFs in centralized data if available
    if data["state"] and 'centralized' in data["state"][0] and 'cbfs' in data["state"][0]['centralized']:
        for frame in data["state"]:
            cbfs = frame['centralized']['cbfs']
            for cbf_name in cbfs.keys():
                if cbf_name.startswith('commCBF_') or cbf_name.startswith('anchorCBF_'):
                    commcbf_relationships.add(cbf_name)

    # If no centralized CBFs found, infer from formation data
    if not commcbf_relationships and data["state"]:
        # Get formation data from first frame
        first_frame = data["state"][0]
        if 'formation' in first_frame:
            formations = first_frame['formation']

            # Infer robot-to-robot relationships from anchorIds
            for robot in formations:
                if 'anchorIds' in robot:
                    for anchor_id in robot['anchorIds']:
                        if anchor_id > 0:  # Only positive IDs are real robots
                            robot_id = robot['id']
                            commcbf_relationships.add(f"commCBF_{max(robot_id, anchor_id)}_{min(robot_id, anchor_id)}")

            # Infer robot-to-anchor relationships from anchorPoints and bases config
            if 'bases' in comm_fixed_config:
                bases = comm_fixed_config['bases']
                for robot in formations:
                    if 'anchorPoints' in robot:
                        robot_id = robot['id']
                        for i, base in enumerate(bases):
                            if [base[0], base[1]] in robot['anchorPoints']:
                                commcbf_relationships.add(f"anchorCBF_{robot_id}_{i}")

        # Fallback: also check distributed CBF names in individual robots
        if not commcbf_relationships and 'robots' in first_frame:
            for robot in first_frame['robots']:
                if 'cbfNoSlack' in robot:
                    for cbf_name in robot['cbfNoSlack'].keys():
                        if cbf_name.startswith('fixedCommCBF') and '#' in cbf_name:
                            # Extract robot ID from fixedCommCBF(#X) format
                            import re
                            match = re.search(r'#(\d+)', cbf_name)
                            if match:
                                other_id = int(match.group(1))
                                robot_id = robot['id']
                                commcbf_relationships.add(f"commCBF_{max(robot_id, other_id)}_{min(robot_id, other_id)}")

    # Process each commCBF relationship
    for cbf_name in commcbf_relationships:
        distances = []
        uncertainties = []

        if cbf_name.startswith('commCBF_'):
            # Robot-to-robot communication: commCBF_i_j
            parts = cbf_name.split('_')
            robot_i_id = int(parts[1])
            robot_j_id = int(parts[2])

            for frame in data["state"]:
                robots = frame["robots"]
                robot_i = robots[robot_i_id - 1]
                robot_j = robots[robot_j_id - 1]

                # Calculate distance
                x1, y1 = robot_i["state"]["x"], robot_i["state"]["y"]
                x2, y2 = robot_j["state"]["x"], robot_j["state"]["y"]
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                # Sum uncertainties
                uncertainty_sum = robot_i.get("uncertainty", 0) + robot_j.get("uncertainty", 0)

                distances.append(distance)
                uncertainties.append(uncertainty_sum)

        elif cbf_name.startswith('anchorCBF_'):
            # Robot-to-anchor communication: anchorCBF_i_j
            parts = cbf_name.split('_')
            robot_id = int(parts[1])
            anchor_idx = int(parts[2])

            anchor_key = f"anchor_{anchor_idx}"
            if anchor_key in anchor_points:
                anchor_pos = anchor_points[anchor_key]

                for frame in data["state"]:
                    robot = frame["robots"][robot_id - 1]

                    # Calculate distance
                    x1, y1 = robot["state"]["x"], robot["state"]["y"]
                    x2, y2 = anchor_pos[0], anchor_pos[1]
                    distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

                    # Robot uncertainty only (anchor has no uncertainty)
                    uncertainty = robot.get("uncertainty", 0)

                    distances.append(distance)
                    uncertainties.append(uncertainty)

        # Store processed data
        processed_data[cbf_name] = {
            'runtime': runtime,
            'distance': distances,
            'uncertainty': uncertainties,
            'total': [d + u for d, u in zip(distances, uncertainties)]
        }

    return processed_data


class CommCBFUncertaintyComponent(BaseComponent):
    """
    Component for visualizing commCBF relationships with stacked area plots
    Shows distance + uncertainty for each communication constraint in separate subplots
    """

    def __init__(self, fig, data, **kwargs):
        self.fig = fig
        self.data = data

        self.title = kwargs.get('title', 'CommCBF Distance + Uncertainty')
        self.xlabel = kwargs.get('xlabel', 'Time / s')
        self.ylabel = kwargs.get('ylabel', 'Distance + Uncertainty')

        self.show_legend = kwargs.get('show_legend', True)
        self.time_range = kwargs.get('time_range', None)

        # CBF filtering options
        params = kwargs.get('params', {})
        self.cbf_filter = params.get('cbf_filter', 'all')  # 'all', 'comm', 'anchor'

        # Layout options
        self.max_cols = params.get('max_cols', 3)  # Maximum number of subplots per row

        # Uncertainty display mode: 'stacked' (default) or 'from_max_range'
        self.uncertainty_mode = params.get('uncertainty_mode', 'stacked')

        # Max range for uncertainty reference line (if uncertainty_mode is 'from_max_range')
        self.max_range = params.get('max_range', 850)  # Default from comm-fixed config

        # Visual style
        self.distance_alpha = kwargs.get('distance_alpha', 0.7)
        self.uncertainty_alpha = kwargs.get('uncertainty_alpha', 0.5)
        self.distance_color = kwargs.get('distance_color', 'lightblue')
        self.uncertainty_color = kwargs.get('uncertainty_color', 'lightcoral')

        # Process data
        self.processed_data = commcbf_uncertainty_interpreter(data)

        if not self.processed_data:
            raise ValueError("No commCBF data found for uncertainty visualization")

        self._apply_time_filter()
        self._create_subplots()

    def _apply_time_filter(self):
        if not self.time_range or len(self.time_range) != 2:
            return

        start_time, end_time = self.time_range

        for cbf_name, cbf_data in self.processed_data.items():
            runtime = cbf_data['runtime']
            if len(runtime) < 2:
                continue

            start_idx = np.searchsorted(runtime, start_time)
            end_idx = np.searchsorted(runtime, end_time, side='right')

            if end_idx > start_idx:
                self.processed_data[cbf_name]['runtime'] = runtime[start_idx:end_idx]
                self.processed_data[cbf_name]['distance'] = cbf_data['distance'][start_idx:end_idx]
                self.processed_data[cbf_name]['uncertainty'] = cbf_data['uncertainty'][start_idx:end_idx]
                self.processed_data[cbf_name]['total'] = cbf_data['total'][start_idx:end_idx]

    def _create_subplots(self):
        """Create separate subplot for each commCBF relationship"""
        filtered_cbfs = self._filter_cbf_names(list(self.processed_data.keys()))

        if not filtered_cbfs:
            return

        # Clear existing axes created by GridLayout (since we manage our own subplots)
        for ax in self.fig.axes[:]:
            ax.remove()

        n_cbfs = len(filtered_cbfs)

        # Calculate grid layout
        n_cols = min(self.max_cols, n_cbfs)
        n_rows = math.ceil(n_cbfs / n_cols)

        # Create subplots
        for i, cbf_name in enumerate(filtered_cbfs):
            if cbf_name not in self.processed_data:
                continue

            ax = self.fig.add_subplot(n_rows, n_cols, i + 1)
            self._plot_single_cbf(ax, cbf_name)

        # Set overall title
        self.fig.suptitle(self.title, fontsize=14, fontweight='bold')

        # Adjust layout
        self.fig.tight_layout(rect=[0, 0, 1, 0.96])  # Make room for suptitle

    def _plot_single_cbf(self, ax, cbf_name):
        """Plot a single commCBF relationship on its subplot"""
        cbf_data = self.processed_data[cbf_name]
        runtime = cbf_data['runtime']
        distance = np.array(cbf_data['distance'])
        uncertainty = np.array(cbf_data['uncertainty'])

        if len(runtime) == 0:
            return

        # Generate label
        label = self._generate_cbf_label(cbf_name)

        if self.uncertainty_mode == 'stacked':
            # Stacked mode: distance + uncertainty
            ax.fill_between(runtime, 0, distance,
                           alpha=self.distance_alpha,
                           color=self.distance_color,
                           label='Distance')

            ax.fill_between(runtime, distance, distance + uncertainty,
                           alpha=self.uncertainty_alpha,
                           color=self.uncertainty_color,
                           label='Uncertainty')

            # Add line for total (distance + uncertainty)
            total = distance + uncertainty
            ax.plot(runtime, total,
                   color='black',
                   linewidth=1.5,
                   alpha=0.8,
                   linestyle='-',
                   label='Total')

            # Add red dashed line at max_range
            ax.axhline(y=self.max_range, color='red', linestyle='--', alpha=0.7,
                      linewidth=1.5, label='Max Range')

            ax.set_ylim(bottom=0)

        elif self.uncertainty_mode == 'from_max_range':
            # From max range mode: draw down from max_range line
            max_range_line = np.full_like(runtime, self.max_range)

            # Draw uncertainty region (from max_range down by uncertainty amount)
            ax.fill_between(runtime, self.max_range, self.max_range - uncertainty,
                           alpha=self.uncertainty_alpha,
                           color=self.uncertainty_color,
                           label='Uncertainty')

            # Draw distance line (no fill)
            ax.plot(runtime, distance,
                   color=self.distance_color,
                   linewidth=2.0,
                   alpha=0.8,
                   linestyle='-',
                   label='Distance')

            # Add red dashed line at max_range
            ax.axhline(y=self.max_range, color='red', linestyle='--', alpha=0.7,
                      linewidth=1.5, label='Max Range')

            # Set y-limits
            y_min = max(0, min(self.max_range - np.max(uncertainty) - 50, 0))
            y_max = max(self.max_range + 100, np.max(distance) + 100)
            ax.set_ylim(bottom=y_min, top=y_max)

        # Set subplot title and labels
        ax.set_title(f'{label}')
        ax.set_xlabel(self.xlabel)
        ax.set_ylabel(self.ylabel)
        ax.grid(True, alpha=0.3)

        if self.show_legend:
            # For better legend display, reduce the number of items if needed
            ax.legend(loc='best', fontsize=7, framealpha=0.9)

    
    def _filter_cbf_names(self, cbf_names):
        """Filter CBF names based on the filter setting"""
        if self.cbf_filter == 'all':
            return cbf_names
        elif self.cbf_filter == 'comm':
            return [name for name in cbf_names if name.startswith('commCBF_')]
        elif self.cbf_filter == 'anchor':
            return [name for name in cbf_names if name.startswith('anchorCBF_')]
        elif callable(self.cbf_filter):
            return [name for name in cbf_names if self.cbf_filter(name)]
        else:
            return cbf_names

    def _generate_cbf_label(self, cbf_name):
        """Generate user-friendly labels for CBF names"""
        if cbf_name.startswith('anchorCBF_'):
            parts = cbf_name.split('_')
            robot_id = parts[1]
            anchor_idx = parts[2]
            return f'UAV {robot_id}-Anchor {anchor_idx}'
        elif cbf_name.startswith('commCBF_'):
            parts = cbf_name.split('_')
            robot_i = parts[1]
            robot_j = parts[2]
            return f'UAV {robot_i}-{robot_j}'
        else:
            return cbf_name

    def update(self, frame):
        """Animation support - update current time indicator"""
        if hasattr(self, 'runtime') and len(self.runtime) > 0:
            # Implementation for animation if needed
            pass