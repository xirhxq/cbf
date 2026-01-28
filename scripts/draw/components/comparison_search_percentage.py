from utils import *
from .base import BaseComponent
import os
import json


def search_percentage_interpreter(data):
    """Extract search percentage data from a single simulation run"""
    processed_data = {
        'runtime': [],
        'search_percentages': []
    }

    grid_world = data["para"]["gridWorld"]
    x_num = grid_world["xNum"]
    y_num = grid_world["yNum"]

    if "valid" in grid_world:
        valid_2d = np.array(grid_world["valid"])
        total_valid_cells = int(np.sum(valid_2d))
        valid_cells = set()
        for y in range(y_num):
            for x in range(x_num):
                if valid_2d[y][x]:
                    valid_cells.add((x, y))
    else:
        total_valid_cells = x_num * y_num
        valid_cells = None

    searched_cells = set()
    percentages = []

    for frame in data["state"]:
        processed_data['runtime'].append(frame["runtime"])

        if "update" in frame and len(frame["update"]) > 0:
            for grid in frame["update"]:
                cell_id = (grid[0], grid[1])
                if valid_cells is None or cell_id in valid_cells:
                    searched_cells.add(cell_id)

        percentage = len(searched_cells) / total_valid_cells if total_valid_cells > 0 else 0.0
        percentages.append(percentage)

    processed_data['search_percentages'] = percentages
    return processed_data


class ComparisonSearchPercentageComponent(BaseComponent):
    """
    Component for comparing search percentage curves across multiple simulation runs.
    Uses hardcoded comparison data from params.
    """
    def __init__(self, ax, data, **kwargs):
        """
        Args:
            ax: matplotlib axis
            data: current data folder path (not used, comparison data comes from params)
        """
        self.ax = ax
        self.params = kwargs.get('params', {})

        # Get hardcoded comparison data from params
        comparison_data = self.params.get('comparison_data', {})

        # Configuration
        self.show_milestones = self.params.get('show_milestones', True)
        self.milestones = self.params.get('milestones', [0.25, 0.5, 0.75, 1.0])
        self.method_colors = self.params.get('method_colors', [
            'blue', 'red', 'green', 'orange', 'purple', 'brown'
        ])
        self.show_violation_markers = self.params.get('show_violation_markers', True)

        # Process all comparison data sources
        self.processed_data = {}
        self._load_and_process_data(comparison_data)

        self._initialize_plot()

    def _load_and_process_data(self, comparison_data):
        """Load and process data from all comparison sources"""
        for method_name, folder_path in comparison_data.items():
            data_file = os.path.join(folder_path, 'data.json')
            if not os.path.exists(data_file):
                print(f"Warning: Data file not found for {method_name}: {data_file}")
                continue
            try:
                with open(data_file, 'r') as f:
                    data = json.load(f)
                self.processed_data[method_name] = search_percentage_interpreter(data)
                print(f"Loaded {method_name} from {folder_path}")
            except Exception as e:
                print(f"Error loading {method_name}: {e}")

    def _initialize_plot(self):
        """Initialize the comparison plot"""
        for i, (method_name, data) in enumerate(self.processed_data.items()):
            color = self.method_colors[i % len(self.method_colors)]
            runtime = np.array(data['runtime'])
            percentages = np.array(data['search_percentages'])

            # Check if simulation ended early due to constraint violation
            # Use duration as indicator: if < 300s, likely terminated by violation
            final_time = runtime[-1]
            final_pct = percentages[-1]
            is_violation = final_time < 300  # Early termination indicates violation
            is_finished = not is_violation  # Completed full simulation

            # Plot line
            label = method_name
            linestyle = '--' if is_violation else '-'

            # Show final percentage for both violations and finished runs
            if is_violation or is_finished:
                label += f' ({final_pct*100:.1f}%)'

            self.ax.plot(runtime, percentages, color=color, linewidth=2,
                        linestyle=linestyle, label=label, zorder=3)

            # Add fill with reduced transparency
            self.ax.fill_between(runtime, 0, percentages, alpha=0.08, color=color, zorder=2)

            # Mark violation point if simulation ended early
            if is_violation and self.show_violation_markers:
                self.ax.plot(runtime[-1], percentages[-1], 'x', color=color, markersize=10,
                           markeredgewidth=2, label='_nolegend_', zorder=4)

        # Add milestone lines
        if self.show_milestones:
            for milestone in self.milestones:
                self.ax.axhline(y=milestone, color='gray', linestyle=':',
                               alpha=0.5, linewidth=1)

        # Labels and styling
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Search Percentage (%)')
        self.ax.set_title('Search Performance Comparison')

        # Format y-axis as percentage
        self.ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f'{int(y*100)}%'))

        # Set limits
        self.ax.set_ylim([0, 1.05])
        self.ax.grid(True, alpha=0.3)

        # Legend
        self.ax.legend(loc='best', fontsize=9)

    def update(self, frame):
        """Animation update (not typically used for comparison plots)"""
        pass
