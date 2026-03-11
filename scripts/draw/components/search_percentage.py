from utils import *
from .lines import Lines


def search_percentage_interpreter(data):
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


class SearchPercentageComponent(Lines):
    def __init__(self, ax, data, **kwargs):
        kwargs.setdefault('title', "Search Percentage Over Time")
        kwargs.setdefault('ylabel', 'Search Percentage (%)')

        params = kwargs.get('params', {})
        self.show_milestones = params.get('show_milestones', True)
        self.milestones = params.get('milestones', [0.25, 0.5, 0.75, 1.0])
        self.milestone_colors = params.get('milestone_colors', ['orange', 'red', 'purple', 'green'])
        self.show_max_text = params.get('show_max_text', True)

        kwargs['data_interpreter'] = search_percentage_interpreter

        Lines.__init__(self, ax, data, **kwargs)

    def _get_line_data(self):
        return [{
            'label': 'Search Percentage',
            'x': self.runtime,
            'y': self.processed_data['search_percentages'],
            'style': {
                'color': 'b',
                'linewidth': 2
            }
        }]

    def _apply_custom_styling(self):
        self.ax.fill_between(self.runtime, 0, self.processed_data['search_percentages'], alpha=0.3)
        self.ax.set_ylim([0, 1.05])
        self.ax.grid(True, alpha=0.3)

        self.ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f'{int(y*100)}%'))

        if self.show_max_text:
            max_pct = self.processed_data['search_percentages'][-1]
            print(f"Search Percentage: Max {max_pct*100:.2f}%")
            self.ax.text(0.02, 0.98, f'Max: {max_pct*100:.1f}%',
                         transform=self.ax.transAxes, verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        if self.show_milestones:
            self._add_milestones()

    def _add_milestones(self):
        for milestone, color in zip(self.milestones, self.milestone_colors):
            milestone_time = None
            for i, percentage in enumerate(self.processed_data['search_percentages']):
                # Use a small epsilon for floating point comparison
                if percentage >= milestone - 1e-6:
                    milestone_time = self.runtime[i]
                    break

            if milestone_time is not None:
                self.ax.axhline(y=milestone, color=color, linestyle='--', alpha=0.7,
                                label=f'{int(milestone*100)}% at t={milestone_time:.1f}s')
                self.ax.axvline(x=milestone_time, color=color, linestyle='--', alpha=0.3)

    def _get_current_value_by_label(self, label, frame):
        if frame < len(self.processed_data['search_percentages']):
            return self.processed_data['search_percentages'][frame]
        return None

    def _format_marker_text(self, label, value):
        return f"{value*100:.1f}%"