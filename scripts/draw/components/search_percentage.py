from utils import *

from .base import BaseComponent


class SearchPercentageComponent(BaseComponent):
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.grid_world = data["para"]["gridWorld"]
        self.title = kwargs.get('title', "Search Percentage Over Time")
        self.mode = kwargs.get('mode', 'global')

        # Get parameters from params field if it exists
        params = kwargs.get('params', {})
        self.show_milestones = params.get('show_milestones', True)

        # Extract grid dimensions
        self.x_num = self.grid_world["xNum"]
        self.y_num = self.grid_world["yNum"]
        self.total_cells = self.x_num * self.y_num

        # Extract runtime data
        self.runtime = [frame["runtime"] for frame in self.data]

        # Calculate search percentage over time
        self.search_percentages = self._calculate_search_percentages()

        self._initialize_plot()

    def _calculate_search_percentages(self):
        """Calculate the percentage of searched area at each time point"""
        searched_cells = set()
        percentages = []

        for frame in self.data:
            if "update" in frame and len(frame["update"]) > 0:
                # Add newly searched cells
                for grid in frame["update"]:
                    # Convert grid coordinates to a unique identifier
                    cell_id = (grid[0], grid[1])  # (x, y) coordinates
                    searched_cells.add(cell_id)

            # Calculate percentage of searched area
            percentage = len(searched_cells) / self.total_cells
            percentages.append(percentage)

        return percentages

    def _plot_basic(self, ax):
        """Plot basic search percentage over time"""
        ax.plot(self.runtime, self.search_percentages, 'b-', linewidth=2, label='Search Percentage')
        ax.fill_between(self.runtime, 0, self.search_percentages, alpha=0.3)
        ax.set_title(self.title)
        ax.set_xlabel('Time / s')
        ax.set_ylabel('Search Percentage')
        ax.set_ylim([0, 1.05])
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')

    def _plot_with_milestones(self, ax):
        """Plot with milestone markers at specific percentages"""
        self._plot_basic(ax)

        # Add milestone lines at 25%, 50%, 75%, 90%
        milestones = [0.25, 0.5, 0.75, 0.9]
        milestone_colors = ['orange', 'red', 'purple', 'darkred']

        for milestone, color in zip(milestones, milestone_colors):
            # Find the first time when this milestone is reached
            milestone_time = None
            for i, percentage in enumerate(self.search_percentages):
                if percentage >= milestone:
                    milestone_time = self.runtime[i]
                    break

            if milestone_time is not None:
                ax.axhline(y=milestone, color=color, linestyle='--', alpha=0.7,
                          label=f'{int(milestone*100)}% at t={milestone_time:.1f}s')
                ax.axvline(x=milestone_time, color=color, linestyle='--', alpha=0.3)

        ax.legend(loc='best')

    
    def _setup_animation_timeline(self, ax):
        """Setup vertical line for animation mode"""
        self.vline = ax.axvline(x=self.runtime[0], color='red', linestyle='--', alpha=0.7, label='Current Time')
        self.y_limits = ax.get_ylim()

    def _initialize_plot(self):
        """Initialize the plot based on mode"""
        if self.mode == 'animation':
            self._plot_basic(self.ax)
            self._setup_animation_timeline(self.ax)
        else:  # global, separate, group modes
            if self.show_milestones:
                self._plot_with_milestones(self.ax)
            else:
                self._plot_basic(self.ax)

    def update(self, num):
        """Update function for animation"""
        if self.mode == 'animation' and hasattr(self, 'vline'):
            self.vline.set_xdata([self.runtime[num], self.runtime[num]])