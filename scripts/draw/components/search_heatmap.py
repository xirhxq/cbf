from utils import *

from .base import BaseComponent


class SearchHeatmapComponent(BaseComponent):
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.grid_world = data["para"]["gridWorld"]

        x_num = self.grid_world["xNum"]
        y_num = self.grid_world["yNum"]
        self.Z = np.full((y_num, x_num), np.nan)

        all_rows = []
        all_cols = []
        all_runtimes = []

        for frame in reversed(self.data):
            if "update" in frame and len(frame["update"]) > 0:
                updates = frame["update"]
                all_rows.extend([grid[1] for grid in updates])
                all_cols.extend([grid[0] for grid in updates])
                all_runtimes.extend([frame["runtime"]] * len(updates))

        rows = np.array(all_rows)
        cols = np.array(all_cols)
        runtimes = np.array(all_runtimes)

        self.Z[rows, cols] = runtimes

        self._initialize_plot()

    def _initialize_plot(self):
        self.ax.set_aspect(1)
        self.ax.set_xlabel('x/m')
        self.ax.set_ylabel('y/m')
        self.ax.set_title('Search Heatmap')

        Z_flipped = np.flipud(self.Z)

        xlim = self.grid_world["xLim"]
        ylim = self.grid_world["yLim"]
        extent = [xlim[0], xlim[1], ylim[0], ylim[1]]

        cmap = plt.cm.jet
        cmap.set_bad(color='white')

        valid_values = self.Z[~np.isnan(self.Z)]
        if len(valid_values) > 0:
            vmax = np.max(valid_values)
            vmin = np.min(valid_values)
        else:
            vmax = 1
            vmin = 0
            
        self.image = self.ax.imshow(Z_flipped, cmap=cmap, aspect='auto', extent=extent, vmin=vmin, vmax=vmax)
        self.colorbar = plt.colorbar(self.image, ax=self.ax)
        self.colorbar.set_label('First Search Time / s')

    def update(self, num, dataNow=None):
        pass
