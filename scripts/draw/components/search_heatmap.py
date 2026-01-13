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

        if "valid" in self.grid_world:
            self.valid_mask = np.array(self.grid_world["valid"], dtype=bool)
        else:
            self.valid_mask = np.ones((y_num, x_num), dtype=bool)

        all_rows = []
        all_cols = []
        all_runtimes = []

        for frame in reversed(self.data):
            if "update" in frame and len(frame["update"]) > 0:
                updates = frame["update"]
                for grid in updates:
                    x, y = grid[0], grid[1]
                    if self.valid_mask[y, x]:
                        all_rows.append(y)
                        all_cols.append(x)
                        all_runtimes.append(frame["runtime"])

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

        Z_masked = Z_flipped.copy()
        Z_masked[~np.flipud(self.valid_mask)] = np.nan

        xlim = self.grid_world["xLim"]
        ylim = self.grid_world["yLim"]
        extent = [xlim[0], xlim[1], ylim[0], ylim[1]]

        cmap = plt.cm.jet
        cmap.set_bad(color='white')

        valid_values = Z_masked[~np.isnan(Z_masked)]
        if len(valid_values) > 0:
            vmax = np.max(valid_values)
            vmin = np.min(valid_values)
        else:
            vmax = 1
            vmin = 0

        self.image = self.ax.imshow(Z_masked, cmap=cmap, aspect='auto', extent=extent, vmin=vmin, vmax=vmax)
        self.colorbar = plt.colorbar(self.image, ax=self.ax)
        self.colorbar.set_label('First Search Time / s')

    def update(self, num, dataNow=None):
        pass
