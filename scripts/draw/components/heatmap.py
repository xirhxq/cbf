from utils import *

from .base import BaseComponent


class HeatmapComponent(BaseComponent):
    def __init__(self, ax, data, robot_id, title=None, **kwargs):
        self.ax = ax
        self.data = data["state"]
        self.robot_id = robot_id
        self.title = title or f"Heatmap, Robot #{robot_id + 1}"

        self.robot_positions = [
            (frame["robots"][robot_id]["state"]["x"], frame["robots"][robot_id]["state"]["y"])
            for frame in self.data
        ]

        x_coords = [pos[0] for pos in self.robot_positions]
        y_coords = [pos[1] for pos in self.robot_positions]

        heatmap, xedges, yedges = np.histogram2d(
            x_coords, y_coords,
            bins=20,
            range=[[-10, 10], [0, 20]]
        )
        self.heatmap = heatmap.T
        self.xedges = xedges
        self.yedges = yedges

        self._initialize_plot()

    def _initialize_plot(self):
        self.ax.set_title(self.title)
        self.ax.set_xlabel('X / m')
        self.ax.set_ylabel('Y / m')
        self.ax.set_aspect('equal')

        self.image = self.ax.imshow(
            self.heatmap,
            extent=[self.xedges[0], self.xedges[-1], self.yedges[0], self.yedges[-1]],
            origin='lower',
            cmap='jet'
        )

        plt.axis('off')

