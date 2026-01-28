from utils import *
from .base import BaseComponent


class UncertaintyHeatmapComponent(BaseComponent):
    """
    Component for visualizing spatial distribution of position uncertainty.
    Shows UAV trajectories colored by uncertainty level, with lighter colors indicating higher uncertainty.
    """
    def __init__(self, ax, data, **kwargs):
        self.ax = ax
        self.data = data["state"]

        # Get grid world bounds for extent
        if "para" in data and "gridWorld" in data["para"]:
            self.grid_world = data["para"]["gridWorld"]
            self.xlim = self.grid_world["xLim"]
            self.ylim = self.grid_world["yLim"]
        else:
            # Default bounds if gridWorld not available
            self.xlim = [-200, 200]
            self.ylim = [0, 400]

        # Parameters
        params = kwargs.get('params', {})
        self.time_frame = params.get('time_frame', 'final')  # 'final' or specific frame index
        self.show_trajectories = params.get('show_trajectories', True)
        self.cmap = params.get('cmap', 'Greys')  # Lighter = higher uncertainty

        # Process data
        self._process_data()
        self._initialize_plot()

    def _process_data(self):
        """Extract positions and uncertainties for visualization"""
        self.robot_data = {}

        # Determine which frame(s) to use
        if self.time_frame == 'final':
            frame_indices = [len(self.data) - 1]
        elif isinstance(self.time_frame, int):
            frame_indices = [min(self.time_frame, len(self.data) - 1)]
        else:
            # Use all frames for trajectory view
            frame_indices = range(len(self.data))

        # Build mapping from robot_id to its data
        for frame_idx in frame_indices:
            frame = self.data[frame_idx]
            for robot in frame["robots"]:
                robot_id = robot["id"]
                if robot_id not in self.robot_data:
                    self.robot_data[robot_id] = {
                        'x': [],
                        'y': [],
                        'uncertainty': []
                    }

                x = robot["state"]["x"]
                y = robot["state"]["y"]
                uncertainty = robot.get("uncertainty", 0)

                self.robot_data[robot_id]['x'].append(x)
                self.robot_data[robot_id]['y'].append(y)
                self.robot_data[robot_id]['uncertainty'].append(uncertainty)

    def _initialize_plot(self):
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('x / m')
        self.ax.set_ylabel('y / m')
        self.ax.set_title('Position Uncertainty Distribution')

        # Set extent to match grid world
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)

        # Collect all uncertainty values for colormap normalization
        all_uncertainties = []
        for robot_id, data in self.robot_data.items():
            all_uncertainties.extend(data['uncertainty'])

        if not all_uncertainties:
            return

        # Normalize colormap: higher uncertainty = lighter color
        vmin = min(all_uncertainties)
        vmax = max(all_uncertainties)
        norm = plt.Normalize(vmin=vmin, vmax=vmax)
        cmap = plt.cm.get_cmap(self.cmap)

        # Plot each robot's trajectory/positions
        for robot_id, data in sorted(self.robot_data.items()):
            x = np.array(data['x'])
            y = np.array(data['y'])
            uncertainties = np.array(data['uncertainty'])

            if self.show_trajectories and len(x) > 1:
                # Plot trajectory with uncertainty-based coloring
                # Create segments for colored line
                points = np.array([x, y]).T.reshape(-1, 1, 2)
                segments = np.concatenate([points[:-1], points[1:]], axis=1)

                # Use average uncertainty of segment endpoints for color
                segment_uncertainties = (uncertainties[:-1] + uncertainties[1:]) / 2

                from matplotlib.collections import LineCollection
                lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=1.5, alpha=0.7)
                lc.set_array(segment_uncertainties)
                self.ax.add_collection(lc)
            else:
                # Plot points
                sc = self.ax.scatter(x, y, c=uncertainties, cmap=cmap, norm=norm,
                                   s=30, alpha=0.8, edgecolors='black', linewidth=0.5)

            # Add robot label at final position
            if len(x) > 0:
                self.ax.text(x[-1], y[-1], f'#{robot_id}', fontsize=8,
                           ha='center', va='bottom', fontweight='bold')

        # Add colorbar
        if all_uncertainties:
            sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
            sm.set_array([])
            self.colorbar = plt.colorbar(sm, ax=self.ax)
            self.colorbar.set_label('Position Uncertainty $\\epsilon_i$ (m)')

        # Add grid
        self.ax.grid(True, alpha=0.3)

    def update(self, frame):
        """Animation support - optional"""
        pass
