from utils import *
from .base import BaseComponent
import numpy as np
import matplotlib.colors as mcolors
import matplotlib.cm as mcm
from matplotlib.patches import Wedge, Circle, Ellipse, FancyArrowPatch
from matplotlib.lines import Line2D


class MapAnimationComponent(BaseComponent):
    MAX_FORMATION_ARROWS = 100
    MAX_COV_LINES = 100

    def __init__(self, ax, data, id_list, **kwargs):
        self.data = data
        self.ax = ax
        self.id_list = id_list
        self.interval = self.data["state"][1]["runtime"] - self.data["state"][0]["runtime"]
        self.fps = int(1 / self.interval)
        self.worldX = [data[0] for data in self.data["para"]["world"]["boundary"]]
        self.worldY = [data[1] for data in self.data["para"]["world"]["boundary"]]
        self.gridWorldJson = self.data["para"]["gridWorld"]
        self.robotAnnotation = True
        self.showYaw = True
        self.showCVT = True
        self.showAxis = False
        self.bigTimeText = kwargs.get('big_time_text', True)
        self.showCovarianceFormation = kwargs.get('show_covariance_formation', True)

        params = kwargs.get('params', {})
        self.colormap = params.get('colormap', 'coolwarm')
        self.shotList = params.get('shotList', [])
        self.showTimeTitle = params.get('show_time_title', True)
        self.showCovEllipse = params.get('show_cov_ellipse', True)
        self.showCovText = params.get('show_cov_text', True)
        self.annotationFontSize = params.get('annotation_font_size', 8)
        self.showCharge = params.get('show_charge', False)

        self.christmas_cmap = mcolors.LinearSegmentedColormap.from_list(
            'christmas',
            ['#1a5f1a', '#8b0000'],
            N=256
        )

        if self.colormap == 'christmas':
            self.search_cmap = self.christmas_cmap
            self.cov_robot_color = 'green'
            self.cov_base_color = '#8b0000'
        elif self.colormap == 'coolwarm':
            self.search_cmap = mcm.get_cmap('coolwarm')
            self.cov_robot_color = 'blue'
            self.cov_base_color = 'red'
        else:
            self.search_cmap = mcm.get_cmap(self.colormap)
            self.cov_robot_color = 'blue'
            self.cov_base_color = 'red'

        self.showPositionCovariance = (
            "position_covariance" in self.data["config"] and
            self.data["config"]["position_covariance"]["enable"]
        )

        self.x = np.linspace(self.gridWorldJson["xLim"][0], self.gridWorldJson["xLim"][1],
                             self.gridWorldJson["xNum"])
        self.y = np.linspace(self.gridWorldJson["yLim"][0], self.gridWorldJson["yLim"][1],
                             self.gridWorldJson["yNum"])
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.Z = np.zeros((self.gridWorldJson["xNum"], self.gridWorldJson["yNum"]))
        self.zExtent = self.gridWorldJson["xLim"] + self.gridWorldJson["yLim"]

        if "valid" in self.gridWorldJson:
            valid_2d = np.array(self.gridWorldJson["valid"])
            self.valid_mask = valid_2d.T
        else:
            self.valid_mask = np.ones((self.gridWorldJson["xNum"], self.gridWorldJson["yNum"]), dtype=bool)

        self.zUpdatedIndex = 0

        world_x_size = self.data["para"]["world"]["lim"][0][1] - self.data["para"]["world"]["lim"][0][0]
        world_y_size = self.data["para"]["world"]["lim"][1][1] - self.data["para"]["world"]["lim"][1][0]
        world_size = max(world_x_size, world_y_size)
        self.wedge_radius = world_size * 0.05

        self._setup_static_elements()
        self._setup_dynamic_artists()

    def _setup_static_elements(self):
        self.ax.set_xlim(self.data["para"]["world"]["lim"][0])
        self.ax.set_ylim(self.data["para"]["world"]["lim"][1])
        self.world_line, = self.ax.plot(self.worldX, self.worldY, 'k')

        if self.showCharge:
            pos_charge = self.data["para"]["world"]["charge"]["pos"]
            dist_charge = self.data["para"]["world"]["charge"]["dist"]
            for pos, dist in zip(pos_charge, dist_charge):
                charge_patch = Circle(xy=(pos[0], pos[1]), radius=dist, alpha=0.5)
                self.ax.add_patch(charge_patch)

        if not getattr(self, 'showAxis', True):
            self.ax.set_axis_off()

    def _setup_dynamic_artists(self):
        self.heatmap_img = self.ax.imshow(
            np.zeros((self.gridWorldJson["yNum"], self.gridWorldJson["xNum"])),
            alpha=0.3, extent=self.zExtent,
            origin='lower', cmap=self.search_cmap, vmin=0, vmax=1
        )

        self.robot_scatter = self.ax.scatter([], [], c='r', s=100, alpha=0.5)

        searching_method = self.data["config"]["searching"]["method"]
        self.searching_method = searching_method
        self.searching_params = self.data["config"]["searching"][searching_method]

        self.search_patches = []
        for _ in self.id_list:
            if searching_method == "downward":
                radius = self.searching_params["radius"]
                patch = Circle((0, 0), radius, alpha=0.3, visible=False)
            elif searching_method == "front-sector":
                outer_radius = self.searching_params["outer-radius"]
                inner_radius = self.searching_params.get("inner-radius", 0)
                if inner_radius > 0:
                    patch = Wedge((0, 0), outer_radius, 0, 0, width=outer_radius - inner_radius, alpha=0.3, visible=False)
                else:
                    patch = Wedge((0, 0), outer_radius, 0, 0, alpha=0.3, visible=False)
            elif searching_method == "front-cone":
                patch = Ellipse((0, 0), 0, 0, angle=0, alpha=0.3, facecolor='gray', edgecolor='none', visible=False)
            else:
                raise ValueError(f"Unknown searching method: {searching_method}")
            self.ax.add_patch(patch)
            self.search_patches.append(patch)

        self.cov_ellipses = []
        if self.showCovEllipse and self.showPositionCovariance:
            for _ in self.id_list:
                ellipse = Ellipse((0, 0), 0, 0, angle=0, facecolor='red', alpha=0.2, edgecolor='red', linewidth=1, visible=False)
                self.ax.add_patch(ellipse)
                self.cov_ellipses.append(ellipse)
        else:
            self.cov_ellipses = [None] * len(self.id_list)

        self.annotations = []
        for _ in self.id_list:
            ann = self.ax.text(0, 0, '', fontsize=self.annotationFontSize, visible=False)
            self.annotations.append(ann)

        self.cvt_lines = []
        self.cvt_centers = []
        if self.data["config"]["cbfs"]["with-slack"]["cvt"]["on"] and self.showCVT:
            for _ in self.id_list:
                line, = self.ax.plot([], [], 'k')
                center, = self.ax.plot([], [], '*', color='lime')
                self.cvt_lines.append(line)
                self.cvt_centers.append(center)

        self.formation_arrows = []
        for _ in range(self.MAX_FORMATION_ARROWS):
            line, = self.ax.plot([], [], '-', color='gray', alpha=0.3, linewidth=1.5, visible=False)
            self.formation_arrows.append(line)

        self.cov_formation_lines = []
        for _ in range(self.MAX_COV_LINES):
            line, = self.ax.plot([], [], '-', alpha=0.4, linewidth=1.5, visible=False)
            self.cov_formation_lines.append(line)

        self.time_text = None
        if self.showTimeTitle:
            if self.bigTimeText:
                self.time_text = self.ax.text(0.5, 0.95, '', transform=self.ax.transAxes,
                                              fontsize=25, ha='center', va='top')
            else:
                self.time_text = self.ax.text(0.05, 0.95, '', transform=self.ax.transAxes)

        self.formation_arrow_idx = 0
        self.cov_line_idx = 0

    def updateZ(self, num, dataNow=None):
        if dataNow is None:
            dataNow = self.data["state"][num]

        if "update" in dataNow and len(dataNow["update"]):
            self.Z[*zip(*dataNow["update"])] = 1

    def update(self, num, dataNow=None):
        if dataNow is None:
            dataNow = self.data["state"][num]

        while self.zUpdatedIndex < num:
            self.updateZ(self.zUpdatedIndex)
            self.zUpdatedIndex += 1

        Z_masked = self.Z.copy()
        Z_masked[~self.valid_mask] = np.nan
        self.heatmap_img.set_data(Z_masked.T)

        robotX = [dataNow["robots"][i]["state"]["x"] for i in self.id_list]
        robotY = [dataNow["robots"][i]["state"]["y"] for i in self.id_list]
        robotYawDeg = [math.degrees(dataNow["robots"][i]["state"]["yawRad"]) for i in self.id_list]

        self.robot_scatter.set_offsets(np.c_[robotX, robotY])

        for line in self.formation_arrows:
            line.set_visible(False)
        for line in self.cov_formation_lines:
            line.set_visible(False)

        self.formation_arrow_idx = 0
        self.cov_line_idx = 0

        if "formation" in dataNow and dataNow["formation"] != [None]:
            id2Position = {robot["id"]: (robot["state"]["x"], robot["state"]["y"]) for robot in dataNow["robots"]}
            for myJson in dataNow["formation"]:
                if len(myJson) == 0 or not myJson:
                    continue
                if myJson["id"] - 1 not in self.id_list:
                    continue
                myPosition = id2Position[myJson["id"]]
                for anchorPoint in myJson.get("anchorPoints", []):
                    if self.formation_arrow_idx < len(self.formation_arrows):
                        line = self.formation_arrows[self.formation_arrow_idx]
                        line.set_data([myPosition[0], anchorPoint[0]], [myPosition[1], anchorPoint[1]])
                        line.set_visible(True)
                        self.formation_arrow_idx += 1
                for neighbor_id in myJson.get("anchorIds", []):
                    neighbor_pos = id2Position[neighbor_id]
                    if self.formation_arrow_idx < len(self.formation_arrows):
                        line = self.formation_arrows[self.formation_arrow_idx]
                        line.set_data([myPosition[0], neighbor_pos[0]], [myPosition[1], neighbor_pos[1]])
                        line.set_visible(True)
                        self.formation_arrow_idx += 1

        if self.showCovarianceFormation and "covariance_formation" in dataNow and dataNow["covariance_formation"] != [None]:
            id2Position = {robot["id"]: (robot["state"]["x"], robot["state"]["y"]) for robot in dataNow["robots"]}
            for myJson in dataNow["covariance_formation"]:
                if len(myJson) == 0 or not myJson:
                    continue
                if myJson["id"] - 1 not in self.id_list:
                    continue
                myPosition = id2Position[myJson["id"]]

                for anchor_id in myJson.get("anchorIds", []):
                    if anchor_id in id2Position:
                        anchor_pos = id2Position[anchor_id]
                        if self.cov_line_idx < len(self.cov_formation_lines):
                            line = self.cov_formation_lines[self.cov_line_idx]
                            line.set_data([myPosition[0], anchor_pos[0]], [myPosition[1], anchor_pos[1]])
                            line.set_color(self.cov_robot_color)
                            line.set_visible(True)
                            self.cov_line_idx += 1

                for base_id in myJson.get("baseIds", []):
                    if "config" in self.data and "bases" in self.data["config"]:
                        bases = self.data["config"]["bases"]
                        if base_id < len(bases):
                            base_pos = bases[base_id]
                            if self.cov_line_idx < len(self.cov_formation_lines):
                                line = self.cov_formation_lines[self.cov_line_idx]
                                line.set_data([myPosition[0], base_pos[0]], [myPosition[1], base_pos[1]])
                                line.set_color(self.cov_base_color)
                                line.set_visible(True)
                                self.cov_line_idx += 1

        for i, (patch, x, y, yaw) in enumerate(zip(self.search_patches, robotX, robotY, robotYawDeg)):
            if self.searching_method == "downward":
                patch.set_center((x, y))
                patch.set_visible(self.showYaw)
            elif self.searching_method == "front-sector":
                half_angle_deg = self.searching_params["half-angle-deg"]
                patch.set_center((x, y))
                patch.set_theta1(yaw - half_angle_deg)
                patch.set_theta2(yaw + half_angle_deg)
                patch.set_visible(self.showYaw)
            elif self.searching_method == "front-cone":
                height = self.searching_params["height"]
                downward_radius = self.searching_params["downward-radius"]
                camera_pitch_deg = self.searching_params["camera-pitch-deg"]

                pitch_rad = math.radians(camera_pitch_deg)
                yaw_rad = math.radians(yaw)
                alpha = math.atan(downward_radius / height)

                d1 = height / math.tan(pitch_rad + alpha)
                d2 = height / math.tan(pitch_rad - alpha)
                semi_major = (d2 - d1) / 2
                distance_to_center = (d1 + d2) / 2
                semi_minor = height / math.sin(pitch_rad) * math.tan(alpha)

                if semi_minor > semi_major:
                    semi_major, semi_minor = semi_minor, semi_major

                ellipse_cx = x + distance_to_center * math.cos(yaw_rad)
                ellipse_cy = y + distance_to_center * math.sin(yaw_rad)

                patch.set_center((ellipse_cx, ellipse_cy))
                patch.width = semi_major * 2
                patch.height = semi_minor * 2
                patch.angle = yaw
                patch.set_visible(self.showYaw)

        for i, (id, x, y) in enumerate(zip(self.id_list, robotX, robotY)):
            if self.showCovEllipse and self.showPositionCovariance:
                if "position_covariance" in dataNow["robots"][id]:
                    cov_data = dataNow["robots"][id]["position_covariance"]
                    cov_xx = cov_data["cov_xx"]
                    cov_xy = cov_data["cov_xy"]
                    cov_yy = cov_data["cov_yy"]

                    cov_matrix = np.array([[cov_xx, cov_xy], [cov_xy, cov_yy]])
                    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

                    sort_indices = np.argsort(eigenvalues)[::-1]
                    eigenvalues = eigenvalues[sort_indices]
                    eigenvectors = eigenvectors[:, sort_indices]

                    width = 4 * np.sqrt(eigenvalues[0])
                    height_ellipse = 4 * np.sqrt(eigenvalues[1])
                    angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))

                    ellipse = self.cov_ellipses[i]
                    ellipse.set_center((x, y))
                    ellipse.width = width
                    ellipse.height = height_ellipse
                    ellipse.angle = angle
                    ellipse.set_visible(True)
                elif self.cov_ellipses[i]:
                    self.cov_ellipses[i].set_visible(False)

            if self.robotAnnotation:
                annoText = f'#{id + 1}'

                if self.showCovText and self.showPositionCovariance and "position_covariance" in dataNow["robots"][id]:
                    cov_data = dataNow["robots"][id]["position_covariance"]
                    cov_xx = cov_data["cov_xx"]
                    cov_xy = cov_data["cov_xy"]
                    cov_yy = cov_data["cov_yy"]
                    annoText += f'[{cov_xx:.1f},{cov_xy:.1f},{cov_yy:.1f}]]'

                names = ["commFixed", "commAuto"]
                for name in names:
                    if "cbfs" in dataNow and name in dataNow["cbfs"]:
                        for comm in dataNow["cbfs"][name]:
                            if comm["id"] == id + 1:
                                annoText += '->' + ', '.join([f'{aid}' for aid in comm["anchorIds"]])
                                annoText += '-->' + ', '.join([f'o' for _ in comm["anchorPoints"]])

                ann = self.annotations[i]
                ann.set_position((x, y))
                ann.set_text(annoText)
                ann.set_visible(True)

            if self.data["config"]["cbfs"]["with-slack"]["cvt"]["on"] and self.showCVT:
                if self.cvt_lines and "cvt" in dataNow["robots"][id]:
                    cvtPolygonX = [pos[0] for pos in dataNow["robots"][id]["cvt"]["pos"]]
                    cvtPolygonY = [pos[1] for pos in dataNow["robots"][id]["cvt"]["pos"]]
                    self.cvt_lines[i].set_data(cvtPolygonX, cvtPolygonY)
                    cvtCenterX = [dataNow["robots"][id]["cvt"]["center"][0]]
                    cvtCenterY = [dataNow["robots"][id]["cvt"]["center"][1]]
                    self.cvt_centers[i].set_data(cvtCenterX, cvtCenterY)
                elif self.cvt_lines:
                    self.cvt_lines[i].set_data([], [])
                    self.cvt_centers[i].set_data([], [])

        if self.showTimeTitle and self.time_text:
            if self.bigTimeText:
                self.time_text.set_text(r'$\mathrm{Time}$' + f' $=$ ${dataNow["runtime"]:.2f}$' + r'$\mathrm{s}$')
            else:
                self.time_text.set_text('Time = {:.2f}s'.format(dataNow["runtime"]))

        artists = [self.heatmap_img, self.robot_scatter]
        if self.time_text:
            artists.append(self.time_text)
        artists.extend(self.search_patches)
        if self.cov_ellipses:
            artists.extend([e for e in self.cov_ellipses if e is not None])
        artists.extend(self.annotations)
        artists.extend(self.formation_arrows[:self.formation_arrow_idx])
        artists.extend(self.cov_formation_lines[:self.cov_line_idx])
        if self.cvt_lines:
            artists.extend(self.cvt_lines)
            artists.extend(self.cvt_centers)

        return artists
