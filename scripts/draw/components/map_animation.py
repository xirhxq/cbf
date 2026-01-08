from utils import *
from .base import BaseComponent
import matplotlib.colors as mcolors
import matplotlib.cm as mcm


class MapAnimationComponent(BaseComponent):
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
        self.bigTimeText = True
        self.shotList = []
        self.showCovarianceFormation = kwargs.get('show_covariance_formation', True)

        params = kwargs.get('params', {})
        self.colormap = params.get('colormap', 'coolwarm')

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

        self.zUpdatedIndex = 0

        world_x_size = self.data["para"]["world"]["lim"][0][1] - self.data["para"]["world"]["lim"][0][0]
        world_y_size = self.data["para"]["world"]["lim"][1][1] - self.data["para"]["world"]["lim"][1][0]
        world_size = max(world_x_size, world_y_size)
        self.wedge_radius = world_size * 0.05

    def updateZ(self, num, dataNow=None):
        if dataNow is None:
            dataNow = self.data["state"][num]

        if "update" in dataNow and len(dataNow["update"]):
            self.Z[*zip(*dataNow["update"])] = 1

    def update(self, num, dataNow=None):
        from matplotlib.patches import Wedge, Circle

        if dataNow is None:
            dataNow = self.data["state"][num]

        self.ax.clear()

        while self.zUpdatedIndex < num:
            self.updateZ(self.zUpdatedIndex)
            self.zUpdatedIndex += 1

        self.ax.imshow(self.Z.T, alpha=0.3, extent=self.zExtent, origin='lower', cmap=self.search_cmap, vmin=0, vmax=1)

        robotNum = self.data["para"]["swarm"]["num"]
        pos_charge = self.data["para"]["world"]["charge"]["pos"]
        dist_charge = self.data["para"]["world"]["charge"]["dist"]
        [self.ax.add_patch(Circle(xy=(pos[0], pos[1]), radius=dist, alpha=0.5)) for pos, dist in zip(pos_charge, dist_charge)]

        robotX = [dataNow["robots"][i]["state"]["x"] for i in self.id_list]
        robotY = [dataNow["robots"][i]["state"]["y"] for i in self.id_list]
        robotBattery = [dataNow["robots"][i]["state"]["battery"] for i in self.id_list]
        robotYawDeg = [math.degrees(dataNow["robots"][i]["state"]["yawRad"]) for i in self.id_list]

        self.ax.scatter(robotX, robotY, c=robotBattery, cmap='RdYlGn', s=100, alpha=0.5)

        if "formation" in dataNow and dataNow["formation"] != [None]:
            id2Position = {robot["id"]: (robot["state"]["x"], robot["state"]["y"]) for robot in dataNow["robots"]}
            for myJson in dataNow["formation"]:
                if len(myJson) == 0 or not myJson:
                    continue
                if myJson["id"] - 1 not in self.id_list:
                    continue
                myPosition = id2Position[myJson["id"]]
                for anchorPoint in myJson.get("anchorPoints", []):
                    self.ax.arrow(myPosition[0], myPosition[1],
                                  anchorPoint[0] - myPosition[0], anchorPoint[1] - myPosition[1],
                                  head_width=0.5, head_length=0.5, fc='gray', ec='gray', alpha=0.3)
                for neighbor_id in myJson.get("anchorIds", []):
                    neighbor_pos = id2Position[neighbor_id]
                    self.ax.arrow(myPosition[0], myPosition[1],
                                  neighbor_pos[0] - myPosition[0], neighbor_pos[1] - myPosition[1],
                                  head_width=0.5, head_length=0.5, fc='gray', ec='gray', alpha=0.3)

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
                        self.ax.plot([myPosition[0], anchor_pos[0]], [myPosition[1], anchor_pos[1]],
                                   color=self.cov_robot_color, linestyle='-', alpha=0.4, linewidth=1.5, label='Covariance Robot' if myJson["id"] == 1 else "")

                for base_id in myJson.get("baseIds", []):
                    if "config" in self.data and "cbfs" in self.data["config"] and "without-slack" in self.data["config"]["cbfs"] and "comm-fixed" in self.data["config"]["cbfs"]["without-slack"] and "bases" in self.data["config"]["cbfs"]["without-slack"]["comm-fixed"]:
                        bases = self.data["config"]["cbfs"]["without-slack"]["comm-fixed"]["bases"]
                        if base_id < len(bases):
                            base_pos = bases[base_id]
                            self.ax.plot([myPosition[0], base_pos[0]], [myPosition[1], base_pos[1]],
                                       color=self.cov_base_color, linestyle='-', alpha=0.4, linewidth=1.5, label='Covariance Base' if myJson["id"] == 1 else "")

        for i, id in enumerate(self.id_list):
            if self.showYaw:
                self.ax.add_patch(Wedge(center=[robotX[i], robotY[i]], r=self.wedge_radius,
                                        theta1=robotYawDeg[i] - 15, theta2=robotYawDeg[i] + 15, alpha=0.3))

            if (self.showPositionCovariance and
                "position_covariance" in dataNow["robots"][id]):
                from matplotlib.patches import Ellipse
                import numpy as np

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
                height = 4 * np.sqrt(eigenvalues[1])
                angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))

                ellipse = Ellipse(
                    xy=(robotX[i], robotY[i]),
                    width=width,
                    height=height,
                    angle=angle,
                    facecolor='red',
                    alpha=0.2,
                    edgecolor='red',
                    linewidth=1
                )
                self.ax.add_patch(ellipse)

            if self.robotAnnotation:
                annoText = f'#{id + 1}'

                if (self.showPositionCovariance and
                    "position_covariance" in dataNow["robots"][id]):
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
                                annoText += '->' + ', '.join([f'{id}' for id in comm["anchorIds"]])
                                annoText += '-->' + ', '.join([f'o' for p in comm["anchorPoints"]])
                self.ax.annotate(annoText, xy=(robotX[i], robotY[i]), fontsize=8)

            if self.data["config"]["cbfs"]["with-slack"]["cvt"]["on"] and self.showCVT:
                cvtPolygonX = [pos[0] for pos in dataNow["robots"][id]["cvt"]["pos"]]
                cvtPolygonY = [pos[1] for pos in dataNow["robots"][id]["cvt"]["pos"]]
                self.ax.plot(cvtPolygonX, cvtPolygonY, 'k')
                cvtCenterX = [dataNow["robots"][id]["cvt"]["center"][0]]
                cvtCenterY = [dataNow["robots"][id]["cvt"]["center"][1]]
                self.ax.plot(cvtCenterX, cvtCenterY, '*', color='lime')

            if self.bigTimeText:
                self.ax.set_title(
                    r'$\mathrm{Time}$' + f' $=$ ${dataNow["runtime"]:.2f}$' + r'$\mathrm{s}$',
                    fontsize=25,
                    y=0.95
                )
            else:
                self.ax.text(0.05, 0.95, 'Time = {:.2f}s'.format(dataNow["runtime"]), transform=self.ax.transAxes)
        self.ax.set_xlim(self.data["para"]["world"]["lim"][0])
        self.ax.set_ylim(self.data["para"]["world"]["lim"][1])
        self.ax.plot(self.worldX, self.worldY, 'k')
        if not getattr(self, 'showAxis', True):
            self.ax.set_axis_off()
