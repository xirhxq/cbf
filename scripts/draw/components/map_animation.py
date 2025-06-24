from utils import *
from .base import BaseComponent


class MapAnimationComponent(BaseComponent):
    def __init__(self, ax, data, **kwargs):
        self.data = data
        self.ax = ax
        self.interval = self.data["state"][1]["runtime"] - self.data["state"][0]["runtime"]
        self.fps = int(1 / self.interval)
        self.worldX = [data[0] for data in self.data["para"]["world"]["boundary"]]
        self.worldY = [data[1] for data in self.data["para"]["world"]["boundary"]]
        self.gridWorldJson = self.data["para"]["gridWorld"]
        self.robotAnnotation = True
        self.showYaw = False
        self.showCVT = True
        self.showAxis = False
        self.bigTimeText = True
        self.shotList = []

        self.x = np.linspace(self.gridWorldJson["xLim"][0], self.gridWorldJson["xLim"][1],
                             self.gridWorldJson["xNum"])
        self.y = np.linspace(self.gridWorldJson["yLim"][0], self.gridWorldJson["yLim"][1],
                             self.gridWorldJson["yNum"])
        self.X, self.Y = np.meshgrid(self.x, self.y)
        self.Z = np.zeros((self.gridWorldJson["xNum"], self.gridWorldJson["yNum"]))
        self.zExtent = self.gridWorldJson["xLim"] + self.gridWorldJson["yLim"]

    def update(self, num, dataNow=None):
        from matplotlib.patches import Wedge, Circle

        if dataNow is None:
            dataNow = self.data["state"][num]

        self.ax.clear()

        if "update" in dataNow and len(dataNow["update"]):
            self.Z[*zip(*dataNow["update"])] = 1

        self.ax.imshow(self.Z.T, alpha=0.2, extent=self.zExtent, origin='lower', cmap='coolwarm', vmin=0, vmax=1)

        robotNum = self.data["para"]["swarm"]["num"]
        pos_charge = self.data["para"]["world"]["charge"]["pos"]
        dist_charge = self.data["para"]["world"]["charge"]["dist"]
        [self.ax.add_patch(Circle(xy=(pos[0], pos[1]), radius=dist, alpha=0.5)) for pos, dist in zip(pos_charge, dist_charge)]

        robotX = [dataNow["robots"][i]["state"]["x"] for i in range(robotNum)]
        robotY = [dataNow["robots"][i]["state"]["y"] for i in range(robotNum)]
        robotBattery = [dataNow["robots"][i]["state"]["battery"] for i in range(robotNum)]
        robotYawDeg = [math.degrees(dataNow["robots"][i]["state"]["yawRad"]) for i in range(robotNum)]

        self.ax.scatter(robotX, robotY, c=robotBattery, cmap='RdYlGn', s=100, alpha=0.5)

        if "formation" in dataNow:
            id2Position = {robot["id"]: (robot["state"]["x"], robot["state"]["y"]) for robot in dataNow["robots"]}
            for myJson in dataNow["formation"]:
                myPosition = id2Position[myJson["id"]]
                for anchorPoint in myJson.get("anchorPoints", []):
                    self.ax.arrow(myPosition[0], myPosition[1],
                                  anchorPoint[0] - myPosition[0], anchorPoint[1] - myPosition[1],
                                  head_width=0.5, head_length=0.5, fc='k', ec='k', alpha=0.2)
                for neighbor_id in myJson.get("anchorIds", []):
                    neighbor_pos = id2Position[neighbor_id]
                    self.ax.arrow(myPosition[0], myPosition[1],
                                  neighbor_pos[0] - myPosition[0], neighbor_pos[1] - myPosition[1],
                                  head_width=0.5, head_length=0.5, fc='k', ec='k', alpha=0.2)

        for i in range(robotNum):
            if self.showYaw:
                self.ax.add_patch(Wedge(center=[robotX[i], robotY[i]], r=0.5,
                                        theta1=robotYawDeg[i] - 15, theta2=robotYawDeg[i] + 15, alpha=0.3))

            if self.robotAnnotation:
                annoText = f'#{i + 1}[{robotBattery[i]:.2f}]'
                names = ["commFixed", "commAuto"]
                for name in names:
                    if "cbfs" in dataNow and name in dataNow["cbfs"]:
                        for comm in dataNow["cbfs"][name]:
                            if comm["id"] == i + 1:
                                annoText += '->' + ', '.join([f'{id}' for id in comm["anchorIds"]])
                                annoText += '-->' + ', '.join([f'o' for p in comm["anchorPoints"]])
                self.ax.annotate(annoText, xy=(robotX[i], robotY[i]), fontsize=8)

            if self.data["config"]["cbfs"]["with-slack"]["cvt"]["on"] and self.showCVT:
                cvtPolygonX = [pos[0] for pos in dataNow["robots"][i]["cvt"]["pos"]]
                cvtPolygonY = [pos[1] for pos in dataNow["robots"][i]["cvt"]["pos"]]
                self.ax.plot(cvtPolygonX, cvtPolygonY, 'k')
                cvtCenterX = [dataNow["robots"][i]["cvt"]["center"][0]]
                cvtCenterY = [dataNow["robots"][i]["cvt"]["center"][1]]
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
