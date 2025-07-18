from utils import *

from .base import BaseComponent

class OptimizationVectorComponent(BaseComponent):
    def __init__(self, ax, data, id_list, title, **kwargs):
        assert len(id_list) == 1, \
            f"OptimizationVectorComponent requires exactly one robot ID, but received {id_list}"
        robot_id = id_list[0]
        self.data = [dt["robots"][robot_id]["opt"] for dt in data["state"]]
        self.title = (title or f"Opt Result") + f", Robot #{robot_id + 1}"
        self.ax = ax
        self.ax.set_title(self.title)
        self.txt = self.ax.text(0.05, 0.85, '', color='red', transform=self.ax.transAxes, fontsize=20)
        self.markerNominal, = self.ax.plot([self.data[0]["nominal"]["vx"]], [self.data[0]["nominal"]["vy"]], "b*")
        self.markerResult, = self.ax.plot([0, self.data[0]["result"]["vx"]], [0, self.data[0]["result"]["vy"]], "r")
        self.xLimit = [-1, 1]
        self.yLimit = [-1, 1]
        self.ax.plot(self.xLimit, [0, 0], '--k')
        self.ax.plot([0, 0], self.yLimit, '--k')
        self.ax.set_xlim(self.xLimit)
        self.ax.set_ylim(self.yLimit)
        nx, ny = 100, 100
        xVec, yVec = np.linspace(self.xLimit[0], self.xLimit[1], nx), np.linspace(self.yLimit[0], self.yLimit[1], ny)
        self.xgrid, self.ygrid = np.meshgrid(xVec, yVec)

        self.keys = ("cbfNoSlack", "cbfSlack")

        self.lines = {key: {} for key in self.keys}
        self.texts = {key: {} for key in self.keys}
        text_style = dict(color='red', alpha=0.8, fontsize=9, bbox=dict(facecolor='white', alpha=0.3, edgecolor='none'))
        self.texts["result"] = self.ax.text(np.nan, np.nan, "", **text_style, horizontalalignment='center', verticalalignment='center')

        for frame in self.data:
            for key in self.keys:
                if key in frame:
                    for cbf in frame[key]:
                        if cbf["name"] not in self.lines[key]:
                            self.lines[key][cbf["name"]] = self.ax.plot([], [], label=cbf["name"])[0]
                            self.texts[key][cbf["name"]] = self.ax.text(np.nan, np.nan, "", **text_style, horizontalalignment='center', verticalalignment='center')

    def update(self, num, data_now=None):
        if data_now is None:
            data_now = self.data[num]

        self.markerNominal.set_data([data_now["nominal"]["vx"]], [data_now["nominal"]["vy"]])
        self.markerNominal.set_zorder(10)
        self.markerResult.set_data([0, data_now["result"]["vx"]], [0, data_now["result"]["vy"]])
        self.markerResult.set_zorder(10)

        min_lim = max(1.2, max(abs(data_now["result"]["vx"]), abs(data_now["result"]["vy"])))

        self.ax.set_xlim([-min_lim, min_lim])
        self.ax.set_ylim([-min_lim, min_lim])

        self.ax.set_title(self.title)
        self.texts["result"].set_position((data_now["result"]["vx"], data_now["result"]["vy"]))
        self.texts["result"].set_text(f"Result\n({data_now['result']['vx']:.2f}, {data_now['result']['vy']:.2f})")

        for key in self.keys:
            for line in self.lines[key].values():
                line.set_data([], [])

        charging = True

        for key in self.keys:
            if key in data_now:
                for cbf in data_now[key]:
                    perpendicular_x = -cbf["const"] * cbf["coe"]["vx"] / (cbf["coe"]["vx"]**2 + cbf["coe"]["vy"]**2)
                    perpendicular_y = -cbf["const"] * cbf["coe"]["vy"] / (cbf["coe"]["vx"]**2 + cbf["coe"]["vy"]**2)
                    norm = np.sqrt(perpendicular_x**2 + perpendicular_y**2)
                    is_feasible = cbf["const"] >= 0
                    self.lines[key][cbf["name"]].set_data(
                        [0, perpendicular_x / norm],
                        [0, perpendicular_y / norm]
                    )
                    self.texts[key][cbf["name"]].set_position(
                        (perpendicular_x / norm, perpendicular_y / norm)
                    )
                    self.texts[key][cbf["name"]].set_text(f"{cbf['name']}\n({norm:.2f})-{'F' if is_feasible else 'I'}")
                    charging = False

        if charging:
            self.txt.set_text('Charging')
        else:
            self.txt.set_text('')
