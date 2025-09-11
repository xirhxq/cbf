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
        
        self.xLimit = [-1, 1]
        self.yLimit = [-1, 1]
        self.ax.plot(self.xLimit, [0, 0], '--k')
        self.ax.plot([0, 0], self.yLimit, '--k')
        
        self.markerResult, = self.ax.plot([0, self.data[0]["result"]["vx"]], [0, self.data[0]["result"]["vy"]], "r", alpha=0.5, linewidth=2)
        self.markerNominal, = self.ax.plot([self.data[0]["nominal"]["vx"]], [self.data[0]["nominal"]["vy"]], "b*")
        self.resultText = self.ax.text(0, 0, "", color='red', fontsize=9, 
                                      bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'),
                                      horizontalalignment='center', verticalalignment='center')
        self.ax.set_xlim(self.xLimit)
        self.ax.set_ylim(self.yLimit)
        nx, ny = 100, 100
        xVec, yVec = np.linspace(self.xLimit[0], self.xLimit[1], nx), np.linspace(self.yLimit[0], self.yLimit[1], ny)
        self.xgrid, self.ygrid = np.meshgrid(xVec, yVec)

        self.keys = ("cbfNoSlack", "cbfSlack")

        self.lines = {key: {} for key in self.keys}
        self.texts = {key: {} for key in self.keys}
        text_style = dict(color='red', alpha=0.0, fontsize=9, bbox=dict(facecolor='white', alpha=0.0, edgecolor='none'))
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

        self.markerResult.set_data([0, data_now["result"]["vx"]], [0, data_now["result"]["vy"]])
        self.markerResult.set_zorder(5)
        self.markerNominal.set_data([data_now["nominal"]["vx"]], [data_now["nominal"]["vy"]])
        self.markerNominal.set_zorder(10)

        min_lim = max(1.2, max(abs(data_now["result"]["vx"]), abs(data_now["result"]["vy"])))

        self.ax.set_xlim([-min_lim, min_lim])
        self.ax.set_ylim([-min_lim, min_lim])

        self.ax.set_title(self.title)
        self.texts["result"].set_position((np.nan, np.nan))
        self.texts["result"].set_text("")

        result_vx = data_now["result"]["vx"]
        result_vy = data_now["result"]["vy"]
        result_norm = np.sqrt(result_vx**2 + result_vy**2)
        
        if result_norm > 0:
            if result_norm > 1:
                text_x = result_vx / result_norm
                text_y = result_vy / result_norm
            else:
                text_x = result_vx
                text_y = result_vy
            
            self.resultText.set_position((text_x, text_y))
            self.resultText.set_text(f"Result\n({result_vx:.2f}, {result_vy:.2f})")
        else:
            self.resultText.set_position((np.nan, np.nan))
            self.resultText.set_text("")

        for key in self.keys:
            for line in self.lines[key].values():
                line.set_data([], [])

        charging = True

        for key in self.keys:
            if key in data_now:
                for cbf in data_now[key]:
                    if cbf["coe"]["vx"]** 2 + cbf["coe"]["vy"]**2 == 0:
                        continue
                    perpendicular_x = -cbf["const"] * cbf["coe"]["vx"] / (cbf["coe"]["vx"]**2 + cbf["coe"]["vy"]**2)
                    perpendicular_y = -cbf["const"] * cbf["coe"]["vy"] / (cbf["coe"]["vx"]**2 + cbf["coe"]["vy"]**2)
                    norm = np.sqrt(perpendicular_x**2 + perpendicular_y**2)
                    is_feasible = cbf["const"] >= 0
                    if norm > 1e-10:
                        self.lines[key][cbf["name"]].set_data(
                            [0, perpendicular_x / norm],
                            [0, perpendicular_y / norm]
                        )
                    else:
                        self.lines[key][cbf["name"]].set_data([0, 0], [0, 0])
                    self.lines[key][cbf["name"]].set_label(f"{cbf['name']}\n({norm:.2f})-{'F' if is_feasible else 'I'}")
                    self.lines[key][cbf["name"]].set_zorder(15)
                    self.texts[key][cbf["name"]].set_position((np.nan, np.nan))
                    self.texts[key][cbf["name"]].set_text("")
                    charging = False

        self.ax.legend(loc='best', fontsize=8)

        if charging:
            self.txt.set_text('Charging')
        else:
            self.txt.set_text('')
