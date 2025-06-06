from utils import *

from .base import PlotComponent

class OptimizationContourPlot(PlotComponent):
    def __init__(self, ax, data, name):
        self.data = data
        self.name = name
        self.ax = ax
        self.ax.set_title(self.name)
        self.txt = self.ax.text(0.05, 0.85, '', color='red', transform=self.ax.transAxes, fontsize=20)
        self.markerNominal, = self.ax.plot([self.data[0]["nominal"]["vx"]], [self.data[0]["nominal"]["vy"]], "b*")
        self.markerResult, = self.ax.plot([0, self.data[0]["result"]["vx"]], [0, self.data[0]["result"]["vy"]], "r")
        self.xLimit = [-5, 5]
        self.yLimit = [-5, 5]
        self.ax.plot(self.xLimit, [0, 0], '--k')
        self.ax.plot([0, 0], self.yLimit, '--k')
        self.ax.set_xlim(self.xLimit)
        self.ax.set_ylim(self.yLimit)
        nx, ny = 100, 100
        xVec, yVec = np.linspace(self.xLimit[0], self.xLimit[1], nx), np.linspace(self.yLimit[0], self.yLimit[1], ny)
        self.xgrid, self.ygrid = np.meshgrid(xVec, yVec)
        self.cbfList = []
        self.cbfName = []

    def setup(self, fig, gs, config=None):
        pass

    def update(self, num, dataNow=None):
        if dataNow is None:
            dataNow = self.data[num]

        self.markerNominal.set_data([dataNow["nominal"]["vx"]], [dataNow["nominal"]["vy"]])
        self.markerResult.set_data([0, dataNow["result"]["vx"]], [0, dataNow["result"]["vy"]])

        for ct in getattr(self, 'cbf_ct', []):
            for cl in ct.collections:
                cl.remove()

        self.cbfList = []
        self.cbfName = []

        now_data = dataNow
        if "cbfNoSlack" in now_data:
            self.cbfList += [self.xgrid * cbf["coe"]["vx"] + self.ygrid * cbf["coe"]["vy"] + cbf["const"] for cbf in now_data["cbfNoSlack"]]
            self.cbfName += [cbf["name"] for cbf in now_data["cbfNoSlack"]]
        if "cbfSlack" in now_data:
            self.cbfList += [self.xgrid * cbf["coe"]["vx"] + self.ygrid * cbf["coe"]["vy"] + cbf["const"] for cbf in now_data["cbfSlack"]]
            self.cbfName += [cbf["name"] for cbf in now_data["cbfSlack"]]

        if len(self.cbfList) > 0:
            self.txt.set_text('')
        else:
            self.txt.set_text('Charging')

        self.cbf_ct = [self.ax.contour(self.xgrid, self.ygrid, cbf, [0], colors=name2Color(self.cbfName[ind])) for ind, cbf in enumerate(self.cbfList)]
        for ct in self.cbf_ct:
            plt.setp(ct.collections, path_effects=[patheffects.withTickedStroke(angle=60)])
