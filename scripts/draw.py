import os
import re
import glob
import json
import math
import time
import tqdm
import ffmpeg

import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.animation as animation
from matplotlib import patheffects
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle
from matplotlib.patches import Wedge
from mpl_toolkits.axes_grid1 import make_axes_locatable


def findNewestFile(ptn: str):
    directories = glob.glob(os.path.join('data', ptn))
    assert len(directories) > 0, "No directory found with pattern {}".format(ptn)
    directories = [d for d in directories if len(glob.glob(d + '/data.json')) > 0]
    assert len(directories) > 0, "No directory found with 'data.json' in it"
    newestFile = max(directories)
    return os.path.join(newestFile, 'data.json')


def name2Color(name):
    name = name.lower()
    if 'cvt' in name:
        return 'orangered'
    if 'energy' in name:
        return 'mediumblue'
    if 'safe' in name:
        return 'red'
    return 'black'


class MyBarPlot:
    def __init__(self, x, y, ax, name, color='default', markerOn=True):
        self.xData = x
        self.yData = y
        self.name = name
        ax.set_title(name)
        if color == 'default':
            color = name2Color(name)
        ax.plot(self.xData, self.yData, color=color)
        if markerOn:
            marker, = ax.plot(self.xData[:1], self.yData[:1], "r*")
            self.marker = marker
            line, = ax.plot(self.xData[:1] * 2, [self.yData[0], 0], "r")
            self.line = line

    def update(self, num):
        self.marker.set_data(self.xData[num], self.yData[num])
        self.line.set_data(self.xData[num], [self.yData[num], 0])


class MyOptPlot:
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
        now_data = self.data[0]
        if "cbfNoSlack" in now_data:
            self.cbfList += [self.xgrid * cbf["coe"]["vx"] + self.ygrid * cbf["coe"]["vy"] + cbf["const"] for cbf in
                             now_data["cbfNoSlack"]]
        if "cbfSlack" in now_data:
            self.cbfList += [self.xgrid * cbf["coe"]["vx"] + self.ygrid * cbf["coe"]["vy"] + cbf["const"] for cbf in
                             now_data["cbfSlack"]]
        self.cbf_ct = [self.ax.contour(self.xgrid, self.ygrid, cbf, [0],
                                       colors='orangered')
                       for cbf in self.cbfList]

    def update(self, _num):
        self.markerNominal.set_data([self.data[_num]["nominal"]["vx"]], [self.data[_num]["nominal"]["vy"]])
        self.markerResult.set_data([0, self.data[_num]["result"]["vx"]], [0, self.data[_num]["result"]["vy"]])
        for ct in self.cbf_ct:
            for cl in ct.collections:
                cl.remove()
        now_data = self.data[_num]
        self.cbfList = []
        self.cbfName = []
        # if "cbfNoSlack" in now_data:
        self.cbfList += [self.xgrid * cbf["coe"]["vx"] + self.ygrid * cbf["coe"]["vy"] + cbf["const"] for cbf in
                         now_data["cbfNoSlack"]]
        self.cbfName += [cbf["name"] for cbf in now_data["cbfNoSlack"]]
        # if "cbfSlack" in now_data:
        self.cbfList += [self.xgrid * cbf["coe"]["vx"] + self.ygrid * cbf["coe"]["vy"] + cbf["const"] for cbf in
                         now_data["cbfSlack"]]
        self.cbfName += [cbf["name"] for cbf in now_data["cbfSlack"]]

        if len(self.cbfList) > 0:
            self.txt.set_text('')
        else:
            self.txt.set_text('Charging')

        self.cbf_ct = [self.ax.contour(self.xgrid, self.ygrid, cbf, [0],
                                       colors=name2Color(self.cbfName[ind]))
                       for ind, cbf in enumerate(self.cbfList)]
        for ct in self.cbf_ct:
            plt.setp(ct.collections, path_effects=[patheffects.withTickedStroke(angle=60)])


class Drawer:
    DEFAULT_CONFIGS = {
        'default': {
            'plotEnergyCBF': False,
            'plotCvtCBF': False,
            'plotYawCBF': False,
            'plotSafeCBF': False,
            'plotCommCBF': False,
            'plotOpt': False,
            'robotAnnotation': True,
            'bigTimeText': False,
            'useTex': False,
            'showYaw': False,
            'showCVT': True,
            'showBar': False,
            'showAxis': False,
            'figureSize': (7, 7),
            'shotList': [],
            'barFormat': (
                    "{percentage:3.0f}%|{bar:50}| {n_fmt}/{total_fmt} [elap: {elapsed}s eta: {remaining}s]"
            ),
        },
        'paper': {
            'robotAnnotation': False,
            'bigTimeText': True,
            'figureSize': (8, 8),
            'shotList': [0, 10, 25, 80, 140, 220, 275]
        },
        'foxy': {
            'bigTimeText': True,
        }
    }

    def __init__(self, files, **kwargs):
        self.files = files
        self.folderNames = [os.path.dirname(file) for file in files]
        self.file = files[0]
        self.folderName = self.folderNames[0]
        self.datas = [json.load(open(file)) for file in files]
        self.data = self.datas[0]

        self.config = self.DEFAULT_CONFIGS['default']
        settingsOption = kwargs.get('settings', 'default')
        self.config.update(self.DEFAULT_CONFIGS[settingsOption])

        additionalConfig = kwargs.get('config', {})
        self.config.update(additionalConfig)

        for key, value in self.config.items():
            setattr(self, key, value)

        self.menu()

    @property
    def hasSingleFile(self):
        return len(self.files) == 1

    @property
    def hasMultipleFiles(self):
        return len(self.files) > 1

    @staticmethod
    def singleFile(func):
        def wrapper(self, *args, **kwargs):
            assert self.hasSingleFile, "This method is only available for single file"
            return func(self, *args, **kwargs)

        return wrapper

    @staticmethod
    def multipleFiles(func):
        def wrapper(self, *args, **kwargs):
            assert self.hasMultipleFiles, "This method is only available for multiple files"
            return func(self, *args, **kwargs)

        return wrapper

    @singleFile
    def drawAnimation(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        robotNum = self.data["para"]["swarm"]["num"]
        divider = 2
        halfNum = math.ceil(robotNum / divider)
        row, col = halfNum, 3

        fig = plt.figure(figsize=self.figureSize)
        if self.plotOpt:
            fig.set_tight_layout(True)
        gs = GridSpec(row, col)

        barPlotOn = self.plotEnergyCBF or self.plotCvtCBF
        optPlotOn = self.plotOpt

        if barPlotOn or optPlotOn:
            ax = plt.subplot(gs[:, :-divider * (int(barPlotOn) + int(optPlotOn))])
        else:
            ax = plt.subplot(gs[:, :])
        ax.set_aspect(1)

        interval = self.data["state"][1]["runtime"] - self.data["state"][0]["runtime"]
        fps = int(1 / interval)
        shotList = [ind * fps for ind in self.shotList]
        totalLength = len(self.data["state"])

        worldX = [data[0] for data in self.data["para"]["world"]["boundary"]]
        worldY = [data[1] for data in self.data["para"]["world"]["boundary"]]

        gridWorldJson = self.data["para"]["gridWorld"]

        x = np.linspace(gridWorldJson["xLim"][0], gridWorldJson["xLim"][1], gridWorldJson["xNum"])
        y = np.linspace(gridWorldJson["yLim"][0], gridWorldJson["yLim"][1], gridWorldJson["yNum"])
        X, Y = np.meshgrid(x, y)

        Z = np.zeros((gridWorldJson["xNum"], gridWorldJson["yNum"]))
        zExtent = gridWorldJson["xLim"] + gridWorldJson["yLim"]
        F = ax.imshow(Z.T, alpha=0.2, extent=zExtent, origin='lower')

        if self.showBar:
            div = make_axes_locatable(ax)
            cax = div.append_axes('right', '5%', '5%')
            cbar = plt.colorbar(F, cax=cax)

        runtime = [dt["runtime"] for dt in self.data["state"]]
        if self.plotEnergyCBF:
            energyCBFPlot = [MyBarPlot(runtime, [dt["robot"][i]["energyCBF"] for dt in self.data["state"]],
                                       plt.subplot(gs[-1 - i // halfNum, i % halfNum]),
                                       "#{}".format(i + 1), color='mediumblue')
                             for i in range(robotNum)]
        if self.plotCvtCBF:
            cvtCBFPlot = [MyBarPlot(runtime, [dt["robot"][i]["cvtCBF"] for dt in self.data["state"]],
                                    plt.subplot(gs[-1 - i // halfNum, i % halfNum]),
                                    "#{}".format(i + 1), color='orangered')
                          for i in range(robotNum)]
        if self.plotYawCBF:
            yawCBFPlot = [MyBarPlot(runtime, [dt["robot"][i]["yawCBF"] for dt in self.data["state"]],
                                    plt.subplot(gs[-2, i]),
                                    "Robot #{}: Yaw CBF Value".format(i + 1))
                          for i in range(robotNum)]

        if self.plotCommCBF:
            commCBFPlot = [MyBarPlot(runtime, [dt["robot"][i]["commTo" + str(j)] for dt in self.data["state"]],
                                     plt.subplot(gs[-2, i]),
                                     "Robot #{}: Comm CBF Value".format(i + 1))
                           for i in range(robotNum) for j in range(robotNum) if
                           "comm_to_" + str(j) in self.data["state"][0]["robot"][i]]

        if self.plotSafeCBF:
            safeCBFPlot = [MyBarPlot(runtime, [dt["robot"][i]["safe_to_" + str(j)] for dt in self.data["state"]],
                                     plt.subplot(gs[-divider, i]),
                                     "Robot #{}: Safe CBF Value".format(i + 1))
                           for i in range(robotNum) for j in range(robotNum) if
                           "safe_to_" + str(j) in self.data["state"][0]["robot"][i]]
        if self.plotOpt:
            optPlot = [MyOptPlot(plt.subplot(gs[i % halfNum, -divider + i // halfNum]),
                                 [dt["robots"][i]["opt"] for dt in self.data["state"]],
                                 "Robot #{}: Opt Result".format(i + 1))
                       for i in range(robotNum)]

        pbar = tqdm.tqdm(total=totalLength, bar_format=self.barFormat)

        def update(num):
            pbar.update(1)

            ax.clear()

            dataNow = self.data["state"][num]

            if "update" in dataNow and len(dataNow["update"]):
                Z[*zip(*dataNow["update"])] = 1

            ax.imshow(Z.T, alpha=0.2, extent=zExtent, origin='lower', cmap='coolwarm', vmin=0, vmax=1)

            pos = self.data["para"]["world"]["charge"]["pos"]
            dist = self.data["para"]["world"]["charge"]["dist"]
            [ax.add_patch(Circle(xy=(pos[i][0], pos[i][1]), radius=dist[i], alpha=0.5)) for i in range(len(pos))]

            robotX = [dataNow["robots"][i]["state"]["x"] for i in range(robotNum)]
            robotY = [dataNow["robots"][i]["state"]["y"] for i in range(robotNum)]
            robotBattery = [dataNow["robots"][i]["state"]["battery"] for i in range(robotNum)]
            robotYawDeg = [math.degrees(dataNow["robots"][i]["state"]["yawRad"]) for i in range(robotNum)]

            if self.data["config"]["cbfs"]["with-slack"]["cvt"] and self.showCVT:
                cvtPolygonX = [[pos[0] for pos in dataRobot["cvt"]["pos"]] for dataRobot in dataNow["robots"]]
                cvtPolygonY = [[pos[1] for pos in dataRobot["cvt"]["pos"]] for dataRobot in dataNow["robots"]]
                cvtPolygonCenter = [dataRobot["cvt"]["center"] for dataRobot in dataNow["robots"]]

            # ax.plot(robotX, robotY, 'b*')
            ax.scatter(
                robotX, robotY,
                c=robotBattery,
                cmap='RdYlGn',
                s=100,
                alpha=0.5
            )

            if "formation" in dataNow:
                commJson = dataNow["formation"]
                id2Position = {dataRobot["id"]: (dataRobot["state"]["x"], dataRobot["state"]["y"]) for dataRobot
                               in dataNow["robots"]}
                for myJson in commJson:
                    myPosition = id2Position[myJson["id"]]
                    for anchorPoint in myJson["anchorPoints"]:
                        # ax.plot([myPosition[0], anchorPoint[0]], [myPosition[1], anchorPoint[1]], 'k--', alpha=0.5)
                        ax.arrow(
                            myPosition[0], myPosition[1],
                            anchorPoint[0] - myPosition[0], anchorPoint[1] - myPosition[1],
                            head_width=0.5, head_length=0.5,
                            fc='k', ec='k',
                            alpha=0.2
                        )
                    for id in myJson["anchorIds"]:
                        anchorPosition = id2Position[id]
                        # ax.plot([myPosition[0], anchorPosition[0]], [myPosition[1], anchorPosition[1]], 'k--', alpha=0.5)
                        ax.arrow(
                            myPosition[0], myPosition[1],
                            anchorPosition[0] - myPosition[0], anchorPosition[1] - myPosition[1],
                            head_width=0.5, head_length=0.5,
                            fc='k', ec='k',
                            alpha=0.2
                        )

            for i in range(robotNum):
                if self.showYaw:
                    ax.add_patch(Wedge(
                        center=[robotX[i], robotY[i]],
                        r=0.5,
                        theta1=robotYawDeg[i] - 15, theta2=robotYawDeg[i] + 15,
                        alpha=0.3
                    ))

                if self.robotAnnotation:
                    # ax.annotate((f'    Robot #{i + 1}:' + '\n'
                    #              + rf'$\quadE = {robotBattery[i]:.2f}$' + '\n'
                    #              + rf'$\quad\theta = {robotYawDeg[i]:.2f}$'
                    #              ),
                    #             xy=(robotX[i], robotY[i]))
                    annoText = f'    #{i + 1}[{robotBattery[i]:.2f}]'
                    names = ["commFixed", "commAuto"]
                    for name in names:
                        if "cbfs" in dataNow and name in dataNow["cbfs"]:
                            for comm in dataNow["cbfs"][name]:
                                if comm["id"] == i + 1:
                                    annoText += '->' + ', '.join([f'{id}' for id in comm["anchorIds"]])
                                    annoText += '-->' + ', '.join([f'o' for p in comm["anchorPoints"]])
                    ax.annotate(annoText, xy=(robotX[i], robotY[i]), fontsize=8)

                if self.data["config"]["cbfs"]["with-slack"]["cvt"] and self.showCVT:
                    ax.plot(cvtPolygonX[i], cvtPolygonY[i], 'k')
                    ax.plot(
                        [ct[0] for ct in cvtPolygonCenter], [ct[1] for ct in cvtPolygonCenter],
                        '*',
                        color='lime'
                    )

            if self.bigTimeText:
                ax.set_title(
                    r'$\mathrm{Time}$' + f' $=$ ${dataNow["runtime"]:.2f}$' + r'$\mathrm{s}$',
                    fontsize=25,
                    y=0.95
                )
                # ax.text(
                #     0.38, 0.95,
                #     r'$\mathrm{Time}$' + f' $=$ ${dataNow["runtime"]:.2f}$' + r'$\mathrm{s}$',
                #     transform=ax.transAxes, fontsize=40
                # )
            else:
                ax.text(0.05, 0.95, 'Time = {:.2f}s'.format(dataNow["runtime"]), transform=ax.transAxes)
            ax.set_xlim(self.data["para"]["world"]["lim"][0])
            ax.set_ylim(self.data["para"]["world"]["lim"][1])

            ax.plot(worldX, worldY, 'k')
            if not self.showAxis:
                ax.set_axis_off()

            for i in range(robotNum):
                if self.plotEnergyCBF:
                    energyCBFPlot[i].update(num)
                if self.plotYawCBF:
                    yawCBFPlot[i].update(num)
                if self.plotOpt:
                    optPlot[i].update(num)
                if self.plotCvtCBF:
                    cvtCBFPlot[i].update(num)
            if self.plotSafeCBF:
                for cbf in safeCBFPlot:
                    cbf.update(num)
            if num in shotList:
                plt.savefig(os.path.join(self.folderName, f'-{int(num / fps)}-second.png'), bbox_inches='tight')
                print('Shot!', end='')
            return

        ani = animation.FuncAnimation(fig, update, totalLength, interval=int(1000 * interval), blit=False)

        outputFilename = 'animation-opt' if self.plotOpt else 'animation'
        filename = os.path.join(self.folderName, outputFilename + '.mp4')
        ani.save(filename, writer='ffmpeg', fps=int(1 / interval))
        print("\nmp4 saved in {}".format(filename))

        pbar.close()

    @singleFile
    def drawStatistics(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 4))

        robotNum = self.data["para"]["swarm"]["num"]

        runtime = [dt["runtime"] for dt in self.data["state"]]

        pb = tqdm.tqdm(total=robotNum, bar_format=self.barFormat)

        for i in range(robotNum):
            plt.subplot(211).clear()
            plt.subplot(212).clear()
            plt.subplot(211).plot(
                runtime, [dt["robots"][i]["state"]["battery"] for dt in self.data["state"]],
                color='C0'
            )
            plt.subplot(211).set_title('Energy Level' + f' of UAV #{i + 1}')
            plt.subplot(211).set_xlabel('Time / s')
            plt.subplot(211).set_ylabel('Energy Level')

            plt.subplot(212).plot(
                runtime, [max(dt["robots"][i]["cbfNoSlack"]["energyCBF"], 0) for dt in self.data["state"]],
                color='C0'
            )
            plt.subplot(212).set_title(r'CBF Value $min(h_{energy}, h_{l10n})$' + f' of UAV #{i + 1}')
            plt.subplot(212).set_xlabel('Time / s')
            plt.subplot(212).set_ylabel('$min(h_{energy}, h_{l10n})$')
            plt.subplots_adjust(hspace=0.7)
            # leg = ax.legend()
            figureFilename = self.folderName + f'/{i + 1}-Energy&CBFValue.png'
            plt.savefig(figureFilename, bbox_inches='tight')
            pb.update(i + 1)

        pb.close()

    @singleFile
    def drawCBFs(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 4))

        robotNum = self.data["para"]["swarm"]["num"]

        runtime = [dt["runtime"] for dt in self.data["state"]]

        pb = tqdm.tqdm(total=robotNum, bar_format=self.barFormat)

        id2Names = {}
        keys = ["cbfNoSlack", "cbfSlack"]
        for r in self.data["state"][0]["robots"]:
            id2Names[r["id"]] = {k: [p for p in r[k]] for k in keys}

        values = {}
        for frame in self.data["state"]:
            for r in frame["robots"]:
                if r["id"] not in values:
                    values[r["id"]] = {k: {} for k in keys}
                for k in keys:
                    for cbf in r[k]:
                        if cbf not in values[r["id"]][k]:
                            values[r["id"]][k][cbf] = []
                        values[r["id"]][k][cbf].append({"value": r[k][cbf], "time": frame["runtime"]})

        def getJsonStructure(data, level=0, max=2):
            structure = {}
            if level > max:
                return structure

            if isinstance(data, dict):
                for key, value in data.items():
                    structure[key] = getJsonStructure(value, level + 1, max)
            elif isinstance(data, list):
                structure["list"] = getJsonStructure(data[0], level + 1, max) if data else "empty list"
            else:
                structure = type(data).__name__

            return structure

        for id in values:
            for ind, key in enumerate(keys):
                figID = 211 + ind
                plt.subplot(figID).clear()
                for cbf in values[id][key]:
                    plt.subplot(figID).plot(
                        [dt["time"] for dt in values[id][key][cbf]],
                        [dt["value"] for dt in values[id][key][cbf]],
                        label=cbf
                    )
                plt.subplot(figID).set_xlabel('Time / s')
                plt.subplot(figID).set_ylabel(key)
                plt.subplot(figID).legend()
            filename = f'{id}-CBFs.png'
            plt.savefig(os.path.join(self.folderName, filename))

        pb.close()

    @singleFile
    def drawAllEnergy(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 4))

        robotNum = self.data["para"]["swarm"]["num"]

        runtime = [dt["runtime"] for dt in self.data["state"]]

        for i in range(robotNum):
            plt.subplot(111).plot(runtime, [dt["robots"][i]["state"]["battery"] for dt in self.data["state"]],
                                  label=f'UAV #{i + 1}')

        leg = plt.subplot(111).legend(bbox_to_anchor=(1.0, -0.15), ncol=5)
        plt.subplot(111).set_xlabel('Time / s')
        plt.subplot(111).set_ylabel('Energy Level')
        plt.subplot(111).set_title('Energy Level' + f' of all UAVs')
        figureFilename = self.folderName + f'/energyAll.png'
        plt.savefig(figureFilename, bbox_inches='tight')

    @multipleFiles
    def drawEnergy2(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 3))

        for i, data in enumerate(self.datas):
            runtime = [dt["runtime"] for dt in data["state"]]
            plt.subplot(111).plot(runtime, [dt["robot"][1]["yawRad"] * 0.005 + 3.7 for dt in data["state"]],
                                  label=f'{self.folderNames[i]}')

        leg = plt.subplot(111).legend(loc='best')
        plt.subplot(111).set_xlabel('Time / s')
        plt.subplot(111).set_ylabel('Energy Level / V')
        plt.subplot(111).set_title('Energy Level of UAV #2 in different approaches')
        plt.axhspan(3.7, 4.2, facecolor='gray', alpha=0.5)

        figureFilename = self.folderName + f'/energy2Contrast.png'
        plt.savefig(figureFilename, bbox_inches='tight')

    @multipleFiles
    def drawDistance1To2ForAll(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 3))

        for i, data in enumerate(self.datas):
            runtime = [dt["runtime"] for dt in data["state"]]
            distance = [np.sqrt((dt["robot"][0]["x"] - dt["robot"][1]["x"]) ** 2 +
                                (dt["robot"][0]["y"] - dt["robot"][1]["y"]) ** 2) * 85
                        for dt in data["state"]]
            plt.subplot(111).plot(runtime, distance,
                                  label=f'{self.folderNames[i]}')

        legend = plt.subplot(111).legend(loc='best')
        plt.subplot(111).set_xlabel('Time / s')
        plt.subplot(111).set_ylabel('Distance / m')
        plt.subplot(111).set_title('Distance from UAV #2 to UAV #1 in different approaches')

        plt.axhspan(0, 850, facecolor='gray', alpha=0.5)

        figureFilename = self.folderName + f'/distance_1_2_all_contrast.png'
        plt.savefig(figureFilename, bbox_inches='tight')

    @singleFile
    def drawAllCvtCBF(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        robotNum = self.data["para"]["swarm"]["num"]
        halfNum = math.ceil(robotNum / 2)
        row, col = halfNum, 2

        fig = plt.figure(figsize=(15, 20))
        gs = GridSpec(row, col)

        runtime = [dt["runtime"] for dt in self.data["state"]]

        cvt_cbf_plot = [MyBarPlot(runtime, [dt["robots"][i]["cbfSlack"]["cvtCBF"] for dt in self.data["state"]],
                                  plt.subplot(gs[i % halfNum, i // halfNum]),
                                  "#{}".format(i + 1), color='orangered', markerOn=False)
                        for i in range(robotNum)]
        figureFilename = self.folderName + f'/all_cvt_cbf.png'
        plt.savefig(figureFilename, bbox_inches='tight')

    @singleFile
    def drawHeatmap(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 8))

        robotNum = self.data["para"]["swarm"]["num"]

        runtime = [dt["runtime"] for dt in self.data["state"]]

        pb = tqdm.tqdm(total=robotNum, bar_format=self.barFormat)

        for i in range(robotNum):
            plt.subplot(111).clear()
            plt.subplot(111).set_aspect(1)
            plt.subplot(111).set_xlabel('x')
            plt.subplot(111).set_ylabel('y')
            plt.subplot(111).set_title('Heatmap of' + f' UAV {i + 1}')

            robotX = [dt["robots"][i]["state"]["x"] for dt in self.data["state"]]
            robotY = [dt["robots"][i]["state"]["y"] for dt in self.data["state"]]

            heatmap, xedges, yedges = np.histogram2d(robotX, robotY, bins=20, range=[[-10, 10], [0, 20]])

            extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
            heatmap = heatmap.T

            plt.clf()
            plt.imshow(heatmap, extent=extent, origin='lower', cmap='jet', aspect='auto')
            # plt.colorbar()

            plt.ylim([0, 20])
            plt.xlim([-10, 10])
            yticks = np.arange(yedges[0], yedges[-1] + 1, 5)
            plt.yticks(yticks)
            # plt.title('Heat-map' + f' of UAV #{i + 1}')
            # axis off
            plt.axis('off')

            figureFilename = self.folderName + f'/{i + 1}_heatmap.png'
            plt.savefig(figureFilename, bbox_inches='tight')
            pb.update(i + 1)

        pb.close()

    @singleFile
    def drawSearchHeatmap(self):
        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 8))

        xnum, ynum = self.data["para"]["gridWorld"]["xNum"], self.data["para"]["gridWorld"]["yNum"]
        Z = np.zeros((ynum, xnum))

        all_rows = []
        all_cols = []
        all_runtimes = []

        data_state = self.data["state"][::-1]

        for data_now in data_state:
            if "update" in data_now and len(data_now["update"]):
                updates = data_now["update"]
                all_rows.extend([grid[1] for grid in updates])
                all_cols.extend([grid[0] for grid in updates])
                all_runtimes.extend([data_now["runtime"]] * len(updates))

        rows = np.array(all_rows)
        cols = np.array(all_cols)
        runtimes = np.array(all_runtimes)

        Z[rows, cols] = runtimes

        plt.subplot(111).clear()
        plt.subplot(111).set_aspect(1)
        plt.subplot(111).set_xlabel('x/m')
        plt.subplot(111).set_ylabel('y/m')
        plt.subplot(111).set_title('Search Heatmap')

        Z = np.flipud(Z)

        plt.imshow(Z, cmap='jet', aspect='auto', extent=[-10, 10, 0, 20])
        plt.colorbar().set_label('Time / s')

        def scale_x(value, _):
            return f'{int(value * 100)}'

        def scale_y(value, _):
            return f'{int(value * 100)}'

        plt.gca().xaxis.set_major_formatter(ticker.FuncFormatter(scale_x))
        plt.gca().yaxis.set_major_formatter(ticker.FuncFormatter(scale_y))

        figureFilename = self.folderName + f'/SearchHeatmap.png'
        plt.savefig(figureFilename, bbox_inches='tight')

    @singleFile
    def makeScreenshotFromVideo(self):
        timepoints = [0, 10, 25, 80, 140, 220, 275]

        input_file = self.folderName + 'res.mp4'

        for t in timepoints:
            output_file = self.folderName + f'res_at_sec_{t}_new.png'

            (
                ffmpeg
                .input(input_file, ss=t)
                .filter('scale', '-1', '720')
                .output(output_file, vframes=1)
                .overwrite_output()
                .run()
            )

            if os.path.isfile(output_file):
                print(f'导出成功：{output_file}')
            else:
                print(f'导出失败：{output_file}')

    @singleFile
    def drawFixedCommDistance(self):
        # check if this run has turned 'fixcomm' on
        if self.data["config"]["cbfs"]["without-slack"]["comm-fixed"] is False:
            print("This run has not turned 'comm-fixed' on, please check the config file")
            return


        if self.useTex:
            matplotlib.rc('text', usetex=True)

        matplotlib.use('agg')

        plt.figure(figsize=(8, 4))

        robotNum = self.data["para"]["swarm"]["num"]
        idList = [i + 1 for i in range(robotNum)]

        runtime = [dt["runtime"] for dt in self.data["state"]]

        dists = {i: {} for i in idList}

        pb = tqdm.tqdm(total=len(self.data["state"]), bar_format=self.barFormat)

        for id, dataNow in enumerate(self.data["state"]):
            id2Pos = {robot["id"]: (robot["state"]["x"], robot["state"]["y"]) for robot in dataNow["robots"]}
            for myFormation in dataNow["formation"]:
                myId = myFormation["id"]
                myPos = id2Pos[myId]
                for neighbour in myFormation["anchorIds"]:
                    neighbourPos = id2Pos[neighbour]
                    dist = np.sqrt((myPos[0] - neighbourPos[0]) ** 2 + (myPos[1] - neighbourPos[1]) ** 2)
                    anchorText = f'To UAV #{neighbour}'
                    if anchorText not in dists[myId]:
                        dists[myId][anchorText] = []
                    dists[myId][anchorText].append({"dist": dist, "time": dataNow["runtime"]})
                for anchor in myFormation["anchorPoints"]:
                    anchorPos = (anchor[0], anchor[1])
                    dist = np.sqrt((myPos[0] - anchorPos[0]) ** 2 + (myPos[1] - anchorPos[1]) ** 2)
                    anchorText = f'To ({anchor[0]:.1f}, {anchor[1]:.1f})'
                    if anchorText not in dists[myId]:
                        dists[myId][anchorText] = []
                    dists[myId][anchorText].append({"dist": dist, "time": dataNow["runtime"]})
            pb.update(1)

        pb.close()

        pb = tqdm.tqdm(total=robotNum, bar_format=self.barFormat)

        for i in idList:
            plt.subplot(111).clear()
            for anchor in dists[i]:
                plt.subplot(111).plot(
                    [dt["time"] for dt in dists[i][anchor]],
                    [dt["dist"] for dt in dists[i][anchor]],
                    label=anchor
                )
            plt.subplot(111).set_title('Distance to neighbours, ' + f'UAV #{i}')
            plt.subplot(111).set_xlabel('Time / s')
            plt.subplot(111).set_ylabel('Distance / m')

            leg = plt.subplot(111).legend(loc='best')
            figureFilename = self.folderName + f'/{i}-FixedCommDistance.png'
            plt.savefig(figureFilename, bbox_inches='tight')
            pb.update(1)

        pb.close()

    def menu(self):
        while True:
            if self.hasSingleFile:
                print('-' * 10 + 'Choose which drawing you want:' + '-' * 10)
                print('[0]: Quit')
                print('[1]: Draw whole map video')
                print('[2]: Draw whole map video with optimisation')
                print('[3]: Draw stats')
                print('[4]: Draw all energy')
                print('[5]: Draw cbf values')
                print('[6]: Draw heat-maps')
                print('[7]: Draw all cvt cbf')
                print('[8]: Screenshots from video')
                print('[9]: Draw search heat-map')
                print('[10]: Draw fixed comm distance')
                op = int(input('Input the number: '))
                if op == 0:
                    break
                elif op == 1:
                    self.shotList = []
                    self.drawAnimation()
                elif op == 2:
                    self.plotOpt = True
                    self.figureSize = (16, 9)
                    self.drawAnimation()
                else:
                    if op == 3:
                        self.drawStatistics()
                    elif op == 4:
                        self.drawAllEnergy()
                    elif op == 5:
                        self.drawCBFs()
                    elif op == 6:
                        self.drawHeatmap()
                    elif op == 7:
                        self.drawAllCvtCBF()
                    elif op == 8:
                        self.makeScreenshotFromVideo()
                    elif op == 9:
                        self.drawSearchHeatmap()
                    elif op == 10:
                        self.drawFixedCommDistance()
            else:
                print('-' * 10 + 'Choose which drawing you want:' + '-' * 10)
                print('[0]: Quit')
                print('[1]: Draw energys')
                print('[2]: Draw distance between 1 and 2')
                op = int(input('Input the number: '))
                if op == 0:
                    break
                elif op == 1:
                    self.drawEnergy2()
                elif op == 2:
                    self.drawDistance1To2ForAll()


if __name__ == '__main__':
    filenames = [findNewestFile('*')]
    print(f'{filenames = }')
    drawer = Drawer(
        filenames,
        settings='foxy',
        config={}
    )
