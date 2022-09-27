import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
from matplotlib.patches import Wedge
from matplotlib.gridspec import GridSpec
import matplotlib
from matplotlib import patheffects
import os
import re
import math
import numpy as np
import time
from tqdm import tqdm, trange
from mpl_toolkits.axes_grid1 import make_axes_locatable

tic = time.time()
ptn = re.compile('.*.json')
src = 'data'
files = os.listdir(src)

# matplotlib.rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
# matplotlib.rc('text', usetex=True)

json_files = []
for file in files:
    if re.match(ptn, file):
        json_files.append(f'{src}/{file}')

# json_files.sort(key=lambda fp: os.path.getctime(fp), reverse=True)
json_files.sort(reverse=True)
newest_json = json_files[0]
print('find {}'.format(newest_json))

filename = newest_json
filename = filename[:-9]

with open(filename + 'data.json') as f:
    data_dict = json.load(f)

matplotlib.use('agg')

fig = plt.figure(figsize=(25, 15))
gs = GridSpec(4, 6)
ax = plt.subplot(gs[:-2, :])
# ax = fig.add_subplot(121)
# fig, ax = plt.subplots(figsize=(20, 10))
ax.set_aspect(1)
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

div = make_axes_locatable(ax)
cax = div.append_axes('right', '5%', '5%')

interval = data_dict["state"][1]["runtime"] - data_dict["state"][0]["runtime"]
total_length = len(data_dict["state"])

robot_num = data_dict["para"]["number"]
world_x_list = [data["x"] for data in data_dict["para"]["world"]]
world_y_list = [data["y"] for data in data_dict["para"]["world"]]

xnum, ynum = 200, 100
x = np.linspace(data_dict["para"]["lim"]["x"][0], data_dict["para"]["lim"]["x"][1], xnum)
y = np.linspace(data_dict["para"]["lim"]["y"][0], data_dict["para"]["lim"]["y"][1], ynum)
X, Y = np.meshgrid(x, y)


def cal_dens(data):
    Z = 1e-8 * X / X
    if "target" not in data:
        return Z
    for tar in data["target"]:
        center_x, center_y = tar["x"], tar["y"]
        L = np.sqrt((X - center_x) ** 2 + (Y - center_y) ** 2)
        # Z += np.exp(-np.power(np.fabs(L - tar["r"]), 3) * tar["k"]) * tar["k"]
        # Z += np.exp(-np.fabs(L - tar["r"]) + tar["k"])
        Z += np.exp((
                            np.power(
                                -np.fabs(L - tar["r"])
                                , 3)
                            + 2) * 10)
        ax.add_patch(Circle(xy=(center_x, center_y), radius=0.1, color='r', alpha=0.2))
        ax.annotate('Target', xy=(center_x, center_y))
    return Z


Z = cal_dens(data_dict["state"][0])
F = ax.imshow(Z, alpha=0.2, extent=(data_dict["para"]["lim"]["x"][0],
                                    data_dict["para"]["lim"]["x"][1],
                                    data_dict["para"]["lim"]["y"][0],
                                    data_dict["para"]["lim"]["y"][1]), origin='lower')
cbar = plt.colorbar(F, cax=cax)


class MyBarPlot:
    def __init__(self, _x, _y, _ax, _name):
        self.x_data = _x
        self.y_data = _y
        self.name = _name
        _ax.set_title(_name)
        _ax.plot(self.x_data, self.y_data)
        marker, = _ax.plot(self.x_data[:1], self.y_data[:1], "r*")
        self.marker = marker
        line, = _ax.plot(self.x_data[:1] * 2, [self.y_data[0], 0], "r")
        self.line = line

    def update(self, _num):
        self.marker.set_data(self.x_data[_num], self.y_data[_num])
        self.line.set_data(self.x_data[_num], [self.y_data[_num], 0])


runtime_list = [dt["runtime"] for dt in data_dict["state"]]
energy_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["energy_cbf"] for dt in data_dict["state"]],
                             plt.subplot(gs[-2, i]),
                             "Robot #{}: Energy CBF Value".format(i + 1))
                   for i in range(robot_num)]
camera_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["camera_cbf"] for dt in data_dict["state"]],
                             plt.subplot(gs[-2, i]),
                             "Robot #{}: Camera CBF Value".format(i + 1))
                   for i in range(robot_num)]


class MyOptPlot:
    def __init__(self, _ax, _data, _name):
        self.data = _data
        self.name = _name
        self.ax = _ax
        self.ax.set_title(self.name)
        self.txt = self.ax.text(0.05, 0.85, '', color='red', transform=self.ax.transAxes, fontsize=20)
        marker_nom, = self.ax.plot([self.data[0]["nominal"]["x"]], [self.data[0]["nominal"]["y"]], "b*")
        self.marker_nom = marker_nom
        marker_res, = self.ax.plot([0, self.data[0]["result"]["x"]], [0, self.data[0]["result"]["y"]], "r")
        self.marker_res = marker_res
        self.xlim = [-5, 5]
        self.ylim = [-5, 5]
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        nx, ny = 100, 100
        xvec, yvec = np.linspace(self.xlim[0], self.xlim[1], nx), np.linspace(self.ylim[0], self.ylim[1], ny)
        self.xgrid, self.ygrid = np.meshgrid(xvec, yvec)
        self.cbf_l = []
        now_data = self.data[0]
        if "cbf_no_slack" in now_data:
            self.cbf_l += [self.xgrid * cbf["x"] + self.ygrid * cbf["y"] + cbf["const"] for cbf in
                           now_data["cbf_no_slack"]]
        if "cbf_slack" in now_data:
            self.cbf_l += [self.xgrid * cbf["x"] + self.ygrid * cbf["y"] + cbf["const"] for cbf in
                           now_data["cbf_slack"]]
        self.cbf_ct = [self.ax.contour(self.xgrid, self.ygrid, cbf, [0],
                                       colors='orangered')
                       for cbf in self.cbf_l]

    def update(self, _num):
        self.marker_nom.set_data([self.data[_num]["nominal"]["x"]], [self.data[_num]["nominal"]["y"]])
        self.marker_res.set_data([0, self.data[_num]["result"]["x"]], [0, self.data[_num]["result"]["y"]])
        for ct in self.cbf_ct:
            for cl in ct.collections:
                cl.remove()
        now_data = self.data[_num]
        self.cbf_l = []
        if "cbf_no_slack" in now_data:
            self.cbf_l += [self.xgrid * cbf["x"] + self.ygrid * cbf["y"] + cbf["const"] for cbf in
                           now_data["cbf_no_slack"]]
        if "cbf_slack" in now_data:
            self.cbf_l += [self.xgrid * cbf["x"] + self.ygrid * cbf["y"] + cbf["const"] for cbf in
                           now_data["cbf_slack"]]

        if len(self.cbf_l) > 0:
            self.txt.set_text('')
        else:
            self.txt.set_text('Charging')

        self.cbf_ct = [self.ax.contour(self.xgrid, self.ygrid, cbf, [0],
                                       colors='orangered')
                       for cbf in self.cbf_l]
        for ct in self.cbf_ct:
            plt.setp(ct.collections, path_effects=[patheffects.withTickedStroke(angle=60)])


opt_plot = [MyOptPlot(plt.subplot(gs[-1, i]),
                      [dt["opt"][i] for dt in data_dict["state"]],
                      "Robot #{}: Opt Result".format(i + 1))
            for i in range(robot_num)]


# pbar = tqdm(total=total_length, colour='BLUE', ncols=100)


def progress_bar(num):
    progress_percentage = num / total_length * 100
    elap_time = time.time() - tic
    if num > 0:
        eta = (100 - progress_percentage) / progress_percentage * elap_time
    else:
        eta = np.nan
    bar_number = (math.ceil(progress_percentage) // 2)
    print("\r\033[1;31m[%s%%]|%s| "
          "[%d] elap: %.2fs eta: %.2fs\033[0m" % (math.ceil(progress_percentage),
                                                  "█" * bar_number + " " * (50 - bar_number),
                                                  num, elap_time,
                                                  eta),
          end="")


def update(num):
    progress_bar(num)

    ax.clear()
    # cax.clear()

    data_now = data_dict["state"][num]
    run_time = data_now["runtime"]

    Z = cal_dens(data_now)
    ax.imshow(Z, alpha=0.2, extent=(data_dict["para"]["lim"]["x"][0],
                                    data_dict["para"]["lim"]["x"][1],
                                    data_dict["para"]["lim"]["y"][0],
                                    data_dict["para"]["lim"]["y"][1]), origin='lower')
    c_min, c_max = np.min(Z), np.max(Z)
    F.set_clim(c_min, c_max)
    # print("{} to {}".format(c_min, c_max), end="")
    # cbar = fig.colorbar(F, cax=cax, alpha=0.2)

    for i in range(data_dict["para"]["charge"]["num"]):
        ax.add_patch(Circle(xy=(data_dict["para"]["charge"]["pos"][i]["x"], data_dict["para"]["charge"]["pos"][i]["y"]),
                            radius=data_dict["para"]["charge"]["dist"][i], alpha=0.5))

    pos_x_list = [data_now["robot"][i]["x"] for i in range(robot_num)]
    pos_y_list = [data_now["robot"][i]["y"] for i in range(robot_num)]
    batt_list = [data_now["robot"][i]["batt"] for i in range(robot_num)]
    camera_list = [math.degrees(data_now["robot"][i]["camera"]) for i in range(robot_num)]

    if "cvt" in data_now:
        poly_x_list = [[data_now["cvt"][i]["pos"][j]["x"]
                        for j in range(data_now["cvt"][i]["num"])] for i in range(robot_num)]
        poly_y_list = [[data_now["cvt"][i]["pos"][j]["y"]
                        for j in range(data_now["cvt"][i]["num"])] for i in range(robot_num)]

    ax.plot(pos_x_list, pos_y_list, 'b*')

    for i in range(robot_num):
        ax.add_patch(Wedge(center=[pos_x_list[i], pos_y_list[i]], r=0.5, alpha=0.3,
                           theta1=camera_list[i] - 15, theta2=camera_list[i] + 15))

        ax.annotate(('    Robot #{}:' + '\n'
                     + r'$\quadE = {:.2f}$' + '\n'
                     + r'$\quad\theta = {:.2f}$'
                     ).format(i + 1, batt_list[i], camera_list[i]),
                    xy=(pos_x_list[i], pos_y_list[i]))
        # ax.annotate(('    Robot #{}:' + '\n'
        #              + r'$\quadE = {:.2f}$' + '\n'
        #              + r'$\quad\theta = {:.2f}$').format(i + 1, batt_list[i], camera_list[i]),
        #             xy=(pos_x_list[i], pos_y_list[i]))

        if "cvt" in data_now:
            ax.plot(poly_x_list[i], poly_y_list[i], 'k')

    ax.text(0.05, 0.95, 'Time = {:.2f}s'.format(data_now["runtime"]), transform=ax.transAxes)
    ax.set_xlim(data_dict["para"]["lim"]["x"])
    ax.set_ylim(data_dict["para"]["lim"]["y"])

    ax.plot(world_x_list, world_y_list, 'k')

    for i in range(robot_num):
        energy_cbf_plot[i].update(num)
        camera_cbf_plot[i].update(num)
        opt_plot[i].update(num)

    return


ani = animation.FuncAnimation(fig, update, len(data_dict["state"]),
                              interval=int(1000 * interval),
                              blit=False)

# ani.save(filename + 'res.gif')
# print("\ngif saved in {}".format(filename + 'res.gif'))

ani.save(filename + 'res.mp4', writer='ffmpeg', fps=int(1 / interval))
print("\nmp4 saved in {}".format(filename + 'res.mp4'))

toc = time.time()
print("{:.2f} seconds elapsed".format(toc - tic))
