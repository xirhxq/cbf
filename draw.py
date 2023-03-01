import json
import math
import os
import re
import time

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patheffects
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle
from matplotlib.patches import Wedge
from mpl_toolkits.axes_grid1 import make_axes_locatable


def find_file(ptn):
    ptn = re.compile(ptn)
    src = 'data'
    files = os.listdir(src)
    json_files = []
    for file in files:
        if re.match(ptn, file):
            json_files.append(f'{src}/{file}')
    # json_files.sort(key=lambda fp: os.path.getctime(fp), reverse=True)
    json_files.sort(reverse=True)
    newest_json = json_files[0]
    print('find {}'.format(newest_json))
    return newest_json[:newest_json.rfind('_') + 1]


def name_to_color(_name):
    _name = _name.lower()
    if 'cvt' in _name:
        return 'orangered'
    if 'energy' in _name:
        return 'mediumblue'
    if 'safe' in _name:
        return 'red'
    return 'black'


class MyBarPlot:
    def __init__(self, _x, _y, _ax, _name, _color='default'):
        self.x_data = _x
        self.y_data = _y
        self.name = _name
        _ax.set_title(_name)
        if _color == 'default':
            _color = name_to_color(_name)
        _ax.plot(self.x_data, self.y_data, color=_color)
        marker, = _ax.plot(self.x_data[:1], self.y_data[:1], "r*")
        self.marker = marker
        line, = _ax.plot(self.x_data[:1] * 2, [self.y_data[0], 0], "r")
        self.line = line

    def update(self, _num):
        self.marker.set_data(self.x_data[_num], self.y_data[_num])
        self.line.set_data(self.x_data[_num], [self.y_data[_num], 0])


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
        self.ax.plot(self.xlim, [0, 0], '--k')
        self.ax.plot([0, 0], self.ylim, '--k')
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
        self.cbf_name = []
        # if "cbf_no_slack" in now_data:
        self.cbf_l += [self.xgrid * cbf["x"] + self.ygrid * cbf["y"] + cbf["const"] for cbf in
                       now_data["cbf_no_slack"]]
        self.cbf_name += [cbf["name"] for cbf in now_data["cbf_no_slack"]]
        # if "cbf_slack" in now_data:
        self.cbf_l += [self.xgrid * cbf["x"] + self.ygrid * cbf["y"] + cbf["const"] for cbf in
                       now_data["cbf_slack"]]
        self.cbf_name += [cbf["name"] for cbf in now_data["cbf_slack"]]

        if len(self.cbf_l) > 0:
            self.txt.set_text('')
        else:
            self.txt.set_text('Charging')

        self.cbf_ct = [self.ax.contour(self.xgrid, self.ygrid, cbf, [0],
                                       colors=name_to_color(self.cbf_name[ind]))
                       for ind, cbf in enumerate(self.cbf_l)]
        for ct in self.cbf_ct:
            plt.setp(ct.collections, path_effects=[patheffects.withTickedStroke(angle=60)])


class MyProgressBar:
    def __init__(self, _total):
        self.total = _total
        self.tic = time.time()

    def update(self, _num):
        progress_percentage = _num / self.total * 100
        elap_time = time.time() - self.tic
        if _num > 0:
            eta = (100 - progress_percentage) / progress_percentage * elap_time
        else:
            eta = np.nan
        bar_number = (math.ceil(progress_percentage) // 2)
        print("\r\033[1;31m[%s%%]|%s| "
              "[%d] elap: %.2fs eta: %.2fs\033[0m" % (math.ceil(progress_percentage),
                                                      "█" * bar_number + " " * (50 - bar_number),
                                                      _num, elap_time,
                                                      eta),
              end="")
        
    def end(self):
        toc = time.time()
        print("{:.2f} seconds elapsed".format(toc - self.tic))
        

def draw_map(file, usetex=False, robot_anno=True, energycbfplot=True, cvtcbfplot=True, optplot=False,
             cameracbfplot=False, commcbfplot=False, safecbfplot=False, bigtimetext=False, figsize=(25, 15),
             show_cvt=True, show_camera=True, show_bar=True, show_axis=True, shot_list=[]):
    if usetex:
        matplotlib.rc('text', usetex=True)

    with open(file + 'data.json') as f:
        data_dict = json.load(f)

    matplotlib.use('agg')

    robot_num = data_dict["para"]["number"]
    row, col = 8, math.ceil(robot_num / 2)

    fig = plt.figure(figsize=figsize)
    gs = GridSpec(row, col)

    bar_plot_on = energycbfplot or cvtcbfplot
    opt_plot_on = optplot

    if bar_plot_on or opt_plot_on:
        ax = plt.subplot(gs[:-2 * (int(bar_plot_on) + int(opt_plot_on)), :])
    else:
        ax = plt.subplot(gs[:,:])
    ax.set_aspect(1)

    if show_bar:
        div = make_axes_locatable(ax)
        cax = div.append_axes('right', '5%', '5%')

    interval = data_dict["state"][1]["runtime"] - data_dict["state"][0]["runtime"]
    fps = int(1 / interval)
    shot_list = [ind * fps for ind in shot_list]
    total_length = len(data_dict["state"])

    world_x_list = [data["x"] for data in data_dict["para"]["world"]]
    world_y_list = [data["y"] for data in data_dict["para"]["world"]]

    xnum, ynum = data_dict["para"]["grid_world"]["x_num"], data_dict["para"]["grid_world"]["y_num"]
    x = np.linspace(data_dict["para"]["grid_world"]["x_lim"][0],
                    data_dict["para"]["grid_world"]["x_lim"][1], xnum)
    y = np.linspace(data_dict["para"]["grid_world"]["y_lim"][0],
                    data_dict["para"]["grid_world"]["y_lim"][1], ynum)
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
            Z += np.exp(
                (
                    np.power(-np.fabs(L - tar["r"]), 3)
                    + 2
                )
                * 10
            )
            ax.add_patch(Circle(xy=(center_x, center_y), radius=0.1, color='r', alpha=0.2))
            ax.annotate('Target', xy=(center_x, center_y))
        return Z

    # Z = np.array(data_dict["state"][0]["grid_world"]).transpose()
    Z = np.zeros((data_dict["para"]["grid_world"]["y_num"],
                  data_dict["para"]["grid_world"]["x_num"]))
    F = ax.imshow(Z, alpha=0.2, extent=(data_dict["para"]["grid_world"]["x_lim"][0],
                                        data_dict["para"]["grid_world"]["x_lim"][1],
                                        data_dict["para"]["grid_world"]["y_lim"][0],
                                        data_dict["para"]["grid_world"]["y_lim"][1]), origin='lower')
    if show_bar:
        cbar = plt.colorbar(F, cax=cax)

    runtime_list = [dt["runtime"] for dt in data_dict["state"]]
    if energycbfplot:
        energy_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["energy_cbf"] for dt in data_dict["state"]],
                                     plt.subplot(gs[-1 - i // 11, i % 11]),
                                     "Robot #{}".format(i + 1), _color='mediumblue')
                           for i in range(robot_num)]
    if cvtcbfplot:
        cvt_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["cvt_cbf"] for dt in data_dict["state"]],
                                  plt.subplot(gs[-1 - i // 11, i % 11]),
                                  "Robot #{}".format(i + 1), _color='orangered')
                        for i in range(robot_num)]

    if cameracbfplot:
        camera_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["camera_cbf"] for dt in data_dict["state"]],
                                     plt.subplot(gs[-2, i]),
                                     "Robot #{}: Camera CBF Value".format(i + 1))
                           for i in range(robot_num)]

    if commcbfplot:
        comm_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["comm_to_" + str(j)] for dt in data_dict["state"]],
                                   plt.subplot(gs[-2, i]),
                                   "Robot #{}: Comm CBF Value".format(i + 1))
                         for i in range(robot_num) for j in range(robot_num) if
                         "comm_to_" + str(j) in data_dict["state"][0]["robot"][i]]

    if safecbfplot:
        safe_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["safe_to_" + str(j)] for dt in data_dict["state"]],
                                   plt.subplot(gs[-2, i]),
                                   "Robot #{}: Safe CBF Value".format(i + 1))
                         for i in range(robot_num) for j in range(robot_num) if
                         "safe_to_" + str(j) in data_dict["state"][0]["robot"][i]]

    if optplot:
        opt_plot = [MyOptPlot(plt.subplot(gs[-1, i]),
                              [dt["opt"][i] for dt in data_dict["state"]],
                              "Robot #{}: Opt Result".format(i + 1))
                    for i in range(robot_num)]

    pb = MyProgressBar(total_length)

    def update(num):
        pb.update(num)

        ax.clear()
        # cax.clear()

        data_now = data_dict["state"][num]
        run_time = data_now["runtime"]

        # grid_world_now = np.array(data_now["grid_world"]).transpose()
        # # print(grid_world_now)
        # Z = grid_world_now
        # Z = cal_dens(data_now)
        for i in range(robot_num):
            update_grids = data_now["update"][i]
            for grid in update_grids:
                Z[grid["y"], grid["x"]] = 1

        ax.imshow(Z, alpha=0.2, extent=(data_dict["para"]["grid_world"]["x_lim"][0],
                                        data_dict["para"]["grid_world"]["x_lim"][1],
                                        data_dict["para"]["grid_world"]["y_lim"][0],
                                        data_dict["para"]["grid_world"]["y_lim"][1]), origin='lower')
        c_min, c_max = np.min(Z), np.max(Z)
        if show_bar:
            F.set_clim(0, 1)
        # print("{} to {}".format(c_min, c_max), end="")
        # cbar = fig.colorbar(F, cax=cax, alpha=0.2)

        for i in range(data_dict["para"]["charge"]["num"]):
            ax.add_patch(Circle(xy=(data_dict["para"]["charge"]["pos"][i]["x"], data_dict["para"]["charge"]["pos"][i]["y"]),
                                radius=data_dict["para"]["charge"]["dist"][i], alpha=0.5))

        pos_x_list = [data_now["robot"][i]["x"] for i in range(robot_num)]
        pos_y_list = [data_now["robot"][i]["y"] for i in range(robot_num)]
        batt_list = [data_now["robot"][i]["batt"] for i in range(robot_num)]
        camera_list = [math.degrees(data_now["robot"][i]["camera"]) for i in range(robot_num)]

        if "cvt" in data_now and show_cvt:
            poly_x_list = [[data_now["cvt"][i]["pos"][j]["x"]
                            for j in range(data_now["cvt"][i]["num"])] for i in range(robot_num)]
            poly_y_list = [[data_now["cvt"][i]["pos"][j]["y"]
                            for j in range(data_now["cvt"][i]["num"])] for i in range(robot_num)]
            poly_center_list = [data_now["cvt"][i]["center"] for i in range(robot_num)]

        ax.plot(pos_x_list, pos_y_list, 'b*')

        for i in range(robot_num):
            if show_camera:
                ax.add_patch(Wedge(center=[pos_x_list[i], pos_y_list[i]], r=0.5, alpha=0.3,
                                   theta1=camera_list[i] - 15, theta2=camera_list[i] + 15))

            if robot_anno:
                ax.annotate((f'    Robot #{i + 1}:' + '\n'
                             + rf'$\quadE = {batt_list[i]:.2f}$' + '\n'
                             + rf'$\quad\theta = {camera_list[i]:.2f}$'
                             ),
                            xy=(pos_x_list[i], pos_y_list[i]))

            if "cvt" in data_now and show_cvt:
                ax.plot(poly_x_list[i], poly_y_list[i], 'k')
                ax.plot([ct["x"] for ct in poly_center_list], [ct["y"] for ct in poly_center_list], '*', color='lime')

        if bigtimetext:
            ax.text(0.38, 0.95, r'$\mathrm{Time}$' + f' $=$ ${data_now["runtime"]:.2f}$' + r'$\mathrm{s}$', transform=ax.transAxes, fontsize=40)
        else:
            ax.text(0.05, 0.95, 'Time = {:.2f}s'.format(data_now["runtime"]), transform=ax.transAxes)
        ax.set_xlim(data_dict["para"]["lim"]["x"])
        ax.set_ylim(data_dict["para"]["lim"]["y"])

        ax.plot(world_x_list, world_y_list, 'k')
        if not show_axis:
            plt.axis('off')

        for i in range(robot_num):
            if energycbfplot:
                energy_cbf_plot[i].update(num)
            if cameracbfplot:
                camera_cbf_plot[i].update(num)
            if optplot:
                opt_plot[i].update(num)
            if cvtcbfplot:
                cvt_cbf_plot[i].update(num)
        if safecbfplot:
            for cbf in safe_cbf_plot:
                cbf.update(num)
        if num in shot_list:
            plt.savefig(f'data/sim1at{int(num / fps)}00.png', bbox_inches='tight')
            print('Shot!', end='')
        return

    ani = animation.FuncAnimation(fig, update, total_length,
                                  interval=int(1000 * interval),
                                  blit=False)

    # ani.save(filename + 'res.gif')
    # print("\ngif saved in {}".format(filename + 'res.gif'))

    ani.save(filename + 'res.mp4', writer='ffmpeg', fps=int(1 / interval))
    print("\nmp4 saved in {}".format(filename + 'res.mp4'))

    pb.end()


if __name__ == '__main__':
    filename = find_file('10-14_10-33.*.json')
    ral_settings = {
        'energycbfplot': False,
        'cvtcbfplot': False,
        'robot_anno': False,
        'usetex': True,
        'bigtimetext': True,
        'show_camera': False,
        'show_cvt': True,
        'show_bar': False,
        'show_axis': False,
        'figsize': (15, 15),
        'shot_list': [0, 3, 15, 25, 50, 63]
    }
    # draw_map(filename)
    draw_map(filename, **ral_settings)
