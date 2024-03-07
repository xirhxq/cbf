import json
import math
import os
import re
import time
import ffmpeg

import matplotlib
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patheffects
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle
from matplotlib.patches import Wedge
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.ticker as ticker


def sort_data():
    for filename in os.listdir('data'):
        if os.path.isfile(os.path.join('data', filename)) and any(c.isnumeric() for c in filename):
            name, ext = os.path.splitext(filename)
            parts = name.split('_')
            subfolder = '_'.join(parts[:-1])
            new_name = parts[-1] + ext
            subfolder_path = os.path.join('data', subfolder)
            if not os.path.exists(subfolder_path):
                os.mkdir(subfolder_path)
            os.rename(os.path.join('data', filename), os.path.join(subfolder_path, new_name))
            print(f'Move {os.path.join("data", filename)} to {os.path.join(subfolder_path, new_name)}')


def find_file(ptn):
    ptn = re.compile(ptn)
    src = 'data'
    files = os.listdir(src)
    json_files = []
    for file in files:
        if re.match(ptn, file) and os.path.isdir(os.path.join(src, file)):
            json_files.append(os.path.join(src, file))
    # json_files.sort(key=lambda fp: os.path.getctime(fp), reverse=True)
    json_files.sort(reverse=True)
    newest_json = json_files[0] + '/'
    print(f'find {newest_json}')
    return newest_json


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
    def __init__(self, _x, _y, _ax, _name, _color='default', markeron=True):
        self.x_data = _x
        self.y_data = _y
        self.name = _name
        _ax.set_title(_name)
        if _color == 'default':
            _color = name_to_color(_name)
        _ax.plot(self.x_data, self.y_data, color=_color)
        if markeron:
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
    half_num = math.ceil(robot_num / 2)
    row, col = 8, half_num

    fig = plt.figure(figsize=figsize)
    gs = GridSpec(row, col)

    bar_plot_on = energycbfplot or cvtcbfplot
    opt_plot_on = optplot

    if bar_plot_on or opt_plot_on:
        ax = plt.subplot(gs[:-2 * (int(bar_plot_on) + int(opt_plot_on)), :])
    else:
        ax = plt.subplot(gs[:, :])
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
                                     plt.subplot(gs[-1 - i // half_num, i % half_num]),
                                     "#{}".format(i + 1), _color='mediumblue')
                           for i in range(robot_num)]
    if cvtcbfplot:
        cvt_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["cvt_cbf"] for dt in data_dict["state"]],
                                  plt.subplot(gs[-1 - i // half_num, i % half_num]),
                                  "#{}".format(i + 1), _color='orangered')
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
                # ax.annotate((f'    Robot #{i + 1}:' + '\n'
                #              + rf'$\quadE = {batt_list[i]:.2f}$' + '\n'
                #              + rf'$\quad\theta = {camera_list[i]:.2f}$'
                #              ),
                #             xy=(pos_x_list[i], pos_y_list[i]))
                ax.annotate(f'    #{i + 1}' + '\n' + f'  E: {batt_list[i]:.2f} ', xy=(pos_x_list[i], pos_y_list[i]), fontsize=8)

            if "cvt" in data_now and show_cvt:
                ax.plot(poly_x_list[i], poly_y_list[i], 'k')
                ax.plot([ct["x"] for ct in poly_center_list], [ct["y"] for ct in poly_center_list], '*', color='lime')

        if bigtimetext:
            ax.set_title(r'$\mathrm{Time}$' + f' $=$ ${data_now["runtime"]:.2f}$' + r'$\mathrm{s}$', fontsize=25, y=0.95)
            # ax.text(0.38, 0.95, r'$\mathrm{Time}$' + f' $=$ ${data_now["runtime"]:.2f}$' + r'$\mathrm{s}$', transform=ax.transAxes, fontsize=40)
        else:
            ax.text(0.05, 0.95, 'Time = {:.2f}s'.format(data_now["runtime"]), transform=ax.transAxes)
        ax.set_xlim(data_dict["para"]["lim"]["x"])
        ax.set_ylim(data_dict["para"]["lim"]["y"])

        ax.plot(world_x_list, world_y_list, 'k')
        if not show_axis:
            ax.set_axis_off()

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
            plt.savefig(filename + f'res_at_sec_{int(num / fps)}.png', bbox_inches='tight')
            print('Shot!', end='')
        return

    ani = animation.FuncAnimation(fig, update, total_length,
                                  interval=int(1000 * interval),
                                  blit=False)

    # ani.save(filename + 'res.gif')
    # print("\ngif saved in {}".format(filename + 'res.gif'))

    ani.save(filename + 'res.mp4', writer='ffmpeg', fps=int(1 / interval))
    print("\nmp4 saved in {}".format(filename[:-1] + '/res.mp4'))

    pb.end()


def draw_stats(file, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
               cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
               show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    with open(file + 'data.json') as f:
        data_dict = json.load(f)

    matplotlib.use('agg')

    plt.figure(figsize=figsize)

    robot_num = data_dict["para"]["number"]

    runtime_list = [dt["runtime"] for dt in data_dict["state"]]

    pb = MyProgressBar(robot_num)

    for i in range(robot_num):
        plt.subplot(211).clear()
        plt.subplot(212).clear()
        plt.subplot(211).plot(runtime_list, [dt["robot"][i]["batt"] for dt in data_dict["state"]], color='C0')
        plt.subplot(211).set_title('Energy Level' + f' of UAV #{i + 1}')
        plt.subplot(211).set_xlabel('Time / s')
        plt.subplot(211).set_ylabel('Energy Level')

        plt.subplot(212).plot(runtime_list, [max(dt["robot"][i]["energy_cbf"], 0) for dt in data_dict["state"]], color='C0')
        plt.subplot(212).set_title(r'CBF Value $min(h_{energy}, h_{l10n})$' + f' of UAV #{i + 1}')
        plt.subplot(212).set_xlabel('Time / s')
        plt.subplot(212).set_ylabel('$min(h_{energy}, h_{l10n})$')
        plt.subplots_adjust(hspace=0.7)
        # leg = ax.legend()
        plot_file_name = file[:-1] + f'/{i + 1}_energy&cbfval.png'
        plt.savefig(plot_file_name, bbox_inches='tight')
        pb.update(i + 1)

    pb.end()


def draw_cbfs(file, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
              cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
              show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    with open(file + 'data.json') as f:
        data_dict = json.load(f)

    matplotlib.use('agg')

    plt.figure(figsize=figsize)

    robot_num = data_dict["para"]["number"]

    runtime_list = [dt["runtime"] for dt in data_dict["state"]]

    pb = MyProgressBar(robot_num)

    for i in range(robot_num):
        plt.subplot(211).clear()
        plt.subplot(212).clear()
        plt.subplot(211).plot(runtime_list, [dt["robot"][i]["cvt_cbf"] for dt in data_dict["state"]], color='C0')
        plt.subplot(211).set_title(r'CBF Value $h_{task}$' + f' of UAV #{i + 1}')
        plt.subplot(211).set_xlabel('Time / s')
        plt.subplot(211).set_ylabel('$h_{task}$')

        plt.subplot(212).plot(runtime_list, [max(dt["robot"][i]["energy_cbf"], 0) for dt in data_dict["state"]], color='C0')
        plt.subplot(212).set_title(r'CBF Value $h_{constraint}$' + f' of UAV #{i + 1}')
        plt.subplot(212).set_xlabel('Time / s')
        plt.subplot(212).set_ylabel('$h_{constraint}$')
        plt.subplots_adjust(hspace=0.7)
        # leg = ax.legend()
        plot_file_name = file[:-1] + f'/{i + 1}_cbfs.png'
        plt.savefig(plot_file_name, bbox_inches='tight')
        pb.update(i + 1)

    pb.end()


def draw_energy_all(file, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
                    cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
                    show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    with open(file + 'data.json') as f:
        data_dict = json.load(f)

    matplotlib.use('agg')

    plt.figure(figsize=figsize)

    robot_num = data_dict["para"]["number"]

    runtime_list = [dt["runtime"] for dt in data_dict["state"]]

    for i in range(robot_num):
        plt.subplot(111).plot(runtime_list, [dt["robot"][i]["batt"] for dt in data_dict["state"]], label=f'UAV #{i + 1}')

    leg = plt.subplot(111).legend(bbox_to_anchor=(1.0, -0.15), ncol=5)
    plt.subplot(111).set_xlabel('Time / s')
    plt.subplot(111).set_ylabel('Energy Level')
    plt.subplot(111).set_title('Energy Level' + f' of all UAVs')
    plot_file_name = file[:-1] + f'/energy_all.png'
    plt.savefig(plot_file_name, bbox_inches='tight')

def draw_energy2(files, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
                    cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
                    show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    matplotlib.use('agg')

    plt.figure(figsize=figsize)

    robot_num = len(files)

    for i in range(robot_num):
        with open(files[i] + 'data.json') as f:
            data_dict = json.load(f)
        runtime_list = [dt["runtime"] for dt in data_dict["state"]]
        plt.subplot(111).plot(runtime_list, [dt["robot"][1]["batt"] * 0.005 + 3.7 for dt in data_dict["state"]],
                              label=f'{files[i][:-1].split("_")[-1]}')

    leg = plt.subplot(111).legend(loc='best')
    plt.subplot(111).set_xlabel('Time / s')
    plt.subplot(111).set_ylabel('Energy Level / V')
    plt.subplot(111).set_title('Energy Level of UAV #2 in different approaches')
    plt.axhspan(3.7, 4.2, facecolor='gray', alpha=0.5)

    plot_file_name = files[0][:-1] + f'/energy_2_contrast.png'
    plt.savefig(plot_file_name, bbox_inches='tight')


def drawDistance1To2ForAll(files, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
                          cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
                          show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    matplotlib.use('agg')

    plt.figure(figsize=figsize)

    fileInd = len(files)

    for i in range(fileInd):
        with open(files[i] + 'data.json') as f:
            data_dict = json.load(f)
        runtime_list = [dt["runtime"] for dt in data_dict["state"]]
        distance = [np.sqrt((dt["robot"][0]["x"] - dt["robot"][1]["x"]) ** 2 +
                            (dt["robot"][0]["y"] - dt["robot"][1]["y"]) ** 2) * 85
                    for dt in data_dict["state"]]
        plt.subplot(111).plot(runtime_list, distance,
                                label=f'{files[i][:-1].split("_")[-1]}')

    leg = plt.subplot(111).legend(loc='best')
    plt.subplot(111).set_xlabel('Time / s')
    plt.subplot(111).set_ylabel('Distance / m')
    plt.subplot(111).set_title('Distance from UAV #2 to UAV #1 in different approaches')

    plt.axhspan(0, 850, facecolor='gray', alpha=0.5)

    plot_file_name = files[0][:-1] + f'/distance_1_2_all_contrast.png'
    plt.savefig(plot_file_name, bbox_inches='tight')




def draw_all_cvt_cbf(file, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
                     cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
                     show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    with open(file + 'data.json') as f:
        data_dict = json.load(f)

    matplotlib.use('agg')

    robot_num = data_dict["para"]["number"]
    half_num = math.ceil(robot_num / 2)
    row, col = half_num, 2

    fig = plt.figure(figsize=figsize)
    gs = GridSpec(row, col)

    runtime_list = [dt["runtime"] for dt in data_dict["state"]]

    cvt_cbf_plot = [MyBarPlot(runtime_list, [dt["robot"][i]["cvt_cbf"] for dt in data_dict["state"]],
                              plt.subplot(gs[i % half_num, i // half_num]),
                              "#{}".format(i + 1), _color='orangered', markeron=False)
                    for i in range(robot_num)]
    plot_file_name = file[:-1] + f'/all_cvt_cbf.png'
    plt.savefig(plot_file_name, bbox_inches='tight')


def draw_heatmap(file, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
                 cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
                 show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    with open(file + 'data.json') as f:
        data_dict = json.load(f)

    matplotlib.use('agg')

    plt.figure(figsize=figsize)

    robot_num = data_dict["para"]["number"]

    runtime_list = [dt["runtime"] for dt in data_dict["state"]]

    pb = MyProgressBar(robot_num)

    for i in range(robot_num):
        plt.subplot(111).clear()
        plt.subplot(111).set_aspect(1)
        plt.subplot(111).set_xlabel('x')
        plt.subplot(111).set_ylabel('y')
        plt.subplot(111).set_title('Heatmap of' + f' UAV {i + 1}')

        x_pos_ls = [dt["robot"][i]["x"] for dt in data_dict["state"]]
        y_pos_ls = [dt["robot"][i]["y"] for dt in data_dict["state"]]

        # 使用 2D 直方图生成热力图
        heatmap, xedges, yedges = np.histogram2d(x_pos_ls, y_pos_ls, bins=20,
                                                 range=[[-10, 10], [0, 20]])

        # 将热力图数据转换为图像数据
        extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
        heatmap = heatmap.T

        # 绘制热力图
        plt.clf()
        plt.imshow(heatmap, extent=extent, origin='lower', cmap='jet', aspect='auto')
        # plt.colorbar()

        plt.ylim([0, 20])
        plt.xlim([-10, 10])
        yticks = np.arange(yedges[0], yedges[-1]+1, 5)
        plt.yticks(yticks)
        # plt.title('Heat-map' + f' of UAV #{i + 1}')
        # axis off
        plt.axis('off')

        plot_file_name = file[:-1] + f'/{i + 1}_heatmap.png'
        plt.savefig(plot_file_name, bbox_inches='tight')
        pb.update(i + 1)

    pb.end()


def drawSearchHeatmap(file, usetex=False, energycbfplot=True, cvtcbfplot=True, optplot=False,
                      cameracbfplot=False, commcbfplot=False, safecbfplot=False, figsize=(25, 15),
                      show_axis=True):
    if usetex:
        matplotlib.rc('text', usetex=True)

    with open(file + 'data.json') as f:
        data_dict = json.load(f)

    matplotlib.use('agg')

    plt.figure(figsize=figsize)

    robot_num = data_dict["para"]["number"]

    runtime_list = [dt["runtime"] for dt in data_dict["state"]]

    xnum, ynum = data_dict["para"]["grid_world"]["x_num"], data_dict["para"]["grid_world"]["y_num"]
    Z = np.zeros((ynum, xnum))
    Z -= 1

    length = len(data_dict["state"])
    for l in range(length):
        data_now = data_dict["state"][l]
        for i in range(robot_num):
            update_grids = data_now["update"][i]
            for grid in update_grids:
                if Z[grid["y"], grid["x"]] == -1:
                    Z[grid["y"], grid["x"]] = l

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


    plot_file_name = file[:-1] + f'/SearchHeatmap.png'
    plt.savefig(plot_file_name, bbox_inches='tight')


def screenshot_from_video(filename):
    # 指定要导出图像的时间点
    timepoints = [0, 10, 25, 80, 140, 220, 275]

    # 指定要导出图像的视频文件
    input_file = filename + 'res.mp4'

    # 循环遍历每个时间点，并导出相应的图像
    for t in timepoints:
        # 设置导出图像的文件名
        output_file = filename + f'res_at_sec_{t}_new.png'

        # 使用FFmpeg导出图像
        (
            ffmpeg
            .input(input_file, ss=t)
            .filter('scale', '-1', '720')
            .output(output_file, vframes=1)
            .overwrite_output()
            .run()
        )

        # 确认导出的文件存在
        if os.path.isfile(output_file):
            print(f'导出成功：{output_file}')
        else:
            print(f'导出失败：{output_file}')


if __name__ == '__main__':
    sort_data()
    filenames = [find_file('.*_Ours'), find_file('.*_CVT'), find_file('.*_EnergyCBF&CVT')]
    print(f'{filenames = }')
    ral_settings = {
        'energycbfplot': False,
        'cvtcbfplot': False,
        'robot_anno': False,
        # 'usetex': True,
        'bigtimetext': True,
        'show_camera': False,
        'show_cvt': True,
        'show_bar': False,
        'show_axis': False,
        'figsize': (8, 8),
        'shot_list': [0, 10, 25, 80, 140, 220, 275]
    }
    main_settings = {
        'energycbfplot': False,
        'cvtcbfplot': False,
        'robot_anno': True,
        # 'usetex': True,
        'bigtimetext': False,
        'show_camera': False,
        'show_cvt': True,
        'show_bar': False,
        'show_axis': False,
        'figsize': (7, 7),
        'shot_list': []
    }
    settings = ral_settings
    if len(filenames) == 1:
        filename = filenames[0]

        while True:
            print('-' * 10 + 'Choose which drawing you want:' + '-' * 10)
            print('[0]: Quit')
            print('[1]: Draw whole map video')
            print('[2]: Draw map video with shots')
            print('[3]: Draw stats')
            print('[4]: Draw all energy')
            print('[5]: Draw cbf values')
            print('[6]: Draw heat-maps')
            print('[7]: Draw all cvt cbf')
            print('[8]: Screenshots from video')
            print('[9]: Draw search heat-map')
            op = int(input('Input the number: '))
            # draw_map(filename)
            if op == 0:
                break
            elif op == 1:
                ral_settings['shot_list'] = []
                draw_map(filename, **settings)
            elif op == 2:
                draw_map(filename, **settings)
            else:
                settings = {k: v for k, v in settings.items() if k in ['energycbfplot', 'cvtcbfplot', 'show_axis']}
                if op == 3:
                    settings['figsize'] = (8, 4)
                    draw_stats(filename, **settings)
                elif op == 4:
                    settings['figsize'] = (8, 4)
                    draw_energy_all(filename, **settings)
                elif op == 5:
                    settings['figsize'] = (8, 4)
                    draw_cbfs(filename, **settings)
                elif op == 6:
                    settings['figsize'] = (8, 8)
                    draw_heatmap(filename, **settings)
                elif op == 7:
                    settings['figsize'] = (15, 20)
                    draw_all_cvt_cbf(filename, **settings)
                elif op == 8:
                    screenshot_from_video(filename)
                elif op == 9:
                    settings['figsize'] = (8, 8)
                    drawSearchHeatmap(filename, **settings)
    else:
        while True:
            settings = {k: v for k, v in settings.items() if k in ['energycbfplot', 'cvtcbfplot', 'show_axis']}
            print('-' * 10 + 'Choose which drawing you want:' + '-' * 10)
            print('[0]: Quit')
            print('[1]: Draw energys')
            print('[2]: Draw distance between 1 and 2')
            op = int(input('Input the number: '))
            # draw_map(filename)
            if op == 0:
                break
            elif op == 1:
                settings['figsize'] = (8, 3)
                draw_energy2(filenames, **settings)
            elif op == 2:
                settings['figsize'] = (8, 3)
                drawDistance1To2ForAll(filenames, **settings)


