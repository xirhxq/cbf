from utils import *
from loader import DataLoader
from components.components import *
from layout.grid_layout import *


class BaseDrawer:
    BAR_FORMAT = "{percentage:3.0f}%|{bar:50}| {n_fmt}/{total_fmt} [elap: {elapsed}s eta: {remaining}s]"
    DPI = 300
    FIGSIZE = (16, 9)

    def __init__(self, files):
        self.loader = DataLoader(files)
        self.data = self.loader.data
        self.folder = self.loader.folder

        plt.switch_backend('agg')

    def _check_plot_type(self, plot_type):
        if plot_type not in REGISTRIED_COMPONENTS:
            raise ValueError(
                f"Plot type '{plot_type}' is not registered. "
                f"Available types: {list(REGISTRIED_COMPONENTS.keys())}"
            )

    def _check_plot_list(self, plot_list):
        if any(plot_type not in REGISTRIED_COMPONENTS for plot_type in plot_list):
            raise ValueError(
                f"Some plot types in {plot_list} are not registered. "
                f"Available types: {list(REGISTRIED_COMPONENTS.keys())}"
            )

    def _check_class(self, class_name):
        if class_name not in globals():
            raise ValueError(f"Component class '{class_name}' not found. ")
        return globals()[class_name]


class StaticGlobalPlotDrawer(BaseDrawer):
    def draw_plots(self, plot_type):
        self._check_plot_type(plot_type)
        config = REGISTRIED_COMPONENTS[plot_type]

        fig, ax = plt.subplots(figsize=config["figsize"])
        fig.set_tight_layout(True)

        component_class = self._check_class(config["class"])
        component = component_class(
            ax=ax,
            data=self.data,
            mode='global'
        )

        filename = os.path.join(self.folder, config["filename"] + '.png')
        fig.savefig(filename, dpi=self.DPI, bbox_inches='tight')
        plt.close(fig)


class StaticSeparatePlotDrawer(BaseDrawer):
    def draw_plots(self, plot_type):
        self._check_plot_type(plot_type)
        config = REGISTRIED_COMPONENTS[plot_type]

        num_robots = self.data["config"]["num"]

        pbar = tqdm.tqdm(total=num_robots, bar_format=self.BAR_FORMAT)

        for robot_id in range(num_robots):
            fig, ax = plt.subplots(figsize=config["figsize"])
            component_class = self._check_class(config["class"])
            component = component_class(
                ax=ax,
                data=self.data,
                robot_id=robot_id,
                mode='separate'
            )

            filename = os.path.join(self.folder, config["filename"] + f"-{robot_id + 1}.png")
            fig.savefig(filename, dpi=self.DPI, bbox_inches='tight')
            plt.close(fig)

            pbar.update(1)

        pbar.close()


class StaticGroupPlotDrawer(BaseDrawer):
    def draw_plots(self, plot_list):
        self._check_plot_list(plot_list)
        fig = plt.figure(figsize=self.FIGSIZE)
        fig.set_tight_layout(True)

        axes_map = GridLayout(fig, self.data["config"]["num"], plot_list).allocate_axes()

        components = []

        for item in axes_map:
            component_class = self._check_class(item["class"])
            components.append(
                component_class(
                    data=self.data,
                    mode='group',
                    **item
                )
            )


        suffix = '-'.join([REGISTRIED_COMPONENTS[plot_type]["filename"] for plot_type in plot_list])
        filename = os.path.join(self.folder, suffix + '-all.png')
        fig.savefig(filename, dpi=self.DPI, bbox_inches='tight')
        plt.close(fig)



class AnimationDrawer(BaseDrawer):

    def run_animation(self, plot_list=None):
        plot_list = ['map'] if plot_list == [] or plot_list is None else plot_list
        self._check_plot_list(plot_list)
        fig = plt.figure(figsize=self.FIGSIZE)
        if len(plot_list) == 1 and 'figsize' in REGISTRIED_COMPONENTS[plot_list[0]].keys():
            fig = plt.figure(figsize=REGISTRIED_COMPONENTS[plot_list[0]]["figsize"])
        fig.set_tight_layout(True)

        axes_map = GridLayout(fig, self.data["config"]["num"], plot_list).allocate_axes()

        components = []

        for item in axes_map:
            component_class = self._check_class(item["class"])
            components.append(
                component_class(
                    data=self.data,
                    mode='animation',
                    **item
                )
            )

        totalLength = len(self.data["state"])

        pbar = tqdm.tqdm(total=totalLength, bar_format=self.BAR_FORMAT)

        def update(num):
            pbar.update(1)
            for comp in components:
                comp.update(num)

        interval = self.data["state"][1]["runtime"] - self.data["state"][0]["runtime"]
        interval_ms = int(1000 * interval)

        ani = animation.FuncAnimation(
            fig, update,
            frames=totalLength,
            interval=interval_ms,
            blit=False
        )

        suffix = '-'.join(plot_list)
        filename = os.path.join(self.folder, 'animation-' + suffix + '.mp4')

        fps = int(1 / interval)
        ani.save(filename, writer='ffmpeg', fps=fps)
        pbar.close()
        print(f"\nmp4 saved in {filename}")


if __name__ == '__main__':
    files = [findNewestFile('../../data', '*')]
    StaticGlobalPlotDrawer(files).draw_plots('sh')
    StaticSeparatePlotDrawer(files).draw_plots('cbf')
    StaticSeparatePlotDrawer(files).draw_plots('fix')
    StaticGroupPlotDrawer(files).draw_plots(
        plot_list=[
            'fix',
            'cbf'
        ]
    )
    AnimationDrawer(files).run_animation(
        plot_list=[
            'map',
        ]
    )
    AnimationDrawer(files).run_animation(
        plot_list=[
            'map',
            'opt',
            'fix',
            'cbf'
        ]
    )
