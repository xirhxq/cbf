from utils import *
from loader import DataLoader
from components.components import *
from layout.grid_layout import *


BAR_FORMAT = "{percentage:3.0f}%|{bar:50}| {n_fmt}/{total_fmt} [elap: {elapsed}s eta: {remaining}s]"


class StaticSeparatePlotDrawer:
    def __init__(self, files):
        self.loader = DataLoader(files)
        self.data = self.loader.data
        self.folder = self.loader.folder

        plt.switch_backend('agg')

    def draw_plots(self, plot_type):
        num_robots = self.data["config"]["num"]

        pbar = tqdm.tqdm(total=num_robots, bar_format=BAR_FORMAT)

        for robot_id in range(num_robots):
            fig, ax = plt.subplots(figsize=(10, 6))
            component_class = REGISTRIED_COMPONENTS[plot_type]["class"]
            if component_class not in globals():
                raise ValueError(f"Component class '{component_class}' not found. ")
            component = globals()[component_class](
                ax=ax,
                data=self.data,
                robot_id=robot_id,
                mode='separate'
            )

            filename = os.path.join(self.folder, plot_type + f"-{robot_id + 1}.png")
            fig.savefig(filename, dpi=300, bbox_inches='tight')
            plt.close(fig)

            pbar.update(1)

        pbar.close()


class StaticGroupPlotDrawer:
    def __init__(self, files):
        self.loader = DataLoader(files)
        self.data = self.loader.data
        self.folder = self.loader.folder

        plt.switch_backend('agg')


    def draw_plots(self, plot_list):
        fig = plt.figure(figsize=(16, 9))
        fig.set_tight_layout(True)

        axes_map = GridLayout(fig, self.data["config"]["num"], plot_list).allocate_axes()

        components = []

        for item in axes_map:
            component_class = item["class"]
            if component_class not in globals():
                raise ValueError(f"Component class '{component_class}' not found. ")
            components.append(
                globals()[component_class](
                    data=self.data,
                    mode='group',
                    **item
                )
            )


        suffix = '-'.join(plot_list)
        filename = os.path.join(self.folder, suffix + '-all.png')
        fig.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)



class AnimationDrawer:
    def __init__(self, files):
        self.loader = DataLoader(files)
        self.data = self.loader.data
        self.folder = self.loader.folder

        self.shotList = []

        plt.switch_backend('agg')

    def run_animation(self, plot_list=None):
        plot_list = ['map'] if plot_list == [] or plot_list is None else plot_list
        fig = plt.figure(figsize=(16, 9))
        fig.set_tight_layout(True)

        axes_map = GridLayout(fig, self.data["config"]["num"], plot_list).allocate_axes()

        components = []

        for item in axes_map:
            component_class = item["class"]
            if component_class not in globals():
                raise ValueError(f"Component class '{component_class}' not found. ")
            components.append(
                globals()[component_class](
                    data=self.data,
                    mode='animation',
                    **item
                )
            )

        totalLength = len(self.data["state"])

        pbar = tqdm.tqdm(total=totalLength, bar_format=BAR_FORMAT)

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
            'opt',
            'fix',
            'cbf'
        ]
    )
