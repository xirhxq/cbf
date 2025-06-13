from utils import *
from loader import DataLoader
from components.map_animation import MapAnimationComponent
from components.optimisation import OptimizationContourPlot
from components.fixed_comm_range import FixedCommRangeComponent
from components.cbf_values import CBFValuesComponent
from layout.grid_layout import GridLayout


class AnimationDrawer:
    def __init__(self, files):
        self.loader = DataLoader(files)
        self.data = self.loader.data
        self.folderName = self.loader.folderName

        self.barFormat = "{percentage:3.0f}%|{bar:50}| {n_fmt}/{total_fmt} [elap: {elapsed}s eta: {remaining}s]"
        self.shotList = []

        plt.switch_backend('agg')

    def run_animation(self, plot_list=None):
        plot_list = ['map'] if plot_list == [] or plot_list is None else plot_list
        fig = plt.figure(figsize=(16, 9))
        fig.set_tight_layout(True)

        axes_map = GridLayout(fig, self.data["config"]["num"], plot_list).allocate_axes()

        components = []
        if 'map' in plot_list:
            map_component = MapAnimationComponent(axes_map['map'], self.data, name="Map")
            components.append(map_component)

        for i in range(self.data["config"]["num"]):
            if 'opt' in plot_list:
                components.append(
                    OptimizationContourPlot(
                        axes_map[f'opt_{i + 1}'],
                        self.data,
                        robot_id=i,
                        name=f"Opt Result, Robot #{i + 1}"
                    )
                )
            if 'fix' in plot_list:
                components.append(
                    FixedCommRangeComponent(
                        axes_map[f'fix_{i + 1}'],
                        self.data,
                        robot_id=i,
                        name=f"Robot #{i + 1} Fixed Communication"
                    )
                )
            if 'cbf' in plot_list:
                components.append(
                    CBFValuesComponent(
                        axes_map[f'cbf_{i + 1}'],
                        self.data,
                        robot_id=i,
                        name=f"CBF Values, Robot #{i + 1}"
                    )
                )

        totalLength = len(self.data["state"])

        pbar = tqdm.tqdm(total=totalLength, bar_format=self.barFormat)

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
        filename = os.path.join(self.folderName, 'animation-' + suffix + '.mp4')

        fps = int(1 / interval)
        ani.save(filename, writer='ffmpeg', fps=fps)
        pbar.close()
        print(f"\nmp4 saved in {filename}")


if __name__ == '__main__':
    files = [findNewestFile('../../data', '*')]
    drawer = AnimationDrawer(
        files
    )
    drawer.run_animation(
        plot_list=[
            'map',
            'opt',
            'fix',
            'cbf'
        ]
    )
