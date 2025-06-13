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

        for item in axes_map:
            component_class = item["class"]
            if component_class not in globals():
                raise ValueError(f"Component class '{component_class}' not found. ")
            components.append(
                globals()[component_class](
                    data=self.data,
                    **item
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
    AnimationDrawer(files).run_animation(
        plot_list=[
            'map',
            'opt',
            'fix',
            'cbf'
        ]
    )
