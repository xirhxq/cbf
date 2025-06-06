import math

from utils import *
from components.map_animation import MapAnimationComponent
from components.optimisation import OptimizationContourPlot
from components.fixed_comm_range import FixedCommRangeComponent
from components.cbf_values import CBFValuesComponent
from layout.grid_layout import GridLayout


class AnimationDrawer:
    def __init__(self, file, **kwargs):
        self.folderName = os.path.dirname(file)
        self.file = file
        self.data = json.load(open(self.file))

        self.barFormat = "{percentage:3.0f}%|{bar:50}| {n_fmt}/{total_fmt} [elap: {elapsed}s eta: {remaining}s]"
        self.shotList = []

        plt.switch_backend('agg')

    def run_animation(self):
        robotNum = self.data["config"]["num"]
        sideNum = 3
        sideRows = 4 if robotNum >= 8 else 1
        sideCols = math.ceil(robotNum / sideRows)
        mapCols = math.ceil(sideCols / 2)

        fig = plt.figure(figsize=(16, 9))
        fig.set_tight_layout(True)

        animation_config = {
            'rows': sideRows * sideNum,
            'cols': sideCols + mapCols,
            'components': [
                {
                    'name': 'map',
                    'grid': [[None, None], [None, -sideCols]]
                }
            ]
        }

        for i in range(robotNum):
            animation_config['components'].append({
                'name': f'robot_{i + 1}_opt',
                'grid': [i // sideCols, -sideCols + (i % sideCols)]
            })
            animation_config['components'].append({
                'name': f'robot_{i + 1}_fix_comm',
                'grid': [i // sideCols + sideRows, -sideCols + (i % sideCols)]
            })
            animation_config['components'].append({
                'name': f'robot_{i + 1}_cbf',
                'type': 'cbf_values',
                'grid': [i // sideCols + sideRows * 2, -sideCols + (i % sideCols)]
            })


        print(animation_config)

        layout = GridLayout(fig, animation_config)
        axes_map = layout.allocate_axes()

        components = []

        map_component = MapAnimationComponent(self.data, ax=axes_map['map'])
        components.append(map_component)

        for i in range(robotNum):
            components.append(
                OptimizationContourPlot(
                    axes_map[f'robot_{i + 1}_opt'],
                    [dt["robots"][i]["opt"] for dt in self.data["state"]],
                    f"Opt Result, Robot #{i + 1}"
                )
            )
            components.append(
                FixedCommRangeComponent(
                    axes_map[f'robot_{i + 1}_fix_comm'],
                    self.data["state"],
                    robot_id=i,
                    name=f"Robot #{i + 1} Fixed Communication"
                )
            )
            components.append(
                CBFValuesComponent(
                    axes_map[f'robot_{i + 1}_cbf'],
                    self.data["state"],
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

        ani = animation.FuncAnimation(fig, update, frames=totalLength, interval=int(1000 * map_component.interval),
                                      blit=False)
        filename = os.path.join(self.folderName, 'animation.mp4')

        fps = int(1 / (self.data["state"][1]["runtime"] - self.data["state"][0]["runtime"]))
        ani.save(filename, writer='ffmpeg', fps=fps)
        pbar.close()
        print(f"\nmp4 saved in {filename}")


if __name__ == '__main__':
    filename = findNewestFile('../../data', '*')
    drawer = AnimationDrawer(
        filename,
        config={}
    )
    drawer.run_animation()
