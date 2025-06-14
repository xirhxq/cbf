from .base import *


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
