from .base import *


class StaticSeparatePlotDrawer(BaseDrawer):
    def draw_plots(self, plot_list):
        self._check_plot_list(plot_list)

        num_robots = self.data["config"]["num"]

        pbar = tqdm.tqdm(total=num_robots, bar_format=self.BAR_FORMAT)

        if len(plot_list) == 1:
            self.FIGSIZE = REGISTRIED_COMPONENTS[plot_list[0]]["figsize"]

        for robot_id in range(num_robots):
            fig = plt.figure(figsize=self.FIGSIZE)
            fig.set_tight_layout(True)

            axes_map = GridLayout(fig, plot_list, expand=False, n=num_robots, robot_id=robot_id).allocate_axes()

            for item in axes_map:
                component_class = self._check_class(item["class"])
                item["mode"] = 'separate'
                component = component_class(
                    data=self.data,
                    **item
                )

            suffix = '-'.join([REGISTRIED_COMPONENTS[plot_type]["filename"] for plot_type in plot_list])
            filename = os.path.join(self.folder, suffix + f'-{robot_id + 1}.png')
            fig.savefig(filename, dpi=self.DPI, bbox_inches='tight')

            plt.close(fig)
            pbar.update(1)

        pbar.close()