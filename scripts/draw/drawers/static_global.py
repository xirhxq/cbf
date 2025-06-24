from .base import *


class StaticGlobalPlotDrawer(BaseDrawer):
    def draw_plots(self, plot_list):
        self._check_plot_list(plot_list)

        num_robots = self.data["config"]["num"]

        if len(plot_list) == 1:
            self.FIGSIZE = REGISTRIED_COMPONENTS[plot_list[0]]["figsize"]

        fig = plt.figure(figsize=self.FIGSIZE)
        fig.set_tight_layout(True)

        axes_map = GridLayout(fig, plot_list, expand=False, n=num_robots).allocate_axes()

        for item in axes_map:
            component_class = self._check_class(item["class"])
            item["mode"] = 'global'
            component = component_class(
                data=self.data,
                **item
            )


        suffix = '-'.join([REGISTRIED_COMPONENTS[plot_type]["filename"] for plot_type in plot_list])
        filename = os.path.join(self.folder, suffix + f'.png')
        fig.savefig(filename, dpi=self.DPI, bbox_inches='tight')

        plt.close(fig)
        print(f"Plot saved to {filename}")
