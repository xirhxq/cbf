from .base import *


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
