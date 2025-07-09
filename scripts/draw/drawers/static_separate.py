from .base import *


class StaticSeparatePlotDrawer(BaseDrawer):
    def draw_plots(self,
                   plot_list,
                   first_seconds=None, last_seconds=None, time_range=None,
                   id_list=None
                   ):
        self._check_plot_list(plot_list)

        num_robots = self.data["config"]["num"]
        id_list = [id - 1 for id in id_list] if id_list else [i for i in range(num_robots)]

        pbar = tqdm.tqdm(total=len(id_list), bar_format=self.BAR_FORMAT)

        if len(plot_list) == 1:
            self.FIGSIZE = REGISTRIED_COMPONENTS[plot_list[0]]["figsize"]

        self._get_index_range(
            first_seconds=first_seconds, last_seconds=last_seconds, time_range=time_range
        )

        for robot_id in id_list:
            fig = plt.figure(figsize=self.FIGSIZE)
            fig.set_tight_layout(True)

            axes_map = GridLayout(fig, plot_list, expand=False, id_list=[robot_id]).allocate_axes()

            for item in axes_map:
                component_class = self._check_class(item["class"])
                item["mode"] = 'separate'
                component = component_class(
                    data=self.data,
                    **item,
                    index_range=self.index_range
                )

            suffix = '-'.join([REGISTRIED_COMPONENTS[plot_type]["filename"] for plot_type in plot_list])
            if last_seconds is not None:
                suffix += '-last-' + str(last_seconds)
            elif first_seconds is not None:
                suffix += '-first-' + str(first_seconds)
            elif time_range is not None:
                suffix += '-range-' + str(time_range[0]) + '-' + str(time_range[1])

            filename = os.path.join(self.folder, suffix + f'-#{robot_id + 1}.png')
            fig.savefig(filename, dpi=self.DPI, bbox_inches='tight')

            plt.close(fig)
            pbar.update(1)

        pbar.close()
        print(f"Plot saved to {filename}")