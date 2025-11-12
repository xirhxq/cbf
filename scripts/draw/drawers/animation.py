from .base import *


class AnimationDrawer(BaseDrawer):

    def run_animation(self,
                      plot_list=None,
                      first_seconds=None, last_seconds=None, time_range=None,
                      id_list=None
                      ):
        plot_list = ['map'] if plot_list == [] or plot_list is None else plot_list
        self._check_plot_list(plot_list)
        fig = plt.figure(figsize=self.FIGSIZE)
        if len(plot_list) == 1 and 'figsize' in REGISTRIED_COMPONENTS[plot_list[0]].keys():
            fig = plt.figure(figsize=REGISTRIED_COMPONENTS[plot_list[0]]["figsize"])
        fig.set_tight_layout(True)

        id_list = [id - 1 for id in id_list] if id_list is not None else [i for i in range(self.data["config"]["num"])]

        if len(self.data["state"]) >= 2:
            interval = self.data["state"][1]["runtime"] - self.data["state"][0]["runtime"]
        else:
            interval = 0.5
        fps = 50
        interval_ms = int(1000 / fps)

        self._get_index_range(
            first_seconds=first_seconds, last_seconds=last_seconds, time_range=time_range
        )

        totalLength = self.index_range[1] - self.index_range[0]

        axes_map = GridLayout(fig, plot_list, id_list=id_list).allocate_axes()

        components = []

        for item in axes_map:
            component_class = self._check_class(item["class"])
            components.append(
                component_class(
                    data=self.data,
                    mode='animation',
                    index_range=self.index_range,
                    **item
                )
            )

        pbar = tqdm.tqdm(total=totalLength, bar_format=self.BAR_FORMAT)

        def update(num):
            pbar.update(1)
            for comp in components:
                comp.update(num)

        ani = animation.FuncAnimation(
            fig, update,
            frames=range(*self.index_range),
            interval=interval_ms,
            blit=False
        )

        suffix = '-'.join(plot_list)
        if last_seconds is not None:
            suffix += '-last-' + str(last_seconds)
        elif first_seconds is not None:
            suffix += '-first-' + str(first_seconds)
        elif time_range is not None:
            suffix += '-range-' + str(time_range[0]) + '-' + str(time_range[1])
        if id_list != [i for i in range(self.data["config"]["num"])]:
            suffix += '-' + '-'.join(['#' + str(id + 1) for id in id_list])
        filename = os.path.join(self.folder, 'animation-' + suffix + '.mp4')

        ani.save(filename, writer='ffmpeg', fps=fps)
        pbar.close()
        print(f"\nAnimation saved in {filename}")
