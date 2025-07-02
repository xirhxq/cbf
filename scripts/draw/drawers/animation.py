from .base import *


class AnimationDrawer(BaseDrawer):

    def run_animation(self, plot_list=None, last_seconds=None):
        plot_list = ['map'] if plot_list == [] or plot_list is None else plot_list
        self._check_plot_list(plot_list)
        fig = plt.figure(figsize=self.FIGSIZE)
        if len(plot_list) == 1 and 'figsize' in REGISTRIED_COMPONENTS[plot_list[0]].keys():
            fig = plt.figure(figsize=REGISTRIED_COMPONENTS[plot_list[0]]["figsize"])
        fig.set_tight_layout(True)

        axes_map = GridLayout(fig, plot_list, n=self.data["config"]["num"]).allocate_axes()

        components = []

        for item in axes_map:
            component_class = self._check_class(item["class"])
            components.append(
                component_class(
                    data=self.data,
                    mode='animation',
                    **item
                )
            )

        interval = self.data["state"][1]["runtime"] - self.data["state"][0]["runtime"]
        interval_ms = int(1000 * interval)

        totalLength = len(self.data["state"]) if last_seconds is None else int(last_seconds / interval)

        pbar = tqdm.tqdm(total=totalLength, bar_format=self.BAR_FORMAT)

        def update(num):
            pbar.update(1)
            for comp in components:
                comp.update(num)

        ani = animation.FuncAnimation(
            fig, update,
            frames=totalLength if last_seconds is None else range(-totalLength, 0),
            interval=interval_ms,
            blit=False
        )

        suffix = '-'.join(plot_list)
        if last_seconds is not None:
            suffix += '-last-' + str(last_seconds)
        filename = os.path.join(self.folder, 'animation-' + suffix + '.mp4')

        fps = int(1 / interval)
        ani.save(filename, writer='ffmpeg', fps=fps)
        pbar.close()
        print(f"\nAnimation saved in {filename}")
