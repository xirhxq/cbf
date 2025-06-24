from .base import *


class AnimationDrawer(BaseDrawer):

    def run_animation(self, plot_list=None):
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

        totalLength = len(self.data["state"])

        pbar = tqdm.tqdm(total=totalLength, bar_format=self.BAR_FORMAT)

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
        filename = os.path.join(self.folder, 'animation-' + suffix + '.mp4')

        fps = int(1 / interval)
        ani.save(filename, writer='ffmpeg', fps=fps)
        pbar.close()
        print(f"\nAnimation saved in {filename}")
