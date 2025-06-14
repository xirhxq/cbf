from utils import *
from loader import DataLoader
from components.components import *
from layout.grid_layout import *


class BaseDrawer:
    BAR_FORMAT = "{percentage:3.0f}%|{bar:50}| {n_fmt}/{total_fmt} [elap: {elapsed}s eta: {remaining}s]"
    DPI = 300
    FIGSIZE = (16, 9)

    def __init__(self, files):
        self.loader = DataLoader(files)
        self.data = self.loader.data
        self.folder = self.loader.folder

        plt.switch_backend('agg')

    def _check_plot_type(self, plot_type):
        if plot_type not in REGISTRIED_COMPONENTS:
            raise ValueError(
                f"Plot type '{plot_type}' is not registered. "
                f"Available types: {list(REGISTRIED_COMPONENTS.keys())}"
            )

    def _check_plot_list(self, plot_list):
        if any(plot_type not in REGISTRIED_COMPONENTS for plot_type in plot_list):
            raise ValueError(
                f"Some plot types in {plot_list} are not registered. "
                f"Available types: {list(REGISTRIED_COMPONENTS.keys())}"
            )

    def _check_class(self, class_name):
        if class_name not in globals():
            raise ValueError(f"Component class '{class_name}' not found. ")
        return globals()[class_name]
