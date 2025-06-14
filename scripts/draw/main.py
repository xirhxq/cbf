from utils import *
from drawers.drawers import *

if __name__ == '__main__':
    files = [findNewestFile('../../data', '*')]
    StaticGlobalPlotDrawer(files).draw_plots('sh')
    StaticSeparatePlotDrawer(files).draw_plots('cbf')
    StaticSeparatePlotDrawer(files).draw_plots('fix')
    StaticGroupPlotDrawer(files).draw_plots(
        plot_list=[
            'fix',
            'cbf'
        ]
    )
    AnimationDrawer(files).run_animation(
        plot_list=[
            'map',
        ]
    )
    AnimationDrawer(files).run_animation(
        plot_list=[
            'map',
            'opt',
            'fix',
            'cbf'
        ]
    )
