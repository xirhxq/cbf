from .map_animation import MapAnimationComponent
from .optimisation_contour import OptimizationContourPlot
from .fixed_comm_range import FixedCommRangeComponent
from .cbf_values import CBFValuesComponent
from .search_heatmap import SearchHeatmapComponent


REGISTRIED_COMPONENTS = {
    'map': {
        'class': 'MapAnimationComponent',
    },
    'opt': {
        'title': 'Opt Result',
        'class': 'OptimizationContourPlot',
    },
    'fix': {
        'title': 'Fixed Comm Range',
        'class': 'FixedCommRangeComponent',
    },
    'cbf': {
        'title': 'CBF Values',
        'class': 'CBFValuesComponent',
    },
    'sh': {
        'title': 'Search Heatmap',
        'class': 'SearchHeatmapComponent',
    }
}