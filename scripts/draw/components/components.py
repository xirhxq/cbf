from .map_animation import MapAnimationComponent
from .optimisation_contour import OptimizationContourPlot
from .fixed_comm_range import FixedCommRangeComponent
from .cbf_values import CBFValuesComponent
from .search_heatmap import SearchHeatmapComponent


REGISTRIED_COMPONENTS = {
    'map': {
        'class': 'MapAnimationComponent',
        'filename': 'map',
        'figsize': (10, 10)
    },
    'opt': {
        'title': 'Opt Result',
        'class': 'OptimizationContourPlot',
        'filename': 'opt',
    },
    'fix': {
        'title': 'Fixed Comm Range',
        'class': 'FixedCommRangeComponent',
        'filename': 'fix',
        'figsize': (10, 6),
    },
    'cbf': {
        'title': 'CBF Values',
        'class': 'CBFValuesComponent',
        'filename': 'cbf',
        'figsize': (10, 6),
    },
    'sh': {
        'title': 'Search Heatmap',
        'class': 'SearchHeatmapComponent',
        'filename': 'search-heatmap',
        'figsize': (8, 8),
    }
}