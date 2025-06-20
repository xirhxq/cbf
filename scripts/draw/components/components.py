from .map_animation import MapAnimationComponent
from .optimisation_contour import OptimizationContourPlot
from .fixed_comm_range import FixedCommRangeComponent
from .cbf_values import CBFValuesComponent
from .search_heatmap import SearchHeatmapComponent
from .heatmap import HeatmapComponent
from .energy import EnergyComponent

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
    },
    'heat': {
        'title': 'Heatmap',
        'class': 'HeatmapComponent',
        'filename': 'heatmap',
        'figsize': (8, 8),
    },
    'cvt': {
        'title': 'CVT CBF Value',
        'class': 'CBFValuesComponent',
        'filename': 'cvtcbf',
        'figsize': (10, 6),
        'params': {
            'cbf_filter': 'cvt'
        }
    },
    'min': {
        'title': 'Mininum of CBF without Slack',
        'class': 'CBFValuesComponent',
        'filename': 'mincbf',
        'figsize': (10, 6),
        'params': {
            'cbf_filter': 'min'
        }
    },
    'energy-all': {
        'title': 'Energy Level (All)',
        'class': 'EnergyComponent',
        'filename': 'energy-all',
        'figsize': (10, 6),
        'mode': 'global'
    },
    'energy': {
        'title': 'Energy Level (Grouped)',
        'class': 'EnergyComponent',
        'filename': 'energy',
        'figsize': (12, 8),
    },
}
