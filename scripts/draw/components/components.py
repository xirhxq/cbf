from .map_animation import MapAnimationComponent
from .optimisation_contour import OptimizationContourPlot
from .fixed_comm_range import FixedCommRangeComponent
from .cbf_values import CBFValuesComponent
from .search_heatmap import SearchHeatmapComponent
from .heatmap import HeatmapComponent
from .energy import EnergyComponent
from .control_input import ControlInputComponent
from .cbf_derivative import CBFDerivativeComponent

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
    'energy': {
        'title': 'Energy Level (Grouped)',
        'class': 'EnergyComponent',
        'filename': 'energy',
        'figsize': (12, 8),
    },
    'u': {
        'class': 'ControlInputComponent',
        'filename': 'u',
        'figsize': (12, 8),
    },
    'cbfd': {
        'title': 'CBF Derivative',
        'class': 'CBFDerivativeComponent',
        'filename': 'cbf-derivative',
        'figsize': (10, 6),
    },
    'cbf-energy': {
        'title': 'Energy CBF Value',
        'class': 'CBFValuesComponent',
        'filename': 'energy-cbf',
        'figsize': (10, 6),
        'params': {
            'cbf_filter': lambda name: name.startswith('energy')
        }
    },
    'cbfd-energy': {
        'title': 'Energy CBF Derivative',
        'class': 'CBFDerivativeComponent',
        'filename': 'energy-cbf-derivative',
        'figsize': (10, 6),
        'params': {
               'cbf_filter': 'energy'
        }
    }
}
