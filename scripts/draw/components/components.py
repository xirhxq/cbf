from .map_animation import MapAnimationComponent
from .optimisation_contour import OptimizationContourPlot
from .fixed_comm_range import FixedCommRangeComponent
from .cbf_values import CBFValuesComponent
from .search_heatmap import SearchHeatmapComponent
from .search_percentage import SearchPercentageComponent
from .heatmap import HeatmapComponent
from .energy import EnergyComponent
from .control_input import ControlInputComponent
from .cbc import CBCComponent
from .h_derivative import HDerivativeComponent
from .optimisation_vector import OptimizationVectorComponent

REGISTRIED_COMPONENTS = {
    'map': {
        'class': 'MapAnimationComponent',
        'filename': 'map',
        'figsize': (10, 10)
    },
    'opt-ct': {
        'title': 'Opt Result',
        'class': 'OptimizationContourPlot',
        'filename': 'opt',
    },
    'opt-vec': {
        'title': 'Opt Result Vector',
        'class': 'OptimizationVectorComponent',
        'filename': 'opt-vec'
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
    'sp': {
        'title': 'Search Percentage Over Time',
        'class': 'SearchPercentageComponent',
        'filename': 'search-percentage',
        'figsize': (10, 6),
        'params': {
            'show_milestones': False
        }
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
    'cbc': {
        'title': 'Control Barrier Certificate',
        'class': 'CBCComponent',
        'filename': 'cbc',
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
    'cbc-energy': {
        'title': 'Control Barrier Certificate, Energy CBF',
        'class': 'CBCComponent',
        'filename': 'cbc-energy',
        'figsize': (10, 6),
        'params': {
               'cbf_filter': 'energy'
        }
    },
    'dh-energy': {
        'class': 'HDerivativeComponent',
        'filename': 'dh',
        'figsize': (10, 6),
        'params': {
            'cbf_filter': 'energy'
        }
    },
    'cbf-comm': {
        'title': 'Communication CBF Value',
        'class': 'CBFValuesComponent',
        'filename': 'comm-cbf',
        'figsize': (10, 6),
        'params': {
            'cbf_filter': lambda name: 'comm' in name
        }
    },
    'cbf-comm-energy': {
        'title': 'Communication & Energy CBF Value',
        'class': 'CBFValuesComponent',
        'filename': 'comm-cbf',
        'figsize': (10, 6),
        'params': {
            'cbf_filter': lambda name: 'comm' in name or 'energy' in name
        }
    }
}
