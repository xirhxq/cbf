from .map_animation import MapAnimationComponent
from .optimisation_contour import OptimizationContourPlot
from .fixed_comm_range import FixedCommRangeComponent
from .cbf_values import CBFValuesComponent


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
    }
}