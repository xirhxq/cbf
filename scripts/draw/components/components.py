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
from .centralized_cbf_value import CentralizedCBFValueComponent
from .commcbf_uncertainty import CommCBFUncertaintyComponent
from .optimization_failure import OptimizationFailureComponent
from .cvt_center_density import CVTCenterDensityComponent
from .line_covariance_magnitude import LineCovarianceMagnitude
from .cbf_analysis import ConstraintCBFsComponent, TaskCBFsComponent, SlackVarsComponent
from .uncertainty_heatmap import UncertaintyHeatmapComponent
from .formation_distance import FormationDistanceComponent
from .h_loc_constraints import HLocComponent
from .h_safe_constraints import HSafeComponent
from .comparison_search_percentage import ComparisonSearchPercentageComponent
from .valid_links import ValidLinksComponent
from .monte_carlo_sp import MonteCarloSearchPercentageComponent

REGISTRIED_COMPONENTS = {
    'map': {
        'class': 'MapAnimationComponent',
        'filename': 'map',
        'figsize': (10, 10),
        'params': {
            '//colormap': ['coolwarm', 'christmas'],
            'colormap': 'coolwarm',
            'show_charge': False,
            # 'show_time_title': False,
            'show_cov_ellipse': False,
            'show_cov_text': False,
            'annotation_font_size': 12,
            # 'shotList': [0.5, 100, 200, 300, 400, 500]
        }
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
        'figsize': (7, 4),
        'params': {
            'show_milestones': True,
            'milestones': [0.25, 0.5, 0.75, 1.0],
            '//show_max_text': [True, False],
            'show_max_text': False,
        }
    },
    'sp-anim': {
        'title': 'Search Percentage Over Time',
        'class': 'SearchPercentageComponent',
        'filename': 'search-percentage-anim',
        'figsize': (7, 4),
        'expand': False,
        'params': {
            'show_milestones': False,
            'show_max_text': False,
        },
        'show_legend': False,
        'show_markers': True,
        'show_value_text': True,
        'marker_style': {'marker': 'o', 'color': 'red', 'markersize': 8},
        'text_style': {'color': 'red', 'fontsize': 12, 'bbox': {'facecolor': 'white', 'alpha': 0.7, 'edgecolor': 'none'}},
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
    },
    'centralized-cbf': {
        'title': 'Centralized CBF Values',
        'class': 'CentralizedCBFValueComponent',
        'filename': 'centralized-cbf',
        'figsize': (12, 8),
        'params': {
            'cbf_filter': 'all'
        }
    },
    'centralized-comm': {
        'title': 'Centralized Communication CBF Values',
        'class': 'CentralizedCBFValueComponent',
        'filename': 'centralized-comm',
        'figsize': (12, 8),
        'params': {
            'cbf_filter': 'comm',
            'show_min': True
        }
    },
    'centralized-cvt': {
        'title': 'Centralized CVT CBF Values',
        'class': 'CentralizedCBFValueComponent',
        'filename': 'centralized-cvt',
        'figsize': (12, 8),
        'params': {
            'cbf_filter': 'cvt'
        }
    },
    'comm-uncertainty': {
        'title': 'CommCBF Distance + Uncertainty',
        'class': 'CommCBFUncertaintyComponent',
        'filename': 'comm-uncertainty',
        'figsize': (12, 8),
        'params': {
            'cbf_filter': 'all',
            'uncertainty_mode': 'stacked'
        }
    },
    'comm-uncertainty-maxrange': {
        'title': 'CommCBF Distance + Uncertainty (From Max Range)',
        'class': 'CommCBFUncertaintyComponent',
        'filename': 'comm-uncertainty-maxrange',
        'figsize': (12, 8),
        'params': {
            'cbf_filter': 'all',
            'uncertainty_mode': 'from_max_range'
        }
    },
    'optimization-failure': {
        'title': 'Optimization Failure Timeline',
        'class': 'OptimizationFailureComponent',
        'filename': 'optimization-failure',
        'figsize': (12, 6)
    },
    'cvt-center-density': {
        'title': 'CVT Center Density Over Time',
        'class': 'CVTCenterDensityComponent',
        'filename': 'cvt-center-density',
        'figsize': (12, 8)
    },
    'position-uncertainty': {
        'title': 'Position Uncertainty Evolution',
        'class': 'LineCovarianceMagnitude',
        'filename': 'position-uncertainty',
        'figsize': (7, 4),
        'params': {
            'yscale': 'linear',
        }
    },
    'uncertainty-heatmap': {
        'title': 'Position Uncertainty Distribution',
        'class': 'UncertaintyHeatmapComponent',
        'filename': 'uncertainty-heatmap',
        'figsize': (8, 8),
        'params': {
            'time_frame': 'final',
            'show_trajectories': True,
            'cmap': 'Greys',
        }
    },
    'h_loc': {
        'title': 'Localization Constraints (h_loc)',
        'class': 'HLocComponent',
        'filename': 'h_loc',
        'figsize': (7, 4),
    },
    'h_safe': {
        'title': 'Safety Constraints (h_safe)',
        'class': 'HSafeComponent',
        'filename': 'h_safe',
        'figsize': (7, 4),
    },
    'constraint_cbfs': {
        'title': 'Constraint CBFs',
        'class': 'ConstraintCBFsComponent',
        'filename': 'constraint_cbfs',
        'figsize': (7, 3.5),
    },
    'task_cbfs': {
        'title': 'Task CBFs',
        'class': 'TaskCBFsComponent',
        'filename': 'task_cbfs',
        'figsize': (7, 3.5),
    },
    'slack_vars': {
        'title': 'Slack Variables',
        'class': 'SlackVarsComponent',
        'filename': 'slack_vars',
        'figsize': (7, 3.5),
    },
    'comparison-sp': {
        'title': 'Search Performance Comparison',
        'class': 'ComparisonSearchPercentageComponent',
        'filename': 'comparison-search-percentage',
        'figsize': (7, 4.2),
        'params': {
            'comparison_data': {
                'Ours (Robust CBF)': '../../data/2026-01-28_14-47-22',
                'CVT+Formation': '../../data/2025-12-24_03-46-37',
                'Conventional CBF': '../../data/2026-01-28_21-05-40',
                'Pure CVT': '../../data/2026-01-28_21-27-24',
            },
            'method_colors': ['blue', 'orange', 'red', 'green'],
            'show_violation_markers': True,
        }
    },
    'valid-links': {
        'title': 'Valid Links (Safe Distance Range)',
        'class': 'ValidLinksComponent',
        'filename': 'valid-links',
        'figsize': (10, 6),
    },
    'mbzirc-comparison-sp': {
        'title': 'MBZIRC Simulation vs Numerical Simulation',
        'class': 'ComparisonSearchPercentageComponent',
        'filename': 'mbzirc-comparison-search-percentage',
        'figsize': (7, 4.2),
        'params': {
            'comparison_data': {
                'MBZIRC Simulation': '../../data/2026-03-07_00-42-23',
                'Numerical Simulation': '../../data/2026-01-28_14-47-22',
            },
            'method_colors': ['blue', 'red'],
            'show_violation_markers': True,
            'extend_to_time': 500,  # Extend shorter simulations to 500s
        }
    },
    'monte-carlo-sp': {
        'title': 'Monte Carlo Search Percentage Comparison',
        'class': 'MonteCarloSearchPercentageComponent',
        'filename': 'monte_carlo_sp',
        'figsize': (10, 6),
        'params': {
            'data_folder': '../../data/monte_carlo',
            'show_milestones': True,
        }
    }
}
