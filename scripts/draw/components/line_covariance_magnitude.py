import numpy as np
from .lines import Lines


class LineCovarianceMagnitude(Lines):
    def __init__(self, ax, data, **kwargs):
        # Parameters are nested under 'params' key in the component system
        params = kwargs.get('params', {})
        self.uncertainty_type = params.get('uncertainty_type', 'max_eigenvalue')  # 'std_avg' or 'max_eigenvalue'
        self.yscale = params.get('yscale', 'linear')  # 'linear' or 'log'
        self.static_comparison = params.get('static_comparison', True)  # If True, show single frame comparison instead of time series
        self.show_value_text = params.get('show_value_text', True)

        # Override default settings for this specific component
        self.line_style = kwargs.get('line_style', {
            'linewidth': 2, 'alpha': 0.8, 'marker': 'o', 'markersize': 6
        })
        self.text_style = kwargs.get('text_style', {
            'color': 'red', 'alpha': 0.8, 'fontsize': 8,
            'bbox': {'facecolor': 'white', 'alpha': 0.7, 'edgecolor': 'none'}
        })
        self.show_zero_line = False
        self.show_markers = True

        # Call parent constructor AFTER setting attributes
        super().__init__(ax, data, **kwargs)

    def _default_data_processor(self):
        """Process data to extract uncertainty directly from data field"""
        uncertainty_data = {}

        for robot_id in self.robot_ids:
            uncertainty_values = []
            timestamps = []

            for frame in self.data["state"]:
                if robot_id < len(frame["robots"]) and "uncertainty" in frame["robots"][robot_id]:
                    # Read uncertainty directly from data (already 3σ semi-major axis)
                    uncertainty = frame["robots"][robot_id]["uncertainty"]
                    uncertainty_values.append(uncertainty)
                    timestamps.append(frame["runtime"])

            if uncertainty_values:
                uncertainty_data[f'robot_{robot_id + 1}'] = {
                    'runtime': timestamps,
                    'value': uncertainty_values
                }

        uncertainty_data['runtime'] = timestamps if timestamps else []
        return uncertainty_data

    def _get_line_data(self):
        """Extract line data for plotting"""
        line_data_list = []

        # Always show time series: each robot has one line over time
        if isinstance(self.processed_data, dict):
            for key, data in self.processed_data.items():
                if key.startswith('robot_') and 'runtime' in data and 'value' in data:
                    robot_label = key.replace('robot_', 'Robot #')
                    line_data_list.append({
                        'label': robot_label,
                        'x': data['runtime'],
                        'y': data['value'],
                        'style': {}
                    })

        return line_data_list

    def _initialize_plot(self):
        """Initialize plot with appropriate labels and styling"""
        super()._initialize_plot()

        # Set labels for time series plot
        self.ax.set_xlabel('Time (s)')
        # Position uncertainty: 3σ semi-major axis (ε_i = 3√(λ_max(Σ)))
        self.ax.set_ylabel('Position Uncertainty\n$\\epsilon_i = 3\\sqrt{\\lambda_{\\max}(\\mathbf{\\Sigma}_i)}$ (m)')
        self.ax.set_title('Position Uncertainty Evolution')

        # Set appropriate scale
        if self.yscale == 'log':
            self.ax.set_yscale('log')

        # Add grid
        self.ax.grid(True, alpha=0.3)

    def _apply_custom_styling(self):
        """Apply custom styling for the uncertainty plot"""
        # No special styling needed for time series plot
        pass