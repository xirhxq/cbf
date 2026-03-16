from utils import *
from .base import BaseComponent
from .comparison_search_percentage import search_percentage_interpreter
import os
import json
from pathlib import Path


class MonteCarloSearchPercentageComponent(BaseComponent):
    """
    Component for comparing search percentage curves across Monte Carlo runs.
    Dynamically loads all runs from the monte_carlo data folder.
    """
    def __init__(self, ax, data, **kwargs):
        """
        Args:
            ax: matplotlib axis
            data: current data folder path (used to resolve relative paths)
        """
        self.ax = ax
        self.params = kwargs.get('params', {})

        # Get data folder
        self.data_folder = self.params.get('data_folder', '../../data/monte_carlo')

        # Configuration
        self.show_milestones = self.params.get('show_milestones', True)
        self.milestones = self.params.get('milestones', [0.25, 0.5, 0.75, 1.0])

        # Process all Monte Carlo runs
        self.processed_data = {}
        self._load_and_process_data()

        self._initialize_plot()

    def _load_and_process_data(self):
        """Load and process data from all Monte Carlo runs"""
        # Find all run directories
        run_dirs = sorted(Path(self.data_folder).glob('run_*'))

        for run_dir in run_dirs:
            # Find timestamp subdirectory
            timestamp_dirs = list(run_dir.glob('20*'))
            if not timestamp_dirs:
                continue

            data_file = timestamp_dirs[0] / 'data.json'
            if not data_file.exists():
                continue

            try:
                with open(data_file, 'r') as f:
                    data = json.load(f)
                processed = search_percentage_interpreter(data)

                run_id = run_dir.name.split('_')[1]
                final_pct = processed['search_percentages'][-1]

                self.processed_data[f'Run {run_id}'] = {
                    'runtime': processed['runtime'],
                    'percentages': processed['search_percentages'],
                    'final_pct': final_pct
                }
                print(f"Loaded Run {run_id}: {final_pct*100:.1f}% coverage")
            except Exception as e:
                print(f"Error loading {run_dir}: {e}")

    def _initialize_plot(self):
        """Initialize the comparison plot"""
        # Separate completed and failed runs
        completed = []
        failed = []

        for method_name, data in self.processed_data.items():
            final_pct = data['final_pct']
            if final_pct >= 0.999:
                completed.append((method_name, data))
            else:
                failed.append((method_name, data))

        # Plot all runs with single color, no individual labels
        for method_name, data in completed:
            runtime = np.array(data['runtime'])
            percentages = np.array(data['percentages'])
            self.ax.plot(runtime, percentages, color='blue', linewidth=1, alpha=0.6)

        for method_name, data in failed:
            runtime = np.array(data['runtime'])
            percentages = np.array(data['percentages'])
            self.ax.plot(runtime, percentages, color='red', linewidth=1, alpha=0.5)

        # Calculate and plot mean curve
        if completed:
            # Interpolate all curves to common time points
            max_time = max(max(data['runtime']) for _, data in completed)
            common_time = np.linspace(0, max_time, 500)
            interpolated = []
            for _, data in completed:
                runtime = np.array(data['runtime'])
                percentages = np.array(data['percentages'])
                interp = np.interp(common_time, runtime, percentages)
                interpolated.append(interp)

            mean_curve = np.mean(interpolated, axis=0)
            std_curve = np.std(interpolated, axis=0)

            # Plot mean with std band
            self.ax.plot(common_time, mean_curve, color='blue', linewidth=2.5,
                        label=f'Mean (N={len(completed)})')
            self.ax.fill_between(common_time,
                                mean_curve - std_curve,
                                mean_curve + std_curve,
                                color='blue', alpha=0.2, label='±1 Std')

        # Add milestone lines
        if self.show_milestones:
            for milestone in self.milestones:
                self.ax.axhline(y=milestone, color='gray', linestyle=':',
                               alpha=0.5, linewidth=1)

        # Labels and styling
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Search Percentage (%)')
        self.ax.set_title(f'Monte Carlo Search Coverage (N={len(self.processed_data)}, {len(completed)} completed)')

        # Format y-axis as percentage
        self.ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda y, _: f'{int(y*100)}%'))

        # Set limits - auto-detect from data
        max_time = max(max(data['runtime']) for _, data in self.processed_data.items()) if self.processed_data else 500
        self.ax.set_xlim([0, max_time * 1.05])
        self.ax.set_ylim([0, 1.05])
        self.ax.grid(True, alpha=0.3)

        # Add legend
        self.ax.legend(loc='lower right', fontsize=9)

        # Add summary text
        summary_text = f'Success: {len(completed)}/{len(self.processed_data)}'
        self.ax.text(0.02, 0.98, summary_text,
                    transform=self.ax.transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    fontsize=10)

    def update(self, frame):
        """Animation update (not used for static comparison plots)"""
        pass
