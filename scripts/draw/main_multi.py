#!/usr/bin/env python3
"""
Main script for plotting parametric study results.
This is separate from the main visualization framework and is specifically designed
for comparing multiple simulation runs with different parameter values.
"""

import json
import matplotlib.pyplot as plt
import os
import sys
from pathlib import Path


def load_summary(folder_path):
    """Load summary.json from parametric study folder."""
    summary_path = os.path.join(folder_path, 'summary.json')
    with open(summary_path, 'r') as f:
        return json.load(f)['parametric_study']


def format_parameter_name(param_name):
    """Convert parameter path to display name."""
    if 'outer-radius' in param_name:
        return 'Detection Radius $r_{det}$ (m)'
    elif 'max-range' in param_name:
        return 'Max Range (m)'
    elif 'half-angle-deg' in param_name:
        return 'Horizontal FOV Angle $\\theta_{hfov}$ (deg)'
    else:
        parts = param_name.split('.')
        return ' '.join(p.capitalize() for p in parts[-2:])


def load_search_percentage_curve(data_folder_path):
    """Load search percentage time series from a single run's data.json."""
    data_file = os.path.join(data_folder_path, 'data.json')
    with open(data_file, 'r') as f:
        data = json.load(f)

    state = data['state']
    para = data['para']
    grid_world = para['gridWorld']
    x_num = grid_world['xNum']
    y_num = grid_world['yNum']
    total_cells = x_num * y_num

    searched_cells = set()
    runtimes = []
    percentages = []

    for frame in state:
        if "update" in frame and len(frame["update"]) > 0:
            for grid in frame["update"]:
                cell_id = (grid[0], grid[1])
                searched_cells.add(cell_id)

        coverage = len(searched_cells) / total_cells * 100 if total_cells > 0 else 0
        runtimes.append(frame['runtime'])
        percentages.append(coverage)

    return runtimes, percentages


def plot_parametric_comparison(folder_path, save_path=None):
    """
    Plot parametric study comparison: search percentage curves for different parameter values.

    Args:
        folder_path: Path to parametric study folder containing summary.json
        save_path: Optional path to save the figure
    """
    # Load summary
    summary = load_summary(folder_path)
    param_name = summary['parameter_name']
    param_display = format_parameter_name(param_name)

    # Setup plot
    fig, ax = plt.subplots(figsize=(5, 3))

    # Colors and linestyles for different runs
    colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
    linestyles = ['-', '--', '-.', ':', '-.', '--']

    # Plot each run
    for i, result in enumerate(summary['results']):
        param_value = result['parameter_value']
        data_folder = result['data_folder']

        # Load time series data
        runtimes, percentages = load_search_percentage_curve(
            os.path.join(folder_path, data_folder)
        )

        # Plot curve
        color = colors[i % len(colors)]
        linestyle = linestyles[i % len(linestyles)]

        # For half-angle, display as full FOV angle (2x the half-angle)
        if 'half-angle-deg' in param_name:
            display_value = param_value * 2
        else:
            display_value = param_value

        label = f'{display_value}'
        ax.plot(runtimes, percentages, color=color, linestyle=linestyle,
               linewidth=2, label=label)

    # Set title based on parameter type
    if 'half-angle-deg' in param_name:
        title = 'Search Performance vs. Horizontal FOV Angle'
    elif 'outer-radius' in param_name:
        title = 'Search Performance vs. Detection Radius'
    elif 'max-range' in param_name:
        title = 'Search Performance vs. Communication Range'
    else:
        title = 'Search Performance Comparison'

    # Labels and styling
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Search Coverage (%)', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.legend(loc='lower right', fontsize=10)

    ax.set_xlim([0, 350])  # Up to 350 seconds
    ax.set_ylim([0, 105])

    plt.tight_layout()

    # Save figure
    if save_path is None:
        save_path = os.path.join(folder_path, 'parametric_search_comparison.png')

    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Plot saved to: {save_path}")

    # Close figure without showing
    plt.close(fig)

    return fig


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(description='Plot parametric study results')
    parser.add_argument('folder', type=str,
                       help='Path to parametric study folder (e.g., ../data_multi/2026-01-28_23-01-49)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output file path (default: save to folder/parametric_search_comparison.png)')

    args = parser.parse_args()

    # Convert to absolute path
    folder_path = Path(args.folder).resolve()

    if not folder_path.exists():
        print(f"Error: Folder does not exist: {folder_path}")
        return 1

    # Check for summary.json
    summary_path = folder_path / 'summary.json'
    if not summary_path.exists():
        print(f"Error: summary.json not found in {folder_path}")
        return 1

    # Create plot
    plot_parametric_comparison(str(folder_path), args.output)

    return 0


if __name__ == '__main__':
    sys.exit(main())
