#!/usr/bin/env python3
"""
Script to draw figures for CBF2026 paper.

This script generates publication-quality figures for the paper, including:
- Front-sector sensing model illustration (2D and 3D, clean without annotations)

Usage:
    .venv/bin/python scripts/visualize_plot/draw_paper_figures.py
"""

import matplotlib.pyplot as plt
import numpy as np
import math
import os
from matplotlib import rcParams
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Set publication-quality figure parameters
rcParams.update({
    'font.size': 12,
    'font.family': 'serif',
    'font.serif': ['Times New Roman'],
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.format': 'png',
    'savefig.bbox': 'tight',
})

# Output directory
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot')

# Parameters from config.json: r_inner=0, r_outer=400, theta_hfov=120 (half_angle=60)
SAMPLE_YAW_DEG = 90
SAMPLE_POSITION_2D = np.array([0, 0])
SECTOR_COLOR = 'royalblue'

# Actual sensing parameters (meters)
INNER_RADIUS_M = 0.0
OUTER_RADIUS_M = 400.0
UAV_HEIGHT_M = 50.0
HALF_ANGLE_DEG = 60.0  # theta_hfov = 90 degrees

# Scale factor for visualization (to fit in figure)
SCALE = 1.0 / 150.0  # Scale 400m to ~2.67 units

# Scaled positions
SAMPLE_POSITION_3D = np.array([0, 0, UAV_HEIGHT_M * SCALE])

# View parameters from visualize_searching_areas.py
# Modified 3D view: rotate 180 degrees horizontally (azim from -180 to 0)
VIEW_PARAMS = {
    'front-sector': {'elev': 15, 'azim': 0},
    'zoom': 1.5
}

# Axis limits from visualize_searching_areas.py
AXIS_LIMITS = {
    'xlim': (-4, 4),
    'ylim': (-4, 4)
}


def draw_front_sector_sensor_2d():
    """
    Draw clean 2D front-sector sensing model (no annotations).
    """
    fig, ax = plt.subplots(figsize=(4, 3))

    # Scaled parameters for visualization
    inner_radius = INNER_RADIUS_M * SCALE
    outer_radius = OUTER_RADIUS_M * SCALE
    half_angle_deg = HALF_ANGLE_DEG

    # Calculate sector angles
    start_angle = math.radians(SAMPLE_YAW_DEG - half_angle_deg)
    end_angle = math.radians(SAMPLE_YAW_DEG + half_angle_deg)

    # Draw the UAV
    ax.plot(SAMPLE_POSITION_2D[0], SAMPLE_POSITION_2D[1], 'o',
            color=SECTOR_COLOR, markersize=8)

    # Draw the sector (filled)
    n_points = 100
    angles = np.linspace(start_angle, end_angle, n_points)

    inner_x = SAMPLE_POSITION_2D[0] + inner_radius * np.cos(angles)
    inner_y = SAMPLE_POSITION_2D[1] + inner_radius * np.sin(angles)
    outer_x = SAMPLE_POSITION_2D[0] + outer_radius * np.cos(angles[::-1])
    outer_y = SAMPLE_POSITION_2D[1] + outer_radius * np.sin(angles[::-1])

    # Combine to form sector polygon
    sector_x = np.concatenate([inner_x, outer_x, [inner_x[0]]])
    sector_y = np.concatenate([inner_y, outer_y, [inner_y[0]]])

    # Plot filled sector
    ax.fill(sector_x, sector_y, color=SECTOR_COLOR, alpha=0.3)

    # Set axis limits - adjust these values to change the visible region
    # Format: ax.set_xlim(x_min, x_max), ax.set_ylim(y_min, y_max)
    # UAV is at (0, 0), facing upward (yaw=90°)
    ax.set_xlim(-outer_radius, outer_radius)
    ax.set_ylim(-outer_radius * 0.2, outer_radius * 1.2)
    ax.set_aspect('equal')
    ax.axis('off')

    # Save figure with fixed size (no tight bbox to ensure consistent height)
    output_path = os.path.join(OUTPUT_DIR, 'front-sector-sensor.png')
    plt.savefig(output_path, dpi=300)
    print(f"Front-sector sensor 2D figure saved to: {output_path}")

    plt.close()


def draw_front_sector_sensor_3d():
    """
    Draw clean 3D front-sector sensing model (no annotations).
    Uses the same view angle as visualize_searching_areas.py.
    """
    fig = plt.figure(figsize=(4, 3))
    ax = fig.add_subplot(111, projection='3d')

    # Scaled parameters for visualization
    inner_radius = INNER_RADIUS_M * SCALE
    outer_radius = OUTER_RADIUS_M * SCALE
    half_angle_deg = HALF_ANGLE_DEG

    # Calculate sector angles
    start_angle = math.radians(SAMPLE_YAW_DEG - half_angle_deg)
    end_angle = math.radians(SAMPLE_YAW_DEG + half_angle_deg)

    # Draw the UAV
    ax.scatter(SAMPLE_POSITION_3D[0], SAMPLE_POSITION_3D[1], SAMPLE_POSITION_3D[2],
               c=SECTOR_COLOR, s=50)

    # Draw vertical line from UAV to ground
    ax.plot([SAMPLE_POSITION_3D[0], SAMPLE_POSITION_3D[0]],
            [SAMPLE_POSITION_3D[1], SAMPLE_POSITION_3D[1]],
            [SAMPLE_POSITION_3D[2], 0], 'k--', linewidth=1)

    # Create vertices for the 3D sector
    n_points = 100
    angles = np.linspace(start_angle, end_angle, n_points)

    # Points for inner and outer arcs at ground level
    inner_x = SAMPLE_POSITION_3D[0] + inner_radius * np.cos(angles)
    inner_y = SAMPLE_POSITION_3D[1] + inner_radius * np.sin(angles)

    outer_x = SAMPLE_POSITION_3D[0] + outer_radius * np.cos(angles)
    outer_y = SAMPLE_POSITION_3D[1] + outer_radius * np.sin(angles)

    # Create vertices for the Poly3DCollection
    verts = []

    # Connect UAV to inner arc
    for i in range(n_points - 1):
        verts.append([
            (SAMPLE_POSITION_3D[0], SAMPLE_POSITION_3D[1], SAMPLE_POSITION_3D[2]),
            (inner_x[i], inner_y[i], 0),
            (inner_x[i+1], inner_y[i+1], 0)
        ])

    # Connect UAV to outer arc
    for i in range(n_points - 1):
        verts.append([
            (SAMPLE_POSITION_3D[0], SAMPLE_POSITION_3D[1], SAMPLE_POSITION_3D[2]),
            (outer_x[i], outer_y[i], 0),
            (outer_x[i+1], outer_y[i+1], 0)
        ])

    # Connect UAV to the start and end points to close the sides
    verts.append([
        (SAMPLE_POSITION_3D[0], SAMPLE_POSITION_3D[1], SAMPLE_POSITION_3D[2]),
        (inner_x[0], inner_y[0], 0),
        (outer_x[0], outer_y[0], 0)
    ])

    verts.append([
        (SAMPLE_POSITION_3D[0], SAMPLE_POSITION_3D[1], SAMPLE_POSITION_3D[2]),
        (inner_x[-1], inner_y[-1], 0),
        (outer_x[-1], outer_y[-1], 0)
    ])

    # Create the Poly3DCollection
    poly3d = Poly3DCollection(verts, alpha=0.3, facecolor=SECTOR_COLOR, edgecolor='none')
    ax.add_collection3d(poly3d)

    # Set axis limits - adjust these values to change the visible region
    # Format: ax.set_xlim(x_min, x_max), ax.set_ylim(y_min, y_max), ax.set_zlim(z_min, z_max)
    # UAV is at (0, 0, 0.5), facing upward (yaw=90°)
    ax.set_xlim(-outer_radius * 0.2, outer_radius * 1.2)
    ax.set_ylim(-outer_radius * 0.2, outer_radius * 1.2)
    ax.set_zlim(0, SAMPLE_POSITION_3D[2] * 1.2)

    ax.set_box_aspect([1, 1, 0.2])

    # Remove axis
    ax.set_axis_off()

    # Set view angle (rotated 180 degrees horizontally)
    ax.view_init(elev=VIEW_PARAMS['front-sector']['elev'],
                 azim=VIEW_PARAMS['front-sector']['azim'])
    ax.set_box_aspect(None, zoom=VIEW_PARAMS['zoom'])

    # Save figure with fixed size (no tight bbox to ensure consistent height)
    output_path = os.path.join(OUTPUT_DIR, 'front-sector-sensor-3d.png')
    plt.savefig(output_path, dpi=300)
    print(f"Front-sector sensor 3D figure saved to: {output_path}")

    plt.close()


if __name__ == '__main__':
    # Create output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("Generating clean figures for CBF2026 paper...")
    print(f"Output directory: {OUTPUT_DIR}")
    print()

    # Draw 2D front-sector sensor model (clean, no annotations)
    print("Drawing front-sector sensor model (2D, clean)...")
    draw_front_sector_sensor_2d()
    print()

    # Draw 3D front-sector sensor model (clean, no annotations)
    print("Drawing front-sector sensor model (3D, clean)...")
    draw_front_sector_sensor_3d()
    print()

    print("All figures generated successfully!")
