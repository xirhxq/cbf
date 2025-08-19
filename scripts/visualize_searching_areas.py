import matplotlib.pyplot as plt
import numpy as np
import json
import math
import os

# Load configuration
config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'config.json')
with open(config_path, 'r') as f:
    config = json.load(f)

# Get world boundary for plotting context
world_boundary = np.array(config['world']['boundary'])

# Get searching parameters
searching_config = config['searching']
methods = ['downward', 'front-sector', 'front-cone']

# Define a sample UAV position and yaw for visualization
sample_position = np.array([0, 5])  # Example position
sample_yaw_deg = 90  # Example yaw in degrees

def plot_downward_search(ax, position, params):
    radius = params['radius']
    
    # Draw the world boundary for context
    ax.plot(np.append(world_boundary[:, 0], world_boundary[0, 0]), 
            np.append(world_boundary[:, 1], world_boundary[0, 1]), 'k-', linewidth=2, label='World Boundary')
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'bo', markersize=8, label='UAV')
    
    # Draw the search circle
    circle = plt.Circle(position, radius, color='blue', fill=False, linestyle='--', linewidth=2, label='Search Area')
    ax.add_patch(circle)
    
    # Set axis limits with padding
    padding = 3
    ax.set_xlim(position[0] - radius - padding, position[0] + radius + padding)
    ax.set_ylim(position[1] - radius - padding, position[1] + radius + padding)
    
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Downward Circular Search Area')
    ax.legend()

def plot_front_sector_search(ax, position, params, yaw_deg):
    inner_radius = params['inner-radius']
    outer_radius = params['outer-radius']
    half_angle_deg = params['half-angle-deg']
    yaw_rad = math.radians(yaw_deg)
    
    # Draw the world boundary for context
    ax.plot(np.append(world_boundary[:, 0], world_boundary[0, 0]), 
            np.append(world_boundary[:, 1], world_boundary[0, 1]), 'k-', linewidth=2, label='World Boundary')
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'bo', markersize=8, label='UAV')
    
    # Draw the sector
    # Define angles for the sector
    start_angle = math.degrees(yaw_rad) - half_angle_deg
    end_angle = math.degrees(yaw_rad) + half_angle_deg
    
    # Draw outer arc
    outer_arc_angles = np.linspace(start_angle, end_angle, 100)
    outer_arc_x = position[0] + outer_radius * np.cos(np.radians(outer_arc_angles))
    outer_arc_y = position[1] + outer_radius * np.sin(np.radians(outer_arc_angles))
    ax.plot(outer_arc_x, outer_arc_y, 'b-', linewidth=2)
    
    # Draw inner arc
    inner_arc_angles = np.linspace(start_angle, end_angle, 100)
    inner_arc_x = position[0] + inner_radius * np.cos(np.radians(inner_arc_angles))
    inner_arc_y = position[1] + inner_radius * np.sin(np.radians(inner_arc_angles))
    ax.plot(inner_arc_x, inner_arc_y, 'b-', linewidth=2)
    
    # Draw connecting lines
    start_angle_rad = math.radians(start_angle)
    end_angle_rad = math.radians(end_angle)
    ax.plot([position[0] + inner_radius * np.cos(start_angle_rad), 
             position[0] + outer_radius * np.cos(start_angle_rad)],
            [position[1] + inner_radius * np.sin(start_angle_rad), 
             position[1] + outer_radius * np.sin(start_angle_rad)], 'b-', linewidth=2)
    ax.plot([position[0] + inner_radius * np.cos(end_angle_rad), 
             position[0] + outer_radius * np.cos(end_angle_rad)],
            [position[1] + inner_radius * np.sin(end_angle_rad), 
             position[1] + outer_radius * np.sin(end_angle_rad)], 'b-', linewidth=2)
    
    # Set axis limits with padding
    padding = 3
    ax.set_xlim(position[0] - outer_radius - padding, position[0] + outer_radius + padding)
    ax.set_ylim(position[1] - outer_radius - padding, position[1] + outer_radius + padding)
    
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Front-Sector Search Area')
    ax.legend()

def plot_front_cone_search(ax, position, params, yaw_deg):
    height = params['height']
    downward_radius = params['downward-radius']
    camera_pitch_deg = params['camera-pitch-deg']
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(camera_pitch_deg)
    
    # Draw the world boundary for context
    ax.plot(np.append(world_boundary[:, 0], world_boundary[0, 0]), 
            np.append(world_boundary[:, 1], world_boundary[0, 1]), 'k-', linewidth=2, label='World Boundary')
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'bo', markersize=8, label='UAV')
    
    # Calculate cone parameters
    # Apex of the cone A: (position.x, position.y, height)
    # Projection of cone central line B:
    # (position.x + height / tan(pitchRad) * cos(yawRad), position.y + height / tan(pitchRad) * sin(yawRad), 0)
    if math.tan(pitch_rad) != 0:
        B_x = position[0] + height / math.tan(pitch_rad) * math.cos(yaw_rad)
        B_y = position[1] + height / math.tan(pitch_rad) * math.sin(yaw_rad)
    else:
        B_x = position[0]
        B_y = position[1]
    
    alpha = math.atan(downward_radius / height)
    
    # Draw the cone projection on the ground
    # We'll approximate it with a polygon
    # Calculate the angle range
    angle_range = math.degrees(alpha)
    
    # Calculate points on the cone base circle
    angles = np.linspace(yaw_deg - angle_range, yaw_deg + angle_range, 100)
    cone_x = []
    cone_y = []
    
    for angle in angles:
        angle_rad = math.radians(angle)
        # Distance from B to point on cone base
        # Using similar triangles
        # In the 2D projection, we simplify the cone to a sector
        # The radius of the base at ground level
        ground_radius = height * math.tan(alpha)
        x = B_x + ground_radius * math.cos(angle_rad)
        y = B_y + ground_radius * math.sin(angle_rad)
        cone_x.append(x)
        cone_y.append(y)
    
    # Close the polygon
    cone_x.append(cone_x[0])
    cone_y.append(cone_y[0])
    
    # Plot the cone area
    ax.fill(cone_x, cone_y, color='blue', alpha=0.3, edgecolor='blue', linewidth=2, label='Search Area')
    
    # Draw lines from UAV to the farthest points of the cone base
    ax.plot([position[0], B_x + ground_radius * math.cos(math.radians(yaw_deg - angle_range))],
            [position[1], B_y + ground_radius * math.sin(math.radians(yaw_deg - angle_range))], 'b--', linewidth=1)
    ax.plot([position[0], B_x + ground_radius * math.cos(math.radians(yaw_deg + angle_range))],
            [position[1], B_y + ground_radius * math.sin(math.radians(yaw_deg + angle_range))], 'b--', linewidth=1)
    
    # Draw the central line of the cone
    ax.plot([position[0], B_x], [position[1], B_y], 'b-', linewidth=2)
    
    # Add annotation for parameters
    annotation_x = B_x + ground_radius * math.cos(yaw_rad) + 1
    annotation_y = B_y + ground_radius * math.sin(yaw_rad) + 1

    # Set axis limits with padding
    padding = 3
    xlim_min = min(min(cone_x), position[0]) - padding
    xlim_max = max(max(cone_x), position[0]) + padding
    ylim_min = min(min(cone_y), position[1]) - padding
    ylim_max = max(max(cone_y), position[1]) + padding
    ax.set_xlim(xlim_min, xlim_max)
    ax.set_ylim(ylim_min, ylim_max)
    
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Front-Cone Search Area')
    ax.legend()

# Create figure with subplots
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

# Plot each searching method
plot_downward_search(axes[0], sample_position, searching_config['downward'])
plot_front_sector_search(axes[1], sample_position, searching_config['front-sector'], sample_yaw_deg)
plot_front_cone_search(axes[2], sample_position, searching_config['front-cone'], sample_yaw_deg)

# Adjust layout
plt.tight_layout()

# Save the figure
output_path = os.path.join(os.path.dirname(__file__), '..', 'plot', 'searching_areas_visualization.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')

# Show the plot
plt.show()

print(f"Visualization saved to {output_path}")