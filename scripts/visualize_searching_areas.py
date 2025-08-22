import matplotlib.pyplot as plt
import numpy as np
import json
import math
import os
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches

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
sample_position = np.array([0, 0, 10])  # Updated UAV position as per your request
sample_yaw_deg = 90  # Example yaw in degrees

def plot_downward_search_2d(ax, position, params):
    """Plot 2D view of downward circular search area"""
    radius = params['radius']
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'bo', markersize=8)
    
    # Draw the search circle (semi-transparent)
    circle = plt.Circle((position[0], position[1]), radius, color='blue', alpha=0.3)
    ax.add_patch(circle)
    
    # Set axis limits with padding
    padding = 3
    ax.set_xlim(position[0] - radius - padding, position[0] + radius + padding)
    ax.set_ylim(position[1] - radius - padding, position[1] + radius + padding)
    
    ax.set_aspect('equal')
    ax.set_title('downward, 2D view')

def plot_downward_search_3d(ax, position, params):
    """Plot 3D view of downward circular search area using Poly3DCollection"""
    radius = params['radius']
    
    # Draw the UAV as a point
    ax.scatter(position[0], position[1], position[2], c='blue', s=50)
    
    # Draw vertical line from UAV to ground
    ax.plot([position[0], position[0]], [position[1], position[1]], [position[2], 0], 'k--', linewidth=1)
    
    # Create a 3D cone (light cone effect) using Poly3DCollection
    # Number of points around the base
    n_points = 100
    theta = np.linspace(0, 2 * np.pi, n_points)
    
    # Points on the base circle (at ground level z=0)
    base_x = position[0] + radius * np.cos(theta)
    base_y = position[1] + radius * np.sin(theta)
    base_z = np.full_like(base_x, 0)
    
    # Apex point (UAV position)
    apex_x, apex_y, apex_z = position
    
    # Create triangles for the cone surface
    verts = []
    for i in range(n_points):
        next_i = (i + 1) % n_points
        verts.append([(apex_x, apex_y, apex_z),
                      (base_x[i], base_y[i], base_z[i]),
                      (base_x[next_i], base_y[next_i], base_z[next_i])])

    # Create the Poly3DCollection for the cone
    poly3d = Poly3DCollection(verts, alpha=0.3, facecolor='blue', edgecolor='none')
    ax.add_collection3d(poly3d)
    
    # Set axis limits with padding
    padding = 3
    ax.set_xlim(position[0] - radius - padding, position[0] + radius + padding)
    ax.set_ylim(position[1] - radius - padding, position[1] + radius + padding)
    ax.set_zlim(0 - padding, position[2] + padding)
    
    # Remove axis
    ax.set_axis_off()
    
    # Set view angle for a better perspective of the light cone
    ax.view_init(elev=20, azim=-70)
    
    ax.set_title('downward, 3D view')

def plot_front_sector_search_2d(ax, position, params, yaw_deg):
    """Plot 2D view of front-sector search area"""
    inner_radius = params['inner-radius']
    outer_radius = params['outer-radius']
    half_angle_deg = params['half-angle-deg']
    yaw_rad = math.radians(yaw_deg)
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'bo', markersize=8)
    
    # Draw the sector (semi-transparent)
    # Define angles for the sector
    start_angle = math.radians(yaw_deg - half_angle_deg)
    end_angle = math.radians(yaw_deg + half_angle_deg)
    
    # Create points for the sector polygon
    angles = np.linspace(start_angle, end_angle, 100)
    inner_x = position[0] + inner_radius * np.cos(angles)
    inner_y = position[1] + inner_radius * np.sin(angles)
    outer_x = position[0] + outer_radius * np.cos(angles[::-1])
    outer_y = position[1] + outer_radius * np.sin(angles[::-1])
    
    # Combine points to form a polygon
    sector_x = np.concatenate([inner_x, outer_x, [inner_x[0]]])
    sector_y = np.concatenate([inner_y, outer_y, [inner_y[0]]])
    
    # Plot the filled polygon
    ax.fill(sector_x, sector_y, color='blue', alpha=0.3)
    
    # Set axis limits with padding
    padding = 3
    ax.set_xlim(position[0] - outer_radius - padding, position[0] + outer_radius + padding)
    ax.set_ylim(position[1] - outer_radius - padding, position[1] + outer_radius + padding)
    
    ax.set_aspect('equal')
    ax.set_title('front-sector, 2D view')

def plot_front_sector_search_3d(ax, position, params, yaw_deg):
    """Plot 3D view of front-sector search area using Poly3DCollection"""
    inner_radius = params['inner-radius']
    outer_radius = params['outer-radius']
    half_angle_deg = params['half-angle-deg']
    
    # Draw the UAV as a point
    ax.scatter(position[0], position[1], position[2], c='blue', s=50)
    
    # Draw vertical line from UAV to ground
    ax.plot([position[0], position[0]], [position[1], position[1]], [position[2], 0], 'k--', linewidth=1)
    
    # Calculate angles for the sector with a small number of points for a smoother shape
    start_angle = math.radians(yaw_deg - half_angle_deg)
    end_angle = math.radians(yaw_deg + half_angle_deg)
    
    # Create vertices for the Poly3DCollection with a small number of points (8 points for smoother edges)
    n_points = 8
    angles = np.linspace(start_angle, end_angle, n_points)
    
    # Points for the inner and outer arcs at ground level
    inner_x = position[0] + inner_radius * np.cos(angles)
    inner_y = position[1] + inner_radius * np.sin(angles)
    
    outer_x = position[0] + outer_radius * np.cos(angles)
    outer_y = position[1] + outer_radius * np.sin(angles)
    
    # Create vertices for the Poly3DCollection (correct cone-like sides)
    verts = []
    
    # Create triangular faces from UAV to all points on the boundary of the sector
    # Connect UAV to inner arc points (forming the inner side of the cone)
    for i in range(n_points - 1):
        verts.append([
            (position[0], position[1], position[2]),  # UAV position
            (inner_x[i], inner_y[i], 0),
            (inner_x[i+1], inner_y[i+1], 0)
        ])
    
    # Connect UAV to outer arc points (forming the outer side of the cone)
    for i in range(n_points - 1):
        verts.append([
            (position[0], position[1], position[2]),  # UAV position
            (outer_x[i], outer_y[i], 0),
            (outer_x[i+1], outer_y[i+1], 0)
        ])
    
    # Connect UAV to the start and end points to close the sides
    # Inner side closing triangle
    verts.append([
        (position[0], position[1], position[2]),  # UAV position
        (inner_x[0], inner_y[0], 0),
        (outer_x[0], outer_y[0], 0)
    ])
    
    # Outer side closing triangle
    verts.append([
        (position[0], position[1], position[2]),  # UAV position
        (inner_x[-1], inner_y[-1], 0),
        (outer_x[-1], outer_y[-1], 0)
    ])
    
    # Create the Poly3DCollection for the sector with correct transparency
    poly3d = Poly3DCollection(verts, alpha=0.3, facecolor='blue', edgecolor='none')
    ax.add_collection3d(poly3d)
    
    # Set axis limits with padding
    padding = 3
    max_radius = max(inner_radius, outer_radius)
    ax.set_xlim(position[0] - max_radius - padding, position[0] + max_radius + padding)
    ax.set_ylim(position[1] - max_radius - padding, position[1] + max_radius + padding)
    ax.set_zlim(0 - padding, position[2] + padding)
    
    # Remove axis
    ax.set_axis_off()
    
    # Set view angle to show from right-front of UAV
    ax.view_init(elev=15, azim=-(yaw_deg + 90))  # View from right-front direction
    
    ax.set_title('front-sector, 3D view')

def plot_front_cone_search_2d(ax, position_3d, params, yaw_deg):
    """Plot 2D view of front-cone search area (corrected projection as ellipse)"""
    # Extract 2D position for plotting on XY plane
    position = position_3d[:2]
    height = params['height']
    downward_radius = params['downward-radius']
    camera_pitch_deg = params['camera-pitch-deg']
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(camera_pitch_deg)
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'bo', markersize=8)
    
    # Calculate the projection of the cone onto the ground (z=0) plane
    # The cone's apex is at (position.x, position.y, position.z)
    # The cone's axis is defined by yaw and pitch angles
    # We need to find where the cone intersects the ground plane
    
    # Direction vector of the cone axis
    dir_x = math.cos(yaw_rad) * math.cos(pitch_rad)
    dir_y = math.sin(yaw_rad) * math.cos(pitch_rad)
    dir_z = -math.sin(pitch_rad)  # Negative because z-axis points upwards, but pitch is downward
    
    # Check if the cone axis intersects the ground (z=0)
    if dir_z != 0:
        # Parameter t for the line equation: P(t) = UAV_pos + t * direction
        # At ground (z=0): position_3d[2] + t * dir_z = 0 => t = -position_3d[2] / dir_z
        t = -position_3d[2] / dir_z
        
        # If t > 0, the cone axis intersects the ground in the "forward" direction
        if t > 0:
            # Point where the cone's central axis intersects the ground
            center_x = position_3d[0] + t * dir_x
            center_y = position_3d[1] + t * dir_y
            
            # The radius of the cone at the ground level
            # Using similar triangles: radius / height = ground_radius / (height + t * |dir_z|)
            # But it's simpler to consider the cone's half-angle.
            # The half-angle alpha of the cone is atan(downward_radius / height)
            alpha = math.atan(downward_radius / height)
            # The radius at ground level is t * tan(alpha)
            ground_radius = t * math.tan(alpha)
            
            # Create an ellipse to represent the cone projection
            # Calculate the angle of the ellipse based on yaw
            ellipse_angle = yaw_deg
            
            # For a more accurate representation, we should calculate the major and minor axes
            # of the ellipse formed by the intersection of the cone with the ground plane
            # For now, we'll approximate with a simple ellipse
            ellipse = matplotlib.patches.Ellipse(
                (center_x, center_y), 
                width=2 * ground_radius, 
                height=ground_radius,  # Make it elliptical
                angle=ellipse_angle,
                color='blue', 
                alpha=0.3
            )
            ax.add_patch(ellipse)
        else:
            # If t <= 0, the cone is pointing upwards or parallel to ground,
            # and doesn't intersect the ground in front of the UAV.
            # In this case, we might not draw anything or draw a very small circle.
            # For now, let's draw a small circle at UAV's XY position.
            circle = plt.Circle((position[0], position[1]), 0.1, color='blue', alpha=0.3)
            ax.add_patch(circle)
    else:
        # If dir_z == 0, the cone axis is parallel to the ground.
        # We'll draw a circle at a distance in the direction of yaw.
        # Let's assume a fixed distance for visualization.
        distance = 5.0
        center_x = position_3d[0] + distance * math.cos(yaw_rad)
        center_y = position_3d[1] + distance * math.sin(yaw_rad)
        # The radius would be (distance / height) * downward_radius
        ground_radius = (distance / height) * downward_radius
        
        # Create an ellipse to represent the cone projection
        ellipse_angle = yaw_deg
        ellipse = matplotlib.patches.Ellipse(
            (center_x, center_y), 
            width=2 * ground_radius, 
            height=ground_radius,  # Make it elliptical
            angle=ellipse_angle,
            color='blue', 
            alpha=0.3
        )
        ax.add_patch(ellipse)
    
    # Set axis limits with padding
    padding = 3
    # Determine a reasonable limit based on the cone's projection
    if dir_z != 0 and t > 0:
        xlim_min = min(position[0], center_x - 2 * ground_radius) - padding
        xlim_max = max(position[0], center_x + 2 * ground_radius) + padding
        ylim_min = min(position[1], center_y - ground_radius) - padding
        ylim_max = max(position[1], center_y + ground_radius) + padding
    else:
        xlim_min = position[0] - padding
        xlim_max = position[0] + padding
        ylim_min = position[1] - padding
        ylim_max = position[1] + padding
        
    ax.set_xlim(xlim_min, xlim_max)
    ax.set_ylim(ylim_min, ylim_max)
    
    ax.set_aspect('equal')
    ax.set_title('front-cone, 2D view')

def plot_front_cone_search_3d(ax, position_3d, params, yaw_deg):
    """Plot 3D view of front-cone search area using Poly3DCollection (optimized light cone effect)"""
    height = params['height']
    downward_radius = params['downward-radius']
    camera_pitch_deg = params['camera-pitch-deg']
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(camera_pitch_deg)
    
    # Draw the UAV as a point
    ax.scatter(position_3d[0], position_3d[1], position_3d[2], c='blue', s=50)
    
    # Draw vertical line from UAV to ground
    ax.plot([position_3d[0], position_3d[0]], [position_3d[1], position_3d[1]], [position_3d[2], 0], 'k--', linewidth=1)
    
    # Calculate cone parameters to match the 2D projection
    # Direction vector of the cone axis
    dir_x = math.cos(yaw_rad) * math.cos(pitch_rad)
    dir_y = math.sin(yaw_rad) * math.cos(pitch_rad)
    dir_z = -math.sin(pitch_rad)  # Negative because z-axis points upwards, but pitch is downward
    
    # Base center of the cone (at ground level z=0)
    if dir_z != 0:
        # Parameter t for the line equation: P(t) = UAV_pos + t * direction
        # At ground (z=0): position_3d[2] + t * dir_z = 0 => t = -position_3d[2] / dir_z
        t = -position_3d[2] / dir_z
        
        # Point where the cone's central axis intersects the ground
        base_center_x = position_3d[0] + t * dir_x
        base_center_y = position_3d[1] + t * dir_y
        base_center_z = 0
        
        # The radius of the cone at the ground level
        # Using similar triangles: radius / height = ground_radius / (height + t * |dir_z|)
        # But it's simpler to consider the cone's half-angle.
        # The half-angle alpha of the cone is atan(downward_radius / height)
        alpha = math.atan(downward_radius / height)
        # The radius at ground level is t * tan(alpha)
        base_radius = t * math.tan(alpha)
    else:
        # If dir_z == 0, the cone axis is parallel to the ground.
        # We'll draw a circle at a distance in the direction of yaw.
        # Let's assume a fixed distance for visualization.
        distance = 5.0
        base_center_x = position_3d[0] + distance * math.cos(yaw_rad)
        base_center_y = position_3d[1] + distance * math.sin(yaw_rad)
        base_center_z = 0
        # The radius would be (distance / height) * downward_radius
        base_radius = (distance / height) * downward_radius
    
    # Create a 3D cone using Poly3DCollection for a smooth light cone effect
    # Number of points around the base
    n_points = 100
    theta = np.linspace(0, 2 * np.pi, n_points)
    
    # Points on the base circle
    base_x = base_center_x + base_radius * np.cos(theta)
    base_y = base_center_y + base_radius * np.sin(theta)
    base_z = np.full_like(base_x, base_center_z)
    
    # Apex point (UAV position)
    apex_x, apex_y, apex_z = position_3d
    
    # Create arrays to hold the points for the cone surface
    # We need to build triangles from the apex to the base
    verts = []
    for i in range(n_points):
        next_i = (i + 1) % n_points
        verts.append([(apex_x, apex_y, apex_z),
                      (base_x[i], base_y[i], base_z[i]),
                      (base_x[next_i], base_y[next_i], base_z[next_i])])
    
    # Create the Poly3DCollection
    poly3d = Poly3DCollection(verts, alpha=0.3, facecolor='blue', edgecolor='none')
    ax.add_collection3d(poly3d)
    
    # Set axis limits with padding
    padding = 3
    all_x = np.concatenate([[apex_x], base_x])
    all_y = np.concatenate([[apex_y], base_y])
    all_z = np.concatenate([[apex_z], base_z])
    
    ax.set_xlim(np.min(all_x) - padding, np.max(all_x) + padding)
    ax.set_ylim(np.min(all_y) - padding, np.max(all_y) + padding)
    ax.set_zlim(np.min(all_z) - padding, np.max(all_z) + padding)
    
    # Remove axis
    ax.set_axis_off()
    
    # Set view angle for a better perspective from right-front of UAV
    ax.view_init(elev=15, azim=-(yaw_deg + 90))  # View from right-front direction
    
    ax.set_title('front-cone, 3D view')

# Create 6 separate figures
fig1, ax1 = plt.subplots(figsize=(8, 6))
plot_downward_search_2d(ax1, sample_position[:2], searching_config['downward'])
plt.tight_layout()
output_path_1 = os.path.join(os.path.dirname(__file__), '..', 'plot', 'downward-2d.png')
plt.savefig(output_path_1, dpi=300, bbox_inches='tight')
plt.close(fig1)

fig2 = plt.figure(figsize=(8, 6))
ax2 = fig2.add_subplot(111, projection='3d')
plot_downward_search_3d(ax2, sample_position, searching_config['downward'])
plt.tight_layout()
output_path_2 = os.path.join(os.path.join(os.path.dirname(__file__), '..', 'plot', 'downward-3d.png'))
plt.savefig(output_path_2, dpi=300, bbox_inches='tight')
plt.close(fig2)

fig3, ax3 = plt.subplots(figsize=(8, 6))
plot_front_sector_search_2d(ax3, sample_position[:2], searching_config['front-sector'], sample_yaw_deg)
plt.tight_layout()
output_path_3 = os.path.join(os.path.dirname(__file__), '..', 'plot', 'front-sector-2d.png')
plt.savefig(output_path_3, dpi=300, bbox_inches='tight')
plt.close(fig3)

fig4 = plt.figure(figsize=(8, 6))
ax4 = fig4.add_subplot(111, projection='3d')
plot_front_sector_search_3d(ax4, sample_position, searching_config['front-sector'], sample_yaw_deg)
plt.tight_layout()
output_path_4 = os.path.join(os.path.dirname(__file__), '..', 'plot', 'front-sector-3d.png')
plt.savefig(output_path_4, dpi=300, bbox_inches='tight')
plt.close(fig4)

fig5, ax5 = plt.subplots(figsize=(8, 6))
plot_front_cone_search_2d(ax5, sample_position, searching_config['front-cone'], sample_yaw_deg) # Pass full 3D position
plt.tight_layout()
output_path_5 = os.path.join(os.path.dirname(__file__), '..', 'plot', 'front-cone-2d.png')
plt.savefig(output_path_5, dpi=300, bbox_inches='tight')
plt.close(fig5)

fig6 = plt.figure(figsize=(8, 6))
ax6 = fig6.add_subplot(111, projection='3d')
plot_front_cone_search_3d(ax6, sample_position, searching_config['front-cone'], sample_yaw_deg)
plt.tight_layout()
output_path_6 = os.path.join(os.path.dirname(__file__), '..', 'plot', 'front-cone-3d.png')
plt.savefig(output_path_6, dpi=300, bbox_inches='tight')
plt.close(fig6)

print(f"Visualizations saved to:")
print(f"  {output_path_1}")
print(f"  {output_path_2}")
print(f"  {output_path_3}")
print(f"  {output_path_4}")
print(f"  {output_path_5}")
print(f"  {output_path_6}")