import matplotlib.pyplot as plt
import numpy as np
import json
import math
import os
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches

# Load configuration
config_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'config.json')
with open(config_path, 'r') as f:
    config = json.load(f)

# Get world boundary for plotting context
world_boundary = np.array(config['world']['boundary'])

# Get searching parameters
searching_config = config['searching']
methods = ['downward', 'front-sector', 'front-cone']

# ========================
# Visualization Parameters
# ========================
# These parameters can be adjusted for better visualization

# Sample UAV position and yaw for visualization
sample_position = np.array([0, 0, 0.5])  # [x, y, z] position
sample_yaw_deg = 90  # Yaw angle in degrees

# Colors for each method
METHOD_COLORS = {
    'downward': 'blue',
    'front-sector': 'green',
    'front-cone': 'red'
}

# Unified view parameters for 3D plots
VIEW_PARAMS = {
    'downward': {'elev': 20, 'azim': -70},           # Original downward view
    'front-sector': {'elev': 15, 'azim': -180},      # Original front-sector view (with yaw_deg=90)
    'front-cone': {'elev': 15, 'azim': -180},        # Original front-cone view (with yaw_deg=90)
    'zoom': 2.0                                      # Zoom level (higher = closer)
}

# Unified axis limits for 2D plots
AXIS_LIMITS = {
    'xlim': (-4, 4),
    'ylim': (-4, 4)
}

# Override parameters for visualization (if needed)
# Set to None to use parameters from config.json
override_params = {
    'downward': None,
    'front-sector': None,
    'front-cone': None
}

# Example of how to override parameters:
# override_params['front-cone'] = {
#     'height': 0.8,
#     'downward-radius': 0.3,
#     'camera-pitch-deg': 45.0
# }

# Function to get parameters (either from override or config)
def get_params(method_name):
    if override_params[method_name] is not None:
        return override_params[method_name]
    return searching_config[method_name]

# If you want to override parameters for visualization without changing config.json,
# uncomment and modify the appropriate lines in override_params above.
# For example, to make the front-cone more visible:
# override_params['front-cone'] = {
#     'height': 1.0,
#     'downward-radius': 0.5,
#     'camera-pitch-deg': 30.0
# }

def plot_downward_search_2d(ax, position, params, color='blue'):
    """Plot 2D view of downward circular search area"""
    radius = params['radius']
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'o', color=color, markersize=8)
    
    # Draw the search circle (semi-transparent)
    circle = plt.Circle((position[0], position[1]), radius, color=color, alpha=0.3)
    ax.add_patch(circle)
    
    # Set unified axis limits
    ax.set_xlim(AXIS_LIMITS['xlim'][0], AXIS_LIMITS['xlim'][1])
    ax.set_ylim(AXIS_LIMITS['ylim'][0], AXIS_LIMITS['ylim'][1])
    
    ax.set_aspect('equal')
    ax.set_title('downward, 2D view')

def plot_downward_search_3d(ax, position, params, color='blue'):
    """Plot 3D view of downward circular search area using Poly3DCollection"""
    radius = params['radius']
    
    # Draw the UAV as a point
    ax.scatter(position[0], position[1], position[2], c=color, s=50)
    
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
    poly3d = Poly3DCollection(verts, alpha=0.3, facecolor=color, edgecolor='none')
    ax.add_collection3d(poly3d)
    
    # Set axis limits with reduced padding to make the visualization larger
    padding = 1.0
    ax.set_xlim(position[0] - radius - padding, position[0] + radius + padding)
    ax.set_ylim(position[1] - radius - padding, position[1] + radius + padding)
    ax.set_zlim(0 - padding, position[2] + padding)
    
    # Remove axis
    ax.set_axis_off()
    
    # Set unified view angle for closer perspective (original downward view)
    ax.view_init(elev=VIEW_PARAMS['downward']['elev'], azim=VIEW_PARAMS['downward']['azim'])
    ax.set_box_aspect(None, zoom=VIEW_PARAMS['zoom'])  # Zoom in to make the view closer
    
    ax.set_title('downward, 3D view')

def plot_front_sector_search_2d(ax, position, params, yaw_deg, color='blue'):
    """Plot 2D view of front-sector search area"""
    inner_radius = params['inner-radius']
    outer_radius = params['outer-radius']
    half_angle_deg = params['half-angle-deg']
    yaw_rad = math.radians(yaw_deg)
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'o', color=color, markersize=8)
    
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
    ax.fill(sector_x, sector_y, color=color, alpha=0.3)
    
    # Set unified axis limits
    ax.set_xlim(AXIS_LIMITS['xlim'][0], AXIS_LIMITS['xlim'][1])
    ax.set_ylim(AXIS_LIMITS['ylim'][0], AXIS_LIMITS['ylim'][1])
    
    ax.set_aspect('equal')
    ax.set_title('front-sector, 2D view')

def plot_front_sector_search_3d(ax, position, params, yaw_deg, color='blue'):
    """Plot 3D view of front-sector search area using Poly3DCollection"""
    inner_radius = params['inner-radius']
    outer_radius = params['outer-radius']
    half_angle_deg = params['half-angle-deg']
    
    # Draw the UAV as a point
    ax.scatter(position[0], position[1], position[2], c=color, s=50)
    
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
    poly3d = Poly3DCollection(verts, alpha=0.3, facecolor=color, edgecolor='none')
    ax.add_collection3d(poly3d)
    
    # Set axis limits with reduced padding to make the visualization larger
    padding = 1.0
    max_radius = max(inner_radius, outer_radius)
    ax.set_xlim(position[0] - max_radius - padding, position[0] + max_radius + padding)
    ax.set_ylim(position[1] - max_radius - padding, position[1] + max_radius + padding)
    ax.set_zlim(0 - padding, position[2] + padding)
    
    # Remove axis
    ax.set_axis_off()
    
    # Set unified view angle for closer perspective (original front-sector view)
    ax.view_init(elev=VIEW_PARAMS['front-sector']['elev'], azim=VIEW_PARAMS['front-sector']['azim'])
    ax.set_box_aspect(None, zoom=VIEW_PARAMS['zoom'])  # Zoom in to make the view closer
    
    ax.set_title('front-sector, 3D view')

def get_ellipse_parameters(center_x, center_y, height, downward_radius, camera_pitch_deg, yaw_deg):
    """
    Calculate optimized ellipse parameters to better match the C++ cone projection.
    
    Let's try a different approach to calculate the ellipse parameters:
    - Use the actual cone intersection with the ground plane to derive the ellipse
    - Consider the cone as a quadratic surface and find its intersection with z=0
    """
    pitch_rad = math.radians(camera_pitch_deg)
    yaw_rad = math.radians(yaw_deg)

    alpha = math.atan(downward_radius / height)

    d1 = height / math.tan(pitch_rad + alpha)
    d2 = height / math.tan(pitch_rad - alpha)
    semi_major = (d2 - d1) / 2

    distance_to_center = (d1 + d2) / 2

    ellipse_cx = center_x + distance_to_center * math.cos(yaw_rad)
    ellipse_cy = center_y + distance_to_center * math.sin(yaw_rad)

    semi_minor = height / math.sin(pitch_rad) * math.tan(alpha)

    if semi_minor > semi_major:
        semi_major, semi_minor = semi_minor, semi_major

    # Rotation angle
    rotation_deg = yaw_deg
    
    return ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg

def plot_front_cone_search_2d(ax, position_3d, params, yaw_deg, color='blue'):
    """Plot 2D view of front-cone search area as an ellipse"""
    # Extract 2D position for plotting on XY plane
    position = position_3d[:2]
    height = params['height']
    downward_radius = params['downward-radius']
    camera_pitch_deg = params['camera-pitch-deg']
    
    # Draw the UAV as a point
    ax.plot(position[0], position[1], 'o', color=color, markersize=8)
    
    # Calculate ellipse parameters
    ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg = get_ellipse_parameters(
        position[0], position[1], height, downward_radius, camera_pitch_deg, yaw_deg)
    
    # Create ellipse
    theta_ellipse = np.linspace(0, 2*np.pi, 100)
    
    # Parametric equation of rotated ellipse
    cos_rot = math.cos(math.radians(rotation_deg))
    sin_rot = math.sin(math.radians(rotation_deg))
    
    ellipse_x = ellipse_cx + semi_major * np.cos(theta_ellipse) * cos_rot - semi_minor * np.sin(theta_ellipse) * sin_rot
    ellipse_y = ellipse_cy + semi_major * np.cos(theta_ellipse) * sin_rot + semi_minor * np.sin(theta_ellipse) * cos_rot
    
    # Plot ellipse
    ax.fill(ellipse_x, ellipse_y, color=color, alpha=0.3)
    
    # Set unified axis limits
    ax.set_xlim(AXIS_LIMITS['xlim'][0], AXIS_LIMITS['xlim'][1])
    ax.set_ylim(AXIS_LIMITS['ylim'][0], AXIS_LIMITS['ylim'][1])
    
    ax.set_aspect('equal')
    ax.set_title('front-cone, 2D view')

def plot_front_cone_search_3d(ax, position_3d, params, yaw_deg, color='blue'):
    """Plot 3D view of front-cone search area using Poly3DCollection"""
    height = params['height']
    downward_radius = params['downward-radius']
    camera_pitch_deg = params['camera-pitch-deg']
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(camera_pitch_deg)
    
    # Draw the UAV as a point
    ax.scatter(position_3d[0], position_3d[1], position_3d[2], c=color, s=50)
    
    # Draw vertical line from UAV to ground
    ax.plot([position_3d[0], position_3d[0]], [position_3d[1], position_3d[1]], [position_3d[2], 0], 'k--', linewidth=1)
    
    # Calculate alpha (half-angle of cone)
    alpha = math.atan(downward_radius / height)
    
    # Calculate ellipse parameters to get the correct center point
    d1 = height / math.tan(pitch_rad + alpha)
    d2 = height / math.tan(pitch_rad - alpha)
    distance_to_center = (d1 + d2) / 2
    
    # Calculate apex (A) and projection of central line (B) using the same center calculation as 2D
    A_x, A_y, A_z = position_3d[0], position_3d[1], height
    B_x = position_3d[0] + distance_to_center * math.cos(yaw_rad)
    B_y = position_3d[1] + distance_to_center * math.sin(yaw_rad)
    B_z = 0.0
    
    # Calculate base radius at ground level
    base_radius = np.linalg.norm([B_x - A_x, B_y - A_y, B_z - A_z]) * math.tan(alpha)
    
    # Create a 3D cone using Poly3DCollection for a smooth light cone effect
    # Number of points around the base
    n_points = 100
    theta = np.linspace(0, 2 * np.pi, n_points)
    
    # Points on the base circle
    base_x = B_x + base_radius * np.cos(theta)
    base_y = B_y + base_radius * np.sin(theta)
    base_z = np.full_like(base_x, B_z)
    
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
    poly3d = Poly3DCollection(verts, alpha=0.3, facecolor=color, edgecolor='none')
    ax.add_collection3d(poly3d)
    
    # Set axis limits with reduced padding to make the visualization larger
    padding = 1.0
    all_x = np.concatenate([[apex_x], base_x])
    all_y = np.concatenate([[apex_y], base_y])
    all_z = np.concatenate([[apex_z], base_z])
    
    # Use the same axis limits as other plots for consistency
    ax.set_xlim(AXIS_LIMITS['xlim'][0] - padding, AXIS_LIMITS['xlim'][1] + padding)
    ax.set_ylim(AXIS_LIMITS['ylim'][0] - padding, AXIS_LIMITS['ylim'][1] + padding)
    ax.set_zlim(0 - padding, position_3d[2] + padding)
    
    # Remove axis
    ax.set_axis_off()
    
    # Set unified view angle for closer perspective (original front-cone view)
    ax.view_init(elev=VIEW_PARAMS['front-cone']['elev'], azim=VIEW_PARAMS['front-cone']['azim'])
    ax.set_box_aspect(None, zoom=VIEW_PARAMS['zoom'])  # Zoom in to make the view closer
    
    ax.set_title('front-cone, 3D view')

# Create individual figures for each method
fig1, ax1 = plt.subplots(figsize=(6, 4.5))
plot_downward_search_2d(ax1, sample_position[:2], get_params('downward'), METHOD_COLORS['downward'])
plt.tight_layout()
output_path_1 = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'downward-2d.png')
plt.savefig(output_path_1, dpi=300)
plt.close(fig1)

fig2 = plt.figure(figsize=(6, 4.5))
ax2 = fig2.add_subplot(111, projection='3d')
plot_downward_search_3d(ax2, sample_position, get_params('downward'), METHOD_COLORS['downward'])
plt.tight_layout()
output_path_2 = os.path.join(os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'downward-3d.png'))
plt.savefig(output_path_2, dpi=300)
plt.close(fig2)

fig3, ax3 = plt.subplots(figsize=(6, 4.5))
plot_front_sector_search_2d(ax3, sample_position[:2], get_params('front-sector'), sample_yaw_deg, METHOD_COLORS['front-sector'])
plt.tight_layout()
output_path_3 = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'front-sector-2d.png')
plt.savefig(output_path_3, dpi=300)
plt.close(fig3)

fig4 = plt.figure(figsize=(6, 4.5))
ax4 = fig4.add_subplot(111, projection='3d')
plot_front_sector_search_3d(ax4, sample_position, get_params('front-sector'), sample_yaw_deg, METHOD_COLORS['front-sector'])
plt.tight_layout()
output_path_4 = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'front-sector-3d.png')
plt.savefig(output_path_4, dpi=300)
plt.close(fig4)

fig5, ax5 = plt.subplots(figsize=(6, 4.5))
plot_front_cone_search_2d(ax5, sample_position, get_params('front-cone'), sample_yaw_deg, METHOD_COLORS['front-cone']) # Pass full 3D position
plt.tight_layout()
output_path_5 = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'front-cone-2d.png')
plt.savefig(output_path_5, dpi=300)
plt.close(fig5)

fig6 = plt.figure(figsize=(6, 4.5))
ax6 = fig6.add_subplot(111, projection='3d')
plot_front_cone_search_3d(ax6, sample_position, get_params('front-cone'), sample_yaw_deg, METHOD_COLORS['front-cone'])
plt.tight_layout()
output_path_6 = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'front-cone-3d.png')
plt.savefig(output_path_6, dpi=300)
plt.close(fig6)

# Create comparison figures with all methods
# 2D comparison
fig7, ax7 = plt.subplots(figsize=(6, 4.5))
plot_downward_search_2d(ax7, sample_position[:2], get_params('downward'), METHOD_COLORS['downward'])
plot_front_sector_search_2d(ax7, sample_position[:2], get_params('front-sector'), sample_yaw_deg, METHOD_COLORS['front-sector'])
plot_front_cone_search_2d(ax7, sample_position, get_params('front-cone'), sample_yaw_deg, METHOD_COLORS['front-cone'])

# Add legend
legend_elements = [plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=METHOD_COLORS['downward'], markersize=8, label='downward'),
                   plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=METHOD_COLORS['front-sector'], markersize=8, label='front-sector'),
                   plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=METHOD_COLORS['front-cone'], markersize=8, label='front-cone')]
ax7.legend(handles=legend_elements, loc='upper right')

plt.tight_layout()
output_path_7 = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'comparison-2d.png')
plt.savefig(output_path_7, dpi=300)
plt.close(fig7)

# 3D comparison
fig8 = plt.figure(figsize=(6, 4.5))
ax8 = fig8.add_subplot(111, projection='3d')
plot_downward_search_3d(ax8, sample_position, get_params('downward'), METHOD_COLORS['downward'])
plot_front_sector_search_3d(ax8, sample_position, get_params('front-sector'), sample_yaw_deg, METHOD_COLORS['front-sector'])
plot_front_cone_search_3d(ax8, sample_position, get_params('front-cone'), sample_yaw_deg, METHOD_COLORS['front-cone'])

# Add legend
legend_elements = [plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=METHOD_COLORS['downward'], markersize=8, label='downward'),
                   plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=METHOD_COLORS['front-sector'], markersize=8, label='front-sector'),
                   plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=METHOD_COLORS['front-cone'], markersize=8, label='front-cone')]
ax8.legend(handles=legend_elements, loc='upper right')

plt.tight_layout()
output_path_8 = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', 'comparison-3d.png')
plt.savefig(output_path_8, dpi=300)
plt.close(fig8)

print(f"Visualizations saved to:")
print(f"  {output_path_1}")
print(f"  {output_path_2}")
print(f"  {output_path_3}")
print(f"  {output_path_4}")
print(f"  {output_path_5}")
print(f"  {output_path_6}")
print(f"  {output_path_7}")
print(f"  {output_path_8}")