#!/usr/bin/env python3

import numpy as np
import math
import matplotlib.pyplot as plt
import os

def compute_cone_projection_cpp_method(height, downward_radius, camera_pitch_deg, yaw_deg, position_2d):
    """Compute cone projection using the C++ method"""
    pitch_rad = math.radians(camera_pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    
    # Cone apex (A)
    A = np.array([position_2d[0], position_2d[1], height])
    
    # Projection of cone central line on ground (B)
    B = np.array([
        position_2d[0] + height / math.tan(pitch_rad) * math.cos(yaw_rad),
        position_2d[1] + height / math.tan(pitch_rad) * math.sin(yaw_rad),
        0.0
    ])
    
    # Cone half-angle
    alpha = math.atan(downward_radius / height)
    
    # Compute search area bounds
    l = height / math.tan(pitch_rad - alpha) - height / math.tan(pitch_rad + alpha)
    
    return A, B, alpha, l

def check_point_in_cone_cpp_method(P, A, B, alpha):
    """Check if point P is inside the cone using C++ method"""
    PA = A - P
    BA = A - B
    
    # Calculate dot product and magnitudes
    dot_product = np.dot(PA, BA)
    magnitude_PA = np.linalg.norm(PA)
    magnitude_BA = np.linalg.norm(BA)
    
    # Skip if zero magnitude
    if magnitude_PA == 0 or magnitude_BA == 0:
        return False
        
    # Calculate angle
    angle = math.acos(dot_product / (magnitude_PA * magnitude_BA))
    
    # Check if inside cone
    return angle <= alpha

def get_ellipse_parameters(center_x, center_y, height, downward_radius, camera_pitch_deg, yaw_deg):
    """
    Calculate optimized ellipse parameters to better match the C++ cone projection.
    
    Let's try a different approach to calculate the ellipse parameters:
    - Use the actual cone intersection with the ground plane to derive the ellipse
    - Consider the cone as a quadratic surface and find its intersection with z=0
    """
    print("\nComputing cone projection parameters...")
    pitch_rad = math.radians(camera_pitch_deg)
    yaw_rad = math.radians(yaw_deg)

    alpha = math.atan(downward_radius / height)
    print(f"Computed cone half-angle (alpha): {math.degrees(alpha):.6f} degrees, tan(alpha)={math.tan(alpha):.6f}")

    d1 = height / math.tan(pitch_rad + alpha)
    d2 = height / math.tan(pitch_rad - alpha)
    print(f"Computed distances to boundary points (d1, d2): ({d1:.6f}, {d2:.6f})")
    semi_major = (d2 - d1) / 2
    print(f"Computed cone semi-major axis (semi_major): {semi_major:.6f}")

    distance_to_center = (d1 + d2) / 2

    ellipse_cx = center_x + distance_to_center * math.cos(yaw_rad)
    ellipse_cy = center_y + distance_to_center * math.sin(yaw_rad)

    semi_minor = height / math.sin(pitch_rad) * math.tan(alpha)

    if semi_minor > semi_major:
        semi_major, semi_minor = semi_minor, semi_major

    # Rotation angle
    rotation_deg = yaw_deg
    
    return ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg

def check_point_in_ellipse(x, y, cx, cy, a, b, rotation_deg):
    """
    Check if point (x,y) is inside an ellipse with center (cx,cy), 
    semi-major axis a, semi-minor axis b, rotated by rotation_deg.
    """
    # Convert to ellipse coordinate system
    rad = math.radians(rotation_deg)
    dx = x - cx
    dy = y - cy
    
    # Rotate point coordinates
    x_rot = dx * math.cos(rad) + dy * math.sin(rad)
    y_rot = -dx * math.sin(rad) + dy * math.cos(rad)
    
    # Check if point is inside ellipse
    return (x_rot**2 / a**2) + (y_rot**2 / b**2) <= 1

def sample_points_in_area(cx, cy, l, num_points=1000):
    """Sample random points in the area around the cone projection"""
    x_min, x_max = cx - l, cx + l
    y_min, y_max = cy - l, cy + l
    
    points = []
    for _ in range(num_points):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        points.append((x, y))
    
    return points

def visualize_comparison(height, downward_radius, camera_pitch_deg, yaw_deg, position_2d, output_path="cone_comparison.png"):
    """Visualize the cone and ellipse to understand the difference"""
    # Compute using C++ method
    A, B, alpha, l = compute_cone_projection_cpp_method(
        height, downward_radius, camera_pitch_deg, yaw_deg, position_2d)
    
    # Compute using ellipse method
    ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg = get_ellipse_parameters(
        position_2d[0], position_2d[1], height, downward_radius, camera_pitch_deg, yaw_deg)
    
    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    
    # Draw UAV position
    ax.plot(position_2d[0], position_2d[1], 'ro', markersize=8, label='UAV')
    
    # Sample points and check if they're in the cone/ellipse
    points = sample_points_in_area(ellipse_cx, ellipse_cy, l, 5000)
    
    # Plot points inside cone
    cpp_points_x = []
    cpp_points_y = []
    ellipse_points_x = []
    ellipse_points_y = []
    both_points_x = []
    both_points_y = []
    
    for x, y in points:
        P = np.array([x, y, 0])
        
        # Check with C++ method
        in_cpp = check_point_in_cone_cpp_method(P, A, B, alpha)
        
        # Check with ellipse method
        in_ellipse = check_point_in_ellipse(x, y, ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg)
        
        if in_cpp and in_ellipse:
            both_points_x.append(x)
            both_points_y.append(y)
        elif in_cpp:
            cpp_points_x.append(x)
            cpp_points_y.append(y)
        elif in_ellipse:
            ellipse_points_x.append(x)
            ellipse_points_y.append(y)
    
    # Plot points
    if cpp_points_x:
        ax.scatter(cpp_points_x, cpp_points_y, s=1, color='blue', alpha=0.5, label='Only in cone (C++ method)')
    if ellipse_points_x:
        ax.scatter(ellipse_points_x, ellipse_points_y, s=1, color='green', alpha=0.5, label='Only in ellipse')
    if both_points_x:
        ax.scatter(both_points_x, both_points_y, s=1, color='purple', alpha=0.5, label='In both')
    
    # Draw ellipse
    theta_ellipse = np.linspace(0, 2*np.pi, 100)
    cos_rot = math.cos(math.radians(rotation_deg))
    sin_rot = math.sin(math.radians(rotation_deg))
    ellipse_x = ellipse_cx + semi_major * np.cos(theta_ellipse) * cos_rot - semi_minor * np.sin(theta_ellipse) * sin_rot
    ellipse_y = ellipse_cy + semi_major * np.cos(theta_ellipse) * sin_rot + semi_minor * np.sin(theta_ellipse) * cos_rot
    ax.plot(ellipse_x, ellipse_y, 'g-', linewidth=2, label='Ellipse')
    
    # Draw cone projection boundary (approximate)
    # Draw a circle with radius l around the center
    theta_circle = np.linspace(0, 2*np.pi, 100)
    circle_x = ellipse_cx + l * np.cos(theta_circle)
    circle_y = ellipse_cy + l * np.sin(theta_circle)
    ax.plot(circle_x, circle_y, 'b--', linewidth=1, label='C++ search area bound')
    
    # ax.set_xlim(ellipse_cx - l, ellipse_cx + l)
    # ax.set_ylim(ellipse_cy - l, ellipse_cy + l)
    ax.set_aspect('equal')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Comparison of C++ Cone Method and Ellipse Approximation')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    # Save to plot folder
    plot_output_path = os.path.join(os.path.dirname(__file__), '..', '..', 'plot', 'visualize_plot', output_path)
    plt.savefig(plot_output_path, dpi=300)
    plt.close()
    print(f"Visualization saved to: {plot_output_path}")

def validate_ellipse_axes(height, downward_radius, camera_pitch_deg, yaw_deg, position_2d):
    """
    Validate the ellipse by checking if the four extreme points (endpoints of major and minor axes)
    are inside the cone according to the C++ method.
    """
    # Compute using C++ method
    A, B, alpha, l = compute_cone_projection_cpp_method(
        height, downward_radius, camera_pitch_deg, yaw_deg, position_2d)
    
    # Compute ellipse parameters
    ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg = get_ellipse_parameters(
        position_2d[0], position_2d[1], height, downward_radius, camera_pitch_deg, yaw_deg)
    
    # Calculate the four extreme points of the ellipse
    # Points at the ends of the major axis
    rad_rot = math.radians(rotation_deg)
    major_end1_x = ellipse_cx + semi_major * math.cos(rad_rot)
    major_end1_y = ellipse_cy + semi_major * math.sin(rad_rot)
    major_end2_x = ellipse_cx - semi_major * math.cos(rad_rot)
    major_end2_y = ellipse_cy - semi_major * math.sin(rad_rot)
    
    # Points at the ends of the minor axis
    minor_end1_x = ellipse_cx - semi_minor * math.sin(rad_rot)
    minor_end1_y = ellipse_cy + semi_minor * math.cos(rad_rot)
    minor_end2_x = ellipse_cx + semi_minor * math.sin(rad_rot)
    minor_end2_y = ellipse_cy - semi_minor * math.cos(rad_rot)
    
    # Check if these points are inside the cone using C++ method
    points = [
        ("Major axis end 1", major_end1_x, major_end1_y),
        ("Major axis end 2", major_end2_x, major_end2_y),
        ("Minor axis end 1", minor_end1_x, minor_end1_y),
        ("Minor axis end 2", minor_end2_x, minor_end2_y)
    ]
    
    print("Validation of ellipse axes endpoints:")
    all_inside = True
    for name, x, y in points:
        P = np.array([x, y, 0])
        inside = check_point_in_cone_cpp_method(P, A, B, alpha)
        print(f"  {name} ({x:.6f}, {y:.6f}): {'Inside' if inside else 'Outside'}")
        if not inside:
            all_inside = False
    
    return all_inside

# Test with sample parameters
if __name__ == "__main__":
    # Sample parameters from config.json
    height = 2
    downward_radius = 0.2
    camera_pitch_deg = 30.0
    yaw_deg = 90.0
    position_2d = [0, 0]  # UAV position
    
    print("Comparing cone projection methods with parameters:")
    print(f"  Height: {height}")
    print(f"  Downward radius: {downward_radius}")
    print(f"  Camera pitch: {camera_pitch_deg} degrees")
    print(f"  Yaw: {yaw_deg} degrees")
    print(f"  UAV position: ({position_2d[0]}, {position_2d[1]})")
    
    # Compute using C++ method
    A, B, alpha, l = compute_cone_projection_cpp_method(
        height, downward_radius, camera_pitch_deg, yaw_deg, position_2d)
    
    print("\nC++ Method Results:")
    print(f"  Apex (A): ({A[0]:.6f}, {A[1]:.6f}, {A[2]:.6f})")
    print(f"  Base center (B): ({B[0]:.6f}, {B[1]:.6f}, {B[2]:.6f})")
    print(f"  Cone half-angle (alpha): {math.degrees(alpha):.6f} degrees")
    print(f"  Search area bound (l): {l:.6f}")
    
    # Compute using ellipse method
    ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg = get_ellipse_parameters(
        position_2d[0], position_2d[1], height, downward_radius, camera_pitch_deg, yaw_deg)
    
    print("\nOptimized Ellipse Method Results:")
    print(f"  Ellipse center: ({ellipse_cx:.6f}, {ellipse_cy:.6f})")
    print(f"  Semi-major axis: {semi_major:.6f}")
    print(f"  Semi-minor axis: {semi_minor:.6f}")
    print(f"  Rotation angle: {rotation_deg:.6f} degrees")
    
    # Validate ellipse axes
    print("\nValidating ellipse axes endpoints:")
    axes_valid = validate_ellipse_axes(height, downward_radius, camera_pitch_deg, yaw_deg, position_2d)
    print(f"  All axes endpoints inside cone: {axes_valid}")
    
    # Sample points and check if they're in the cone/ellipse
    print("\nSampling points to compare methods:")
    points = sample_points_in_area(ellipse_cx, ellipse_cy, l, 10000)
    
    # Count points inside each shape
    cpp_count = 0
    ellipse_count = 0
    both_count = 0
    neither_count = 0
    
    for x, y in points:
        P = np.array([x, y, 0])
        
        # Check with C++ method
        in_cpp = check_point_in_cone_cpp_method(P, A, B, alpha)
        
        # Check with ellipse method
        in_ellipse = check_point_in_ellipse(x, y, ellipse_cx, ellipse_cy, semi_major, semi_minor, rotation_deg)
        
        if in_cpp:
            cpp_count += 1
        if in_ellipse:
            ellipse_count += 1
        if in_cpp and in_ellipse:
            both_count += 1
        if not in_cpp and not in_ellipse:
            neither_count += 1
    
    print(f"  Points inside cone (C++ method): {cpp_count}")
    print(f"  Points inside ellipse: {ellipse_count}")
    print(f"  Points inside both: {both_count}")
    print(f"  Points inside neither: {neither_count}")
    print(f"  Jaccard similarity: {both_count / (cpp_count + ellipse_count - both_count):.6f}" if cpp_count + ellipse_count - both_count > 0 else "  Jaccard similarity: undefined (no points in either)")
    
    # Additional analysis
    print("\nAdditional analysis:")
    print(f"  Percentage of cone points captured by ellipse: {both_count / cpp_count * 100:.2f}%" if cpp_count > 0 else "  Percentage of cone points captured by ellipse: undefined")
    print(f"  Percentage of ellipse points that are in cone: {both_count / ellipse_count * 100:.2f}%" if ellipse_count > 0 else "  Percentage of ellipse points that are in cone: undefined")
    
    # Now let's create a visualization to see the difference
    print("\nGenerating visualization...")
    visualize_comparison(height, downward_radius, camera_pitch_deg, yaw_deg, position_2d, "cone_comparison.png")