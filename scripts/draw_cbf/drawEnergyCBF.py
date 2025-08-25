import math
import matplotlib
import numpy as np
import os
import json
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

scriptPath = os.path.dirname(os.path.realpath(__file__))
dir = os.path.join(scriptPath, '..', '..', 'plot', 'draw_cbf')
os.makedirs(dir, exist_ok=True)

config_path = os.path.join(scriptPath, '..', '..', 'config', 'config.json')
with open(config_path, 'r') as f:
    config = json.load(f)

matplotlib.use('Agg')

# Battery range constants (should match BaseModel.hpp)
BATTERY_MIN = 3700.0
BATTERY_MAX = 4200.0

pCharge = np.array(config['world']['charge'])
kBatt = config['cbfs']['without-slack']['energy']['k']
dCharge = 0.3

print(f"pCharge: {pCharge}")
print(f"kBatt: {kBatt}")
print(f"dCharge: {dCharge}")
print(f"Battery range: {BATTERY_MIN} to {BATTERY_MAX}")

def minDis2Charge(pos):
    dePos = pos[np.newaxis, :] - pCharge[:, :, np.newaxis, np.newaxis]
    dis = np.linalg.norm(dePos, axis=1)
    minDis = np.min(dis, axis=0)
    return minDis

def rho(pos):
    min_dis = minDis2Charge(pos)
    print(f"Min distance range: {min_dis.min()} to {min_dis.max()}")
    ratio = min_dis / dCharge
    print(f"Ratio range: {ratio.min()} to {ratio.max()}")
    log_ratio = np.log(ratio)
    print(f"Log ratio range: {log_ratio.min()} to {log_ratio.max()}")
    result = kBatt * log_ratio
    print(f"Rho range: {result.min()} to {result.max()}")
    return result

def normalized_battery(battery_level):
    return (battery_level - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN) * 100.0

def energy_cbf(battery_level, position):
    normalized_batt = normalized_battery(battery_level)
    rho_value = rho(position)
    return normalized_batt - rho_value

world_boundary = np.array(config['world']['boundary'])
x_min, x_max = world_boundary[:, 0].min(), world_boundary[:, 0].max()
y_min, y_max = world_boundary[:, 1].min(), world_boundary[:, 1].max()

print(f"World boundary - X: {x_min} to {x_max}, Y: {y_min} to {y_max}")

# Create meshgrid for position
x = np.linspace(x_min, x_max, 200)
y = np.linspace(y_min, y_max, 200)
lsPos = np.array(np.meshgrid(x, y))

# First, create a dedicated plot for the rho function
plt.figure(figsize=(12, 10))

# Calculate rho values
z_rho = rho(lsPos)
print(f"Rho function range: {z_rho.min()} to {z_rho.max()}")

# Create contour plot for rho function
levels_rho = np.linspace(z_rho.min(), z_rho.max(), 15)
ct_rho = plt.contour(x, y, z_rho, levels_rho, colors='black')
plt.clabel(ct_rho, inline=1, fontsize=10)

# Plot charging stations
for charge_pos in pCharge:
    plt.plot(charge_pos[0], charge_pos[1], 'ro', markersize=10, alpha=0.7)

plt.xlabel('x')
plt.ylabel('y')
plt.title('Rho Function (Distance-based energy requirement to reach charging station)')
plt.grid(True, alpha=0.3)

output_path_rho = os.path.join(dir, 'rho_function_contour.png')
plt.savefig(output_path_rho, dpi=300, bbox_inches='tight')
print(f"Rho function contour plot saved to {output_path_rho}")

# Now create the energy CBF plots for different battery levels
battery_levels = [BATTERY_MIN, 3800, 3900, 4000, 4100, BATTERY_MAX]

fig, axes = plt.subplots(2, 3, figsize=(20, 12))
axes = axes.flatten()

for i, battery_level in enumerate(battery_levels):
    # Calculate energy CBF values
    z = energy_cbf(battery_level, lsPos)
    print(f"Battery level {battery_level}: Energy CBF range: {z.min()} to {z.max()}")
    
    # Create contour plot
    ax = axes[i]
    levels = np.linspace(z.min(), z.max(), 15)
    ct = ax.contour(x, y, z, levels, colors='black')
    ax.clabel(ct, inline=1, fontsize=8)
    
    # Plot charging stations
    for charge_pos in pCharge:
        ax.plot(charge_pos[0], charge_pos[1], 'ro', markersize=8, alpha=0.7)
    
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_title(f'Energy CBF (Battery: {battery_level:.0f})')
    ax.grid(True, alpha=0.3)

plt.tight_layout()

output_path = os.path.join(dir, 'energyCBF_contours.png')
plt.savefig(output_path, dpi=300, bbox_inches='tight')
print(f"Energy CBF contour plots saved to {output_path}")

# Also create a 3D surface plot for energy CBF
fig = plt.figure(figsize=(15, 10))

# For 3D plot, use a coarser grid to reduce computation
x_coarse = np.linspace(x_min, x_max, 100)
y_coarse = np.linspace(y_min, y_max, 100)
X, Y = np.meshgrid(x_coarse, y_coarse)
lsPos_coarse = np.array([X, Y])

# Use a fixed battery level for 3D plot (e.g., middle of range)
battery_level = (BATTERY_MIN + BATTERY_MAX) / 2
Z = energy_cbf(battery_level, lsPos_coarse)

ax = fig.add_subplot(111, projection='3d')
surf = ax.plot_surface(X, Y, Z, cmap='viridis', alpha=0.8)
ax.contour(X, Y, Z, zdir='z', offset=Z.min(), cmap='viridis', alpha=0.5)

# Plot charging stations
for charge_pos in pCharge:
    ax.scatter(charge_pos[0], charge_pos[1], Z.min(), color='red', s=100, alpha=0.7)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('Energy CBF Value')
ax.set_title(f'Energy CBF 3D Surface (Battery: {battery_level:.0f})')

output_path_3d = os.path.join(dir, 'energyCBF_3D.png')
plt.savefig(output_path_3d, dpi=300, bbox_inches='tight')
print(f"Energy CBF 3D figure saved to {output_path_3d}")