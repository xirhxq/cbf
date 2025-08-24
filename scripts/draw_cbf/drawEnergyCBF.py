import math
import matplotlib
import numpy as np
import os
import json
import time

import matplotlib.pyplot as plt

scriptPath = os.path.dirname(os.path.realpath(__file__))
dir = os.path.join(scriptPath, '..', '..', 'plot', 'draw_cbf')
os.makedirs(dir, exist_ok=True)

config_path = os.path.join(scriptPath, '..', '..', 'config', 'config.json')
with open(config_path, 'r') as f:
    config = json.load(f)

matplotlib.use('Agg')

pCharge = np.array(config['world']['charge'])
kBatt = config['cbfs']['without-slack']['energy']['k']
dCharge = 0.3

def minDis2Charge(pos):
    dePos = pos[np.newaxis, :] - pCharge[:, :, np.newaxis, np.newaxis]
    dis = np.linalg.norm(dePos, axis=1)
    minDis = np.min(dis, axis=0)
    return minDis

def rho(pos):
    return kBatt * np.log(minDis2Charge(pos) / dCharge)

world_boundary = np.array(config['world']['boundary'])
x_min, x_max = world_boundary[:, 0].min(), world_boundary[:, 0].max()
y_min, y_max = world_boundary[:, 1].min(), world_boundary[:, 1].max()

plt.figure(figsize=(10, 10))
x = np.linspace(x_min, x_max, 200)
y = np.linspace(y_min, y_max, 200)

lsPos = np.array(np.meshgrid(x, y))

lsBattery = np.arange(0, 100, 10)

z = rho(lsPos)
ct = plt.contour(x, y, z, lsBattery, colors='black')
plt.clabel(ct, inline=1, fontsize=10)

for charge_pos in pCharge:
    plt.plot(charge_pos[0], charge_pos[1], 'ro', markersize=8, alpha=0.3)

plt.xlabel('x')
plt.ylabel('y')
plt.title('Minimum battery level to reach charging station')

plt.savefig(os.path.join(dir, 'cbfEnergy.png'))
