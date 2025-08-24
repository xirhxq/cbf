import math
import matplotlib
import numpy as np
import os
import time

import matplotlib.pyplot as plt

scriptPath = os.path.dirname(os.path.realpath(__file__))
dir = os.path.join(scriptPath, '..', '..', 'plot', 'draw_cbf')
os.makedirs(dir, exist_ok=True)

matplotlib.use('Agg')
# matplotlib.rcParams['text.usetex'] = True

dCharge = 0.3
pCharge = np.array([[-3, 1], [0, 1], [3, 1]])

def minDis2Charge(pos):
    dePos = pos[np.newaxis, :] - pCharge[:, :, np.newaxis, np.newaxis]
    dis = np.linalg.norm(dePos, axis=1)
    minDis = np.min(dis, axis=0)
    return minDis


kBatt = 15

def rho(pos):
    return kBatt * np.log(minDis2Charge(pos) / dCharge)


plt.figure(figsize=(5, 5))
x = np.linspace(-10, 10, 100)
y = np.linspace(0, 20, 100)

lsPos = np.array(np.meshgrid(x, y))

lsBattery = np.arange(0, 100, 10)

z = rho(lsPos)
ct = plt.contour(x, y, z, lsBattery, colors='black')
plt.clabel(ct, inline=1, fontsize=10)

plt.xlabel('x')
plt.ylabel('y')
plt.title('Minimun battery level to reach charging station')

plt.savefig(os.path.join(dir, 'cbfEnergy.png'))
