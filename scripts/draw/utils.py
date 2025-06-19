import os
import re
import glob
import json
import math
import time
import tqdm
import ffmpeg

import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.animation as animation
from matplotlib import patheffects
from matplotlib.gridspec import GridSpec
from matplotlib.patches import Circle
from matplotlib.patches import Wedge
from mpl_toolkits.axes_grid1 import make_axes_locatable


def name2Color(name):
    name = name.lower()
    if 'cvt' in name:
        return 'orangered'
    if 'energy' in name:
        return 'mediumblue'
    if 'safe' in name:
        return 'red'
    return 'black'
