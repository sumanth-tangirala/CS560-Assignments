import numpy as np

upper_space_limits = np.array([50, 50, 50])
lower_space_limits = np.array([-50, -50, 0])

red = "0xff0000"
green = "0x00ff00"
blue = "0x0000ff"
orange = "0xffa500"
yellow = "0xffff00"
purple = "0x800080"
grey = "#808080"

VEHICLE_SIZE = [5, 4, 2]

ARM_LINK_SIZES = [
    np.array([2, 2, 0.5]),
    np.array([1, 1, 4]),
    np.array([1, 1, 4]),
]

ALPHA = 0.7

# PRM
NUM_PRM_NODES = 5000
NUM_PRM_NEIGHBORS = 6

# Collision Checking Params
NUM_ARM_INTERPOLATIONS = 5
VEHICLE_INTERPOLATION_DIST = 1.9

# Visualization Params
VEHICLE_INTERPOLATION_TIME = 0.5  # seconds
ARM_INTERPOLATION_TIME = 1  # seconds
TIMESTEP = 0.05  # seconds

# Car Like Params
CAR_DIMS = [2, 1, 1]
DT = 0.1
L = 1.5
Z_VALUE = 0.5