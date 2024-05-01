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

# Car Params
CAR_DIMS = [2, 1, 1]
DT = 0.1
L = 1.5
Z_VALUE = 0.5

VISUALIZATION_TIMESTEP = 0.05  # seconds

INTERPOLATION_FACTOR = 10

# Noise Params
LOW_ACT_NOISE_STDDEV = [np.sqrt(0.1), np.sqrt(0.05)]
HIGH_ACT_NOISE_STDDEV = [np.sqrt(0.3), np.sqrt(0.2)]

LOW_ODO_NOISE_STDDEV = [np.sqrt(0.05), np.sqrt(0.03)]
HIGH_ODO_NOISE_STDDEV = [np.sqrt(0.15), np.sqrt(0.1)]

LOW_OBS_NOISE_STDDEV = [np.sqrt(0.1), np.sqrt(0.1)]
HIGH_OBS_NOISE_STDDEV = [np.sqrt(0.5), np.sqrt(0.25)]

# RRT Params
ALPHA = 0.7

MAXIMUM_PROP_COUNT = 3000
MAX_PROP_STEPS = 10
MIN_PROP_STEPS = 1

GOAL_POS_DIST = 2
GOAL_ANGLE_DIST = 0.5
GOAL_BIAS_PROB = 0.6

MAX_VEL = 2
MIN_VEL = -1

MIN_STEERING = -np.pi/3
MAX_STEERING = np.pi/3


MIN_START_GOAL_DIST = 30
