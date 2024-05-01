from multiprocessing import Pool, cpu_count
from itertools import count
from math import ceil
from tqdm import tqdm

import numpy as np

from constants import ALPHA

tiebreaker = count()


def closest_point_on_cuboid_to_sphere(
    cuboid_center,
    cuboid_orientation,
    cuboid_size,
    sphere_center,
):
    sphere_center_local = cuboid_orientation.inverse.rotate(
        sphere_center - cuboid_center)

    half_dimensions = np.array(cuboid_size) / 2.0

    closest_point_local = np.maximum(
        -half_dimensions,
        np.minimum(half_dimensions, sphere_center_local),
    )

    closest_point_world = cuboid_center + \
        cuboid_orientation.rotate(closest_point_local)

    return closest_point_world


def angular_diff(angles1, angles2):
    diff = angles1 - angles2

    is_scalar = False
    if not isinstance(diff, np.ndarray):
        is_scalar = True
        diff = np.array([diff])

    diff[np.abs(diff) > np.abs((2*np.pi) + diff)] += 2*np.pi

    return diff[0] if is_scalar else diff


def compute_config_distance(config1, config2):
    pos_dist = float(np.linalg.norm(config1[:2] - config2[:2]))
    angle_diff = float(np.abs(angular_diff(config1[2], config2[2])))

    return (ALPHA * pos_dist) + ((1 - ALPHA) * angle_diff)


def parallel_run(func, arg_list):
    results = []
    with Pool(min(cpu_count(), len(arg_list))) as p:
        for result in tqdm(p.imap_unordered(func, arg_list), total=len(arg_list)):
            results.append(result)
    return results


def generate_random_config():
    random_pos = np.random.uniform(-50, 50, size=2)
    random_angle = np.random.uniform(-np.pi, np.pi)
    return np.array([random_pos[0], random_pos[1], random_angle])
