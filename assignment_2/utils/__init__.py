from itertools import count
from math import ceil
from queue import PriorityQueue

import numpy as np
from pyquaternion import Quaternion

from constants import ARM_LINK_SIZES, ALPHA, NUM_ARM_INTERPOLATIONS, VEHICLE_INTERPOLATION_DIST
from utils.graphNode import GraphNode
from utils.rotation_utils import convert_quat_to_euler

tiebreaker = count()

def closest_point_on_cuboid_to_sphere(cuboid_center, cuboid_orientation, cuboid_size, sphere_center):
    sphere_center_local = cuboid_orientation.inverse.rotate(sphere_center - cuboid_center)

    half_dimensions = np.array(cuboid_size) / 2.0

    closest_point_local = np.maximum(-half_dimensions, np.minimum(half_dimensions, sphere_center_local))

    closest_point_world = cuboid_center + cuboid_orientation.rotate(closest_point_local)

    return closest_point_world


def compute_link_configurations(angles):
    joint1_quat = Quaternion(axis=[0, 0, 1], angle=angles[0])
    joint2_quat = Quaternion(axis=[0, 1, 0], angle=angles[1])
    joint3_quat = Quaternion(axis=[0, 1, 0], angle=angles[2])

    link1_quat = joint1_quat
    link2_quat = joint1_quat * joint2_quat
    link3_quat = link2_quat * joint3_quat

    link1_height = ARM_LINK_SIZES[0][2]
    link2_height = ARM_LINK_SIZES[1][2]
    link3_height = ARM_LINK_SIZES[2][2]

    link1_center_wrt_world = np.array([0, 0, link1_height / 2])
    link1_end_wrt_world = np.array([0, 0, link1_height])
    link2_center_wrt_l1 = np.array([0, 0, link2_height / 2])
    link2_end_wrt_l1 = np.array([0, 0, link2_height])
    link3_center_wrt_l2 = np.array([0, 0, link3_height / 2])
    end_point_wrl_l2 = np.array([0, 0, link3_height])

    link2_center_wrt_world = link2_quat.rotate(link2_center_wrt_l1) + link1_end_wrt_world
    link2_end_wrt_world = link2_quat.rotate(link2_end_wrt_l1) + link1_end_wrt_world
    link3_center_wrt_world = link3_quat.rotate(link3_center_wrt_l2) + link2_end_wrt_world
    end_point_wrt_world = link3_quat.rotate(end_point_wrl_l2) + link2_end_wrt_world

    link_positions = [
        link1_center_wrt_world,
        link2_center_wrt_world,
        link3_center_wrt_world
    ]

    link_quats = [
        link1_quat,
        link2_quat,
        link3_quat
    ]

    return link_positions, link_quats

def angular_diff(angles1, angles2):
    diff = (angles1 - angles2)

    is_scalar = False
    if type(diff) is not np.ndarray:
        is_scalar = True
        diff = np.array([diff])

    diff[np.abs(diff) > np.abs((2*np.pi) + diff)] += 2*np.pi
    
    return diff[0] if is_scalar else diff

def compute_arm_distance(config1, config2):
    abs_difference = np.abs((config1 - config2))

    difference = np.minimum(abs_difference, 2 * np.pi - abs_difference)

    return np.sum(difference)


def compute_vehicle_distance(config1, config2):
    abs_pos_diff = np.sum(np.linalg.norm(config1[:3] - config2[:3]))

    config1_angle = convert_quat_to_euler(config1[3:], degrees=False)
    config2_angle = convert_quat_to_euler(config2[3:], degrees=False)

    abs_angle_diff = np.abs((config1_angle - config2_angle)) % (2 * np.pi)

    angle_diff = np.sum(np.minimum(abs_angle_diff, 2 * np.pi - abs_angle_diff))

    return ALPHA * abs_pos_diff + (1 - ALPHA) * angle_diff


def read_configs_file(file_path):
    loaded_configs = []

    with open(file_path, 'r') as file:
        for line in file:
            line = line.split('#')[0]
            line = line.strip()
            if line:
                elements = line.split(' ')
                elements = [el.strip() for el in elements if el.strip()]
                loaded_configs.append(np.array([eval(val) for val in elements]).astype(np.float64))

    return np.array(loaded_configs)


def a_star(start_node: GraphNode, goal_node: GraphNode):
    fringe = PriorityQueue()
    visited = {start_node}

    start_heuristic = start_node.compute_distance(goal_node)
    fringe.put((
        start_heuristic,
        next(tiebreaker),
        start_node,
        0,
        [start_node],
    ))

    while not fringe.empty():
        current_heuristic, _, current_node, current_cost, current_path = fringe.get()

        if current_node == goal_node:
            return current_path

        for next_node in current_node.adjacent:
            if next_node not in visited:
                visited.add(next_node)
                next_cost = current_cost + next_node.compute_distance(current_node)
                total_estimated_cost = next_cost + next_node.compute_distance(goal_node)

                fringe.put((
                    total_estimated_cost,
                    next(tiebreaker),
                    next_node,
                    next_cost,
                    current_path + [next_node]
                ))

    return None


