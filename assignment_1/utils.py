import numpy as np
from pyquaternion import Quaternion

from assignment_2.constants import lower_space_limits, upper_space_limits, ROBOT_SIZE


def closest_point_on_cuboid_to_sphere(cuboid_dimensions, cuboid_orientation, cuboid_center, sphere_center):
    sphere_center_local = cuboid_orientation.inverse.rotate(sphere_center - cuboid_center)

    half_dimensions = np.array(cuboid_dimensions) / 2.0

    closest_point_local = np.maximum(-half_dimensions, np.minimum(half_dimensions, sphere_center_local))

    closest_point_world = cuboid_center + cuboid_orientation.rotate(closest_point_local)

    return closest_point_world


def are_cuboid_and_sphere_overlapping(cuboid_center, cuboid_size, cuboid_orientation,  sphere_center, sphere_radius):
    closest_point = closest_point_on_cuboid_to_sphere(cuboid_size, cuboid_orientation, cuboid_center, sphere_center)

    distance = np.linalg.norm(closest_point - sphere_center)

    return distance <= sphere_radius


# Utils for 3d Rigid Body

ROBOT_DIMS = np.array([ROBOT_SIZE, ROBOT_SIZE, ROBOT_SIZE])
ROBOT_ORIENTATION = Quaternion(axis=[0, 0, 1], angle=0)


def is_robot_colliding(robot_position, scene):
    for obstacle in scene['obstacles']:
        if are_cuboid_and_sphere_overlapping(robot_position, ROBOT_DIMS, ROBOT_ORIENTATION, obstacle[:3], obstacle[3]):
            return True

    return False


def get_next_robot_positions(robot_position):
    next_moves = [
        [-5, 0, 0],
        [5, 0, 0],
        [0, -5, 0],
        [0, 5, 0],
        [0, 0, -5],
        [0, 0, 5]
    ]

    possible_positions = []

    for position in next_moves:
        new_position = [robot_position[0] + position[0], robot_position[1] + position[1], robot_position[2] + position[2]]
        if (
                lower_space_limits[0] <= new_position[0] <= upper_space_limits[0] and
                lower_space_limits[1] <= new_position[1] <= upper_space_limits[1] and
                lower_space_limits[2] <= new_position[2] <= upper_space_limits[2]
        ):
            possible_positions.append(np.array(new_position))

    return possible_positions


def get_robot_pq_node(position, path, cost, heuristic):
    return {
        'position': position,
        'path': path,
        'cost': cost,
        'heuristic': heuristic
    }

# Utils for Arm


def check_link_collision(link_position, link_quat, link_size, obstacle):
    # Compute the 8 vertices of the link and check if any of them is inside the obstacle
    return are_cuboid_and_sphere_overlapping(link_position, link_size, link_quat, obstacle[:3], obstacle[3])


def is_arm_colliding(link_positions, link_quats, link_sizes, scene):
    for obstacle in scene['obstacles']:
        for i in range(3):
            if check_link_collision(link_positions[i], link_quats[i], link_sizes[i], obstacle):
                return True
    return False


def compute_angle_diff(theta1, theta2):
    difference = np.abs((theta1 - theta2) % (2 * np.pi))

    return min(difference, 2 * np.pi - difference)


def compute_config_diff(config1, config2):
    return (
            compute_angle_diff(config1[0], config2[0]) +
            compute_angle_diff(config1[1], config2[1]) +
            compute_angle_diff(config1[2], config2[2])
    )


def get_arm_pq_node(config, path, cost, heuristic):
    return {
        'config': config,
        'path': path,
        'cost': cost,
        'heuristic': heuristic
    }


ARM_MOVES = [
    np.array([-1, 0, 0]),
    np.array([1, 0, 0]),
    np.array([0, -1, 0]),
    np.array([0, 1, 0]),
    np.array([0, 0, -1]),
    np.array([0, 0, 1]),
]


def get_next_arm_positions(current_config, min_move):
    next_configs = []

    for move in ARM_MOVES:
        new_config = current_config + (move * min_move)
        if (
                lower_space_limits[0] <= new_config[0] <= upper_space_limits[0] and
                lower_space_limits[1] <= new_config[1] <= upper_space_limits[1] and
                lower_space_limits[2] <= new_config[2] <= upper_space_limits[2]
        ):
            next_configs.append(new_config)

    return next_configs


