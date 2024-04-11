import numpy as np
from pyquaternion import Quaternion

from constants import CAR_DIMS, VEHICLE_SIZE, ARM_LINK_SIZES, Z_VALUE
from utils.__init__ import compute_link_configurations, closest_point_on_cuboid_to_sphere
from utils.graphNode import GraphNode

def are_cuboid_and_sphere_overlapping(cuboid_center, cuboid_orientation, cuboid_size, sphere_center, sphere_radius):
    closest_point = closest_point_on_cuboid_to_sphere(cuboid_center, cuboid_orientation, cuboid_size, sphere_center)

    distance = np.linalg.norm(closest_point - sphere_center)

    return distance <= sphere_radius


def is_vehicle_colliding(graph_node: GraphNode = None, obstacle=None, config=None):
    if obstacle is None:
        raise ValueError("Obstacle must be provided")

    if config is None and graph_node is None:
        raise ValueError("Either a graph node or a configuration must be provided")

    if graph_node is None:
        robot_position = config[:3]
        robot_orientation = Quaternion(config[3:])
    else:
        robot_position = graph_node.position
        robot_orientation = graph_node.orientation

    return are_cuboid_and_sphere_overlapping(robot_position, robot_orientation, VEHICLE_SIZE, obstacle[:3], obstacle[3])


def is_car_colliding(config, obstacles):
    for obstacle in obstacles:
        position = np.array([*config[:2], Z_VALUE])
        orientation = Quaternion(axis=[0, 0, 1], angle=config[2])
        if are_cuboid_and_sphere_overlapping(position, orientation, CAR_DIMS, obstacle[:3], obstacle[3]):
            return True

    return False


def check_link_collision(link_position, link_quat, link_size, obstacle):
    # Compute the 8 vertices of the link and check if any of them is inside the obstacle
    return are_cuboid_and_sphere_overlapping(link_position, link_quat, link_size, obstacle[:3], obstacle[3])


def is_arm_colliding(graph_node: GraphNode=None, obstacle=None, config=None):
    if obstacle is None:
        raise ValueError("Obstacle must be provided")

    if config is None and graph_node is None:
        raise ValueError("Either a graph node or a configuration must be provided")

    if graph_node is None:
        link_positions, link_quats = compute_link_configurations(config)
    else:
        link_positions = graph_node.link_positions
        link_quats = graph_node.link_orientations

    for link_position, link_quat, link_size in zip(link_positions, link_quats, ARM_LINK_SIZES):
        if are_cuboid_and_sphere_overlapping(link_position, link_quat, link_size, obstacle[:3], obstacle[3]):
            return True

    return False


def is_node_in_collision(graph_node: GraphNode, obstacles, is_arm: bool):
    collision_check_func = is_arm_colliding if is_arm else is_vehicle_colliding
    for i, obstacle in enumerate(obstacles):
        if collision_check_func(graph_node, obstacle):
            return True

    return False


def is_edge_in_collision(node1: GraphNode, node2: GraphNode, obstacles, is_arm: bool, cache=True):
    collision_check_func = is_arm_colliding if is_arm else is_vehicle_colliding

    if cache and node2 in node1.safe_neighbors or node1 in node2.safe_neighbors:
        return False

    interpolated_configs = node1.interpolate_configs(node2)

    for obstacle in obstacles:
        for config in interpolated_configs:
            if collision_check_func(config=config, obstacle=obstacle):
                return True

    if cache:
        node1.safe_neighbors.add(node2)

    return False
