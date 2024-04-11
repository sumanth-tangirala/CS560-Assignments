from math import ceil

import numpy as np
from pyquaternion import Quaternion

from constants import lower_space_limits, upper_space_limits, NUM_ARM_INTERPOLATIONS, VEHICLE_INTERPOLATION_DIST


class GraphNode:
    def __init__(self, config=None, is_arm: bool = False, adjacent=(), random=False):
        self.neighbors = list()
        self.safe_neighbors = set()
        self._is_arm = is_arm
        self.collision_idx = None

        if not random:
            if config is None:
                raise ValueError("Config must be provided if random is False")
            if is_arm:
                # Sanitize the angles to be in the range [-pi, pi]
                self.config = sanitize_angles(config)

            else:
                self.config = np.array(config)

        if random:
            if is_arm:
                self.config = np.random.uniform([-np.pi, -np.pi, -np.pi], [np.pi, np.pi, np.pi], 3)
            else:
                self.config = np.array([
                    *np.random.uniform(lower_space_limits, upper_space_limits, 3),
                    *Quaternion.random().elements,
                ])

        if is_arm:
            from utils import compute_link_configurations
            self.link_positions, self.link_orientations, self.end_point = compute_link_configurations(self.config, return_end_point=True)
        else:
            self.position = self.config[:3]
            self.orientation = Quaternion(self.config[3:])

        self.adjacent = set(adjacent).copy()

    def __str__(self):
        return str(self.config)

    def __repr__(self):
        return str(self.config)

    def __hash__(self):
        return hash(tuple(self.config))

    def add_adjacent(self, adj_node):
        self.adjacent.add(adj_node)

    def compute_distance(self, other):
        if self._is_arm:
            from utils import compute_arm_distance
            return compute_arm_distance(self.config, other.config)
        else:
            from utils import compute_vehicle_distance
            return compute_vehicle_distance(self.config, other.config)

    def interpolate_configs(self, other, num_interpolations=None):
        if self._is_arm:
            return interpolate_arm_configs(self, other, num_interpolations)
        else:
            return interpolate_vehicle_configs(self, other, num_interpolations)


def sanitize_angles(angles):
    sanitized_angles = np.array([angle % (2 * np.pi) for angle in angles])

    for i in range(len(sanitized_angles)):
        if sanitized_angles[i] > np.pi:
            sanitized_angles[i] -= 2 * np.pi

        if sanitized_angles[i] < -np.pi:
            sanitized_angles[i] += 2 * np.pi

    return np.array(sanitized_angles)


def interpolate_arm_configs(node1: GraphNode, node2: GraphNode, num_interpolations=None):
    if num_interpolations is None:
        num_interpolations = NUM_ARM_INTERPOLATIONS

    node1_config = node1.config
    node2_config = node2.config

    diff = node2_config - node1_config

    goal_config = node2_config.copy()
    goal_config[diff < -np.pi] += 2 * np.pi
    goal_config[diff > np.pi] -= 2 * np.pi

    return np.concatenate((
        np.linspace(node1_config, goal_config, num_interpolations)[:-1],
        node2_config[None, :]
    ), axis=0)


def interpolate_vehicle_configs(node1: GraphNode, node2: GraphNode, num_interpolations=None):
    distance = np.linalg.norm(node2.position - node1.position)

    if num_interpolations is None:
        num_interpolations = max(ceil(distance / VEHICLE_INTERPOLATION_DIST), 3)

    interpolated_configs = np.zeros((num_interpolations, 7))

    positions = np.linspace(node1.position, node2.position, num_interpolations)

    quats = Quaternion.intermediates(
        node1.orientation,
        node2.orientation,
        num_interpolations - 2,
        include_endpoints=True
    )

    quats_array = np.array([quat.elements for quat in quats])

    interpolated_configs[:, :3] = positions
    interpolated_configs[:, 3:] = quats_array

    return interpolated_configs



