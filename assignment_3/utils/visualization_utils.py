from math import ceil
from typing import Iterable

from pyquaternion import Quaternion

import numpy as np

from constants import CAR_DIMS, Z_VALUE, green, blue, VISUALIZATION_TIMESTEP, grey, INTERPOLATION_FACTOR, red
from utils.rotation_utils import convert_euler_to_quat
from visualizer import sphere as sphere_geom, box as box_geom


def visualize_obstacles(obstacles, viz_out):
    for i, sphere in enumerate(obstacles):
        center = sphere[:3]
        radius = sphere[3]
        geom = sphere_geom(f"sphere_{i}", radius, center, [1, 0, 0, 0])
        viz_out.add_obstacle(geom, green)


def visualize_car_trajectory(
    path: np.ndarray,
    viz_out,
    secondary_path: Iterable[np.ndarray] = (),
    estimated_landmarks=None,
):
    trajectory = []
    path_line = []
    secondary_path_line = []

    if len(secondary_path):
        for state in secondary_path:
            secondary_path_line.append([*state[:2], Z_VALUE])

        viz_out.add_line(secondary_path_line, blue)

    prev_pos = path[0, :2]
    prev_quat = Quaternion(convert_euler_to_quat(
        [0, 0, path[0, 2]], degrees=False))

    for state in path[1:]:
        curr_pos = state[:2]
        curr_quat = Quaternion(convert_euler_to_quat(
            [0, 0, state[2]], degrees=False))

        interpolated_states = np.zeros((INTERPOLATION_FACTOR, 7))

        positions = np.linspace(
            prev_pos, curr_pos, INTERPOLATION_FACTOR)

        quats = Quaternion.intermediates(
            prev_quat,
            curr_quat,
            INTERPOLATION_FACTOR - 2,
            include_endpoints=True
        )

        quats_array = np.array([quat.elements for quat in quats])

        interpolated_states[:, :2] = positions
        interpolated_states[:, 2] = Z_VALUE
        interpolated_states[:, 3:] = quats_array

        trajectory.extend(interpolated_states)
        path_line.append([*curr_pos, Z_VALUE])

        prev_pos = curr_pos
        prev_quat = curr_quat

    trajectory = np.array(trajectory)

    car = box_geom(
        name="car",
        width=CAR_DIMS[0],
        height=CAR_DIMS[1],
        depth=CAR_DIMS[2],
        position=trajectory[0, :3],
        quaternion=trajectory[0, 3:]
    )

    animation_trajectory = []

    idx = 0
    for t in np.arange(0, VISUALIZATION_TIMESTEP * len(trajectory), VISUALIZATION_TIMESTEP):
        if idx >= len(trajectory):
            break

        animation_trajectory.append([
            t,
            trajectory[idx][:3],
            trajectory[idx][3:],
            blue,
        ])
        idx += 1

    viz_out.add_animation(car, animation_trajectory)
    viz_out.add_line(path_line, grey)

    if len(estimated_landmarks) == 0:
        return

    landmark_geoms = []
    landmarks_animations = []

    for i, landmark in enumerate(estimated_landmarks[0]):
        landmarks_animations.append([])
        landmark_geoms.append(
            sphere_geom(
                f"landmark_{i}",
                1,
                [*landmark, Z_VALUE],
                [1, 0, 0, 0]
            )
        )

    idx = 0

    for t in np.arange(0, VISUALIZATION_TIMESTEP * len(trajectory), VISUALIZATION_TIMESTEP*INTERPOLATION_FACTOR):
        if idx >= len(estimated_landmarks):
            break

        for i, landmark in enumerate(estimated_landmarks[idx]):
            landmarks_animations[i].append([
                t,
                [*landmark, Z_VALUE],
                [1, 0, 0, 0],
                red,
            ])

        idx += 1

    for i, landmark_geom in enumerate(landmark_geoms):
        viz_out.add_animation(landmark_geom, landmarks_animations[i])
