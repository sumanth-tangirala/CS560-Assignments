from math import ceil
from typing import List

import numpy as np

from constants import CAR_DIMS, Z_VALUE, green, ARM_LINK_SIZES, orange, yellow, purple, VEHICLE_SIZE, blue, red, \
    VEHICLE_INTERPOLATION_TIME, TIMESTEP, ARM_INTERPOLATION_TIME, grey
from utils import compute_link_configurations, GraphNode
from utils.rotation_utils import convert_euler_to_quat
from visualizer import sphere as sphere_geom, box as box_geom


def visualize_obstacles(obstacles, viz_out):
    for i, sphere in enumerate(obstacles):
        center = sphere[:3]
        radius = sphere[3]
        geom = sphere_geom(f"sphere_{i}", radius, center, [1, 0, 0, 0])
        viz_out.add_obstacle(geom, green)


def visualize_arm_config(config, viz_out, name_prefix="", colors=None):
    link_positions, link_quats = compute_link_configurations(config)

    if colors is None:
        colors = [orange, yellow, purple]

    link1 = box_geom(
        name=name_prefix + "link1",
        width=ARM_LINK_SIZES[0][0],
        height=ARM_LINK_SIZES[0][1],
        depth=ARM_LINK_SIZES[0][2],
        position=link_positions[0],
        quaternion=link_quats[0]
    )

    link2 = box_geom(
        name=name_prefix + "link2",
        width=ARM_LINK_SIZES[1][0],
        height=ARM_LINK_SIZES[1][1],
        depth=ARM_LINK_SIZES[1][2],
        position=link_positions[1],
        quaternion=link_quats[1]
    )

    link3 = box_geom(
        name=name_prefix + "link3",
        width=ARM_LINK_SIZES[2][0],
        height=ARM_LINK_SIZES[2][1],
        depth=ARM_LINK_SIZES[2][2],
        position=link_positions[2],
        quaternion=link_quats[2]
    )

    viz_out.add_obstacle(link1, colors[0])
    viz_out.add_obstacle(link2, colors[1])
    viz_out.add_obstacle(link3, colors[2])


def visualize_vehicle(config, viz_out, name="vehicle", color=blue):
    vehicle = box_geom(
        name=name,
        width=VEHICLE_SIZE[0],
        height=VEHICLE_SIZE[1],
        depth=VEHICLE_SIZE[2],
        position=config[:3],
        quaternion=config[3:]
    )

    viz_out.add_obstacle(vehicle, color)


def visualize_graph(graph, is_arm, viz_out):
    visualized_lines = dict()

    if is_arm:
        for i, node in enumerate(graph):
            end_point = node.end_point
            radius = 0.5

            geom = sphere_geom(f"node_{i}", radius, end_point, blue)
            viz_out.add_obstacle(geom, green)

            for neighbor in node.adjacent:
                if (node, neighbor) in visualized_lines:
                    continue

                line = [node.end_point, neighbor.end_point]
                viz_out.add_line(line, red)

                visualized_lines[(node, neighbor)] = True
                visualized_lines[(neighbor, node)] = True

    else:
        for i, node in enumerate(graph):
            visualize_vehicle(node.config, viz_out, name=f"vehicle_{i}")
            for neighbor in node.adjacent:
                if (node, neighbor) in visualized_lines:
                    continue

                line = [node.config[:3], neighbor.config[:3]]
                viz_out.add_line(line, red)

                visualized_lines[(node, neighbor)] = True
                visualized_lines[(neighbor, node)] = True


def visualize_vehicle_trajectory(trajectory, viz_out):
    animation_trajectory = []

    robot = box_geom(
        name="vehicle",
        width=VEHICLE_SIZE[0],
        height=VEHICLE_SIZE[1],
        depth=VEHICLE_SIZE[2],
        position=trajectory[0, :3],
        quaternion=trajectory[0, 3:]
    )

    idx = 0
    for t in np.arange(0, TIMESTEP * len(trajectory), TIMESTEP):
        animation_trajectory.append([
            t,
            trajectory[idx][:3],
            trajectory[idx][3:],
            blue,
        ])
        idx += 1

    viz_out.add_animation(robot, animation_trajectory)


def visualize_arm_trajectory(trajectory, viz_out):
    link1_pos_traj, link1_quat_traj, link1_animation_traj = [], [], []
    link2_pos_traj, link2_quat_traj, link2_animation_traj = [], [], []
    link3_pos_traj, link3_quat_traj, link3_animation_traj = [], [], []

    for config in trajectory:
        link_positions, link_quats = compute_link_configurations(config)
        link1_pos_traj.append(link_positions[0])
        link1_quat_traj.append(link_quats[0])
        link2_pos_traj.append(link_positions[1])
        link2_quat_traj.append(link_quats[1])
        link3_pos_traj.append(link_positions[2])
        link3_quat_traj.append(link_quats[2])

    link1 = box_geom(
        name="link1",
        width=ARM_LINK_SIZES[0][0],
        height=ARM_LINK_SIZES[0][1],
        depth=ARM_LINK_SIZES[0][2],
        position=link1_pos_traj[0],
        quaternion=link1_quat_traj[0]
    )

    link2 = box_geom(
        name="link2",
        width=ARM_LINK_SIZES[1][0],
        height=ARM_LINK_SIZES[1][1],
        depth=ARM_LINK_SIZES[1][2],
        position=link2_pos_traj[1],
        quaternion=link2_quat_traj[1]
    )

    link3 = box_geom(
        name="link3",
        width=ARM_LINK_SIZES[2][0],
        height=ARM_LINK_SIZES[2][1],
        depth=ARM_LINK_SIZES[2][2],
        position=link3_pos_traj[2],
        quaternion=link3_quat_traj[2]
    )

    idx = 0
    for t in np.arange(0, TIMESTEP * len(link1_pos_traj), TIMESTEP):
        if idx >= len(link1_pos_traj):
            break
        link1_animation_traj.append([
            t,
            link1_pos_traj[idx],
            link1_quat_traj[idx],
            orange,
        ])

        link2_animation_traj.append([
            t,
            link2_pos_traj[idx],
            link2_quat_traj[idx],
            yellow,
        ])

        link3_animation_traj.append([
            t,
            link3_pos_traj[idx],
            link3_quat_traj[idx],
            purple,
        ])

        idx += 1

    viz_out.add_animation(link1, link1_animation_traj)
    viz_out.add_animation(link2, link2_animation_traj)
    viz_out.add_animation(link3, link3_animation_traj)


def visualize_path(path: List[GraphNode], is_arm, viz_out):
    trajectory = []

    num_interpolations = ceil(
        ARM_INTERPOLATION_TIME / TIMESTEP) if is_arm else None

    for i in range(len(path) - 1):
        interpolated_configs = path[i].interpolate_configs(
            path[i+1], num_interpolations)
        trajectory.extend(interpolated_configs)

    trajectory = np.array(trajectory)

    if is_arm:
        visualize_arm_trajectory(trajectory, viz_out)
    else:
        visualize_vehicle_trajectory(trajectory, viz_out)


def visualize_car_trajectory(path: List[np.ndarray], viz_out):
    trajectory = []
    path_line = []

    for state in path:
        pos_config = [state[0], state[1], Z_VALUE]
        quat_config = convert_euler_to_quat([0, 0, state[2]], degrees=False)

        config = np.concatenate([pos_config, quat_config])

        trajectory.append(config)
        path_line.append(pos_config)

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
    for t in np.arange(0, TIMESTEP * len(trajectory), TIMESTEP):
        if idx >= len(trajectory):
            break

        animation_trajectory.append([
            t,
            trajectory[idx][:3],
            trajectory[idx][3:],
            blue,
        ])
        idx += 1

    breakpoint()

    viz_out.add_animation(car, animation_trajectory)
    viz_out.add_line(path_line, grey)
