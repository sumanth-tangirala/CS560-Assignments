import heapq

from pyquaternion import Quaternion
from itertools import count

from constants import orange, yellow, purple, green
from create_scene import scene_from_file
from utils import is_arm_colliding, compute_config_diff, get_next_arm_positions, get_arm_pq_node
from visualizer.visualization import box as box_geom, sphere as sphere_geom
from visualizer.visualization.threejs_group import *



TIMESTEP = 0.05  # seconds
INTERPOLATION_TIME = .5  # second
GOAL_DISTANCE = 0.3
MIN_MOVE = 0.3

link_sizes = [
    np.array([2, 2, 0.5]),
    np.array([1, 1, 4]),
    np.array([1, 1, 4]),
]


tiebreaker = count()


def compute_link_configurations(angles: list[np.ndarray[np.dtype[np.floating]] | np.floating]) -> tuple[list[np.ndarray], list[Quaternion], np.ndarray]:
    joint1_quat = Quaternion(axis=[0, 0, 1], angle=angles[0])
    joint2_quat = Quaternion(axis=[0, 1, 0], angle=angles[1])
    joint3_quat = Quaternion(axis=[0, 1, 0], angle=angles[2])
    
    link1_quat = joint1_quat
    link2_quat = joint1_quat * joint2_quat
    link3_quat = link2_quat * joint3_quat

    link1_height = link_sizes[0][2]
    link2_height = link_sizes[1][2]
    link3_height = link_sizes[2][2]
    
    link1_center_wrt_world = np.array([0, 0, link1_height/2])
    link1_end_wrt_world = np.array([0, 0, link1_height])
    link2_center_wrt_l1 = np.array([0, 0, link2_height/2])
    link2_end_wrt_l1 = np.array([0, 0, link2_height])
    link3_center_wrt_l2 = np.array([0, 0, link3_height/2])
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

    return link_positions, link_quats, end_point_wrt_world


def interpolate_arm_movement(start_configuration, end_configuration):
    link1_start_angle = start_configuration[0]
    link2_start_angle = start_configuration[1]
    link3_start_angle = start_configuration[2]

    link1_end_angle = end_configuration[0]
    link2_end_angle = end_configuration[1]
    link3_end_angle = end_configuration[2]

    link1_angles = np.linspace(link1_start_angle, link1_end_angle, int(INTERPOLATION_TIME / TIMESTEP))
    link2_angles = np.linspace(link2_start_angle, link2_end_angle, int(INTERPOLATION_TIME / TIMESTEP))
    link3_angles = np.linspace(link3_start_angle, link3_end_angle, int(INTERPOLATION_TIME / TIMESTEP))

    link1_pos_traj = []
    link2_pos_traj = []
    link3_pos_traj = []
    link1_quat_traj = []
    link2_quat_traj = []
    link3_quat_traj = []

    for idx in range(len(link1_angles)):
        link1_angle = link1_angles[idx]
        link2_angle = link2_angles[idx]
        link3_angle = link3_angles[idx]

        link_positions, link_quats, end_point_wrt_world = compute_link_configurations(
            angles=[link1_angle, link2_angle, link3_angle]
        )

        link1_pos_traj.append(link_positions[0])
        link2_pos_traj.append(link_positions[1])
        link3_pos_traj.append(link_positions[2])

        link1_quat_traj.append(link_quats[0])
        link2_quat_traj.append(link_quats[1])
        link3_quat_traj.append(link_quats[2])


    return link1_pos_traj, link2_pos_traj, link3_pos_traj, link1_quat_traj, link2_quat_traj, link3_quat_traj


def visualize_scene(scene, link_positions, link_quats):
    viz_out = threejs_group(js_dir="../js")

    link1 = box_geom(
        name="link1",
        width=link_sizes[0][0],
        height=link_sizes[0][1],
        depth=link_sizes[0][2],
        position=link_positions[0],
        quaternion=link_quats[0]
    )

    link2 = box_geom(
        name="link2",
        width=link_sizes[1][0],
        height=link_sizes[1][1],
        depth=link_sizes[1][2],
        position=link_positions[1],
        quaternion=link_quats[1]
    )

    link3 = box_geom(
        name="link3",
        width=link_sizes[2][0],
        height=link_sizes[2][1],
        depth=link_sizes[2][2],
        position=link_positions[2],
        quaternion=link_quats[2]
    )

    viz_out.add_obstacle(link1, orange)
    viz_out.add_obstacle(link2, yellow)
    viz_out.add_obstacle(link3, purple)

    for i, sphere in enumerate(scene["obstacles"]):
        center = sphere[:3]
        radius = sphere[3]
        obstacle_geom = sphere_geom(f"sphere_{i}", radius, center)
        viz_out.add_obstacle(obstacle_geom, green)

    viz_out.to_html(f"./visualizer/out/arm.html")


def compute_arm_path(start_configuration, end_configuration, scene):
    start_link_positions, start_link_quats, _ = compute_link_configurations(start_configuration)
    end_link_positions, end_link_quats, _ = compute_link_configurations(end_configuration)

    if is_arm_colliding(start_link_positions, start_link_quats, link_sizes, scene):
        print('Start configuration is in collision')
        return None

    if is_arm_colliding(end_link_positions, end_link_quats, link_sizes, scene):
        print('End configuration is in collision')
        return None

    print('Query is valid')

    fringe = []
    visited = set(tuple(start_configuration))
    visited_list = [start_configuration]

    start_heuristic = compute_config_diff(start_configuration, end_configuration)
    heapq.heappush(fringe, (
        start_heuristic,
        next(tiebreaker),
        get_arm_pq_node(start_configuration, [start_configuration], 0, start_heuristic)
    ))

    while len(fringe):
        _, _, current = heapq.heappop(fringe)

        current_config = current["config"]
        # print(current_config)
        current_path = current["path"]
        current_cost = current["cost"]

        if np.all(np.abs(current_config - end_configuration) <= GOAL_DISTANCE):
            return current_path + [end_configuration]

        for next_config in get_next_arm_positions(current_config, MIN_MOVE):
            link_positions, link_quats, end_point_wrt_world = compute_link_configurations(next_config)
            if tuple(next_config) not in visited and not is_arm_colliding(link_positions, link_quats, link_sizes, scene):
                visited.add(tuple(next_config))
                visited_list.append(next_config)
                heuristic = compute_config_diff(next_config, end_configuration)
                total_cost = (current_cost + 0.1 + heuristic)
                priority_queue_node = get_arm_pq_node(next_config, current_path + [next_config], current_cost + 0.1, heuristic)
                heapq.heappush(fringe, (
                    total_cost,
                    next(tiebreaker),
                    priority_queue_node
                ))

    return None


def get_visualization_trajectory(path):
    link1_pos_traj = []
    link2_pos_traj = []
    link3_pos_traj = []
    link1_quat_traj = []
    link2_quat_traj = []
    link3_quat_traj = []
    link1_ani_step_traj = []
    link2_ani_step_traj = []
    link3_ani_step_traj = []

    for i in range(1, len(path)):
        link1_ipos_traj, link2_ipos_traj, link3_ipos_traj, link1_iquat_traj, link2_iquat_traj, link3_iquat_traj = interpolate_arm_movement(path[i-1], path[i])

        link1_pos_traj = link1_pos_traj + link1_ipos_traj
        link2_pos_traj = link2_pos_traj + link2_ipos_traj
        link3_pos_traj = link3_pos_traj + link3_ipos_traj
        link1_quat_traj = link1_quat_traj + link1_iquat_traj
        link2_quat_traj = link2_quat_traj + link2_iquat_traj
        link3_quat_traj = link3_quat_traj + link3_iquat_traj

    idx = 0
    for t in np.arange(0, TIMESTEP * len(link1_pos_traj), TIMESTEP):
        link1_ani_step_traj.append([
            t,
            link1_pos_traj[idx],
            link1_quat_traj[idx],
            orange,
        ])

        link2_ani_step_traj.append([
            t,
            link2_pos_traj[idx],
            link2_quat_traj[idx],
            yellow,
        ])

        link3_ani_step_traj.append([
            t,
            link3_pos_traj[idx],
            link3_quat_traj[idx],
            purple,
        ])

        idx += 1

    return link1_ani_step_traj, link2_ani_step_traj, link3_ani_step_traj


def visualize_arm_path(path, scene, scene_name):

    link1_ani_step_traj, link2_ani_step_traj, link3_ani_step_traj = get_visualization_trajectory(path)
    init_link_pos, init_link_quat, _ = compute_link_configurations(path[0])

    viz_out = threejs_group(js_dir="../js")

    link1 = box_geom(
        name="link1",
        width=link_sizes[0][0],
        height=link_sizes[0][1],
        depth=link_sizes[0][2],
        position=init_link_pos[0],
        quaternion=init_link_quat[0]
    )

    link2 = box_geom(
        name="link2",
        width=link_sizes[1][0],
        height=link_sizes[1][1],
        depth=link_sizes[1][2],
        position=init_link_pos[1],
        quaternion=init_link_quat[1]
    )

    link3 = box_geom(
        name="link3",
        width=link_sizes[2][0],
        height=link_sizes[2][1],
        depth=link_sizes[2][2],
        position=init_link_pos[2],
        quaternion=init_link_quat[2]
    )

    viz_out.add_animation(link1, link1_ani_step_traj)
    viz_out.add_animation(link2, link2_ani_step_traj)
    viz_out.add_animation(link3, link3_ani_step_traj)

    for i, sphere in enumerate(scene["obstacles"]):
        center = sphere[:3]
        radius = sphere[3]
        obstacle_geom = sphere_geom(f"sphere_{i}", radius, center)
        viz_out.add_obstacle(obstacle_geom, green)

    viz_out.to_html(f"./visualizer/out/arm_path_{scene_name}.html")


def main():
    scene_name = 'scene4'
    scene = scene_from_file(scene_name)

    start = np.array([0, 0, 0])
    end = np.array([-np.pi/2, np.pi/3, np.pi/6])

    path = compute_arm_path(start, end, scene)

    if path is None:
        print("No path found")
        return
    else:
        print('Path Found')

    visualize_arm_path(path, scene, scene_name)


if __name__ == '__main__':
    main()
