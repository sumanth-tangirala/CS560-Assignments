import time

from pyquaternion import Quaternion

from assignment_2.constants import ROBOT_SIZE
from create_scene import *
from utils import are_cuboid_and_sphere_overlapping
from visualizer.visualization import box as box_geom

TIMESTEP = 0.5
TOTAL_TIME = 10

np.set_printoptions(suppress=True)


def check_collision(cube, sphere):
    sphere_center = sphere[:3]
    sphere_radius = sphere[3]
    cube_center = cube
    cube_size = np.array([ROBOT_SIZE, ROBOT_SIZE, ROBOT_SIZE])
    cube_orientation = Quaternion(axis=[0, 0, 1], angle=0)

    return are_cuboid_and_sphere_overlapping(cube_center, cube_size, cube_orientation, sphere_center, sphere_radius)


def visualize_collisions(scene, scene_name="scene"):
    viz_out = threejs_group(js_dir="../js")

    obstacles = scene["obstacles"]
    obstacle_trajectories = scene['obstacle_trajectories']
    robot_trajectory = scene['robot_trajectory']

    robot_center = scene['robot']

    robot = box_geom("robot", ROBOT_SIZE, ROBOT_SIZE, ROBOT_SIZE, robot_center)
    viz_out.add_animation(robot, robot_trajectory)

    for i, sphere in enumerate(obstacles):
        center = sphere[:3]
        radius = sphere[3]
        obstacle_geom = sphere_geom(f"sphere_{i}", radius, center)

        viz_out.add_animation(obstacle_geom, obstacle_trajectories[i])

    viz_out.to_html(f"./visualizer/out/{scene_name}.html")


def main():
    for i in range(1, 7):
        scene_name = f'scene{i}'

        scene = scene_from_file(scene_name)
        obstacle_trajectories = [[] for _ in scene['obstacles']]
        robot_trajectory = []

        obstacle_durations = []
        scene_durations = []

        for t in np.arange(0, TOTAL_TIME, TIMESTEP):
            robot_center_upper_limits = upper_space_limits - 2.5
            robot_center_lower_limits = lower_space_limits + 2.5

            scene['robot'] = np.random.uniform(robot_center_lower_limits, robot_center_upper_limits, 3)

            scene_time = time.time()
            for obs_idx, obstacle in enumerate(scene['obstacles']):
                obs_time = time.time()

                color = red if check_collision(scene['robot'], obstacle) else green

                obstacle_durations.append(time.time() - obs_time)

                obstacle_trajectories[obs_idx].append([
                    t,
                    obstacle[:3],
                    [1, 0, 0, 0],
                    color
                ])

            scene_durations.append(time.time() - scene_time)

            robot_trajectory.append([
                t,
                scene['robot'],
                [1, 0, 0, 0],
                blue
            ])

        scene['obstacle_trajectories'] = obstacle_trajectories
        scene['robot_trajectory'] = robot_trajectory

        visualize_collisions(scene, scene_name=scene_name)

        obstacle_durations = np.array(obstacle_durations)
        scene_durations = np.array(scene_durations)

        print(f"Per Obstacle duration ({scene_name}): {obstacle_durations.mean()} +/- {obstacle_durations.std()}")
        print(f"Per Scene duration ({scene_name}): {scene_durations.mean()} +/- {scene_durations.std()}")


if __name__ == '__main__':
    main()
