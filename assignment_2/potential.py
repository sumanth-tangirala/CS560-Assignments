import argparse
import numpy as np

from constants import ARM_LINK_SIZES
from utils import angular_diff, closest_point_on_cuboid_to_sphere, compute_link_configurations
from utils.graphNode import sanitize_angles
from utils.visualization_utils import visualize_arm_trajectory, visualize_obstacles

from visualizer.visualization.threejs_group import *

ATTRACTION_CONSTANT = 0.1
REPLUSION_CONSTANT  = 0.01

MAX_ITERATIONS = 1e6

INFLUENCE_DIST = 0.5
GOAL_DIST = 0.01


def compute_attraction_gradient(config, goal):
    goal_angle_dist = angular_diff(config, goal)

    if np.linalg.norm(goal_angle_dist) < GOAL_DIST:
        return np.zeros(3)

    return ATTRACTION_CONSTANT * goal_angle_dist/(2* np.linalg.norm(goal_angle_dist))

def compute_repulsion_gradient(config, obstacle):
    link_positions, link_orientations = compute_link_configurations(config)

    minimum_obstacle_distance = np.inf

    for link_position, link_orientation, link_size in zip(link_positions, link_orientations, ARM_LINK_SIZES):
        closest_point = closest_point_on_cuboid_to_sphere(link_position, link_orientation, link_size, obstacle[:3])

        distance = np.linalg.norm(closest_point - obstacle[:3]) - obstacle[3]

        minimum_obstacle_distance = min(minimum_obstacle_distance, distance)

    if minimum_obstacle_distance > INFLUENCE_DIST:
        return 0
    
    return REPLUSION_CONSTANT * ((1/INFLUENCE_DIST) - (1/minimum_obstacle_distance))/(minimum_obstacle_distance**2)


def perform_potential_planning(start, goal, obstacle):
    current = start

    path = [current.copy()]

    count = 0
    while True:
        attr_grad = compute_attraction_gradient(current, goal)
        repl_grad = compute_repulsion_gradient(current, obstacle)
        gradient = attr_grad + repl_grad

        print(current, attr_grad, repl_grad)
        current -= gradient

        current = sanitize_angles(current)

        path.append(current.copy())

        if np.linalg.norm(gradient) < 1e-4:
            break

        if count > MAX_ITERATIONS:
            return None

        count += 1
    
    return path

def visualize_path(path, obstacle):
    viz_out = threejs_group(js_dir="../js")
    visualize_obstacles([obstacle], viz_out)
    visualize_arm_trajectory(path, viz_out)
    viz_out.to_html(f"./visualizer/out/potential_func.html")

def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--start', type=str, required=True, nargs='+')
    arg_parser.add_argument('--goal', type=str, required=True, nargs='+')

    parsed_args = arg_parser.parse_args()

    if len(parsed_args.start) != 3:
        raise ValueError('Start state for arm should have 3 values')

    if len(parsed_args.goal) != 3:
        raise ValueError('Goal state for arm should have 3 values')

    start = sanitize_angles(np.array([float(x) for x in parsed_args.start]))
    goal = sanitize_angles(np.array([float(x) for x in parsed_args.goal]))
    obstacle = np.array([0, 0, 4, 1])
    
    path = perform_potential_planning(start, goal, obstacle)

    if path is None:
        print('No Path Found')
        return
    
    visualize_path(path, obstacle)


if __name__ == '__main__':
    main()