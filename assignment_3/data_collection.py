import argparse
import numpy as np

import tqdm

from utils import compute_config_distance, generate_random_config
from car_prop import Car
from constants import MIN_START_GOAL_DIST
from rrt import RRT
from map_generator import load_map


def generate_plan():
    start = generate_random_config()
    goal = generate_random_config()

    print(start, goal)

    while np.linalg.norm(start[:2] - goal[:2]) < MIN_START_GOAL_DIST:
        goal = generate_random_config()

    print('Goal Distance:', np.linalg.norm(start[:2] - goal[:2]))

    rrt = RRT(start, goal)
    rrt.run()

    while not rrt.trajectory_found:
        print('RRT Failed. Retrying')
        rrt = RRT(start, goal)
        rrt.run()

    return rrt.trajectory, rrt.controls


def visualize_data(car: Car, name):
    car.visualize_trajectory(f'data/trajectory_{name}')
    car.plot_trajectory(f'data/trajectory_{name}')


def generate_data(map_path, problem, noise):
    landmarks = load_map(map_path)

    print('Starting Plan Generation')

    trajectory, controls = generate_plan()
    start = trajectory[0]

    print('Plan Generated')

    car = Car(
        start=start,
        landmarks=landmarks,
        noise=noise,
    )

    print('Starting Plan Execution')

    for control in controls:
        car.step(control)

    print('Plan Executed')

    ground_truth = car.traj
    odometry_controls = car.odo_controls
    observations = car.observations

    with open(f'data/plan_{problem}_{noise}.txt', 'w', encoding='utf8') as f:
        f.write(f'{start[0]} {start[1]} {start[2]}\n')

        for control in controls:
            f.write(f'{control[0]} {control[1]}\n')

    with open(f'data/gt_{problem}_{noise}.txt', 'w', encoding='utf8') as f:
        for state in ground_truth:
            f.write(f'{state[0]} {state[1]} {state[2]}\n')

    with open(f'data/odometry_{problem}_{noise}.txt', 'w', encoding='utf8') as f:
        for control in odometry_controls:
            f.write(f'{control[0]} {control[1]}\n')

    with open(f'data/landmarks_{problem}_{noise}.txt', 'w', encoding='utf8') as f:
        for observation in observations:
            for landmark in observation:
                f.write(f'{landmark[0]} {landmark[1]} ')

            f.write('\n')

    visualize_data(car, f'{problem}_{noise}')


# def main():
#     arg_parser = argparse.ArgumentParser()
#     arg_parser.add_argument('--map', type=str, required=True)
#     arg_parser.add_argument('--problem', type=int, required=True)
#     arg_parser.add_argument('--noise', type=str, required=True)

#     parsed_args = arg_parser.parse_args()

#     generate_data(parsed_args.map, parsed_args.problem, parsed_args.noise)

def main():
    for problem in tqdm.tqdm(range(10)):
        map_num = problem//2
        map_path = f'maps/map{map_num}.txt'
        noise = 'H' if problem % 2 == 0 else 'L'

        generate_data(map_path, problem, noise)


if __name__ == '__main__':
    main()
