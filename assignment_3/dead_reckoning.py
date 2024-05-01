import argparse

import matplotlib.pyplot as plt
import numpy as np

from map_generator import load_map
from car_prop import Car


def perform_dead_reckoning(start, odometry, landmarks, gt, viz_file):
    car = Car(
        start=start,
        landmarks=landmarks,
        noise=None,
        gt=gt,
    )

    estimated_landmarks = []

    for i, control in enumerate(odometry):
        actual_state = gt[i]
        car.step(control)
        state = car.state

        state_estimated_landmarks = []

        for landmark in landmarks:
            # Estimate landmark position and orientation in actual car frame
            landmark_pos = landmark[:2] - actual_state[:2]
            landmark_dist = np.linalg.norm(landmark_pos)
            landmark_angle = np.arctan2(
                landmark_pos[1], landmark_pos[0]) - actual_state[2]

            # Estimate landmark position and orientation in estimated car frame

            estimated_landmark_pos = np.array([
                landmark_dist * np.cos(landmark_angle),
                landmark_dist * np.sin(landmark_angle)
            ])

            estimated_landmark_pos += state[:2]

            state_estimated_landmarks.append(estimated_landmark_pos)

        estimated_landmarks.append(state_estimated_landmarks)

    car.visualize_trajectory(viz_file, include_gt=True,
                             estimated_landmarks=estimated_landmarks)

    return car.traj


def run_dead_reckoning(map_file, odometry_file, plan_file, gt_file, output_file):
    landmarks = load_map(map_file)

    with open(odometry_file, 'r', encoding='utf8') as f:
        lines = f.readlines()

        odometry = []

        for line in lines:
            odometry.append(
                np.array(list(map(float, line.strip().split(' ')))))

    with open(plan_file, 'r', encoding='utf8') as f:
        lines = f.readlines()

        start = tuple(map(float, lines[0].strip().split(' ')))

    with open(gt_file, 'r', encoding='utf8') as f:
        lines = f.readlines()

        gt = []

        for line in lines:
            gt.append(np.array(list(map(float, line.strip().split(' ')))))

        gt = np.array(gt)

    dead_reckon_traj = np.array(
        perform_dead_reckoning(
            start,
            odometry,
            landmarks,
            gt,
            viz_file=output_file.split('/')[-1]
        )
    )

    plt.plot(dead_reckon_traj[:, 0], dead_reckon_traj[:,
             1], label='Dead Reckoning', color='red')
    plt.plot(gt[:, 0], gt[:, 1], label='Ground Truth', color='blue')

    plt.legend()

    plt.savefig(f'{output_file}.png')

    plt.clf()


if __name__ == '__main__':
    # arg_parser = argparse.ArgumentParser()
    # arg_parser.add_argument('--map', type=str, required=True)
    # arg_parser.add_argument('--odometry', type=str, required=True)
    # arg_parser.add_argument('--landmarks', type=str, required=True)
    # arg_parser.add_argument('--plan', type=str, required=True)
    # arg_parser.add_argument('--gt', type=str, required=True)
    # parsed_args1 = arg_parser.parse_args()

    # main()

    for prob_num in range(10):
        noise = 'H' if prob_num % 2 == 0 else 'L'
        file_suffix = f'_{prob_num}_{noise}.txt'

        map_file = 'maps/map' + str(prob_num//2) + '.txt'
        gt_file = 'data/gt' + file_suffix
        odometry_file = 'data/odometry' + file_suffix
        plan_file = 'data/plan' + file_suffix
        landmarks_file = 'data/landmarks' + file_suffix

        run_dead_reckoning(map_file, odometry_file, plan_file,
                           gt_file, f'dead_reckon_output/{prob_num}_{noise}')
