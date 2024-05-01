import numpy as np
import matplotlib.pyplot as plt

from map_generator import load_map

from car_prop import Car


def plot_data(map_file, odometry_file, plan_file, gt_file, output_file):
    landmarks = load_map(map_file)

    with open(odometry_file, 'r', encoding='utf8') as f:
        lines = f.readlines()

        odometry = []

        for line in lines:
            odometry.append(
                np.array(list(map(float, line.strip().split(' ')))))

    with open(plan_file, 'r', encoding='utf8') as f:
        lines = f.readlines()

        plan = []

        start = list(map(float, lines[0].strip().split(' ')))

        for line in lines[1:]:
            plan.append(np.array(list(map(float, line.strip().split(' ')))))

    with open(landmarks_file, 'r', encoding='utf8') as f:
        lines = f.readlines()

        observations = []

        for line in lines:
            idx = 0
            values = line.strip().split(' ')
            observation = []
            while idx < len(values):
                observation.append(
                    np.array(list(map(float, values[idx:idx + 2])))
                )
                idx += 2

            observations.append(observation)

    if gt_file is not None:
        with open(gt_file, 'r', encoding='utf8') as f:
            lines = f.readlines()

            gt = []

            for line in lines:
                gt.append(np.array(list(map(float, line.strip().split(' ')))))
        gt = np.array(gt)

    car = Car(
        start=start,
    )

    for control in plan:
        car.step(control)

    planned_traj = np.array(car.traj)

    plt.plot(planned_traj[:, 0], planned_traj[:,
             1], label='Planned Trajectory', color='red')
    plt.plot(gt[:, 0], gt[:, 1], label='Executred Trajectory', color='blue')

    plt.legend()

    plt.savefig(f'{output_file}.png')

    plt.clf()


for prob_num in range(10):
    noise = 'H' if prob_num % 2 == 0 else 'L'
    file_suffix = f'_{prob_num}_{noise}.txt'

    map_file = 'maps/map' + str(prob_num//2) + '.txt'
    gt_file = 'data/gt' + file_suffix
    odometry_file = 'data/odometry' + file_suffix
    plan_file = 'data/plan' + file_suffix
    landmarks_file = 'data/landmarks' + file_suffix

    plot_data(map_file, odometry_file, plan_file,
              gt_file, f'plan_data/{prob_num}_{noise}')
