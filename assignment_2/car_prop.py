import argparse
from typing import List

import numpy as np

from utils.visualization_utils import visualize_car_trajectory
from constants import DT, L

from visualizer.visualization.threejs_group import *


TOTAL_TIME = 10

dirpath = os.path.dirname(os.path.abspath(__file__))


def forward_prop(u: np.ndarray, state=None, prop_steps=int(TOTAL_TIME / DT)) -> List[np.ndarray]:
    traj = []

    if state is None:
        state = np.array([0.0, 0.0, 0.0])

    traj.append(state.copy())

    for _ in range(prop_steps):
        state_dot = np.array([
            u[0] * np.cos(state[2]),
            u[0] * np.sin(state[2]),
            u[0] * np.tan(u[1]) / L,
        ])

        state += state_dot * DT

        state[2] = state[2] % (2 * np.pi)

        if state[2] > np.pi:
            state[2] -= 2 * np.pi

        elif state[2] < -np.pi:
            state[2] += 2 * np.pi

        traj.append(state.copy())

    return traj


def plot_trajectory(traj: List[np.ndarray], name):
    import matplotlib.pyplot as plt

    traj = np.array(traj)

    plt.xlim(-15, 15)
    plt.ylim(-15, 15)

    plt.plot(traj[:, 0], traj[:, 1])
    plt.savefig(f'{dirpath}/visualizer/out/car_prop/car_traj_{name}.png')


def visualize_trajectory(traj: List[np.ndarray], name):
    viz_out = threejs_group(js_dir="../../js")

    visualize_car_trajectory(traj, viz_out)

    viz_out.to_html(f"./visualizer/out/car_prop/{name}.html")


def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('-u', type=str, required=True, nargs='+')

    parsed_args = arg_parser.parse_args()

    if len(parsed_args.u) != 2:
        raise ValueError('Control input for car should have 2 values')

    u = np.array([float(x) for x in parsed_args.u])

    traj = forward_prop(u)

    name = '_'.join(parsed_args.u)

    plot_trajectory(traj, name)
    visualize_trajectory(traj, name)


if __name__ == '__main__':
    main()
