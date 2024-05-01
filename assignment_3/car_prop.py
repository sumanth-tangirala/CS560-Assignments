import argparse
import os

import matplotlib.pyplot as plt
import numpy as np

from map_generator import load_map
from utils.visualization_utils import visualize_car_trajectory
from constants import DT, HIGH_ACT_NOISE_STDDEV, HIGH_OBS_NOISE_STDDEV, \
    HIGH_ODO_NOISE_STDDEV, L, LOW_ACT_NOISE_STDDEV, LOW_OBS_NOISE_STDDEV, \
    LOW_ODO_NOISE_STDDEV

from visualizer.visualization.threejs_group import threejs_group

dirpath = os.path.dirname(os.path.abspath(__file__))


TOTAL_TIME = 10
PROP_STEPS = 5


def forward_prop(u, state):
    traj = [state.copy()]

    for _ in range(5):
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


class Car:
    def __init__(
        self,
        start=None,
        noise=None,
        landmarks=(),
        gt=None
    ):
        self.planned_controls = []
        self.act_controls = []
        self.odo_controls = []
        self.observations = []
        self.traj = []
        self.landmarks = landmarks
        self.gt = gt

        # Set Initial State
        if start is not None:
            self.state = np.array(start)
        else:
            self.state = np.array([0.0, 0.0, 0.0])

        self.traj.append(self.state.copy())

        # Set Noise Parameters
        if noise == 'L':
            self.act_noise_scale = LOW_ACT_NOISE_STDDEV
            self.odo_noise_scale = LOW_ODO_NOISE_STDDEV
            self.obs_noise_scale = LOW_OBS_NOISE_STDDEV
        elif noise == 'H':
            self.act_noise_scale = HIGH_ACT_NOISE_STDDEV
            self.odo_noise_scale = HIGH_ODO_NOISE_STDDEV
            self.obs_noise_scale = HIGH_OBS_NOISE_STDDEV
        else:
            self.act_noise_scale = [0, 0]
            self.odo_noise_scale = [0, 0]
            self.obs_noise_scale = [0, 0]

    def _compute_act_controls(self, u):
        act_controls = u.copy()

        noise_scale = self.act_noise_scale

        if u[0] == 0:
            noise_scale[0] = 0
        if u[1] == 0:
            noise_scale[1] = 0

        act_controls += np.random.normal(0, noise_scale, 2)

        self.act_controls.append(act_controls)

        return act_controls

    def _compute_odo_controls(self, u):
        odo_controls = u.copy()

        noise_scale = self.odo_noise_scale

        if u[0] == 0:
            noise_scale[0] = 0
        if u[1] == 0:
            noise_scale[1] = 0

        odo_controls += np.random.normal(0, noise_scale, 2)

        self.odo_controls.append(odo_controls)

        return odo_controls

    def _compute_observations(self):
        observations = []

        for landmark in self.landmarks:
            dx = landmark[0] - self.state[0]
            dy = landmark[1] - self.state[1]

            r = np.linalg.norm([dx, dy])
            phi = np.arctan2(dy, dx) - self.state[2]

            observation = np.array([r, phi]) + \
                np.random.normal(0, self.obs_noise_scale, 2)

            observations.append(observation)

        self.observations.append(np.array(observations))

    def step(self, u: np.ndarray):
        self.planned_controls.append(u.copy())

        act_u = self._compute_act_controls(u)
        self._compute_odo_controls(act_u)

        traj = forward_prop(act_u, self.state)
        self.state = traj[-1]

        self.traj.append(self.state.copy())
        self._compute_observations()

    def plot_trajectory(self, path):

        traj = np.array(self.traj)
        plt.clf()
        plt.plot(traj[:, 0], traj[:, 1])
        plt.savefig(f'./{path}.png')

    def visualize_trajectory(self, path, include_gt=False, estimated_landmarks=None):
        viz_out = threejs_group(js_dir="../js")

        visualize_car_trajectory(
            np.array(self.traj),
            secondary_path=self.gt if include_gt else None,  # type: ignore
            estimated_landmarks=estimated_landmarks,
            viz_out=viz_out
        )

        viz_out.to_html(f"./visualizer/out/dead_reckon_{path}.html")


def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('-u', type=str, required=True, nargs='+')

    parsed_args = arg_parser.parse_args()

    if len(parsed_args.u) != 2:
        raise ValueError('Control input for car should have 2 values')

    u = np.array([float(x) for x in parsed_args.u])

    landmarks = load_map('map0')

    landmarks = [
        [1, 1],
        *landmarks,
    ]

    car = Car(
        noise='H',
        landmarks=landmarks
    )

    for _ in range(int(TOTAL_TIME / DT)):
        car.step(u, PROP_STEPS)

    name = '_'.join(parsed_args.u)

    car.plot_trajectory(name)
    car.visualize_trajectory(name)


if __name__ == '__main__':
    main()
