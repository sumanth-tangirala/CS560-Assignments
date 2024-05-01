import argparse
from scipy.stats import norm
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
from tqdm import tqdm
import time

from map_generator import load_map
from car_prop import forward_prop
from utils import generate_random_config
from constants import HIGH_OBS_NOISE_STDDEV, LOW_OBS_NOISE_STDDEV


class Particle:
    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def __str__(self):
        return f'Pose: {self.pose}, Weight: {self.weight}'

    def __repr__(self):
        return str(self)

    def copy(self, weight=None):
        return Particle(self.pose.copy(), weight or self.weight)


class ParticleFilter:
    improbable_state_detected = False
    execution_time = None

    def __init__(self, num_particles, landmarks, odometry, observations, noise, gt, start_pose=None):
        self.particles = []
        self.num_particles = num_particles
        self.landmarks = landmarks
        self.odometry = odometry
        self.observations = observations
        self.gt = gt
        self.start_pose = start_pose

        if noise == 'H':
            landmark_std = HIGH_OBS_NOISE_STDDEV
            self.odometry_std = HIGH_OBS_NOISE_STDDEV
        elif noise == 'L':
            landmark_std = LOW_OBS_NOISE_STDDEV
            self.odometry_std = LOW_OBS_NOISE_STDDEV
        else:
            landmark_std = 0
            self.odometry_std = [0, 0]

        self.landmark_gaussian = norm([0, 0], landmark_std)

        self.initialize_particles()

        self.past_particles = [[p.copy() for p in self.particles]]

    def initialize_particles(self):
        for _ in range(self.num_particles):
            if self.start_pose is None:
                init_pose = generate_random_config()
            else:
                init_pose = self.start_pose
            self.particles.append(Particle(
                pose=init_pose,
                weight=1.0,
            ))

    def update(self, odometry, landmark_observations):
        self.predict(odometry)
        self.update_weights(landmark_observations)
        if self.improbable_state_detected:
            return
        self.past_particles.append([p.copy() for p in self.particles])
        self.resample()

    def predict(self, odometry):
        for particle in self.particles:
            sampled_odometry = np.random.normal(odometry, self.odometry_std)

            particle.pose = forward_prop(sampled_odometry, particle.pose)[-1]

    def update_weights(self, landmark_observations):
        for particle in self.particles:
            for landmark_idx, sensor_reading in enumerate(landmark_observations):
                dx = self.landmarks[landmark_idx][0] - particle.pose[0]
                dy = self.landmarks[landmark_idx][1] - particle.pose[1]

                expected_distance = np.linalg.norm([dx, dy])
                expected_phi = np.arctan2(dy, dx) - particle.pose[2]

                prob = self.landmark_gaussian.pdf(  # type: ignore
                    [sensor_reading[0] - expected_distance, sensor_reading[1] - expected_phi])

                particle.weight *= prob[0] * prob[1]

        total_weight_sum = sum([p.weight for p in self.particles])

        if total_weight_sum == 0:
            self.improbable_state_detected = True
            return

        for particle in self.particles:
            particle.weight /= total_weight_sum

    def resample(self):
        weights = [particle.weight for particle in self.particles]
        resampled_indices = np.random.choice(
            len(self.particles), size=self.num_particles, p=weights)
        self.particles = [self.particles[i].copy(1.0)
                          for i in resampled_indices]  # type: ignore

    def run(self):
        start_time = time.time()
        for odometry, observation in zip(self.odometry, self.observations):
            self.update(odometry, observation)
            if self.improbable_state_detected:
                break

        if self.improbable_state_detected:
            self.initialize_particles()
            self.past_particles = [[p.copy() for p in self.particles]]
            self.improbable_state_detected = False
            self.run()

        self.execution_time = time.time() - start_time

    def get_estimate(self, idx=None):
        if idx is None:
            return np.average([p.pose for p in self.particles], axis=0)

        return np.average([p.pose for p in self.past_particles[idx]], axis=0)

    def animate_particles(self, path):
        print('Generating Animation...')

        fig, ax = plt.subplots()
        ax.set_xlim(-50, 50)
        ax.set_ylim(-50, 50)
        ax.set_aspect('equal')

        ax.plot(self.gt[:, 0], self.gt[:, 1])

        for landmark in self.landmarks:
            ax.plot(landmark[0], landmark[1], 'ro')

        # Create a scatter plot object which will be updated in each frame
        # Use a plot object that can be updated
        particle_scatter_plot, = ax.plot([], [], 'b.', alpha=1)
        gt_scatter_plot, = ax.plot([], [], 'g.', alpha=1)

        # Function to update the scatter plot for each frame

        def update(frame):
            particles, gt = frame

            gt_x = gt[0]
            gt_y = gt[1]

            gt_scatter_plot.set_data(gt_x, gt_y)

            x = [p.pose[0] for p in particles]
            y = [p.pose[1] for p in particles]
            particle_scatter_plot.set_data(x, y)
            return particle_scatter_plot, gt_scatter_plot

        # FuncAnimation to generate each frame of the animation
        animation = FuncAnimation(
            fig, update, frames=zip(self.past_particles, self.gt), blit=True, interval=200, repeat=False)

        # Save the animation
        animation.save(path, writer='ffmpeg')
        print('Animation saved as particle_animation.mp4')

    def error_plots(self, path):
        plt.clf()
        errors = []

        for i in range(len(self.gt)):
            errors.append(np.linalg.norm(self.get_estimate(i) - self.gt[i]))

        plt.plot(errors)
        plt.savefig(path)


def run_particle_filter(map_file, odometry_file, plan_file, landmarks_file, gt_file, num_particles, output_name=None):
    landmarks = load_map(map_file)

    noise = 'H' if 'H' in odometry_file else 'L'

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

    pf = ParticleFilter(
        num_particles=num_particles,
        landmarks=landmarks,
        odometry=odometry,
        observations=observations,
        noise=noise,
        start_pose=start,
        gt=gt
    )

    pf.run()

    pf.animate_particles(
        output_name + '.mp4' if output_name else 'particle_animation.mp4')
    pf.error_plots(output_name + '.png' if output_name else 'error_plots.png')

    return pf.past_particles, pf.execution_time, pf.get_estimate(), gt


def main():
    for num_particles in [200, 2000]:
        for prob_num in range(10):
            print('Running Particle Filter: ', prob_num, num_particles)
            noise = 'H' if prob_num % 2 == 0 else 'L'
            file_suffix = f'_{prob_num}_{noise}.txt'

            map_file = 'maps/map' + str(prob_num//2) + '.txt'
            gt_file = 'data/gt' + file_suffix
            odometry_file = 'data/odometry' + file_suffix
            plan_file = 'data/plan' + file_suffix
            landmarks_file = 'data/landmarks' + file_suffix

            past_particles, execution_time, estimate, gt = run_particle_filter(
                map_file, odometry_file, plan_file, landmarks_file, gt_file, num_particles, f'output/{num_particles}_{prob_num}_{noise}')

            with open('output/execution_times.txt', 'a') as f:
                f.write(
                    f'{num_particles}, {prob_num}, {noise}, {execution_time}\n')


if __name__ == '__main__':
    main()
    # arg_parser = argparse.ArgumentParser()
    # arg_parser.add_argument('--map', type=str, required=True)
    # arg_parser.add_argument('--odometry', type=str, required=True)
    # arg_parser.add_argument('--landmarks', type=str, required=True)
    # arg_parser.add_argument('--plan', type=str, required=True)
    # arg_parser.add_argument('--num_particles', type=int, required=True)
    # arg_parser.add_argument('--gt', type=str, required=True)

    # parsed_args = arg_parser.parse_args()
    # run_particle_filter(parsed_args.map, parsed_args.odometry, parsed_args.plan,
    #      parsed_args.landmarks, parsed_args.gt, parsed_args.num_particles)
