import argparse

import numpy as np

from map_generator import load_map
from particle_filter import ParticleFilter


def run_kidnapped_pf(map_file, odometry_file, plan_file, landmarks_file, gt_file, num_particles, output_name=None):
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

            past_particles, execution_time, estimate, gt = run_kidnapped_pf(
                map_file, odometry_file, plan_file, landmarks_file, gt_file, num_particles, f'output_kidnapped/{num_particles}_{prob_num}_{noise}')

            with open('output_kidnapped/execution_times.txt', 'a') as f:
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
    # run_kidnapped_pf(parsed_args.map, parsed_args.odometry, parsed_args.plan,
    #      parsed_args.landmarks, parsed_args.gt, parsed_args.num_particles)
