import argparse
from queue import PriorityQueue

from utils.graphNode import sanitize_angles
from utils import compute_arm_distance, compute_vehicle_distance, read_configs_file
from utils.visualization_utils import visualize_arm_config, visualize_vehicle

from visualizer.visualization.threejs_group import *


def compute_nearest_neighbors(target, configs, k, is_arm):
    pq = PriorityQueue(k)
    for i, config in enumerate(configs):
        distance = compute_arm_distance(target, config) if is_arm else compute_vehicle_distance(target, config)

        if pq.qsize() < k:
            pq.put_nowait((-distance, i, config))

        else:
            max_dist_element = pq.get_nowait()
            if distance < -max_dist_element[0]:
                pq.put_nowait((-distance, i, config))
            else:
                pq.put_nowait(max_dist_element)

    knn = [
        idx for _, idx, _ in pq.queue
    ]

    return knn


def visualize_nearest_neighbors(target, nearest_neighbors, is_arm):
    viz_out = threejs_group(js_dir="../../js")
    for i, config in enumerate([target, *nearest_neighbors]):
        name = f"{'arm' if is_arm else 'vehicle'}_" + ("target" if i == 0 else f"neighbor_{i}")
        if is_arm:
            visualize_arm_config(config, viz_out)
        else:
            visualize_vehicle(config, viz_out)

        viz_out.to_html(f"./visualizer/out/knn/{name}.html")


def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--robot', type=str, required=True, choices=['arm', 'vehicle'])
    arg_parser.add_argument('-k', type=int, required=True)
    arg_parser.add_argument('--configs', type=str, required=True)
    arg_parser.add_argument('--target', type=str, required=True, nargs='+')

    parsed_args = arg_parser.parse_args()

    if parsed_args.robot == 'arm' and len(parsed_args.target) != 3:
        raise ValueError('Arm robot requires a target description of 3 joint angles')

    if parsed_args.robot == 'vehicle' and len(parsed_args.target) != 7:
        raise ValueError('Vehicle robot requires a target description of 7 configurations')

    if parsed_args.k <= 0:
        raise ValueError('k must be a positive integer')

    loaded_configs = read_configs_file(parsed_args.configs)

    parsed_args.target = np.array([eval(val) for val in parsed_args.target], dtype=np.float64)

    if parsed_args.robot == 'arm':
        parsed_args.target = sanitize_angles(parsed_args.target)
        for i in range(len(loaded_configs)):
            loaded_configs[i] = sanitize_angles(loaded_configs[i])

    nearest_neighbors_indices = compute_nearest_neighbors(parsed_args.target, loaded_configs, parsed_args.k,
                                                  parsed_args.robot == 'arm')

    nearest_neighbors = [loaded_configs[idx] for idx in nearest_neighbors_indices]

    visualize_nearest_neighbors(parsed_args.target, nearest_neighbors, parsed_args.robot == 'arm')


if __name__ == "__main__":
    main()
