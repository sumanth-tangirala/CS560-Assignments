import argparse
import tqdm
import numpy as np

from constants import NUM_PRM_NEIGHBORS, NUM_PRM_NODES
from utils import read_configs_file
from prm import PRM


class PRM_Star(PRM):
    visualization_directory = 'prm_star'

    def add_nodes(self, num_nodes=NUM_PRM_NODES):
        for _ in tqdm.tqdm(range(num_nodes - 2)):
            node = self.add_node(recursively=True)
            num_neighbors = NUM_PRM_NEIGHBORS + np.ceil(np.log(10)).astype(dtype=int)
            self.add_neighbors(node, num_neighbors)

def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--robot', type=str, required=True, choices=['arm', 'vehicle'])
    arg_parser.add_argument('--map', type=str, required=True)
    arg_parser.add_argument('--start', type=str, required=True, nargs='+')
    arg_parser.add_argument('--goal', type=str, required=True, nargs='+')

    parsed_args = arg_parser.parse_args()

    if parsed_args.robot == 'arm':
        if len(parsed_args.start) != 3:
            raise ValueError('Start state for arm should have 3 values')

        if len(parsed_args.goal) != 3:
            raise ValueError('Goal state for arm should have 3 values')

    if parsed_args.robot == 'vehicle':
        if len(parsed_args.start) != 7:
            raise ValueError('Start state for vehicle should have 7 values')
    
        if len(parsed_args.goal) != 7:
            raise ValueError('Goal state for vehicle should have 7 values')

    obstacle_map = read_configs_file(parsed_args.map)
    start = np.array([float(x) for x in parsed_args.start])
    goal = np.array([float(x) for x in parsed_args.goal])

    prm_star = PRM_Star(start, goal, obstacle_map, is_arm=parsed_args.robot == 'arm')

    prm_star.run()

    prm_star.visualize_graph()

    prm_star.visualize_path()


if __name__ == '__main__':
    main()