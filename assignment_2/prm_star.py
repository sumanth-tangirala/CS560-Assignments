import argparse
import time
import tqdm
import numpy as np

from constants import NUM_PRM_NEIGHBORS, NUM_PRM_NODES
from utils import parallel_run, read_configs_file
from prm import PRM


class PRM_Star(PRM):
    visualization_directory = 'prm_star'

    def add_nodes(self, num_nodes=NUM_PRM_NODES):
        for _ in tqdm.tqdm(range(num_nodes - 2)):
            node = self.add_node(recursively=True)
            num_neighbors = NUM_PRM_NEIGHBORS + np.ceil(np.log(len(self.graph))).astype(dtype=int)
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

# def main(args):
#     map_file, start, goal, robot, name, index = args
#     obstacle_map = read_configs_file(map_file)
#     start = np.array([float(x) for x in start])
#     goal = np.array([float(x) for x in goal])

#     start_time = time.time()
#     prm = PRM_Star(start, goal, obstacle_map, is_arm=robot == 'arm')

#     prm.run()

#     end_time = time.time()

#     # prm.visualize_graph(name_suffix=name)

#     # prm.visualize_path(name_suffix=name)

#     if prm.path:
#         return index, True, end_time - start_time, prm.path_cost
#     else:
#         return index, False, end_time - start_time, 0

# if __name__ == '__main__':
#     starts = [
#         [0, 0, 0],
#         [np.pi/2, 0, 0],
#         [np.pi/2, np.pi/2, np.pi/2],
#         [-np.pi/2, np.pi, np.pi/2],
#         [np.pi, -np.pi/6, -np.pi/6]
#     ]

#     goals = [
#         [5 * np.pi/6, np.pi/2, 0],
#         [-np.pi/2, 0, np.pi/3],
#         [0, 0, np.pi/2],
#         [0, 0, np.pi],
#         [np.pi, np.pi/2, 0]
#     ]

#     args = []

#     for i in range(50):
#         index = i % 5
#         arg = [
#             f"configs/prm_arm_polygons{index+1}.txt",
#             starts[index],
#             goals[index],
#             'arm',
#             f"map_arm_{i}",
#             i
#         ]

#         args.append(arg)

#     results = parallel_run(main, args, show_progress=True)

#     results_dict = {
#         0: {"success": 0, "time": 0, "path_cost": 0},
#         1: {"success": 0, "time": 0, "path_cost": 0},
#         2: {"success": 0, "time": 0, "path_cost": 0},
#         3: {"success": 0, "time": 0, "path_cost": 0},
#         4: {"success": 0, "time": 0, "path_cost": 0},
#     }

#     for result in results:
#         i, success, time_taken, path_cost = result

#         index = i % 5

#         if success:
#             results_dict[index]["success"] += 1
#         results_dict[index]["time"] += time_taken
#         results_dict[index]["path_cost"] += path_cost

#     for i in range(5):
#         print('\n')
#         print(f"Results for arm {i}")
#         print(f"Success Rate: {results_dict[i]['success'] / 10}")
#         print(f"Average Time: {results_dict[i]['time'] / 10}")
#         print(f"Average Path Cost: {results_dict[i]['path_cost'] / 10}")


if __name__ == '__main__':
    main()
