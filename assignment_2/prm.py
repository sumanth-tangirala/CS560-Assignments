import argparse

import tqdm

from constants import NUM_PRM_NODES, NUM_PRM_NEIGHBORS
from nearest_neighbors import compute_nearest_neighbors
from utils import read_configs_file, GraphNode, a_star
from utils.collision_checking_utils import is_node_in_collision, is_edge_in_collision
from utils.visualization_utils import visualize_graph, visualize_obstacles, visualize_path

from visualizer.visualization.threejs_group import *


class PRM:
    visualization_directory = 'prm'

    def __init__(self, start, goal, obstacle_map, is_arm) -> None:
        self.start = start
        self.goal = goal
        self.obstacle_map = obstacle_map
        self.is_arm = is_arm
        self.graph = []

        self.start_node = self.add_node(config=start)
        self.goal_node = self.add_node(config=goal)

        if self.start_node is None:
            raise ValueError('Start node is in collision')
        
        if self.goal_node is None:
            raise ValueError('Goal node is in collision')
    
    def __len__(self) -> int:
        return len(self.graph)
    
    def _add_graph_edge(self, node1: GraphNode, node2: GraphNode):
        node1.add_adjacent(node2)
        node2.add_adjacent(node1)
        
    def add_node(self, config=None, recursively=False) -> GraphNode:
        if config is None:
            node = GraphNode(random=True, is_arm=self.is_arm)
        else:
            node = GraphNode(config, is_arm=self.is_arm)

        if is_node_in_collision(node, self.obstacle_map, self.is_arm):
            if not recursively:
                return None # type: ignore

            while is_node_in_collision(node, self.obstacle_map, self.is_arm):
                node = GraphNode(random=True, is_arm=self.is_arm)

        self.graph.append(node)
        return node

    def add_nodes(self, num_nodes = NUM_PRM_NODES):
        for _ in tqdm.tqdm(range(num_nodes - 2)):
            node = self.add_node(recursively=True)
            self.add_neighbors(node)

    def add_neighbors(self, node: GraphNode, k=NUM_PRM_NEIGHBORS):
        all_neighbor_configs = [node.config for node in self.graph]

        nearest_neighbor_indices = compute_nearest_neighbors(
            target=node.config,
            configs=all_neighbor_configs,
            k=k + 1,
            is_arm=self.is_arm,
        )

        node.neighbors = [
            self.graph[neighbor_index] for neighbor_index in nearest_neighbor_indices if self.graph[neighbor_index] != node
        ]

    def add_edges(self):
        for node1 in tqdm.tqdm(self.graph):
            for node2 in node1.neighbors:
                if not is_edge_in_collision(node1, node2, self.obstacle_map, self.is_arm):
                    self._add_graph_edge(node1, node2)

    def find_path(self):
        self.path = a_star(self.start_node, self.goal_node)

    def run(self):
        print('Adding nodes...')
        self.add_nodes()

        print('Adding edges between nodes...')
        self.add_edges()

        print('Finding path from start to goal...')
        self.find_path()

    def visualize_graph(self):
        robot = "arm" if self.is_arm else "vehicle"

        viz_out = threejs_group(js_dir="../../js")

        visualize_graph(self.graph, self.is_arm, viz_out)
        visualize_obstacles(self.obstacle_map, viz_out)

        viz_out.to_html(f"./visualizer/out/{self.visualization_directory}/graph_{robot}.html")


    def visualize_path(self):
        if not self.path:
            print('No Path Found')
            return

        robot = "arm" if self.is_arm else "vehicle"

        viz_out = threejs_group(js_dir="../../js")

        visualize_path(self.path, self.is_arm, viz_out)
        visualize_obstacles(self.obstacle_map, viz_out)

        viz_out.to_html(f"./visualizer/out/{self.visualization_directory}/path_{robot}.html")


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

    prm = PRM(start, goal, obstacle_map, is_arm=parsed_args.robot == 'arm')

    prm.run()

    prm.visualize_graph()

    prm.visualize_path()

if __name__ == '__main__':
    main()
