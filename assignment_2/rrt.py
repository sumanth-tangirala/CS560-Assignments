import argparse
import random


from matplotlib.collections import LineCollection
import numpy as np

from car_prop import forward_prop
from constants import DT
from utils import angular_diff, read_configs_file
from utils.collision_checking_utils import is_car_colliding
from utils.rrt_utils import RRTNode

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation

from utils.visualization_utils import visualize_car_trajectory, visualize_obstacles
from visualizer.visualization.threejs_group import *

MAXIMUM_PROP_COUNT = 1e3
PROP_STEPS = 5

GOAL_POS_DIST = 0.1
GOAL_ANGLE_DIST = 0.5
GOAL_BIAS_PROB = 0.5
MAX_VEL = 2

class RRT:
    visualization_directory = 'rrt'

    def __init__(self, start, goal, obstacle_map):
        self.start = start
        self.goal = goal
        self.obstacle_map = obstacle_map

        if is_car_colliding(start, self.obstacle_map):
            raise ValueError('Start state is in collision')
        
        if is_car_colliding(goal, self.obstacle_map):
            raise ValueError('Goal state is in collision')
       
        self.tree = [RRTNode(start)]

    def terminate(self, config):
        return np.linalg.norm(config[:2] - self.goal[:2]) < GOAL_POS_DIST and np.abs(angular_diff(config[2], self.goal[2])) < GOAL_ANGLE_DIST

    def propagate(self, random_node):
        u0 = np.random.rand() * MAX_VEL
        u1 = np.random.uniform(-1, 1)

        u = np.array([u0, u1])

        traj = forward_prop(u, random_node.config.copy(), PROP_STEPS)

        if is_car_colliding(traj[-1], self.obstacle_map):
            return None, None, None
        
        return traj[-1], u, traj

    def run(self):
        min_dist = np.inf
        goal_node = None
        while len(self.tree) < MAXIMUM_PROP_COUNT:
            if np.random.rand() < GOAL_BIAS_PROB:
                random_config = self.goal
            else:
                random_pos = (np.random.rand(2) * 100) - 50
                random_angle = np.random.rand() * 2 * np.pi - np.pi
                random_config = np.array([random_pos[0], random_pos[1], random_angle])

            min_random_dist = np.inf
            random_node = None
            for node in self.tree:
                dist = node.compute_distance(random_config, alpha=1)

                if min_random_dist > dist:
                    min_random_dist = dist

                    random_node = node

            new_config, control, trajectory = self.propagate(random_node)

            if new_config is None:
                continue

            new_node = RRTNode(new_config, parent=random_node)

            random_node.add_child(new_node, control, trajectory)

            self.tree.append(new_node)

            dist = new_node.compute_distance(self.goal, individual=False)
            pos_dist, ang_dist = new_node.compute_distance(self.goal, individual=True)

            if min_dist > dist:
                min_dist = dist
                print(min_dist, pos_dist, ang_dist, len(self.tree))


            if self.terminate(new_config):
                goal_node = new_node
                print('Reached Goal')
                break

        if goal_node is None:
            print('Could not reach goal')
            return None

        reverse_path = []

        node = goal_node

        while node is not None:
            reverse_path.append(node)
            node = node.parent

        self.path = list(reversed(reverse_path))

        return self.path

    def visualize_rrt(self):
        fig, ax = plt.subplots()
        ax.set_xlim(-50, 50)
        ax.set_ylim(-50, 50)
        ax.set_aspect('equal')

        # Draw static elements
        obstacles = [plt.Circle((obs[0], obs[1]), obs[3], color='g') for obs in self.obstacle_map]
        for obstacle in obstacles:
            ax.add_patch(obstacle)
        ax.add_patch(plt.Circle((self.goal[0], self.goal[1]), GOAL_POS_DIST, color='red', fill=False))
        ax.plot(self.start[0], self.start[1], 'bo')

        # Pre-compute edges
        edges = []
        for node in self.tree:
            if node.parent is not None:
                edges.append([(node.parent.config[0], node.parent.config[1]), (node.config[0], node.config[1])])

        line_segments = LineCollection(edges, colors='blue', linewidths=0.5)
        ax.add_collection(line_segments)

        def update(frame):
            # Only modify what needs to be changed
            line_segments.set_segments(edges[:frame])
            return line_segments,

        anim = FuncAnimation(fig, update, frames=len(self.tree) + 1, interval=30, blit=True)

        print('Generated Animation...')
        file_name = './visualizer/out/rrt/tree.'

        anim.save(file_name + 'mp4', writer='ffmpeg')
   
        # html = anim.to_jshtml()

        # with open(file_name + 'html', 'w') as f:
        #     f.write(html)


        # print('Saved RRT Tree')

    def visualize_path(self):
        trajectory = []

        for i in range(len(self.path) - 1):
            node = self.path[i]
            next_node = self.path[i + 1]

            traj = node.children[next_node][1]
            
            trajectory.extend(traj)

        viz_out = threejs_group(js_dir="../../js")
        
        visualize_obstacles(self.obstacle_map, viz_out)
        visualize_car_trajectory(trajectory, viz_out)
        
        viz_out.to_html("./visualizer/out/rrt/path.html")


def main():
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--map', type=str, required=True)
    arg_parser.add_argument('--start', type=str, required=True, nargs='+')
    arg_parser.add_argument('--goal', type=str, required=True, nargs='+')

    parsed_args = arg_parser.parse_args()

    if len(parsed_args.start) != 3:
        raise ValueError('Start state for arm should have 3 values')

    if len(parsed_args.goal) != 3:
        raise ValueError('Goal state for arm should have 3 values')

    obstacle_map = read_configs_file(parsed_args.map)
    start = np.array([float(x) for x in parsed_args.start])
    goal = np.array([float(x) for x in parsed_args.goal])

    rrt = RRT(start, goal, obstacle_map)

    path = rrt.run()

    rrt.visualize_rrt()

    if path is None: return

    rrt.visualize_path()


if __name__ == '__main__':
    main()