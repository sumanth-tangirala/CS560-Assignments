import random

from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

from car_prop import forward_prop
from constants import MAXIMUM_PROP_COUNT, MAX_PROP_STEPS, MIN_PROP_STEPS, \
    GOAL_POS_DIST, GOAL_ANGLE_DIST, GOAL_BIAS_PROB, MAX_VEL, MIN_VEL, \
    MIN_STEERING, MAX_STEERING

from utils import angular_diff, generate_random_config
from utils.rrt_utils import RRTNode
from utils.visualization_utils import visualize_car_trajectory

from visualizer.visualization.threejs_group import threejs_group


class RRT:
    visualization_directory = 'rrt'
    trajectory_found = False
    trajectory = []
    controls = []
    prop_steps = []

    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

        self.tree = [RRTNode(start)]

    def _should_terminate(self, config):
        return (
            np.linalg.norm(config[:2] - self.goal[:2]) < GOAL_POS_DIST and
            np.abs(angular_diff(config[2], self.goal[2])) < GOAL_ANGLE_DIST
        )

    def _propagate(self, random_node):
        u0 = np.random.uniform(MIN_VEL, MAX_VEL)
        u1 = np.random.uniform(MIN_STEERING, MAX_STEERING)

        u = np.array([u0, u1])

        # prop_steps = random.randint(MIN_PROP_STEPS, MAX_PROP_STEPS)

        prop_steps = 5

        traj = forward_prop(u, random_node.config.copy())

        return traj[-1], u, prop_steps

    def run(self):
        min_dist = np.inf
        goal_node = None

        out_of_bounds_count = 0

        while len(self.tree) < MAXIMUM_PROP_COUNT:
            if np.random.uniform(0, 1) < GOAL_BIAS_PROB:
                random_config = self.goal
            else:
                random_config = generate_random_config()

            min_random_dist = np.inf

            for node in self.tree:
                dist = node.compute_distance(random_config)

                if min_random_dist > dist:  # type: ignore
                    min_random_dist = dist

                    random_node = node

            new_config, control, prop_steps = self._propagate(random_node)

            new_x, new_y = new_config[:2]
            # breakpoint()
            if np.abs(new_x) > 49 or np.abs(new_y) > 49:
                out_of_bounds_count += 1
                if out_of_bounds_count > 100:
                    print('Out of bounds')
                    return None
                continue

            if new_config is None:
                continue

            new_node = RRTNode(new_config, parent=random_node)

            random_node.add_child(new_node, control, prop_steps)

            self.tree.append(new_node)

            dist = new_node.compute_distance(self.goal, individual=False)

            # pos_dist, ang_dist = new_node.compute_distance(
            #     self.goal, individual=True)
            # if min_dist > dist:
            #     print(min_dist, pos_dist, ang_dist, len(self.tree))

            min_dist = min(min_dist, dist)

            if self._should_terminate(new_config):
                goal_node = new_node
                break

        if goal_node is None:
            print('Could not reach goal')
            return None

        self.trajectory_found = True

        reverse_traj = []
        reverse_controls = []
        reverse_prop_steps = []

        node = goal_node

        while node is not None:
            if node.parent:
                control, prop_steps = node.parent.children[node]
                reverse_controls.append(control)
                reverse_prop_steps.append(prop_steps)

            reverse_traj.append(node.config.copy())

            node = node.parent

        assert len(reverse_traj) == len(reverse_controls) + 1
        assert len(reverse_traj) == len(reverse_prop_steps) + 1

        self.trajectory = list(reversed(reverse_traj))
        self.controls = list(reversed(reverse_controls))
        self.prop_steps = list(reversed(reverse_prop_steps))

        return self.trajectory

    def visualize_rrt(self):
        print('Generating Animation...')
        fig, ax = plt.subplots()
        ax.set_xlim(-50, 50)
        ax.set_ylim(-50, 50)
        ax.set_aspect('equal')

        # Draw static elements
        ax.add_patch(plt.Circle(  # type: ignore
            (self.goal[0], self.goal[1]), GOAL_POS_DIST, color='red', fill=False))
        ax.plot(self.start[0], self.start[1], 'bo')

        # Pre-compute edges
        edges = []
        for node in self.tree:
            if node.parent is not None:
                edges.append([
                    (node.parent.config[0], node.parent.config[1]),
                    (node.config[0], node.config[1]),
                ])

        line_segments = LineCollection(edges, colors='blue', linewidths=0.5)
        ax.add_collection(line_segments)  # type: ignore

        def update(frame):
            # Only modify what needs to be changed
            line_segments.set_segments(edges[:frame])
            return line_segments,

        anim = FuncAnimation(fig, update, frames=len(
            self.tree) + 1, interval=30, blit=True)

        print('Saving Animation...')
        file_name = './visualizer/out/rrt/tree.'

        anim.save(file_name + 'mp4', writer='ffmpeg')

    def visualize_path(self):
        trajectory = []

        for i in range(len(self.trajectory) - 1):
            node = self.trajectory[i]
            next_node = self.trajectory[i + 1]

            traj = node.children[next_node][1]

            trajectory.extend(traj)

        viz_out = threejs_group(js_dir="../../js")

        visualize_car_trajectory(trajectory, viz_out)

        viz_out.to_html("./visualizer/out/rrt/path.html")
