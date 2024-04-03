import heapq

from numpy.linalg import norm
from itertools import count


from create_scene import scene_from_file
from utils import get_next_robot_positions, is_robot_colliding, get_robot_pq_node
from assignment_2.constants import green, ROBOT_SIZE, blue
from visualizer.visualization import sphere as sphere_geom, box as box_geom
from visualizer.visualization.threejs_group import *

TIMESTEP = 0.05  # seconds
COLLISION_CHECK_TIME = .5 # second
INTERPOLATION_TIME = .5  # second

tiebreaker = count()


def interpolate_rigid_body(start_position: np.ndarray, goal_position: np.ndarray, is_collision_check: bool = False) -> np.ndarray:
    total_time = INTERPOLATION_TIME if not is_collision_check else COLLISION_CHECK_TIME
    return np.linspace(start_position, goal_position, int(total_time / TIMESTEP))


def visualize_robot_path(path: list, scene: dict, scene_name: str) -> None:
    viz_out = threejs_group(js_dir="../js")

    robot_trajectory = []
    robot_animation_traj = []
    for i in range(1, len(path)):
        states = interpolate_rigid_body(path[i-1], path[i])
        robot_trajectory.extend(states)

    idx = 0
    for t in np.arange(0, TIMESTEP * len(robot_trajectory), TIMESTEP):
        robot_animation_traj.append([
            t,
            robot_trajectory[idx],
            [1, 0, 0, 0],
            blue,
        ])
        idx += 1

    obstacles = scene["obstacles"]

    for i, sphere in enumerate(obstacles):
        center = sphere[:3]
        radius = sphere[3]
        obstacle_geom = sphere_geom(f"sphere_{i}", radius, center)
        viz_out.add_obstacle(obstacle_geom, green)

    robot = box_geom("robot", ROBOT_SIZE, ROBOT_SIZE, ROBOT_SIZE, robot_trajectory[0])
    viz_out.add_animation(robot, robot_animation_traj)

    viz_out.to_html(f"./visualizer/out/{scene_name}_a_star.html")


def A_star(start_pos, goal_pos, scene):

    if is_robot_colliding(start_pos, scene):
        print('Start position is invalid')

    if is_robot_colliding(goal_pos, scene):
        print('Goal position is invalid')

    fringe = []
    visited = set(tuple(start_pos))
    visited_list = [start_pos]

    start_heuristic = norm(start_pos - goal_pos)
    heapq.heappush(fringe, (
        start_heuristic,
        next(tiebreaker),
        get_robot_pq_node(start_pos, [start_pos], 0, start_heuristic)
    ))

    while len(fringe):
        _, _, current = heapq.heappop(fringe)

        current_pos = current["position"]

        current_path = current["path"]
        current_cost = current["cost"]

        if np.all(current_pos == goal_pos):
            return current_path + [goal_pos]

        for next_pos in get_next_robot_positions(current_pos):
            if tuple(next_pos) not in visited and check_if_move_is_valid(current_pos, next_pos, scene):
                visited.add(tuple(next_pos))
                visited_list.append(next_pos)
                heuristic = norm(next_pos - goal_pos)
                total_cost = (current_cost + 1 + heuristic)
                priority_queue_node = get_robot_pq_node(next_pos, current_path + [next_pos], current_cost + 1, heuristic)
                heapq.heappush(fringe, (
                    total_cost,
                    next(tiebreaker),
                    priority_queue_node
                ))
    return None


def check_if_move_is_valid(start_pos, goal_pos, scene):
    interpolated_path = interpolate_rigid_body(start_pos, goal_pos, is_collision_check=True)
    for pos in interpolated_path:
        if is_robot_colliding(pos, scene):
            return False
    return True


def main():
    scene_name = 'scene5'
    scene = scene_from_file(scene_name)

    path = A_star(np.array([40, 40, 2.5]), np.array([-40, -40, 2.5]), scene)



    if path is None:
        print('No path found')
    else:
        visualize_robot_path(path, scene, scene_name)


if __name__ == '__main__':
    main()
