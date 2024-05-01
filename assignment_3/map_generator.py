import numpy as np

from constants import upper_space_limits, lower_space_limits


def generate_maps(landmark_counts):
    for i, landmark_count in enumerate(landmark_counts):
        with open(f'maps/map{i}.txt', 'w', encoding='utf-8') as f:
            for _ in range(landmark_count):
                x = np.random.uniform(
                    lower_space_limits[0], upper_space_limits[0])
                y = np.random.uniform(
                    lower_space_limits[1], upper_space_limits[1])

                f.write(f'{x} {y}\n')


def load_map(map_path):
    landmarks = []

    with open(map_path, 'r', encoding='utf-8') as f:
        for line in f:
            x, y = map(float, line.strip().split())
            landmarks.append([x, y])

    return np.array(landmarks)


if __name__ == '__main__':
    generate_maps([5, 5, 8, 12, 12])
