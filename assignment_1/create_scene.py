import json

from assignment_2.constants import upper_space_limits, green, lower_space_limits
from visualizer.visualization import sphere as sphere_geom
from visualizer.visualization.threejs_group import *


def generate_scene(num_spheres, r_min, r_max):
    sphere_center_upper_limits = upper_space_limits - r_max
    sphere_center_lower_limits = lower_space_limits + r_max

    sphere_centers = np.random.uniform(sphere_center_lower_limits, sphere_center_upper_limits, (num_spheres, 3))

    sphere_radii = np.random.uniform(r_min, r_max, num_spheres)

    return {
        "obstacles": np.concatenate([sphere_centers, sphere_radii[:, None]], axis=1)
    }


def scene_to_file(scene, filename):
    scene["obstacles"] = scene["obstacles"].tolist()
    if "robot" in scene:
        scene["robot"] = scene["robot"].tolist()
    with open(f"scenes/{filename}.json", "w") as f:
        f.write(json.dumps(scene, indent=2))


def scene_from_file(filename):
    with open(f"scenes/{filename}.json", "r") as f:
        scene = json.loads(f.read())

    scene["obstacles"] = np.array(scene["obstacles"])

    if "robot" in scene:
        scene["robot"] = np.array(scene["robot"])

    return scene


def visualize_scene(scene, filename="scene"):
    viz_out = threejs_group(js_dir="../js")

    for i, sphere in enumerate(scene["obstacles"]):
        center = sphere[:3]
        radius = sphere[3]
        geom = sphere_geom(f"sphere_{i}", radius, center, [1, 0, 0, 0])
        viz_out.add_obstacle(geom, green)

    viz_out.to_html(f"./visualizer/out/{filename}.html")


def main():
    # scene = generate_scene(0, 0.5, 1)
    filename = "scene5"
    # scene_to_file(scene, filename)
    scene = scene_from_file(filename)
    visualize_scene(scene, filename)


if __name__ == '__main__':
    main()
