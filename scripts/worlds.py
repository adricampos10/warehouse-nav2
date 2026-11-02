#!/usr/bin/env python3
"""The worlds a scenario can run in, and the launch arguments for each one.

tb3_simulation_launch.py takes the Gazebo world, the Nav2 map and the spawn
position as arguments, so switching worlds needs no change to Nav2 itself.
No ROS imports, so this is unit-tested with pytest.

Usage (inside the container), prints the arguments for ros2 launch:
    python3 worlds.py /ws/scenarios/warehouse_narrow_aisle.yaml
    python3 worlds.py warehouse
"""
import sys
from pathlib import Path

DEFAULT_WORLD = "tb3_sandbox"

# world name -> (world file, map file, default start), files relative to the repo root.
# None means Nav2's own TurtleBot3 sandbox world and map.
WORLDS = {
    "tb3_sandbox": (None, None, (-2.0, -0.5)),
    "warehouse": ("worlds/warehouse.sdf.xacro", "maps/warehouse.yaml", (-5.5, 0.0)),
}


def default_start(world):
    return WORLDS[world][2]


def launch_args(world, start, root):
    """Arguments for tb3_simulation_launch.py: spawn the robot at start in world."""
    world_file, map_file, _ = WORLDS[world]
    args = [f"x_pose:={start[0]}", f"y_pose:={start[1]}"]
    if world_file:
        args += [f"world:={root}/{world_file}", f"map:={root}/{map_file}"]
    return args


def main():
    import yaml

    from metrics import validate_scenario

    root = Path(__file__).resolve().parent.parent
    target = sys.argv[1]
    if target.endswith(".yaml"):
        scenario = validate_scenario(yaml.safe_load(Path(target).read_text()))
        world, start = scenario["world"], scenario["start"]
    else:
        world, start = target, default_start(target)
    print(" ".join(launch_args(world, start, root)))


if __name__ == "__main__":
    main()
