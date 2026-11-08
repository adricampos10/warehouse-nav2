#!/usr/bin/env python3
"""Set the robot's initial pose and wait until Nav2 is active.

Nav2 aborts its bringup if it does not get an initial pose in time, so
run_sim_viz.sh calls this right after starting the simulation. After that,
goals can be sent from Foxglove / Lichtblick (topic /goal_pose).

Usage: python3 set_initial_pose.py [world]           (default: tb3_sandbox)
       python3 set_initial_pose.py scenario.yaml   (the scenario's start)
"""
import sys
from pathlib import Path

import rclpy
import yaml
from nav2_simple_commander.robot_navigator import BasicNavigator

from metrics import validate_scenario
from nav_utils import make_pose
from worlds import DEFAULT_WORLD, default_start


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_WORLD
    # The robot spawns at the scenario's start, or at the world's default
    # start (see worlds.py), like sim.launch.py does.
    if target.endswith(".yaml"):
        start = validate_scenario(yaml.safe_load(Path(target).read_text()))["start"]
    else:
        start = default_start(target)
    rclpy.init()
    navigator = BasicNavigator()
    navigator.setInitialPose(make_pose(navigator, *start))
    navigator.waitUntilNav2Active()
    print("Nav2 is active: send goals on /goal_pose", flush=True)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
