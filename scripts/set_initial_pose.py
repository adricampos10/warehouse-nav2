#!/usr/bin/env python3
"""Set the robot's initial pose and wait until Nav2 is active.

Nav2 aborts its bringup if it does not get an initial pose in time, so
run_sim_viz.sh calls this right after starting the simulation. After that,
goals can be sent from Foxglove / Lichtblick (topic /goal_pose).

Usage: python3 set_initial_pose.py [world]   (default: tb3_sandbox)
"""
import sys

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

from nav_utils import make_pose
from worlds import DEFAULT_WORLD, default_start


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_WORLD
    rclpy.init()
    navigator = BasicNavigator()
    # The robot spawns at the world's default start (see worlds.py).
    navigator.setInitialPose(make_pose(navigator, *default_start(world)))
    navigator.waitUntilNav2Active()
    print("Nav2 is active: send goals on /goal_pose", flush=True)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
