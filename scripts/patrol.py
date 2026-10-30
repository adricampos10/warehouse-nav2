#!/usr/bin/env python3
"""Drive the robot around a loop of waypoints so I can watch it navigate.

Run inside the container while run_sim_viz.sh is up:
    python3 /ws/scripts/patrol.py [rounds]
"""
import sys

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from smoke_test import make_pose

# Free spots between the pillars of the TurtleBot3 example world (map frame).
WAYPOINTS = [(-1.5, 1.6), (1.8, 0.3), (0.5, -0.5), (-2.0, -0.5)]


def main():
    rounds = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer="robot_localization")

    for r in range(1, rounds + 1):
        for x, y in WAYPOINTS:
            print(f"Round {r}/{rounds}: going to ({x}, {y})...", flush=True)
            navigator.goToPose(make_pose(navigator, x, y))
            while not navigator.isTaskComplete():
                rclpy.spin_once(navigator, timeout_sec=0.5)
            result = navigator.getResult()
            print(f"  {'reached' if result == TaskResult.SUCCEEDED else result}", flush=True)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
