#!/usr/bin/env python3
"""Set the robot's initial pose and wait until Nav2 is active.

Nav2 aborts its bringup if it does not get an initial pose in time, so
run_sim_viz.sh calls this right after starting the simulation. After that,
goals can be sent from Foxglove / Lichtblick (topic /goal_pose).
"""
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator

from smoke_test import make_pose


def main():
    rclpy.init()
    navigator = BasicNavigator()
    # The TurtleBot3 example world spawns the robot at (-2.0, -0.5).
    navigator.setInitialPose(make_pose(navigator, -2.0, -0.5))
    navigator.waitUntilNav2Active()
    print("Nav2 is active: send goals on /goal_pose", flush=True)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
