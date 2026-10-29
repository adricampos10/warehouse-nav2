#!/usr/bin/env python3
"""Smoke test: check that Nav2 can drive the simulated robot to a goal.

Run inside the container while the Nav2 simulation is up:
    python3 /ws/scripts/smoke_test.py
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # The TurtleBot3 example world spawns the robot at (-2.0, -0.5).
    navigator.setInitialPose(make_pose(navigator, -2.0, -0.5))
    print("Waiting for Nav2 to become active...", flush=True)
    navigator.waitUntilNav2Active()

    goal = make_pose(navigator, 0.5, -0.5)
    print("Sending goal (0.5, -0.5)...", flush=True)
    start = time.time()
    navigator.goToPose(goal)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"  distance remaining: {feedback.distance_remaining:.2f} m", flush=True)
        if time.time() - start > 120:
            navigator.cancelTask()
            break
        time.sleep(2)

    result = navigator.getResult()
    elapsed = time.time() - start
    ok = result == TaskResult.SUCCEEDED
    print(f"RESULT: {'SUCCEEDED' if ok else result} in {elapsed:.1f} s", flush=True)

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
