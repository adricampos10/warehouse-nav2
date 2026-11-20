"""Small helpers shared by the scripts that talk to Nav2."""
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState

# s of my computer's time for Nav2 to become active (an env var, to test the failure path)
STARTUP_TIMEOUT = float(os.environ.get("NAV2_STARTUP_TIMEOUT", 180))
INFRA_EXIT_CODE = 3       # "Nav2 didn't start": the shell runners retry once on it


def make_pose(navigator, x, y):
    """Pose at (x, y) in the map frame, facing +x, stamped with the current time."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def wait_for_nav2(navigator, start, timeout=STARTUP_TIMEOUT):
    """Set the initial pose and wait until Nav2 is active; False if it isn't within timeout.

    BasicNavigator.waitUntilNav2Active() does the same, but it can hang
    forever: it waits for each lifecycle get_state answer without a time
    limit, and on GitHub Actions one answer got lost and a scenario sat
    there for 10 minutes. It also re-sends the initial pose with its first
    timestamp, which AMCL can't use once it's older than its TF buffer.
    Here every call has a time limit and the whole startup has a deadline,
    and the initial pose is re-stamped on every attempt.
    """
    deadline = time.monotonic() + timeout

    def active(node):
        client = navigator.create_client(GetState, f"{node}/get_state")
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(navigator, future, timeout_sec=2.0)
                if future.done() and future.result() and future.result().current_state.label == "active":
                    return True
            navigator.info(f"{node} not active yet, waiting...")
        return False

    if not active("amcl"):
        return False
    navigator.initial_pose_received = False
    while not navigator.initial_pose_received:  # set by BasicNavigator's /amcl_pose callback
        if time.monotonic() >= deadline:
            return False
        navigator.setInitialPose(make_pose(navigator, *start))
        rclpy.spin_once(navigator, timeout_sec=1.0)
    return active("bt_navigator")
