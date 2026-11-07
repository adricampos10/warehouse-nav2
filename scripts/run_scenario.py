#!/usr/bin/env python3
"""Run one navigation scenario in the simulation and save its result.

Run inside the container while the Nav2 simulation is up:
    python3 /ws/scripts/run_scenario.py /ws/scenarios/short_goal.yaml /ws/results

It drives the robot through the scenario's goals, records the time (simulated
seconds, from Gazebo's /clock, so a slow machine doesn't make it time out), the path
travelled (/odom) and how close the robot's edge came to an obstacle (/scan,
moved into the robot's frame with TF), and writes
<results>/<scenario>.json. Exit code 0 means the scenario passed.
"""
import json
import sys
import time
from pathlib import Path

import rclpy
import tf2_ros
import yaml
from laser_geometry import LaserProjection
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2
from tf2_sensor_msgs import do_transform_cloud

from metrics import edge_clearance, evaluate, path_length, validate_scenario
from nav_utils import make_pose


ROBOT_FRAME = "base_link"  # the frame the robot's footprint is centred on


class Recorder:
    """Collects robot positions and the smallest edge clearance while recording."""

    def __init__(self, node):
        self.recording = False
        self.positions = []
        self.min_clearance = None
        self.projector = LaserProjection()
        # TF knows where the laser is mounted (a static transform from the URDF)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        # Sensor-data QoS (best effort) matches the simulation whether it
        # publishes these topics as reliable or best effort.
        node.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_odom(self, msg):
        if self.recording:
            p = msg.pose.pose.position
            self.positions.append((p.x, p.y))

    def on_scan(self, msg):
        if not self.recording:
            return
        try:
            laser_to_robot = self.tf_buffer.lookup_transform(ROBOT_FRAME, msg.header.frame_id, Time())
        except tf2_ros.TransformException:
            return  # TF not received yet; the next scan tries again
        # Ranges (polar, in the laser's frame) -> points (x, y) in the robot's frame.
        # projectLaser drops readings outside [range_min, range_max], inf and NaN.
        cloud = do_transform_cloud(self.projector.projectLaser(msg), laser_to_robot)
        points = [(float(p[0]), float(p[1]))
                  for p in point_cloud2.read_points(cloud, field_names=("x", "y"), skip_nans=True)]
        gap = edge_clearance(points)
        if gap is not None and (self.min_clearance is None or gap < self.min_clearance):
            self.min_clearance = gap


def current_sim_time(node):
    """The simulated time, after spinning until the node's clock moves.

    With use_sim_time the clock only moves when a /clock message is handled
    while the node spins, and waitUntilNav2Active() ends with a 2 s sleep that
    doesn't spin, so without this the clock can be seconds behind.
    """
    clock = node.get_clock()
    last = clock.now()
    while clock.now() == last:
        rclpy.spin_once(node, timeout_sec=0.1)
    return clock.now()


def main():
    scenario_path, results_dir = Path(sys.argv[1]), Path(sys.argv[2])
    name = scenario_path.stem
    scenario = validate_scenario(yaml.safe_load(scenario_path.read_text()))

    rclpy.init()
    navigator = BasicNavigator()
    # Use Gazebo's /clock instead of my computer's clock (simulated time is 0
    # until the first /clock message arrives).
    navigator.set_parameters([Parameter("use_sim_time", value=True)])
    clock = navigator.get_clock()
    current_sim_time(navigator)
    recorder = Recorder(navigator)  # isTaskComplete() spins the navigator, so these callbacks run

    navigator.setInitialPose(make_pose(navigator, *scenario["start"]))
    print(f"[{name}] Waiting for Nav2 to become active...", flush=True)
    navigator.waitUntilNav2Active()

    recorder.recording = True
    sim_start, wall_start = current_sim_time(navigator), time.monotonic()

    def sim_elapsed():
        return (clock.now() - sim_start).nanoseconds / 1e9

    goals_reached, timed_out = 0, False
    for x, y in scenario["goals"]:
        print(f"[{name}] Going to ({x}, {y})...", flush=True)
        navigator.goToPose(make_pose(navigator, x, y))
        while not navigator.isTaskComplete():
            if sim_elapsed() > scenario["timeout"]:
                navigator.cancelTask()
                timed_out = True
                break
        if timed_out or navigator.getResult() != TaskResult.SUCCEEDED:
            break
        goals_reached += 1
    elapsed, wall_elapsed = sim_elapsed(), time.monotonic() - wall_start
    recorder.recording = False

    passed, reason = evaluate(scenario, {
        "goals_reached": goals_reached, "timed_out": timed_out, "clearance": recorder.min_clearance,
    })
    result = {
        "name": name,
        "description": scenario["description"],
        "expect": scenario["expect"],
        "passed": passed,
        "reason": reason,
        "goals_reached": goals_reached,
        "goals_total": len(scenario["goals"]),
        "elapsed": round(elapsed, 2),  # simulated seconds
        "wall_elapsed": round(wall_elapsed, 2),  # real seconds on this machine
        "path_length": round(path_length(recorder.positions), 3),
        "clearance": None if recorder.min_clearance is None else round(recorder.min_clearance, 3),
    }
    results_dir.mkdir(parents=True, exist_ok=True)
    (results_dir / f"{name}.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"[{name}] {'PASS' if passed else 'FAIL'}: {reason}", flush=True)

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
