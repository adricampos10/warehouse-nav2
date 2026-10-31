#!/usr/bin/env python3
"""Run one navigation scenario in the simulation and save its result.

Run inside the container while the Nav2 simulation is up:
    python3 /ws/scripts/run_scenario.py /ws/scenarios/short_goal.yaml /ws/results

It drives the robot through the scenario's goals, records the time, the path
travelled (/odom) and the closest laser reading (/scan), and writes
<results>/<scenario>.json. Exit code 0 means the scenario passed.
"""
import json
import sys
import time
from pathlib import Path

import rclpy
import yaml
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from metrics import evaluate, min_valid_range, path_length, validate_scenario
from smoke_test import make_pose


class Recorder:
    """Collects robot positions and the closest laser reading while recording."""

    def __init__(self, node):
        self.recording = False
        self.positions = []
        self.min_range = None
        # Sensor-data QoS (best effort) matches the simulation whether it
        # publishes these topics as reliable or best effort.
        node.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_odom(self, msg):
        if self.recording:
            p = msg.pose.pose.position
            self.positions.append((p.x, p.y))

    def on_scan(self, msg):
        if self.recording:
            closest = min_valid_range(msg.ranges, msg.range_min, msg.range_max)
            if closest is not None and (self.min_range is None or closest < self.min_range):
                self.min_range = closest


def main():
    scenario_path, results_dir = Path(sys.argv[1]), Path(sys.argv[2])
    name = scenario_path.stem
    scenario = validate_scenario(yaml.safe_load(scenario_path.read_text()))

    rclpy.init()
    navigator = BasicNavigator()
    recorder = Recorder(navigator)  # isTaskComplete() spins the navigator, so these callbacks run

    navigator.setInitialPose(make_pose(navigator, *scenario["start"]))
    print(f"[{name}] Waiting for Nav2 to become active...", flush=True)
    navigator.waitUntilNav2Active()

    recorder.recording = True
    start = time.time()
    goals_reached, timed_out = 0, False
    for x, y in scenario["goals"]:
        print(f"[{name}] Going to ({x}, {y})...", flush=True)
        navigator.goToPose(make_pose(navigator, x, y))
        while not navigator.isTaskComplete():
            if time.time() - start > scenario["timeout"]:
                navigator.cancelTask()
                timed_out = True
                break
        if timed_out or navigator.getResult() != TaskResult.SUCCEEDED:
            break
        goals_reached += 1
    elapsed = time.time() - start
    recorder.recording = False

    passed, reason = evaluate(scenario, {
        "goals_reached": goals_reached, "timed_out": timed_out, "min_range": recorder.min_range,
    })
    result = {
        "name": name,
        "description": scenario["description"],
        "expect": scenario["expect"],
        "passed": passed,
        "reason": reason,
        "goals_reached": goals_reached,
        "goals_total": len(scenario["goals"]),
        "elapsed": round(elapsed, 2),
        "path_length": round(path_length(recorder.positions), 3),
        "min_range": None if recorder.min_range is None else round(recorder.min_range, 3),
    }
    results_dir.mkdir(parents=True, exist_ok=True)
    (results_dir / f"{name}.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"[{name}] {'PASS' if passed else 'FAIL'}: {reason}", flush=True)

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
