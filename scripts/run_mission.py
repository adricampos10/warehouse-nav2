#!/usr/bin/env python3
"""Run a patrol mission (a py_trees behaviour tree) against Nav2 in the simulation.

Run inside the container while the simulation of the mission's world is up
(scripts/run_mission.sh does both):
    python3 /ws/scripts/run_mission.py /ws/missions/house_patrol.yaml /ws/results

The robot starts at the world's default start. The tree (scripts/mission.py)
decides where to go; this script only connects it to Nav2 and ticks it,
10 times a second of my computer's time, with the timeout in simulated
seconds. It prints the tree as it ends and writes
<results>/mission_<name>.json. Exit code 0 means the mission passed.
"""
import json
import sys
from pathlib import Path

import py_trees
import rclpy
import yaml
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.parameter import Parameter

from mission import Log, build_tree, outcome, run, validate_mission
from nav_utils import make_pose
from run_scenario import current_sim_time
from worlds import default_start

TICK_SECONDS = 0.1


class Nav2:
    """The navigation interface the tree uses (go_to, done, succeeded, cancel), on Nav2."""

    def __init__(self, navigator):
        self.navigator = navigator

    def go_to(self, x, y):
        print(f"  -> going to ({x}, {y})", flush=True)
        self.navigator.goToPose(make_pose(self.navigator, x, y))

    def done(self):
        return self.navigator.isTaskComplete()  # also spins the node

    def succeeded(self):
        return self.navigator.getResult() == TaskResult.SUCCEEDED

    def cancel(self):
        self.navigator.cancelTask()


def main():
    mission_path, results_dir = Path(sys.argv[1]), Path(sys.argv[2])
    name = mission_path.stem
    mission = validate_mission(yaml.safe_load(mission_path.read_text()))

    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter("use_sim_time", value=True)])
    current_sim_time(navigator)
    navigator.setInitialPose(make_pose(navigator, *default_start(mission["world"])))
    print(f"[{name}] Waiting for Nav2 to become active...", flush=True)
    navigator.waitUntilNav2Active()

    log = Log()
    tree = build_tree(mission, Nav2(navigator), log)
    print(py_trees.display.unicode_tree(tree), flush=True)
    clock, start = navigator.get_clock(), current_sim_time(navigator)

    def sim_elapsed():
        return (clock.now() - start).nanoseconds / 1e9

    status = run(tree,
                 tick=lambda: rclpy.spin_once(navigator, timeout_sec=TICK_SECONDS),
                 timed_out=lambda: sim_elapsed() > mission["timeout"])
    print(py_trees.display.unicode_tree(tree, show_status=True), flush=True)

    passed, reason = outcome(status, mission, log)
    result = {
        "name": name,
        "description": mission["description"],
        "passed": passed,
        "reason": reason,
        "visited": log.visited,
        "skipped": log.skipped,
        "attempts": log.attempts,
        "elapsed": round(sim_elapsed(), 2),  # simulated seconds
    }
    results_dir.mkdir(parents=True, exist_ok=True)
    (results_dir / f"mission_{name}.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"[{name}] {'PASS' if passed else 'FAIL'}: {reason}", flush=True)

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
