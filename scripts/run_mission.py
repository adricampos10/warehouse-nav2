#!/usr/bin/env python3
"""Run a patrol mission (a py_trees behaviour tree) against Nav2 in the simulation.

Run inside the container while the simulation of the mission's world is up
(scripts/run_mission.sh does both):
    python3 /ws/scripts/run_mission.py /ws/missions/house_patrol.yaml /ws/results

The robot starts at the world's default start. The tree (scripts/mission.py)
decides where to go; this script only connects it to Nav2 and ticks it,
10 times a second of my computer's time, with the timeout in simulated
seconds. It prints the tree as it ends and writes
<results>/missions/mission_<name>.json, and its telemetry in
<results>/telemetry/. Exit code 0 means the mission passed.
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
from nav_utils import INFRA_EXIT_CODE, STARTUP_TIMEOUT, make_pose, wait_for_nav2
from run_scenario import Recorder, current_sim_time
from telemetry_ros import TelemetrySampler
from worlds import default_start

TICK_SECONDS = 0.1


class Nav2:
    """The navigation interface the tree uses (go_to, done, succeeded, cancel), on Nav2.

    Each goal the tree sends (retries included) is one leg of the telemetry.
    """

    def __init__(self, navigator, telemetry, recorder, now):
        self.navigator, self.telemetry, self.recorder, self.now = navigator, telemetry, recorder, now
        self.leg, self.goal, self.leg_start, self.feedback = -1, None, None, None

    def go_to(self, x, y):
        print(f"  -> going to ({x}, {y})", flush=True)
        self.leg, self.goal, self.leg_start, self.feedback = self.leg + 1, (x, y), self.now(), None
        self.navigator.goToPose(make_pose(self.navigator, x, y))

    def done(self):
        complete = self.navigator.isTaskComplete()  # also spins the node
        self.feedback = self.navigator.getFeedback() or self.feedback
        if not complete:
            self.telemetry.update(self.now(), self.leg, "driving", self.feedback, self.recorder.clearance)
        return complete

    def succeeded(self):
        ok = self.navigator.getResult() == TaskResult.SUCCEEDED
        self.end_leg("succeeded" if ok else "failed")
        return ok

    def cancel(self):
        self.navigator.cancelTask()
        self.end_leg("canceled")

    def end_leg(self, result):
        self.telemetry.update(self.now(), self.leg, result, self.feedback, self.recorder.clearance, force=True)
        self.telemetry.leg(self.leg, self.goal, self.leg_start, self.now(), result,
                           self.feedback.number_of_recoveries if self.feedback else 0)


def main():
    mission_path, results_dir = Path(sys.argv[1]), Path(sys.argv[2])
    name = mission_path.stem
    mission = validate_mission(yaml.safe_load(mission_path.read_text()))

    rclpy.init()
    navigator = BasicNavigator()
    navigator.set_parameters([Parameter("use_sim_time", value=True)])
    clock_runs = current_sim_time(navigator, timeout=STARTUP_TIMEOUT) is not None
    print(f"[{name}] Waiting for Nav2 to become active...", flush=True)
    if not clock_runs or not wait_for_nav2(navigator, default_start(mission["world"])):
        reason = (f"infrastructure: Nav2 didn't become active in {STARTUP_TIMEOUT:.0f} s" if clock_runs
                  else "infrastructure: the simulation clock never started")
        (results_dir / "missions").mkdir(parents=True, exist_ok=True)
        (results_dir / "missions" / f"mission_{name}.json").write_text(json.dumps({
            "name": name, "description": mission["description"], "passed": False, "infra": True,
            "reason": reason, "visited": [], "skipped": [], "attempts": {}, "elapsed": None,
        }, indent=2) + "\n")
        print(f"[{name}] FAIL: {reason}", flush=True)
        sys.exit(INFRA_EXIT_CODE)

    clock, start = navigator.get_clock(), current_sim_time(navigator)

    def sim_elapsed():
        return (clock.now() - start).nanoseconds / 1e9

    recorder = Recorder(navigator)
    recorder.recording = True
    telemetry = TelemetrySampler(navigator, results_dir / "telemetry", f"mission_{name}")
    log = Log()
    tree = build_tree(mission, Nav2(navigator, telemetry, recorder, sim_elapsed), log)
    print(py_trees.display.unicode_tree(tree), flush=True)

    status = run(tree,
                 tick=lambda: rclpy.spin_once(navigator, timeout_sec=TICK_SECONDS),
                 timed_out=lambda: sim_elapsed() > mission["timeout"])
    print(py_trees.display.unicode_tree(tree, show_status=True), flush=True)
    telemetry.close()

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
    (results_dir / "missions").mkdir(parents=True, exist_ok=True)
    (results_dir / "missions" / f"mission_{name}.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"[{name}] {'PASS' if passed else 'FAIL'}: {reason}", flush=True)

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
