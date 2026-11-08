"""Unit tests for scripts/metrics.py. They run in milliseconds, without ROS."""
import math
from pathlib import Path

import pytest
import yaml

from metrics import (ROBOT_RADIUS, edge_clearance, evaluate, format_table, obstacle_gap,
                     path_length, validate_scenario)

SCENARIOS = sorted((Path(__file__).parent.parent / "scenarios").glob("*.yaml"))


def scenario(**overrides):
    data = {"start": [0, 0], "goals": [[1, 0], [2, 0]], "timeout": 60}
    data.update(overrides)
    return validate_scenario(data)


def run(goals_reached=2, timed_out=False, clearance=0.3):
    return {"goals_reached": goals_reached, "timed_out": timed_out, "clearance": clearance}


# --- validate_scenario -------------------------------------------------------

def test_defaults_are_filled_in():
    s = scenario()
    assert s["expect"] == "success"
    assert s["world"] == "tb3_sandbox"
    assert s["min_clearance"] == pytest.approx(0.05)
    assert s["goals"] == [(1.0, 0.0), (2.0, 0.0)]


@pytest.mark.parametrize("bad, message", [
    ({"goals": [[1, 0]], "timeout": 60}, "missing key: start"),
    ({"start": [0, 0], "goals": [], "timeout": 60}, "non-empty"),
    ({"start": [0, 0], "goals": [[1, 0, 0]], "timeout": 60}, r"\[x, y\]"),
    ({"start": [0, 0], "goals": [[1, 0]], "timeout": -5}, "positive"),
    ({"start": [0, 0], "goals": [[1, 0]], "timeout": 60, "expect": "maybe"}, "expect"),
    ({"start": [0, 0], "goals": [[1, 0]], "timeout": 60, "world": "moon"}, "world"),
])
def test_invalid_scenarios_are_rejected(bad, message):
    with pytest.raises(ValueError, match=message):
        validate_scenario(bad)


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
def test_every_scenario_file_is_valid(path):
    validate_scenario(yaml.safe_load(path.read_text()))


# --- path_length and edge_clearance -------------------------------------------

def test_path_length():
    assert path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7.0)


@pytest.mark.parametrize("points", [[], [(1, 1)]])
def test_path_length_of_no_movement_is_zero(points):
    assert path_length(points) == 0


def test_robot_radius_covers_the_waffle():
    assert ROBOT_RADIUS == pytest.approx(0.22)  # Nav2's robot_radius
    assert ROBOT_RADIUS >= math.hypot(0.281, 0.306) / 2  # half the Waffle's diagonal


def test_edge_clearance_is_measured_from_the_robot_edge():
    assert edge_clearance([(0.30, 0.0)]) == pytest.approx(0.08)


def test_edge_clearance_uses_the_closest_point_in_any_direction():
    points = [(1.0, 0.0), (0.0, -0.40), (-0.25, 0.0), (0.3, 0.4)]
    assert edge_clearance(points) == pytest.approx(0.03)  # the one behind the robot


def test_edge_clearance_without_points_is_none():
    assert edge_clearance([]) is None


# --- evaluate -----------------------------------------------------------------

def test_all_goals_reached_passes():
    assert evaluate(scenario(), run()) == (True, "all goals reached")


def test_some_goals_missed_fails():
    passed, reason = evaluate(scenario(), run(goals_reached=1))
    assert not passed
    assert reason == "reached 1/2 goals"


def test_timeout_fails_even_if_goals_count_matches():
    passed, reason = evaluate(scenario(), run(timed_out=True))
    assert not passed
    assert "timed out" in reason


def test_too_close_to_an_obstacle_fails():
    passed, reason = evaluate(scenario(), run(clearance=0.03))
    assert not passed
    assert reason == "too close to an obstacle: 0.03 m from the robot's edge (limit 0.05 m)"


def test_just_outside_the_clearance_limit_passes():
    assert evaluate(scenario(), run(clearance=0.06))[0]


def test_a_scenario_can_ask_for_more_clearance():
    assert not evaluate(scenario(min_clearance=0.10), run(clearance=0.06))[0]


def test_missing_laser_data_fails():
    passed, reason = evaluate(scenario(), run(clearance=None))
    assert not passed
    assert "/scan or TF missing" in reason


def test_expected_failure_does_not_need_laser_data():
    assert evaluate(scenario(expect="failure"), run(goals_reached=0, clearance=None))[0]


def test_expected_failure_that_fails_passes():
    s = scenario(expect="failure")
    assert evaluate(s, run(goals_reached=0)) == (True, "failed as expected")


def test_expected_failure_that_succeeds_fails():
    assert not evaluate(scenario(expect="failure"), run())[0]


# --- moving obstacles: obstacle_gap and contact --------------------------------

PEOPLE = [{"path": [[0, 1], [1, 1]]}]


def test_obstacle_gap_is_between_the_two_edges():
    # 0.22 m robot radius + 0.25 m person radius
    assert obstacle_gap((0, 0), (1.0, 0)) == pytest.approx(0.53)
    assert obstacle_gap((0, 0), (0.4, 0)) == pytest.approx(-0.07)  # touching


def test_hitting_a_moving_obstacle_fails():
    s = scenario(world="warehouse", obstacles=PEOPLE)
    passed, reason = evaluate(s, {**run(), "obstacle_gap": -0.13})
    assert not passed
    assert reason == "hit a moving obstacle: 0.13 m of overlap"


def test_passing_close_to_a_moving_obstacle_is_fine():
    s = scenario(world="warehouse", obstacles=PEOPLE)
    assert evaluate(s, {**run(), "obstacle_gap": 0.02})[0]


def test_moving_obstacles_without_gazebo_poses_fail():
    s = scenario(world="warehouse", obstacles=PEOPLE)
    passed, reason = evaluate(s, {**run(), "obstacle_gap": None})
    assert not passed
    assert "contact could not be checked" in reason


# --- format_table -------------------------------------------------------------

def test_format_table():
    results = [
        {"name": "b", "passed": False, "goals_reached": 0, "goals_total": 1,
         "elapsed": 12.34, "path_length": None, "clearance": None, "reason": "x"},
        {"name": "a", "passed": True, "goals_reached": 1, "goals_total": 1,
         "elapsed": 20.0, "path_length": 2.5, "clearance": 0.09, "obstacle_gap": 0.4,
         "reason": "ok"},
    ]
    table = format_table(results)
    lines = table.splitlines()
    assert lines[2] == "| a | PASS | 1/1 | 20.0 | 2.50 | 0.09 | 0.40 | ok |"
    assert lines[3] == "| b | FAIL | 0/1 | 12.3 | - | - | - | x |"
    assert table.endswith("1/2 scenarios passed")
