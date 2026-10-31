"""Unit tests for scripts/metrics.py. They run in milliseconds, without ROS."""
import math
from pathlib import Path

import pytest
import yaml

from metrics import (evaluate, format_table, min_valid_range, path_length,
                     validate_scenario)

SCENARIOS = sorted((Path(__file__).parent.parent / "scenarios").glob("*.yaml"))


def scenario(**overrides):
    data = {"start": [0, 0], "goals": [[1, 0], [2, 0]], "timeout": 60}
    data.update(overrides)
    return validate_scenario(data)


def run(goals_reached=2, timed_out=False, min_range=0.5):
    return {"goals_reached": goals_reached, "timed_out": timed_out, "min_range": min_range}


# --- validate_scenario -------------------------------------------------------

def test_defaults_are_filled_in():
    s = scenario()
    assert s["expect"] == "success"
    assert s["min_clearance"] == pytest.approx(0.15)
    assert s["goals"] == [(1.0, 0.0), (2.0, 0.0)]


@pytest.mark.parametrize("bad, message", [
    ({"goals": [[1, 0]], "timeout": 60}, "missing key: start"),
    ({"start": [0, 0], "goals": [], "timeout": 60}, "non-empty"),
    ({"start": [0, 0], "goals": [[1, 0, 0]], "timeout": 60}, r"\[x, y\]"),
    ({"start": [0, 0], "goals": [[1, 0]], "timeout": -5}, "positive"),
    ({"start": [0, 0], "goals": [[1, 0]], "timeout": 60, "expect": "maybe"}, "expect"),
])
def test_invalid_scenarios_are_rejected(bad, message):
    with pytest.raises(ValueError, match=message):
        validate_scenario(bad)


@pytest.mark.parametrize("path", SCENARIOS, ids=lambda p: p.stem)
def test_every_scenario_file_is_valid(path):
    validate_scenario(yaml.safe_load(path.read_text()))


# --- path_length and min_valid_range ------------------------------------------

def test_path_length():
    assert path_length([(0, 0), (3, 0), (3, 4)]) == pytest.approx(7.0)


@pytest.mark.parametrize("points", [[], [(1, 1)]])
def test_path_length_of_no_movement_is_zero(points):
    assert path_length(points) == 0


def test_min_valid_range_ignores_invalid_readings():
    ranges = [math.inf, 0.05, math.nan, 0.8, 0.4, 20.0]
    assert min_valid_range(ranges, range_min=0.12, range_max=3.5) == pytest.approx(0.4)


def test_min_valid_range_with_no_valid_readings_is_none():
    assert min_valid_range([math.inf, math.nan], 0.12, 3.5) is None


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
    passed, reason = evaluate(scenario(), run(min_range=0.10))
    assert not passed
    assert "too close" in reason


def test_missing_laser_data_does_not_fail_on_clearance():
    assert evaluate(scenario(), run(min_range=None))[0]


def test_expected_failure_that_fails_passes():
    s = scenario(expect="failure")
    assert evaluate(s, run(goals_reached=0)) == (True, "failed as expected")


def test_expected_failure_that_succeeds_fails():
    assert not evaluate(scenario(expect="failure"), run())[0]


# --- format_table -------------------------------------------------------------

def test_format_table():
    results = [
        {"name": "b", "passed": False, "goals_reached": 0, "goals_total": 1,
         "elapsed": 12.34, "path_length": None, "min_range": None, "reason": "x"},
        {"name": "a", "passed": True, "goals_reached": 1, "goals_total": 1,
         "elapsed": 20.0, "path_length": 2.5, "min_range": 0.31, "reason": "ok"},
    ]
    table = format_table(results)
    lines = table.splitlines()
    assert lines[2] == "| a | PASS | 1/1 | 20.0 | 2.50 | 0.31 | ok |"
    assert lines[3] == "| b | FAIL | 0/1 | 12.3 | - | - | x |"
    assert table.endswith("1/2 scenarios passed")
