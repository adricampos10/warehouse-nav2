"""Checks on maps/house, the map I made from my iPhone scan with scan_to_map.py,
and on the scenarios that use it.

scan_to_map.py needs numpy and OpenCV, so it runs in the container; these
tests only read the map files, with plain Python.
"""
import math
from pathlib import Path

import pytest
import yaml

from make_warehouse import FREE, OCCUPIED, UNKNOWN
from map_to_world import Map
from metrics import ROBOT_RADIUS, validate_scenario

ROOT = Path(__file__).parent.parent
HOUSE = Map(ROOT / "maps" / "house.yaml")
SCENARIOS = [validate_scenario(yaml.safe_load(p.read_text()))
             for p in sorted((ROOT / "scenarios").glob("house_*.yaml"))]


def test_map_metadata():
    assert HOUSE.resolution == 0.05
    assert yaml.safe_load((ROOT / "maps" / "house.yaml").read_text())["mode"] == "trinary"


def test_map_only_has_free_occupied_and_unknown_cells():
    assert {v for row in HOUSE.grid for v in row} == {FREE, OCCUPIED, UNKNOWN}


def test_where_i_started_the_scan_is_free():
    assert HOUSE.value(0.0, 0.0) == FREE  # the map's origin is my starting point


def robot_fits(x, y):
    """Every cell under the robot's footprint (a circle) is free."""
    rim = [(ROBOT_RADIUS * math.cos(a), ROBOT_RADIUS * math.sin(a))
           for a in [k * math.pi / 8 for k in range(16)]]
    return all(HOUSE.value(x + dx, y + dy) == FREE for dx, dy in [(0, 0), *rim])


def test_there_are_house_scenarios():
    assert SCENARIOS and all(s["world"] == "house" for s in SCENARIOS)


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda s: s["description"])
def test_house_scenarios_start_and_end_where_the_robot_fits(scenario):
    for x, y in [scenario["start"], *scenario["goals"]]:
        assert robot_fits(x, y), f"the robot doesn't fit at ({x}, {y})"
