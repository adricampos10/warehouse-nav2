"""Unit tests for scripts/worlds.py, and a check that warehouse scenarios fit the map."""
from pathlib import Path

import pytest
import yaml

import make_warehouse as wh
from metrics import validate_scenario
from worlds import WORLDS, default_start, launch_args

ROOT = Path(__file__).parent.parent
SCENARIOS = [validate_scenario(yaml.safe_load(p.read_text()))
             for p in sorted((ROOT / "scenarios").glob("*.yaml"))]


def test_sandbox_only_sets_the_spawn_position():
    assert launch_args("tb3_sandbox", (-2.0, -0.5), "/ws") == {"x_pose": "-2.0", "y_pose": "-0.5"}


def test_warehouse_sets_world_and_map():
    assert launch_args("warehouse", (-5.5, 0.0), "/ws") == {
        "x_pose": "-5.5", "y_pose": "0.0",
        "world": "/ws/worlds/warehouse.sdf.xacro", "map": "/ws/maps/warehouse.yaml",
    }


@pytest.mark.parametrize("world", WORLDS)
def test_world_files_exist(world):
    for f in WORLDS[world][:2]:
        assert f is None or (ROOT / f).exists()


def test_warehouse_default_start_is_free():
    grid = wh.rasterize(wh.boxes())
    assert wh.cell_value(grid, *default_start("warehouse")) == wh.FREE


@pytest.mark.parametrize(
    "scenario", [s for s in SCENARIOS if s["world"] == "warehouse"],
    ids=lambda s: s["description"])
def test_warehouse_scenarios_start_and_end_in_free_space(scenario):
    grid = wh.rasterize(wh.boxes())
    for x, y in [scenario["start"], *scenario["goals"]]:
        assert wh.cell_value(grid, x, y) == wh.FREE, f"({x}, {y}) is not free"
