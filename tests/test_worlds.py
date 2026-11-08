"""Unit tests for scripts/worlds.py, and a check that warehouse scenarios fit the map."""
import math
from pathlib import Path

import pytest
import yaml

import make_warehouse as wh
from metrics import validate_scenario
from obstacles import RADIUS
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


def along(path, step=0.05):
    """Points every step metres along a path of (x, y) points."""
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        n = max(1, math.ceil(math.dist((x0, y0), (x1, y1)) / step))
        for k in range(n + 1):
            yield x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n


@pytest.mark.parametrize(
    "scenario", [s for s in SCENARIOS if s["world"] == "warehouse" and s["obstacles"]],
    ids=lambda s: s["description"])
def test_warehouse_obstacles_walk_in_free_space(scenario):
    grid = wh.rasterize(wh.boxes())
    rim = [(RADIUS * math.cos(a), RADIUS * math.sin(a)) for a in [k * math.pi / 4 for k in range(8)]]
    for obstacle in scenario["obstacles"]:
        for x, y in along(obstacle["path"]):
            for dx, dy in [(0, 0), *rim]:
                assert wh.cell_value(grid, x + dx, y + dy) == wh.FREE, f"({x:.2f}, {y:.2f}) hits the map"
