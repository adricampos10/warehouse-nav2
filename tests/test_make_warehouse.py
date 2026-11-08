"""Unit tests for scripts/make_warehouse.py: the layout, the map and the files in the repo."""
from pathlib import Path

import pytest

import make_warehouse as wh

ROOT = Path(__file__).parent.parent


@pytest.fixture(scope="module")
def grid():
    return wh.rasterize(wh.boxes())


def test_aisles_include_the_narrow_one():
    assert wh.aisle_widths() == [1.2, 1.2, 1.0]


def test_shelves_are_inside_the_building():
    for b in wh.boxes():
        if b.kind == "shelf":
            assert wh.X_MIN < b.cx - b.sx / 2 and b.cx + b.sx / 2 < wh.X_MAX
            assert wh.Y_MIN < b.cy - b.sy / 2 and b.cy + b.sy / 2 < wh.Y_MAX


def test_map_size_covers_the_building_and_margin(grid):
    width_m = len(grid[0]) * wh.RESOLUTION
    height_m = len(grid) * wh.RESOLUTION
    assert width_m == pytest.approx(wh.X_MAX - wh.X_MIN + 2 * (wh.WALL + wh.MARGIN))
    assert height_m == pytest.approx(wh.Y_MAX - wh.Y_MIN + 2 * (wh.WALL + wh.MARGIN))


@pytest.mark.parametrize("x, y, expected", [
    (-5.5, 0.0, wh.FREE),          # robot start in the loading area
    (1.0, -1.1, wh.FREE),          # middle of the narrow aisle
    (1.0, 3.3, wh.OCCUPIED),       # shelf A
    (1.0, -1.9, wh.OCCUPIED),      # shelf D
    (0.0, wh.Y_MAX + 0.07, wh.OCCUPIED),  # north wall
    (wh.X_MIN - 0.07, 0.0, wh.OCCUPIED),  # west wall
    (wh.X_MAX + 0.4, 0.0, wh.UNKNOWN),    # outside the building
])
def test_map_cells_match_the_layout(grid, x, y, expected):
    assert wh.cell_value(grid, x, y) == expected


def test_map_rows_are_top_first(grid):
    # Shelf A is near the top of the building, so it must be in the upper half of the image.
    ox, oy = wh.map_origin()
    row_of_shelf_a = len(grid) - 1 - int((3.3 - oy) / wh.RESOLUTION)
    assert row_of_shelf_a < len(grid) / 2
    assert grid[row_of_shelf_a][int((1.0 - ox) / wh.RESOLUTION)] == wh.OCCUPIED


def test_world_has_one_model_per_box():
    sdf = wh.world_sdf(wh.boxes())
    for b in wh.boxes():
        assert f'<model name="{b.name}">' in sdf


def test_world_publishes_poses_even_headless():
    # move_obstacles.py reads /world/default/pose/info from the scene broadcaster
    sdf = wh.world_sdf(wh.boxes())
    assert "gz::sim::systems::SceneBroadcaster" in sdf
    assert "xacro:unless" not in sdf


def test_generated_files_are_up_to_date():
    # If this fails, run: python3 scripts/make_warehouse.py
    layout = wh.boxes()
    assert (ROOT / "worlds" / "warehouse.sdf.xacro").read_text() == wh.world_sdf(layout)
    assert (ROOT / "maps" / "warehouse.pgm").read_bytes() == wh.pgm(wh.rasterize(layout))
    assert (ROOT / "maps" / "warehouse.yaml").read_text() == wh.map_yaml("warehouse.pgm")
