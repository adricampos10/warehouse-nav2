"""Unit tests for scripts/map_to_world.py: Gazebo walls from a Nav2 map."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from make_warehouse import FREE, OCCUPIED, UNKNOWN
from map_to_world import Map, read_pgm, rectangles, walls, world_from_map

ROOT = Path(__file__).parent.parent
X, O, U = OCCUPIED, FREE, UNKNOWN


def covered(rects, rows, cols):
    cells = set()
    for r, c, nr, nc in rects:
        cells |= {(r + i, c + j) for i in range(nr) for j in range(nc)}
    return cells


@pytest.mark.parametrize("grid", [
    [[X, X, O], [X, X, O], [O, O, X]],
    [[X, O, X], [X, X, X], [U, X, O]],
    [[O, O], [O, O]],
    [[X] * 5] * 4,
])
def test_rectangles_cover_exactly_the_occupied_cells(grid):
    rects = rectangles(grid)
    cells = covered(rects, len(grid), len(grid[0]))
    assert cells == {(r, c) for r, row in enumerate(grid) for c, v in enumerate(row) if v == X}
    assert sum(nr * nc for _, _, nr, nc in rects) == len(cells)  # no overlaps


def test_runs_below_each_other_merge_into_one_box():
    assert rectangles([[X, X, O], [X, X, O]]) == [(0, 0, 2, 2)]


def test_read_pgm():
    assert read_pgm(b"P5\n3 2\n255\n" + bytes([1, 2, 3, 4, 5, 6])) == [[1, 2, 3], [4, 5, 6]]
    with pytest.raises(ValueError):
        read_pgm(b"P5\n3 2\n255\n" + bytes([1, 2]))


def test_walls_are_placed_in_metres(tmp_path):
    # a 2 x 3 map, 0.5 m cells, bottom-left corner at (-1, 0); one occupied cell top right
    (tmp_path / "m.pgm").write_bytes(b"P5\n3 2\n255\n" + bytes([O, O, X, O, O, O]))
    (tmp_path / "m.yaml").write_text("image: m.pgm\nresolution: 0.5\norigin: [-1.0, 0.0, 0.0]\n")
    [box] = walls(Map(tmp_path / "m.yaml"))
    assert (box.cx, box.cy, box.sx, box.sy) == (0.25, 0.75, 0.5, 0.5)


def test_house_walls_are_one_static_model():
    walls_model = ET.parse(ROOT / "worlds" / "house_walls.sdf").find("model[@name='walls']")
    assert walls_model.find("static").text == "true"


def test_house_world_includes_the_walls_and_publishes_poses():
    sdf = (ROOT / "worlds" / "house.sdf.xacro").read_text()
    assert "<uri>file://${xacro.abs_filename('house_walls.sdf')}</uri>" in sdf
    assert "gz::sim::systems::SceneBroadcaster" in sdf  # for moving obstacles
    assert len(sdf) < 5000  # small, so xacro is done before Gazebo opens it


def test_house_world_is_up_to_date():
    # If this fails, run: python3 scripts/map_to_world.py maps/house.yaml worlds/house.sdf.xacro
    world, walls_sdf = world_from_map("maps/house.yaml", "worlds/house.sdf.xacro", ROOT)
    assert (ROOT / "worlds" / "house.sdf.xacro").read_text() == world
    assert (ROOT / "worlds" / "house_walls.sdf").read_text() == walls_sdf
