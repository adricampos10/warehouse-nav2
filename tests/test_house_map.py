"""Checks on maps/house, the map I made from my iPhone scan with scan_to_map.py.

scan_to_map.py needs numpy and OpenCV, so it runs in the container; these
tests only read the map files, with plain Python.
"""
from pathlib import Path

import pytest
import yaml

from make_warehouse import FREE, OCCUPIED, UNKNOWN

MAPS = Path(__file__).parent.parent / "maps"


def read_pgm(path):
    """Width, height and pixels (rows from the top) of a binary PGM."""
    data = path.read_bytes()
    fields, pos = [], 0
    while len(fields) < 4:  # magic, width, height, max value
        while data[pos:pos + 1].isspace():
            pos += 1
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    assert fields[0] == b"P5"
    width, height = int(fields[1]), int(fields[2])
    pixels = data[pos + 1:]
    assert len(pixels) == width * height
    return width, height, pixels


@pytest.fixture(scope="module")
def house():
    meta = yaml.safe_load((MAPS / "house.yaml").read_text())
    width, height, pixels = read_pgm(MAPS / meta["image"])
    return meta, width, height, pixels


def value_at(house, x, y):
    meta, width, height, pixels = house
    ox, oy, _ = meta["origin"]
    col = int((x - ox) / meta["resolution"])
    row = height - 1 - int((y - oy) / meta["resolution"])
    return pixels[row * width + col]


def test_map_metadata(house):
    meta = house[0]
    assert meta["resolution"] == 0.05
    assert meta["mode"] == "trinary"


def test_map_only_has_free_occupied_and_unknown_cells(house):
    assert set(house[3]) == {FREE, OCCUPIED, UNKNOWN}


def test_where_i_started_the_scan_is_free(house):
    assert value_at(house, 0.0, 0.0) == FREE  # the map's origin is my starting point
