#!/usr/bin/env python3
"""Build a Gazebo world from a Nav2 map: every occupied cell becomes a wall.

My home map comes from an iPhone scan (scan_to_map.py), so it has no list of
walls like my warehouse. This script reads the map image and covers the
occupied cells with as few boxes as it can (runs of cells in a row, merged
with the same run in the rows below), 1 m high. The robot's laser then sees
in Gazebo the same walls that Nav2 has in its map.

Usage (from the repo root):
    python3 scripts/map_to_world.py maps/house.yaml worlds/house.sdf.xacro

It writes the world and, next to it, the walls (worlds/house_walls.sdf).

No ROS imports or numpy, so it is unit-tested with pytest.
"""
import sys
from pathlib import Path

import yaml

from make_warehouse import COLOURS, HEIGHT, OCCUPIED, Box, world_sdf


def read_pgm(data):
    """Rows of pixel values (top row first) of a binary PGM image."""
    fields, pos = [], 0
    while len(fields) < 4:  # magic, width, height, max value
        while data[pos:pos + 1].isspace():
            pos += 1
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    if fields[0] != b"P5":
        raise ValueError("not a binary PGM image")
    width, height = int(fields[1]), int(fields[2])
    pixels = data[pos + 1:pos + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError("PGM image is shorter than its size")
    return [list(pixels[r * width:(r + 1) * width]) for r in range(height)]


class Map:
    """A Nav2 map: the grid (top row first) and where it sits in the world."""

    def __init__(self, yaml_path):
        yaml_path = Path(yaml_path)
        meta = yaml.safe_load(yaml_path.read_text())
        self.resolution = meta["resolution"]
        self.origin = meta["origin"][:2]  # bottom-left corner, metres
        self.grid = read_pgm((yaml_path.parent / meta["image"]).read_bytes())

    def cell(self, x, y):
        """(row, col) of the cell that contains the point (x, y)."""
        col = int((x - self.origin[0]) / self.resolution)
        row = len(self.grid) - 1 - int((y - self.origin[1]) / self.resolution)
        return row, col

    def value(self, x, y):
        row, col = self.cell(x, y)
        return self.grid[row][col]


def rectangles(grid, value=OCCUPIED):
    """Cover the cells equal to value with rectangles (row, col, rows, cols).

    Each row is split into runs of those cells; a run that is exactly below
    a rectangle of the previous row extends it downwards.
    """
    done, open_ = [], {}  # open_: (first col, last col) -> [row, col, rows, cols]
    for r, line in enumerate(grid + [[]]):  # an empty last row closes everything
        runs, c = [], 0
        while c < len(line):
            if line[c] == value:
                start = c
                while c < len(line) and line[c] == value:
                    c += 1
                runs.append((start, c - 1))
            else:
                c += 1
        still_open = {}
        for run in runs:
            if run in open_:
                open_[run][2] += 1
                still_open[run] = open_.pop(run)
            else:
                still_open[run] = [r, run[0], 1, run[1] - run[0] + 1]
        done.extend(tuple(rect) for rect in open_.values())
        open_ = still_open
    return sorted(done)


def walls(world_map):
    """The occupied cells of the map as boxes, in metres."""
    res, (ox, oy) = world_map.resolution, world_map.origin
    rows = len(world_map.grid)
    result = []
    for i, (r, c, nr, nc) in enumerate(rectangles(world_map.grid)):
        cx = ox + (c + nc / 2) * res
        cy = oy + (rows - r - nr / 2) * res
        result.append(Box(f"wall_{i}", "wall", round(cx, 3), round(cy, 3), round(nc * res, 3),
                          round(nr * res, 3)))
    return result


def walls_model(boxes):
    """One static model with a box per wall: lighter for Gazebo than a model each."""
    colour = COLOURS["wall"]
    parts = []
    for b in boxes:
        pose = f"<pose>{b.cx} {b.cy} {HEIGHT / 2} 0 0 0</pose>"
        geometry = f"<geometry><box><size>{b.sx} {b.sy} {HEIGHT}</size></box></geometry>"
        parts.append(f"""
        <collision name="{b.name}">{pose}{geometry}</collision>
        <visual name="{b.name}">{pose}{geometry}
          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>""")
    return f"""
    <model name="walls">
      <static>true</static>
      <link name="link">{"".join(parts)}
      </link>
    </model>"""


def walls_file_name(world_file):
    """worlds/house.sdf.xacro -> house_walls.sdf, next to it."""
    return Path(world_file).name.split(".")[0] + "_walls.sdf"


def world_from_map(map_yaml, world_file, root=Path(".")):
    """(world xacro, walls SDF) for the map; paths are relative to root (the repo).

    The walls go in their own SDF file that the world includes: Nav2's launch
    starts xacro (which writes the world to a temporary file) and Gazebo at
    the same time, so with ~1000 walls inline xacro could finish after Gazebo
    tried to open the file. Kept small, the world is as quick as my warehouse.
    xacro turns the include into an absolute path, since its output lands in /tmp.
    """
    boxes = walls(Map(Path(root) / map_yaml))
    source = f"scripts/map_to_world.py from {Path(map_yaml).as_posix()}: edit the map"
    include = f"""
    <include>
      <uri>file://${{xacro.abs_filename('{walls_file_name(world_file)}')}}</uri>
    </include>"""
    walls_sdf = f"""<?xml version="1.0"?>
<!-- Generated by {source}, not here. -->
<sdf version="1.6">{walls_model(boxes)}
</sdf>
"""
    return world_sdf(boxes, include, source), walls_sdf


def main():
    map_yaml, world_file = sys.argv[1], Path(sys.argv[2])
    world, walls_sdf = world_from_map(map_yaml, world_file)
    world_file.write_text(world)
    (world_file.parent / walls_file_name(world_file)).write_text(walls_sdf)
    print(f"Wrote {world_file} and {walls_file_name(world_file)} with {walls_sdf.count('<collision')} boxes")


if __name__ == "__main__":
    main()
