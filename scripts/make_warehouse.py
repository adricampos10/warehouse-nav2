#!/usr/bin/env python3
"""Generate my warehouse: the Gazebo world and the matching Nav2 map.

The layout below is the single source of truth. Every wall and shelf is a
box, written both as a static model in worlds/warehouse.sdf.xacro and as
occupied cells in maps/warehouse.pgm, so the simulation and the map always
agree. No ROS imports, so the layout is unit-tested with pytest.

Usage (from the repo root):
    python3 scripts/make_warehouse.py
"""
import math
from collections import namedtuple
from pathlib import Path

# Inside of the building, in metres (map frame).
X_MIN, X_MAX = -7.0, 7.0
Y_MIN, Y_MAX = -5.0, 5.0
WALL = 0.15          # wall thickness
HEIGHT = 1.0         # wall and shelf height (the laser is at ~0.17 m)
RESOLUTION = 0.05    # map cell size
MARGIN = 0.5         # unknown space around the building in the map

# Four rows of shelves along x. Centre y of each row, top to bottom: the
# aisles between them are 1.2 m, 1.2 m and 1.0 m wide (the narrow one).
SHELF_X = (-2.0, 4.0)
SHELF_DEPTH = 0.6
SHELF_ROWS = {"A": 3.3, "B": 1.5, "C": -0.3, "D": -1.9}

# Pixel values in the map image, as Nav2's map_server reads them.
FREE, OCCUPIED, UNKNOWN = 254, 0, 205

Box = namedtuple("Box", "name kind cx cy sx sy")  # centre and size in metres


def boxes():
    """Every wall and shelf of the warehouse."""
    width = X_MAX - X_MIN + 2 * WALL
    height = Y_MAX - Y_MIN + 2 * WALL
    cx, cy = (X_MIN + X_MAX) / 2, (Y_MIN + Y_MAX) / 2
    result = [
        Box("wall_north", "wall", cx, Y_MAX + WALL / 2, width, WALL),
        Box("wall_south", "wall", cx, Y_MIN - WALL / 2, width, WALL),
        Box("wall_west", "wall", X_MIN - WALL / 2, cy, WALL, height),
        Box("wall_east", "wall", X_MAX + WALL / 2, cy, WALL, height),
    ]
    shelf_cx = (SHELF_X[0] + SHELF_X[1]) / 2
    shelf_len = SHELF_X[1] - SHELF_X[0]
    for row, y in SHELF_ROWS.items():
        result.append(Box(f"shelf_{row}", "shelf", shelf_cx, y, shelf_len, SHELF_DEPTH))
    return result


def aisle_widths():
    """Free width between consecutive shelf rows, top to bottom."""
    ys = sorted(SHELF_ROWS.values(), reverse=True)
    return [round(a - b - SHELF_DEPTH, 3) for a, b in zip(ys, ys[1:])]


def contains(box, x, y):
    return abs(x - box.cx) <= box.sx / 2 and abs(y - box.cy) <= box.sy / 2


def map_origin():
    """Bottom-left corner of the map, in metres."""
    return (X_MIN - WALL - MARGIN, Y_MIN - WALL - MARGIN)


def rasterize(layout):
    """Occupancy grid as a list of rows, top row first (as in the image).

    A cell is occupied if its centre is inside a box, free if it is inside
    the building, and unknown outside it.
    """
    ox, oy = map_origin()
    cols = math.ceil((X_MAX - X_MIN + 2 * (WALL + MARGIN)) / RESOLUTION)
    rows = math.ceil((Y_MAX - Y_MIN + 2 * (WALL + MARGIN)) / RESOLUTION)
    grid = []
    for r in range(rows):
        y = oy + (rows - 1 - r + 0.5) * RESOLUTION
        line = []
        for c in range(cols):
            x = ox + (c + 0.5) * RESOLUTION
            if any(contains(b, x, y) for b in layout):
                line.append(OCCUPIED)
            elif X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX:
                line.append(FREE)
            else:
                line.append(UNKNOWN)
        grid.append(line)
    return grid


def cell_value(grid, x, y):
    """Value of the map cell that contains the point (x, y)."""
    ox, oy = map_origin()
    c = int((x - ox) / RESOLUTION)
    r = len(grid) - 1 - int((y - oy) / RESOLUTION)
    return grid[r][c]


def pgm(grid):
    """Binary PGM image (P5), the format map_server loads."""
    header = f"P5\n{len(grid[0])} {len(grid)}\n255\n".encode()
    return header + bytes(v for row in grid for v in row)


def map_yaml(image_name):
    ox, oy = map_origin()
    return (
        f"image: {image_name}\n"
        f"mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{ox}, {oy}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )


COLOURS = {"wall": "0.6 0.6 0.6 1", "shelf": "0.85 0.45 0.1 1"}


def sdf_box(box):
    size = f"{box.sx} {box.sy} {HEIGHT}"
    return f"""
    <model name="{box.name}">
      <static>true</static>
      <pose>{box.cx} {box.cy} {HEIGHT / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>{COLOURS[box.kind]}</ambient>
            <diffuse>{COLOURS[box.kind]}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def world_sdf(layout):
    """Gazebo world, a xacro file like the TurtleBot3 sandbox that Nav2 ships.

    tb3_simulation_launch.py runs xacro on it with headless:=True/False, so
    the scene broadcaster (needed only by the Gazebo GUI) is optional.
    """
    models = "".join(sdf_box(b) for b in layout)
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/make_warehouse.py: edit the layout there, not here. -->
<sdf version="1.6" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="headless" default="true"/>

  <world name="default">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <xacro:unless value="$(arg headless)">
      <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    </xacro:unless>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <physics name="3ms" type="ode">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
    <scene>
      <shadows>0</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{models}
  </world>
</sdf>
"""


def main():
    root = Path(__file__).resolve().parent.parent
    layout = boxes()
    (root / "worlds").mkdir(exist_ok=True)
    (root / "maps").mkdir(exist_ok=True)
    (root / "worlds" / "warehouse.sdf.xacro").write_text(world_sdf(layout))
    (root / "maps" / "warehouse.pgm").write_bytes(pgm(rasterize(layout)))
    (root / "maps" / "warehouse.yaml").write_text(map_yaml("warehouse.pgm"))
    print(f"Wrote worlds/warehouse.sdf.xacro and maps/warehouse.{{pgm,yaml}}; "
          f"aisles between shelves: {aisle_widths()} m")


if __name__ == "__main__":
    main()
