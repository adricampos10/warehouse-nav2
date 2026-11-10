#!/usr/bin/env python3
"""Turn a room scan from my iPhone into a Nav2 map (PGM + YAML).

I scan with the RTAB-Map iOS app (LiDAR + ARKit) and save the database with
"Optimize", so it also stores the corrected pose of every frame. This script
reads that database (SQLite) and builds a 2D occupancy grid from the LiDAR
depth images, so it needs no RTAB-Map install. Run it in the container
(it needs numpy and OpenCV):

    docker run --rm -v "$PWD":/ws warehouse-nav2:jazzy \\
        python3 /ws/scripts/scan_to_map.py /ws/house1.db /ws/maps/house \\
        --fill=0.68,5.63 --fill=-0.08,5.88 --fill=1.58,5.58

(that is how I made maps/house; write --fill=X,Y with "=", or argparse takes
a negative X for an option)

Steps:
  1. Every depth image becomes 3D points in the scan's frame (camera pose x
     the camera's mounting x pinhole back-projection).
  2. The floor is the most common height below the camera. Points 0.10-1.5 m
     above it are obstacles; points on it are seen floor.
  3. The map is turned so the walls line up with the x and y axes, and
     (0, 0) is where I started the scan.
  4. A cell is occupied if it has enough obstacle points and the camera
     didn't see through it; it is free if the floor was seen there or the
     camera saw through it. Only free space connected to my path is kept, so
     what was seen through windows or mirrors is dropped.
  5. --fill x,y turns the unknown pocket around (x, y) into free space, for a
     part of a room I didn't scan.
  6. Walls lose one layer of cells on the free side (the scan makes them
     fatter than they are), but never get thinner than two cells.

The database holds photos of the place, so it stays out of git; only the map
(the occupancy grid, no images) goes in the repo.
"""
import argparse
import sqlite3
import struct
import zlib

import cv2
import numpy as np

from make_warehouse import FREE, OCCUPIED, UNKNOWN

RESOLUTION = 0.05            # map cell size, m
MAX_DEPTH = 3.5              # m; the iPhone's LiDAR smears further away
MIN_DEPTH = 0.2
OBSTACLE_BAND = (0.10, 1.5)  # m above the floor
FLOOR_TOLERANCE = 0.06       # m
MIN_HITS = 15                # obstacle points for a cell to count as occupied
MARGIN = 0.5                 # unknown space around the map, m


def pose_matrix(values):
    """4x4 transform from RTAB-Map's 12 numbers (a 3x4 matrix, row by row)."""
    m = np.eye(4)
    m[:3, :4] = np.asarray(values, np.float64).reshape(3, 4)
    return m


def compressed_matrix(blob, dtype):
    """RTAB-Map stores matrices zlib-compressed, followed by rows, cols and type."""
    return np.frombuffer(zlib.decompress(blob[:-12]), dtype)


def read_frames(path):
    """(camera position, Nx3 points) for every frame, in the scan's frame."""
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    optimized = {}
    ids, poses = db.execute("select opt_ids, opt_poses from Admin").fetchone()
    if ids and poses:
        ids = compressed_matrix(ids, np.int32)
        poses = compressed_matrix(poses, np.float32).reshape(-1, 12)
        optimized = {int(i): pose_matrix(p) for i, p in zip(ids, poses)}

    frames = []
    for node, pose, depth, calibration in db.execute(
            "select n.id, n.pose, d.depth, d.calibration from Node n join Data d on n.id = d.id "
            "where length(d.depth) > 0 order by n.id"):
        if optimized and node not in optimized:
            continue  # not part of the optimized map
        camera_pose = optimized.get(node, pose_matrix(struct.unpack("12f", pose)))
        width, _ = struct.unpack_from("2i", calibration, 16)
        k = struct.unpack_from("9d", calibration, 44)       # pinhole matrix of the colour image
        mounting = pose_matrix(struct.unpack("12f", calibration[-48:]))
        image = cv2.imdecode(np.frombuffer(depth, np.uint8), cv2.IMREAD_UNCHANGED)
        if image.ndim == 3:  # float32 depth saved as the bytes of an RGBA PNG
            image = np.ascontiguousarray(image).view(np.float32).reshape(image.shape[:2])
        scale = image.shape[1] / width                       # depth is smaller than the colour image
        fx, cx, fy, cy = k[0] * scale, k[2] * scale, k[4] * scale, k[5] * scale
        v, u = np.mgrid[0:image.shape[0], 0:image.shape[1]]
        ok = (image > MIN_DEPTH) & (image < MAX_DEPTH)
        z = image[ok]
        points = np.stack([(u[ok] - cx) * z / fx, (v[ok] - cy) * z / fy, z, np.ones_like(z)])
        to_scan = camera_pose @ mounting
        frames.append((to_scan[:3, 3], (to_scan @ points)[:3].T))
    return frames


def floor_height(heights, camera_height):
    """The most common height at least 0.5 m below the camera."""
    hist, edges = np.histogram(heights, bins=np.arange(heights.min(), heights.max() + 0.02, 0.02))
    below = edges[:-1] < camera_height - 0.5
    return edges[np.argmax(np.where(below, hist, 0))] + 0.01


def wall_yaw(points_xy):
    """The rotation that lines the walls up with the axes: the one that piles
    the obstacle points into the fewest rows and columns of cells."""
    def sharpness(yaw):
        xy = points_xy @ rotation(yaw)
        return sum((np.histogram(xy[:, k], bins=np.arange(xy[:, k].min(), xy[:, k].max() + RESOLUTION,
                                                          RESOLUTION))[0] ** 2).sum() for k in (0, 1))
    yaws = np.radians(np.arange(-45, 45, 0.25))
    return yaws[np.argmax([sharpness(a) for a in yaws])]


def rotation(yaw):
    """Matrix that turns row vectors by -yaw."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s], [-s, c]])


def build_map(frames, fill=()):
    """Occupancy grid (rows from the top, like the image) and its origin in metres."""
    floor = floor_height(np.concatenate([p[:, 2] for _, p in frames]),
                         np.median([c[2] for c, _ in frames]))
    is_obstacle = [(p[:, 2] > floor + OBSTACLE_BAND[0]) & (p[:, 2] < floor + OBSTACLE_BAND[1])
                   for _, p in frames]
    is_floor = [np.abs(p[:, 2] - floor) < FLOOR_TOLERANCE for _, p in frames]
    obstacles = np.concatenate([p[o][:, :2] for (_, p), o in zip(frames, is_obstacle)])
    turn = rotation(wall_yaw(obstacles[::20]))
    start = frames[0][0][:2]

    def to_map(xy):
        return (xy[:, :2] - start) @ turn

    every = np.concatenate([to_map(p) for _, p in frames])
    origin = np.floor(every.min(0) / RESOLUTION) * RESOLUTION - MARGIN
    width, height = ((every.max(0) + MARGIN - origin) / RESOLUTION).astype(int) + 1

    def cells(xy):
        ij = ((xy - origin) / RESOLUTION).astype(int)
        return ij[:, 0], ij[:, 1]

    hits = np.zeros((width, height), np.int32)
    floor_hits = np.zeros((width, height), np.int32)
    seen_through = np.zeros((width, height), np.int32)
    for (camera, points), obstacle, on_floor in zip(frames, is_obstacle, is_floor):
        xy, eye = to_map(points), to_map(camera[None])[0]
        np.add.at(hits, cells(xy[obstacle]), 1)
        np.add.at(floor_hits, cells(xy[on_floor]), 1)
        # Walk from the camera towards each obstacle point: the cells on the
        # way were seen empty (one vote per frame).
        ends = xy[obstacle][::3]
        distance = np.linalg.norm(ends - eye, axis=1)
        seen = np.zeros((width, height), bool)
        for t in np.arange(0, 1, 0.5 * RESOLUTION / max(distance.max(), RESOLUTION)):
            before_the_end = distance * (1 - t) > 2 * RESOLUTION
            seen[cells((eye + (ends - eye) * t)[before_the_end])] = True
        seen_through += seen

    occupied = (hits >= MIN_HITS) & (hits / np.maximum(hits + 30 * seen_through, 1) > 0.3)
    occupied = drop_specks(occupied, min_cells=4)
    free = ((floor_hits >= 2) | (seen_through >= 2)) & ~occupied
    free = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)).astype(bool)
    free &= ~occupied
    path = cells(to_map(np.array([c for c, _ in frames])))
    free = keep_components(free, path)
    for x, y in fill:
        free |= unknown_pocket(free, occupied, cells(np.array([[x, y]])))
    peeled = thin_walls(occupied, free)
    occupied &= ~peeled
    free |= peeled

    grid = np.full((width, height), UNKNOWN, np.uint8)
    grid[free] = FREE
    grid[occupied] = OCCUPIED
    return np.flipud(grid.T), origin


def thin_walls(occupied, free):
    """Occupied cells to give back to free space: one layer on the free side.

    The scan makes walls and furniture a few centimetres fatter than they
    are (LiDAR noise, small pose errors), which turns my doorways into slots
    the robot barely fits through. A cell is peeled if free space is on one
    side and at least two more occupied cells are behind it, so a wall is
    never left thinner than two cells (10 cm) and no hole opens in it.
    """
    pad = np.pad(occupied, 3), np.pad(free, 3)
    peeled = np.zeros_like(pad[0])
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        def shifted(mask, k):  # mask at (i + k*dx, j + k*dy)
            return np.roll(mask, (-k * dx, -k * dy), axis=(0, 1))
        peeled |= (pad[0] & shifted(pad[1], -1) & shifted(pad[0], 1) & shifted(pad[0], 2))
    return peeled[3:-3, 3:-3]


def drop_specks(mask, min_cells):
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    return np.isin(labels, [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= min_cells])


def keep_components(mask, cells):
    """The parts of mask (4-connected) that contain one of the cells."""
    _, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=4)
    return np.isin(labels, [i for i in np.unique(labels[cells]) if i])


def unknown_pocket(free, occupied, cell):
    """The unknown area around cell, if it is enclosed (doesn't reach the map's edge)."""
    _, labels = cv2.connectedComponents((~free & ~occupied).astype(np.uint8), connectivity=4)
    label = labels[cell][0]
    edge = np.concatenate([labels[0], labels[-1], labels[:, 0], labels[:, -1]])
    if label == 0 or label in edge:
        raise ValueError(f"--fill {cell}: not inside an enclosed unknown area")
    return labels == label


def map_yaml(image_name, origin, resolution=RESOLUTION):
    """Nav2 map_server metadata for the image."""
    return (f"image: {image_name}\nmode: trinary\nresolution: {resolution}\n"
            f"origin: [{origin[0]:.2f}, {origin[1]:.2f}, 0.0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.196\n")


def parse_point(text):
    x, y = (float(v) for v in text.split(","))
    return x, y


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("database", help="RTAB-Map database saved by the iOS app")
    parser.add_argument("output", help="map path without extension, e.g. maps/house")
    parser.add_argument("--fill", type=parse_point, action="append", default=[], metavar="X,Y",
                        help="turn the enclosed unknown area around this point (m) into free space")
    args = parser.parse_args()

    grid, origin = build_map(read_frames(args.database), args.fill)
    name = args.output.rsplit("/", 1)[-1]
    cv2.imwrite(f"{args.output}.pgm", grid)
    with open(f"{args.output}.yaml", "w") as f:
        f.write(map_yaml(f"{name}.pgm", origin))
    print(f"Wrote {args.output}.pgm ({grid.shape[1]} x {grid.shape[0]} cells) and {args.output}.yaml")


if __name__ == "__main__":
    main()
