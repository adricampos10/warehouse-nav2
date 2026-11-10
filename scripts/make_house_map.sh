#!/usr/bin/env bash
# Rebuild my home's Nav2 map and Gazebo world from my iPhone scan.
# The scan (house1.db) holds photos of my home, so it is not in the repo.
#
# Usage (from the repo root, with house1.db there):
#     scripts/make_house_map.sh
set -euo pipefail

# --fill: parts of rooms I didn't scan (enclosed unknown areas -> free).
# --clear: scan noise I circled on the map (occupied cells -> free).
docker run --rm -v "$PWD":/ws "${IMAGE:-warehouse-nav2:jazzy}" \
  python3 /ws/scripts/scan_to_map.py /ws/house1.db /ws/maps/house \
    --fill=0.68,5.63 --fill=-0.08,5.88 --fill=1.58,5.58 \
    --clear=-2.015,5.24,0.12 \
    --clear=-1.675,4.805,0.2 \
    --clear=0.88,5.21,0.42 \
    --clear=-0.365,4.32,0.15 \
    --clear=-4.1,3.925,0.35 \
    --clear=-4.15,3.625,0.18 \
    --clear=-5.2,3.38,0.5 \
    --clear=-4.22,3.11,0.47 \
    --clear=1.12,3.02,0.62 \
    --clear=-1.98,2.1,0.25 \
    --clear=-2.055,1.385,0.12 \
    --clear=-1.58,1.5,0.08 \
    --clear=-4.125,4.275,0.06 \
    --clear=-4.01,3.575,0.08

docker run --rm -v "$PWD":/ws -w /ws "${IMAGE:-warehouse-nav2:jazzy}" \
  python3 scripts/map_to_world.py maps/house.yaml worlds/house.sdf.xacro
