#!/usr/bin/env bash
# Runs a behaviour-tree patrol mission end to end: a fresh headless
# simulation of the mission's world in its own container, then
# run_mission.py ticks the tree against Nav2. The result goes to
# results/missions/mission_<mission>.json (telemetry in results/telemetry/). Exit code 0 only if the mission passed.
#
# Usage (from the repo root):
#     scripts/run_mission.sh house_patrol      # missions/house_patrol.yaml
set -euo pipefail

IMAGE="${IMAGE:-warehouse-nav2:jazzy}"
NAME="nav2mission"
MISSION="${1:?usage: scripts/run_mission.sh <mission name in missions/>}"
WORLD=$(sed -n 's/^world: *//p' "missions/$MISSION.yaml")

cleanup() {
  docker stop "$NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

mkdir -p results
rm -f "results/missions/mission_$MISSION.json"
echo "=== $MISSION: starting the Nav2 simulation of '$WORLD' (headless)..."
docker run -d --rm --name "$NAME" -v "$PWD":/ws "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch /ws/launch/sim.launch.py world_name:=$WORLD" \
  >/dev/null

docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash && cd /ws/scripts && timeout 900 python3 run_mission.py /ws/missions/$MISSION.yaml /ws/results"
