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
# Exit code 3 from run_mission.py means Nav2 never became active (an
# infrastructure problem): retry once with a fresh simulation.
for attempt in 1 2; do
  rm -f "results/missions/mission_$MISSION.json"
  echo "=== $MISSION: starting the Nav2 simulation of '$WORLD' (headless, attempt $attempt)..."
  docker run -d --rm --name "$NAME" -v "$PWD":/ws "$IMAGE" \
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch /ws/launch/sim.launch.py world_name:=$WORLD" \
    >/dev/null
  status=0
  docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash && cd /ws/scripts && timeout 900 python3 run_mission.py /ws/missions/$MISSION.yaml /ws/results" || status=$?
  cleanup
  while docker inspect "$NAME" >/dev/null 2>&1; do sleep 1; done
  if [ "$status" -eq 3 ] && [ "$attempt" -eq 1 ]; then
    echo "=== $MISSION: Nav2 didn't start; retrying with a fresh simulation"
    continue
  fi
  exit "$status"
done
