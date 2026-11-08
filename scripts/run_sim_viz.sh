#!/usr/bin/env bash
# Starts the headless Nav2 simulation together with foxglove_bridge, so I can
# watch the robot from my Mac in Foxglove / Lichtblick (ws://localhost:8765).
# It sets the initial pose automatically; then goals can be sent from the
# viewer on /goal_pose. Stop it with Ctrl-C.
#
# Usage (from the repo root):
#     scripts/run_sim_viz.sh                     # Nav2's TurtleBot3 sandbox
#     scripts/run_sim_viz.sh warehouse           # my warehouse
#     scripts/run_sim_viz.sh warehouse_people    # a scenario: its world, start
#                                                # and moving obstacles
# A scenario's people stand still until the robot starts moving, so they
# start walking when I send the first goal.
set -euo pipefail

IMAGE="${IMAGE:-warehouse-nav2:jazzy}"
NAME="nav2viz"
PORT="${PORT:-8765}"
TARGET="${1:-tb3_sandbox}"  # a world name, or a scenario in scenarios/

if [ -f "scenarios/$TARGET.yaml" ]; then
  LAUNCH_ARG="scenario:=/ws/scenarios/$TARGET.yaml"
  POSE_ARG="/ws/scenarios/$TARGET.yaml"
else
  LAUNCH_ARG="world_name:=$TARGET"
  POSE_ARG="$TARGET"
fi

cleanup() {
  docker stop "$NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "Starting the Nav2 simulation ($TARGET) + foxglove_bridge..."
docker run -d --rm --name "$NAME" -p "${PORT}:8765" -v "$PWD":/ws "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch /ws/launch/sim.launch.py $LAUNCH_ARG viz:=true"

echo "Setting the initial pose and waiting for Nav2..."
docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash && python3 /ws/scripts/set_initial_pose.py $POSE_ARG"

echo "Ready. Connect Foxglove / Lichtblick to ws://localhost:${PORT} (Ctrl-C to stop)."
docker logs -f "$NAME" > /dev/null 2>&1 || true
