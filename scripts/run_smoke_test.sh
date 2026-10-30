#!/usr/bin/env bash
# Runs the Nav2 smoke test end to end: starts the headless simulation in a
# container, drives the robot to a goal and stops everything afterwards.
# The exit code (0 = passed) is what GitHub Actions uses to mark the run.
#
# Usage (from the repo root):
#     scripts/run_smoke_test.sh
set -euo pipefail

IMAGE="${IMAGE:-warehouse-nav2:jazzy}"
NAME="nav2test"

cleanup() {
  docker stop "$NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "Starting the Nav2 simulation (headless)..."
docker run -d --rm --name "$NAME" -v "$PWD":/ws "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False"

echo "Running the smoke test..."
if docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash && timeout 300 python3 /ws/scripts/smoke_test.py"; then
  echo "Smoke test PASSED"
else
  status=$?
  echo "Smoke test FAILED (exit code $status). Last simulation logs:"
  docker logs "$NAME" 2>&1 | tail -60
  exit "$status"
fi
