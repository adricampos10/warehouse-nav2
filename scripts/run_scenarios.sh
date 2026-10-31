#!/usr/bin/env bash
# Runs navigation scenarios end to end. Each scenario gets a fresh headless
# simulation in its own container, so they all start from the same state.
# Results go to results/<scenario>.json and a summary table is printed.
# Exit code 0 only if every scenario passed.
#
# Usage (from the repo root):
#     scripts/run_scenarios.sh                  # every file in scenarios/
#     scripts/run_scenarios.sh short_goal       # just the ones named
set -euo pipefail

IMAGE="${IMAGE:-warehouse-nav2:jazzy}"
NAME="nav2scenario"

cleanup() {
  docker stop "$NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if [ $# -gt 0 ]; then
  scenarios=("$@")
else
  scenarios=()
  for f in scenarios/*.yaml; do scenarios+=("$(basename "$f" .yaml)"); done
fi

mkdir -p results
failed=0
for scenario in "${scenarios[@]}"; do
  rm -f "results/$scenario.json"
  echo "=== $scenario: starting the Nav2 simulation (headless)..."
  docker run -d --rm --name "$NAME" -v "$PWD":/ws "$IMAGE" \
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False" \
    >/dev/null

  if ! docker exec "$NAME" bash -c "source /opt/ros/jazzy/setup.bash && cd /ws/scripts && timeout 600 python3 run_scenario.py /ws/scenarios/$scenario.yaml /ws/results"; then
    failed=1
    if [ ! -f "results/$scenario.json" ]; then
      echo "=== $scenario: crashed before saving a result. Last simulation logs:"
      docker logs "$NAME" 2>&1 | tail -60
    fi
  fi
  cleanup
  # Wait until the container is gone so the next one can reuse the name.
  while docker inspect "$NAME" >/dev/null 2>&1; do sleep 1; done
done

echo
python3 scripts/report.py results/*.json
exit "$failed"
