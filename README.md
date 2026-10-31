# warehouse-nav2

[![CI](https://github.com/adricampos10/warehouse-nav2/actions/workflows/ci.yml/badge.svg)](https://github.com/adricampos10/warehouse-nav2/actions/workflows/ci.yml)

I'm building an autonomous warehouse robot with **ROS 2 Jazzy + Nav2**, simulated in **Gazebo Harmonic** (and later NVIDIA Isaac Sim), with a **behavior-tree mission** and an **automated scenario test suite** that runs headless in Docker and on GitHub Actions.

> Work in progress.

## What I'm building
- Differential-drive robot navigating a warehouse with Nav2.
- Mission logic as a behavior tree: go to a shelf, wait, return, recover if blocked.
- Scenario suite (moving obstacles, narrow aisles, people crossing) run headless in Docker, reporting success rate, time to goal and minimum obstacle distance.
- CI with GitHub Actions running the scenarios on every change.

## Docs
- [Command cheatsheet](docs/cheatsheet.md) — the Docker, ROS 2 and Git commands I use, explained.
- [Diagrams](docs/diagrams/README.md) — how the project is put together.

## Run the environment
```bash
docker build -t warehouse-nav2:jazzy docker/
docker run --rm -it warehouse-nav2:jazzy bash
```

## Run the scenario suite
Each file in [`scenarios/`](scenarios) describes a navigation test: start pose, one or more goals, a time limit, and whether Nav2 should succeed or fail:
```yaml
description: Patrol loop through four waypoints
start: [-2.0, -0.5]
goals:
  - [-1.5, 1.6]
  - [1.8, 0.3]
  - [0.5, -0.5]
  - [-2.0, -0.5]
timeout: 300
```
```bash
scripts/run_scenarios.sh              # every scenario
scripts/run_scenarios.sh short_goal   # just one
```
Every scenario gets a fresh headless simulation in its own container. [`run_scenario.py`](scripts/run_scenario.py) drives the robot through the goals while recording `/odom` and `/scan`, then saves `results/<scenario>.json`. A scenario passes if the robot reaches every goal in time without getting closer than 0.15 m to an obstacle (or, for `expect: failure`, if Nav2 gives up). The script ends with a summary table; this is a run on my Mac:

| Scenario | Result | Goals | Time (s) | Path (m) | Min clearance (m) | Notes |
|---|---|---|---|---|---|---|
| cross_map | PASS | 1/1 | 25.6 | 4.61 | 0.36 | all goals reached |
| patrol_loop | PASS | 4/4 | 50.7 | 11.52 | 0.32 | all goals reached |
| short_goal | PASS | 1/1 | 6.1 | 2.31 | 0.36 | all goals reached |
| unreachable_goal | PASS | 0/1 | 0.0 | 0.00 | 0.48 | failed as expected |

The pass/fail logic and the metrics live in [`metrics.py`](scripts/metrics.py), with no ROS imports, so they are unit-tested with pytest in milliseconds:
```bash
docker run --rm -v "$PWD":/ws -w /ws warehouse-nav2:jazzy pytest -v
```

### In VS Code
`metrics.py` has no ROS dependencies, so I also run and debug its tests on my Mac, in a virtual environment:
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt
```
The settings in [`.vscode/`](.vscode) point VS Code at `.venv` and pytest, so the tests show up in the Testing panel, where I can run or debug each one with breakpoints.

### On GitHub Actions
On every push, the [workflow](.github/workflows/ci.yml) runs the unit tests first, then one parallel job per scenario file (a build matrix, so adding a YAML file adds a job), and finally writes the results table to the run's summary page.

## Run the smoke test
```bash
scripts/run_smoke_test.sh
```
A quicker single check: it starts the headless Nav2 simulation in a container, drives the TurtleBot3 to a goal and exits with 0 if it gets there.

## Watch the robot
Docker on my Mac cannot open Gazebo's window, so I watch the simulation in [Lichtblick](https://github.com/lichtblick-suite/lichtblick) (the open-source Foxglove) through `foxglove_bridge`:
```bash
scripts/run_sim_viz.sh   # terminal 1: simulation + foxglove_bridge on ws://localhost:8765
scripts/run_viewer.sh    # terminal 2: Lichtblick on http://localhost:8080
```
Then open <http://localhost:8080/?ds=foxglove-websocket&ds.url=ws://localhost:8765>. The layout in [`viz/lichtblick_layout.json`](viz/lichtblick_layout.json) shows the map, global costmap, laser scan, planned path and robot model. `run_sim_viz.sh` sets the initial pose automatically, so goals can be sent straight away with the 3D panel's *Publish pose* tool (it publishes on `/goal_pose`). To watch it drive around on its own:
```bash
docker exec nav2viz bash -c "source /opt/ros/jazzy/setup.bash && python3 /ws/scripts/patrol.py 5"
```
