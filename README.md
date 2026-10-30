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

## Run the smoke test
```bash
scripts/run_smoke_test.sh
```
It starts the headless Nav2 simulation in a container, drives the TurtleBot3 to a goal and exits with 0 if it gets there. GitHub Actions runs the same script on every push ([workflow](.github/workflows/ci.yml)).

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
