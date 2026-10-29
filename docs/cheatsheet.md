# Command cheatsheet

The commands I use in this project, what each part does, and why. I keep it updated as the project grows.

---

## Docker basics

### Make the `docker` CLI available
```bash
export PATH="/Applications/Docker.app/Contents/Resources/bin:$PATH"
```
- `PATH` is the list of folders the shell searches for executables. Docker Desktop ships the `docker` binary inside the app bundle, which is not on macOS's default `PATH`.
- `export` only affects the current terminal session. To make it permanent, add the same line to `~/.zshrc`.

### Build the image
```bash
docker build -t warehouse-nav2:jazzy docker/
```
| Part | What it does |
|---|---|
| `docker build` | Builds an image from a `Dockerfile` |
| `-t warehouse-nav2:jazzy` | Tags the image as `name:tag` so I can refer to it later |
| `docker/` | The build context: the folder containing the `Dockerfile` |

### Start the Nav2 simulation (headless)
```bash
docker run -d --rm --name nav2test -v "$PWD":/ws warehouse-nav2:jazzy \
  bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False"
```
| Part | What it does |
|---|---|
| `docker run` | Creates a container from an image and starts it |
| `-d` (detached) | Runs it in the background and returns the terminal; prints only the container ID |
| `--rm` | Deletes the container automatically when it stops (the image stays) |
| `--name nav2test` | Fixed name, used later with `logs`, `exec` and `stop` instead of the ID |
| `-v "$PWD":/ws` | Bind mount: the current folder (the repo) appears inside the container at `/ws`; changes are visible on both sides |
| `warehouse-nav2:jazzy` | The image to run |
| `bash -c "…"` | Runs the string in a bash shell — needed because `source` and `&&` are shell features |
| `source /opt/ros/jazzy/setup.bash` | Loads the ROS 2 environment (`PATH`, `PYTHONPATH`, `AMENT_PREFIX_PATH`…); without it `ros2` is not found |
| `ros2 launch <pkg> <file>` | Runs a launch file: a Python script that starts several nodes with their parameters |
| `nav2_bringup tb3_simulation_launch.py` | Official Nav2 example: Gazebo + TurtleBot3 + map + the full Nav2 stack (AMCL, planner, controller, BT navigator) |
| `headless:=True` | Launch argument (`name:=value`): Gazebo server only, no window |
| `use_rviz:=False` | Do not open RViz |

The container lives as long as its main process (`ros2 launch`) runs.

### Read the container logs
```bash
sleep 30 && docker logs nav2test 2>&1 | grep -iE "error|fatal|died" | tail -25
```
| Part | What it does |
|---|---|
| `sleep 30` | Waits for Gazebo and Nav2 to finish starting |
| `&&` | Runs the next command only if the previous one exited with code 0 |
| `docker logs nav2test` | Prints the main process's stdout/stderr |
| `2>&1` | Redirects stderr (2) into stdout (1) — ROS logs a lot to stderr, and `grep` only reads stdout |
| `\|` | Pipe: the output of one command becomes the input of the next |
| `grep -iE "a\|b"` | Filters lines; `-i` case-insensitive, `-E` extended regex (`\|` means "or") |
| `tail -25` | Keeps the last 25 lines |

> While Nav2 waits for an initial pose, it repeatedly logs *"frame `map` does not exist"*. That is expected: AMCL only publishes `map → odom` once it knows where the robot is.

### Run a command inside a running container
```bash
docker exec nav2test bash -c "source /opt/ros/jazzy/setup.bash && timeout 8 ros2 topic hz /scan"
```
- `docker exec` starts an extra process inside an already-running container (`run` would create a new one). Both processes share the container's network, so they see the same ROS 2 topics and actions.
- `ros2 topic hz /scan` measures how often a topic is published — a quick way to check that a sensor works.
- `timeout 8` stops it after 8 seconds; otherwise it keeps measuring forever.

### Stop and check
```bash
docker stop nav2test && docker ps
```
- `docker stop` sends SIGTERM to the main process and SIGKILL after 10 s if it is still running. With `--rm`, the container is then removed.
- `docker ps` lists running containers — an empty list means nothing is left running.

---

## Smoke test

```bash
docker exec nav2test bash -c "source /opt/ros/jazzy/setup.bash && python3 /ws/scripts/smoke_test.py"
```
[`scripts/smoke_test.py`](../scripts/smoke_test.py) uses `nav2_simple_commander` (Nav2's Python API):
1. `setInitialPose(...)` publishes on `/initialpose`; AMCL starts from there and publishes the `map → odom` transform.
2. `waitUntilNav2Active()` waits until every Nav2 *lifecycle node* is `active` (`unconfigured → inactive → active`).
3. `goToPose(goal)` sends a goal to the `/navigate_to_pose` action.
4. `getFeedback()` reads the action feedback (e.g. `distance_remaining`).
5. `getResult()` returns `SUCCEEDED`, `FAILED` or `CANCELED`.
6. `sys.exit(0 or 1)` — the exit code is what CI uses to mark the test as passed or failed.

---

## Git basics

```bash
git add scripts/smoke_test.py
git commit -m "Short title" -m "Optional longer description"
git push
git log --oneline -4
git status
```
| Command | What it does |
|---|---|
| `git add <file>` | Stages a change: marks it to be included in the next commit |
| `git commit -m "…"` | Saves a snapshot of the staged changes with a message (each `-m` is a paragraph) |
| `git push` | Uploads new local commits to GitHub (`origin`) |
| `git log --oneline -4` | Shows the last 4 commits, one per line. `(HEAD -> main, origin/main)` means my local branch and GitHub are at the same commit |
| `git status` | Shows what changed and what is staged. *"working tree clean"* means everything is committed |
