"""A patrol mission as a behaviour tree (py_trees): visit places, then go home.

A mission file lists the places to visit, in order:

    description: Patrol my home and come back to the entrance
    world: house
    retries: 1            # extra attempts per place before skipping it
    timeout: 400          # simulated seconds for the whole mission
    places:
      - name: hallway
        pose: [-3.5, 0.8]
      - name: living room
        pose: [1.1, 6.0]
    home: [0.0, 0.0]

The tree:

    Mission (sequence)
    ├── Patrol (sequence)
    │   ├── Visit hallway (selector)
    │   │   ├── Retry: go to hallway
    │   │   └── Skip hallway
    │   └── Visit living room ...
    └── Return home (retry: go home)

A place that can't be reached after its retries is skipped, so one blocked
room doesn't end the patrol; the mission only fails if the robot can't get
home. The navigation is behind a small interface (go_to, done, succeeded,
cancel), so the tree is unit-tested with a fake navigator, without ROS;
scripts/run_mission.py plugs in Nav2.
"""
import py_trees
from py_trees.common import Status

DEFAULT_RETRIES = 1


def validate_mission(data):
    """Check a mission file and fill in the defaults."""
    if not isinstance(data, dict):
        raise ValueError("a mission must be a mapping")
    for key in ("world", "places", "home", "timeout"):
        if key not in data:
            raise ValueError(f"a mission needs '{key}'")
    places = data["places"]
    if not isinstance(places, list) or not places:
        raise ValueError("places must be a non-empty list")
    checked = []
    for i, place in enumerate(places):
        if not isinstance(place, dict) or "name" not in place or "pose" not in place:
            raise ValueError(f"place {i} needs a name and a pose")
        checked.append({"name": str(place["name"]), "pose": point(place["pose"], f"place {i}")})
    retries = data.get("retries", DEFAULT_RETRIES)
    if not isinstance(retries, int) or retries < 0:
        raise ValueError("retries must be a whole number, 0 or more")
    if not isinstance(data["timeout"], (int, float)) or data["timeout"] <= 0:
        raise ValueError("timeout must be a positive number of seconds")
    return {
        "description": data.get("description", ""),
        "world": data["world"],
        "places": checked,
        "home": point(data["home"], "home"),
        "retries": retries,
        "timeout": float(data["timeout"]),
    }


def point(value, what):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(isinstance(v, (int, float)) for v in value)):
        raise ValueError(f"{what}: pose must be [x, y]")
    return float(value[0]), float(value[1])


class Log:
    """What happened during the mission, for the results file."""

    def __init__(self):
        self.visited, self.skipped, self.attempts = [], [], {}


class GoTo(py_trees.behaviour.Behaviour):
    """Drive to a pose: RUNNING while Nav2 works on it, then SUCCESS or FAILURE."""

    def __init__(self, name, pose, navigator, log):
        super().__init__(name)
        self.pose, self.navigator, self.log = pose, navigator, log

    def initialise(self):
        self.log.attempts[self.name] = self.log.attempts.get(self.name, 0) + 1
        self.navigator.go_to(*self.pose)

    def update(self):
        if not self.navigator.done():
            return Status.RUNNING
        return Status.SUCCESS if self.navigator.succeeded() else Status.FAILURE

    def terminate(self, new_status):
        if new_status == Status.INVALID:  # interrupted (e.g. the mission timed out)
            self.navigator.cancel()


class Record(py_trees.behaviour.Behaviour):
    """Write a place into a list of the log, and succeed."""

    def __init__(self, name, place, entries):
        super().__init__(name)
        self.place, self.entries = place, entries

    def update(self):
        self.entries.append(self.place)
        return Status.SUCCESS


def visit(place, retries, navigator, log):
    """Go to the place (with retries) and note it, or note it as skipped."""
    name = place["name"]
    reach = py_trees.composites.Sequence(f"Reach {name}", memory=True, children=[
        py_trees.decorators.Retry(f"Retry {name}", GoTo(f"Go to {name}", place["pose"], navigator, log),
                                  num_failures=retries + 1),
        Record(f"Visited {name}", name, log.visited),
    ])
    return py_trees.composites.Selector(f"Visit {name}", memory=True, children=[
        reach, Record(f"Skip {name}", name, log.skipped)])


def build_tree(mission, navigator, log):
    """The mission's behaviour tree (see the module docstring)."""
    patrol = py_trees.composites.Sequence("Patrol", memory=True, children=[
        visit(place, mission["retries"], navigator, log) for place in mission["places"]])
    go_home = py_trees.decorators.Retry(
        "Return home", GoTo("Go home", mission["home"], navigator, log),
        num_failures=mission["retries"] + 1)
    return py_trees.composites.Sequence("Mission", memory=True, children=[patrol, go_home])


def run(tree, tick, timed_out):
    """Tick the tree until it finishes or timed_out() says to stop.

    tick() is called between ticks (the ROS runner spins Nav2 there).
    Returns the tree's final status; a timeout halts the tree, which cancels
    the goal Nav2 is working on.
    """
    while True:
        tree.tick_once()
        if tree.status != Status.RUNNING:
            return tree.status
        if timed_out():
            tree.stop(Status.INVALID)
            return Status.INVALID
        tick()


def outcome(status, mission, log):
    """(passed, reason): the mission passes if the robot got back home."""
    if status == Status.SUCCESS:
        skipped = f", skipped {', '.join(log.skipped)}" if log.skipped else ""
        return True, f"visited {len(log.visited)}/{len(mission['places'])} places and got home{skipped}"
    if status == Status.INVALID:
        return False, "timed out"
    return False, "couldn't get back home"
