"""Unit tests for scripts/mission.py: the patrol behaviour tree, with a fake navigator."""
from pathlib import Path

import pytest
import yaml
from py_trees.common import Status

from mission import Log, build_tree, outcome, run, validate_mission

ROOT = Path(__file__).parent.parent
MISSION = validate_mission({
    "world": "house", "timeout": 100, "retries": 1, "home": [0, 0],
    "places": [{"name": "a", "pose": [1, 0]}, {"name": "b", "pose": [2, 0]}],
})


class FakeNavigator:
    """Reaches every pose after `steps` ticks, except the ones listed in fail (they fail)."""

    def __init__(self, fail=(), steps=2):
        self.fail, self.steps = set(fail), steps
        self.goals, self.cancelled = [], 0

    def go_to(self, x, y):
        self.goals.append((x, y))
        self.left = self.steps

    def done(self):
        self.left -= 1
        return self.left <= 0

    def succeeded(self):
        return self.goals[-1] not in self.fail

    def cancel(self):
        self.cancelled += 1


def fly(navigator, mission=MISSION, max_ticks=1000):
    log = Log()
    tree = build_tree(mission, navigator, log)
    ticks = iter(range(max_ticks))
    status = run(tree, tick=lambda: None, timed_out=lambda: next(ticks) >= max_ticks - 1)
    return status, log


def test_visits_every_place_in_order_then_goes_home():
    nav = FakeNavigator()
    status, log = fly(nav)
    assert status == Status.SUCCESS
    assert nav.goals == [(1, 0), (2, 0), (0, 0)]
    assert log.visited == ["a", "b"] and log.skipped == []


def test_retries_a_place_then_skips_it_and_carries_on():
    nav = FakeNavigator(fail={(1, 0)})
    status, log = fly(nav)
    assert status == Status.SUCCESS  # one blocked place doesn't end the patrol
    assert nav.goals == [(1, 0), (1, 0), (2, 0), (0, 0)]  # 1 try + 1 retry
    assert log.skipped == ["a"] and log.visited == ["b"]
    assert log.attempts["Go to a"] == 2
    assert outcome(status, MISSION, log) == (True, "visited 1/2 places and got home, skipped a")


def test_fails_if_it_cannot_get_home():
    status, log = fly(FakeNavigator(fail={(0, 0)}))
    assert status == Status.FAILURE
    assert outcome(status, MISSION, log) == (False, "couldn't get back home")


def test_timeout_stops_the_mission_and_cancels_the_goal():
    nav = FakeNavigator(steps=10**6)  # never arrives
    status, log = fly(nav, max_ticks=5)
    assert status == Status.INVALID
    assert nav.cancelled == 1
    assert outcome(status, MISSION, log) == (False, "timed out")


@pytest.mark.parametrize("bad, message", [
    ({"world": "house", "timeout": 10, "home": [0, 0]}, "needs 'places'"),
    ({"world": "house", "timeout": 10, "home": [0, 0], "places": []}, "non-empty"),
    ({"world": "house", "timeout": 10, "home": [0, 0], "places": [{"name": "a"}]}, "name and a pose"),
    ({"world": "house", "timeout": 10, "home": [0], "places": [{"name": "a", "pose": [0, 0]}]}, "home"),
    ({"world": "house", "timeout": 0, "home": [0, 0], "places": [{"name": "a", "pose": [0, 0]}]}, "timeout"),
    ({"world": "house", "timeout": 9, "home": [0, 0], "retries": -1,
      "places": [{"name": "a", "pose": [0, 0]}]}, "retries"),
])
def test_invalid_missions_are_rejected(bad, message):
    with pytest.raises(ValueError, match=message):
        validate_mission(bad)


@pytest.mark.parametrize("path", sorted((ROOT / "missions").glob("*.yaml")), ids=lambda p: p.stem)
def test_mission_files_are_valid(path):
    validate_mission(yaml.safe_load(path.read_text()))
