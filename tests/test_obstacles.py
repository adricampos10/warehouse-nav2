"""Unit tests for scripts/obstacles.py. They run in milliseconds, without ROS or Gazebo."""
import math
import xml.etree.ElementTree as ET

import pytest

from metrics import validate_scenario
from obstacles import (DEFAULT_SPEED, HEIGHT, RADIUS, body_velocity, has_moved, next_target,
                       obstacle_sdf, should_wait, validate_obstacles, yaw_from_quaternion)

PATH = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]


# --- validate_obstacles --------------------------------------------------------

def test_scenarios_have_no_obstacles_by_default():
    s = validate_scenario({"start": [0, 0], "goals": [[1, 0]], "timeout": 60})
    assert s["obstacles"] == []


def test_obstacles_need_a_world_that_publishes_poses():
    data = {"start": [0, 0], "goals": [[1, 0]], "timeout": 60,
            "obstacles": [{"path": [[0, 1], [1, 1]]}]}
    with pytest.raises(ValueError, match="moving obstacles need"):
        validate_scenario(data)  # the default world is Nav2's sandbox
    assert validate_scenario({**data, "world": "warehouse"})["obstacles"]


def test_default_speed_is_filled_in():
    obstacles = validate_obstacles([{"path": [[0, 0], [1, 1]]}])
    assert obstacles == [{"path": [(0.0, 0.0), (1.0, 1.0)], "speed": DEFAULT_SPEED}]


@pytest.mark.parametrize("bad, message", [
    ({"path": [[0, 0], [1, 1]]}, "must be a list"),
    ([{"speed": 0.5}], "needs a path"),
    ([{"path": [[0, 0]]}], "at least two"),
    ([{"path": [[0, 0], [1, 1, 1]]}], "at least two"),
    ([{"path": [[0, 0], [1, 1]], "speed": 0}], "positive"),
])
def test_invalid_obstacles_are_rejected(bad, message):
    with pytest.raises(ValueError, match=message):
        validate_obstacles(bad)


# --- has_moved ------------------------------------------------------------------

def test_obstacles_wait_until_the_robot_moves():
    assert not has_moved((-5.5, 0.0), (-5.47, 0.02))  # settling after the spawn
    assert has_moved((-5.5, 0.0), (-5.3, 0.0))


# --- should_wait ------------------------------------------------------------------
# The person is at (0, 0) walking to (5, 0), towards +x.

def test_stops_when_the_robot_is_close_and_in_front():
    assert should_wait((0, 0), (5, 0), (0.8, 0.1), gap=0.33)


def test_keeps_walking_when_the_robot_is_far():
    assert not should_wait((0, 0), (5, 0), (2.0, 0.0), gap=1.53)


def test_keeps_walking_when_the_robot_is_close_but_behind():
    assert not should_wait((0, 0), (5, 0), (-0.8, 0.0), gap=0.33)


def test_keeps_walking_when_the_robot_is_right_beside_it():
    assert not should_wait((0, 0), (5, 0), (0.0, 0.8), gap=0.33)


# --- obstacle_sdf ---------------------------------------------------------------

def test_obstacle_sdf_is_a_floating_cylinder_at_the_start():
    model = ET.fromstring(obstacle_sdf("obstacle_0", 1.5, -2.0)).find("model")
    assert model.get("name") == "obstacle_0"
    x, y, z = map(float, model.find("pose").text.split()[:3])
    assert (x, y) == (1.5, -2.0)
    assert z > HEIGHT / 2  # off the floor, so it can't rub or tip over
    assert model.find("link/gravity").text == "false"
    assert float(model.find("link/collision/geometry/cylinder/radius").text) == RADIUS
    assert model.find("plugin").get("name") == "gz::sim::systems::VelocityControl"


# --- next_target ----------------------------------------------------------------

def test_keeps_walking_to_the_target_until_it_is_reached():
    assert next_target(PATH, 1, 1, (1.0, 0.0)) == (1, 1)


def test_moves_on_to_the_next_point():
    assert next_target(PATH, 1, 1, (1.98, 0.0)) == (2, 1)


def test_turns_around_at_the_last_point():
    assert next_target(PATH, 2, 1, (2.0, 2.0)) == (1, -1)


def test_turns_around_at_the_first_point():
    assert next_target(PATH, 0, -1, (0.0, 0.01)) == (1, 1)


# --- yaw_from_quaternion and body_velocity ---------------------------------------

def test_yaw_from_quaternion():
    assert yaw_from_quaternion(0, 0, 0, 1) == pytest.approx(0)
    half = math.pi / 4  # 90 degrees around z
    assert yaw_from_quaternion(0, 0, math.sin(half), math.cos(half)) == pytest.approx(math.pi / 2)


def test_walks_straight_to_the_target_at_its_speed():
    vx, vy = body_velocity((0, 0), 0.0, (3, 4), 0.5)
    assert (vx, vy) == (pytest.approx(0.3), pytest.approx(0.4))


def test_velocity_is_in_the_obstacle_frame_when_it_has_turned():
    # Turned 90 degrees left, walking +x in the world is walking to its right (-y).
    vx, vy = body_velocity((0, 0), math.pi / 2, (1, 0), 0.5)
    assert (vx, vy) == (pytest.approx(0), pytest.approx(-0.5))


def test_stands_still_on_the_target():
    assert body_velocity((1, 1), 0.0, (1, 1), 0.5) == (0.0, 0.0)
