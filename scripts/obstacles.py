"""Moving obstacles for scenarios: person-sized cylinders walking back and forth.

A scenario can list obstacles, each with a path of (x, y) points and a speed:

    obstacles:
      - path: [[-3.5, -3.0], [-3.5, 3.0]]
        speed: 0.5

Each obstacle stands at its first point until the robot starts moving, then
walks to the last point and back, forever.
This module has no ROS or Gazebo imports, so it is unit-tested with pytest;
scripts/move_obstacles.py does the Gazebo part.
"""
import math

WORLD = "default"              # my warehouse and Nav2's sandbox are both called "default"
ROBOT_MODEL = "turtlebot3_waffle"  # the robot model Nav2's bringup spawns in Gazebo

RADIUS = 0.25        # a person, as the laser sees them from the side
HEIGHT = 1.7
DEFAULT_SPEED = 0.5  # m/s, a slow walk
TOLERANCE = 0.05     # m, how close counts as having reached a path point
STOP_DISTANCE = 0.5  # m between the robot's edge and theirs: closer, and they stop


def validate_obstacles(data):
    """Check the obstacles of a scenario and fill in the default speed."""
    if not isinstance(data, list):
        raise ValueError("obstacles must be a list")
    obstacles = []
    for i, ob in enumerate(data):
        if not isinstance(ob, dict) or "path" not in ob:
            raise ValueError(f"obstacle {i} needs a path")
        path = ob["path"]
        if (not isinstance(path, list) or len(path) < 2
                or not all(isinstance(p, (list, tuple)) and len(p) == 2
                           and all(isinstance(v, (int, float)) for v in p) for p in path)):
            raise ValueError(f"obstacle {i}: path must be a list of at least two [x, y] points")
        speed = ob.get("speed", DEFAULT_SPEED)
        if not isinstance(speed, (int, float)) or speed <= 0:
            raise ValueError(f"obstacle {i}: speed must be a positive number")
        obstacles.append({"path": [(float(x), float(y)) for x, y in path], "speed": float(speed)})
    return obstacles


def model_name(index):
    return f"obstacle_{index}"


def has_moved(first, current, threshold=0.1):
    """Whether the robot has left the spot where it was first seen.

    The obstacles wait until then, so they meet the robot at the same point
    of its drive in every run, however long Nav2 takes to start.
    """
    return math.dist(first, current) > threshold


def should_wait(position, target, robot_xy, gap, stop_distance=STOP_DISTANCE):
    """Whether the obstacle stops to let the robot pass, like a real person would.

    It stops when the robot is closer than stop_distance (edge to edge) and
    in front of it, on the side it is walking towards. If the robot is behind
    or beside it, it keeps walking, so it never blocks a robot that is
    already getting out of its way.
    """
    if gap >= stop_distance:
        return False
    heading = (target[0] - position[0], target[1] - position[1])
    to_robot = (robot_xy[0] - position[0], robot_xy[1] - position[1])
    return heading[0] * to_robot[0] + heading[1] * to_robot[1] > 0


def obstacle_sdf(name, x, y):
    """Gazebo model of one obstacle, standing at (x, y).

    Gravity is off and the cylinder floats 2 cm above the floor, so it never
    rubs on the ground or tips over: the VelocityControl system moves it at
    exactly the speed it is given. It still collides with the robot.
    """
    shape = f"<cylinder><radius>{RADIUS}</radius><length>{HEIGHT}</length></cylinder>"
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <pose>{x} {y} {HEIGHT / 2 + 0.02} 0 0 0</pose>
    <link name="body">
      <gravity>false</gravity>
      <inertial><mass>60</mass><inertia><ixx>15</ixx><iyy>15</iyy><izz>2</izz></inertia></inertial>
      <visual name="visual"><geometry>{shape}</geometry>
        <material><diffuse>0.2 0.4 0.9 1</diffuse></material></visual>
      <collision name="collision"><geometry>{shape}</geometry></collision>
    </link>
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
  </model>
</sdf>
"""


def next_target(path, index, step, position, tolerance=TOLERANCE):
    """Which path point to walk to next: (index, step).

    index is the point the obstacle is walking to and step is +1 or -1, the
    direction along the path. Once the point is reached, it moves on to the
    next one, turning around at either end.
    """
    if math.dist(position, path[index]) > tolerance:
        return index, step
    if not 0 <= index + step < len(path):
        step = -step
    return index + step, step


def yaw_from_quaternion(x, y, z, w):
    """Rotation around the vertical axis, in radians."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def body_velocity(position, yaw, target, speed):
    """Velocity (vx, vy) that walks straight to target, in the obstacle's own frame.

    VelocityControl takes the velocity in the model's frame, so the direction
    in the world is rotated by -yaw (a bump from the robot can turn the
    cylinder).
    """
    dx, dy = target[0] - position[0], target[1] - position[1]
    distance = math.hypot(dx, dy)
    if distance == 0:
        return 0.0, 0.0
    wx, wy = speed * dx / distance, speed * dy / distance
    c, s = math.cos(yaw), math.sin(yaw)
    return c * wx + s * wy, -s * wx + c * wy
