#!/usr/bin/env python3
"""Spawn a scenario's moving obstacles in Gazebo and walk them along their paths.

launch/sim.launch.py starts it when the scenario has obstacles:
    python3 /ws/scripts/move_obstacles.py /ws/scenarios/warehouse_people.yaml

It talks to Gazebo directly through gz-transport, not through ROS: it creates
each obstacle with the world's create service, reads where they are from
/world/<world>/pose/info and sends each one a velocity for its VelocityControl
system. The obstacles wait until the robot starts moving, and stop while the
robot is right in front of them. It also prints the
closest the robot's edge has come to each obstacle, from Gazebo's exact poses.
The decisions (when and where to walk, how fast) are in obstacles.py.
"""
import sys
import time
from pathlib import Path

import yaml
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

from metrics import obstacle_gap, validate_scenario
from obstacles import (ROBOT_MODEL, WORLD, body_velocity, has_moved, model_name, next_target,
                       obstacle_sdf, should_wait, yaw_from_quaternion)


def spawn(node, name, x, y):
    """Create the obstacle, retrying while Gazebo is still starting."""
    request = EntityFactory(sdf=obstacle_sdf(name, x, y))
    while True:
        ok, reply = node.request(f"/world/{WORLD}/create", request, EntityFactory, Boolean, 2000)
        if ok and reply.data:
            print(f"Spawned {name} at ({x}, {y})", flush=True)
            return
        time.sleep(1)


def main():
    scenario = validate_scenario(yaml.safe_load(Path(sys.argv[1]).read_text()))
    obstacles = scenario["obstacles"]
    node = Node()

    index_of = {}
    publishers, targets = [], []
    for i, ob in enumerate(obstacles):
        name = model_name(i)
        spawn(node, name, *ob["path"][0])
        index_of[name] = i
        publishers.append(node.advertise(f"/model/{name}/cmd_vel", Twist))
        targets.append((1, 1))  # walk to the second point first

    robot = {"first": None, "moving": False}
    closest = [None] * len(obstacles)

    def on_poses(msg):
        poses = {p.name: p for p in msg.pose}
        if ROBOT_MODEL not in poses:
            return  # the robot hasn't been spawned yet
        robot_xy = (poses[ROBOT_MODEL].position.x, poses[ROBOT_MODEL].position.y)
        robot["first"] = robot["first"] or robot_xy
        if not robot["moving"] and has_moved(robot["first"], robot_xy):
            robot["moving"] = True
            print("The robot is moving: the obstacles start walking", flush=True)
        for name, i in index_of.items():
            if name not in poses:
                continue
            pose = poses[name]
            path, speed = obstacles[i]["path"], obstacles[i]["speed"]
            position = (pose.position.x, pose.position.y)
            gap = obstacle_gap(robot_xy, position)
            if closest[i] is None or gap < closest[i] - 0.05:
                closest[i] = gap
                print(f"Closest the robot's edge has come to {name}: {gap:.2f} m", flush=True)
            targets[i] = next_target(path, *targets[i], position)
            if not robot["moving"] or should_wait(position, path[targets[i][0]], robot_xy, gap):
                speed = 0.0
            q = pose.orientation
            vx, vy = body_velocity(position, yaw_from_quaternion(q.x, q.y, q.z, q.w),
                                   path[targets[i][0]], speed)
            command = Twist()
            command.linear.x, command.linear.y = vx, vy
            publishers[i].publish(command)

    node.subscribe(Pose_V, f"/world/{WORLD}/pose/info", on_poses)
    while True:  # the callbacks run in gz-transport's thread; stopped with the launch
        time.sleep(1)


if __name__ == "__main__":
    main()
