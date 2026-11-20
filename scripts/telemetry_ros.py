"""The ROS side of the telemetry: gathers what telemetry.Sample needs from the topics.

Used by run_scenario.py and run_mission.py. Each source is a subscription
that keeps its latest value; update() turns them into a row every
SAMPLE_PERIOD simulated seconds. Sources:

    TF map->base_footprint  where the robot thinks it is: AMCL's last correction
                        plus the odometry since then, which is what Nav2 uses.
                        /amcl_pose itself is only published when AMCL updates
                        (every 0.25 m or 0.2 rad), so at 0.46 m/s it can be half
                        a second old and would look like a localisation error.
    /amcl_pose          AMCL's covariance (how unsure it is)
    /odom               speed
    /plan               count of global paths published. Nav2's default
                        behaviour tree replans once a second even when nothing
                        changes, so this grows with time: use its rate, not
                        the count, as a sign of trouble.
    Gazebo pose/info    the true pose (only in worlds that publish poses)
    Nav2 feedback       recoveries and distance remaining (passed in by the runner)
    the scan clearance  from run_scenario.Recorder (passed in)
"""
import os
from datetime import datetime, timezone

from geometry_msgs.msg import PoseWithCovarianceStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry, Path
import tf2_ros
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from obstacles import ROBOT_MODEL, WORLD
from telemetry import Sample, TelemetryLog, distance, due, speed, stds_from_covariance


def run_identity():
    """(robot_id, run_id): from the environment, so a fleet runner can set them."""
    robot_id = os.environ.get("ROBOT_ID", "robot-01")
    run_id = os.environ.get("RUN_ID", datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    return robot_id, run_id


class TelemetrySampler:
    def __init__(self, node, directory, scenario):
        self.robot_id, self.run_id = run_identity()
        self.scenario = scenario
        self.log = TelemetryLog(directory, f"{scenario}_{self.robot_id}")
        self.stds = self.true_xy = None
        self.speed = None
        self.plans = 0
        self.last = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        node.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        node.create_subscription(Path, "/plan", self.on_plan, 10)
        self.gz = GzNode()
        self.gz.subscribe(Pose_V, f"/world/{WORLD}/pose/info", self.on_poses)

    def estimated_xy(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time()).transform.translation
        except tf2_ros.TransformException:
            return None  # no localisation yet
        return (t.x, t.y)

    def on_amcl(self, msg):
        self.stds = stds_from_covariance(list(msg.pose.covariance))

    def on_odom(self, msg):
        v = msg.twist.twist.linear
        self.speed = speed(v.x, v.y)

    def on_plan(self, _msg):
        self.plans += 1

    def on_poses(self, msg):  # gz-transport thread
        for p in msg.pose:
            if p.name == ROBOT_MODEL:
                self.true_xy = (p.position.x, p.position.y)

    def update(self, t, leg, state, feedback=None, clearance=None, force=False):
        """Write a row if one is due (or force=True, e.g. when a leg ends)."""
        if not force and not due(t, self.last):
            return
        self.last = t
        estimate = self.estimated_xy()
        x, y = estimate or (None, None)
        gx, gy = self.true_xy or (None, None)
        sx, sy, syaw = self.stds or (None, None, None)
        self.log.sample(Sample(
            t=t, robot_id=self.robot_id, run_id=self.run_id, scenario=self.scenario,
            leg=leg, state=state, x=x, y=y, gt_x=gx, gt_y=gy,
            loc_error=distance(estimate, self.true_xy), std_x=sx, std_y=sy, std_yaw=syaw,
            speed=self.speed, clearance=clearance,
            recoveries=feedback.number_of_recoveries if feedback else None,
            plans=self.plans,
            distance_remaining=float(feedback.distance_remaining) if feedback else None))

    def leg(self, leg, goal, start, end, result, recoveries):
        self.log.leg(robot_id=self.robot_id, run_id=self.run_id, scenario=self.scenario, leg=leg,
                     goal_x=goal[0], goal_y=goal[1], start=start, end=end,
                     duration=end - start, result=result, recoveries=recoveries)

    def close(self):
        self.log.close()
