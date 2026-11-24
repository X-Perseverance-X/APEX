#!/usr/bin/env python3
"""Active SIL probe: distinguish contact walking from drift / falling."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String


class Probe(Node):
    def __init__(self):
        super().__init__("tripod_gait_probe")
        self.odom = None
        self.joints = None
        self.imu = None
        self.gait = None
        self.max_joint_delta = 0.0
        self.reference_joints = None
        self.max_tilt_deg = 0.0
        self.publisher = self.create_publisher(Twist, "/apex/cmd_vel", 10)
        self.create_subscription(Odometry, "/apex/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, qos_profile_sensor_data)
        self.create_subscription(Imu, "/apex/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, "/apex/gait/status", self.on_gait, 10)

    def on_odom(self, message):
        self.odom = message

    def on_joints(self, message):
        self.joints = message
        if self.reference_joints is not None and len(message.position) >= 18:
            delta = max(abs(a - b) for a, b in zip(message.position[:18], self.reference_joints[:18]))
            self.max_joint_delta = max(self.max_joint_delta, delta)

    def on_imu(self, message):
        self.imu = message
        q = message.orientation
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        self.max_tilt_deg = max(self.max_tilt_deg, abs(math.degrees(roll)), abs(math.degrees(pitch)))

    def on_gait(self, message):
        self.gait = json.loads(message.data)

    def spin_for(self, seconds, command=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if command is not None:
                self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)

    def position(self):
        point = self.odom.pose.pose.position
        return point.x, point.y


def distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def main():
    rclpy.init()
    node = Probe()
    try:
        deadline = time.monotonic() + 35.0
        zero = Twist()
        while time.monotonic() < deadline and any(
            value is None for value in (node.odom, node.joints, node.imu, node.gait)
        ):
            node.publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.1)
        if any(value is None for value in (node.odom, node.joints, node.imu, node.gait)):
            missing = [name for name, value in (
                ("odom", node.odom), ("joints", node.joints), ("imu", node.imu), ("gait", node.gait)
            ) if value is None]
            raise RuntimeError(f"Required topics did not become ready: {missing}")

        node.spin_for(3.0, zero)
        node.reference_joints = list(node.joints.position)
        idle_start = node.position()
        node.spin_for(2.0, zero)
        idle_end = node.position()
        idle_drift = distance(idle_start, idle_end)

        forward = Twist()
        forward.linear.x = 0.25
        walk_start = node.position()
        node.spin_for(6.0, forward)
        walk_end = node.position()
        node.spin_for(1.5, zero)
        travel = distance(walk_start, walk_end)
        dx = walk_end[0] - walk_start[0]
        dy = walk_end[1] - walk_start[1]

        assert idle_drift < 0.08, f"idle drift too high: {idle_drift:.3f} m"
        assert node.max_joint_delta > 0.08, f"legs did not articulate: {node.max_joint_delta:.3f} rad"
        assert travel > 0.05, f"contact gait did not translate body: {travel:.3f} m"
        assert dx > 0.02, f"forward command did not move +X: dx={dx:.3f} m"
        assert node.max_tilt_deg < 45.0, f"robot likely tipped: {node.max_tilt_deg:.1f} deg"
        assert node.gait.get("mode") == "physical_contact_tripod"
        print(f"[PASS] idle drift: {idle_drift:.3f} m / 2 s")
        print(f"[PASS] max leg joint motion: {node.max_joint_delta:.3f} rad")
        print(f"[PASS] contact walk: dx={dx:.3f} m dy={dy:.3f} m total={travel:.3f} m / 6 s")
        print(f"[PASS] maximum IMU roll/pitch magnitude: {node.max_tilt_deg:.1f} deg")
        print(f"[PASS] gait status: {node.gait}")
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
