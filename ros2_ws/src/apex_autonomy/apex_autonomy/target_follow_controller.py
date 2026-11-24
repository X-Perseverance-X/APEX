import json
import math

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .tracking_math import forward_command, turn_command


class TargetFollowController(Node):
    """Hardware-independent target follower with watchdog and ToF interlock."""

    def __init__(self):
        super().__init__("target_follow_controller")
        defaults = {
            "enabled": False,
            "command_hz": 10.0,
            "observation_timeout_s": 0.45,
            "minimum_confidence": 0.50,
            "target_distance_m": 1.20,
            "tof_stop_distance_m": 0.45,
            "tof_timeout_s": 0.60,
            "max_linear_mps": 0.45,
            "max_angular_rps": 0.85,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.disabled_stop_sent = False
        self.last_observation = None
        self.last_observation_time = None
        self.tof_distance = None
        self.tof_time = None
        self.command_publisher = self.create_publisher(Twist, "/apex/cmd_vel", 10)
        self.status_publisher = self.create_publisher(String, "/apex/tracking/status", 10)
        self.create_subscription(Vector3Stamped, "/apex/target_observation", self.on_observation, 10)
        self.create_subscription(LaserScan, "/apex/tof_scan", self.on_tof, qos_profile_sensor_data)
        self.create_service(SetBool, "/apex/tracking/enable", self.on_enable)
        period = 1.0 / float(self.get_parameter("command_hz").value)
        self.create_timer(period, self.on_control)

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.command_publisher.publish(Twist())
            self.disabled_stop_sent = True
        else:
            self.disabled_stop_sent = False
        response.success = True
        response.message = "tracking enabled" if self.enabled else "tracking disabled; stop sent"
        return response

    def on_observation(self, message):
        self.last_observation = message
        self.last_observation_time = self.get_clock().now()

    def on_tof(self, message):
        finite = [value for value in message.ranges if math.isfinite(value) and value > 0.0]
        self.tof_distance = min(finite) if finite else None
        self.tof_time = self.get_clock().now()

    def age(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def on_control(self):
        command = Twist()
        reason = "disabled"
        visible = False
        observation_age = self.age(self.last_observation_time)
        timeout = float(self.get_parameter("observation_timeout_s").value)

        if not self.enabled:
            # Send exactly one stop on startup / disable, then release cmd_vel
            # so manual driving or another mutually-exclusive controller is
            # not overwritten by a 10 Hz stream of zeros.
            if not self.disabled_stop_sent:
                self.command_publisher.publish(command)
                self.disabled_stop_sent = True
            self.publish_status(reason, visible, observation_age, command)
            return

        if self.last_observation is not None and observation_age <= timeout:
            observation = self.last_observation.vector
            confidence = float(observation.z)
            visible = confidence >= float(self.get_parameter("minimum_confidence").value)
            if visible:
                tof_recent = self.age(self.tof_time) <= float(self.get_parameter("tof_timeout_s").value)
                blocked = (tof_recent and self.tof_distance is not None and
                           self.tof_distance <= float(self.get_parameter("tof_stop_distance_m").value))
                normalized_forward = forward_command(
                    observation.y,
                    observation.x,
                    target_m=float(self.get_parameter("target_distance_m").value),
                )
                normalized_turn = turn_command(observation.x, observation.y)
                if not blocked:
                    command.linear.x = normalized_forward / 0.70 * float(
                        self.get_parameter("max_linear_mps").value
                    )
                # Image error is positive to the right; ROS yaw is positive left.
                command.angular.z = -normalized_turn / 0.85 * float(
                    self.get_parameter("max_angular_rps").value
                )
                reason = "tof_blocked" if blocked else "tracking"
            else:
                reason = "target_not_visible"
        else:
            reason = "observation_timeout"

        self.command_publisher.publish(command)
        self.publish_status(reason, visible, observation_age, command)

    def publish_status(self, reason, visible, observation_age, command):
        status = String()
        status.data = json.dumps({
            "enabled": self.enabled,
            "visible": visible,
            "reason": reason,
            "observation_age_s": None if math.isinf(observation_age) else round(observation_age, 3),
            "tof_m": self.tof_distance,
            "linear_x": round(command.linear.x, 3),
            "angular_z": round(command.angular.z, 3),
        })
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFollowController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
