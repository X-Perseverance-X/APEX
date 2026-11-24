import json
import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64MultiArray, String

from .gait_math import approach, clamp, tripod_targets


LEG_ORDER = ("RF", "RM", "RR", "LF", "LM", "LR")


class TripodGaitController(Node):
    """Translate body Twist commands into the v55 18-joint tripod gait."""

    def __init__(self):
        super().__init__("tripod_gait_controller")
        defaults = {
            "update_rate_hz": 50.0,
            "command_timeout_s": 0.40,
            "command_linear_scale_mps": 0.45,
            "command_angular_scale_rps": 0.85,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        model_path = Path(get_package_share_directory("apex_description")) / "config" / "model_parameters.yaml"
        with model_path.open("r", encoding="utf-8") as stream:
            model = yaml.safe_load(stream)
        legs = model["legs"]
        gait = legs["gait"]
        self.mount_yaws = [math.radians(float(legs["mount_yaw_deg"][name])) for name in LEG_ORDER]
        self.phase_offsets = [float(gait["phase"][name]) for name in LEG_ORDER]
        self.radial_m = float(legs["stance"]["radial_m"]["value"])
        self.stance_z_m = float(legs["stance"]["z_m"]["value"])
        self.step_length_m = float(gait["step_length_m"]["value"])
        self.step_lift_m = float(gait["step_lift_m"]["value"])
        self.cycle_s = float(gait["cycle_s_at_full_command"]["value"])

        self.phase = 0.0
        self.smoothed_forward = 0.0
        self.smoothed_lateral = 0.0
        self.smoothed_turn_right = 0.0
        self.command = Twist()
        self.last_command_time = None
        self.last_update_time = None
        self.publisher = self.create_publisher(
            Float64MultiArray, "/joint_group_position_controller/commands", 10
        )
        self.status_publisher = self.create_publisher(String, "/apex/gait/status", 10)
        self.create_subscription(Twist, "/apex/cmd_vel", self.on_command, 10)
        self.create_timer(1.0 / float(self.get_parameter("update_rate_hz").value), self.on_update)

    def on_command(self, message):
        self.command = message
        self.last_command_time = self.get_clock().now()

    def command_age(self):
        if self.last_command_time is None:
            return math.inf
        return (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9

    def on_update(self):
        now = self.get_clock().now()
        if self.last_update_time is None:
            dt = 1.0 / float(self.get_parameter("update_rate_hz").value)
        else:
            dt = clamp((now - self.last_update_time).nanoseconds / 1e9, 0.001, 0.1)
        self.last_update_time = now

        command_fresh = self.command_age() <= float(self.get_parameter("command_timeout_s").value)
        linear_scale = float(self.get_parameter("command_linear_scale_mps").value)
        angular_scale = float(self.get_parameter("command_angular_scale_rps").value)
        raw_forward = clamp(self.command.linear.x / linear_scale, -1.0, 1.0) if command_fresh else 0.0
        raw_lateral = clamp(self.command.linear.y / linear_scale, -1.0, 1.0) if command_fresh else 0.0
        # Firmware joyR is positive for a right turn; ROS angular.z is positive left.
        raw_turn_right = clamp(-self.command.angular.z / angular_scale, -1.0, 1.0) if command_fresh else 0.0

        max_delta = 6.0 * dt  # v55: 0.12 per 20 ms
        self.smoothed_forward = approach(self.smoothed_forward, raw_forward, max_delta)
        self.smoothed_lateral = approach(self.smoothed_lateral, raw_lateral, max_delta)
        self.smoothed_turn_right = approach(self.smoothed_turn_right, raw_turn_right, max_delta)
        raw_magnitude = math.sqrt(raw_forward ** 2 + raw_lateral ** 2 + raw_turn_right ** 2)
        smooth_magnitude = math.sqrt(
            self.smoothed_forward ** 2 + self.smoothed_lateral ** 2 + self.smoothed_turn_right ** 2
        )
        if raw_magnitude > 0.05:
            self.phase = (self.phase + dt * max(smooth_magnitude, 0.1) / self.cycle_s) % 1.0

        targets = tripod_targets(
            self.phase,
            self.smoothed_forward,
            self.smoothed_lateral,
            self.smoothed_turn_right,
            self.mount_yaws,
            self.phase_offsets,
            self.radial_m,
            self.stance_z_m,
            self.step_length_m,
            self.step_lift_m,
        )
        message = Float64MultiArray()
        message.data = [angle for target, _swing in targets for angle in target] + [0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(message)

        status = String()
        gait_active = smooth_magnitude > 0.02
        status.data = json.dumps({
            "phase": round(self.phase, 4),
            "command_fresh": command_fresh,
            "forward": round(self.smoothed_forward, 3),
            "lateral": round(self.smoothed_lateral, 3),
            "turn_right": round(self.smoothed_turn_right, 3),
            "swing_legs": [LEG_ORDER[index] for index, (_target, swing) in enumerate(targets) if swing] if gait_active else [],
            "active": gait_active,
            "mode": "physical_contact_tripod",
        })
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TripodGaitController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
