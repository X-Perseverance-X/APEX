import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import SetBool


class SimTargetDriver(Node):
    """Optional repeatable motion source for the Gazebo magenta target."""

    def __init__(self):
        super().__init__("sim_target_driver")
        self.declare_parameter("enabled", False)
        self.declare_parameter("mode", "circle")
        self.declare_parameter("linear_speed_mps", 0.12)
        self.declare_parameter("yaw_rate_rps", 0.20)
        self.enabled = bool(self.get_parameter("enabled").value)
        self.publisher = self.create_publisher(Twist, "/apex/sim_target/cmd_vel", 10)
        self.create_service(SetBool, "/apex/sim_target/enable", self.on_enable)
        self.create_timer(0.1, self.on_timer)

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.publisher.publish(Twist())
        response.success = True
        response.message = "target moving" if self.enabled else "target stopped"
        return response

    def on_timer(self):
        command = Twist()
        if self.enabled:
            mode = str(self.get_parameter("mode").value)
            elapsed = self.get_clock().now().nanoseconds / 1e9
            command.linear.x = float(self.get_parameter("linear_speed_mps").value)
            yaw_rate = float(self.get_parameter("yaw_rate_rps").value)
            command.angular.z = yaw_rate if mode == "circle" else yaw_rate * math.sin(elapsed * 0.35)
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = SimTargetDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
