import json
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class SimPerformanceMonitor(Node):
    """Reports real-time factor and delivered sensor rates every five seconds."""

    def __init__(self):
        super().__init__("sim_performance_monitor")
        self.declare_parameter("report_period_s", 5.0)
        self.wall_start = time.monotonic()
        self.sim_start = None
        self.sim_latest = None
        self.camera_count = 0
        self.lidar_count = 0
        self.publisher = self.create_publisher(String, "/apex/sim/performance", 10)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Image, "/apex/camera", self.on_camera, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/apex/lidar_scan", self.on_lidar, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter("report_period_s").value), self.report)

    @staticmethod
    def seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def on_clock(self, message):
        value = self.seconds(message.clock)
        if self.sim_start is None:
            self.sim_start = value
        self.sim_latest = value

    def on_camera(self, _message):
        self.camera_count += 1

    def on_lidar(self, _message):
        self.lidar_count += 1

    def report(self):
        wall_now = time.monotonic()
        wall_elapsed = max(1e-6, wall_now - self.wall_start)
        clock_ready = self.sim_latest is not None and self.sim_start is not None
        sim_elapsed = 0.0 if not clock_ready else self.sim_latest - self.sim_start
        data = {
            "state": "running" if clock_ready else "waiting_for_clock",
            "rtf": round(sim_elapsed / wall_elapsed, 3) if clock_ready else None,
            "camera_fps": round(self.camera_count / wall_elapsed, 2),
            "lidar_hz": round(self.lidar_count / wall_elapsed, 2),
            "window_s": round(wall_elapsed, 2),
        }
        message = String()
        message.data = json.dumps(data)
        self.publisher.publish(message)
        self.get_logger().info(message.data)
        self.wall_start = wall_now
        self.sim_start = self.sim_latest
        self.camera_count = 0
        self.lidar_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = SimPerformanceMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
