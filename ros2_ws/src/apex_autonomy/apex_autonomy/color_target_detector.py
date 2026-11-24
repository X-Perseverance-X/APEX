import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ColorTargetDetector(Node):
    """Deterministic camera adapter for the magenta SIL person target.

    TargetObservation contract (Vector3Stamped.vector):
      x: horizontal image error [-1, +1], positive means target is right
      y: pinhole range estimate in metres
      z: confidence [0, 1], zero means not visible
    The real Hailo adapter can publish the same contract later.
    """

    def __init__(self):
        super().__init__("color_target_detector")
        self.declare_parameter("image_topic", "/apex/camera")
        self.declare_parameter("observation_topic", "/apex/target_observation")
        self.declare_parameter("debug_image_topic", "/apex/tracking/debug_image")
        self.declare_parameter("target_height_m", 1.40)
        self.declare_parameter("horizontal_fov_deg", 60.0)
        self.declare_parameter("minimum_area_px", 120.0)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Vector3Stamped, self.get_parameter("observation_topic").value, 10
        )
        self.debug_publisher = self.create_publisher(
            Image, self.get_parameter("debug_image_topic").value, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.on_image,
            qos_profile_sensor_data,
        )

    def on_image(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([135, 90, 70]), np.array([179, 255, 255]))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        observation = Vector3Stamped()
        observation.header = message.header
        observation.header.frame_id = "camera_optical_frame"
        minimum_area = float(self.get_parameter("minimum_area_px").value)
        candidates = [contour for contour in contours if cv2.contourArea(contour) >= minimum_area]
        if not candidates:
            observation.vector.z = 0.0
            self.publisher.publish(observation)
            self.publish_debug(frame, message)
            return

        contour = max(candidates, key=cv2.contourArea)
        x, y, width, height = cv2.boundingRect(contour)
        image_height, image_width = frame.shape[:2]
        center_x = x + width / 2.0
        error = (center_x - image_width / 2.0) / (image_width / 2.0)
        hfov = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        focal_px = image_width / (2.0 * math.tan(hfov / 2.0))
        target_height = float(self.get_parameter("target_height_m").value)
        distance = target_height * focal_px / max(1.0, float(height))
        area_ratio = cv2.contourArea(contour) / float(image_width * image_height)
        confidence = min(1.0, 0.45 + 8.0 * area_ratio)

        observation.vector.x = float(max(-1.0, min(1.0, error)))
        observation.vector.y = float(distance)
        observation.vector.z = float(confidence)
        self.publisher.publish(observation)

        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 255), 2)
        cv2.line(frame, (image_width // 2, image_height // 2),
                 (int(center_x), int(y + height / 2)), (0, 255, 255), 1)
        cv2.putText(frame, f"SIL TARGET  err={error:+.2f}  d={distance:.2f}m",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
        self.publish_debug(frame, message)

    def publish_debug(self, frame, source):
        debug = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        debug.header = source.header
        self.debug_publisher.publish(debug)


def main(args=None):
    rclpy.init(args=args)
    node = ColorTargetDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
