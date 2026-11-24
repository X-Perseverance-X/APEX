from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("apex_autonomy"))
    parameters = str(share / "config" / "tracking.yaml")
    return LaunchDescription([
        Node(package="apex_autonomy", executable="color_target_detector",
             name="color_target_detector", parameters=[parameters], output="screen"),
        Node(package="apex_autonomy", executable="target_follow_controller",
             name="target_follow_controller", parameters=[parameters], output="screen"),
        Node(package="apex_autonomy", executable="sim_target_driver",
             name="sim_target_driver", parameters=[parameters], output="screen"),
        Node(package="apex_autonomy", executable="sim_performance_monitor",
             name="sim_performance_monitor", output="screen"),
        Node(package="apex_autonomy", executable="tripod_gait_controller",
             name="tripod_gait_controller", parameters=[parameters], output="screen"),
    ])
