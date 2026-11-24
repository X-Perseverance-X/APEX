import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PROFILES = {
    # Best choice for WSL software rendering and interactive autonomy work.
    "fast": dict(camera_width=320, camera_height=180, camera_rate=10,
                 lidar_samples=180, lidar_rate=5, imu_rate=50, tof_rate=15),
    # Default: enough image detail for the SIL detector without 640x480 cost.
    "balanced": dict(camera_width=424, camera_height=240, camera_rate=15,
                     lidar_samples=360, lidar_rate=10, imu_rate=100, tof_rate=20),
    # Dataset / final verification profile; expected to be slower on WSL Mesa.
    "fidelity": dict(camera_width=640, camera_height=480, camera_rate=20,
                     lidar_samples=720, lidar_rate=10, imu_rate=100, tof_rate=20),
}


def launch_setup(context):
    description = Path(get_package_share_directory("apex_description"))
    gazebo = Path(get_package_share_directory("apex_gazebo"))
    autonomy = Path(get_package_share_directory("apex_autonomy"))
    ros_gz = Path(get_package_share_directory("ros_gz_sim"))
    world = gazebo / "worlds" / "apex_lab.sdf"

    profile_name = LaunchConfiguration("profile").perform(context).strip().lower()
    if profile_name not in PROFILES:
        raise RuntimeError(
            f"Unknown profile '{profile_name}'. Choose: {', '.join(PROFILES)}"
        )
    profile = PROFILES[profile_name]
    xacro_file = description / "urdf" / "apex_meta1.urdf.xacro"
    xacro_arguments = []
    for key, value in profile.items():
        xacro_arguments.extend([f" {key}:=", str(value)])
    robot_description = Command(["xacro ", str(xacro_file), *xacro_arguments])

    gui_enabled = LaunchConfiguration("gui").perform(context).lower() == "true"
    gz_args = f"-r -v 3 {world}" if gui_enabled else f"-s --headless-rendering -r -v 3 {world}"
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
            "use_sim_time": True,
        }],
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "apex_meta1", "-topic", "robot_description",
                   "-x", "0", "-y", "0", "-z", "0.122"],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": str(gazebo / "config" / "bridge.yaml")}],
        output="screen",
    )
    joint_state = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )
    position = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[joint_state, position])
    )
    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(autonomy / "launch" / "tracking.launch.py")),
        condition=IfCondition(LaunchConfiguration("autonomy_nodes")),
    )
    # Avoid racing ros_gz_sim/create against Gazebo's world discovery.  Without
    # this delay an old server can receive the entity before the new server is
    # ready, leaving this launch without a controller_manager.
    delayed_spawn = TimerAction(period=2.0, actions=[spawn])
    return [gz, state_publisher, delayed_spawn, bridge, controllers_after_spawn, tracking]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Open Gazebo GUI; false runs server-only headless rendering.",
        ),
        DeclareLaunchArgument(
            "software_rendering", default_value="true",
            description="Use stable Mesa software rendering for WSL OGRE2.",
        ),
        DeclareLaunchArgument(
            "profile", default_value="balanced",
            description="Sensor workload: fast, balanced or fidelity.",
        ),
        DeclareLaunchArgument(
            "autonomy_nodes", default_value="true",
            description="Start detector/controller nodes (tracking remains safety-disabled).",
        ),
        DeclareLaunchArgument(
            "partition",
            default_value=os.environ.get("IGN_PARTITION", f"apex_meta1_sil_{os.getpid()}"),
            description="Isolated Gazebo Transport partition; prevents stale-server collisions.",
        ),
        SetEnvironmentVariable(
            name="IGN_PARTITION", value=LaunchConfiguration("partition"),
        ),
        SetEnvironmentVariable(
            name="LIBGL_ALWAYS_SOFTWARE", value="1",
            condition=IfCondition(LaunchConfiguration("software_rendering")),
        ),
        OpaqueFunction(function=launch_setup),
    ])
