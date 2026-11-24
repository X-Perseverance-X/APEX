from setuptools import find_packages, setup

package_name = "apex_autonomy"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/tracking.launch.py"]),
        ("share/" + package_name + "/config", ["config/tracking.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="APEX Robotics",
    maintainer_email="maintainer@example.com",
    description="Hardware-independent APEX perception and following nodes.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "color_target_detector = apex_autonomy.color_target_detector:main",
            "target_follow_controller = apex_autonomy.target_follow_controller:main",
            "sim_target_driver = apex_autonomy.sim_target_driver:main",
            "sim_performance_monitor = apex_autonomy.sim_performance_monitor:main",
            "tripod_gait_controller = apex_autonomy.tripod_gait_controller:main",
        ]
    },
)
