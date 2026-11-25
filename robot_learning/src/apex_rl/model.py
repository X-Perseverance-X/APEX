"""Build a separate 18-DOF MuJoCo physics proxy from the META-1 contract.

This model intentionally has no live hardware transport. Link masses, actuator
torque/lag and body shell are estimates, not identified physical parameters.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from .contract import LEG_ORDER, load_contract


def stand_joint_angles(config: dict | None = None) -> list[float]:
    config = config or load_contract()
    return [
        angle
        for leg in LEG_ORDER
        for angle in (
            float(config["stand_coxa_rad"][leg]),
            float(config["stand_joint_rad"]["femur"]),
            float(config["stand_joint_rad"]["tibia"]),
        )
    ]


def build_xml(config: dict | None = None) -> str:
    cfg = config or load_contract()
    fmt = lambda values: " ".join(f"{float(value):.9g}" for value in values)
    root = ET.Element("mujoco", model="apex_meta1_rl_proxy")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast", gravity="0 0 -9.81")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", armature="0.002", damping="0.2")
    ET.SubElement(defaults, "geom", solref="0.02 1", solimp="0.9 0.95 0.001")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 0 3", directional="true", dir="0 0 -1")
    ET.SubElement(
        world, "camera", name="overview", mode="targetbody", target="base", pos="0.75 -1.3 0.65"
    )
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="plane",
        size="3 3 0.1",
        friction=f"{cfg['ground_friction']} 0.005 0.0001",
        rgba="0.12 0.15 0.17 1",
    )
    height = -float(cfg["links_m"]["femur"]) * math.sin(cfg["stand_joint_rad"]["femur"])
    height -= float(cfg["links_m"]["tibia"]) * math.sin(
        cfg["stand_joint_rad"]["femur"] + cfg["stand_joint_rad"]["tibia"]
    )
    height += float(cfg["foot_radius_m"])
    torso = ET.SubElement(world, "body", name="base", pos=f"0 0 {height:.9g}")
    ET.SubElement(torso, "freejoint", name="base_free")
    size = [float(v) / 2 for v in cfg["body"]["size_m"]]
    ET.SubElement(
        torso,
        "geom",
        name="body_shell",
        type="box",
        size=fmt(size),
        mass=str(cfg["body"]["mass_kg"]),
        rgba="0.07 0.11 0.14 1",
    )
    ET.SubElement(torso, "site", name="imu_site", pos="0.115 0.015 0.048", size="0.01")
    ET.SubElement(
        torso,
        "camera",
        name="front_camera",
        pos="0.213 0 0.050",
        euler=f"0 {-math.radians(8):.9g} 0",
        fovy="46",
    )
    ET.SubElement(torso, "site", name="lidar_site", pos="0.135 0 0.108", size="0.008")
    actuators = ET.SubElement(root, "actuator")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="imu_gyro", site="imu_site")
    ET.SubElement(sensors, "accelerometer", name="imu_accel", site="imu_site")
    links = cfg["links_m"]
    masses = cfg["link_mass_kg"]
    limits = {
        "coxa": (-1.48353, 1.48353),
        "femur": (-1.48353, 1.48353),
        "tibia": (-3.05433, -0.08727),
    }
    colors = {"coxa": "0.02 0.65 0.68 1", "femur": "0.10 0.28 0.40 1", "tibia": "0.9 0.31 0.04 1"}
    for leg in LEG_ORDER:
        prefix = leg.lower()
        mount = math.radians(float(cfg["mount_yaw_deg"][leg]))
        hip = ET.SubElement(
            torso,
            "body",
            name=f"{prefix}_coxa",
            pos=fmt(cfg["hip_xyz_m"][leg]),
            euler=f"0 0 {mount:.9g}",
        )
        parent = hip
        for joint in ("coxa", "femur", "tibia"):
            if joint != "coxa":
                previous = "coxa" if joint == "femur" else "femur"
                parent = ET.SubElement(
                    parent,
                    "body",
                    name=f"{prefix}_{joint}",
                    pos=f"{float(links[previous]):.9g} 0 0",
                )
            lower, upper = limits[joint]
            joint_name = f"{prefix}_{joint}_joint"
            ET.SubElement(
                parent,
                "joint",
                name=joint_name,
                type="hinge",
                axis="0 0 1" if joint == "coxa" else "0 -1 0",
                range=f"{lower} {upper}",
            )
            ET.SubElement(
                parent,
                "geom",
                name=f"{prefix}_{joint}_link",
                type="capsule",
                fromto=f"0 0 0 {float(links[joint]):.9g} 0 0",
                size="0.012",
                mass=str(masses[joint]),
                rgba=colors[joint],
                contype="0",
                conaffinity="0",
            )
            torque = float(cfg["joint_torque_limit_nm"][joint])
            ET.SubElement(
                actuators,
                "position",
                name=f"{prefix}_{joint}_servo",
                joint=joint_name,
                kp=str(cfg["actuator"]["kp"]),
                kv=str(cfg["actuator"]["kv"]),
                ctrlrange=f"{lower} {upper}",
                forcerange=f"{-torque} {torque}",
            )
        foot = ET.SubElement(
            parent, "body", name=f"{prefix}_foot", pos=f"{float(links['tibia']):.9g} 0 0"
        )
        ET.SubElement(
            foot,
            "geom",
            name=f"{prefix}_foot_geom",
            type="sphere",
            size=str(cfg["foot_radius_m"]),
            mass=str(masses["foot"]),
            friction="1 0.005 0.0001",
            rgba="1 0.4 0.08 1",
        )
        ET.SubElement(foot, "site", name=f"{prefix}_foot_site", type="sphere", size="0.019")
        ET.SubElement(sensors, "touch", name=f"{prefix}_contact", site=f"{prefix}_foot_site")
    angles = stand_joint_angles(cfg)
    qpos = [0, 0, height, 1, 0, 0, 0, *angles]
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", name="stand", qpos=fmt(qpos), ctrl=fmt(angles))
    return ET.tostring(root, encoding="unicode")
