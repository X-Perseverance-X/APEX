"""Build the separate META-1 MuJoCo physics and sensor proxy.

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

    def fmt(values):
        return " ".join(f"{float(value):.9g}" for value in values)

    root = ET.Element("mujoco", model="apex_meta1_rl_proxy")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast", gravity="0 0 -9.81")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", armature="0.002", damping="0.2")
    ET.SubElement(defaults, "geom", solref="0.02 1", solimp="0.9 0.95 0.001")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(
        world,
        "light",
        pos="0.3 -0.4 2",
        directional="true",
        dir="-0.3 0.4 -2",
        ambient="0.25 0.25 0.25",
        diffuse="0.85 0.85 0.85",
    )
    ET.SubElement(
        world,
        "camera",
        name="overview",
        mode="targetbody",
        target="base",
        pos="0.62 -0.42 0.88",
        fovy="39",
    )
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="plane",
        size="3 3 0.1",
        friction=f"{cfg['ground_friction']} 0.005 0.0001",
        rgba="0.26 0.29 0.31 1",
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
        rgba="0.07 0.11 0.14 0",
    )
    # Reference positions are from the robot survey. The ToF location and
    # camera FOV are provisional; a LiDAR site alone is not a 2D scan model.
    sensor_cfg = cfg["sensors"]
    ET.SubElement(
        torso,
        "site",
        name="imu_site",
        pos=fmt(sensor_cfg["imu"]["xyz_m"]),
        size="0.01",
        rgba="0 0 0 0",
    )
    ET.SubElement(
        torso,
        "site",
        name="lidar_site",
        pos=fmt(sensor_cfg["lidar"]["xyz_m"]),
        size="0.008",
        rgba="0 0 0 0",
    )
    ET.SubElement(
        torso,
        "site",
        name="tof_site",
        pos=fmt(sensor_cfg["tof"]["xyz_m"]),
        xyaxes="0 1 0 0 0 1",
        size="0.006",
        rgba="0 0 0 0",
    )
    ET.SubElement(
        torso,
        "geom",
        name="lidar_visual",
        type="cylinder",
        pos=fmt(sensor_cfg["lidar"]["xyz_m"]),
        size="0.044 0.019",
        mass="0",
        contype="0",
        conaffinity="0",
        rgba="0.025 0.025 0.03 1",
    )
    ET.SubElement(
        torso,
        "geom",
        name="tof_visual",
        type="box",
        pos=fmt(sensor_cfg["tof"]["xyz_m"]),
        size="0.014 0.030 0.011",
        mass="0",
        contype="0",
        conaffinity="0",
        rgba="0.02 0.02 0.025 1",
    )
    # Approximate the narrow grey cage and recessed side slots in the photos.
    decorative = {"mass": "0", "contype": "0", "conaffinity": "0"}
    for plate, z in (("top", 0.079), ("bottom", 0.006)):
        ET.SubElement(
            torso,
            "geom",
            name=f"chassis_{plate}",
            type="box",
            pos=fmt([0, 0, z]),
            size="0.181 0.067 0.003",
            rgba="0.42 0.47 0.51 1",
            **decorative,
        )
    ET.SubElement(
        torso,
        "geom",
        name="chassis_inner",
        type="box",
        pos="0 0 0.041",
        size="0.171 0.056 0.032",
        rgba="0.10 0.12 0.13 1",
        **decorative,
    )
    for side, y in (("left", 0.064), ("right", -0.064)):
        for i, x in enumerate(
            (-0.177, -0.139, -0.101, -0.063, -0.025, 0.025, 0.063, 0.101, 0.139, 0.177)
        ):
            ET.SubElement(
                torso,
                "geom",
                name=f"chassis_{side}_rib_{i}",
                type="box",
                pos=fmt([x, y, 0.042]),
                size="0.007 0.003 0.035",
                rgba="0.36 0.40 0.44 1",
                **decorative,
            )
    actuators = ET.SubElement(root, "actuator")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="imu_gyro", site="imu_site")
    ET.SubElement(sensors, "accelerometer", name="imu_accel", site="imu_site")
    ET.SubElement(
        sensors,
        "rangefinder",
        name="forward_tof",
        site="tof_site",
        cutoff=str(sensor_cfg["tof"]["max_range_m"]),
    )
    links = cfg["links_m"]
    masses = cfg["link_mass_kg"]
    limits = {
        "coxa": (-1.48353, 1.48353),
        "femur": (-1.48353, 1.48353),
        "tibia": (-3.05433, -0.08727),
    }
    colors = {"coxa": "0.05 0.06 0.07 1", "femur": "0.04 0.05 0.06 1", "tibia": "0.04 0.05 0.06 1"}
    for leg in LEG_ORDER:
        prefix = leg.lower()
        mount = math.radians(float(cfg["mount_yaw_deg"][leg]))
        x, y, z = cfg["hip_xyz_m"][leg]
        ET.SubElement(
            torso,
            "geom",
            name=f"{prefix}_hip_bracket",
            type="box",
            pos=fmt([x, y * 0.8, z + 0.009]),
            size="0.024 0.026 0.012",
            rgba="0.10 0.11 0.13 1",
            **decorative,
        )
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
            # Dual dark side plates approximate the visible 3D-printed leg
            # brackets. They are visual-only, so photos do not invent inertia.
            if joint in ("femur", "tibia"):
                ET.SubElement(
                    parent,
                    "geom",
                    name=f"{prefix}_{joint}_servo_case",
                    type="box",
                    pos=fmt([float(links[joint]) * 0.22, 0, 0.012]),
                    size="0.021 0.015 0.012",
                    rgba="0.10 0.11 0.13 1",
                    **decorative,
                )
                for side, offset in (("inner", -0.016), ("outer", 0.016)):
                    ET.SubElement(
                        parent,
                        "geom",
                        name=f"{prefix}_{joint}_{side}_plate",
                        type="box",
                        pos=f"{float(links[joint]) / 2:.9g} {offset} 0",
                        size=f"{float(links[joint]) / 2:.9g} 0.003 0.017",
                        mass="0",
                        contype="0",
                        conaffinity="0",
                        rgba="0.018 0.02 0.024 1",
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
            rgba="0.82 0.87 0.87 1",
        )
        ET.SubElement(
            foot, "site", name=f"{prefix}_foot_site", type="sphere", size="0.019", rgba="0 0 0 0"
        )
        ET.SubElement(sensors, "touch", name=f"{prefix}_contact", site=f"{prefix}_foot_site")
    gimbal = sensor_cfg["gimbal"]
    pan_xyz = gimbal["pan_axis_xyz_m"]
    tilt_xyz = gimbal["tilt_axis_xyz_m"]
    camera_xyz = sensor_cfg["camera"]["xyz_m"]
    pan = ET.SubElement(torso, "body", name="camera_pan_body", pos=fmt(pan_xyz))
    ET.SubElement(
        pan, "inertial", pos="0 0 0", mass="0.001", diaginertia="0.000001 0.000001 0.000001"
    )
    gimbal_range = fmt(map(math.radians, gimbal["limit_deg"]))
    ET.SubElement(
        pan, "joint", name="camera_pan_joint", type="hinge", axis="0 0 1", range=gimbal_range
    )
    ET.SubElement(
        pan,
        "geom",
        name="camera_pan_visual",
        type="cylinder",
        pos="0 0 0.038",
        size="0.038 0.014",
        mass="0",
        contype="0",
        conaffinity="0",
        rgba="0.02 0.025 0.03 1",
    )
    tilt = ET.SubElement(
        pan, "body", name="camera_tilt_body", pos=fmt([b - a for a, b in zip(pan_xyz, tilt_xyz)])
    )
    ET.SubElement(
        tilt, "inertial", pos="0 0 0", mass="0.001", diaginertia="0.000001 0.000001 0.000001"
    )
    ET.SubElement(
        tilt, "joint", name="camera_tilt_joint", type="hinge", axis="0 1 0", range=gimbal_range
    )
    ET.SubElement(
        tilt,
        "geom",
        name="camera_head_visual",
        type="box",
        pos="0.008 0 0",
        size="0.024 0.043 0.025",
        mass="0",
        contype="0",
        conaffinity="0",
        rgba="0.025 0.03 0.035 1",
    )
    camera_local = [b - a for a, b in zip(tilt_xyz, camera_xyz)]
    fov_h = math.radians(float(sensor_cfg["camera"]["horizontal_fov_deg"]))
    fov_v = math.degrees(2 * math.atan(math.tan(fov_h / 2) * 720 / 1280))
    ET.SubElement(
        tilt,
        "camera",
        name="front_camera",
        pos=fmt(camera_local),
        xyaxes="0 -1 0 0 0 1",
        fovy=f"{fov_v:.9g}",
    )
    ET.SubElement(
        tilt,
        "geom",
        name="camera_lens_visual",
        type="cylinder",
        pos=fmt([camera_local[0] - 0.008, *camera_local[1:]]),
        euler=f"0 {math.pi / 2:.9g} 0",
        size="0.011 0.006",
        mass="0",
        contype="0",
        conaffinity="0",
        rgba="0.01 0.01 0.02 1",
    )
    for side, y in (("right", -0.028), ("left", 0.028)):
        ET.SubElement(
            tilt,
            "geom",
            name=f"camera_ir_{side}_visual",
            type="sphere",
            pos=fmt([camera_local[0], y, camera_local[2]]),
            size="0.007",
            mass="0",
            contype="0",
            conaffinity="0",
            rgba="0.5 0.55 0.6 1",
        )
    for joint in ("pan", "tilt"):
        ET.SubElement(
            actuators,
            "position",
            name=f"camera_{joint}_servo",
            joint=f"camera_{joint}_joint",
            kp="5",
            kv="0.2",
            ctrlrange=gimbal_range,
            forcerange="-0.5 0.5",
        )
    angles = stand_joint_angles(cfg)
    qpos = [0, 0, height, 1, 0, 0, 0, *angles, 0, 0]
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", name="stand", qpos=fmt(qpos), ctrl=fmt([*angles, 0, 0]))
    return ET.tostring(root, encoding="unicode")
