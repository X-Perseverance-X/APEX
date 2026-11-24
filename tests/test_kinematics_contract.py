from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "apex_kinematics_lab" / "config" / "default_robot.json"
FIRMWARE_PATH = ROOT / "firmware" / "v57_spine_safe_boot" / "v57_spine_safe_boot.ino"
APP_PATH = ROOT / "apex_kinematics_lab" / "static" / "app.js"
XACRO_PATH = ROOT / "ros2_ws" / "src" / "apex_description" / "urdf" / "apex_meta1.urdf.xacro"
MODEL_CONFIG_PATH = ROOT / "ros2_ws" / "src" / "apex_description" / "config" / "model_parameters.yaml"
WORLD_PATH = ROOT / "ros2_ws" / "src" / "apex_gazebo" / "worlds" / "apex_lab.sdf"
BRIDGE_PATH = ROOT / "ros2_ws" / "src" / "apex_gazebo" / "config" / "bridge.yaml"
FW_ORDER = ("RF", "RM", "RR", "LF", "LM", "LR")
JOINTS = ("coxa", "femur", "tibia")


def _array(source: str, name: str) -> list[float]:
    match = re.search(rf"{re.escape(name)}\[6\]\s*=\s*\{{([^}}]+)\}}", source)
    if not match:
        raise AssertionError(f"firmware array not found: {name}")
    return [float(x) for x in re.findall(r"[-+]?\d+(?:\.\d+)?", match.group(1))]


def _constant(source: str, name: str) -> float:
    match = re.search(rf"const float {re.escape(name)}\s*=\s*([-+]?\d+(?:\.\d+)?)", source)
    if not match:
        raise AssertionError(f"firmware constant not found: {name}")
    return float(match.group(1))


def _geometry_to_servo(leg: dict, joint: str, geometry_deg: float) -> float:
    spec = leg["joints"][joint]
    return float(spec["zero"]) + float(spec["dir"]) * (geometry_deg - 90.0)


def _servo_to_geometry(leg: dict, joint: str, servo_deg: float) -> float:
    spec = leg["joints"][joint]
    return 90.0 + (servo_deg - float(spec["zero"])) * float(spec["dir"])


def _ik(config: dict, name: str, target: dict) -> dict[str, float]:
    leg = config["legs"][name]
    lengths = config["legCommon"]
    dx = target["x"] - leg["hip"]["x"]
    dy = target["y"] - leg["hip"]["y"]
    z = target["z"] - leg["hip"]["z"]
    y_out = -dy if leg["side"] == "right" else dy
    r = max(5.0, math.hypot(dx, y_out) - lengths["coxa"])
    c = math.hypot(r, z)
    cos_f = (lengths["femur"] ** 2 + c**2 - lengths["tibia"] ** 2) / (2 * lengths["femur"] * c)
    cos_t = (lengths["femur"] ** 2 + lengths["tibia"] ** 2 - c**2) / (2 * lengths["femur"] * lengths["tibia"])
    geometry = {
        "coxa": math.degrees(math.atan2(y_out, dx)),
        "femur": math.degrees(math.atan2(r, -z) + math.acos(max(-1.0, min(1.0, cos_f)))),
        "tibia": 180.0 - math.degrees(math.acos(max(-1.0, min(1.0, cos_t)))),
    }
    return {joint: _geometry_to_servo(leg, joint, geometry[joint]) for joint in JOINTS}


def _fk(config: dict, name: str, servo: dict[str, float]) -> tuple[float, float, float]:
    leg = config["legs"][name]
    lengths = config["legCommon"]
    coxa = math.radians(_servo_to_geometry(leg, "coxa", servo["coxa"]))
    femur = math.radians(_servo_to_geometry(leg, "femur", servo["femur"]) - 90.0)
    tibia = math.radians(-_servo_to_geometry(leg, "tibia", servo["tibia"]))
    side = -1.0 if leg["side"] == "right" else 1.0
    ux, uy = math.cos(coxa), side * math.sin(coxa)
    radial = (
        lengths["coxa"]
        + lengths["femur"] * math.cos(femur)
        + lengths["tibia"] * math.cos(femur + tibia)
    )
    z = leg["hip"]["z"] + lengths["femur"] * math.sin(femur) + lengths["tibia"] * math.sin(femur + tibia)
    return leg["hip"]["x"] + ux * radial, leg["hip"]["y"] + uy * radial, z


class KinematicsContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        cls.firmware = FIRMWARE_PATH.read_text(encoding="utf-8")
        cls.model_config = yaml.safe_load(MODEL_CONFIG_PATH.read_text(encoding="utf-8"))

    def test_config_geometry_matches_firmware_constants(self) -> None:
        self.assertEqual(self.config["legCommon"]["femur"], _constant(self.firmware, "a"))
        self.assertEqual(self.config["legCommon"]["tibia"], _constant(self.firmware, "b"))
        self.assertEqual(self.config["legCommon"]["coxa"], _constant(self.firmware, "l"))
        xs = _array(self.firmware, "bodyOffsetX")
        ys = _array(self.firmware, "bodyOffsetY")
        for index, name in enumerate(FW_ORDER):
            self.assertEqual(self.config["legs"][name]["hip"]["x"], xs[index])
            self.assertEqual(self.config["legs"][name]["hip"]["y"], ys[index])

    def test_default_targets_are_firmware_stance(self) -> None:
        for name in FW_ORDER:
            leg = self.config["legs"][name]
            self.assertEqual(leg["target"]["x"] - leg["hip"]["x"], 0)
            self.assertEqual(leg["target"]["y"] - leg["hip"]["y"], -150 if leg["side"] == "right" else 150)
            self.assertEqual(leg["target"]["z"] - leg["hip"]["z"], -100)
            self.assertEqual(self.config["body"]["pose"]["z"] + leg["target"]["z"], 0)

    def test_ik_fk_round_trip_for_all_six_legs(self) -> None:
        for name in FW_ORDER:
            target = self.config["legs"][name]["target"]
            servo = _ik(self.config, name, target)
            foot = _fk(self.config, name, servo)
            error = math.dist(foot, (target["x"], target["y"], target["z"]))
            self.assertLess(error, 1e-9, f"{name}: {servo=} {foot=}")
            self.assertTrue(all(5.0 <= value <= 175.0 for value in servo.values()))

    def test_ik_fk_round_trip_across_gait_workspace(self) -> None:
        for name in FW_ORDER:
            base = self.config["legs"][name]["target"]
            for dx, dz in ((-50, 0), (-25, 35), (0, 50), (25, 35), (50, 0)):
                target = {"x": base["x"] + dx, "y": base["y"], "z": base["z"] + dz}
                servo = _ik(self.config, name, target)
                foot = _fk(self.config, name, servo)
                self.assertLess(math.dist(foot, (target["x"], target["y"], target["z"])), 1e-9)
                self.assertTrue(all(5.0 <= value <= 175.0 for value in servo.values()))

    def test_left_right_servo_mirroring(self) -> None:
        right = _ik(self.config, "RF", self.config["legs"]["RF"]["target"])
        left = _ik(self.config, "LF", self.config["legs"]["LF"]["target"])
        self.assertAlmostEqual(right["coxa"], 90.0, places=9)
        self.assertAlmostEqual(left["coxa"], 90.0, places=9)
        self.assertAlmostEqual(right["femur"] + left["femur"], 180.0, places=9)
        self.assertAlmostEqual(right["tibia"] + left["tibia"], 180.0, places=9)

    def test_frontend_and_xacro_keep_physical_angle_contract(self) -> None:
        app = APP_PATH.read_text(encoding="utf-8")
        self.assertIn('const q3 = rad(-servoToGeometry(name, "tibia", servo.tibia));', app)
        self.assertIn("const pts = displayRes.points.map(toWorld);", app)
        self.assertNotIn("servoToJoint", app)
        xacro = XACRO_PATH.read_text(encoding="utf-8")
        self.assertIn('<xacro:property name="stand_tibia" value="-2.276991532"/>', xacro)
        self.assertIn("value=\"${xacro.load_yaml('$(find apex_description)/config/model_parameters.yaml')}\"", xacro)
        for name in FW_ORDER:
            key = name.lower()
            self.assertIn(f"prefix=\"{key}\"", xacro)
            self.assertIn(f"legs_cfg['hip_xyz_m']['{name}'][0]", xacro)
            self.assertIn(f"legs_cfg['mount_yaw_deg']['{name}']*pi/180", xacro)
            expected_hip = [self.config["legs"][name]["hip"][axis] / 1000.0 for axis in ("x", "y", "z")]
            self.assertEqual(self.model_config["legs"]["hip_xyz_m"][name], expected_hip)

    def test_live_twin_separates_body_ik_and_visual_pitch_signs(self) -> None:
        app = APP_PATH.read_text(encoding="utf-8")
        self.assertIn('mode: "live_drive",', app)
        self.assertIn('cfg.live.mode = "live_drive";', app)
        self.assertIn('imuPitchSign: -1,', app)
        self.assertIn('cfg.live.imuPitchSign = Number(cfg.live.imuPitchSign ?? -1);', app)
        self.assertIn('        ? -liveImuValue("pitch", telemetry)', app)
        self.assertNotIn('        ? liveImuValue("pitch", telemetry)\n        : Number(b.pose.pitch);', app)

    def test_fortress_and_sensor_mount_contract(self) -> None:
        xacro = XACRO_PATH.read_text(encoding="utf-8")
        self.assertIn("sensor_cfg['camera']['xyz_m']['value'][0]", xacro)
        self.assertIn("sensor_cfg['tof']['xyz_m']['value'][0]", xacro)
        self.assertIn("sensor_cfg['imu']['xyz_m']['value'][0]", xacro)
        sensor_config = self.model_config["sensor_cluster"]
        self.assertEqual(sensor_config["parent"], "base_link")
        self.assertEqual(sensor_config["xyz_m"]["status"], "measured")
        self.assertEqual(sensor_config["camera"]["xyz_m"]["status"], "measured")
        self.assertEqual(sensor_config["lidar"]["xyz_m"]["status"], "measured")
        self.assertEqual(sensor_config["imu"]["xyz_m"]["status"], "partially_measured")
        self.assertEqual(sensor_config["tof"]["xyz_m"]["status"], "provisional")
        world = WORLD_PATH.read_text(encoding="utf-8")
        self.assertIn('filename="ignition-gazebo-physics-system"', world)
        self.assertNotIn('filename="gz-sim-physics-system"', world)
        bridge = BRIDGE_PATH.read_text(encoding="utf-8")
        self.assertIn("ros_topic_name: /apex/camera", bridge)
        self.assertIn("ros_topic_name: /apex/lidar_scan", bridge)
        self.assertIn("gz_type_name: ignition.msgs.Image", bridge)


if __name__ == "__main__":
    unittest.main()
