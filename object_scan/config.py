"""Validated sensor-rig configuration and frame transforms.

Physical poses use the robot convention (+X forward, +Y left, +Z up).
Camera projection uses the OpenCV optical convention (+X right, +Y down,
+Z forward), so the conversion is kept in one tested place.
"""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np


DEFAULT_SENSOR_RIG = Path(__file__).with_name("sensor_rig.json")
_SENSORS = ("lidar", "camera", "imu")


def _finite(value: object, label: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _rpy_matrix(rotation: Mapping[str, object]) -> np.ndarray:
    roll = math.radians(_finite(rotation.get("roll", 0.0), "roll"))
    pitch = math.radians(_finite(rotation.get("pitch", 0.0), "pitch"))
    yaw = math.radians(_finite(rotation.get("yaw", 0.0), "yaw"))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.asarray(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.asarray(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def _reference_from_sensor(sensor: Mapping[str, object]) -> np.ndarray:
    translation = sensor.get("translation_m", {})
    rotation = sensor.get("rotation_rpy_deg", {})
    if not isinstance(translation, Mapping) or not isinstance(rotation, Mapping):
        raise ValueError("sensor translation/rotation must be objects")
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _rpy_matrix(rotation)
    transform[:3, 3] = (
        _finite(translation.get("x", 0.0), "translation.x"),
        _finite(translation.get("y", 0.0), "translation.y"),
        _finite(translation.get("z", 0.0), "translation.z"),
    )
    return transform


def load_sensor_rig(path: str | Path = DEFAULT_SENSOR_RIG) -> dict:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema") != "apex-sensor-rig-v1":
        raise ValueError("unsupported sensor-rig schema")
    if payload.get("coordinate_convention") != "x_forward_y_left_z_up":
        raise ValueError("sensor rig must use x_forward_y_left_z_up")
    for name in _SENSORS:
        if not isinstance(payload.get(name), dict):
            raise ValueError(f"missing sensor configuration: {name}")
        _reference_from_sensor(payload[name])
    fov = _finite(payload["camera"].get("horizontal_fov_deg", 0.0), "camera.horizontal_fov_deg")
    if not 20.0 <= fov <= 179.0:
        raise ValueError("camera.horizontal_fov_deg must be within [20, 179]")
    return payload


def with_translation_overrides(rig: dict, overrides: Mapping[str, float | None]) -> dict:
    """Return a copy with CLI translations such as camera_x or imu_z applied."""
    updated = copy.deepcopy(rig)
    for sensor in ("camera", "imu"):
        for axis in ("x", "y", "z"):
            key = f"{sensor}_{axis}"
            value = overrides.get(key)
            if value is not None:
                updated[sensor]["translation_m"][axis] = _finite(value, key)
    return updated


def camera_from_lidar(rig: Mapping[str, object]) -> np.ndarray:
    """Transform LiDAR XYZ into OpenCV camera optical coordinates."""
    reference_from_lidar = _reference_from_sensor(rig["lidar"])
    reference_from_camera_mechanical = _reference_from_sensor(rig["camera"])
    camera_mechanical_from_lidar = np.linalg.inv(reference_from_camera_mechanical) @ reference_from_lidar
    camera_optical_from_mechanical = np.eye(4, dtype=np.float64)
    camera_optical_from_mechanical[:3, :3] = np.asarray(
        ((0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0)),
        dtype=np.float64,
    )
    return camera_optical_from_mechanical @ camera_mechanical_from_lidar


def camera_matrix(rig: Mapping[str, object], width: int, height: int) -> np.ndarray | None:
    """Return scaled calibrated intrinsics, or None to use the FOV fallback."""
    intrinsics = rig["camera"].get("intrinsics", {})
    if not isinstance(intrinsics, Mapping):
        return None
    required = ("calibration_width_px", "calibration_height_px", "fx", "fy", "cx", "cy")
    if any(intrinsics.get(name) is None for name in required):
        return None
    calibration_width = _finite(intrinsics["calibration_width_px"], "calibration_width_px")
    calibration_height = _finite(intrinsics["calibration_height_px"], "calibration_height_px")
    if calibration_width <= 0 or calibration_height <= 0:
        raise ValueError("camera calibration dimensions must be positive")
    sx, sy = width / calibration_width, height / calibration_height
    return np.asarray((
        (_finite(intrinsics["fx"], "fx") * sx, 0.0, _finite(intrinsics["cx"], "cx") * sx),
        (0.0, _finite(intrinsics["fy"], "fy") * sy, _finite(intrinsics["cy"], "cy") * sy),
        (0.0, 0.0, 1.0),
    ), dtype=np.float64)


def camera_distortion(rig: Mapping[str, object]) -> np.ndarray | None:
    intrinsics = rig["camera"].get("intrinsics", {})
    if not isinstance(intrinsics, Mapping) or intrinsics.get("status") != "calibrated_checkerboard":
        return None
    values = intrinsics.get("distortion_coefficients", [])
    if not isinstance(values, list) or not values:
        return None
    coefficients = np.asarray([_finite(value, "distortion coefficient") for value in values], dtype=np.float64)
    return coefficients
