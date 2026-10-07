"""Pause map integration whenever planar assumptions are not met."""

from __future__ import annotations

import math


def mapping_pause_reason(
    roll_rad: float | None,
    pitch_rad: float | None,
    *,
    max_roll_deg: float,
    max_pitch_deg: float,
    imu_axes_verified: bool,
    lidar_angle_verified: bool,
) -> str | None:
    if not imu_axes_verified or not lidar_angle_verified:
        return "UNVERIFIED SENSOR FRAMES"
    if roll_rad is None or pitch_rad is None:
        return "NO IMU ATTITUDE"
    if abs(math.degrees(roll_rad)) > max_roll_deg or abs(math.degrees(pitch_rad)) > max_pitch_deg:
        return "EXCESSIVE TILT"
    return None

