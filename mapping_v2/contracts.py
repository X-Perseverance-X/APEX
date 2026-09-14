"""Timestamped sensor contracts used by the mapping sidecar.

Frames follow the robot convention used by the HUD and URDF:
``+X`` forward, ``+Y`` left and ``+Z`` up.  Timestamps are Raspberry Pi
monotonic nanoseconds; they are never wall-clock time.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class Pose2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    trusted: bool = False
    source: str = "fixed_local"


@dataclass(frozen=True)
class LidarScan:
    timestamp_ns: int
    sequence: int
    points: Sequence[tuple[float, float]]  # angle degrees, range millimetres
    frame_id: str = "lidar_link"


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    valid: bool
    frame_id: str = "imu_link"


@dataclass(frozen=True)
class TofSample:
    timestamp_ns: int
    distance_mm: float
    valid: bool
    frame_id: str = "tof_link"
