"""LiDAR scan contract. Angles are normalized to lidar_link radians."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

from core.frames.geometry import polar_xy


@dataclass(frozen=True)
class LidarPoint:
    angle_rad: float
    range_m: float
    quality: int

    def xy_m(self) -> tuple[float, float]:
        return polar_xy(self.angle_rad, self.range_m)


@dataclass(frozen=True)
class LidarScan:
    timestamp_start_ns: int
    timestamp_end_ns: int
    sequence: int
    points: tuple[LidarPoint, ...]
    frame: str = "lidar_link"


def normalize_scan(
    payload: dict,
    *,
    min_range_m: float,
    max_range_m: float,
    min_quality: int,
) -> LidarScan:
    start = int(payload["timestamp_start_ns"])
    end = int(payload["timestamp_end_ns"])
    if start < 0 or end < start:
        raise ValueError("invalid scan timestamp interval")
    sequence = int(payload["sequence"])
    if sequence < 0:
        raise ValueError("negative scan sequence")
    points: list[LidarPoint] = []
    for source in payload["points"]:
        angle = float(source[0])
        distance = float(source[1])
        quality = int(source[2])
        if (
            math.isfinite(angle)
            and math.isfinite(distance)
            and min_range_m <= distance <= max_range_m
            and quality >= min_quality
        ):
            points.append(LidarPoint(angle % (2 * math.pi), distance, quality))
    return LidarScan(start, end, sequence, tuple(points))


def parse_scan_line(line: str, **thresholds: float | int) -> LidarScan:
    return normalize_scan(json.loads(line), **thresholds)

