"""Geometry in metres and radians. Rotations are counterclockwise about +z."""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def polar_xy(angle_rad: float, range_m: float) -> tuple[float, float]:
    return range_m * math.cos(angle_rad), range_m * math.sin(angle_rad)


@dataclass(frozen=True)
class Pose2:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def transform_xy(self, x_m: float, y_m: float) -> tuple[float, float]:
        c, s = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return self.x_m + c * x_m - s * y_m, self.y_m + s * x_m + c * y_m

    def moved(self, dx_m: float, dy_m: float, dyaw_rad: float) -> "Pose2":
        x_m, y_m = self.transform_xy(dx_m, dy_m)
        return Pose2(x_m, y_m, wrap_angle(self.yaw_rad + dyaw_rad))


@dataclass(frozen=True)
class Quaternion:
    """Hamilton quaternion in w,x,y,z order; body to world rotation."""

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def normalized(self) -> "Quaternion":
        magnitude = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if magnitude < 1e-12:
            raise ValueError("zero quaternion")
        return Quaternion(*(value / magnitude for value in (self.w, self.x, self.y, self.z)))

    def __mul__(self, other: "Quaternion") -> "Quaternion":
        a, b = self, other
        return Quaternion(
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        )

    def rotate(self, vector: tuple[float, float, float]) -> tuple[float, float, float]:
        q = self.normalized()
        v = Quaternion(0.0, *vector)
        out = q * v * Quaternion(q.w, -q.x, -q.y, -q.z)
        return out.x, out.y, out.z

    def interpolated(self, other: "Quaternion", fraction: float) -> "Quaternion":
        """Normalized interpolation along the short arc between nearby attitudes."""
        if not 0.0 <= fraction <= 1.0:
            raise ValueError("fraction must be within [0, 1]")
        first, second = self.normalized(), other.normalized()
        a = (first.w, first.x, first.y, first.z)
        b = (second.w, second.x, second.y, second.z)
        if sum(x * y for x, y in zip(a, b)) < 0.0:
            b = tuple(-value for value in b)
        return Quaternion(*(x + fraction * (y - x) for x, y in zip(a, b))).normalized()

    def euler_rad(self) -> tuple[float, float, float]:
        q = self.normalized()
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        return roll, pitch, yaw
