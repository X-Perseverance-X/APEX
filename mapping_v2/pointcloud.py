"""SE(3) projection and bounded voxel cloud for calibrated sparse 3D mapping."""

from __future__ import annotations

import math

import numpy as np

from .contracts import LidarScan


def rotation_matrix_rpy(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """Return the REP-103 fixed-axis matrix ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=np.float64)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=np.float64)
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)), dtype=np.float64)
    return rz @ ry @ rx


def project_lidar_scan(
    scan: LidarScan,
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    translation_xyz_m: tuple[float, float, float] = (0.0, 0.0, 0.0),
    min_range_m: float = 0.12,
    max_range_m: float = 8.0,
) -> np.ndarray:
    """Project a planar scan into 3D using a measured sensor pose.

    No height is invented: every source point starts at ``z=0`` in lidar_link.
    Z appears only through the supplied physical rotation/translation.
    """
    points = []
    for angle_deg, distance_mm in scan.points:
        distance_m = float(distance_mm) / 1000.0
        if not math.isfinite(distance_m) or not min_range_m <= distance_m <= max_range_m:
            continue
        angle = math.radians(float(angle_deg))
        points.append((distance_m * math.cos(angle), distance_m * math.sin(angle), 0.0))
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    xyz = np.asarray(points, dtype=np.float64)
    rotation = rotation_matrix_rpy(roll_rad, pitch_rad, yaw_rad)
    translation = np.asarray(translation_xyz_m, dtype=np.float64)
    return (rotation @ xyz.T).T + translation


class VoxelCloud:
    def __init__(self, voxel_size_m: float = 0.05, max_points: int = 50000):
        if voxel_size_m <= 0 or max_points <= 0:
            raise ValueError("voxel size and max_points must be positive")
        self.voxel_size_m = float(voxel_size_m)
        self.max_points = int(max_points)
        self._voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}

    def integrate(self, xyz: np.ndarray) -> None:
        for point in np.asarray(xyz, dtype=np.float64):
            if point.shape != (3,) or not np.isfinite(point).all():
                continue
            key = tuple(int(round(float(v) / self.voxel_size_m)) for v in point)
            self._voxels[key] = (float(point[0]), float(point[1]), float(point[2]))
        while len(self._voxels) > self.max_points:
            self._voxels.pop(next(iter(self._voxels)))

    def points(self) -> list[tuple[float, float, float]]:
        return list(self._voxels.values())

    def clear(self) -> None:
        self._voxels.clear()

    def __len__(self) -> int:
        return len(self._voxels)
