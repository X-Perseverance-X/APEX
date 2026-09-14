"""Bounded local occupancy grid fed by the existing LiDAR owner.

This module never opens a serial device.  The Command Center remains the sole
RPLIDAR owner and passes immutable scan copies here.  Without trusted odometry
the grid is intentionally labelled LOCAL_ONLY; it must not be presented as a
global SLAM map.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .contracts import LidarScan, Pose2D, TofSample


def bresenham(x0: int, y0: int, x1: int, y1: int):
    """Yield integer grid cells on a line, including both endpoints."""
    dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
    dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


@dataclass(frozen=True)
class GridConfig:
    width: int = 241
    height: int = 241
    resolution_m: float = 0.05
    min_range_m: float = 0.12
    max_range_m: float = 6.0
    occupied_delta: float = 0.90
    free_delta: float = -0.24
    occupied_threshold: float = 1.20
    free_threshold: float = -0.65
    log_min: float = -4.0
    log_max: float = 6.0
    temporal_decay: float = 0.82
    forget_epsilon: float = 0.08


class OccupancyGrid:
    def __init__(self, config: GridConfig | None = None):
        self.config = config or GridConfig()
        if self.config.width % 2 == 0 or self.config.height % 2 == 0:
            raise ValueError("grid dimensions must be odd so the robot has a centre cell")
        self.log_odds = np.zeros((self.config.height, self.config.width), dtype=np.float32)
        self.observed = np.zeros_like(self.log_odds, dtype=np.bool_)
        self.scan_count = 0
        self.last_scan_sequence = -1
        self.pose = Pose2D()

    @property
    def origin_x_m(self) -> float:
        return -(self.config.width // 2) * self.config.resolution_m

    @property
    def origin_y_m(self) -> float:
        return -(self.config.height // 2) * self.config.resolution_m

    def world_to_grid(self, x_m: float, y_m: float) -> tuple[int, int]:
        gx = int(round((x_m - self.origin_x_m) / self.config.resolution_m))
        gy = int(round((y_m - self.origin_y_m) / self.config.resolution_m))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return (
            self.origin_x_m + gx * self.config.resolution_m,
            self.origin_y_m + gy * self.config.resolution_m,
        )

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.config.width and 0 <= gy < self.config.height

    def update_lidar(self, scan: LidarScan, pose: Pose2D | None = None) -> bool:
        if scan.sequence == self.last_scan_sequence:
            return False
        pose = pose or self.pose
        self.pose = pose
        start = self.world_to_grid(pose.x_m, pose.y_m)
        if not self.in_bounds(*start):
            return False

        # Güvenilir odometri yokken bu bir dünya haritası değil, robot merkezli
        # canlı costmap'tir. Eski engeller sonsuza kadar ekranda kalmasın;
        # görülmeyen kanıtı kademeli söndür, yeni taramaları baskın tut.
        self.log_odds *= self.config.temporal_decay
        forgotten = np.abs(self.log_odds) < self.config.forget_epsilon
        self.log_odds[forgotten] = 0.0
        self.observed[forgotten] = False

        cos_yaw, sin_yaw = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        for angle_deg, distance_mm in scan.points:
            distance_m = float(distance_mm) / 1000.0
            if not math.isfinite(distance_m) or not self.config.min_range_m <= distance_m <= self.config.max_range_m:
                continue
            angle = math.radians(float(angle_deg))
            lx, ly = distance_m * math.cos(angle), distance_m * math.sin(angle)
            wx = pose.x_m + lx * cos_yaw - ly * sin_yaw
            wy = pose.y_m + lx * sin_yaw + ly * cos_yaw
            end = self.world_to_grid(wx, wy)
            if not self.in_bounds(*end):
                continue
            ray = list(bresenham(start[0], start[1], end[0], end[1]))
            for gx, gy in ray[1:-1]:
                self.log_odds[gy, gx] += self.config.free_delta
                self.observed[gy, gx] = True
            gx, gy = end
            self.log_odds[gy, gx] += self.config.occupied_delta
            self.observed[gy, gx] = True

        np.clip(self.log_odds, self.config.log_min, self.config.log_max, out=self.log_odds)
        self.scan_count += 1
        self.last_scan_sequence = scan.sequence
        return True

    def update_tof(self, sample: TofSample, pose: Pose2D | None = None) -> bool:
        if not sample.valid or not 50.0 <= sample.distance_mm <= 4000.0:
            return False
        pose = pose or self.pose
        distance_m = sample.distance_mm / 1000.0
        wx = pose.x_m + distance_m * math.cos(pose.yaw_rad)
        wy = pose.y_m + distance_m * math.sin(pose.yaw_rad)
        gx, gy = self.world_to_grid(wx, wy)
        if not self.in_bounds(gx, gy):
            return False
        self.log_odds[gy, gx] = min(self.config.log_max, self.log_odds[gy, gx] + 1.4)
        self.observed[gy, gx] = True
        return True

    def states(self) -> np.ndarray:
        states = np.full(self.log_odds.shape, -1, dtype=np.int8)
        states[self.observed & (self.log_odds <= self.config.free_threshold)] = 0
        states[self.observed & (self.log_odds >= self.config.occupied_threshold)] = 100
        return states

    def clear(self) -> None:
        self.log_odds.fill(0.0)
        self.observed.fill(False)
        self.scan_count = 0
        self.last_scan_sequence = -1


def rle_encode(values: np.ndarray) -> list[list[int]]:
    """Compact a flat int grid for low-overhead browser transport."""
    flat = values.reshape(-1)
    if flat.size == 0:
        return []
    encoded: list[list[int]] = []
    last, count = int(flat[0]), 1
    for raw in flat[1:]:
        value = int(raw)
        if value == last:
            count += 1
        else:
            encoded.append([last, count])
            last, count = value, 1
    encoded.append([last, count])
    return encoded
