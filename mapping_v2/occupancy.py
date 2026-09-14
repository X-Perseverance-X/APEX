"""Bounded local occupancy grid fed by the existing LiDAR owner.

This module never opens a serial device.  The Command Center remains the sole
RPLIDAR owner and passes immutable scan copies here. Pose estimation is supplied
by the local scan matcher; this bounded active submap is intentionally not
presented as a globally loop-closed map.
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
    free_delta: float = -0.55
    occupied_threshold: float = 0.70
    free_threshold: float = -0.50
    log_min: float = -4.0
    log_max: float = 6.0
    forget_epsilon: float = 0.08
    free_ray_width_cells: int = 1
    stale_grace_s: float = 1.5
    stale_half_life_s: float = 4.0
    confirmation_hits: int = 4
    free_hit_count_decrement: int = 2
    confirmed_clear_misses: int = 3


class OccupancyGrid:
    def __init__(self, config: GridConfig | None = None):
        self.config = config or GridConfig()
        if self.config.width % 2 == 0 or self.config.height % 2 == 0:
            raise ValueError("grid dimensions must be odd so the robot has a centre cell")
        self.log_odds = np.zeros((self.config.height, self.config.width), dtype=np.float32)
        self.observed = np.zeros_like(self.log_odds, dtype=np.bool_)
        self.last_hit_ns = np.zeros_like(self.log_odds, dtype=np.int64)
        self.hit_count = np.zeros_like(self.log_odds, dtype=np.uint16)
        self.miss_streak = np.zeros_like(self.log_odds, dtype=np.uint8)
        self.scan_count = 0
        self.last_scan_sequence = -1
        self._last_decay_ns = 0
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

        self._decay_stale_obstacles(scan.timestamp_ns)

        cos_yaw, sin_yaw = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        hit_cells: set[tuple[int, int]] = set()
        miss_cells: set[tuple[int, int]] = set()
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
                width = self.config.free_ray_width_cells
                for oy in range(-width, width + 1):
                    for ox in range(-width, width + 1):
                        cell = (gx + ox, gy + oy)
                        if self.in_bounds(*cell):
                            miss_cells.add(cell)
            hit_cells.add(end)

        # Cartographer's update-marker principle: a cell is changed at most
        # once per scan, and a measured return wins over free-space carving.
        miss_cells.difference_update(hit_cells)
        if miss_cells:
            mx, my = zip(*miss_cells)
            miss_x, miss_y = np.asarray(mx), np.asarray(my)
            self.log_odds[miss_y, miss_x] += self.config.free_delta
            self.observed[miss_y, miss_x] = True
            counts = self.hit_count[miss_y, miss_x].astype(np.int32)
            was_confirmed = counts >= self.config.confirmation_hits
            streaks = self.miss_streak[miss_y, miss_x].astype(np.int16)
            streaks[was_confirmed] += 1
            clear_confirmed = was_confirmed & (streaks >= self.config.confirmed_clear_misses)
            counts[clear_confirmed] = 0
            streaks[clear_confirmed] = 0
            counts[~was_confirmed] -= self.config.free_hit_count_decrement
            self.hit_count[miss_y, miss_x] = np.maximum(0, counts).astype(np.uint16)
            self.miss_streak[miss_y, miss_x] = np.minimum(255, streaks).astype(np.uint8)
        if hit_cells:
            hx, hy = zip(*hit_cells)
            hit_x, hit_y = np.asarray(hx), np.asarray(hy)
            self.log_odds[hit_y, hit_x] += self.config.occupied_delta
            self.observed[hit_y, hit_x] = True
            self.last_hit_ns[hit_y, hit_x] = int(scan.timestamp_ns)
            counts = self.hit_count[hit_y, hit_x].astype(np.uint32) + 1
            self.hit_count[hit_y, hit_x] = np.minimum(np.iinfo(np.uint16).max, counts).astype(np.uint16)
            self.miss_streak[hit_y, hit_x] = 0

        np.clip(self.log_odds, self.config.log_min, self.config.log_max, out=self.log_odds)
        self.scan_count += 1
        self.last_scan_sequence = scan.sequence
        return True

    def _decay_stale_obstacles(self, timestamp_ns: int) -> None:
        """Use elapsed time, not scan count, to forget unconfirmed hit noise."""
        timestamp_ns = int(timestamp_ns)
        if timestamp_ns <= 0:
            return
        if self._last_decay_ns <= 0:
            self._last_decay_ns = timestamp_ns
            return
        elapsed_s = max(0.0, (timestamp_ns - self._last_decay_ns) / 1e9)
        self._last_decay_ns = max(self._last_decay_ns, timestamp_ns)
        if elapsed_s <= 0.0:
            return
        age_s = (timestamp_ns - self.last_hit_ns) / 1e9
        provisional = self.hit_count < self.config.confirmation_hits
        stale = provisional & (self.last_hit_ns > 0) & (age_s >= self.config.stale_grace_s) & (self.log_odds > 0.0)
        if np.any(stale):
            factor = math.exp(-math.log(2.0) * elapsed_s / max(0.1, self.config.stale_half_life_s))
            self.log_odds[stale] *= factor
        forgotten = self.observed & (np.abs(self.log_odds) < self.config.forget_epsilon)
        self.log_odds[forgotten] = 0.0
        self.observed[forgotten] = False
        self.last_hit_ns[forgotten] = 0
        self.hit_count[forgotten] = 0
        self.miss_streak[forgotten] = 0

    def update_tof(self, sample: TofSample, pose: Pose2D | None = None) -> bool:
        if not sample.valid or not 50.0 <= sample.distance_mm <= 4000.0:
            return False
        pose = pose or self.pose
        distance_m = sample.distance_mm / 1000.0
        wx = pose.x_m + distance_m * math.cos(pose.yaw_rad)
        wy = pose.y_m + distance_m * math.sin(pose.yaw_rad)
        end = self.world_to_grid(wx, wy)
        start = self.world_to_grid(pose.x_m, pose.y_m)
        if not self.in_bounds(*end) or not self.in_bounds(*start):
            return False
        ray = list(bresenham(start[0], start[1], end[0], end[1]))
        for gx, gy in ray[1:-1]:
            # The single forward ToF beam is useful corroboration but must not
            # erase a same-cycle LiDAR hit. LiDAR owns occupied-cell clearing.
            if self.log_odds[gy, gx] < self.config.occupied_threshold:
                self.log_odds[gy, gx] = max(self.config.log_min, self.log_odds[gy, gx] + self.config.free_delta)
            self.observed[gy, gx] = True
        gx, gy = end
        self.log_odds[gy, gx] = min(self.config.log_max, self.log_odds[gy, gx] + 1.4)
        self.observed[gy, gx] = True
        self.last_hit_ns[gy, gx] = int(sample.timestamp_ns)
        return True

    def confirmed_mask(self) -> np.ndarray:
        return (
            self.observed
            & (self.log_odds >= self.config.occupied_threshold)
            & (self.hit_count >= self.config.confirmation_hits)
        )

    def states(self) -> np.ndarray:
        states = np.full(self.log_odds.shape, -1, dtype=np.int8)
        states[self.observed & (self.log_odds <= self.config.free_threshold)] = 0
        states[self.observed & (self.log_odds >= self.config.occupied_threshold)] = 100
        return states

    def clear(self) -> None:
        self.log_odds.fill(0.0)
        self.observed.fill(False)
        self.last_hit_ns.fill(0)
        self.hit_count.fill(0)
        self.miss_streak.fill(0)
        self.scan_count = 0
        self.last_scan_sequence = -1
        self._last_decay_ns = 0


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
