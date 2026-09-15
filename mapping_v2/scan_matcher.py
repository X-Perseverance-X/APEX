"""Bounded correlative scan matching for the Raspberry Pi local mapper.

This is deliberately a small local-SLAM component, not a claim of full Google
Cartographer compatibility.  It follows the same important ordering: predict a
pose, match the new range data against the active probability grid, reject weak
matches, and only then insert the scan.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .contracts import LidarScan, Pose2D
from .occupancy import OccupancyGrid


def normalize_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class ScanMatcherConfig:
    min_scan_points: int = 35
    min_map_cells: int = 24
    max_scan_points: int = 120
    translation_window_m: float = 0.10
    translation_step_m: float = 0.05
    rotation_window_rad: float = math.radians(8.0)
    rotation_step_rad: float = math.radians(2.0)
    fine_translation_window_m: float = 0.02
    fine_translation_step_m: float = 0.01
    fine_rotation_window_rad: float = math.radians(1.5)
    fine_rotation_step_rad: float = math.radians(0.75)
    min_score: float = 0.24
    min_in_bounds_ratio: float = 0.55


@dataclass(frozen=True)
class MatchResult:
    accepted: bool
    pose: Pose2D
    score: float
    in_bounds_ratio: float
    points_used: int
    imu_sign: int = 0
    reason: str = ""


class CorrelativeScanMatcher:
    """Coarse-to-fine endpoint correlation against an occupancy likelihood."""

    def __init__(self, config: ScanMatcherConfig | None = None):
        self.config = config or ScanMatcherConfig()

    @staticmethod
    def _scan_xy(scan: LidarScan, grid: OccupancyGrid, limit: int) -> np.ndarray:
        usable: list[tuple[float, float]] = []
        for angle_deg, distance_mm in scan.points:
            distance_m = float(distance_mm) / 1000.0
            if not math.isfinite(distance_m) or not grid.config.min_range_m <= distance_m <= grid.config.max_range_m:
                continue
            angle = math.radians(float(angle_deg))
            usable.append((distance_m * math.cos(angle), distance_m * math.sin(angle)))
        if len(usable) > limit:
            indices = np.linspace(0, len(usable) - 1, limit, dtype=np.int32)
            usable = [usable[int(index)] for index in indices]
        return np.asarray(usable, dtype=np.float32).reshape((-1, 2))

    @staticmethod
    def _likelihood(grid: OccupancyGrid) -> np.ndarray:
        occupied = grid.log_odds >= grid.config.occupied_threshold
        confirmed = grid.confirmed_mask()
        provisional = occupied & ~confirmed
        # Mature walls become the registration anchor. New red measurements
        # still contribute, but cannot pull the robot away from repeatedly
        # verified geometry with the same strength.
        field = provisional.astype(np.float32) * 0.55
        field[confirmed] = 1.0
        # A tiny distance-like field makes sparse A1M8 endpoints match walls
        # without scipy and without allowing distant geometry to look correct.
        for seeds, values in ((occupied, (0.38, 0.22, 0.10)), (confirmed, (0.78, 0.48, 0.24))):
            expanded = seeds.copy()
            for value in values:
                previous = expanded
                padded = np.pad(previous, 1, mode="constant", constant_values=False)
                neighbours = [
                    padded[1 + dy:1 + dy + previous.shape[0], 1 + dx:1 + dx + previous.shape[1]]
                    for dy in (-1, 0, 1)
                    for dx in (-1, 0, 1)
                ]
                expanded = np.logical_or.reduce(neighbours)
                ring = expanded & ~previous
                field[ring] = np.maximum(field[ring], value)
        return field

    @staticmethod
    def _axis_values(window: float, step: float) -> np.ndarray:
        count = max(1, int(math.ceil(window / step)))
        return np.arange(-count, count + 1, dtype=np.float32) * float(step)

    def _score_pose(
        self,
        points: np.ndarray,
        field: np.ndarray,
        grid: OccupancyGrid,
        x_m: float,
        y_m: float,
        yaw_rad: float,
    ) -> tuple[float, float]:
        cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
        wx = points[:, 0] * cosine - points[:, 1] * sine + x_m
        wy = points[:, 0] * sine + points[:, 1] * cosine + y_m
        gx = np.rint((wx - grid.origin_x_m) / grid.config.resolution_m).astype(np.int32)
        gy = np.rint((wy - grid.origin_y_m) / grid.config.resolution_m).astype(np.int32)
        valid = (gx >= 0) & (gx < grid.config.width) & (gy >= 0) & (gy < grid.config.height)
        ratio = float(np.count_nonzero(valid)) / max(1, len(points))
        if not np.any(valid):
            return 0.0, ratio
        return float(np.mean(field[gy[valid], gx[valid]])) * ratio, ratio

    def _search(
        self,
        points: np.ndarray,
        field: np.ndarray,
        grid: OccupancyGrid,
        centre: tuple[float, float, float],
        translation_window: float,
        translation_step: float,
        rotation_window: float,
        rotation_step: float,
    ) -> tuple[float, float, float, float, float]:
        best = (centre[0], centre[1], centre[2], -1.0, 0.0)
        translations = self._axis_values(translation_window, translation_step)
        rotations = self._axis_values(rotation_window, rotation_step)
        for dy in translations:
            for dx in translations:
                for dyaw in rotations:
                    yaw = normalize_angle(centre[2] + float(dyaw))
                    score, ratio = self._score_pose(
                        points, field, grid,
                        centre[0] + float(dx), centre[1] + float(dy), yaw,
                    )
                    # Prefer the prediction when geometrically equivalent. This
                    # suppresses pose chatter on long, feature-poor walls.
                    displacement = math.hypot(float(dx), float(dy)) / max(translation_window, 1e-6)
                    rotation = abs(float(dyaw)) / max(rotation_window, 1e-6)
                    ranked = score - 0.025 * displacement - 0.015 * rotation
                    if ranked > best[3]:
                        best = (centre[0] + float(dx), centre[1] + float(dy), yaw, ranked, ratio)
        raw_score, ratio = self._score_pose(points, field, grid, best[0], best[1], best[2])
        return best[0], best[1], best[2], raw_score, ratio

    def match(
        self,
        scan: LidarScan,
        grid: OccupancyGrid,
        previous: Pose2D,
        imu_yaw_delta_rad: float | None = None,
        imu_sign: int | None = None,
    ) -> MatchResult:
        points = self._scan_xy(scan, grid, self.config.max_scan_points)
        occupied_count = int(np.count_nonzero(grid.log_odds >= grid.config.occupied_threshold))
        if len(points) < self.config.min_scan_points:
            return MatchResult(False, previous, 0.0, 0.0, len(points), reason="INSUFFICIENT_SCAN_POINTS")
        if occupied_count < self.config.min_map_cells:
            return MatchResult(False, previous, 0.0, 0.0, len(points), reason="INITIALIZING_SUBMAP")

        field = self._likelihood(grid)
        seeds: list[tuple[int, float]] = [(0, previous.yaw_rad)]
        if imu_yaw_delta_rad is not None and math.isfinite(imu_yaw_delta_rad):
            if imu_sign in (-1, 1):
                seeds = [(imu_sign, normalize_angle(previous.yaw_rad + imu_sign * imu_yaw_delta_rad))]
            elif abs(imu_yaw_delta_rad) >= math.radians(0.5):
                seeds = [
                    (1, normalize_angle(previous.yaw_rad + imu_yaw_delta_rad)),
                    (-1, normalize_angle(previous.yaw_rad - imu_yaw_delta_rad)),
                ]

        best: tuple[float, float, float, float, float, int] | None = None
        for sign, yaw_seed in seeds:
            coarse = self._search(
                points, field, grid, (previous.x_m, previous.y_m, yaw_seed),
                self.config.translation_window_m, self.config.translation_step_m,
                self.config.rotation_window_rad, self.config.rotation_step_rad,
            )
            fine = self._search(
                points, field, grid, coarse[:3],
                self.config.fine_translation_window_m, self.config.fine_translation_step_m,
                self.config.fine_rotation_window_rad, self.config.fine_rotation_step_rad,
            )
            candidate = (*fine, sign)
            if best is None or candidate[3] > best[3]:
                best = candidate

        assert best is not None
        x_m, y_m, yaw_rad, score, in_bounds_ratio, selected_sign = best
        accepted = score >= self.config.min_score and in_bounds_ratio >= self.config.min_in_bounds_ratio
        pose = Pose2D(
            x_m=round(x_m, 5), y_m=round(y_m, 5), yaw_rad=normalize_angle(yaw_rad),
            trusted=accepted, source="lidar_imu_correlative" if accepted else previous.source,
        ) if accepted else previous
        reason = "MATCHED" if accepted else "LOW_CORRELATION"
        return MatchResult(accepted, pose, score, in_bounds_ratio, len(points), selected_sign, reason)
