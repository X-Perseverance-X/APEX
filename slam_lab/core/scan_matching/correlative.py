"""Small local search over x/y/yaw against the previous scan.

This is deliberately readable and bounded. The score is the fraction of
transformed current points landing near occupied cells from the previous scan.
Low-confidence results do not update translation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from core.frames.geometry import Pose2
from drivers.lidar.scan import LidarScan


@dataclass(frozen=True)
class MatchResult:
    delta: Pose2
    score: float
    accepted: bool
    points_used: int
    gain_over_static: float = 0.0


class CorrelativeMatcher:
    def __init__(
        self,
        cell_m: float = 0.10,
        max_translation_m: float = 0.20,
        translation_step_m: float = 0.05,
        yaw_window_deg: float = 8.0,
        yaw_step_deg: float = 2.0,
        min_score: float = 0.25,
        min_motion_gain: float = 0.04,
        downsample: int = 8,
    ) -> None:
        self.cell_m = cell_m
        self.max_translation_m = max_translation_m
        self.translation_step_m = translation_step_m
        self.yaw_window_rad = math.radians(yaw_window_deg)
        self.yaw_step_rad = math.radians(yaw_step_deg)
        self.min_score = min_score
        self.min_motion_gain = min_motion_gain
        self.downsample = max(1, downsample)

    def _cell(self, x: float, y: float) -> tuple[int, int]:
        return round(x / self.cell_m), round(y / self.cell_m)

    def match(self, previous: LidarScan, current: LidarScan, yaw_guess_rad: float) -> MatchResult:
        reference = {self._cell(*point.xy_m()) for point in previous.points[::self.downsample]}
        samples = [point.xy_m() for point in current.points[::self.downsample]]
        if len(reference) < 10 or len(samples) < 10:
            return MatchResult(Pose2(), 0.0, False, len(samples))

        # Build the one-cell tolerance mask once, not five hash probes for
        # every point in every x/y/yaw candidate. This matters on a Pi 4:
        # the original inner loop cost ~290 ms/scan before map integration.
        nearby = {
            (cx + ox, cy + oy)
            for cx, cy in reference
            for ox, oy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))
        }
        static_score = sum(self._cell(x, y) in nearby for x, y in samples) / len(samples)

        best_score = -1.0
        best_rank = -float("inf")
        best = Pose2()
        best_offset = (0.0, 0.0, 0.0)
        tested: set[tuple[float, float, float]] = set()

        def search(yaw_candidates: list[float], xy_candidates: list[tuple[float, float]]) -> None:
            nonlocal best_score, best_rank, best, best_offset
            for yaw_offset in yaw_candidates:
                yaw = yaw_guess_rad + yaw_offset
                cosine, sine = math.cos(yaw), math.sin(yaw)
                rotated = [(cosine*x - sine*y, sine*x + cosine*y) for x, y in samples]
                for dx, dy in xy_candidates:
                    key = (round(yaw_offset, 8), round(dx, 8), round(dy, 8))
                    if key in tested:
                        continue
                    tested.add(key)
                    hits = 0
                    for x, y in rotated:
                        cx, cy = self._cell(x + dx, y + dy)
                        if (cx, cy) in nearby:
                            hits += 1
                    score = hits / len(samples)
                    # Broad cells can produce score plateaus. Prefer the
                    # smallest motion consistent with the observations.
                    translation_cost = math.hypot(dx, dy) / max(self.max_translation_m, self.cell_m)
                    yaw_cost = abs(yaw_offset) / max(self.yaw_window_rad, self.yaw_step_rad)
                    rank = score - 0.015 * (translation_cost + yaw_cost)
                    if rank > best_rank:
                        best_rank = rank
                        best_score = score
                        best = Pose2(dx, dy, yaw)
                        best_offset = (yaw_offset, dx, dy)

        # Stage 1 covers the full configured range at twice the final step.
        # Stage 2 restores the configured resolution only around its winner.
        # On the Pi this is ~150 candidate poses, instead of 729.
        coarse_xy = self._offsets(self.max_translation_m, 2*self.translation_step_m)
        coarse_yaw = self._offsets(self.yaw_window_rad, 2*self.yaw_step_rad)
        search(coarse_yaw, [(dx, dy) for dx in coarse_xy for dy in coarse_xy])
        yaw0, dx0, dy0 = best_offset
        fine_yaw = [yaw0 + i*self.yaw_step_rad for i in (-1, 0, 1)
                    if abs(yaw0 + i*self.yaw_step_rad) <= self.yaw_window_rad + 1e-9]
        fine_xy = [(dx0 + i*self.translation_step_m, dy0 + j*self.translation_step_m)
                   for i in (-1, 0, 1) for j in (-1, 0, 1)
                   if abs(dx0 + i*self.translation_step_m) <= self.max_translation_m + 1e-9
                   and abs(dy0 + j*self.translation_step_m) <= self.max_translation_m + 1e-9]
        search(fine_yaw, fine_xy)
        gain = best_score - static_score
        if best != Pose2() and gain < self.min_motion_gain:
            # One or two accidental point matches on a stationary scanner
            # must not accumulate into centimetres of phantom trajectory.
            return MatchResult(Pose2(), static_score,
                               static_score >= self.min_score, len(samples), gain)
        return MatchResult(best, best_score, best_score >= self.min_score,
                           len(samples), gain)

    @staticmethod
    def _offsets(limit: float, step: float) -> list[float]:
        if step <= 0 or limit < 0:
            raise ValueError("invalid search extent")
        count = math.floor(limit / step + 1e-9)
        return [index * step for index in range(-count, count + 1)]
