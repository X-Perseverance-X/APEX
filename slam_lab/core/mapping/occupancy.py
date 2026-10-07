"""Sparse, expanding 2D log-odds occupancy grid with Bresenham rays."""

from __future__ import annotations

import math

from core.frames.geometry import Pose2
from drivers.lidar.scan import LidarScan


def bresenham(start: tuple[int, int], end: tuple[int, int]):
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        twice = 2 * err
        if twice > -dy:
            err -= dy
            x0 += sx
        if twice < dx:
            err += dx
            y0 += sy


class OccupancyGrid:
    def __init__(
        self,
        resolution_m: float = 0.05,
        free_delta: float = -0.4,
        occupied_delta: float = 0.85,
        min_log_odds: float = -4.0,
        max_log_odds: float = 4.0,
    ) -> None:
        if resolution_m <= 0:
            raise ValueError("resolution must be positive")
        self.resolution_m = resolution_m
        self.free_delta = free_delta
        self.occupied_delta = occupied_delta
        self.min_log_odds = min_log_odds
        self.max_log_odds = max_log_odds
        self.cells: dict[tuple[int, int], float] = {}

    def cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        return math.floor(x_m / self.resolution_m), math.floor(y_m / self.resolution_m)

    def probability(self, cell: tuple[int, int]) -> float | None:
        value = self.cells.get(cell)
        return None if value is None else 1.0 / (1.0 + math.exp(-value))

    def integrate(self, scan: LidarScan, pose: Pose2) -> None:
        origin = self.cell(pose.x_m, pose.y_m)
        for point in scan.points:
            local_x, local_y = point.xy_m()
            world_x, world_y = pose.transform_xy(local_x, local_y)
            ray = list(bresenham(origin, self.cell(world_x, world_y)))
            for cell in ray[:-1]:
                old = self.cells.get(cell, 0.0)
                self.cells[cell] = max(self.min_log_odds, old + self.free_delta)
            if ray:
                hit = ray[-1]
                old = self.cells.get(hit, 0.0)
                self.cells[hit] = min(self.max_log_odds, old + self.occupied_delta)

    def compact(self, max_cells: int = 20000) -> list[list[float]]:
        """For UI transfer: [cell_x, cell_y, probability]."""
        if max_cells <= 0:
            return []
        keys = list(self.cells)
        stride = max(1, math.ceil(len(keys) / max_cells))
        return [
            [x, y, round(self.probability((x, y)) or 0.5, 3)]
            for x, y in keys[::stride][:max_cells]
        ]
