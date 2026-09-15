"""Conservative LiDAR outlier rejection for glass/mirror multipath returns."""

from __future__ import annotations

import math

import numpy as np

from .contracts import LidarScan


class LidarOutlierFilter:
    """Reject isolated spatial spikes and one-frame temporal range jumps.

    It cannot identify every specular surface without intensity data, but it
    prevents a single reflected ray from becoming map geometry or influencing
    scan matching. Persistent changes are admitted gradually so doors and
    moved objects can still enter the map.
    """

    def __init__(self, bins: int = 720, persistence: int = 4):
        self.bins = int(bins)
        self.persistence = int(persistence)
        self._stable = np.full(self.bins, np.nan, dtype=np.float32)
        self._pending = np.full(self.bins, np.nan, dtype=np.float32)
        self._votes = np.zeros(self.bins, dtype=np.uint8)
        self._scans_seen = 0
        self.accepted_points = 0
        self.rejected_points = 0

    def clear(self) -> None:
        self._stable.fill(np.nan)
        self._pending.fill(np.nan)
        self._votes.fill(0)
        self._scans_seen = 0
        self.accepted_points = 0
        self.rejected_points = 0

    def filter(self, scan: LidarScan, yaw_delta_deg: float | None = None) -> LidarScan:
        raw = sorted(
            (float(angle) % 360.0, float(distance))
            for angle, distance in scan.points
            if math.isfinite(float(angle)) and math.isfinite(float(distance)) and 120.0 <= float(distance) <= 6000.0
        )
        if not raw:
            return LidarScan(scan.timestamp_ns, scan.sequence, (), scan.frame_id)
        distances = np.asarray([distance / 1000.0 for _, distance in raw], dtype=np.float32)
        angles = np.asarray([angle for angle, _ in raw], dtype=np.float32)
        rotating_fast = yaw_delta_deg is not None and abs(float(yaw_delta_deg)) > 2.5
        kept: list[tuple[float, float]] = []
        for index, (angle, distance_mm) in enumerate(raw):
            distance_m = float(distance_mm) / 1000.0
            neighbour_distances: list[float] = []
            for offset in (-2, -1, 1, 2):
                neighbour = (index + offset) % len(raw)
                if neighbour == index:
                    continue
                angle_gap = abs(((float(angles[neighbour]) - angle + 540.0) % 360.0) - 180.0)
                if angle_gap <= 4.0:
                    neighbour_distances.append(float(distances[neighbour]))
            spatial_spike = False
            if len(neighbour_distances) >= 2:
                median = float(np.median(neighbour_distances))
                close_neighbours = sum(abs(value - median) <= max(0.18, median * 0.12) for value in neighbour_distances)
                spatial_spike = close_neighbours >= 2 and distance_m > median + max(0.65, median * 0.35)
            # During cold start there is no temporal baseline yet. Do not let
            # a sparse, long multipath return seed the map until a few complete
            # scans have established one. Dense far walls remain admissible.
            bootstrap_far_unverified = (
                self._scans_seen < self.persistence
                and distance_m > 4.0
                and len(neighbour_distances) < 2
            )

            beam = int(round(angle / 360.0 * self.bins)) % self.bins
            previous = float(self._stable[beam])
            temporal_jump = False
            if not rotating_fast and math.isfinite(previous):
                threshold = max(0.55, previous * 0.38)
                temporal_jump = abs(distance_m - previous) > threshold
                if temporal_jump:
                    pending = float(self._pending[beam])
                    if math.isfinite(pending) and abs(distance_m - pending) <= max(0.20, distance_m * 0.10):
                        self._votes[beam] = min(255, int(self._votes[beam]) + 1)
                    else:
                        self._pending[beam] = distance_m
                        self._votes[beam] = 1
                    if int(self._votes[beam]) >= self.persistence:
                        temporal_jump = False
                        self._stable[beam] = distance_m
                        self._pending[beam] = np.nan
                        self._votes[beam] = 0

            if spatial_spike or temporal_jump or bootstrap_far_unverified:
                self.rejected_points += 1
                continue
            if not math.isfinite(previous) or rotating_fast:
                self._stable[beam] = distance_m
            else:
                self._stable[beam] = 0.75 * previous + 0.25 * distance_m
            self._pending[beam] = np.nan
            self._votes[beam] = 0
            self.accepted_points += 1
            kept.append((angle, distance_mm))
        self._scans_seen += 1
        return LidarScan(scan.timestamp_ns, scan.sequence, tuple(kept), scan.frame_id)

    def status(self) -> dict:
        total = self.accepted_points + self.rejected_points
        return {
            "accepted_points": self.accepted_points,
            "rejected_points": self.rejected_points,
            "rejected_ratio": round(self.rejected_points / total, 3) if total else 0.0,
        }
