"""Monocular riser measurement from a visible vertical face and its floor seam."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Riser:
    range_m: float
    height_m: float
    top_row: int
    base_row: int
    contrast: float


@dataclass(frozen=True)
class CameraCalibration:
    height_m: float
    pitch_rad: float
    vertical_fov_rad: float

    def ray_slope(self, row: int, rows: int) -> float:
        focal = rows / (2 * math.tan(self.vertical_fov_rad / 2))
        return math.tan(self.pitch_rad + math.atan((row + 0.5 - rows / 2) / focal))


def estimate_riser(rgb: np.ndarray, calibration: CameraCalibration) -> Riser | None:
    """Estimate a head-on upward riser without depth, pose or map inputs."""
    rows, cols, _ = rgb.shape
    width = max(4, cols // 15)
    centre = cols // 2
    roi = rgb[:, centre - width : centre + width].astype(np.float32)
    luma = roi @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    profile = np.median(luma, axis=1)
    # Compare a few pixels on either side of each candidate line to tolerate
    # raster aliasing and small roll without learning the scene's colours.
    smoothed = np.convolve(profile, np.ones(3) / 3, mode="same")
    derivative = smoothed[4:] - smoothed[:-4]
    peaks = []
    for raw in np.argsort(np.abs(derivative))[::-1]:
        row = int(raw) + 2
        strength = float(derivative[raw])
        if abs(strength) < 11:
            break
        if 7 <= row < rows - 7 and all(abs(row - other) >= 6 for other, _ in peaks):
            peaks.append((row, strength))
        if len(peaks) >= 18:
            break
    best = None
    best_score = 0.0
    for base, change in peaks:
        if change <= 0:
            continue
        bottom_slope = calibration.ray_slope(base, rows)
        if bottom_slope < 0.12:
            continue
        distance = calibration.height_m / bottom_slope
        if not 0.11 < distance < 1.05:
            continue
        for top, upper_change in peaks:
            if upper_change >= 0 or base - top < 8:
                continue
            h = calibration.height_m - distance * calibration.ray_slope(top, rows)
            if not 0.025 <= h <= 0.24:
                continue
            face = float(np.median(profile[top + 3 : base - 2]))
            upper = float(np.median(profile[top - 6 : top - 1]))
            lower = float(np.median(profile[base + 2 : base + 7]))
            contrast = min(upper - face, lower - face, -upper_change, change)
            if contrast < 12:
                continue
            score = contrast - 30 * distance
            if score > best_score:
                best_score = score
                best = Riser(distance, h, top, base, contrast)
    return best
