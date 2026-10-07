#!/usr/bin/env python3
"""Time mapping stages from recorded scans without touching live sensors."""

from __future__ import annotations

import argparse
import itertools
import sys
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.frames.geometry import Pose2
from core.mapping.occupancy import OccupancyGrid
from core.scan_matching.correlative import CorrelativeMatcher
from replay.session import read_scans


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--scans", type=int, default=5)
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args()
    settings = yaml.safe_load((args.session / "hardware_snapshot.yaml").read_text())["mapping"]
    matcher = CorrelativeMatcher(
        cell_m=settings["scan_match_cell_m"],
        max_translation_m=settings["scan_match_max_translation_m"],
        translation_step_m=settings["scan_match_translation_step_m"],
        yaw_window_deg=settings["scan_match_yaw_window_deg"],
        yaw_step_deg=settings["scan_match_yaw_step_deg"],
        min_score=settings["scan_match_min_score"],
        min_motion_gain=settings.get("scan_match_min_motion_gain", 0.04),
        downsample=settings["scan_match_downsample"],
    )
    grid = OccupancyGrid(settings["resolution_m"])
    previous = None
    selected = itertools.islice(read_scans(args.session), 0, args.scans * args.stride, args.stride)
    for index, scan in enumerate(selected, start=1):
        t0 = time.perf_counter()
        result = matcher.match(previous, scan, 0.0) if previous else None
        zero_score = None
        if previous:
            reference = {matcher._cell(*point.xy_m()) for point in previous.points[::matcher.downsample]}
            nearby = {(cx+ox, cy+oy) for cx, cy in reference
                      for ox, oy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))}
            samples = [point.xy_m() for point in scan.points[::matcher.downsample]]
            zero_score = sum(matcher._cell(x, y) in nearby for x, y in samples) / len(samples)
        t1 = time.perf_counter()
        grid.integrate(scan, Pose2())
        t2 = time.perf_counter()
        cells = grid.compact()
        t3 = time.perf_counter()
        print(f"scan={index} points={len(scan.points)} map_cells={len(cells)} "
              f"match_ms={(t1-t0)*1000:.1f} integrate_ms={(t2-t1)*1000:.1f} "
              f"compact_ms={(t3-t2)*1000:.1f} "
              f"score={result.score if result else None} zero={zero_score} "
              f"delta={result.delta if result else None}")
        previous = scan


if __name__ == "__main__":
    main()
