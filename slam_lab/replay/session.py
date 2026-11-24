"""Replay the exact normalized sensor contracts, with no serial hardware."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Iterator

from drivers.imu_serial.protocol import ImuSample
from drivers.lidar.scan import LidarScan, normalize_scan


def read_imu(path: Path) -> Iterator[ImuSample]:
    with (path / "imu_raw.csv").open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            yield ImuSample(
                int(row["esp_time_us"]), int(row["host_receive_ns"]),
                tuple(float(row[f"a{axis}_m_s2"]) for axis in "xyz"),
                tuple(float(row[f"g{axis}_rad_s"]) for axis in "xyz"),
                tuple(float(row[f"m{axis}_uT"]) for axis in "xyz"),
                float(row["temperature_c"]),
                tuple(json.loads(row["raw_counts_json"])),
                row["source_frame"],
            )


def read_scans(path: Path) -> Iterator[LidarScan]:
    with (path / "lidar_scans.jsonl").open(encoding="utf-8") as source:
        for line in source:
            yield normalize_scan(
                json.loads(line), min_range_m=0.0,
                max_range_m=float("inf"), min_quality=0,
            )


def merged_events(path: Path):
    """Sensor events ordered by host monotonic receive/end time."""
    from heapq import merge

    imu = ((sample.host_receive_ns, "imu", sample) for sample in read_imu(path))
    scans = ((scan.timestamp_end_ns, "lidar", scan) for scan in read_scans(path))
    yield from merge(imu, scans, key=lambda item: item[0])

