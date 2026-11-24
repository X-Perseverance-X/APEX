"""Parse IMU bridge v1/v2 SI packets without assuming sensor axes."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ImuSample:
    esp_time_us: int
    host_receive_ns: int
    accel_raw_frame_m_s2: tuple[float, float, float]
    gyro_raw_frame_rad_s: tuple[float, float, float]
    mag_raw_frame_uT: tuple[float, float, float]
    temperature_c: float
    raw_counts: tuple[int, ...]
    source_frame: str = "imu_sensor_raw"

    @property
    def accel_norm_m_s2(self) -> float:
        return math.sqrt(sum(value * value for value in self.accel_raw_frame_m_s2))


def _finite_triplet(values: list[str], label: str, *, allow_nan: bool = False) -> tuple[float, float, float]:
    output = tuple(float(value) for value in values)
    if len(output) != 3 or any(math.isinf(value) or (math.isnan(value) and not allow_nan) for value in output):
        raise ValueError(f"invalid {label}")
    return output  # type: ignore[return-value]


def parse_imu_line(line: str, host_receive_ns: int) -> ImuSample:
    parts = line.strip().split(",")
    if len(parts) not in (12, 21) or parts[0] != "IMU":
        raise ValueError("not an IMU packet or wrong field count")
    esp_time_us = int(parts[1])
    if not 0 <= esp_time_us <= 0xFFFFFFFF:
        raise ValueError("ESP timestamp out of range")
    accel = _finite_triplet(parts[2:5], "acceleration")
    gyro = _finite_triplet(parts[5:8], "gyro")
    mag = _finite_triplet(parts[8:11], "magnetometer", allow_nan=True)
    temp = float(parts[11])
    if not math.isfinite(temp):
        raise ValueError("invalid temperature")
    raw = tuple(int(value) for value in parts[12:])
    return ImuSample(esp_time_us, host_receive_ns, accel, gyro, mag, temp, raw)

