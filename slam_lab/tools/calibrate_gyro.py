#!/usr/bin/env python3
"""Stationary gyroscope bias calibration; writes only calibration.yaml."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from pathlib import Path

import serial
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drivers.imu_serial.protocol import parse_imu_line
from drivers.imu_serial.port import open_esp32_serial


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    hardware = yaml.safe_load((root / "config/hardware.yaml").read_text())
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--port", default=hardware["imu"]["port"])
    parser.add_argument("--session", type=Path,
                        help="Read the currently recording session instead of opening the busy serial port")
    args = parser.parse_args()
    if args.seconds < 2:
        parser.error("collect at least 2 seconds")
    print("Keep the IMU completely still during collection.")
    rows: list[tuple[float, float, float]] = []
    accel_norms: list[float] = []
    if args.session:
        source = args.session / "imu_raw.csv"
        if not source.is_file():
            print(f"FAIL: session IMU log not found: {source}")
            return 1
        start_ns = time.monotonic_ns()
        time.sleep(args.seconds)
        end_ns = time.monotonic_ns()
        with source.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                stamp = int(row["host_receive_ns"])
                if start_ns <= stamp <= end_ns:
                    rows.append(tuple(float(row[key]) for key in ("gx_rad_s", "gy_rad_s", "gz_rad_s")))
                    accel = tuple(float(row[key]) for key in ("ax_m_s2", "ay_m_s2", "az_m_s2"))
                    accel_norms.append(math.sqrt(sum(value * value for value in accel)))
    else:
        start = time.monotonic()
        with open_esp32_serial(args.port, hardware["imu"]["baudrate"], timeout=1) as stream:
            while time.monotonic() - start < args.seconds:
                received_ns = time.monotonic_ns()
                line = stream.readline().decode("ascii", errors="replace").strip()
                if line.startswith("IMU,"):
                    try:
                        sample = parse_imu_line(line, received_ns)
                    except ValueError:
                        continue
                    rows.append(sample.gyro_raw_frame_rad_s)
                    accel_norms.append(sample.accel_norm_m_s2)
    if len(rows) < args.seconds * 30:
        print(f"FAIL: only {len(rows)} valid samples; expected at least 30 Hz")
        return 1
    bias = [statistics.mean(row[axis] for row in rows) for axis in range(3)]
    spread = [statistics.pstdev(row[axis] for row in rows) for axis in range(3)]
    if max(spread) > 0.05:
        print(f"FAIL: gyro moved/noisy; standard deviation={spread} rad/s")
        return 1
    accel_mean = statistics.mean(accel_norms)
    accel_spread = statistics.pstdev(accel_norms)
    if accel_spread > 0.2:
        print(f"FAIL: accelerometer indicates motion; |a| spread={accel_spread:.3f} m/s2")
        return 1
    path = root / "config/calibration.yaml"
    calibration = yaml.safe_load(path.read_text())
    calibration["imu"]["gyro_bias_raw_frame_rad_s"] = bias
    calibration["imu"]["gyro_bias_calibrated"] = True
    calibration["imu"]["gyro_bias_sample_count"] = len(rows)
    path.write_text(yaml.safe_dump(calibration, sort_keys=False), encoding="utf-8")
    print(f"PASS: {len(rows)} samples; raw-frame bias={bias} rad/s; spread={spread}")
    print(f"Stationary accelerometer |a|={accel_mean:.3f} m/s2, spread={accel_spread:.3f}; "
          "this is NOT a six-position accelerometer calibration")
    if abs(accel_mean - 9.80665) > 0.5:
        print("WARNING: accelerometer magnitude is far from gravity; keep attitude/map gated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
