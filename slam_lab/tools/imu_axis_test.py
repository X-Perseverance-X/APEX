#!/usr/bin/env python3
"""Show raw axes and gravity magnitude; no coordinate assumption is made."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import serial
import yaml

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drivers.imu_serial.protocol import parse_imu_line
from drivers.imu_serial.port import open_esp32_serial


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    hardware = yaml.safe_load((root / "config/hardware.yaml").read_text())
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=hardware["imu"]["port"])
    parser.add_argument("--baud", type=int, default=hardware["imu"]["baudrate"])
    args = parser.parse_args()
    print("Keep level, pitch nose up, roll left, then yaw clockwise. Note axis/sign responses.")
    print("Expected stationary |a| near 9.81 m/s^2. Raw axes are NOT yet base_link axes.")
    try:
        with open_esp32_serial(args.port, args.baud, timeout=1.0) as stream:
            while True:
                received_ns = time.monotonic_ns()
                line = stream.readline().decode("ascii", errors="replace").strip()
                if not line.startswith("IMU,"):
                    if line:
                        print(line)
                    continue
                try:
                    sample = parse_imu_line(line, received_ns)
                except ValueError as error:
                    print(f"CORRUPT: {error}")
                    continue
                a = sample.accel_raw_frame_m_s2
                g = sample.gyro_raw_frame_rad_s
                m = sample.mag_raw_frame_uT
                print(
                    f"a={a[0]:+7.3f},{a[1]:+7.3f},{a[2]:+7.3f} m/s2 "
                    f"|a|={sample.accel_norm_m_s2:6.3f} "
                    f"g={g[0]:+7.3f},{g[1]:+7.3f},{g[2]:+7.3f} rad/s "
                    f"m={m[0]:+7.2f},{m[1]:+7.2f},{m[2]:+7.2f} uT"
                )
    except KeyboardInterrupt:
        return 0
    except serial.SerialException as error:
        print(f"SERIAL ERROR: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
