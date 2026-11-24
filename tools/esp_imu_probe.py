#!/usr/bin/env python3
"""Read ESP IMU telemetry briefly without asserting auto-reset lines."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SLAM_ROOT = ROOT / "slam_lab"
if str(SLAM_ROOT) not in sys.path:
    sys.path.insert(0, str(SLAM_ROOT))

from drivers.imu_serial.port import open_esp32_serial


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    deadline = time.monotonic() + max(0.5, args.seconds)
    received = 0
    with open_esp32_serial(args.port, args.baud, timeout=0.20) as stream:
        while time.monotonic() < deadline:
            raw = stream.readline()
            if not raw:
                continue
            print(raw.decode("ascii", errors="replace").strip())
            received += 1
    if not received:
        print("NO_SERIAL_TELEMETRY")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
