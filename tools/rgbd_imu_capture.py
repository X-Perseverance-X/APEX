#!/usr/bin/env python3
"""Capture synchronized RGB, planar LiDAR and IMU samples without actuators."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import httpx
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://127.0.0.1:18090")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--max-sync-skew", type=float, default=0.25)
    args = parser.parse_args()

    output = Path(args.output)
    frames = output / "frames"
    frames.mkdir(parents=True, exist_ok=True)
    records: list[dict] = []
    journal_path = output / "capture.jsonl"
    last_scan = -1
    interval = 1.0 / max(0.25, args.rate)
    deadline = None if args.seconds <= 0 else time.monotonic() + max(1.0, args.seconds)

    with httpx.Client(base_url=args.base_url.rstrip("/"), timeout=4.0) as client:
        try:
            while deadline is None or time.monotonic() < deadline:
                started = time.monotonic()
                try:
                    lidar_response = client.get("/api/lidar/snapshot")
                    image_response = client.get("/camera/raw-snapshot")
                    imu_response = client.get("/api/imu/snapshot")
                    lidar_response.raise_for_status()
                    image_response.raise_for_status()
                    imu_response.raise_for_status()
                    lidar = lidar_response.json()
                    imu = imu_response.json()
                except (httpx.HTTPError, ValueError) as exc:
                    print(f"[SKIP] sensor bridge: {exc}", flush=True)
                    time.sleep(max(0.01, interval - (time.monotonic() - started)))
                    continue
                scan = int(lidar.get("scan_seq", -1))
                if scan == last_scan or not lidar.get("fresh") or not imu.get("fresh"):
                    time.sleep(max(0.01, interval - (time.monotonic() - started)))
                    continue
                camera_ts = float(image_response.headers["x-apex-frame-ts"])
                lidar_ts = float(lidar["timestamp_monotonic_s"])
                imu_ts = float(imu["timestamp_monotonic_s"])
                skew = max(abs(camera_ts - lidar_ts), abs(camera_ts - imu_ts))
                if skew > args.max_sync_skew:
                    print(f"[SKIP] sensor skew={skew * 1000:.0f}ms", flush=True)
                    time.sleep(max(0.01, interval - (time.monotonic() - started)))
                    continue
                image = cv2.imdecode(np.frombuffer(image_response.content, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                filename = f"{len(records):05d}.jpg"
                cv2.imwrite(str(frames / filename), image, [cv2.IMWRITE_JPEG_QUALITY, 94])
                records.append({
                    "index": len(records),
                    "frame": f"frames/{filename}",
                    "scan_seq": scan,
                    "timestamps": {"camera": camera_ts, "lidar": lidar_ts, "imu": imu_ts},
                    "sync_skew_s": skew,
                    "lidar_points": lidar.get("points", []),
                    "imu": imu.get("imu"),
                })
                with journal_path.open("a", encoding="utf-8") as journal:
                    journal.write(json.dumps(records[-1], separators=(",", ":")) + "\n")
                last_scan = scan
                if len(records) == 1 or len(records) % 20 == 0:
                    (output / "capture.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
                online = bool((imu.get("imu") or {}).get("online"))
                print(f"[OK] frame={len(records):03d} lidar={len(lidar.get('points', []))} imu={online} skew={skew*1000:.0f}ms", flush=True)
                time.sleep(max(0.01, interval - (time.monotonic() - started)))
        except KeyboardInterrupt:
            print("\n[STOP] operator requested stop; finalizing capture...", flush=True)

    if not records:
        raise RuntimeError("no synchronized sensor sample was captured")
    (output / "capture.json").write_text(json.dumps(records, indent=2), encoding="utf-8")
    print(f"Captured {len(records)} synchronized samples in {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
