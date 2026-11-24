#!/usr/bin/env python3
"""Capture a safe, actuator-free hand-held object scan from the APEX head."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import httpx
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from object_scan import (
    DEFAULT_SENSOR_RIG,
    ObjectScanSession,
    camera_distortion,
    camera_from_lidar,
    camera_matrix,
    load_sensor_rig,
    with_translation_overrides,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://apex:8080")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--rate", type=float, default=4.0)
    parser.add_argument("--output", default=str(ROOT / ".runtime" / "object-scans" / time.strftime("%Y%m%d-%H%M%S")))
    parser.add_argument("--max-range", type=float, default=2.5)
    parser.add_argument("--motion-mode", choices=("free", "pivot"), default="free")
    parser.add_argument("--rig-config", default=str(DEFAULT_SENSOR_RIG))
    parser.add_argument("--camera-x", type=float, default=None)
    parser.add_argument("--camera-y", type=float, default=None)
    parser.add_argument("--camera-z", type=float, default=None)
    parser.add_argument("--imu-x", type=float, default=None)
    parser.add_argument("--imu-y", type=float, default=None)
    parser.add_argument("--imu-z", type=float, default=None)
    parser.add_argument("--max-sync-skew", type=float, default=0.18,
                        help="Reject camera/LiDAR pairs farther apart than this many seconds")
    args = parser.parse_args()

    rig = with_translation_overrides(load_sensor_rig(args.rig_config), vars(args))
    lidar_to_camera = camera_from_lidar(rig)
    if args.max_sync_skew <= 0.0:
        raise ValueError("--max-sync-skew must be positive")

    try:
        import cv2
    except ImportError:
        print("HATA: OpenCV gerekli (opencv-python-headless).", file=sys.stderr)
        return 2

    output = Path(args.output)
    frames_dir = output / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    client = httpx.Client(base_url=args.base_url.rstrip("/"), timeout=3.0, follow_redirects=False)
    session = None
    last_sequence = -1
    deadline = time.monotonic() + max(1.0, args.seconds)
    interval = 1.0 / max(0.5, args.rate)
    samples: list[dict] = []
    capture_path = output / "capture.json"

    while time.monotonic() < deadline:
        cycle_started = time.monotonic()
        try:
            lidar_response = client.get("/api/lidar/snapshot")
            camera_response = client.get("/camera/raw-snapshot")
            lidar_response.raise_for_status()
            camera_response.raise_for_status()
        except (httpx.HTTPError, httpx.TimeoutException) as exc:
            print(f"[SKIP] geçici sensör kesintisi: {exc}", flush=True)
            time.sleep(max(0.05, interval - (time.monotonic() - cycle_started)))
            continue
        lidar = lidar_response.json()
        sequence = int(lidar.get("scan_seq", -1))
        if not lidar.get("fresh") or sequence == last_sequence:
            time.sleep(max(0.01, interval - (time.monotonic() - cycle_started)))
            continue
        frame = cv2.imdecode(np.frombuffer(camera_response.content, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("ham kamera JPEG çözülemedi")
        camera_ts_raw = camera_response.headers.get("x-apex-frame-ts")
        lidar_ts_raw = lidar.get("timestamp_monotonic_s")
        try:
            sync_skew_s = abs(float(camera_ts_raw) - float(lidar_ts_raw))
        except (TypeError, ValueError):
            sync_skew_s = None
        if sync_skew_s is None or sync_skew_s > args.max_sync_skew:
            label = "yok" if sync_skew_s is None else f"{sync_skew_s * 1000.0:.0f}ms"
            print(f"[SKIP] kamera/LiDAR zaman farkı {label}", flush=True)
            time.sleep(max(0.01, interval - (time.monotonic() - cycle_started)))
            continue
        if session is None:
            height, width = frame.shape[:2]
            intrinsics = camera_matrix(rig, width, height)
            session = ObjectScanSession(
                width,
                height,
                horizontal_fov_deg=float(rig["camera"]["horizontal_fov_deg"]),
                max_range_m=args.max_range,
                camera_from_lidar=lidar_to_camera,
                camera_matrix=intrinsics,
                distortion_coefficients=camera_distortion(rig),
                rig_metadata=rig,
                motion_mode=args.motion_mode,
            )
        update = session.add_frame(frame, lidar.get("points", []), time.time())
        last_sequence = sequence
        frame_path = frames_dir / f"{sequence:08d}.jpg"
        cv2.imwrite(str(frame_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        record = {
            "scan_seq": sequence,
            "frame": frame_path.name,
            "captured_at_s": time.time(),
            "camera_frame_age_s": camera_response.headers.get("x-apex-frame-age"),
            "lidar_scan_age_s": lidar.get("scan_age_s"),
            "sensor_sync_skew_s": sync_skew_s,
            "lidar_points": lidar.get("points", []),
            **update.__dict__,
        }
        samples.append(record)
        capture_path.write_text(json.dumps(samples, ensure_ascii=False, indent=2), encoding="utf-8")
        state = "OK" if update.accepted else "RED"
        rmse = "--" if update.icp_rmse_m is None else f"{update.icp_rmse_m * 100:.1f}cm"
        print(f"[{state}] kare={update.frame_index} bulut={update.cloud_points} inlier={update.visual_inliers} ICP={update.icp_fitness:.2f}/{rmse} {update.reason}", flush=True)
        time.sleep(max(0.01, interval - (time.monotonic() - cycle_started)))

    if session is None:
        raise RuntimeError("hiç eşzamanlı LiDAR/kamera karesi alınamadı")
    result = session.export(output)
    capture_path.write_text(json.dumps(samples, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
