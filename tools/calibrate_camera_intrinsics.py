#!/usr/bin/env python3
"""Calibrate the APEX camera from live checkerboard snapshots.

Run while slowly presenting a flat checkerboard at different positions and
tilts. The result is written atomically into object_scan/sensor_rig.json.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import httpx
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from object_scan import DEFAULT_SENSOR_RIG, load_sensor_rig


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://apex:8080")
    parser.add_argument("--rig-config", default=str(DEFAULT_SENSOR_RIG))
    parser.add_argument("--columns", type=int, default=9, help="checkerboard inner-corner columns")
    parser.add_argument("--rows", type=int, default=6, help="checkerboard inner-corner rows")
    parser.add_argument("--square-mm", type=float, default=25.0)
    parser.add_argument("--frames", type=int, default=24)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--interval", type=float, default=0.35)
    args = parser.parse_args()
    if args.columns < 3 or args.rows < 3 or args.square_mm <= 0 or args.frames < 8:
        raise ValueError("checkerboard must be >=3x3, square size positive, and frames >=8")

    try:
        import cv2
    except ImportError:
        print("HATA: OpenCV gerekli (opencv-python-headless).", file=sys.stderr)
        return 2

    pattern = (args.columns, args.rows)
    object_template = np.zeros((args.columns * args.rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2)
    object_template *= float(args.square_mm) / 1000.0
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size = None
    deadline = time.monotonic() + max(10.0, args.timeout)
    client = httpx.Client(base_url=args.base_url.rstrip("/"), timeout=3.0, follow_redirects=False)

    while len(image_points) < args.frames and time.monotonic() < deadline:
        try:
            response = client.get("/camera/raw-snapshot")
            response.raise_for_status()
            frame = cv2.imdecode(np.frombuffer(response.content, dtype=np.uint8), cv2.IMREAD_COLOR)
        except (httpx.HTTPError, httpx.TimeoutException) as exc:
            print(f"[SKIP] kamera: {exc}", flush=True)
            time.sleep(args.interval)
            continue
        if frame is None:
            time.sleep(args.interval)
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCornersSB(
            gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
        )
        if found:
            object_points.append(object_template.copy())
            image_points.append(corners.astype(np.float32))
            print(f"[OK] kalibrasyon karesi {len(image_points)}/{args.frames}", flush=True)
        else:
            print("[BEKLE] dama tahtası tam görünmüyor", flush=True)
        time.sleep(max(0.05, args.interval))

    if image_size is None or len(image_points) < 8:
        raise RuntimeError(f"yetersiz geçerli kare: {len(image_points)}")
    rms, matrix, distortion, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None,
    )
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or rms > 2.0:
        raise RuntimeError(f"kalibrasyon reddedildi; RMS={rms:.3f}px")

    path = Path(args.rig_config)
    rig = load_sensor_rig(path)
    rig["camera"]["intrinsics"] = {
        "status": "calibrated_checkerboard",
        "calibration_width_px": image_size[0],
        "calibration_height_px": image_size[1],
        "fx": float(matrix[0, 0]),
        "fy": float(matrix[1, 1]),
        "cx": float(matrix[0, 2]),
        "cy": float(matrix[1, 2]),
        "distortion_model": "opencv_plumb_bob",
        "distortion_coefficients": [float(value) for value in distortion.reshape(-1)],
        "rms_reprojection_error_px": float(rms),
        "accepted_frames": len(image_points),
    }
    backup = path.with_suffix(path.suffix + ".bak")
    temporary = path.with_suffix(path.suffix + ".tmp")
    shutil.copy2(path, backup)
    temporary.write_text(json.dumps(rig, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)
    print(json.dumps(rig["camera"]["intrinsics"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
