#!/usr/bin/env python3
"""Continuously produce true dense RGB-D frames for the APEX navigation HUD.

Depth Anything supplies dense inverse depth and the camera-visible LiDAR sector
metrizes it.  Visual odometry, IMU rotation and metric ICP track a handheld
scanner around a static object; navigation SLAM is not used as object pose.
This process is read-only with respect to the robot: it does not import or call
any actuator, GPIO or PWM API.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import httpx
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from object_scan import ColoredVoxelCloud, camera_from_lidar, camera_matrix, load_sensor_rig
from object_scan.core import lidar_polar_to_camera, transform_points
from tools.dense_rgbd_fusion import (
    cloud_from_depth,
    compact_cloud,
    imu_prev_from_current,
    metric_depth_from_lidar,
    odometry_bridge,
    register_pair,
    visual_relative,
)


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # Windows can briefly deny replacing a file while FastAPI is opening the
    # previous snapshot.  A process-unique temporary plus bounded retry keeps
    # this harmless reader/writer race from killing the long-running worker.
    temporary = path.with_suffix(path.suffix + f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    for attempt in range(50):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 49:
                raise
            time.sleep(0.02)


def status_payload(status: str, message: str, *, sequence: int = 0) -> dict:
    return {
        "schema": "apex-live-dense-rgbd-v1",
        "status": status,
        "message": message,
        "sequence": sequence,
        "generated_at": time.time(),
        "points": [],
    }


def encode_points(xyz: np.ndarray, rgb: np.ndarray) -> list[list[float | int]]:
    output: list[list[float | int]] = []
    for point, color in zip(xyz, rgb):
        if np.isfinite(point).all():
            output.append([
                round(float(point[0]), 3), round(float(point[1]), 3), round(float(point[2]), 3),
                int(color[0]), int(color[1]), int(color[2]),
            ])
    return output


def sensor_pair_skew_s(lidar_age_s: float, camera_age_s: float,
                       elapsed_after_lidar_s: float) -> float:
    """Compare frame ages in one time domain despite mixed source clocks."""
    ages = (lidar_age_s, camera_age_s, elapsed_after_lidar_s)
    if not all(math.isfinite(value) and value >= 0 for value in ages):
        raise ValueError("invalid sensor frame age")
    return abs(camera_age_s - (lidar_age_s + elapsed_after_lidar_s))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-url", default="http://192.168.1.76:8090")
    parser.add_argument("--navigation-url", default="http://127.0.0.1:8878")
    parser.add_argument("--output", default=str(ROOT / ".runtime" / "live-dense-cloud.json"))
    parser.add_argument("--rig", default=str(ROOT / "object_scan" / "sensor_rig.json"))
    parser.add_argument("--model-repo", default=str(ROOT / ".runtime" / "vendor" / "Depth-Anything-V2"))
    parser.add_argument("--weights", default=str(ROOT / ".runtime" / "models" / "depth_anything_v2_vits.pth"))
    parser.add_argument("--input-size", type=int, default=256)
    parser.add_argument("--stride", type=int, default=6)
    parser.add_argument("--max-depth", type=float, default=3.0)
    parser.add_argument("--period", type=float, default=0.35)
    args = parser.parse_args()

    output = Path(args.output)
    atomic_json(output, status_payload("LOADING", "Yoğun derinlik modeli yükleniyor"))

    print("[DENSE] importing torch/model", flush=True)
    import torch
    torch.set_num_threads(max(1, min(4, os.cpu_count() or 1)))
    sys.path.insert(0, str(Path(args.model_repo)))
    from depth_anything_v2.dpt import DepthAnythingV2

    print("[DENSE] constructing model", flush=True)
    model = DepthAnythingV2(encoder="vits", features=64, out_channels=[48, 96, 192, 384])
    print("[DENSE] loading weights", flush=True)
    model.load_state_dict(torch.load(args.weights, map_location="cpu", weights_only=True))
    model.eval()
    print("[DENSE] model ready", flush=True)

    rig = load_sensor_rig(args.rig)
    camera_from_reference = camera_from_lidar(rig)
    reference_from_camera = np.linalg.inv(camera_from_reference)
    cloud = ColoredVoxelCloud(voxel_size_m=0.045, max_points=40_000)
    client = httpx.Client(timeout=httpx.Timeout(4.0, connect=2.0))
    sequence = 0
    last_scan = -1
    last_session = ""
    last_archive = ""
    last_error = ""
    last_payload: dict | None = None
    last_heartbeat = 0.0
    filtered_tilt: tuple[float, float] | None = None
    previous_image: np.ndarray | None = None
    previous_local: np.ndarray | None = None
    previous_rpy: list[float] | None = None
    previous_probe: np.ndarray | None = None
    world_from_reference = np.eye(4, dtype=np.float64)
    registration_rejects = 0
    recent_translation_steps: list[float] = []
    metric_history: list[tuple[float, float]] = []
    camera_half_fov_deg = float(rig["camera"]["horizontal_fov_deg"]) * 0.5

    while True:
        started = time.monotonic()
        reset_cloud = False
        try:
            navigation = client.get(f"{args.navigation_url}/api/navigation/status").json()
            session = str((navigation.get("mapping_session") or {}).get("state") or "LIVE_PREVIEW")
            archive = str((navigation.get("mapping_session") or {}).get("archive_id") or "")
            if session == "MAPPING" and last_session != "MAPPING":
                cloud = ColoredVoxelCloud(voxel_size_m=0.045, max_points=40_000)
                previous_image = previous_local = previous_rpy = previous_probe = None
                world_from_reference = np.eye(4, dtype=np.float64)
                registration_rejects = 0
                recent_translation_steps.clear()
                metric_history.clear()
                reset_cloud = True
            just_frozen = session != "MAPPING" and last_session == "MAPPING" and len(cloud) > 0
            if just_frozen:
                map_id = archive or time.strftime("dense-%Y%m%d-%H%M%S")
                cloud.export_ply(ROOT / ".runtime" / "maps" / f"{map_id}-dense-rgb.ply")
                last_archive = map_id
            last_session = session

            # After finishing a mapping run, keep its accumulated cloud immutable.
            # On a fresh process start the navigation engine may already be FROZEN;
            # still provide a dense one-frame preview so 3D never appears blank.
            if just_frozen or (session not in {"MAPPING", "LIVE_PREVIEW"} and len(cloud) > 0):
                time.sleep(max(0.1, args.period))
                continue
            display_mode = "MAPPING" if session == "MAPPING" else "LIVE_PREVIEW"

            lidar = client.get(f"{args.sensor_url}/api/lidar/snapshot").json()
            lidar_received_at = time.monotonic()
            imu = client.get(f"{args.sensor_url}/api/imu/snapshot").json()
            scan = int(lidar.get("scan_seq") or -1)
            if not lidar.get("fresh") or scan < 0 or scan == last_scan:
                time.sleep(0.08)
                continue
            last_scan = scan

            imu_sample = (imu or {}).get("imu") or imu or {}
            rpy = list(imu_sample.get("rpy_deg") or [0.0, 0.0, 0.0])
            if filtered_tilt is None:
                filtered_tilt = (float(rpy[0]), float(rpy[1]))
            else:
                filtered_tilt = (
                    filtered_tilt[0] * 0.55 + float(rpy[0]) * 0.45,
                    filtered_tilt[1] * 0.55 + float(rpy[1]) * 0.45,
                )
            rpy[0], rpy[1] = filtered_tilt
            camera_response = client.get(f"{args.sensor_url}/camera/raw-snapshot")
            camera_response.raise_for_status()
            image = cv2.imdecode(np.frombuffer(camera_response.content, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("kamera JPEG karesi çözülemedi")
            camera_age = camera_response.headers.get("x-apex-frame-age")
            lidar_age = lidar.get("scan_age_s")
            if camera_age is None or lidar_age is None:
                raise RuntimeError("kamera/LiDAR yaş damgası eksik")
            skew_s = sensor_pair_skew_s(float(lidar_age), float(camera_age),
                                        time.monotonic() - lidar_received_at)
            if skew_s > 0.35:
                raise RuntimeError(f"kamera/LiDAR zaman farkı yüksek: {skew_s * 1000:.0f}ms")
            h, w = image.shape[:2]
            intrinsics = camera_matrix(rig, w, h)
            if intrinsics is None:
                f = w / (2 * math.tan(math.radians(float(rig["camera"]["horizontal_fov_deg"])) / 2))
                intrinsics = np.array(((f, 0, w / 2), (0, f, h / 2), (0, 0, 1)), dtype=np.float64)

            probe = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (48, 27), interpolation=cv2.INTER_AREA)
            if previous_probe is not None and previous_rpy is not None:
                visual_change = float(np.mean(cv2.absdiff(probe, previous_probe)))
                tilt_change = max(abs(float(rpy[0]) - previous_rpy[0]), abs(float(rpy[1]) - previous_rpy[1]))
                if visual_change < 2.2 and tilt_change < 1.0:
                    if last_payload is not None and time.monotonic() - last_heartbeat >= 4.0:
                        last_payload["generated_at"] = time.time()
                        atomic_json(output, last_payload)
                        last_heartbeat = time.monotonic()
                    time.sleep(0.08)
                    continue

            relative = model.infer_image(image, args.input_size)
            lidar_camera = lidar_polar_to_camera(
                lidar.get("points") or [], camera_from_reference,
                max_range_m=args.max_depth, forward_half_angle_deg=camera_half_fov_deg,
            )
            depth, calibration = metric_depth_from_lidar(
                relative, lidar_camera, intrinsics, 0.18, args.max_depth,
                allowed_modes=("inverse",),
            )
            if float(calibration["median_relative_error"]) > 0.25:
                raise RuntimeError(f"LiDAR/derinlik uyumu zayıf: {calibration['median_relative_error']:.2f}")

            # Kare başına bağımsız LiDAR ölçeği yüzeyleri ileri/geri zıplatır.
            # Son beş güvenilir inverse-depth fitinin medyanı gerçek metre
            # ölçeğini korurken tek-kare sıçramalarını bastırır.
            metric_history.append((float(calibration["a"]), float(calibration["b"])))
            del metric_history[:-5]
            stable_a, stable_b = np.median(np.asarray(metric_history), axis=0)
            fitted = stable_a * relative.astype(np.float64) + stable_b
            depth = 1.0 / fitted
            depth[~np.isfinite(depth)] = np.nan
            depth[(depth < 0.18) | (depth > args.max_depth)] = np.nan
            depth = depth.astype(np.float32)
            calibration["stable_a"] = float(stable_a)
            calibration["stable_b"] = float(stable_b)

            camera_xyz, rgb = cloud_from_depth(image, depth, intrinsics, args.stride)
            reference_xyz = transform_points(camera_xyz, reference_from_camera)
            reference_xyz, rgb = compact_cloud(reference_xyz, rgb, voxel=0.040, limit=3200)
            registration_xyz, _ = compact_cloud(reference_xyz, rgb, voxel=0.060, limit=1800)
            registration = {"accepted": True, "reason": "first frame", "fitness": 1.0, "rmse_m": 0.0}
            parallax_px = 0.0
            if previous_local is not None and previous_image is not None and previous_rpy is not None:
                imu_seed = imu_prev_from_current(previous_rpy, rpy)
                visual_motion, parallax_px = visual_relative(previous_image, image, intrinsics)
                if visual_motion is None:
                    registration_rejects += 1
                    world_from_reference = world_from_reference @ imu_seed
                    previous_image, previous_local, previous_rpy, previous_probe = image, registration_xyz, list(rpy), probe
                    raise RuntimeError("görsel poz geçici kayıp; IMU ile takip korunuyor")
                relative_pose, registration = register_pair(
                    registration_xyz, previous_local, visual_motion, imu_seed, reference_from_camera,
                )
                if not registration["accepted"] or relative_pose is None:
                    registration_rejects += 1
                    step = float(np.median(recent_translation_steps[-8:])) if recent_translation_steps else 0.025
                    use_icp_bridge = (
                        relative_pose is not None and np.isfinite(relative_pose).all()
                        and math.isfinite(float(registration["rmse_m"]))
                        and float(registration["rmse_m"]) <= 0.30
                        and float(registration["fitness"]) >= 0.10
                    )
                    bridge = relative_pose if use_icp_bridge else odometry_bridge(
                        visual_motion, imu_seed, reference_from_camera, step,
                    )
                    world_from_reference = world_from_reference @ bridge
                    previous_image, previous_local, previous_rpy, previous_probe = image, registration_xyz, list(rpy), probe
                    raise RuntimeError(
                        f"ICP kareyi buluta almadı; takip köprülendi: fit={registration['fitness']:.2f} "
                        f"rmse={registration['rmse_m']:.3f}m"
                    )
                world_from_reference = world_from_reference @ relative_pose
                step_length = float(np.linalg.norm(relative_pose[:3, 3]))
                if 0.002 <= step_length <= 0.30:
                    recent_translation_steps.append(step_length)
            world_xyz = transform_points(reference_xyz, world_from_reference)
            sequence += 1

            if session == "MAPPING":
                cloud.integrate(world_xyz, rgb)
            previous_image = image
            previous_local = registration_xyz
            previous_rpy = list(rpy)
            previous_probe = probe
            registration_rejects = 0
            yaw_rad = math.atan2(world_from_reference[1, 0], world_from_reference[0, 0])
            payload = {
                "schema": "apex-live-dense-rgbd-v1",
                "status": "READY",
                "message": "Gerçek kamera piksellerinden yoğun RGB-D",
                "sequence": sequence,
                "generated_at": time.time(),
                "mode": display_mode,
                "scan_seq": scan,
                "metric_fit_error": round(float(calibration["median_relative_error"]), 4),
                "metric_mode": calibration["mode"],
                "metric_samples": int(calibration["samples"]),
                "camera_fov_deg": float(rig["camera"]["horizontal_fov_deg"]),
                "lidar_sector_deg": [-camera_half_fov_deg, camera_half_fov_deg],
                "tracking": {
                    "source": "visual_imu_icp",
                    "fitness": round(float(registration.get("fitness") or 0.0), 4),
                    "rmse_m": round(float(registration.get("rmse_m") or 0.0), 4),
                    "parallax_px": round(float(parallax_px), 2),
                },
                "scanner_pose": [
                    round(float(world_from_reference[0, 3]), 4),
                    round(float(world_from_reference[1, 3]), 4),
                    round(float(world_from_reference[2, 3]), 4),
                    round(float(yaw_rad), 5),
                ],
                "frame_points": len(world_xyz),
                "accumulated_points": len(cloud),
                "saved_archive": last_archive,
                "reset": reset_cloud,
                "points": encode_points(world_xyz, rgb),
            }
            atomic_json(output, payload)
            last_payload = payload
            last_heartbeat = time.monotonic()
            if sequence <= 3 or sequence % 10 == 0:
                print(f"[DENSE] frame={sequence} points={len(world_xyz)} fit={calibration['median_relative_error']:.3f}", flush=True)
            last_error = ""
        except (httpx.HTTPError, ValueError, RuntimeError, KeyError, json.JSONDecodeError) as exc:
            message = str(exc)
            if message != last_error:
                atomic_json(output, status_payload("WAITING", message, sequence=sequence))
                last_error = message
        elapsed = time.monotonic() - started
        time.sleep(max(0.05, args.period - elapsed))


if __name__ == "__main__":
    raise SystemExit(main())
