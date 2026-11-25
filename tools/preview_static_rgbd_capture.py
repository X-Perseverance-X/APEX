"""Build a bounded colored 3D preview on the laptop from one Pi sensor pair."""
from __future__ import annotations

import json
import math
import pathlib
import shutil
import sys

import cv2
import numpy as np
import torch

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / ".runtime/vendor/Depth-Anything-V2"))
from depth_anything_v2.dpt import DepthAnythingV2
from object_scan import ColoredVoxelCloud, camera_from_lidar, camera_matrix, load_sensor_rig
from object_scan.core import lidar_polar_to_camera, transform_points
from tools.dense_rgbd_fusion import cloud_from_depth, compact_cloud, metric_depth_from_lidar


def main() -> None:
    folder = ROOT / "artifacts/static_rgbd_latest"
    captured = json.loads((folder / "sensors.json").read_text(encoding="utf-8"))
    image = cv2.imread(str(folder / "camera.jpg"))
    if image is None:
        raise RuntimeError("Captured camera image unavailable")
    median_luma = float(np.median(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)))
    low_light_enhanced = median_luma < 45.0
    if low_light_enhanced:
        # Only this offline preview is brightened; the robot camera pipeline
        # and its exposure remain untouched. This cannot restore real colors.
        image = np.clip((image.astype(np.float32) / 255.0) ** 0.52 * 255.0,
                        0, 255).astype(np.uint8)
        cv2.imwrite(str(folder / "camera-enhanced.jpg"), image)
    rig = load_sensor_rig(ROOT / "object_scan/sensor_rig.json")
    angles = (captured["telemetry"].get("cameraPanDeg"),
              captured["telemetry"].get("cameraTiltDeg"))
    if any(value is None or abs(float(value) - 90.0) >= 3.0 for value in angles):
        raise RuntimeError("Camera was not centered; static extrinsics invalid")

    torch.set_num_threads(4)
    model = DepthAnythingV2(encoder="vits", features=64,
                           out_channels=[48, 96, 192, 384])
    model.load_state_dict(torch.load(ROOT / ".runtime/models/depth_anything_v2_vits.pth",
                                     map_location="cpu", weights_only=True))
    model.eval()
    with torch.inference_mode():
        relative_depth = model.infer_image(image, 256)
    intrinsics = camera_matrix(rig, image.shape[1], image.shape[0])
    if intrinsics is None:
        focal = image.shape[1] / (2 * math.tan(math.radians(rig["camera"]["horizontal_fov_deg"]) / 2))
        intrinsics = np.array(((focal, 0, image.shape[1]/2),
                               (0, focal, image.shape[0]/2), (0, 0, 1)), dtype=np.float64)
    camera_from_reference = camera_from_lidar(rig)
    lidar_camera = lidar_polar_to_camera(
        captured["lidar"]["points"], camera_from_reference,
        max_range_m=3.0, forward_half_angle_deg=rig["camera"]["horizontal_fov_deg"] / 2,
    )
    depth, fit = metric_depth_from_lidar(relative_depth, lidar_camera, intrinsics,
                                         0.18, 3.0, allowed_modes=("inverse",))
    if fit["samples"] < 8 or fit["median_relative_error"] > 0.25:
        raise RuntimeError(f"Metric LiDAR/depth fit rejected: {fit}")
    camera_xyz, rgb = cloud_from_depth(image, depth, intrinsics, stride=6)
    reference_xyz = transform_points(camera_xyz, np.linalg.inv(camera_from_reference))
    reference_xyz, rgb = compact_cloud(reference_xyz, rgb, voxel=0.035, limit=16000)
    cloud = ColoredVoxelCloud(voxel_size_m=0.035, max_points=16000)
    cloud.integrate(reference_xyz, rgb)
    output = cloud.export_ply(folder / "static-rgb-preview.ply")
    xyz, colors, _ = cloud.arrays()
    points = [[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3),
               int(c[0]), int(c[1]), int(c[2])] for p, c in zip(xyz, colors)]
    (folder / "points.json").write_text(json.dumps(points, separators=(",", ":")), encoding="utf-8")
    metadata = {"points": len(points), "camera_pixels": list(image.shape[1::-1]),
                "source_lidar_returns": len(captured["lidar"]["points"]),
                "sensor_skew_s": captured["skew_s"], "metric_fit": fit,
                "rig_intrinsics_status": rig["camera"]["intrinsics"]["status"],
                "low_light_enhanced": low_light_enhanced,
                "type": "single-view estimated RGB-D preview; not measured dense 3D or Gaussian splat"}
    (folder / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    viewer = (ROOT / "tools/static_rgbd_viewer_template.html").read_text(encoding="utf-8")
    viewer = viewer.replace("/*POINTS_PLACEHOLDER*/", json.dumps(points, separators=(",", ":")))
    (folder / "viewer.html").write_text(viewer, encoding="utf-8")
    shutil.copyfile(ROOT / ".runtime/deploy_command_center_20261112/apex_kinematics_lab/static/three.min.js",
                    folder / "three.min.js")
    print(json.dumps({"ply": str(output), "viewer": str(folder / "viewer.html"), **metadata},
                     ensure_ascii=False, default=float), flush=True)


if __name__ == "__main__":
    main()
