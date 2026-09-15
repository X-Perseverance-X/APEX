"""Metric multi-view reconstruction from a moving camera + planar LiDAR head.

The LiDAR contributes measured metric surface points. The camera estimates the
six-degree-of-freedom inter-frame motion and supplies RGB for those points.
ICP refines the visual pose against already accumulated LiDAR geometry. Weak
frames are rejected instead of being smeared into the model.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.spatial import cKDTree


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    xyz = np.asarray(points, dtype=np.float64)
    matrix = np.asarray(transform, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or matrix.shape != (4, 4):
        raise ValueError("points must be Nx3 and transform must be 4x4")
    return (matrix[:3, :3] @ xyz.T).T + matrix[:3, 3]


def estimate_rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Kabsch transform mapping paired ``source`` points onto ``target``."""
    src = np.asarray(source, dtype=np.float64)
    dst = np.asarray(target, dtype=np.float64)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3 or len(src) < 3:
        raise ValueError("source/target must be matching Nx3 arrays with N >= 3")
    src_center = src.mean(axis=0)
    dst_center = dst.mean(axis=0)
    covariance = (src - src_center).T @ (dst - dst_center)
    u, _singular, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = dst_center - rotation @ src_center
    return result


def _rotation_angle(rotation: np.ndarray) -> float:
    trace = float(np.trace(rotation))
    return math.acos(max(-1.0, min(1.0, (trace - 1.0) * 0.5)))


@dataclass(frozen=True)
class ICPResult:
    transform: np.ndarray
    accepted: bool
    fitness: float
    rmse_m: float
    correspondences: int
    iterations: int
    reason: str


def icp_register(
    source: np.ndarray,
    target: np.ndarray,
    initial_transform: np.ndarray | None = None,
    *,
    max_correspondence_m: float = 0.12,
    max_iterations: int = 18,
    min_correspondences: int = 10,
    max_step_translation_m: float = 0.08,
    max_step_rotation_deg: float = 8.0,
) -> ICPResult:
    """Point-to-point ICP with bounded updates and fail-closed acceptance."""
    src = np.asarray(source, dtype=np.float64)
    dst = np.asarray(target, dtype=np.float64)
    transform = np.eye(4, dtype=np.float64) if initial_transform is None else np.asarray(initial_transform, dtype=np.float64).copy()
    if src.ndim != 2 or src.shape[1:] != (3,) or dst.ndim != 2 or dst.shape[1:] != (3,):
        raise ValueError("source and target must be Nx3")
    if len(src) < min_correspondences or len(dst) < min_correspondences:
        return ICPResult(transform, False, 0.0, math.inf, 0, 0, "NOT_ENOUGH_POINTS")

    tree = cKDTree(dst)
    previous_rmse = math.inf
    correspondences = 0
    iteration = 0
    for iteration in range(1, max_iterations + 1):
        transformed = transform_points(src, transform)
        distances, indices = tree.query(transformed, k=1)
        mask = np.isfinite(distances) & (distances <= max_correspondence_m)
        correspondences = int(mask.sum())
        if correspondences < min_correspondences:
            break
        paired_src = transformed[mask]
        paired_dst = dst[np.asarray(indices[mask], dtype=np.int64)]
        delta = estimate_rigid_transform(paired_src, paired_dst)
        translation = float(np.linalg.norm(delta[:3, 3]))
        rotation = _rotation_angle(delta[:3, :3])
        if translation > max_step_translation_m or rotation > math.radians(max_step_rotation_deg):
            return ICPResult(transform, False, correspondences / len(src), previous_rmse, correspondences, iteration, "UNBOUNDED_ICP_STEP")
        transform = delta @ transform
        rmse = float(np.sqrt(np.mean(np.square(distances[mask]))))
        if abs(previous_rmse - rmse) < 0.0005:
            previous_rmse = rmse
            break
        previous_rmse = rmse

    fitness = correspondences / max(1, len(src))
    accepted = (
        correspondences >= min_correspondences
        and fitness >= 0.30
        and previous_rmse <= min(0.045, max_correspondence_m * 0.35)
    )
    reason = "OK" if accepted else "WEAK_GEOMETRIC_OVERLAP"
    return ICPResult(transform, accepted, fitness, previous_rmse, correspondences, iteration, reason)


def lidar_polar_to_camera(
    polar_points: Iterable[Sequence[float]],
    camera_from_lidar: np.ndarray,
    *,
    min_range_m: float = 0.18,
    max_range_m: float = 2.5,
    forward_half_angle_deg: float = 72.0,
) -> np.ndarray:
    """Convert the useful front LiDAR sector to OpenCV camera coordinates."""
    lidar_xyz: list[tuple[float, float, float]] = []
    for angle_deg, distance_mm, *_rest in polar_points:
        distance_m = float(distance_mm) / 1000.0
        signed_angle = (float(angle_deg) + 180.0) % 360.0 - 180.0
        if not math.isfinite(distance_m) or not min_range_m <= distance_m <= max_range_m:
            continue
        if abs(signed_angle) > forward_half_angle_deg:
            continue
        angle = math.radians(float(angle_deg))
        lidar_xyz.append((distance_m * math.cos(angle), distance_m * math.sin(angle), 0.0))
    if not lidar_xyz:
        return np.empty((0, 3), dtype=np.float64)
    return transform_points(np.asarray(lidar_xyz, dtype=np.float64), camera_from_lidar)


class ColoredVoxelCloud:
    def __init__(self, voxel_size_m: float = 0.012, max_points: int = 120_000):
        if voxel_size_m <= 0 or max_points <= 0:
            raise ValueError("voxel_size_m and max_points must be positive")
        self.voxel_size_m = float(voxel_size_m)
        self.max_points = int(max_points)
        self._voxels: dict[tuple[int, int, int], tuple[np.ndarray, np.ndarray, int]] = {}

    def integrate(self, xyz: np.ndarray, rgb: np.ndarray | None = None) -> int:
        points = np.asarray(xyz, dtype=np.float64)
        colors = np.zeros((len(points), 3), dtype=np.float64) if rgb is None else np.asarray(rgb, dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (3,) or colors.shape != points.shape:
            raise ValueError("xyz and rgb must both be Nx3")
        added = 0
        for point, color in zip(points, colors):
            if not np.isfinite(point).all() or not np.isfinite(color).all():
                continue
            key = tuple(int(math.floor(float(value) / self.voxel_size_m)) for value in point)
            if key in self._voxels:
                old_point, old_color, count = self._voxels[key]
                next_count = min(count + 1, 255)
                weight = 1.0 / next_count
                self._voxels[key] = (
                    old_point * (1.0 - weight) + point * weight,
                    old_color * (1.0 - weight) + color * weight,
                    next_count,
                )
            else:
                self._voxels[key] = (point.copy(), np.clip(color, 0, 255), 1)
                added += 1
        while len(self._voxels) > self.max_points:
            self._voxels.pop(next(iter(self._voxels)))
        return added

    def arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self._voxels:
            return np.empty((0, 3)), np.empty((0, 3), dtype=np.uint8), np.empty((0,), dtype=np.uint8)
        values = list(self._voxels.values())
        xyz = np.asarray([item[0] for item in values], dtype=np.float64)
        rgb = np.clip(np.asarray([item[1] for item in values]), 0, 255).astype(np.uint8)
        counts = np.asarray([item[2] for item in values], dtype=np.uint8)
        return xyz, rgb, counts

    def export_ply(self, path: str | Path, *, minimum_observations: int = 1) -> Path:
        xyz, rgb, counts = self.arrays()
        keep = counts >= max(1, int(minimum_observations))
        xyz, rgb = xyz[keep], rgb[keep]
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="ascii", newline="\n") as handle:
            handle.write("ply\nformat ascii 1.0\n")
            handle.write(f"element vertex {len(xyz)}\n")
            handle.write("property float x\nproperty float y\nproperty float z\n")
            handle.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
            for point, color in zip(xyz, rgb):
                handle.write(
                    f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                    f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
                )
        return output

    def __len__(self) -> int:
        return len(self._voxels)


class _VisualOdometry:
    def __init__(self, width: int, height: int, horizontal_fov_deg: float):
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - depends on deployment image
            raise RuntimeError("OpenCV object scanning için gerekli") from exc
        self.cv2 = cv2
        focal = width / (2.0 * math.tan(math.radians(horizontal_fov_deg) * 0.5))
        self.camera_matrix = np.asarray(((focal, 0.0, width * 0.5), (0.0, focal, height * 0.5), (0.0, 0.0, 1.0)), dtype=np.float64)
        self.orb = cv2.ORB_create(nfeatures=2400, scaleFactor=1.2, nlevels=8, fastThreshold=10)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.previous_keypoints = None
        self.previous_descriptors = None
        self._candidate = None

    def commit(self) -> None:
        if self._candidate is not None:
            self.previous_keypoints, self.previous_descriptors = self._candidate
            self._candidate = None

    def update(self, frame_bgr: np.ndarray) -> tuple[np.ndarray | None, int, int, float]:
        cv2 = self.cv2
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape[:2]
        mask = np.zeros_like(gray)
        mask[int(height * 0.10):int(height * 0.90), int(width * 0.12):int(width * 0.88)] = 255
        keypoints, descriptors = self.orb.detectAndCompute(gray, mask)
        self._candidate = (keypoints, descriptors)
        if descriptors is None or len(keypoints) < 30:
            return None, len(keypoints), 0, 0.0
        if self.previous_descriptors is None or self.previous_keypoints is None:
            self.commit()
            return np.eye(4, dtype=np.float64), len(keypoints), 0, 0.0
        pairs = self.matcher.knnMatch(self.previous_descriptors, descriptors, k=2)
        good = [first for first, second in pairs if first.distance < 0.73 * second.distance]
        if len(good) < 24:
            return None, len(keypoints), len(good), 0.0
        previous = np.float32([self.previous_keypoints[item.queryIdx].pt for item in good])
        current = np.float32([keypoints[item.trainIdx].pt for item in good])
        essential, mask_e = cv2.findEssentialMat(previous, current, self.camera_matrix, cv2.RANSAC, 0.999, 1.5)
        if essential is None:
            return None, len(keypoints), 0, 0.0
        inliers, rotation, translation, _pose_mask = cv2.recoverPose(essential, previous, current, self.camera_matrix, mask=mask_e)
        mask_values = np.asarray(_pose_mask).reshape(-1) > 0
        displacements = np.linalg.norm(current - previous, axis=1)
        parallax_px = float(np.median(displacements[mask_values])) if np.any(mask_values) else 0.0
        if int(inliers) < 32 or parallax_px < 1.5:
            return None, len(keypoints), int(inliers), parallax_px
        current_from_previous = np.eye(4, dtype=np.float64)
        current_from_previous[:3, :3] = rotation
        current_from_previous[:3, 3] = translation.reshape(3)
        return current_from_previous, len(keypoints), int(inliers), parallax_px


@dataclass(frozen=True)
class ScanUpdate:
    accepted: bool
    reason: str
    frame_index: int
    cloud_points: int
    lidar_points: int
    visual_features: int
    visual_inliers: int
    icp_fitness: float
    icp_rmse_m: float | None
    translation_scale_m: float | None


class ObjectScanSession:
    """Fail-closed hand-held scanner for a static object."""

    def __init__(
        self,
        frame_width: int,
        frame_height: int,
        *,
        horizontal_fov_deg: float = 120.0,
        voxel_size_m: float = 0.012,
        max_range_m: float = 2.5,
        camera_from_lidar: np.ndarray | None = None,
        motion_mode: str = "free",
    ):
        if motion_mode not in {"free", "pivot"}:
            raise ValueError("motion_mode must be 'free' or 'pivot'")
        self.visual = _VisualOdometry(frame_width, frame_height, horizontal_fov_deg)
        self.cloud = ColoredVoxelCloud(voxel_size_m=voxel_size_m)
        self.max_range_m = float(max_range_m)
        # Lidar frame: +X forward, +Y left, +Z up. OpenCV camera frame:
        # +X right, +Y down, +Z forward. Translation is provisional bracket
        # geometry (lidar is ~40 mm behind and ~55 mm above the camera).
        if camera_from_lidar is None:
            camera_from_lidar = np.asarray(
                ((0.0, -1.0, 0.0, 0.0), (0.0, 0.0, -1.0, -0.055), (1.0, 0.0, 0.0, -0.040), (0.0, 0.0, 0.0, 1.0)),
                dtype=np.float64,
            )
        self.camera_from_lidar = np.asarray(camera_from_lidar, dtype=np.float64)
        self.motion_mode = motion_mode
        self.world_from_camera = np.eye(4, dtype=np.float64)
        self.frame_index = 0
        self.accepted_frames = 0
        self.rejected_frames = 0
        self.trajectory: list[dict] = []

    def _sample_colors(self, frame_bgr: np.ndarray, points_camera: np.ndarray) -> np.ndarray:
        matrix = self.visual.camera_matrix
        z = points_camera[:, 2]
        projectable = z > 0.05
        u = np.full(len(points_camera), -1, dtype=np.int64)
        v = np.full(len(points_camera), -1, dtype=np.int64)
        u[projectable] = np.rint(matrix[0, 0] * points_camera[projectable, 0] / z[projectable] + matrix[0, 2]).astype(np.int64)
        v[projectable] = np.rint(matrix[1, 1] * points_camera[projectable, 1] / z[projectable] + matrix[1, 2]).astype(np.int64)
        height, width = frame_bgr.shape[:2]
        valid = projectable & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        rgb = np.full((len(points_camera), 3), 128, dtype=np.uint8)
        rgb[valid] = frame_bgr[v[valid], u[valid], ::-1]
        return rgb

    def _initial_pose_candidates(self, current_from_previous: np.ndarray) -> Iterable[tuple[float, np.ndarray]]:
        for scale in np.linspace(0.0, 0.24, 13):
            relative = current_from_previous.copy()
            relative[:3, 3] *= float(scale)
            yield float(scale), self.world_from_camera @ np.linalg.inv(relative)

    def add_frame(self, frame_bgr: np.ndarray, polar_points: Iterable[Sequence[float]], timestamp_s: float) -> ScanUpdate:
        frame = np.asarray(frame_bgr)
        self.frame_index += 1
        lidar_camera = lidar_polar_to_camera(
            polar_points,
            self.camera_from_lidar,
            max_range_m=self.max_range_m,
        )
        relative, features, inliers, _parallax_px = self.visual.update(frame)
        if len(lidar_camera) < 18:
            self.rejected_frames += 1
            return ScanUpdate(False, "YETERSIZ_LIDAR_NOKTASI", self.frame_index, len(self.cloud), len(lidar_camera), features, inliers, 0.0, None, None)

        colors = self._sample_colors(frame, lidar_camera)
        if self.accepted_frames == 0:
            world = transform_points(lidar_camera, self.world_from_camera)
            self.cloud.integrate(world, colors)
            self.accepted_frames = 1
            self.visual.commit()
            self.trajectory.append({"timestamp_s": timestamp_s, "transform": self.world_from_camera.tolist(), "fitness": 1.0, "rmse_m": 0.0})
            return ScanUpdate(True, "ILK_KARE", self.frame_index, len(self.cloud), len(lidar_camera), features, inliers, 1.0, 0.0, 0.0)
        if relative is None:
            self.rejected_frames += 1
            return ScanUpdate(False, "GORSEL_POZ_KAYIP", self.frame_index, len(self.cloud), len(lidar_camera), features, inliers, 0.0, None, None)

        if self.motion_mode == "pivot":
            rotation_angle = _rotation_angle(relative[:3, :3])
            if rotation_angle > math.radians(20.0):
                self.rejected_frames += 1
                return ScanUpdate(False, "PIVOT_DONUSU_COK_BUYUK", self.frame_index, len(self.cloud), len(lidar_camera), features, inliers, 0.0, None, 0.0)
            rotation_only = np.eye(4, dtype=np.float64)
            rotation_only[:3, :3] = relative[:3, :3]
            candidate_pose = self.world_from_camera @ np.linalg.inv(rotation_only)
            world = transform_points(lidar_camera, candidate_pose)
            self.world_from_camera = candidate_pose
            self.cloud.integrate(world, colors)
            self.accepted_frames += 1
            self.visual.commit()
            self.trajectory.append({
                "timestamp_s": float(timestamp_s),
                "transform": self.world_from_camera.tolist(),
                "fitness": None,
                "rmse_m": None,
                "visual_inliers": inliers,
                "translation_scale_m": 0.0,
                "motion_mode": "pivot",
            })
            return ScanUpdate(True, "PIVOT_ROTATION", self.frame_index, len(self.cloud), len(lidar_camera), features, inliers, 0.0, None, 0.0)

        target_xyz, _target_rgb, target_counts = self.cloud.arrays()
        stable_target = target_xyz[target_counts >= 1]
        best: tuple[float, ICPResult] | None = None
        for scale, initial in self._initial_pose_candidates(relative):
            result = icp_register(
                lidar_camera,
                stable_target,
                initial,
                max_correspondence_m=0.14,
                max_step_translation_m=0.16,
                max_step_rotation_deg=18.0,
            )
            score = result.fitness - min(result.rmse_m if math.isfinite(result.rmse_m) else 1.0, 1.0)
            if best is None or score > (best[1].fitness - min(best[1].rmse_m if math.isfinite(best[1].rmse_m) else 1.0, 1.0)):
                best = (scale, result)
        assert best is not None
        scale, registration = best
        bootstrap_accept = (
            self.accepted_frames < 4
            and registration.reason == "WEAK_GEOMETRIC_OVERLAP"
            and registration.correspondences >= 12
            and registration.fitness >= 0.55
            and math.isfinite(registration.rmse_m)
            and registration.rmse_m <= 0.060
            and inliers >= 60
        )
        if not registration.accepted and not bootstrap_accept:
            self.rejected_frames += 1
            return ScanUpdate(False, registration.reason, self.frame_index, len(self.cloud), len(lidar_camera), features, inliers, registration.fitness, registration.rmse_m if math.isfinite(registration.rmse_m) else None, scale)

        self.world_from_camera = registration.transform
        world = transform_points(lidar_camera, self.world_from_camera)
        self.cloud.integrate(world, colors)
        self.accepted_frames += 1
        self.visual.commit()
        self.trajectory.append({
            "timestamp_s": float(timestamp_s),
            "transform": self.world_from_camera.tolist(),
            "fitness": registration.fitness,
            "rmse_m": registration.rmse_m,
            "visual_inliers": inliers,
            "translation_scale_m": scale,
        })
        return ScanUpdate(True, "BOOTSTRAP" if bootstrap_accept else "OK", self.frame_index, len(self.cloud), len(lidar_camera), features, inliers, registration.fitness, registration.rmse_m, scale)

    def export(self, directory: str | Path) -> dict:
        output = Path(directory)
        output.mkdir(parents=True, exist_ok=True)
        ply = self.cloud.export_ply(output / "object-cloud.ply", minimum_observations=1)
        metadata = {
            "schema": "apex-object-scan-v1",
            "frames": self.frame_index,
            "accepted_frames": self.accepted_frames,
            "rejected_frames": self.rejected_frames,
            "points": len(self.cloud),
            "voxel_size_m": self.cloud.voxel_size_m,
            "camera_from_lidar": self.camera_from_lidar.tolist(),
            "motion_mode": self.motion_mode,
            "extrinsics_status": "PROVISIONAL",
            "intrinsics_status": "APPROXIMATE_FOV",
            "trajectory": self.trajectory,
        }
        (output / "scan.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        return {"ply": str(ply), **{key: value for key, value in metadata.items() if key != "trajectory"}}
