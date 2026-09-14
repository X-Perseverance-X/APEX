"""Bounded SE(2) pose graph and scan-to-scan loop closure.

The online scan matcher supplies local odometry.  This module keeps sparse
keyframes, validates non-neighbouring scan revisits with ICP, relaxes the pose
graph, and returns corrected poses so the occupancy grid can be rebuilt once
mapping is finished.  It deliberately has no actuator or transport code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .contracts import LidarScan, Pose2D
from .scan_matcher import normalize_angle


def _pose_array(pose: Pose2D | np.ndarray) -> np.ndarray:
    if isinstance(pose, Pose2D):
        return np.asarray([pose.x_m, pose.y_m, pose.yaw_rad], dtype=np.float64)
    return np.asarray(pose, dtype=np.float64).copy()


def compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return ``T_world_a * T_a_b`` in compact x/y/yaw form."""
    c, s = math.cos(float(a[2])), math.sin(float(a[2]))
    return np.asarray([
        a[0] + c * b[0] - s * b[1],
        a[1] + s * b[0] + c * b[1],
        normalize_angle(float(a[2] + b[2])),
    ], dtype=np.float64)


def inverse(a: np.ndarray) -> np.ndarray:
    c, s = math.cos(float(a[2])), math.sin(float(a[2]))
    return np.asarray([
        -c * a[0] - s * a[1],
        s * a[0] - c * a[1],
        normalize_angle(float(-a[2])),
    ], dtype=np.float64)


def between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return compose(inverse(a), b)


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    c, s = math.cos(float(transform[2])), math.sin(float(transform[2]))
    rotation = np.asarray([[c, -s], [s, c]], dtype=np.float64)
    return points @ rotation.T + transform[:2]


def scan_xy(scan: LidarScan, max_points: int = 160) -> np.ndarray:
    points: list[tuple[float, float]] = []
    for angle_deg, distance_mm in scan.points:
        distance_m = float(distance_mm) / 1000.0
        if not math.isfinite(distance_m) or not 0.12 <= distance_m <= 6.0:
            continue
        angle = math.radians(float(angle_deg))
        points.append((distance_m * math.cos(angle), distance_m * math.sin(angle)))
    if not points:
        return np.empty((0, 2), dtype=np.float64)
    result = np.asarray(points, dtype=np.float64)
    if len(result) > max_points:
        indices = np.linspace(0, len(result) - 1, max_points, dtype=np.int32)
        result = result[indices]
    return result


@dataclass(frozen=True)
class GraphConfig:
    keyframe_translation_m: float = 0.30
    keyframe_rotation_rad: float = math.radians(10.0)
    keyframe_interval_s: float = 2.0
    max_keyframes: int = 180
    max_observations: int = 12000
    min_loop_separation: int = 12
    min_loop_path_length_m: float = 1.50
    loop_search_radius_m: float = 1.20
    loop_measurement_translation_max_m: float = 0.65
    loop_max_pair_distance_m: float = 0.35
    loop_min_inlier_ratio: float = 0.52
    loop_max_rmse_m: float = 0.13
    loop_max_correction_m: float = 0.80
    loop_max_correction_rad: float = math.radians(28.0)
    max_loop_constraints: int = 12
    max_loop_candidate_checks: int = 72
    relaxation_iterations: int = 70


@dataclass
class Keyframe:
    pose: np.ndarray
    scan: LidarScan
    points: np.ndarray
    observation_index: int


@dataclass
class ScanObservation:
    pose: np.ndarray
    timestamp_ns: int
    sequence: int
    polar_points: np.ndarray
    frame_id: str


@dataclass(frozen=True)
class Constraint:
    source: int
    target: int
    relative_pose: np.ndarray
    weight: float
    kind: str
    rmse_m: float = 0.0
    inlier_ratio: float = 1.0


class PoseGraph:
    """Small deterministic graph suitable for Raspberry Pi map finalization."""

    def __init__(self, config: GraphConfig | None = None):
        self.config = config or GraphConfig()
        self.keyframes: list[Keyframe] = []
        self.observations: list[ScanObservation] = []
        self.constraints: list[Constraint] = []
        self.loop_candidates_checked = 0
        self.observation_overflow = False
        self.optimized = False
        self.correction_translation_m = 0.0
        self.correction_yaw_rad = 0.0

    def clear(self) -> None:
        self.keyframes.clear()
        self.observations.clear()
        self.constraints.clear()
        self.loop_candidates_checked = 0
        self.observation_overflow = False
        self.optimized = False
        self.correction_translation_m = 0.0
        self.correction_yaw_rad = 0.0

    def record_scan(self, pose: Pose2D, scan: LidarScan) -> int | None:
        """Keep every accepted mapping scan in compact form for safe rebuilding."""
        if len(self.observations) >= self.config.max_observations:
            self.observation_overflow = True
            return None
        polar = np.asarray(scan.points, dtype=np.float32)
        if polar.ndim != 2 or polar.shape[1:] != (2,):
            return None
        valid = np.isfinite(polar).all(axis=1)
        polar = polar[valid]
        if len(polar) < 1:
            return None
        if len(polar) > 240:
            indices = np.linspace(0, len(polar) - 1, 240, dtype=np.int32)
            polar = polar[indices]
        self.observations.append(ScanObservation(
            _pose_array(pose), int(scan.timestamp_ns), int(scan.sequence), polar.copy(), str(scan.frame_id),
        ))
        return len(self.observations) - 1

    def maybe_add_keyframe(
        self, pose: Pose2D, scan: LidarScan, force: bool = False, observation_index: int | None = None,
    ) -> bool:
        points = scan_xy(scan)
        if len(points) < 24:
            return False
        current = _pose_array(pose)
        if self.keyframes and not force:
            previous = self.keyframes[-1]
            delta = between(previous.pose, current)
            elapsed_s = max(0.0, (scan.timestamp_ns - previous.scan.timestamp_ns) / 1e9)
            if (math.hypot(float(delta[0]), float(delta[1])) < self.config.keyframe_translation_m
                    and abs(float(delta[2])) < self.config.keyframe_rotation_rad
                    and elapsed_s < self.config.keyframe_interval_s):
                return False
        if len(self.keyframes) >= self.config.max_keyframes:
            return False
        if observation_index is None:
            observation_index = self.record_scan(pose, scan)
        if observation_index is None:
            return False
        if self.keyframes:
            source = len(self.keyframes) - 1
            relative = between(self.keyframes[-1].pose, current)
            self.constraints.append(Constraint(source, source + 1, relative, 1.0, "odometry"))
        self.keyframes.append(Keyframe(current, scan, points, int(observation_index)))
        return True

    @staticmethod
    def _rigid_fit(source: np.ndarray, target: np.ndarray) -> np.ndarray | None:
        if len(source) < 3 or len(source) != len(target):
            return None
        source_mean, target_mean = source.mean(axis=0), target.mean(axis=0)
        covariance = (source - source_mean).T @ (target - target_mean)
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1, :] *= -1
            rotation = vt.T @ u.T
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
        translation = target_mean - rotation @ source_mean
        return np.asarray([translation[0], translation[1], yaw], dtype=np.float64)

    def _icp(self, reference: Keyframe, current: Keyframe) -> tuple[np.ndarray, float, float]:
        transform = between(reference.pose, current.pose)
        target = reference.points
        max_distance_sq = self.config.loop_max_pair_distance_m ** 2
        ratio, rmse = 0.0, math.inf
        for _ in range(12):
            moved = transform_points(current.points, transform)
            distances = ((moved[:, None, :] - target[None, :, :]) ** 2).sum(axis=2)
            nearest = np.argmin(distances, axis=1)
            best_sq = distances[np.arange(len(moved)), nearest]
            mask = best_sq <= max_distance_sq
            ratio = float(np.count_nonzero(mask)) / max(1, len(moved))
            if np.count_nonzero(mask) < 18:
                return transform, math.inf, ratio
            delta = self._rigid_fit(moved[mask], target[nearest[mask]])
            if delta is None:
                return transform, math.inf, ratio
            transform = compose(delta, transform)
            rmse = float(np.sqrt(np.mean(best_sq[mask])))
            if math.hypot(float(delta[0]), float(delta[1])) < 0.002 and abs(float(delta[2])) < math.radians(0.08):
                break
        moved = transform_points(current.points, transform)
        distances = ((moved[:, None, :] - target[None, :, :]) ** 2).sum(axis=2)
        best_sq = np.min(distances, axis=1)
        mask = best_sq <= max_distance_sq
        ratio = float(np.count_nonzero(mask)) / max(1, len(moved))
        rmse = float(np.sqrt(np.mean(best_sq[mask]))) if np.any(mask) else math.inf
        return transform, rmse, ratio

    @staticmethod
    def _shape_distance(reference: Keyframe, current: Keyframe) -> float:
        """Cheap rotation-invariant gate before the quadratic ICP step."""
        sample_count = 48
        ref_radius = np.linalg.norm(reference.points, axis=1)
        cur_radius = np.linalg.norm(current.points, axis=1)
        quantiles = np.linspace(0.0, 1.0, sample_count)
        ref_profile = np.quantile(ref_radius, quantiles)
        cur_profile = np.quantile(cur_radius, quantiles)
        scale = max(0.25, float(np.mean(ref_profile)), float(np.mean(cur_profile)))
        return float(np.sqrt(np.mean((ref_profile - cur_profile) ** 2)) / scale)

    def detect_loop_closures(self) -> int:
        """Validate at most one old revisit per keyframe; never trust proximity alone."""
        if self.observation_overflow:
            return 0
        existing = {(edge.source, edge.target) for edge in self.constraints if edge.kind == "loop"}
        added = 0
        checks = 0
        cumulative_distance = [0.0]
        for previous, current in zip(self.keyframes, self.keyframes[1:]):
            cumulative_distance.append(cumulative_distance[-1] + float(np.linalg.norm(current.pose[:2] - previous.pose[:2])))
        # The most useful closure is normally the final return toward the map
        # origin, so inspect recent keyframes first under a strict CPU budget.
        for target_index in range(len(self.keyframes) - 1, self.config.min_loop_separation - 1, -1):
            if added >= self.config.max_loop_constraints:
                break
            current = self.keyframes[target_index]
            candidates: list[tuple[float, int]] = []
            for source_index in range(0, target_index - self.config.min_loop_separation + 1):
                if (source_index, target_index) in existing:
                    continue
                reference = self.keyframes[source_index]
                distance = float(np.linalg.norm(reference.pose[:2] - current.pose[:2]))
                travelled = cumulative_distance[target_index] - cumulative_distance[source_index]
                if distance <= self.config.loop_search_radius_m and travelled >= self.config.min_loop_path_length_m:
                    candidates.append((distance, source_index))
            best: tuple[float, float, int, np.ndarray] | None = None
            for _, source_index in sorted(candidates)[:4]:
                if checks >= self.config.max_loop_candidate_checks:
                    break
                if self._shape_distance(self.keyframes[source_index], current) > 0.24:
                    continue
                checks += 1
                self.loop_candidates_checked += 1
                measurement, rmse, ratio = self._icp(self.keyframes[source_index], current)
                predicted = between(self.keyframes[source_index].pose, current.pose)
                correction = between(predicted, measurement)
                if (ratio < self.config.loop_min_inlier_ratio or rmse > self.config.loop_max_rmse_m
                        or math.hypot(float(measurement[0]), float(measurement[1])) > self.config.loop_measurement_translation_max_m
                        or math.hypot(float(correction[0]), float(correction[1])) > self.config.loop_max_correction_m
                        or abs(float(correction[2])) > self.config.loop_max_correction_rad):
                    continue
                candidate = (rmse, -ratio, source_index, measurement)
                if best is None or candidate[:2] < best[:2]:
                    best = candidate
            if best is not None:
                rmse, negative_ratio, source_index, measurement = best
                weight = max(0.25, min(0.75, (-negative_ratio) * (1.0 - rmse / self.config.loop_max_rmse_m)))
                self.constraints.append(Constraint(source_index, target_index, measurement, weight, "loop", rmse, -negative_ratio))
                existing.add((source_index, target_index))
                added += 1
            if checks >= self.config.max_loop_candidate_checks:
                break
        return added

    def add_loop_constraint(self, source: int, target: int, relative_pose: np.ndarray, weight: float = 0.6) -> None:
        """Test/calibration hook; callers must provide an already validated constraint."""
        if not 0 <= source < target < len(self.keyframes):
            raise ValueError("invalid loop-constraint indices")
        self.constraints.append(Constraint(source, target, _pose_array(relative_pose), float(weight), "loop"))

    def optimize(self) -> list[np.ndarray]:
        poses = [frame.pose.copy() for frame in self.keyframes]
        loops = [edge for edge in self.constraints if edge.kind == "loop"]
        if self.observation_overflow or len(poses) < 2 or not loops:
            return poses
        before = [pose.copy() for pose in poses]
        for iteration in range(self.config.relaxation_iterations):
            gain = 0.34 * (1.0 - 0.65 * iteration / max(1, self.config.relaxation_iterations - 1))
            for edge in self.constraints:
                expected_target = compose(poses[edge.source], edge.relative_pose)
                translation_error = expected_target[:2] - poses[edge.target][:2]
                yaw_error = normalize_angle(float(expected_target[2] - poses[edge.target][2]))
                alpha = gain * edge.weight
                if edge.source == 0:
                    poses[edge.target][:2] += alpha * translation_error
                    poses[edge.target][2] = normalize_angle(float(poses[edge.target][2] + alpha * yaw_error))
                else:
                    poses[edge.source][:2] -= 0.5 * alpha * translation_error
                    poses[edge.source][2] = normalize_angle(float(poses[edge.source][2] - 0.5 * alpha * yaw_error))
                    poses[edge.target][:2] += 0.5 * alpha * translation_error
                    poses[edge.target][2] = normalize_angle(float(poses[edge.target][2] + 0.5 * alpha * yaw_error))
            poses[0] = before[0].copy()
        corrections = [between(old, new) for old, new in zip(before, poses)]
        self.correction_translation_m = max(math.hypot(float(p[0]), float(p[1])) for p in corrections)
        self.correction_yaw_rad = max(abs(float(p[2])) for p in corrections)
        self.optimized = True
        for frame, pose in zip(self.keyframes, poses):
            frame.pose = pose.copy()
        return poses

    def finalize(self) -> list[np.ndarray]:
        self.detect_loop_closures()
        return self.optimize()

    @staticmethod
    def _blend_pose(a: np.ndarray, b: np.ndarray, alpha: float) -> np.ndarray:
        alpha = min(1.0, max(0.0, float(alpha)))
        yaw_delta = normalize_angle(float(b[2] - a[2]))
        return np.asarray([
            (1.0 - alpha) * a[0] + alpha * b[0],
            (1.0 - alpha) * a[1] + alpha * b[1],
            normalize_angle(float(a[2] + alpha * yaw_delta)),
        ], dtype=np.float64)

    def corrected_observations(self, corrected_keyframes: list[np.ndarray]):
        """Yield all accepted scans with graph correction interpolated in SE(2)."""
        if len(corrected_keyframes) != len(self.keyframes):
            raise ValueError("corrected keyframe count mismatch")
        if not self.keyframes:
            return
        segment = 0
        for observation_index, observation in enumerate(self.observations):
            while (segment + 1 < len(self.keyframes)
                   and self.keyframes[segment + 1].observation_index < observation_index):
                segment += 1
            left = self.keyframes[segment]
            left_prediction = compose(corrected_keyframes[segment], between(left.pose, observation.pose))
            if segment + 1 >= len(self.keyframes):
                corrected = left_prediction
            else:
                right = self.keyframes[segment + 1]
                right_prediction = compose(corrected_keyframes[segment + 1], between(right.pose, observation.pose))
                width = max(1, right.observation_index - left.observation_index)
                alpha = (observation_index - left.observation_index) / width
                corrected = self._blend_pose(left_prediction, right_prediction, alpha)
            yield observation, corrected

    @property
    def loop_count(self) -> int:
        return sum(edge.kind == "loop" for edge in self.constraints)

    def status(self) -> dict:
        return {
            "keyframes": len(self.keyframes),
            "observations": len(self.observations),
            "observation_overflow": self.observation_overflow,
            "constraints": len(self.constraints),
            "loop_closures": self.loop_count,
            "candidates_checked": self.loop_candidates_checked,
            "optimized": self.optimized,
            "correction_translation_m": round(self.correction_translation_m, 3),
            "correction_yaw_deg": round(math.degrees(self.correction_yaw_rad), 2),
            "state": "OPTIMIZED" if self.optimized else ("COLLECTING" if self.keyframes else "WAITING"),
        }
