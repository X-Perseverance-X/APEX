"""Thread-safe mapping and route-decision engine for guarded execution."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import asdict

from .contracts import ImuSample, LidarScan, Pose2D, TofSample
from .occupancy import GridConfig, OccupancyGrid, rle_encode
from .planner import PlanResult, plan_route
from .scan_matcher import CorrelativeScanMatcher, MatchResult, normalize_angle


class MappingEngine:
    """Sensor/route state only; actuator transport remains in apex_server."""

    EXECUTION_LOCKED = False

    def __init__(self, config: GridConfig | None = None):
        self._lock = threading.RLock()
        self.grid = OccupancyGrid(config)
        self.pose = Pose2D()
        self.goal: tuple[float, float] | None = None
        self.plan: PlanResult | None = None
        self.approval_mode = "manual"
        self.approved = False
        self.decision = "NO_GOAL"
        self.last_lidar_ns = 0
        self.last_map_update_ns = 0
        self.last_imu: ImuSample | None = None
        self.last_tof: TofSample | None = None
        self.last_camera_ns = 0
        self.camera_healthy = False
        self._revision = 0
        # model_parameters.yaml içindeki sensör pozları hâlâ provisional.
        # Ölçülmüş dış kalibrasyon gelmeden 3D/renk füzyonu güvenilir sayılmaz.
        self.extrinsics_trusted = False
        self.camera_calibrated = False
        self.scan_matcher = CorrelativeScanMatcher()
        self.last_match = MatchResult(False, self.pose, 0.0, 0.0, 0, reason="WAITING_FIRST_SCAN")
        self._last_lidar_imu_yaw_deg: float | None = None
        self._imu_yaw_sign: int | None = None
        self._imu_sign_candidate: int | None = None
        self._imu_sign_votes = 0
        self._accepted_matches = 0
        self._rejected_matches = 0
        self._last_input_key: tuple[int, int] | None = None

    def ingest_lidar(self, scan: LidarScan) -> bool:
        with self._lock:
            input_key = (int(scan.sequence), int(scan.timestamp_ns))
            if input_key == self._last_input_key:
                return False
            self._last_input_key = input_key

            imu_delta_rad: float | None = None
            current_imu_yaw: float | None = None
            if self.last_imu and self.last_imu.valid:
                current_imu_yaw = float(self.last_imu.yaw_deg)
                if self._last_lidar_imu_yaw_deg is not None:
                    delta_deg = (current_imu_yaw - self._last_lidar_imu_yaw_deg + 180.0) % 360.0 - 180.0
                    if abs(delta_deg) <= 60.0:
                        imu_delta_rad = math.radians(delta_deg)

            first_scan = self.grid.scan_count == 0
            if first_scan:
                changed = self.grid.update_lidar(scan, self.pose)
                self.last_match = MatchResult(False, self.pose, 0.0, 0.0, len(scan.points), reason="INITIALIZING_SUBMAP")
            else:
                match = self.scan_matcher.match(
                    scan, self.grid, self.pose, imu_yaw_delta_rad=imu_delta_rad, imu_sign=self._imu_yaw_sign,
                )
                self.last_match = match
                if match.accepted:
                    self._accepted_matches += 1
                    self._rejected_matches = 0
                    if match.imu_sign in (-1, 1) and imu_delta_rad is not None and abs(imu_delta_rad) >= math.radians(0.5):
                        if self._imu_sign_candidate == match.imu_sign:
                            self._imu_sign_votes += 1
                        else:
                            self._imu_sign_candidate = match.imu_sign
                            self._imu_sign_votes = 1
                        if self._imu_sign_votes >= 3:
                            self._imu_yaw_sign = match.imu_sign
                    self.pose = Pose2D(
                        x_m=match.pose.x_m,
                        y_m=match.pose.y_m,
                        yaw_rad=normalize_angle(match.pose.yaw_rad),
                        trusted=self._accepted_matches >= 3,
                        source="lidar_imu_correlative",
                    )
                    changed = self.grid.update_lidar(scan, self.pose)
                elif match.reason == "INITIALIZING_SUBMAP":
                    # Bootstrap only: build enough geometry for matching. Once
                    # the submap is mature, rejected scans are never overlaid.
                    changed = self.grid.update_lidar(scan, self.pose)
                else:
                    self._rejected_matches += 1
                    changed = False

            if current_imu_yaw is not None:
                self._last_lidar_imu_yaw_deg = current_imu_yaw
            self.last_lidar_ns = scan.timestamp_ns
            if changed:
                self.last_map_update_ns = scan.timestamp_ns
                if self.goal is not None:
                    self._replan_locked(preserve_approval_if_same=True)
                self._revision += 1
            elif not first_scan and (
                self._rejected_matches in {1, 3} or self._rejected_matches % 10 == 0
            ):
                # Publish tracking/rejection diagnostics even when a weak scan
                # was intentionally kept out of the probability grid. Rate
                # limit repeated identical failures to avoid UI churn.
                self._revision += 1
            return changed

    def ingest_imu(self, sample: ImuSample) -> None:
        with self._lock:
            self.last_imu = sample
            # MPU6050 yaw drifts and cannot become map yaw without scan/visual
            # odometry. Roll/pitch are retained for the future 3D projector.

    def ingest_tof(self, sample: TofSample) -> None:
        with self._lock:
            self.last_tof = sample
            self.grid.update_tof(sample, self.pose)

    def set_camera_health(self, healthy: bool, timestamp_ns: int | None = None) -> None:
        with self._lock:
            changed = self.camera_healthy != bool(healthy)
            self.camera_healthy = bool(healthy)
            self.last_camera_ns = int(timestamp_ns or time.monotonic_ns())
            if changed:
                self._revision += 1

    def revision(self) -> int:
        """Return a cheap monotonic state revision for event-driven publishers."""
        with self._lock:
            return self._revision

    def set_goal(self, x_m: float, y_m: float) -> dict:
        if not all(math.isfinite(v) for v in (x_m, y_m)):
            raise ValueError("goal coordinates must be finite")
        with self._lock:
            self.goal = (round(float(x_m), 3), round(float(y_m), 3))
            self.approved = False
            self._replan_locked()
            self._revision += 1
            return self._plan_status_locked()

    def set_approval_mode(self, mode: str) -> dict:
        if mode not in {"manual", "auto"}:
            raise ValueError("mode must be manual or auto")
        with self._lock:
            self.approval_mode = mode
            self.approved = False
            if self.goal is not None:
                self._replan_locked()
            self._revision += 1
            return self._plan_status_locked()

    def approve_preview(self) -> dict:
        with self._lock:
            if not self.plan or not self.plan.ok:
                raise ValueError("there is no valid route to approve")
            self.approved = True
            self.decision = "APPROVED_FOR_EXECUTION"
            self._revision += 1
            return self._plan_status_locked()

    def clear_goal(self) -> dict:
        with self._lock:
            self.goal = None
            self.plan = None
            self.approved = False
            self.decision = "NO_GOAL"
            self._revision += 1
            return self._plan_status_locked()

    def clear_map(self) -> None:
        with self._lock:
            self.grid.clear()
            self.pose = Pose2D()
            self.last_map_update_ns = 0
            self.last_match = MatchResult(False, self.pose, 0.0, 0.0, 0, reason="WAITING_FIRST_SCAN")
            self._last_lidar_imu_yaw_deg = None
            self._imu_yaw_sign = None
            self._imu_sign_candidate = None
            self._imu_sign_votes = 0
            self._accepted_matches = 0
            self._rejected_matches = 0
            self._last_input_key = None
            self.goal = None
            self.plan = None
            self.approved = False
            self.decision = "NO_GOAL"
            self._revision += 1

    @staticmethod
    def _route_signature(plan: PlanResult | None) -> tuple[tuple[float, float], ...]:
        if not plan or not plan.ok:
            return ()
        return tuple((round(x, 2), round(y, 2)) for x, y in plan.path_world)

    def _replan_locked(self, preserve_approval_if_same: bool = False) -> None:
        if self.goal is None:
            return
        old_signature = self._route_signature(self.plan)
        was_approved = self.approved
        self.plan = plan_route(self.grid, (self.pose.x_m, self.pose.y_m), self.goal)
        same_route = old_signature == self._route_signature(self.plan)
        self.approved = bool(preserve_approval_if_same and was_approved and same_route)
        if not self.plan.ok:
            self.decision = self.plan.status
        elif self.approval_mode == "manual":
            self.decision = "APPROVED_FOR_EXECUTION" if self.approved else "WAITING_OPERATOR_APPROVAL"
        elif self.plan.unknown_ratio <= 0.35:
            self.approved = True
            self.decision = "AUTO_APPROVED_FOR_EXECUTION"
        else:
            self.decision = "AUTO_REVIEW_UNKNOWN_SPACE"

    def _plan_status_locked(self) -> dict:
        plan = self.plan
        return {
            "goal": None if self.goal is None else {"x_m": self.goal[0], "y_m": self.goal[1]},
            "route": [] if not plan else [[round(x, 3), round(y, 3)] for x, y in plan.path_world],
            "route_ok": bool(plan and plan.ok),
            "route_status": None if not plan else plan.status,
            "route_message": None if not plan else plan.message,
            "route_length_m": 0.0 if not plan else plan.length_m,
            "unknown_ratio": 1.0 if not plan else plan.unknown_ratio,
            "expanded_nodes": 0 if not plan else plan.expanded_nodes,
            "approval_mode": self.approval_mode,
            "approved": self.approved,
            "decision": self.decision,
            "execution_locked": self.EXECUTION_LOCKED,
        }

    @staticmethod
    def _age_s(timestamp_ns: int) -> float | None:
        if not timestamp_ns:
            return None
        return round(max(0.0, (time.monotonic_ns() - timestamp_ns) / 1e9), 3)

    def snapshot(self, include_grid: bool = True) -> dict:
        with self._lock:
            states = self.grid.states()
            observed = int(self.grid.observed.sum())
            occupied = int((states >= 100).sum())
            result = {
                "type": "navigation",
                "schema": "apex.mapping.v2",
                "map_mode": "LOCAL_ONLY" if not self.pose.trusted else "ODOMETRY",
                "pose": asdict(self.pose),
                "grid": {
                    "width": self.grid.config.width,
                    "height": self.grid.config.height,
                    "resolution_m": self.grid.config.resolution_m,
                    "origin_x_m": self.grid.origin_x_m,
                    "origin_y_m": self.grid.origin_y_m,
                    "scan_count": self.grid.scan_count,
                    "observed_cells": observed,
                    "occupied_cells": occupied,
                },
                "sensors": {
                    "lidar": {"fresh": self._age_s(self.last_lidar_ns) is not None and self._age_s(self.last_lidar_ns) < 1.0, "age_s": self._age_s(self.last_lidar_ns)},
                    "imu": {
                        "valid": bool(self.last_imu and self.last_imu.valid),
                        "age_s": None if not self.last_imu else self._age_s(self.last_imu.timestamp_ns),
                        "roll_deg": None if not self.last_imu else self.last_imu.roll_deg,
                        "pitch_deg": None if not self.last_imu else self.last_imu.pitch_deg,
                        "yaw_deg": None if not self.last_imu else self.last_imu.yaw_deg,
                    },
                    "tof": {"valid": bool(self.last_tof and self.last_tof.valid), "distance_mm": None if not self.last_tof else self.last_tof.distance_mm, "age_s": None if not self.last_tof else self._age_s(self.last_tof.timestamp_ns)},
                    "camera": {"healthy": self.camera_healthy, "age_s": self._age_s(self.last_camera_ns)},
                },
                "local_slam": {
                    "state": "TRACKING" if self.last_match.accepted else self.last_match.reason,
                    "score": round(self.last_match.score, 3),
                    "in_bounds_ratio": round(self.last_match.in_bounds_ratio, 3),
                    "points_used": self.last_match.points_used,
                    "accepted_matches": self._accepted_matches,
                    "consecutive_rejections": self._rejected_matches,
                    "imu_yaw_sign": self._imu_yaw_sign,
                    "imu_sign_votes": self._imu_sign_votes,
                    "map_age_s": self._age_s(self.last_map_update_ns),
                },
                "fusion_readiness": {
                    "navigation_2d": self._age_s(self.last_map_update_ns) is not None
                    and self._age_s(self.last_map_update_ns) < 1.0
                    and self._rejected_matches < 3,
                    "global_slam": self.pose.trusted,
                    "sparse_3d": bool(self.last_imu and self.last_imu.valid and self.extrinsics_trusted),
                    "camera_semantic": bool(self.camera_healthy and self.camera_calibrated and self.extrinsics_trusted),
                    "blockers": [
                        label for blocked, label in (
                            (not self.pose.trusted, "GÜVENİLİR ODOMETRİ YOK"),
                            (not (self.last_imu and self.last_imu.valid), "IMU YOK/GEÇERSİZ"),
                            (not self.extrinsics_trusted, "SENSÖR DIŞ KALİBRASYONU PROVISIONAL"),
                            (not self.camera_calibrated, "KAMERA INTRINSICS KALİBRE DEĞİL"),
                        ) if blocked
                    ],
                },
                **self._plan_status_locked(),
            }
            if include_grid:
                result["grid"]["encoding"] = "rle-v1"
                result["grid"]["cells_rle"] = rle_encode(states)
            return result
