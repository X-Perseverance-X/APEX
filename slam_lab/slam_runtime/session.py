"""Session files use wall time only for names/metadata; data uses monotonic ns."""

from __future__ import annotations

import csv
import json
import platform
import shutil
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from drivers.imu_serial.protocol import ImuSample
from drivers.lidar.scan import LidarScan


def _git_commit(project_root: Path) -> str | None:
    result = subprocess.run(
        ["git", "-C", str(project_root), "rev-parse", "HEAD"],
        capture_output=True, text=True, check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


class SessionWriter:
    def __init__(self, project_root: Path, hardware: dict, frames: dict, calibration: dict) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        directory = project_root / "sessions" / stamp
        suffix = 1
        while directory.exists():
            directory = project_root / "sessions" / f"{stamp}_{suffix}"
            suffix += 1
        directory.mkdir(parents=True)
        self.directory = directory
        self._event_lock = threading.Lock()
        self._metadata_lock = threading.Lock()
        limits = hardware.get("logging", {})
        self.max_session_bytes = int(float(limits.get("max_session_mb", 512)) * 1024 * 1024)
        self.min_free_bytes = int(float(limits.get("min_free_mb", 512)) * 1024 * 1024)
        if self.max_session_bytes <= 0 or self.min_free_bytes <= 0:
            raise ValueError("logging limits must be positive")
        self.active = True
        self.pause_reason: str | None = None
        self._last_space_check = 0.0

        (directory / "hardware_snapshot.yaml").write_text(yaml.safe_dump(hardware), encoding="utf-8")
        (directory / "frames_snapshot.yaml").write_text(yaml.safe_dump(frames), encoding="utf-8")
        (directory / "calibration_snapshot.yaml").write_text(yaml.safe_dump(calibration), encoding="utf-8")
        self.metadata: dict[str, Any] = {
            "started_wall_time": datetime.now().astimezone().isoformat(),
            "os": platform.platform(),
            "python": platform.python_version(),
            "git_commit": _git_commit(project_root),
            "coordinate_convention": frames.get("convention"),
            "extrinsics": frames.get("imu_link"),
            "lidar_expected_serial": hardware["lidar"].get("expected_serial"),
            "lidar_firmware": None,
            "lidar_actual_serial": None,
            "esp32_serial_info": {"port": hardware["imu"].get("port")},
            "sensor_rates_target_hz": {"imu": hardware["imu"].get("sample_rate_hz")},
            "calibration": calibration,
            "recording_status": "ACTIVE",
        }
        self.metadata_path = directory / "metadata.yaml"
        self.metadata_path.write_text(yaml.safe_dump(self.metadata), encoding="utf-8")

        self.imu_file = (directory / "imu_raw.csv").open("w", newline="", encoding="utf-8")
        self.imu = csv.writer(self.imu_file)
        self.imu.writerow((
            "host_receive_ns", "esp_time_us", "ax_m_s2", "ay_m_s2", "az_m_s2",
            "gx_rad_s", "gy_rad_s", "gz_rad_s", "mx_uT", "my_uT", "mz_uT",
            "temperature_c", "raw_counts_json", "source_frame",
        ))
        self.imu_file.flush()
        self.fused_file = (directory / "imu_fused.csv").open("w", newline="", encoding="utf-8")
        self.fused = csv.writer(self.fused_file)
        self.fused.writerow(("host_receive_ns", "qw", "qx", "qy", "qz", "roll_rad", "pitch_rad", "yaw_relative_rad"))
        self.fused_file.flush()
        self.scans_file = (directory / "lidar_scans.jsonl").open("w", encoding="utf-8")
        self.pose_file = (directory / "pose.csv").open("w", newline="", encoding="utf-8")
        self.pose = csv.writer(self.pose_file)
        self.pose.writerow(("timestamp_ns", "x_m", "y_m", "yaw_rad", "score", "mapping_state"))
        self.pose_file.flush()
        self.events_file = (directory / "events.log").open("w", encoding="utf-8")

    def _pause(self, reason: str) -> None:
        if not self.active:
            return
        self.active = False
        self.pause_reason = reason
        with self._metadata_lock:
            self.metadata["recording_status"] = f"PAUSED: {reason}"
            self.metadata["recording_stop_wall_time"] = datetime.now().astimezone().isoformat()
            try:
                self.metadata_path.write_text(yaml.safe_dump(self.metadata), encoding="utf-8")
            except OSError:
                pass  # Keep the live mapper running even if the filesystem is full.

    def _allow(self) -> bool:
        if not self.active:
            return False
        now = time.monotonic()
        if now - self._last_space_check < 5.0:
            return True
        self._last_space_check = now
        try:
            session_bytes = sum(p.stat().st_size for p in self.directory.iterdir() if p.is_file())
            free_bytes = shutil.disk_usage(self.directory).free
        except OSError as error:
            self._pause(f"STORAGE CHECK FAILED: {error}")
            return False
        if session_bytes >= self.max_session_bytes:
            self._pause("SESSION SIZE LIMIT")
        elif free_bytes <= self.min_free_bytes:
            self._pause("LOW DISK SPACE")
        return self.active

    def _write(self, stream, write) -> None:
        if not self._allow():
            return
        try:
            write()
            stream.flush()
        except OSError as error:
            self._pause(f"WRITE FAILED: {error}")

    def update_sensor_metadata(self, sensor: str, info: dict[str, Any]) -> None:
        if not self._allow():
            return
        with self._metadata_lock:
            if sensor == "lidar":
                self.metadata["lidar_actual_serial"] = info.get("serial")
                self.metadata["lidar_firmware"] = info.get("firmware")
                self.metadata["lidar_device_info"] = info
            elif sensor == "esp32":
                self.metadata["esp32_serial_info"] = info
            else:
                raise ValueError(f"unknown sensor metadata: {sensor}")
            try:
                self.metadata_path.write_text(yaml.safe_dump(self.metadata), encoding="utf-8")
            except OSError as error:
                self.active = False
                self.pause_reason = f"METADATA WRITE FAILED: {error}"

    def update_observed_rates(self, rates: dict[str, float]) -> None:
        if not self._allow():
            return
        with self._metadata_lock:
            self.metadata["sensor_rates_observed_hz"] = rates
            try:
                self.metadata_path.write_text(yaml.safe_dump(self.metadata), encoding="utf-8")
            except OSError:
                pass

    def write_imu(self, sample: ImuSample) -> None:
        self._write(self.imu_file, lambda: self.imu.writerow((
            sample.host_receive_ns, sample.esp_time_us,
            *sample.accel_raw_frame_m_s2, *sample.gyro_raw_frame_rad_s,
            *sample.mag_raw_frame_uT, sample.temperature_c,
            json.dumps(sample.raw_counts), sample.source_frame,
        )))

    def write_fused(self, timestamp_ns: int, quaternion, euler: tuple[float, float, float]) -> None:
        self._write(self.fused_file, lambda: self.fused.writerow(
            (timestamp_ns, quaternion.w, quaternion.x, quaternion.y, quaternion.z, *euler)))

    def write_scan(self, scan: LidarScan) -> None:
        payload = {
            "timestamp_start_ns": scan.timestamp_start_ns,
            "timestamp_end_ns": scan.timestamp_end_ns,
            "sequence": scan.sequence,
            "frame": scan.frame,
            "points": [[p.angle_rad, p.range_m, p.quality] for p in scan.points],
        }
        self._write(self.scans_file, lambda: self.scans_file.write(
            json.dumps(payload, separators=(",", ":")) + "\n"))

    def write_pose(self, timestamp_ns: int, pose, score: float, mapping_state: str) -> None:
        self._write(self.pose_file, lambda: self.pose.writerow(
            (timestamp_ns, pose.x_m, pose.y_m, pose.yaw_rad, score, mapping_state)))

    def event(self, timestamp_ns: int, message: str) -> None:
        with self._event_lock:
            self._write(self.events_file, lambda: self.events_file.write(f"{timestamp_ns} {message}\n"))

    def close(self) -> None:
        for stream in (self.imu_file, self.fused_file, self.scans_file, self.pose_file, self.events_file):
            try:
                stream.flush()
            except OSError:
                pass
            stream.close()
