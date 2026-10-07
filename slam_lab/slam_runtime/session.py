"""Session files use wall time only for names/metadata; data uses monotonic ns."""

from __future__ import annotations

import csv
import json
import platform
import subprocess
import threading
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

    def update_sensor_metadata(self, sensor: str, info: dict[str, Any]) -> None:
        with self._metadata_lock:
            if sensor == "lidar":
                self.metadata["lidar_actual_serial"] = info.get("serial")
                self.metadata["lidar_firmware"] = info.get("firmware")
                self.metadata["lidar_device_info"] = info
            elif sensor == "esp32":
                self.metadata["esp32_serial_info"] = info
            else:
                raise ValueError(f"unknown sensor metadata: {sensor}")
            self.metadata_path.write_text(yaml.safe_dump(self.metadata), encoding="utf-8")

    def update_observed_rates(self, rates: dict[str, float]) -> None:
        with self._metadata_lock:
            self.metadata["sensor_rates_observed_hz"] = rates
            self.metadata_path.write_text(yaml.safe_dump(self.metadata), encoding="utf-8")

    def write_imu(self, sample: ImuSample) -> None:
        self.imu.writerow((
            sample.host_receive_ns, sample.esp_time_us,
            *sample.accel_raw_frame_m_s2, *sample.gyro_raw_frame_rad_s,
            *sample.mag_raw_frame_uT, sample.temperature_c,
            json.dumps(sample.raw_counts), sample.source_frame,
        ))
        self.imu_file.flush()

    def write_fused(self, timestamp_ns: int, quaternion, euler: tuple[float, float, float]) -> None:
        self.fused.writerow((timestamp_ns, quaternion.w, quaternion.x, quaternion.y, quaternion.z, *euler))
        self.fused_file.flush()

    def write_scan(self, scan: LidarScan) -> None:
        payload = {
            "timestamp_start_ns": scan.timestamp_start_ns,
            "timestamp_end_ns": scan.timestamp_end_ns,
            "sequence": scan.sequence,
            "frame": scan.frame,
            "points": [[p.angle_rad, p.range_m, p.quality] for p in scan.points],
        }
        self.scans_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
        self.scans_file.flush()

    def write_pose(self, timestamp_ns: int, pose, score: float, mapping_state: str) -> None:
        self.pose.writerow((timestamp_ns, pose.x_m, pose.y_m, pose.yaw_rad, score, mapping_state))
        self.pose_file.flush()

    def event(self, timestamp_ns: int, message: str) -> None:
        with self._event_lock:
            self.events_file.write(f"{timestamp_ns} {message}\n")
            self.events_file.flush()

    def close(self) -> None:
        for stream in (self.imu_file, self.fused_file, self.scans_file, self.pose_file, self.events_file):
            stream.flush()
            stream.close()
