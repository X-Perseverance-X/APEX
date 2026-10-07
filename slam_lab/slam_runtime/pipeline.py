"""Live acquisition and processing with bounded sensor queues."""

from __future__ import annotations

import json
import math
import os
import queue
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import serial
import yaml

from core.attitude.mahony import AttitudeEstimator
from core.frames.axis_map import AxisMap
from core.frames.geometry import Pose2, wrap_angle
from core.mapping.gate import mapping_pause_reason
from core.mapping.occupancy import OccupancyGrid
from core.scan_matching.correlative import CorrelativeMatcher
from core.timing.clock import EspClock
from drivers.imu_serial.protocol import ImuSample, parse_imu_line
from drivers.imu_serial.port import open_esp32_serial
from drivers.lidar.scan import LidarPoint, LidarScan, normalize_scan
from slam_runtime.session import SessionWriter


ROOT = Path(__file__).resolve().parents[1]


def load_configuration(root: Path = ROOT) -> tuple[dict, dict, dict]:
    def read(name: str) -> dict:
        return yaml.safe_load((root / "config" / name).read_text(encoding="utf-8"))

    return read("hardware.yaml"), read("frames.yaml"), read("calibration.yaml")


class Processor:
    def __init__(self, hardware: dict, frames: dict, calibration: dict, writer: SessionWriter | None = None) -> None:
        self.hardware = hardware
        self.frames = frames
        self.writer = writer
        self.lock = threading.Lock()
        self.started_ns = time.monotonic_ns()
        self.attitude = AttitudeEstimator()
        tokens = frames.get("imu_link", {}).get("axis_map_raw_to_base")
        self.axis_map = AxisMap(tokens) if isinstance(tokens, list) else None
        imu_cal = calibration.get("imu", {})
        self.accel_bias_raw = tuple(map(float, imu_cal.get("accel_bias_raw_frame_m_s2", [0, 0, 0])))
        self.accel_scale_raw = tuple(map(float, imu_cal.get("accel_scale_raw_frame", [1, 1, 1])))
        raw_bias = imu_cal.get("gyro_bias_raw_frame_rad_s")
        if raw_bias is not None and self.axis_map:
            self.attitude.bias_rad_s = self.axis_map.apply(tuple(map(float, raw_bias)))
        else:
            self.attitude.bias_rad_s = tuple(map(float, imu_cal.get("gyro_bias_rad_s", [0, 0, 0])))
        mapping = hardware["mapping"]
        self.matcher = CorrelativeMatcher(
            cell_m=mapping["scan_match_cell_m"],
            max_translation_m=mapping["scan_match_max_translation_m"],
            translation_step_m=mapping["scan_match_translation_step_m"],
            yaw_window_deg=mapping["scan_match_yaw_window_deg"],
            yaw_step_deg=mapping["scan_match_yaw_step_deg"],
            min_score=mapping["scan_match_min_score"],
            min_motion_gain=mapping.get("scan_match_min_motion_gain", 0.04),
            downsample=mapping["scan_match_downsample"],
        )
        self.grid = OccupancyGrid(mapping["resolution_m"])
        self.pose = Pose2()
        self.previous_scan: LidarScan | None = None
        self.previous_yaw: float | None = None
        self.esp_clock = EspClock()
        self.last_esp_us: int | None = None
        self.imu_times_ns: deque[int] = deque(maxlen=100)
        self.scan_times_ns: deque[int] = deque(maxlen=30)
        self.mapping_times_ns: deque[int] = deque(maxlen=30)
        self.scans_seen = 0
        self.process_every_n_scans = max(1, int(mapping.get("process_every_n_scans", 1)))
        visual = hardware.get("visualization", {})
        self.web_map_cells = max(1, int(visual.get("web_map_cells", 3000)))
        self.local_map_cells = max(1, int(visual.get("local_map_cells", 600)))
        self.local_scan_points = max(1, int(visual.get("local_scan_points", 180)))
        self._health_cache_time_ns = 0
        self._health_cache: dict | None = None
        self.trajectory: deque[list[float]] = deque(maxlen=1000)
        self.state: dict[str, Any] = {
            "system": {"uptime_s": 0, "cpu_load": None, "ram_used_percent": None,
                       "temperature_c": None, "ip": None},
            "sensors": {"lidar": "DISCONNECTED", "imu": "DISCONNECTED"},
            "lidar": {"points": 0, "scan_hz": 0, "health": "UNVERIFIED", "scan": []},
            "imu": {"accel": None, "accel_corrected": None, "gyro": None, "mag": None,
                    "accel_norm": None, "accel_corrected_norm": None,
                    "roll_deg": None, "pitch_deg": None, "yaw_relative_deg": None,
                    "quaternion_wxyz": [1, 0, 0, 0], "update_hz": 0,
                    "esp_time_us": None, "host_receive_ns": None},
            "slam": {"x_m": 0, "y_m": 0, "yaw_deg": 0, "score": 0,
                     "mapping_status": "PAUSED: UNVERIFIED SENSOR FRAMES",
                     "trajectory": [], "map_cells": [], "resolution_m": mapping["resolution_m"],
                     "update_hz": 0},
            "warnings": ["WARNING: IMU extrinsic translation has not been measured",
                         "WARNING: accelerometer has only Z two-pose correction; full 6-position calibration pending"],
            "mode": "ORIENTATION PREVIEW / NOT FULL 3D SLAM",
        }

    def event(self, message: str) -> None:
        if self.writer:
            self.writer.event(time.monotonic_ns(), message)

    def process_imu(self, sample: ImuSample) -> None:
        if self.writer:
            self.writer.write_imu(sample)
        with self.lock:
            try:
                self.last_esp_us = self.esp_clock.unwrap_us(sample.esp_time_us)
            except ValueError:
                self.event("ESP timestamp moved backwards; clock epoch reset")
                self.esp_clock = EspClock()
                self.last_esp_us = self.esp_clock.unwrap_us(sample.esp_time_us)
            self.imu_times_ns.append(sample.host_receive_ns)
            imu = self.state["imu"]
            accel_corrected = tuple(
                (value - bias) * scale for value, bias, scale in
                zip(sample.accel_raw_frame_m_s2, self.accel_bias_raw, self.accel_scale_raw)
            )
            imu.update({
                "accel": sample.accel_raw_frame_m_s2,
                "accel_corrected": accel_corrected,
                "gyro": sample.gyro_raw_frame_rad_s,
                "mag": [value if math.isfinite(value) else None for value in sample.mag_raw_frame_uT],
                "accel_norm": sample.accel_norm_m_s2,
                "accel_corrected_norm": math.sqrt(sum(value * value for value in accel_corrected)),
                "esp_time_us": self.last_esp_us,
                "host_receive_ns": sample.host_receive_ns,
                "update_hz": self._rate(self.imu_times_ns),
            })
            self.state["sensors"]["imu"] = "STREAMING"
            if self.axis_map and self.hardware["imu"].get("axis_frame_verified"):
                accel = self.axis_map.apply(accel_corrected)
                gyro = self.axis_map.apply(sample.gyro_raw_frame_rad_s)
                q = self.attitude.update(sample.host_receive_ns, accel, gyro)
                roll, pitch, yaw = q.euler_rad()
                imu.update({
                    "roll_deg": math.degrees(roll),
                    "pitch_deg": math.degrees(pitch),
                    "yaw_relative_deg": math.degrees(yaw),
                    "quaternion_wxyz": [q.w, q.x, q.y, q.z],
                })
                if self.writer:
                    self.writer.write_fused(sample.host_receive_ns, q, (roll, pitch, yaw))

    def process_scan(self, scan: LidarScan) -> None:
        if self.writer:
            self.writer.write_scan(scan)
        with self.lock:
            self.scan_times_ns.append(scan.timestamp_end_ns)
            self.state["sensors"]["lidar"] = "STREAMING"
            self.state["lidar"].update({
                "points": len(scan.points), "scan_hz": self._rate(self.scan_times_ns),
                "health": "OK", "scan": [
                    [point.angle_rad, point.range_m] for point in scan.points[::max(1, len(scan.points)//360)]
                ],
            })
            self.scans_seen += 1
            if (self.scans_seen - 1) % self.process_every_n_scans:
                return
            self.mapping_times_ns.append(scan.timestamp_end_ns)
            imu = self.state["imu"]
            roll = math.radians(imu["roll_deg"]) if imu["roll_deg"] is not None else None
            pitch = math.radians(imu["pitch_deg"]) if imu["pitch_deg"] is not None else None
            reason = mapping_pause_reason(
                roll, pitch,
                max_roll_deg=self.hardware["mapping"]["max_roll_deg"],
                max_pitch_deg=self.hardware["mapping"]["max_pitch_deg"],
                imu_axes_verified=bool(self.hardware["imu"].get("axis_frame_verified")),
                lidar_angle_verified=bool(self.hardware["lidar"].get("angle_frame_verified")),
            )
            score = 0.0
            if reason is None:
                yaw = math.radians(imu["yaw_relative_deg"])
                yaw_guess = wrap_angle(yaw - self.previous_yaw) if self.previous_yaw is not None else 0.0
                if self.previous_scan is not None:
                    result = self.matcher.match(self.previous_scan, scan, yaw_guess)
                    score = result.score
                    if result.accepted:
                        self.pose = self.pose.moved(
                            result.delta.x_m, result.delta.y_m, result.delta.yaw_rad,
                        )
                    else:
                        reason = "LOW SCAN MATCH CONFIDENCE"
                if reason is None:
                    self.grid.integrate(scan, self.pose)
                    self.trajectory.append([self.pose.x_m, self.pose.y_m])
                self.previous_scan = scan
                self.previous_yaw = yaw
            else:
                self.previous_scan = None
                self.previous_yaw = None
            status = "ACTIVE" if reason is None else f"PAUSED: {reason}"
            self.state["slam"].update({
                "x_m": self.pose.x_m, "y_m": self.pose.y_m,
                "yaw_deg": math.degrees(self.pose.yaw_rad), "score": score,
                "mapping_status": status,
                "trajectory": list(self.trajectory),
                "map_cells": self.grid.compact(max_cells=self.web_map_cells),
                "update_hz": self._rate(self.mapping_times_ns),
            })
            if self.writer:
                self.writer.write_pose(scan.timestamp_end_ns, self.pose, score, status)

    @staticmethod
    def _rate(times: deque[int]) -> float:
        return (len(times) - 1) * 1e9 / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0

    def snapshot(self, *, local: bool = False) -> dict:
        with self.lock:
            # Large scan/map lists are replaced, never mutated in-place. A
            # shallow snapshot avoids serializing the same map twice per HTTP
            # request and keeps the processing lock short.
            data = {key: value.copy() if isinstance(value, dict) else value
                    for key, value in self.state.items()}
        if local:
            scan = data["lidar"]["scan"]
            cells = data["slam"]["map_cells"]
            data["lidar"]["scan"] = scan[::max(1, math.ceil(len(scan) / self.local_scan_points))]
            data["slam"]["map_cells"] = cells[::max(1, math.ceil(len(cells) / self.local_map_cells))]
        data["system"] = self._system_health()
        return data

    def _system_health(self) -> dict:
        now_ns = time.monotonic_ns()
        if self._health_cache is not None and now_ns - self._health_cache_time_ns < 1_000_000_000:
            return self._health_cache.copy()
        total_kib = available_kib = None
        try:
            for line in Path("/proc/meminfo").read_text().splitlines():
                if line.startswith("MemTotal:"):
                    total_kib = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    available_kib = int(line.split()[1])
        except OSError:
            pass
        try:
            milli_c = int(Path("/sys/class/thermal/thermal_zone0/temp").read_text())
            temp = milli_c / 1000
        except (OSError, ValueError):
            temp = None
        try:
            address = subprocess.check_output(["hostname", "-I"], text=True, timeout=1).split()[0]
        except (OSError, subprocess.SubprocessError, IndexError):
            address = None
        result = {
            "uptime_s": (time.monotonic_ns() - self.started_ns) * 1e-9,
            "cpu_load": os.getloadavg()[0] if hasattr(os, "getloadavg") else None,
            "ram_used_percent": 100 * (1 - available_kib / total_kib) if total_kib and available_kib else None,
            "temperature_c": temp,
            "ip": address,
        }
        self._health_cache = result
        self._health_cache_time_ns = now_ns
        return result.copy()


class LiveRunner:
    def __init__(self, root: Path = ROOT, *, record: bool = True) -> None:
        hardware, frames, calibration = load_configuration(root)
        self.root = root
        self.hardware = hardware
        self.writer = SessionWriter(root, hardware, frames, calibration) if record else None
        self.processor = Processor(hardware, frames, calibration, self.writer)
        self.sensor_queue: queue.Queue[tuple[str, Any]] = queue.Queue(
            maxsize=max(hardware["queues"]["imu_samples"], hardware["queues"]["lidar_scans"])
        )
        self.stop_event = threading.Event()
        self.threads: list[threading.Thread] = []
        self.dropped = {"imu": 0, "lidar": 0}
        self.lidar_process: subprocess.Popen | None = None
        self.lidar_enabled = threading.Event()
        self.lidar_enabled.set()
        self.lidar_off_held = threading.Event()

    def pause_lidar(self) -> str:
        """Stop acquisition and wait for the SDK-backed DTR-off holder."""
        if not self.lidar_enabled.is_set():
            return "OFF_HELD" if self.lidar_off_held.wait(8.0) else "STOPPING"
        self.lidar_enabled.clear()
        self.lidar_off_held.clear()
        with self.processor.lock:
            self.processor.state["sensors"]["lidar"] = "STOPPING"
            self.processor.state["slam"]["mapping_status"] = "PAUSED: LIDAR OFF"
            self.processor.previous_scan = None
            self.processor.previous_yaw = None
        process = self.lidar_process
        if process and process.poll() is None:
            process.terminate()
        self.processor.event("LIDAR_PAUSE_REQUESTED")
        return "OFF_HELD" if self.lidar_off_held.wait(8.0) else "STOPPING"

    def resume_lidar(self) -> str:
        if self.lidar_enabled.is_set():
            return "ALREADY_ON"
        self.lidar_off_held.clear()
        with self.processor.lock:
            self.processor.state["sensors"]["lidar"] = "STARTING"
        self.lidar_enabled.set()
        self.processor.event("LIDAR_RESUME_REQUESTED")
        return "STARTING"

    def lidar_motor_status(self) -> str:
        if self.lidar_enabled.is_set():
            return self.processor.snapshot()["sensors"]["lidar"]
        return "OFF_HELD" if self.lidar_off_held.is_set() else "STOPPING"

    def _enqueue(self, kind: str, item: Any) -> None:
        try:
            self.sensor_queue.put_nowait((kind, item))
        except queue.Full:
            self.dropped[kind] += 1
            self.processor.event(f"QUEUE_OVERFLOW sensor={kind} dropped={self.dropped[kind]}")

    def _read_imu(self) -> None:
        port = self.hardware["imu"]["port"]
        while not self.stop_event.is_set():
            try:
                with open_esp32_serial(port, self.hardware["imu"]["baudrate"], timeout=0.5) as stream:
                    self.processor.event(f"IMU_PORT_OPEN {port}")
                    while not self.stop_event.is_set():
                        raw = stream.readline()
                        received_ns = time.monotonic_ns()
                        if not raw:
                            continue
                        line = raw.decode("ascii", errors="replace").strip()
                        if line.startswith("IMU,"):
                            try:
                                self._enqueue("imu", parse_imu_line(line, received_ns))
                            except (ValueError, OverflowError) as error:
                                self.processor.event(f"IMU_PACKET_CORRUPT {error}")
                        elif line:
                            if line.startswith("META,") and self.writer:
                                fields = line.split(",")
                                info = {"protocol": fields[1], "port": port}
                                info.update(part.split("=", 1) for part in fields[2:] if "=" in part)
                                self.writer.update_sensor_metadata("esp32", info)
                            self.processor.event(f"ESP {line}")
            except (serial.SerialException, OSError) as error:
                self.processor.event(f"IMU_DISCONNECTED {error}")
                with self.processor.lock:
                    self.processor.state["sensors"]["imu"] = "DISCONNECTED"
            self.stop_event.wait(2.0)

    def _read_lidar(self) -> None:
        control = self.root / "tools" / "lidar_motor"
        bridge = self.root / "drivers" / "lidar" / "scan_bridge"
        args = [str(bridge), self.hardware["lidar"]["port"],
                str(self.hardware["lidar"]["baudrate"]),
                self.hardware["lidar"]["expected_serial"]]
        direct_env = {**os.environ, "SLAM_LIDAR_DIRECT": "1"}
        while not self.stop_event.is_set():
            if not self.lidar_enabled.is_set():
                if not self.lidar_off_held.is_set():
                    result = subprocess.run([str(control), "off"], check=False,
                                            capture_output=True, text=True, env=direct_env)
                    if result.returncode == 0:
                        self.lidar_off_held.set()
                        with self.processor.lock:
                            self.processor.state["sensors"]["lidar"] = "PAUSED"
                    else:
                        self.processor.event(f"LIDAR_OFF_FAILED {result.stderr.strip()}")
                self.lidar_enabled.wait(0.2)
                continue
            self.lidar_off_held.clear()
            subprocess.run([str(control), "release"], check=False,
                           capture_output=True, env=direct_env)
            if not self.lidar_enabled.is_set():
                continue
            pid_file = self.root / ".runtime" / "scan_bridge.pid"
            try:
                with subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                      text=True, bufsize=1) as process:
                    self.lidar_process = process
                    if not self.lidar_enabled.is_set():
                        process.terminate()
                    pid_file.parent.mkdir(exist_ok=True)
                    pid_file.write_text(str(process.pid), encoding="ascii")
                    assert process.stdout is not None
                    for line in process.stdout:
                        if self.stop_event.is_set() or not self.lidar_enabled.is_set():
                            break
                        try:
                            packet = json.loads(line)
                            if "meta" in packet:
                                if self.writer:
                                    self.writer.update_sensor_metadata("lidar", packet["meta"])
                                self.processor.event(f"LIDAR_META {packet['meta']}")
                                continue
                            scan = normalize_scan(
                                packet,
                                min_range_m=self.hardware["lidar"]["min_range_m"],
                                max_range_m=self.hardware["lidar"]["max_range_m"],
                                min_quality=self.hardware["lidar"]["min_quality"],
                            )
                            if self.hardware["lidar"].get("angle_frame_verified"):
                                sign = int(self.hardware["lidar"].get("angle_sign", 1))
                                offset = float(self.hardware["lidar"].get("angle_offset_rad", 0.0))
                                scan = LidarScan(
                                    scan.timestamp_start_ns, scan.timestamp_end_ns,
                                    scan.sequence,
                                    tuple(LidarPoint((sign * p.angle_rad + offset) % (2 * math.pi),
                                                     p.range_m, p.quality) for p in scan.points),
                                )
                            self._enqueue("lidar", scan)
                        except (ValueError, KeyError, TypeError) as error:
                            self.processor.event(f"LIDAR_PACKET_CORRUPT {error}")
                    if process.poll() is None:
                        process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            except (OSError, subprocess.SubprocessError) as error:
                self.processor.event(f"LIDAR_DISCONNECTED {error}")
            finally:
                if pid_file.exists():
                    try:
                        if self.lidar_process and pid_file.read_text(encoding="ascii").strip() == str(self.lidar_process.pid):
                            pid_file.unlink()
                    except OSError:
                        pass
                self.lidar_process = None
                result = subprocess.run([str(control), "off"], check=False,
                                        capture_output=True, text=True, env=direct_env)
                if result.returncode == 0:
                    self.lidar_off_held.set()
                else:
                    self.processor.event(f"LIDAR_OFF_FAILED {result.stderr.strip()}")
                if not self.lidar_enabled.is_set():
                    with self.processor.lock:
                        self.processor.state["sensors"]["lidar"] = (
                            "PAUSED" if result.returncode == 0 else "OFF_FAILED"
                        )
                        self.processor.state["lidar"].update({"points": 0, "scan_hz": 0, "scan": []})
            if self.lidar_enabled.is_set() and not self.stop_event.is_set():
                self.stop_event.wait(2.0)

    def _process(self) -> None:
        while not self.stop_event.is_set() or not self.sensor_queue.empty():
            try:
                kind, item = self.sensor_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if kind == "imu":
                self.processor.process_imu(item)
            elif kind == "lidar" and self.lidar_enabled.is_set():
                self.processor.process_scan(item)
            self.sensor_queue.task_done()

    def start(self) -> None:
        for target, name in ((self._read_imu, "imu-reader"),
                             (self._read_lidar, "lidar-reader"),
                             (self._process, "processor")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self.threads.append(thread)

    def close(self) -> None:
        self.stop_event.set()
        if self.lidar_process and self.lidar_process.poll() is None:
            self.lidar_process.terminate()
        for thread in self.threads:
            thread.join(timeout=5)
        if self.writer:
            state = self.processor.snapshot()
            self.writer.update_observed_rates({
                "imu": state["imu"]["update_hz"],
                "lidar": state["lidar"]["scan_hz"],
                "mapping": state["slam"]["update_hz"],
            })
            self.writer.close()
