#!/usr/bin/env python3
"""Temporary actuator-free camera/RPLidar bridge for Pi 4 mapping tests.

The bridge only opens a V4L2 camera and a Slamtec-compatible serial LiDAR.  It
does not import or call any robot motion, GPIO, PWM, or servo code.
"""

from __future__ import annotations

import argparse
import atexit
import subprocess
import threading
import time
from dataclasses import dataclass, field

import serial
from flask import Flask, Response, jsonify
from rplidar import RPLidar


@dataclass
class SharedState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    jpeg: bytes | None = None
    camera_timestamp: float | None = None
    lidar_points: list[list[float]] = field(default_factory=list)
    lidar_timestamp: float | None = None
    scan_sequence: int = 0
    imu: dict | None = None
    imu_timestamp: float | None = None
    camera_error: str | None = None
    lidar_error: str | None = None
    imu_error: str | None = None


def camera_worker(state: SharedState, device: str, width: int, height: int, fps: int) -> None:
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "error",
        "-f", "v4l2", "-input_format", "mjpeg",
        "-video_size", f"{width}x{height}", "-framerate", str(fps),
        "-i", device, "-an", "-c:v", "copy", "-f", "mjpeg", "pipe:1",
    ]
    while True:
        process = None
        try:
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
            buffer = bytearray()
            assert process.stdout is not None
            while True:
                chunk = process.stdout.read(65536)
                if not chunk:
                    message = process.stderr.read(512).decode("utf-8", "replace") if process.stderr else ""
                    raise RuntimeError(message.strip() or "ffmpeg camera stream ended")
                buffer.extend(chunk)
                while True:
                    start = buffer.find(b"\xff\xd8")
                    if start < 0:
                        buffer.clear()
                        break
                    end = buffer.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        if start:
                            del buffer[:start]
                        break
                    frame = bytes(buffer[start:end + 2])
                    del buffer[:end + 2]
                    with state.lock:
                        state.jpeg = frame
                        state.camera_timestamp = time.monotonic()
                        state.camera_error = None
        except Exception as exc:  # hardware loop must remain observable
            with state.lock:
                state.camera_error = f"{type(exc).__name__}: {exc}"
            time.sleep(1.0)
        finally:
            if process is not None and process.poll() is None:
                process.terminate()


def lidar_worker(state: SharedState, port: str) -> None:
    while True:
        lidar = None
        try:
            lidar = RPLidar(port, timeout=3)
            for scan in lidar.iter_scans(max_buf_meas=800):
                points = [
                    [float(angle), float(distance), int(quality)]
                    for quality, angle, distance in scan
                    if distance > 0.0
                ]
                with state.lock:
                    state.lidar_points = points
                    state.lidar_timestamp = time.monotonic()
                    state.scan_sequence += 1
                    state.lidar_error = None
        except Exception as exc:  # hardware loop must remain observable
            with state.lock:
                state.lidar_error = f"{type(exc).__name__}: {exc}"
            time.sleep(1.0)
        finally:
            if lidar is not None:
                for method in ("stop", "stop_motor", "disconnect"):
                    try:
                        getattr(lidar, method)()
                    except Exception:
                        pass


def imu_worker(state: SharedState, port: str, baud: int = 115200) -> None:
    """Read the actuator-free @IMU1 telemetry without toggling ESP reset pins."""
    while True:
        connection = None
        try:
            connection = serial.Serial()
            connection.port = port
            connection.baudrate = baud
            connection.timeout = 1.0
            connection.dtr = False
            connection.rts = False
            connection.open()
            while True:
                raw = connection.readline().decode("ascii", "replace").strip()
                if not raw:
                    continue
                if raw.startswith("IMU1,"):
                    raw = "@" + raw
                if not raw.startswith("@IMU1,"):
                    continue
                fields = raw.split(",")
                if len(fields) != 13:
                    continue
                packet = {
                    "sequence": int(fields[1]),
                    "esp_micros": int(fields[2]),
                    "online": fields[3] == "1",
                    "accel_g": [float(fields[4]), float(fields[5]), float(fields[6])],
                    "gyro_dps": [float(fields[7]), float(fields[8]), float(fields[9])],
                    "rpy_deg": [float(fields[10]), float(fields[11]), float(fields[12])],
                }
                with state.lock:
                    state.imu = packet
                    state.imu_timestamp = time.monotonic()
                    state.imu_error = None
        except Exception as exc:
            with state.lock:
                state.imu_error = f"{type(exc).__name__}: {exc}"
            time.sleep(1.0)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass


def create_app(state: SharedState) -> Flask:
    app = Flask(__name__)

    @app.get("/status")
    def status():
        now = time.monotonic()
        with state.lock:
            return jsonify({
                "camera": state.jpeg is not None,
                "camera_age_s": None if state.camera_timestamp is None else now - state.camera_timestamp,
                "camera_error": state.camera_error,
                "lidar": bool(state.lidar_points),
                "lidar_age_s": None if state.lidar_timestamp is None else now - state.lidar_timestamp,
                "lidar_points": len(state.lidar_points),
                "lidar_error": state.lidar_error,
                "scan_seq": state.scan_sequence,
                "imu": state.imu is not None,
                "imu_online": bool(state.imu and state.imu.get("online")),
                "imu_age_s": None if state.imu_timestamp is None else now - state.imu_timestamp,
                "imu_error": state.imu_error,
                "actuators": "DISABLED_BY_DESIGN",
            })

    @app.get("/health")
    def health():
        """Compatibility health probe for the unified APEX mapping HUD."""
        now = time.monotonic()
        with state.lock:
            camera_ok = state.jpeg is not None and state.camera_timestamp is not None and now - state.camera_timestamp < 2.0
            lidar_ok = bool(state.lidar_points) and state.lidar_timestamp is not None and now - state.lidar_timestamp < 1.0
            imu_ok = state.imu is not None and state.imu_timestamp is not None and now - state.imu_timestamp < 0.5
        return jsonify({"ok": camera_ok and lidar_ok and imu_ok, "camera": camera_ok, "lidar": lidar_ok, "imu": imu_ok})

    @app.get("/camera/raw-snapshot")
    def camera_snapshot():
        now = time.monotonic()
        with state.lock:
            if state.jpeg is None or state.camera_timestamp is None:
                return jsonify({"error": state.camera_error or "camera not ready"}), 503
            frame = state.jpeg
            timestamp = state.camera_timestamp
        return Response(frame, mimetype="image/jpeg", headers={
            "X-Apex-Frame-Ts": f"{timestamp:.9f}",
            "X-Apex-Frame-Age": f"{now - timestamp:.6f}",
            "Cache-Control": "no-store",
        })

    @app.get("/api/lidar/snapshot")
    def lidar_snapshot():
        now = time.monotonic()
        with state.lock:
            timestamp = state.lidar_timestamp
            points = list(state.lidar_points)
            sequence = state.scan_sequence
            error = state.lidar_error
        age = None if timestamp is None else now - timestamp
        return jsonify({
            "fresh": bool(points) and age is not None and age < 1.0,
            "scan_seq": sequence,
            "timestamp_monotonic_s": timestamp,
            "scan_age_s": age,
            "points": points,
            "error": error,
        })

    @app.get("/api/imu/snapshot")
    def imu_snapshot():
        now = time.monotonic()
        with state.lock:
            timestamp = state.imu_timestamp
            packet = None if state.imu is None else dict(state.imu)
            error = state.imu_error
        age = None if timestamp is None else now - timestamp
        return jsonify({
            "fresh": packet is not None and age is not None and age < 0.25,
            "timestamp_monotonic_s": timestamp,
            "sample_age_s": age,
            "imu": packet,
            "error": error,
        })

    return app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--lidar", default="/dev/ttyUSB0")
    parser.add_argument("--imu", default="/dev/ttyUSB1")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args()

    state = SharedState()
    threading.Thread(target=camera_worker, args=(state, args.camera, args.width, args.height, args.fps), daemon=True).start()
    threading.Thread(target=lidar_worker, args=(state, args.lidar), daemon=True).start()
    threading.Thread(target=imu_worker, args=(state, args.imu), daemon=True).start()
    create_app(state).run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
