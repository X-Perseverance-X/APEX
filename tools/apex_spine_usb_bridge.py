#!/usr/bin/env python3
"""META-1 ESP32-S3 wired USB telemetry and peripheral bridge.

The high-rate IMU packet and slower system packet are cached as local HTTP
JSON.  Serial writes are restricted to a small non-motion allow-list (laser,
thermal fan and DFPlayer); this process cannot arm PWM or move a servo.
"""

from __future__ import annotations

import argparse
import json
import os
import select
import termios
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse


@dataclass
class BridgeState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    started_monotonic: float = field(default_factory=time.monotonic)
    connected: bool = False
    port: str = ""
    last_error: str | None = None
    last_parse_error: str | None = None
    reconnects: int = 0
    lines_seen: int = 0
    unknown_lines: int = 0
    parse_errors: int = 0
    imu: dict[str, Any] | None = None
    imu_received: float | None = None
    imu_frames: int = 0
    imu_gaps: int = 0
    spine: dict[str, Any] | None = None
    spine_received: float | None = None
    spine_frames: int = 0
    spine_gaps: int = 0
    foot: dict[str, Any] | None = None
    foot_received: float | None = None
    foot_frames: int = 0
    foot_gaps: int = 0
    serial_fd: int | None = None
    write_lock: threading.Lock = field(default_factory=threading.Lock)
    ack_condition: threading.Condition = field(default_factory=threading.Condition)
    acknowledgements: dict[str, dict[str, Any]] = field(default_factory=dict)
    command_sequence: int = 0
    commands_sent: int = 0
    command_errors: int = 0


def _configure_serial(fd: int, baud: int) -> None:
    speeds = {115200: termios.B115200, 57600: termios.B57600, 9600: termios.B9600}
    if baud not in speeds:
        raise ValueError(f"unsupported baud: {baud}")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = speeds[baud]
    attrs[5] = speeds[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _sequence_gap(previous: dict[str, Any] | None, current: int) -> int:
    if previous is None:
        return 0
    expected = (int(previous["sequence"]) + 1) & 0xFFFFFFFF
    return (current - expected) & 0xFFFFFFFF if current != expected else 0


def _parse_imu(raw: str) -> dict[str, Any]:
    f = raw.split(",")
    if len(f) != 13:
        raise ValueError(f"IMU1 field count {len(f)}")
    return {
        "protocol": "IMU1",
        "sequence": int(f[1]),
        "esp_micros": int(f[2]),
        "online": f[3] == "1",
        "accel_g": [float(f[4]), float(f[5]), float(f[6])],
        "gyro_dps": [float(f[7]), float(f[8]), float(f[9])],
        "rpy_deg": [float(f[10]), float(f[11]), float(f[12])],
    }


def _parse_spine(raw: str) -> dict[str, Any]:
    f = raw.split(",")
    if len(f) not in (20, 24, 26, 31, 35, 38, 39, 41):
        raise ValueError(f"TEL1 field count {len(f)}")
    packet = {
        "protocol": "TEL1",
        "sequence": int(f[1]),
        "esp_millis": int(f[2]),
        "pca_right_online": f[3] == "1",
        "pca_left_online": f[4] == "1",
        "imu_online": f[5] == "1",
        "tof_online": f[6] == "1",
        "tof_mm": int(f[7]),
        "dfplayer_online": f[8] == "1",
        "gps_chars": int(f[9]),
        "gps_fix": f[10] == "1",
        "gps_satellites": int(f[11]),
        "latitude": float(f[12]),
        "longitude": float(f[13]),
        "battery_right_v": float(f[14]),
        "battery_left_v": float(f[15]),
        "spine_state": f[16],
        "outputs_armed": f[17] == "1",
        "mode": int(f[18]),
        "config_valid": f[19] == "1",
    }
    if len(f) >= 24:
        packet.update({
            "laser_on": f[20] == "1",
            "laser_power_pct": int(f[21]),
            "tof_sequence": int(f[22]),
            "tof_age_ms": int(f[23]),
        })
    if len(f) >= 26:
        contact_mask = int(f[24]) & 0x3F
        packet.update({
            "foot_contact_mask": contact_mask,
            "foot_contacts": [bool(contact_mask & (1 << i)) for i in range(6)],
            "foot_contact_sequence": int(f[25]),
        })
    if len(f) >= 31:
        packet.update({
            "gps_passed_checksum": int(f[26]),
            "gps_failed_checksum": int(f[27]),
            "gps_sentences_with_fix": int(f[28]),
            "gps_location_age_ms": int(f[29]),
            "gps_hdop_x100": int(f[30]),
        })
    if len(f) >= 35:
        packet.update({
            "internal_temp_c": float(f[31]),
            "fan_power_pct": int(f[32]),
            "fan_auto": f[33] == "1",
            "ntc_valid": f[34] == "1",
        })
    if len(f) >= 38:
        packet.update({
            "camera_pan_deg": float(f[35]),
            "camera_tilt_deg": float(f[36]),
            "camera_gimbal_enabled": f[37] == "1",
        })
    if len(f) >= 39:
        packet["camera_gimbal_mode"] = int(f[38])
    if len(f) >= 41:
        packet["home_set"] = f[39] == "1"
        packet["firmware_version"] = f[40]
    return packet


def _parse_ack(raw: str) -> tuple[str, dict[str, Any]]:
    f = raw.split(",", 3)
    if len(f) != 4:
        raise ValueError(f"ACK1 field count {len(f)}")
    return f[1], {"ok": f[2] == "OK", "status": f[2], "message": f[3]}


def _parse_foot(raw: str) -> dict[str, Any]:
    f = raw.split(",")
    if len(f) != 5:
        raise ValueError(f"FOOT1 field count {len(f)}")
    contact_mask = int(f[3]) & 0x3F
    return {
        "protocol": "FOOT1",
        "sequence": int(f[1]),
        "esp_millis": int(f[2]),
        "contact_mask": contact_mask,
        "contacts": [bool(contact_mask & (1 << i)) for i in range(6)],
        "change_sequence": int(f[4]),
    }


def send_peripheral_command(
    state: BridgeState, device: str, action: str, value: str = "", timeout: float = 2.5
) -> dict[str, Any]:
    device = device.upper()
    action = action.upper()
    allowed = {
        "LASER": {"ON", "OFF", "SET"},
        "FAN": {"AUTO", "OFF", "SET"},
        "CAM_PAN": {"SET"},
        "CAM_TILT": {"SET"},
        "CAM_GIMBAL": {"CENTER", "ON", "OFF", "SET", "STABILIZE"},
        "AUDIO": {"INIT", "PLAY", "STOP", "NEXT", "PREV", "VOL"},
        "LEG_LAB": {"SET", "POSE", "ON", "OFF"},
        "SPINE": {"STAND"},
    }
    if device not in allowed or action not in allowed[device]:
        return {"ok": False, "message": "command not allowed", "status": "ERR"}
    # Camera pan+tilt is intentionally sent as one atomic value ("pan,tilt")
    # so the two axes start together.  Other peripherals still accept only a
    # single numeric value; do not broaden their command grammar.
    allowed_value_chars = "-+.0123456789,"
    if any(ch not in allowed_value_chars for ch in value):
        return {"ok": False, "message": "invalid value", "status": "ERR"}
    if "," in value:
        atomic_pair = device == "CAM_GIMBAL" and action == "SET" and value.count(",") == 1
        leg_lab_set = device == "LEG_LAB" and action == "SET" and value.count(",") == 2
        leg_lab_pose = device == "LEG_LAB" and action == "POSE" and value.count(",") == 3
        leg_lab_switch = device == "LEG_LAB" and action in {"ON", "OFF"} and value.count(",") == 1
        if not (atomic_pair or leg_lab_set or leg_lab_pose or leg_lab_switch):
            return {"ok": False, "message": "invalid value", "status": "ERR"}

    with state.lock:
        fd = state.serial_fd
        connected = state.connected
        state.command_sequence = (state.command_sequence + 1) & 0xFFFFFFFF
        command_id = str(state.command_sequence)
    if not connected or fd is None:
        return {"ok": False, "message": "serial disconnected", "status": "ERR"}

    line = f"@CMD1,{command_id},{device},{action}"
    if value:
        line += f",{value}"
    line += "\n"
    try:
        with state.write_lock:
            os.write(fd, line.encode("ascii"))
        with state.lock:
            state.commands_sent += 1
        deadline = time.monotonic() + timeout
        with state.ack_condition:
            while command_id not in state.acknowledgements:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("ESP acknowledgement timeout")
                state.ack_condition.wait(remaining)
            result = state.acknowledgements.pop(command_id)
        result.update({"device": device, "action": action, "command_id": command_id})
        if not result["ok"]:
            with state.lock:
                state.command_errors += 1
        return result
    except (OSError, TimeoutError) as exc:
        with state.lock:
            state.command_errors += 1
        return {"ok": False, "status": "ERR", "message": str(exc), "device": device, "action": action}


def serial_worker(state: BridgeState, port: str, baud: int) -> None:
    buffer = bytearray()
    while True:
        fd = -1
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            _configure_serial(fd, baud)
            with state.lock:
                state.connected = True
                state.port = os.path.realpath(port)
                state.last_error = None
                state.reconnects += 1
                state.serial_fd = fd
            while True:
                readable, _, _ = select.select([fd], [], [], 0.5)
                if not readable:
                    continue
                chunk = os.read(fd, 4096)
                if not chunk:
                    raise OSError("serial device returned EOF")
                buffer.extend(chunk)
                if len(buffer) > 65536:
                    del buffer[:-8192]
                while b"\n" in buffer:
                    line, _, remainder = buffer.partition(b"\n")
                    buffer[:] = remainder
                    raw = line.decode("ascii", "replace").strip().lstrip("\x00")
                    if not raw:
                        continue
                    now = time.monotonic()
                    with state.lock:
                        state.lines_seen += 1
                    try:
                        if raw.startswith("@IMU1,"):
                            packet = _parse_imu(raw)
                            with state.lock:
                                state.imu_gaps += _sequence_gap(state.imu, packet["sequence"])
                                state.imu = packet
                                state.imu_received = now
                                state.imu_frames += 1
                        elif raw.startswith("@TEL1,"):
                            packet = _parse_spine(raw)
                            with state.lock:
                                state.spine_gaps += _sequence_gap(state.spine, packet["sequence"])
                                state.spine = packet
                                state.spine_received = now
                                state.spine_frames += 1
                        elif raw.startswith("@FOOT1,"):
                            packet = _parse_foot(raw)
                            with state.lock:
                                state.foot_gaps += _sequence_gap(state.foot, packet["sequence"])
                                state.foot = packet
                                state.foot_received = now
                                state.foot_frames += 1
                        elif raw.startswith("@ACK1,"):
                            command_id, acknowledgement = _parse_ack(raw)
                            with state.ack_condition:
                                state.acknowledgements[command_id] = acknowledgement
                                state.ack_condition.notify_all()
                        else:
                            with state.lock:
                                state.unknown_lines += 1
                    except (ValueError, OverflowError) as exc:
                        with state.lock:
                            state.parse_errors += 1
                            state.last_parse_error = str(exc)
        except Exception as exc:
            with state.lock:
                state.connected = False
                state.serial_fd = None
                state.last_error = f"{type(exc).__name__}: {exc}"
            time.sleep(1.0)
        finally:
            with state.lock:
                if state.serial_fd == fd:
                    state.serial_fd = None
                    state.connected = False
            if fd >= 0:
                os.close(fd)


def snapshot(state: BridgeState) -> dict[str, Any]:
    now = time.monotonic()
    with state.lock:
        uptime = max(now - state.started_monotonic, 0.001)
        imu_age = None if state.imu_received is None else now - state.imu_received
        spine_age = None if state.spine_received is None else now - state.spine_received
        foot_age = None if state.foot_received is None else now - state.foot_received
        return {
            "ok": state.connected and imu_age is not None and imu_age < 0.25,
            "transport": "usb-wired",
            "connected": state.connected,
            "port": state.port,
            "last_error": state.last_error,
            "last_parse_error": state.last_parse_error,
            "uptime_s": uptime,
            "reconnects": state.reconnects,
            "lines_seen": state.lines_seen,
            "unknown_lines": state.unknown_lines,
            "parse_errors": state.parse_errors,
            "imu_fps": state.imu_frames / uptime,
            "imu_age_s": imu_age,
            "imu_gaps": state.imu_gaps,
            "imu": None if state.imu is None else dict(state.imu),
            "spine_fps": state.spine_frames / uptime,
            "spine_age_s": spine_age,
            "spine_gaps": state.spine_gaps,
            "spine": None if state.spine is None else dict(state.spine),
            "foot_fps": state.foot_frames / uptime,
            "foot_age_s": foot_age,
            "foot_gaps": state.foot_gaps,
            "foot": None if state.foot is None else dict(state.foot),
            "motion_writes": False,
            "peripheral_writes": True,
            "commands_sent": state.commands_sent,
            "command_errors": state.command_errors,
        }


def make_handler(state: BridgeState):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - stdlib callback name
            parsed = urlparse(self.path)
            if parsed.path == "/command":
                query = parse_qs(parsed.query)
                device = query.get("device", [""])[0]
                action = query.get("action", [""])[0]
                value = query.get("value", [""])[0]
                payload = send_peripheral_command(state, device, action, value)
                self._send_json(payload, 200 if payload.get("ok") else 409)
                return
            if parsed.path not in ("/", "/health", "/telemetry", "/diagnostics"):
                self.send_error(404)
                return
            payload = snapshot(state)
            if parsed.path == "/health":
                payload = {k: payload[k] for k in (
                    "ok", "transport", "connected", "port", "last_error",
                    "imu_fps", "imu_age_s", "spine_fps", "spine_age_s",
                    "motion_writes", "peripheral_writes",
                )}
            self._send_json(payload, 200 if payload.get("ok") else 503)

        def _send_json(self, payload: dict[str, Any], status: int) -> None:
            body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format: str, *_args: object) -> None:
            return

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5069RR4-if00-port0",
        help="stable ESP FTDI path",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--listen", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8091)
    args = parser.parse_args()

    state = BridgeState()
    threading.Thread(target=serial_worker, args=(state, args.port, args.baud), daemon=True).start()
    ThreadingHTTPServer((args.listen, args.http_port), make_handler(state)).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
