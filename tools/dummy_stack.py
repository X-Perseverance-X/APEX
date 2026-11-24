"""Cross-platform supervisor for the hardware-free APEX stack."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.request import urlopen
from urllib.parse import urlencode

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


ROOT = Path(__file__).resolve().parents[1]
LAB = ROOT / "apex_kinematics_lab"
RUNTIME = ROOT / ".runtime" / "dummy"

SERVICES = (
    ("dummy_robot", 8081, [sys.executable, "-m", "uvicorn", "sim.dummy_robot:app", "--app-dir", str(ROOT), "--host", "127.0.0.1", "--port", "8081"]),
    ("dummy_camera", 8888, [sys.executable, "-m", "uvicorn", "sim.dummy_camera:app", "--app-dir", str(ROOT), "--host", "127.0.0.1", "--port", "8888"]),
    ("command_center", 8080, [sys.executable, "-m", "sim.run_command_center"]),
    ("kinematics_lab", 8090, [sys.executable, "-m", "uvicorn", "app:app", "--app-dir", str(LAB), "--host", "127.0.0.1", "--port", "8090"]),
)


def _port_open(port: int) -> bool:
    with socket.socket() as sock:
        sock.settimeout(0.15)
        return sock.connect_ex(("127.0.0.1", port)) == 0


def _healthy(port: int) -> bool:
    try:
        with urlopen(f"http://127.0.0.1:{port}/", timeout=1.0) as response:
            return 200 <= response.status < 500
    except Exception:
        return False


def start() -> int:
    RUNTIME.mkdir(parents=True, exist_ok=True)
    started: list[str] = []
    for name, port, command in SERVICES:
        pid_file = RUNTIME / f"{name}.pid"
        if _port_open(port):
            print(f"[INFO] {name} port {port} zaten kullanımda; yeniden başlatılmadı.")
            continue
        stdout = (RUNTIME / f"{name}.out.log").open("ab", buffering=0)
        stderr = (RUNTIME / f"{name}.err.log").open("ab", buffering=0)
        creationflags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            close_fds=True,
            creationflags=creationflags,
            start_new_session=os.name != "nt",
        )
        pid_file.write_text(str(process.pid), encoding="ascii")
        started.append(name)
        print(f"[START] {name} PID {process.pid}, port {port}")

    deadline = time.monotonic() + 15.0
    pending = {name: port for name, port, _ in SERVICES}
    while pending and time.monotonic() < deadline:
        for name, port in list(pending.items()):
            if _healthy(port):
                print(f"[OK] {name}: http://127.0.0.1:{port}")
                pending.pop(name)
        if pending:
            time.sleep(0.2)
    if pending:
        print(f"[ERROR] Hazır olmayan servisler: {pending}. Log: {RUNTIME}", file=sys.stderr)
        return 1

    print("\nAPEX dummy stack hazır:")
    print("  Command Center : http://127.0.0.1:8080")
    print("  Kinematics Lab : http://127.0.0.1:8090")
    print("  Dummy Robot API: http://127.0.0.1:8081/docs")
    print("  Dummy Camera   : http://127.0.0.1:8888/stream")
    print(f"  Logs           : {RUNTIME}")
    return 0


def _windows_command_line(pid: int) -> str:
    if os.name != "nt":
        return ""
    command = [
        "powershell.exe",
        "-NoProfile",
        "-Command",
        f"(Get-CimInstance Win32_Process -Filter 'ProcessId={pid}').CommandLine",
    ]
    result = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
    return result.stdout.strip()


def _windows_port_owner(port: int) -> int | None:
    command = [
        "powershell.exe",
        "-NoProfile",
        "-Command",
        f"Get-NetTCPConnection -State Listen -LocalPort {port} -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty OwningProcess",
    ]
    result = subprocess.run(command, capture_output=True, text=True, timeout=5, check=False)
    value = result.stdout.strip()
    return int(value) if value.isdigit() else None


def stop() -> int:
    if not RUNTIME.exists():
        print("Dummy stack çalışmıyor.")
        return 0
    expected = {
        "dummy_robot": "sim.dummy_robot",
        "dummy_camera": "sim.dummy_camera",
        "command_center": "sim.run_command_center",
        "kinematics_lab": "apex_kinematics_lab",
    }
    for name, port, _command in reversed(SERVICES):
        pid_file = RUNTIME / f"{name}.pid"
        if not pid_file.exists():
            continue
        pid = int(pid_file.read_text(encoding="ascii").strip())
        if os.name == "nt":
            command_line = _windows_command_line(pid)
            owns_port = _windows_port_owner(port) == pid
            if owns_port or (command_line and expected[name] in command_line):
                subprocess.run(["taskkill", "/PID", str(pid), "/T", "/F"], capture_output=True, check=False)
                print(f"[STOP] {name} PID {pid}")
            elif command_line:
                print(f"[WARN] PID {pid} beklenen {name} prosesi değil; dokunulmadı.")
        else:
            try:
                os.killpg(pid, signal.SIGTERM)
                print(f"[STOP] {name} PID {pid}")
            except ProcessLookupError:
                pass
        pid_file.unlink(missing_ok=True)
    return 0


def status() -> int:
    data = {name: {"port": port, "online": _healthy(port)} for name, port, _ in SERVICES}
    print(json.dumps(data, indent=2))
    return 0 if all(item["online"] for item in data.values()) else 1


def _get_json(port: int, path: str, query: dict | None = None) -> dict:
    suffix = f"?{urlencode(query)}" if query else ""
    with urlopen(f"http://127.0.0.1:{port}{path}{suffix}", timeout=3.0) as response:
        return json.loads(response.read().decode("utf-8"))


def smoke() -> int:
    result = start()
    if result:
        return result
    try:
        _get_json(8080, "/mode", {"m": "bezier_joy"})
        _get_json(8080, "/joy", {"x": 0, "y": 0.8, "r": 0})
        time.sleep(0.12)
        moving = _get_json(8080, "/telemetry")
        assert moving["mode"] == "YURUYUS" and abs(float(moving["vy"]) - 0.8) < 0.01

        _get_json(8081, "/sim/fault", {"kind": "link_drop", "state": 1})
        time.sleep(0.45)
        stopped = _get_json(8080, "/telemetry")
        assert stopped["mode"] == "IDLE" and stopped["failsafe"] is True and abs(float(stopped["vy"])) < 0.001

        _get_json(8080, "/legOff", {"leg": 0, "j": 1})
        legs = _get_json(8080, "/legStatus")
        assert legs["legs"][0]["enabled"][1] is False

        camera = _get_json(8888, "/status")
        live = _get_json(8090, "/api/live")
        assert camera["fresh"] is True and live["online"] is True and live["telemetry"]["dummy"] is True
        print("[PASS] motion -> link loss -> failsafe")
        print("[PASS] persistent legOff")
        print("[PASS] camera freshness")
        print("[PASS] Command Center -> dummy robot -> Kinematics Lab")
        return 0
    except (AssertionError, OSError, KeyError, ValueError) as exc:
        print(f"[FAIL] dummy smoke test: {exc!r}", file=sys.stderr)
        return 1
    finally:
        stop()


if __name__ == "__main__":
    action = sys.argv[1] if len(sys.argv) > 1 else "status"
    raise SystemExit({"start": start, "stop": stop, "status": status, "test": smoke}.get(action, status)())
