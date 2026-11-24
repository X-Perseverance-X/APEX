#!/usr/bin/env python3
"""Dependency-free CSI camera test server for a freshly provisioned APEX brain.

The server owns a single ``rpicam-vid`` process and republishes its freshest
JPEG frame to any number of browser clients.  It deliberately implements the
same ``/stream``, ``/snapshot`` and ``/health`` endpoints used by the production
APEX camera service, so the temporary test service can be replaced without
changing the HUD.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


INDEX_HTML = """<!doctype html>
<html lang="tr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>APEX CSI0 Camera Test</title><style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#050607;color:#d9fff5;font:14px ui-monospace,Consolas,monospace}
header{height:52px;display:flex;align-items:center;justify-content:space-between;padding:0 16px;border-bottom:2px solid #00e7b0;background:#090c0d}
h1{font-size:16px;letter-spacing:1.2px;margin:0;color:#00ffc8}.badge{color:#ffb000}.stage{height:calc(100vh - 104px);display:grid;place-items:center;overflow:hidden;background:#000}
#feed{max-width:100%;max-height:100%;object-fit:contain;transform-origin:center center}.bar{height:52px;display:flex;gap:8px;align-items:center;padding:8px 12px;border-top:1px solid #173630;background:#090c0d}
button{border:1px solid #00c99a;background:#111918;color:#d9fff5;padding:8px 12px;font:inherit;cursor:pointer}button:hover{background:#123229}.spacer{flex:1}.warn{color:#ffb000}
</style></head><body><header><h1>APEX CSI0 / OV5647 / 1080P TEST</h1><span id="status" class="badge">BAĞLANIYOR</span></header>
<main class="stage"><img id="feed" src="/stream" alt="CSI0 live feed"></main><footer class="bar">
<button onclick="rot=(rot+90)%360;apply()">DÖNDÜR 90°</button><button onclick="flipX=!flipX;apply()">YATAY AYNA</button><button onclick="flipY=!flipY;apply()">DİKEY AYNA</button>
<button onclick="location.href='/snapshot'">KAREYİ AÇ</button><span class="spacer"></span><span class="warn">Netlik: lens halkası elle ayarlanır</span></footer>
<script>let rot=0,flipX=false,flipY=false;const feed=document.getElementById('feed'),st=document.getElementById('status');
function apply(){feed.style.transform=`rotate(${rot}deg) scaleX(${flipX?-1:1}) scaleY(${flipY?-1:1})`}
feed.onload=()=>{st.textContent='CANLI';st.style.color='#00ffc8'};feed.onerror=()=>{st.textContent='AKIŞ KOPTU';st.style.color='#ff4050';setTimeout(()=>feed.src='/stream?'+Date.now(),1000)};
setInterval(async()=>{try{const r=await fetch('/health',{cache:'no-store'}),j=await r.json();st.textContent=j.ok?`CANLI · ${j.width}×${j.height} · ${j.fps.toFixed(1)} FPS`:'KAMERA BEKLENİYOR';st.style.color=j.ok?'#00ffc8':'#ffb000'}catch(e){st.textContent='SERVİS YOK';st.style.color='#ff4050'}},1000);
</script></body></html>""".encode("utf-8")


class CameraState:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.frame: bytes | None = None
        self.frame_time = 0.0
        self.frame_count = 0
        self.started_at = time.monotonic()
        self.error: str | None = None
        self.process: subprocess.Popen[bytes] | None = None
        self.stop_event = threading.Event()
        self.condition = threading.Condition()

    def command(self) -> list[str]:
        command = [
            "rpicam-vid", "--camera", str(self.args.camera), "--codec", "mjpeg",
            "--width", str(self.args.width), "--height", str(self.args.height),
            "--framerate", str(self.args.framerate), "--quality", str(self.args.quality),
            "--timeout", "0", "--nopreview", "--flush", "--output", "-",
        ]
        if self.args.rotation:
            command += ["--rotation", str(self.args.rotation)]
        if self.args.hflip:
            command.append("--hflip")
        if self.args.vflip:
            command.append("--vflip")
        return command

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.error = None
                self._capture_once()
                if not self.stop_event.is_set():
                    self.error = "rpicam-vid exited unexpectedly"
            except Exception as exc:  # keep the diagnostic endpoint alive
                self.error = f"{type(exc).__name__}: {exc}"
            self._terminate_process()
            self.stop_event.wait(1.0)

    def _capture_once(self) -> None:
        self.process = subprocess.Popen(
            self.command(), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=0, start_new_session=True,
        )
        assert self.process.stdout is not None
        buffer = bytearray()
        while not self.stop_event.is_set():
            chunk = self.process.stdout.read(65536)
            if not chunk:
                stderr = b""
                if self.process.stderr is not None:
                    stderr = self.process.stderr.read(4096)
                message = stderr.decode("utf-8", "replace").strip()
                raise RuntimeError(message or f"camera process exited ({self.process.poll()})")
            buffer.extend(chunk)
            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break
                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start:
                        del buffer[:start]
                    if len(buffer) > 16 * 1024 * 1024:
                        buffer.clear()
                    break
                frame = bytes(buffer[start:end + 2])
                del buffer[:end + 2]
                now = time.monotonic()
                with self.condition:
                    self.frame = frame
                    self.frame_time = now
                    self.frame_count += 1
                    self.condition.notify_all()

    def _terminate_process(self) -> None:
        process, self.process = self.process, None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def stop(self) -> None:
        self.stop_event.set()
        self._terminate_process()
        with self.condition:
            self.condition.notify_all()

    def health(self) -> dict[str, object]:
        now = time.monotonic()
        age = None if self.frame is None else now - self.frame_time
        elapsed = max(0.001, now - self.started_at)
        return {
            "ok": self.frame is not None and age is not None and age < 2.0,
            "camera": self.frame is not None,
            "fresh": age is not None and age < 2.0,
            "frame_age_s": age,
            "frames": self.frame_count,
            "fps": self.frame_count / elapsed,
            "width": self.args.width,
            "height": self.args.height,
            "requested_fps": self.args.framerate,
            "error": self.error,
        }


class CameraHandler(BaseHTTPRequestHandler):
    server_version = "APEX-CSI-Test/1.0"

    @property
    def state(self) -> CameraState:
        return self.server.camera_state  # type: ignore[attr-defined]

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._bytes(HTTPStatus.OK, "text/html; charset=utf-8", INDEX_HTML)
        elif path in {"/health", "/status"}:
            payload = json.dumps(self.state.health(), separators=(",", ":")).encode()
            self._bytes(HTTPStatus.OK, "application/json", payload)
        elif path in {"/snapshot", "/raw-snapshot"}:
            frame = self.state.frame
            if frame is None:
                self._bytes(HTTPStatus.SERVICE_UNAVAILABLE, "application/json", b'{"error":"camera not ready"}')
            else:
                self._bytes(HTTPStatus.OK, "image/jpeg", frame, cache=False)
        elif path == "/stream":
            self._stream()
        else:
            self._bytes(HTTPStatus.NOT_FOUND, "application/json", b'{"error":"not found"}')

    def _bytes(self, status: HTTPStatus, content_type: str, payload: bytes, cache: bool = True) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "public, max-age=60" if cache else "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        last_frame = -1
        try:
            while not self.state.stop_event.is_set():
                with self.state.condition:
                    self.state.condition.wait_for(
                        lambda: self.state.frame_count != last_frame or self.state.stop_event.is_set(),
                        timeout=2.0,
                    )
                    frame, current = self.state.frame, self.state.frame_count
                if frame is None or current == last_frame:
                    continue
                last_frame = current
                self.wfile.write(
                    b"--FRAME\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
                )
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def log_message(self, fmt: str, *args: object) -> None:
        if self.path.split("?", 1)[0] not in {"/health", "/status"}:
            super().log_message(fmt, *args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=os.environ.get("APEX_CAMERA_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("APEX_CAMERA_PORT", "8888")))
    parser.add_argument("--camera", type=int, default=int(os.environ.get("APEX_CAMERA_INDEX", "0")))
    parser.add_argument("--width", type=int, default=int(os.environ.get("APEX_CAMERA_WIDTH", "1920")))
    parser.add_argument("--height", type=int, default=int(os.environ.get("APEX_CAMERA_HEIGHT", "1080")))
    parser.add_argument("--framerate", type=int, default=int(os.environ.get("APEX_CAMERA_FPS", "30")))
    parser.add_argument("--quality", type=int, default=int(os.environ.get("APEX_CAMERA_JPEG_QUALITY", "80")))
    parser.add_argument("--rotation", type=int, choices=(0, 180), default=int(os.environ.get("APEX_CAMERA_ROTATION", "0")))
    parser.add_argument("--hflip", action="store_true", default=os.environ.get("APEX_CAMERA_HFLIP") == "1")
    parser.add_argument("--vflip", action="store_true", default=os.environ.get("APEX_CAMERA_VFLIP") == "1")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    state = CameraState(args)
    worker = threading.Thread(target=state.run, name="csi-camera", daemon=True)
    worker.start()
    server = ThreadingHTTPServer((args.host, args.port), CameraHandler)
    server.daemon_threads = True
    server.camera_state = state  # type: ignore[attr-defined]

    def stop_server(_signum: int, _frame: object) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)
    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        state.stop()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
