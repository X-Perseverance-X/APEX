"""FastAPI telemetry server; the acquisition pipeline never waits on clients."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response

from slam_runtime.pipeline import LiveRunner


def create_app(runner: LiveRunner) -> FastAPI:
    app = FastAPI(title="SLAM Lab V0")
    static = Path(__file__).resolve().parent

    @app.get("/")
    async def index():
        return FileResponse(static / "index.html")

    @app.get("/sweep3d.js")
    async def sweep3d_script():
        return FileResponse(static / "sweep3d.js", media_type="application/javascript")

    @app.get("/api/state")
    async def state():
        return Response(content=json.dumps(runner.processor.snapshot(), separators=(",", ":")),
                        media_type="application/json")

    @app.get("/api/local_state")
    async def local_state():
        return Response(content=json.dumps(runner.processor.snapshot(local=True), separators=(",", ":")),
                        media_type="application/json")

    @app.get("/api/lidar/motor")
    async def lidar_motor_status(request: Request):
        if request.client is None or request.client.host not in ("127.0.0.1", "::1"):
            raise HTTPException(status_code=403, detail="Motor control is localhost-only")
        if not hasattr(runner, "lidar_motor_status"):
            raise HTTPException(status_code=409, detail="Replay has no motor")
        return {"motor_state": runner.lidar_motor_status()}

    @app.post("/api/lidar/motor")
    async def lidar_motor(request: Request, action: str):
        if request.client is None or request.client.host not in ("127.0.0.1", "::1"):
            raise HTTPException(status_code=403, detail="Motor control is localhost-only")
        if not hasattr(runner, "pause_lidar"):
            raise HTTPException(status_code=409, detail="Replay has no motor")
        if action == "off":
            result = await asyncio.to_thread(runner.pause_lidar)
        elif action == "on":
            result = runner.resume_lidar()
        else:
            raise HTTPException(status_code=400, detail="Use action=on or action=off")
        return {"motor_state": result}

    @app.websocket("/ws")
    async def stream(socket: WebSocket):
        await socket.accept()
        try:
            while True:
                await socket.send_json(runner.processor.snapshot())
                await asyncio.sleep(0.1)
        except (WebSocketDisconnect, RuntimeError):
            pass

    return app
