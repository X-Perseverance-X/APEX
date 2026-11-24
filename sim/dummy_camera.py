"""MJPEG test camera with freshness and freeze fault injection."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from io import BytesIO
import time

from fastapi import FastAPI
from fastapi.responses import Response, StreamingResponse
from PIL import Image, ImageDraw


FREEZE = False
LAST_FRAME_MONO = time.monotonic()
FROZEN_FRAME: bytes | None = None


def _jpeg_frame() -> bytes:
    global LAST_FRAME_MONO, FROZEN_FRAME
    if FREEZE and FROZEN_FRAME is not None:
        return FROZEN_FRAME
    now = time.time()
    image = Image.new("RGB", (960, 540), (5, 10, 14))
    draw = ImageDraw.Draw(image)
    for x in range(0, 960, 48):
        draw.line((x, 0, x, 540), fill=(12, 38, 42), width=1)
    for y in range(0, 540, 48):
        draw.line((0, y, 960, y), fill=(12, 38, 42), width=1)
    cx = int(480 + 260 * __import__("math").sin(now * 0.55))
    cy = int(270 + 110 * __import__("math").cos(now * 0.41))
    draw.rectangle((cx - 45, cy - 45, cx + 45, cy + 45), outline=(255, 48, 48), width=6)
    draw.text((26, 24), "APEX META-1 / DUMMY CAMERA", fill=(0, 255, 210))
    draw.text((26, 52), time.strftime("%Y-%m-%d %H:%M:%S"), fill=(220, 230, 235))
    draw.text((26, 80), "NO PHYSICAL CAMERA CONNECTED", fill=(255, 180, 40))
    output = BytesIO()
    image.save(output, format="JPEG", quality=82)
    data = output.getvalue()
    LAST_FRAME_MONO = time.monotonic()
    FROZEN_FRAME = data
    return data


async def _capture_loop() -> None:
    while True:
        if not FREEZE:
            _jpeg_frame()
        await asyncio.sleep(0.1)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    task = asyncio.create_task(_capture_loop())
    try:
        yield
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)


app = FastAPI(title="APEX Dummy Camera", version="1.0", lifespan=lifespan)


async def _stream():
    while True:
        frame = _jpeg_frame()
        yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
        await asyncio.sleep(0.1)


@app.get("/")
async def root() -> dict:
    return {"service": "apex_dummy_camera", "ok": True, "stream": "/stream"}


@app.get("/stream")
async def stream() -> StreamingResponse:
    return StreamingResponse(_stream(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/snapshot")
async def snapshot() -> Response:
    return Response(_jpeg_frame(), media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/status")
async def status() -> dict:
    age = time.monotonic() - LAST_FRAME_MONO
    return {"camera": age < 1.0, "fresh": age < 1.0, "frame_age": age, "dummy": True, "frozen": FREEZE}


@app.get("/cube/status")
async def cube_status() -> dict:
    return {"found": True, "dummy": True, "x_cam_mm": 40.0, "y_cam_mm": 30.0, "z_cam_mm": 420.0, "confidence": 0.99}


@app.get("/sim/freeze")
async def freeze(state: int = 1) -> dict:
    global FREEZE
    if not FREEZE and state:
        _jpeg_frame()
    FREEZE = bool(state)
    return await status()
