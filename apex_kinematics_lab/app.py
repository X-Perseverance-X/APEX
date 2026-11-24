#!/usr/bin/env python3
"""APEX Kinematics Lab — META-1 semi-digital-twin UI.

This service intentionally does not modify the Command Center or ESP32 firmware.
It reads live robot state from the already-running Command Center on port 8080
and exposes a single cached endpoint to the browser on port 8090.
"""
from __future__ import annotations

import asyncio
import hashlib
import hmac
import json
import os
import time
from pathlib import Path
from typing import Any

import httpx
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"
CONFIG = ROOT / "config" / "default_robot.json"
COMMAND_CENTER_URL = os.environ.get("APEX_COMMAND_CENTER_URL", "http://127.0.0.1:8080").rstrip("/")

app = FastAPI(title="APEX Kinematics Lab", version="3.1.0")

AUTH_ENABLED = os.environ.get("APEX_AUTH_ENABLED", "0").strip().lower() in {"1", "true", "yes", "on"}
AUTH_USER = os.environ.get("APEX_AUTH_USER", "apex")
AUTH_SECRET = os.environ.get("APEX_AUTH_SECRET", "")


def _session_valid(value: str | None) -> bool:
    if not AUTH_ENABLED:
        return True
    if not value or not AUTH_SECRET:
        return False
    try:
        user, expires_text, signature = value.rsplit("|", 2)
        expires = int(expires_text)
    except (ValueError, TypeError):
        return False
    payload = f"{user}|{expires}"
    expected = hmac.new(AUTH_SECRET.encode(), payload.encode(), hashlib.sha256).hexdigest()
    return user == AUTH_USER and expires >= int(time.time()) and hmac.compare_digest(signature, expected)


@app.middleware("http")
async def require_control_session(request: Request, call_next):
    if _session_valid(request.cookies.get("apex_session")):
        return await call_next(request)
    host = (request.headers.get("host") or "192.168.7.1").split(":", 1)[0]
    return Response(status_code=303, headers={"Location": f"http://{host}:8080/login"})
app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")


def read_config() -> dict[str, Any]:
    if not CONFIG.exists():
        raise HTTPException(status_code=500, detail="default_robot.json bulunamadı")
    return json.loads(CONFIG.read_text(encoding="utf-8"))


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return (STATIC / "index.html").read_text(encoding="utf-8")


@app.get("/api/config")
def get_config() -> dict[str, Any]:
    return read_config()


@app.post("/api/config")
async def save_config(request: Request) -> dict[str, Any]:
    data = await request.json()
    if not isinstance(data, dict):
        raise HTTPException(status_code=400, detail="JSON object bekleniyor")
    CONFIG.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return {"ok": True, "path": str(CONFIG)}


@app.get("/api/export")
def export_config() -> dict[str, Any]:
    return read_config()


class LiveCache:
    """Small cache to avoid overloading the ESP32's single-threaded HTTP server.

    /telemetry is allowed to refresh more often than /legStatus because the latter
    is only needed for command-state overlay and angle comparison.
    """

    def __init__(self) -> None:
        self.lock = asyncio.Lock()
        self.telemetry: dict[str, Any] | None = None
        self.telemetry_ts = 0.0
        self.legs: dict[str, Any] | None = None
        self.legs_ts = 0.0
        self.last_error = ""


LIVE = LiveCache()
TELEMETRY_TTL_S = float(os.environ.get("APEX_LIVE_TELEMETRY_TTL", "0.10"))
LEG_STATUS_TTL_S = float(os.environ.get("APEX_LIVE_LEG_TTL", "0.12"))
LIVE_HTTP_TIMEOUT_S = float(os.environ.get("APEX_LIVE_HTTP_TIMEOUT", "0.8"))
LIVE_STALE_S = float(os.environ.get("APEX_LIVE_STALE", "0.75"))


_HTTP_CLIENT: httpx.AsyncClient | None = None


async def _live_client() -> httpx.AsyncClient:
    global _HTTP_CLIENT
    if _HTTP_CLIENT is None:
        limits = httpx.Limits(max_connections=8, max_keepalive_connections=4)
        timeout = httpx.Timeout(LIVE_HTTP_TIMEOUT_S, connect=0.35)
        _HTTP_CLIENT = httpx.AsyncClient(timeout=timeout, limits=limits)
    return _HTTP_CLIENT


@app.on_event("shutdown")
async def _close_live_client() -> None:
    global _HTTP_CLIENT
    if _HTTP_CLIENT is not None:
        await _HTTP_CLIENT.aclose()
        _HTTP_CLIENT = None


async def _get_json(path: str) -> dict[str, Any]:
    client = await _live_client()
    response = await client.get(
        f"{COMMAND_CENTER_URL}{path}",
        headers={"Cache-Control": "no-cache", "Connection": "keep-alive"},
    )
    if response.is_error:
        raise RuntimeError(f"{path} HTTP {response.status_code}")
    data = response.json()
    if not isinstance(data, dict):
        raise ValueError(f"{path} JSON object döndürmedi")
    return data


async def _refresh_live() -> tuple[dict[str, Any] | None, dict[str, Any] | None, str]:
    now = time.monotonic()
    async with LIVE.lock:
        needs_telemetry = LIVE.telemetry is None or (now - LIVE.telemetry_ts) >= TELEMETRY_TTL_S
        needs_legs = LIVE.legs is None or (now - LIVE.legs_ts) >= LEG_STATUS_TTL_S

        tasks: list[tuple[str, asyncio.Task[dict[str, Any]]]] = []
        if needs_telemetry:
            tasks.append(("telemetry", asyncio.create_task(_get_json("/telemetry"))))
        if needs_legs:
            tasks.append(("legs", asyncio.create_task(_get_json("/legStatus"))))

        errors: list[str] = []
        for kind, task in tasks:
            try:
                data = await task
                if kind == "telemetry":
                    LIVE.telemetry = data
                    LIVE.telemetry_ts = time.monotonic()
                else:
                    LIVE.legs = data
                    LIVE.legs_ts = time.monotonic()
            except Exception as exc:  # keep last good cache when a poll fails
                errors.append(f"{kind}: {_exception_text(exc)}")

        LIVE.last_error = " | ".join(errors)
        return LIVE.telemetry, LIVE.legs, LIVE.last_error


def _cache_is_fresh(timestamp: float, now: float | None = None) -> bool:
    if timestamp <= 0:
        return False
    current = time.monotonic() if now is None else now
    return 0 <= current - timestamp <= LIVE_STALE_S


def _exception_text(exc: Exception) -> str:
    message = str(exc).strip()
    return message or exc.__class__.__name__


@app.get("/api/live")
async def live_snapshot() -> JSONResponse:
    started = time.perf_counter()
    telemetry, legs, error = await _refresh_live()
    now_mono = time.monotonic()
    telemetry_fresh = telemetry is not None and _cache_is_fresh(LIVE.telemetry_ts, now_mono)
    legs_fresh = legs is not None and _cache_is_fresh(LIVE.legs_ts, now_mono)
    payload: dict[str, Any] = {
        "online": telemetry_fresh,
        "source": "command_center_proxy",
        "commandCenter": COMMAND_CENTER_URL,
        "serverTs": time.time(),
        "latencyMs": round((time.perf_counter() - started) * 1000.0, 2),
        "telemetryAgeMs": None if telemetry is None else round((now_mono - LIVE.telemetry_ts) * 1000.0, 1),
        "legStatusAgeMs": None if legs is None else round((now_mono - LIVE.legs_ts) * 1000.0, 1),
        "telemetry": telemetry,
        "legStatus": legs,
        "telemetryFresh": telemetry_fresh,
        "legStatusFresh": legs_fresh,
        "error": error or None,
        "provenance": {
            "imu_tof_battery": "live_sensor",
            "gait_phase": "firmware_control_state",
            "servo_angles": "firmware_command_state_not_encoder_feedback",
            "contact": "physical_if_present_else_phase_estimated",
        },
    }
    return JSONResponse(payload, headers={"Cache-Control": "no-store, max-age=0"})


@app.get("/api/live/health")
async def live_health() -> dict[str, Any]:
    telemetry, legs, error = await _refresh_live()
    now_mono = time.monotonic()
    telemetry_fresh = telemetry is not None and _cache_is_fresh(LIVE.telemetry_ts, now_mono)
    legs_fresh = legs is not None and _cache_is_fresh(LIVE.legs_ts, now_mono)
    return {
        "ok": telemetry_fresh,
        "telemetry": telemetry_fresh,
        "legStatus": legs_fresh,
        "error": error or None,
        "source": COMMAND_CENTER_URL,
    }


@app.get("/favicon.ico")
def favicon() -> Response:
    return Response(status_code=204)
