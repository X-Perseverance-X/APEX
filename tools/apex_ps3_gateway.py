#!/usr/bin/env python3
"""Sony DualShock 3 Bluetooth joystick -> local USB spine bridge.

Motion is opt-in: L1 is a deadman, and L1+Cross must be held two seconds
before STAND is requested. The ESP never arms on a joystick packet alone.
"""

from __future__ import annotations

import json
import math
import os
import select
import struct
import subprocess
import time
import urllib.parse
import urllib.request
from collections import deque


BRIDGE = "http://127.0.0.1:8091"
HUD = "http://127.0.0.1:8080"
DEVICE = "/dev/input/js0"
PERIOD = 0.1
BUTTON = {"cross": 0, "circle": 1, "square": 2, "triangle": 3,
          "l1": 4, "r1": 5, "l2": 6, "r2": 7, "select": 8,
          "start": 9, "ps": 10, "l3": 11, "r3": 12,
          "up": 13, "down": 14, "left": 15, "right": 16}


def command(device: str, action: str, value: str = "", timeout: float = 0.35) -> dict:
    query = urllib.parse.urlencode({"device": device, "action": action, "value": value})
    try:
        with urllib.request.urlopen(f"{BRIDGE}/command?{query}", timeout=timeout) as response:
            return json.load(response)
    except Exception as exc:
        return {"ok": False, "message": str(exc)}


def spine() -> dict:
    try:
        with urllib.request.urlopen(f"{BRIDGE}/telemetry", timeout=0.25) as response:
            raw = json.load(response)
        if not raw.get("ok") or raw.get("spine_age_s", 99) > 0.3:
            return {}
        return raw.get("spine") or {}
    except Exception:
        return {}


def hud(path: str, payload: dict | None = None) -> dict:
    try:
        body = None if payload is None else json.dumps(payload).encode()
        request = urllib.request.Request(HUD + path, data=body,
                                         headers={"Content-Type": "application/json"} if body else {})
        with urllib.request.urlopen(request, timeout=1.0) as response:
            return json.load(response)
    except Exception as exc:
        return {"ok": False, "message": str(exc)}


def shaped(raw: int) -> float:
    value = max(-1.0, min(1.0, raw / 32767.0))
    if abs(value) < 0.13:
        return 0.0
    value = math.copysign((abs(value) - 0.13) / 0.87, value)
    return value * abs(value)


def network_toggle() -> None:
    """Switch only between an existing home profile and the robot AP."""
    home = os.environ.get("APEX_HOME_PROFILE", "").strip()
    if not home:
        print("[PS3] Home Wi-Fi profile not configured; network unchanged", flush=True)
        return
    profiles = subprocess.run(["nmcli", "-g", "NAME", "connection", "show"],
                              capture_output=True, text=True, timeout=3, check=False).stdout.splitlines()
    if home not in profiles or "APEX-META1" not in profiles:
        print("[PS3] Named network profile missing; network unchanged", flush=True)
        return
    active = subprocess.run(["nmcli", "-g", "NAME", "connection", "show", "--active"],
                            capture_output=True, text=True, timeout=3, check=False).stdout.splitlines()
    target = "APEX-META1" if home in active else home
    result = subprocess.run(["nmcli", "connection", "up", target],
                            capture_output=True, text=True, timeout=20, check=False)
    print(f"[PS3] Network {target}: {result.returncode}", flush=True)


def run() -> None:
    speed = 0.18
    camera_pan = camera_tilt = 90.0
    lidar_enabled = False
    ai_enabled = False
    approval_mode = "manual"
    while True:
        try:
            fd = os.open(DEVICE, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            time.sleep(1)
            continue
        print("[PS3] Wireless controller present; waiting for L1 + Cross", flush=True)
        axes = [0] * 6
        buttons = [False] * 17
        edges: set[int] = set()
        stand_since = None
        switch_since = None
        stand_latched = False
        switch_latched = False
        moving = False
        walking = False
        drive_inhibit = False
        button_changes = deque()
        noisy_until = 0.0
        next_tick = time.monotonic()
        last_camera_tick = 0.0
        try:
            while True:
                readable, _, _ = select.select([fd], [], [], max(0, next_tick - time.monotonic()))
                if readable:
                    data = os.read(fd, 8)
                    if len(data) != 8:
                        break
                    _, value, kind, number = struct.unpack("<IhBB", data)
                    kind &= 0x7F
                    if kind == 1 and number < len(buttons):
                        if bool(value) != buttons[number]:
                            button_changes.append(time.monotonic())
                        if value and not buttons[number]:
                            edges.add(number)
                        buttons[number] = bool(value)
                    elif kind == 2 and number < len(axes):
                        axes[number] = value
                now = time.monotonic()
                if now < next_tick:
                    continue
                next_tick = now + PERIOD
                pressed, edges = edges, set()
                while button_changes and now - button_changes[0] > 1.0:
                    button_changes.popleft()
                if len(button_changes) >= 8:
                    if now >= noisy_until:
                        print("[PS3] Noisy button input; motion inhibited", flush=True)
                    noisy_until = now + 3.0
                if now < noisy_until:
                    if walking or moving:
                        command("SPINE", "JOY", "0,0,0")
                        command("SPINE", "MODE", "0")
                        walking = moving = False
                    drive_inhibit = True
                    continue
                held = buttons[BUTTON["l1"]]
                cross = buttons[BUTTON["cross"]]
                combo = buttons[BUTTON["select"]] and buttons[BUTTON["start"]]
                if combo:
                    switch_since = now if switch_since is None else switch_since
                    if now - switch_since >= 3 and not switch_latched:
                        command("SPINE", "JOY", "0,0,0")
                        command("SPINE", "MODE", "0")
                        walking = moving = False
                        drive_inhibit = True
                        switch_latched = True
                        network_toggle()
                else:
                    switch_since = None
                    switch_latched = False
                if BUTTON["ps"] in pressed or BUTTON["circle"] in pressed:
                    command("SPINE", "JOY", "0,0,0")
                    command("SPINE", "MODE", "0")
                    walking = moving = False
                    drive_inhibit = True
                    stand_latched = True
                    continue
                if BUTTON["triangle"] in pressed:
                    result = hud(f"/lidar/motor?state={0 if lidar_enabled else 1}")
                    if result.get("applied"):
                        lidar_enabled = bool(result.get("enabled"))
                if BUTTON["square"] in pressed:
                    result = hud(f"/ai_tracking?state={0 if ai_enabled else 1}")
                    if "enabled" in result:
                        ai_enabled = bool(result["enabled"])
                if BUTTON["r3"] in pressed:
                    camera_pan = camera_tilt = 90.0
                    command("CAM_GIMBAL", "CENTER")
                if BUTTON["l3"] in pressed and not moving:
                    speed = 0.10 if speed > 0.12 else 0.18
                if BUTTON["start"] in pressed and not buttons[BUTTON["select"]] and not moving:
                    target_mode = "auto" if approval_mode == "manual" else "manual"
                    result = hud("/api/navigation/mode", {"mode": target_mode})
                    if result.get("ok"):
                        approval_mode = target_mode
                if BUTTON["select"] in pressed and not buttons[BUTTON["start"]]:
                    print(f"[PS3] state={spine().get('spine_state')} speed={speed:.2f} "
                          f"lidar={lidar_enabled} ai={ai_enabled}", flush=True)
                if BUTTON["left"] in pressed:
                    camera_pan = max(45.0, camera_pan - 3.0)
                    command("CAM_PAN", "SET", f"{camera_pan:.1f}")
                if BUTTON["right"] in pressed:
                    camera_pan = min(135.0, camera_pan + 3.0)
                    command("CAM_PAN", "SET", f"{camera_pan:.1f}")
                if now - last_camera_tick >= 0.3:
                    pan_input = int(buttons[BUTTON["r2"]]) - int(buttons[BUTTON["l2"]])
                    tilt_input = shaped(axes[3])
                    if pan_input:
                        camera_pan = max(45.0, min(135.0, camera_pan + 2.0 * pan_input))
                        command("CAM_PAN", "SET", f"{camera_pan:.1f}")
                    if abs(tilt_input) > 0.05:
                        camera_tilt = max(45.0, min(135.0, camera_tilt + 2.0 * tilt_input))
                        command("CAM_TILT", "SET", f"{camera_tilt:.1f}")
                    last_camera_tick = now
                state = spine()
                if held and cross and not combo and not moving and state.get("spine_state") == "DISARMED":
                    stand_since = now if stand_since is None else stand_since
                    if now - stand_since >= 2 and not stand_latched:
                        result = command("SPINE", "STAND")
                        stand_latched = True
                        print(f"[PS3] STAND: {result}", flush=True)
                else:
                    stand_since = None
                    if not cross:
                        stand_latched = False
                if cross:
                    if walking or moving:
                        command("SPINE", "JOY", "0,0,0")
                        command("SPINE", "MODE", "0")
                        walking = moving = False
                    continue
                if not held and not button_changes:
                    drive_inhibit = False
                if not held or combo or drive_inhibit or state.get("spine_state") not in {"STAND", "WALK"}:
                    if walking or moving:
                        command("SPINE", "JOY", "0,0,0")
                        command("SPINE", "MODE", "0")
                        walking = moving = False
                    continue
                x = shaped(axes[0]) * speed
                y = -shaped(axes[1]) * speed
                r = shaped(axes[2]) * speed
                if state.get("spine_state") == "STAND":
                    if max(abs(x), abs(y), abs(r)) <= 0.02:
                        continue
                    result = command("SPINE", "MODE", "1")
                    if not result.get("ok"):
                        drive_inhibit = True
                        continue
                    walking = True
                if buttons[BUTTON["up"]] and speed < 0.35:
                    speed = min(0.35, speed + 0.01)
                elif buttons[BUTTON["down"]] and speed > 0.08:
                    speed = max(0.08, speed - 0.01)
                # Preserve a forward arc. Pure yaw is possible only with the
                # left stick centered and is separately limited by the gait.
                if abs(y) > 0.02:
                    r = max(-abs(y) * 0.8, min(abs(y) * 0.8, r))
                result = command("SPINE", "JOY", f"{x:.3f},{y:.3f},{r:.3f}")
                if result.get("ok"):
                    moving = max(abs(x), abs(y), abs(r)) > 0.02
                else:
                    print(f"[PS3] JOY failed; stopping: {result.get('message')}", flush=True)
                    command("SPINE", "JOY", "0,0,0")
                    command("SPINE", "MODE", "0")
                    walking = moving = False
                    drive_inhibit = True
        except (OSError, ValueError) as exc:
            print(f"[PS3] Controller disconnected: {exc}", flush=True)
        finally:
            # The ESP independently times out within 750 ms if USB is lost.
            if walking or moving:
                command("SPINE", "JOY", "0,0,0")
                command("SPINE", "MODE", "0")
            os.close(fd)
            time.sleep(0.5)


if __name__ == "__main__":
    run()
