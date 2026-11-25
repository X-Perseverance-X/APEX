#!/usr/bin/env python3
"""Approved, bounded body IK exercise on the supported robot."""
import getpass
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".runtime" / "pydeps"))
from bootstrap_pi_home_wifi_once import remote, verified_transport  # noqa: E402

password = getpass.getpass("Pi SSH password: ")
transport = verified_transport("apex.local", password)


def send(pitch: int, roll: int):
    path = ("http://127.0.0.1:8091/command?device=SPINE&action=BODY"
            f"&value={pitch},{roll},0")
    result = json.loads(remote(transport, f"curl -fsS --max-time 8 '{path}'"))
    print(f"BODY p={pitch} r={roll}: {result}", flush=True)
    if not result.get("ok"):
        raise RuntimeError("ESP rejected body IK")


try:
    current = json.loads(remote(transport,
        "curl -fsS --max-time 5 http://127.0.0.1:8091/telemetry"))
    spine = current.get("spine", {})
    if not (current.get("connected") and current.get("spine_age_s", 999) < 1
            and spine.get("firmware_version") == "v57.31-stand-splay-body-ik"
            and spine.get("spine_state") == "STAND" and spine.get("outputs_armed")):
        raise RuntimeError("Body IK preconditions failed; no command sent")
    try:
        send(3, 0)
        time.sleep(2)
        send(0, 0)
        time.sleep(1)
        send(0, 3)
        time.sleep(2)
    finally:
        send(0, 0)
finally:
    transport.close()
