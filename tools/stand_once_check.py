#!/usr/bin/env python3
"""One approved, guarded STAND test; no body motion or walking."""
import getpass
import json
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / ".runtime" / "pydeps"))
from bootstrap_pi_home_wifi_once import remote, verified_transport  # noqa: E402


def main():
    password = getpass.getpass("Pi SSH password: ")
    transport = verified_transport("apex.local", password)
    try:
        def telemetry():
            return json.loads(remote(transport,
                "curl -fsS --max-time 5 http://127.0.0.1:8091/telemetry"))
        initial = telemetry()
        spine = initial.get("spine", {})
        print("Before:", {k: spine.get(k) for k in (
            "firmware_version", "spine_state", "outputs_armed", "pca_right_online",
            "pca_left_online", "home_set", "config_valid")}, flush=True)
        if not (initial.get("connected") and initial.get("spine_age_s", 999) < 1
                and spine.get("firmware_version") == "v57.31-stand-splay-body-ik"
                and spine.get("spine_state") == "DISARMED"
                and not spine.get("outputs_armed")
                and spine.get("pca_right_online") and spine.get("pca_left_online")
                and spine.get("home_set") and spine.get("config_valid")):
            raise RuntimeError("STAND preconditions failed; no command sent")
        reply = json.loads(remote(transport,
            "curl -fsS --max-time 8 'http://127.0.0.1:8091/command?device=SPINE&action=STAND'"))
        print("STAND acknowledgement:", reply, flush=True)
        if not reply.get("ok"):
            return 2
        for seconds in (2, 5, 10, 15, 20):
            time.sleep(seconds - (0 if seconds == 2 else previous))
            previous = seconds
            state = telemetry()
            core = state.get("spine", {})
            print(f"t={seconds}s state={core.get('spine_state')} armed={core.get('outputs_armed')} "
                  f"mode={core.get('mode')} age={state.get('spine_age_s'):.2f}s", flush=True)
            if core.get("spine_state") in ("FAILSAFE", "FAULT", "DISARMED"):
                break
        return 0
    finally:
        transport.close()


if __name__ == "__main__":
    raise SystemExit(main())
