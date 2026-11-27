"""Replay accepted right-front targets in MuJoCo while the robot waves."""

from __future__ import annotations

import argparse
import getpass
import inspect
import json
import math
import shlex
import sys
import time
from pathlib import Path
from queue import Empty, Queue
from threading import Thread

import mujoco
import mujoco.viewer
import numpy as np

from apex_rl.env import Meta1Env
from preview_stand_wave import fast_wave_offsets


def run_on_pi():
    import json
    import time
    from urllib.request import urlopen

    def get(path):
        with urlopen("http://127.0.0.1:8091/" + path, timeout=1.5) as response:
            return json.load(response)

    def command(query):
        packet = get("command?" + query)
        if packet.get("ok") is not True:
            raise RuntimeError(packet.get("message", "ESP command rejected"))
        return packet["message"]

    def ready(data, modes):
        spine = data["spine"]
        return (
            data["connected"] and data["motion_writes"]
            and data["spine_age_s"] < 0.5 and data["imu_age_s"] < 0.3
            and data["imu"]["online"] and spine["outputs_armed"]
            and spine["spine_state"] == "STAND" and spine["mode"] in modes
            and spine["pca_right_online"] and spine["pca_left_online"]
            and spine["config_valid"] and spine["home_set"]
            and spine["firmware_version"] == "v57.33-gait-clearance-camera"
            and 7.0 <= spine["battery_right_v"] <= 8.45
        )

    def emit(**fields):
        print(json.dumps(fields), flush=True)

    initial = get("telemetry")
    baseline = initial["imu"]["rpy_deg"][:2]
    if not ready(initial, {0}) or max(map(abs, baseline)) > 5:
        raise RuntimeError("Robot must be level, armed and in STAND")
    if not command("device=SPINE&action=PARAMS").startswith("PARAMS_-100.0_"):
        raise RuntimeError("STAND height differs from the verified reference")

    # Match the neutral RF IK in v57.33. The hardware tibia command is minus
    # the simulated bend angle. Calibration trims stay inside the firmware.
    a, b, coxa_length = 103.24, 168.41, 69.66
    radial = math.hypot(106.066, 106.066) - coxa_length
    reach = math.hypot(radial, 100.0)
    neutral = (
        135.0,
        math.degrees(math.atan2(radial, 100) + math.acos((a*a + reach*reach - b*b)/(2*a*reach))),
        180 - math.degrees(math.acos((a*a + b*b - reach*reach)/(2*a*b))),
    )

    def pose(delta):
        coxa, femur, tibia = delta
        target = (neutral[0] + coxa, neutral[1] + femur, neutral[2] - tibia)
        if not all(8 <= value <= 172 for value in target):
            raise RuntimeError("Right-front joint target outside the test range")
        command("device=LEG_LAB&action=POSE&value=0," + ",".join(f"{v:.2f}" for v in target))

    started = False
    completed = False
    peak = 0.0
    count = 0
    try:
        started = True
        pose((0, 0, 0))
        deadline = time.monotonic() + 0.8
        while True:
            data = get("telemetry")
            if ready(data, {2}):
                break
            if not ready(data, {0, 2}) or time.monotonic() > deadline:
                raise RuntimeError("Manual hold was not confirmed")
            time.sleep(0.06)
        begin = time.monotonic() - 1.0
        while True:
            t = min(19.4, time.monotonic() - begin)
            data = get("telemetry")
            if not ready(data, {2}):
                raise RuntimeError("Spine state or telemetry changed")
            attitude = data["imu"]["rpy_deg"][:2]
            peak = max(peak, *(abs(v - ref) for v, ref in zip(attitude, baseline)))
            if peak > 3.0:
                raise RuntimeError("Measured roll/pitch drift exceeded 3 degrees")
            delta = fast_wave_offsets(t)
            pose(delta)
            count += 1
            phase = "Lift" if t < 3.4 else "Wave" if t < 17.0 else "Lower"
            emit(phase=phase, seconds=round(t - 1, 3), delta=delta, attitude=attitude)
            if t >= 19.4:
                completed = True
                break
            time.sleep(0.075)
    finally:
        if started:
            spine = get("telemetry")["spine"]
            if spine["outputs_armed"] and spine["spine_state"] == "STAND" and spine["mode"] == 2:
                command("device=SPINE&action=MODE&value=0")
        time.sleep(0.9)
        final = get("telemetry")
        emit(complete=completed and ready(final, {0}), poses_accepted=count,
             max_attitude_change_deg=round(peak, 2), state=final["spine"]["spine_state"])


def stream_robot(host, password, events):
    tools = Path(__file__).resolve().parents[2] / "tools"
    sys.path.insert(0, str(tools))
    from bootstrap_pi_home_wifi_once import verified_transport

    source = "import math\n" + inspect.getsource(fast_wave_offsets) + "\n"
    source += inspect.getsource(run_on_pi) + "\nrun_on_pi()\n"
    transport = verified_transport(host, password)
    try:
        with transport.open_session(timeout=5) as channel:
            channel.settimeout(40)
            channel.set_combine_stderr(True)
            channel.exec_command("python3 -u -c " + shlex.quote(source))
            for line in channel.makefile():
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    print(line.rstrip(), flush=True)
                    continue
                events.put(event)
                if "complete" in event:
                    print(json.dumps(event), flush=True)
            if channel.recv_exit_status() != 0:
                raise RuntimeError("Pi stopped the wave")
    finally:
        transport.close()


def view(host, password):
    env = Meta1Env()
    events = Queue()

    def worker():
        try:
            stream_robot(host, password, events)
        except Exception as error:
            events.put({"error": str(error)})
            print(f"Wave stopped: {error}", flush=True)

    thread = Thread(target=worker)
    try:
        env.reset(seed=42)
        for _ in range(50):
            env._apply_joint_targets(env.nominal)
        delta = np.zeros(3)
        labels = ("Standby", "Robot command pending")
        with mujoco.viewer.launch_passive(env.model, env.data) as window:
            window.cam.lookat[:] = (0.04, 0, 0.1)
            window.cam.distance, window.cam.azimuth, window.cam.elevation = 1.12, 128, -25
            window.sync()
            print("MuJoCo window opened; wave begins in four seconds.", flush=True)
            trigger_at = time.monotonic() + 4
            while window.is_running():
                tick = time.monotonic()
                if thread.ident is None and tick >= trigger_at:
                    thread.start()
                while not events.empty():
                    try:
                        packet = events.get_nowait()
                    except Empty:
                        break
                    if "delta" in packet:
                        delta = np.deg2rad(packet["delta"])
                        roll, pitch = packet["attitude"]
                        labels = (
                            f"{packet['phase']} {packet['seconds']:.1f} s",
                            f"Robot IMU  roll {roll:+.1f}  pitch {pitch:+.1f} deg",
                        )
                    elif "complete" in packet:
                        delta = np.zeros(3)
                        labels = ("STAND" if packet["complete"] else "Stopped", packet["state"])
                    elif "error" in packet:
                        labels = ("Stopped", packet["error"])
                target = env.nominal.copy()
                target[:3] += delta
                with window.lock():
                    env._apply_joint_targets(target)
                window.set_texts([
                    (None, None, "MUJOCO | accepted targets", labels[0]),
                    (None, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, labels[1], ""),
                ])
                window.sync()
                time.sleep(max(0, 0.02 - (time.monotonic() - tick)))
    finally:
        if thread.ident is not None:
            thread.join()
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", metavar="PI_IP", required=True,
                        help="perform one guarded wave on a robot already in STAND")
    args = parser.parse_args()
    password = getpass.getpass("Pi SSH password: ") if sys.stdin.isatty() else sys.stdin.readline().strip()
    view(args.execute, password)
