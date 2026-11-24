"""Stateful ESP32 substitute for UI, transport, and failsafe testing."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
import math
import random
import time
from typing import Any

from fastapi import FastAPI, HTTPException, Query, Request
from fastapi.responses import JSONResponse

from shared.apex_protocol import CommandFlags, ControlMode, MotionCommand


LEG_NAMES = ("RF", "RM", "RR", "LF", "LM", "LR")
LEG_PHASE = (0.0, 0.5, 0.0, 0.5, 0.0, 0.5)
LEG_PINS = ((0, 1, 2), (4, 5, 6), (8, 10, 9), (0, 1, 2), (4, 5, 6), (8, 9, 10))
LEG_DIR = ((-1, 1, 1),) * 3 + ((1, -1, -1),) * 3
HTTP_DEADMAN_S = 0.300
UPDATE_PERIOD_S = 0.020


def _nominal_angles(leg: int) -> list[float]:
    return [90.0, 131.0, 130.0] if leg < 3 else [90.0, 49.0, 50.0]


def _approach(current: float, target: float, max_delta: float) -> float:
    return max(current - max_delta, min(current + max_delta, target))


@dataclass
class DummyState:
    lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    boot_mono: float = field(default_factory=time.monotonic)
    session_id: int = field(default_factory=lambda: random.randint(1, 0xFFFF))
    sequence: int = 0
    last_command_mono: float = field(default_factory=time.monotonic)
    last_protocol_frame: str = ""
    mode: str = "IDLE"
    joy_x: float = 0.0
    joy_y: float = 0.0
    joy_r: float = 0.0
    gait_phase: float = 0.0
    gait_z: float = -100.0
    step_length: float = 100.0
    step_lift: float = 50.0
    cycle_ms: float = 1000.0
    body_pitch: float = 0.0
    body_roll: float = 0.0
    body_yaw: float = 0.0
    gimbal: bool = False
    gimbal_filter: float = 20.0
    failsafe: bool = False
    failsafe_count: int = 0
    drop_commands: bool = False
    telemetry_offline: bool = False
    imu_fault: bool = False
    tof_fault: bool = False
    battery_right_v: float = 12.60
    battery_left_v: float = 12.55
    pwm_enabled: list[list[bool]] = field(default_factory=lambda: [[True] * 3 for _ in range(6)])
    current_angles: list[list[float]] = field(default_factory=lambda: [_nominal_angles(i) for i in range(6)])
    target_angles: list[list[float]] = field(default_factory=lambda: [_nominal_angles(i) for i in range(6)])
    home_angles: list[list[float]] = field(default_factory=lambda: [_nominal_angles(i) for i in range(6)])
    arm_angles: dict[str, float] = field(default_factory=lambda: {"base": 90.0, "joint1": 5.0, "joint2": 5.0, "gripper": 90.0})
    arm_enabled: bool = False
    calibration: list[dict[str, int]] = field(
        default_factory=lambda: [
            {"cMin": 500, "cMax": 2500, "cOff": 0, "fMin": 500, "fMax": 2500, "fOff": 0, "tMin": 500, "tMax": 2500, "tOff": 0}
            for _ in range(6)
        ]
    )

    def command_age_ms(self) -> int:
        return max(0, int((time.monotonic() - self.last_command_mono) * 1000.0))


STATE = DummyState()


async def _apply_motion_command(command: MotionCommand) -> None:
    # Encode + decode deliberately exercises the exact radio/SIL wire contract.
    frame = command.encode()
    decoded = MotionCommand.decode(frame)
    STATE.last_protocol_frame = frame.hex()
    STATE.sequence = decoded.sequence
    STATE.last_command_mono = time.monotonic()
    STATE.joy_x = decoded.joy_x_milli / 1000.0
    STATE.joy_y = decoded.joy_y_milli / 1000.0
    STATE.joy_r = decoded.joy_r_milli / 1000.0
    STATE.gait_z = float(decoded.gait_z_mm)
    STATE.step_length = float(decoded.step_length_mm)
    STATE.step_lift = float(decoded.step_lift_mm)
    STATE.cycle_ms = float(decoded.cycle_ms)
    STATE.body_pitch = decoded.body_pitch_cdeg / 100.0
    STATE.body_roll = decoded.body_roll_cdeg / 100.0
    STATE.body_yaw = decoded.body_yaw_cdeg / 100.0
    STATE.gimbal = bool(decoded.flags & CommandFlags.GIMBAL_ENABLE)


async def _simulation_loop() -> None:
    last = time.monotonic()
    while True:
        await asyncio.sleep(UPDATE_PERIOD_S)
        now = time.monotonic()
        dt = min(0.1, now - last)
        last = now
        async with STATE.lock:
            if STATE.mode == "YURUYUS" and now - STATE.last_command_mono > HTTP_DEADMAN_S:
                STATE.joy_x = STATE.joy_y = STATE.joy_r = 0.0
                STATE.mode = "IDLE"
                if not STATE.failsafe:
                    STATE.failsafe = True
                    STATE.failsafe_count += 1

            magnitude = min(1.0, math.sqrt(STATE.joy_x**2 + STATE.joy_y**2 + STATE.joy_r**2))
            if STATE.mode == "YURUYUS" and magnitude > 0.02:
                STATE.gait_phase = (STATE.gait_phase + dt * 1000.0 / max(250.0, STATE.cycle_ms) * magnitude) % 1.0

            for leg in range(6):
                nominal = _nominal_angles(leg)
                target = nominal[:]
                if STATE.mode == "YURUYUS":
                    phase = (STATE.gait_phase + LEG_PHASE[leg]) % 1.0
                    wave = math.sin(phase * 2.0 * math.pi)
                    lift = max(0.0, math.sin(phase * 2.0 * math.pi))
                    side = 1.0 if leg < 3 else -1.0
                    target[0] += wave * (STATE.joy_y * 16.0 - side * STATE.joy_r * 12.0)
                    target[1] += lift * STATE.step_lift * 0.22
                    target[2] -= lift * STATE.step_lift * 0.30
                STATE.target_angles[leg] = [max(5.0, min(175.0, x)) for x in target]
                for joint in range(3):
                    if STATE.pwm_enabled[leg][joint]:
                        STATE.current_angles[leg][joint] = _approach(
                            STATE.current_angles[leg][joint], STATE.target_angles[leg][joint], 180.0 * dt
                        )


@asynccontextmanager
async def lifespan(_app: FastAPI):
    task = asyncio.create_task(_simulation_loop())
    try:
        yield
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)


app = FastAPI(title="APEX Dummy Robot", version="1.0", lifespan=lifespan)


@app.get("/")
async def root() -> dict[str, Any]:
    return {"service": "apex_dummy_robot", "ok": True, "protocol": 1, "session": STATE.session_id}


@app.get("/joy")
async def joy(
    x: float = Query(0.0, ge=-1.0, le=1.0),
    y: float = Query(0.0, ge=-1.0, le=1.0),
    r: float = Query(0.0, ge=-1.0, le=1.0),
) -> dict[str, Any]:
    async with STATE.lock:
        if STATE.drop_commands:
            raise HTTPException(status_code=503, detail="simulated command link loss")
        command = MotionCommand(
            sequence=(STATE.sequence + 1) & 0xFFFF,
            session_id=STATE.session_id,
            flags=CommandFlags.GIMBAL_ENABLE if STATE.gimbal else CommandFlags.NONE,
            mode=ControlMode.GAIT if STATE.mode == "YURUYUS" else ControlMode.IDLE,
            joy_x_milli=round(x * 1000),
            joy_y_milli=round(y * 1000),
            joy_r_milli=round(r * 1000),
            gait_z_mm=round(STATE.gait_z),
            step_length_mm=round(STATE.step_length),
            step_lift_mm=round(STATE.step_lift),
            cycle_ms=round(STATE.cycle_ms),
            body_pitch_cdeg=round(STATE.body_pitch * 100),
            body_roll_cdeg=round(STATE.body_roll * 100),
            body_yaw_cdeg=round(STATE.body_yaw * 100),
        )
        await _apply_motion_command(command)
        return {"ok": True, "sequence": STATE.sequence, "frame": STATE.last_protocol_frame}


@app.get("/mode")
async def mode(m: str) -> dict[str, Any]:
    mapping = {"idle": "IDLE", "bezier_joy": "YURUYUS", "cal": "CAL(KILITLI)", "home_rise": "KALKIS"}
    if m not in mapping:
        raise HTTPException(status_code=400, detail="unknown mode")
    async with STATE.lock:
        if STATE.drop_commands:
            raise HTTPException(status_code=503, detail="simulated command link loss")
        STATE.mode = mapping[m]
        STATE.last_command_mono = time.monotonic()
        if m in {"idle", "bezier_joy"}:
            STATE.failsafe = False
        return {"ok": True, "mode": STATE.mode}


@app.get("/params")
async def params(z: float = -100, len: float = 100, lift: float = 50, spd: float = 1000) -> dict[str, Any]:
    if not (-300 <= z <= 0 and 0 <= len <= 250 and 0 <= lift <= 180 and 250 <= spd <= 5000):
        raise HTTPException(status_code=422, detail="parameter outside protocol range")
    async with STATE.lock:
        STATE.gait_z, STATE.step_length, STATE.step_lift, STATE.cycle_ms = z, len, lift, spd
    return {"ok": True}


@app.get("/bodyIk")
async def body_ik(p: float = 0, r: float = 0, y: float = 0) -> dict[str, Any]:
    if not (-30 <= p <= 30 and -30 <= r <= 30 and -45 <= y <= 45):
        raise HTTPException(status_code=422, detail="body angle outside protocol range")
    async with STATE.lock:
        STATE.body_pitch, STATE.body_roll, STATE.body_yaw = p, r, y
    return {"ok": True}


@app.get("/setGimbal")
async def set_gimbal(state: str = "0") -> dict[str, Any]:
    STATE.gimbal = state == "1"
    return {"ok": True, "gimbal": STATE.gimbal}


@app.get("/setGimbalFilter")
async def set_gimbal_filter(v: float = Query(20, ge=0, le=100)) -> dict[str, Any]:
    STATE.gimbal_filter = v
    return {"ok": True}


@app.get("/telemetry")
async def telemetry() -> JSONResponse:
    if STATE.telemetry_offline:
        raise HTTPException(status_code=503, detail="simulated telemetry outage")
    async with STATE.lock:
        t = time.monotonic() - STATE.boot_mono
        pitch = 0.0 if STATE.imu_fault else STATE.body_pitch + math.sin(t * 0.7) * 1.2
        roll = 0.0 if STATE.imu_fault else STATE.body_roll + math.sin(t * 0.43) * 0.8
        payload = {
            "dummy": True,
            "protocolVersion": 1,
            "session": STATE.session_id,
            "sequence": STATE.sequence,
            "lastFrame": STATE.last_protocol_frame,
            "cmdAgeMs": STATE.command_age_ms(),
            "failsafe": STATE.failsafe,
            "failsafeCount": STATE.failsafe_count,
            "fix": True,
            "lat": 41.0256,
            "lng": 28.8895,
            "sats": 12,
            "mpuP": pitch,
            "mpuR": roll,
            "mpuY": STATE.body_yaw + math.sin(t * 0.1) * 2.0,
            "z": int(STATE.gait_z),
            "vx": STATE.joy_x,
            "vy": STATE.joy_y,
            "vr": STATE.joy_r,
            "gP": 0.0,
            "gR": 0.0,
            "gaitPhase": STATE.gait_phase,
            "homeSet": True,
            "tofOnline": not STATE.tof_fault,
            "tofMm": -1 if STATE.tof_fault else int(850 + math.sin(t * 0.5) * 120),
            "batR": STATE.battery_right_v,
            "batL": STATE.battery_left_v,
            "batRCell": STATE.battery_right_v / 3.0,
            "batLCell": STATE.battery_left_v / 3.0,
            "batRAdcMv": int(STATE.battery_right_v / 4.2 * 1000),
            "batLAdcMv": int(STATE.battery_left_v / 4.2 * 1000),
            "batRCal": 1.0,
            "batLCal": 1.0,
            "mode": STATE.mode,
        }
        return JSONResponse(payload, headers={"Cache-Control": "no-store"})


@app.get("/legStatus")
async def leg_status() -> dict[str, Any]:
    async with STATE.lock:
        return {
            "dummy": True,
            "legs": [
                {
                    "leg": leg,
                    "name": LEG_NAMES[leg],
                    "pca": "0x40" if leg < 3 else "0x41",
                    "channels": list(LEG_PINS[leg]),
                    "dir": list(LEG_DIR[leg]),
                    "enabled": list(STATE.pwm_enabled[leg]),
                    "angle": [round(v, 1) for v in STATE.current_angles[leg]],
                    "target": [round(v, 1) for v in STATE.target_angles[leg]],
                    "home": [round(v, 1) for v in STATE.home_angles[leg]],
                }
                for leg in range(6)
            ],
        }


def _selection(leg: int, joint: int) -> tuple[range, range]:
    if leg < -1 or leg > 5 or joint < -1 or joint > 2:
        raise HTTPException(status_code=400, detail="bad leg/joint")
    return (range(6) if leg < 0 else range(leg, leg + 1), range(3) if joint < 0 else range(joint, joint + 1))


@app.get("/legOff")
async def leg_off(leg: int = -1, j: int = -1) -> dict[str, Any]:
    legs, joints = _selection(leg, j)
    async with STATE.lock:
        for i in legs:
            for k in joints:
                STATE.pwm_enabled[i][k] = False
    return {"ok": True, "pwm": "off", "persistent": True}


@app.get("/legOn")
async def leg_on(leg: int = -1, j: int = -1) -> dict[str, Any]:
    legs, joints = _selection(leg, j)
    async with STATE.lock:
        for i in legs:
            for k in joints:
                STATE.pwm_enabled[i][k] = True
    return {"ok": True, "pwm": "on"}


@app.get("/legServo")
@app.get("/manual")
@app.get("/homePreview")
async def leg_servo(leg: int, j: int, angle: float | None = None, v: float | None = None) -> dict[str, Any]:
    value = angle if angle is not None else v
    if leg not in range(6) or j not in range(3) or value is None or not 5 <= value <= 175:
        raise HTTPException(status_code=422, detail="bad leg/joint/angle")
    async with STATE.lock:
        STATE.target_angles[leg][j] = float(value)
        STATE.current_angles[leg][j] = float(value)
        STATE.mode = "MANUEL TEST"
    return {"ok": True, "leg": leg, "joint": j, "angle": value}


@app.get("/legMountPose")
async def leg_mount_pose(leg: int = -1) -> dict[str, Any]:
    legs, _ = _selection(leg, -1)
    async with STATE.lock:
        for i in legs:
            STATE.target_angles[i] = [90.0, 5.0, 5.0] if i < 3 else [90.0, 175.0, 175.0]
        STATE.mode = "MANUEL TEST"
    return {"ok": True, "pose": "mount"}


@app.get("/legHomePose")
async def leg_home_pose(leg: int = -1) -> dict[str, Any]:
    legs, _ = _selection(leg, -1)
    async with STATE.lock:
        for i in legs:
            STATE.target_angles[i] = list(STATE.home_angles[i])
        STATE.mode = "MANUEL TEST"
    return {"ok": True, "pose": "home"}


@app.get("/getCalib")
async def get_calib(leg: int) -> dict[str, int]:
    if leg not in range(6):
        raise HTTPException(status_code=400, detail="bad leg")
    return dict(STATE.calibration[leg])


@app.get("/saveCalib")
async def save_calib(request: Request, leg: int) -> dict[str, Any]:
    if leg not in range(6):
        raise HTTPException(status_code=400, detail="bad leg")
    values = dict(STATE.calibration[leg])
    for key in values:
        if key in request.query_params:
            values[key] = int(request.query_params[key])
    for joint in "cft":
        lo, hi, off = values[f"{joint}Min"], values[f"{joint}Max"], values[f"{joint}Off"]
        if not (400 <= lo < hi <= 2600 and -90 <= off <= 90):
            raise HTTPException(status_code=422, detail=f"invalid {joint} calibration")
    STATE.calibration[leg] = values
    return {"ok": True}


@app.get("/getHome")
async def get_home(leg: int) -> dict[str, float]:
    if leg not in range(6):
        raise HTTPException(status_code=400, detail="bad leg")
    return dict(zip(("c", "f", "t"), STATE.home_angles[leg]))


@app.get("/saveHome")
async def save_home(leg: int, c: float, f: float, t: float) -> dict[str, Any]:
    if leg not in range(6) or not all(5 <= v <= 175 for v in (c, f, t)):
        raise HTTPException(status_code=422, detail="invalid home")
    STATE.home_angles[leg] = [c, f, t]
    return {"ok": True}


@app.get("/goHome")
async def go_home() -> dict[str, Any]:
    async with STATE.lock:
        STATE.target_angles = [list(x) for x in STATE.home_angles]
        STATE.mode = "KALKIS"
    return {"ok": True}


@app.get("/armStatus")
@app.get("/arm/status")
async def arm_status() -> dict[str, Any]:
    return {"ok": True, "enabled": STATE.arm_enabled, "angle": STATE.arm_angles, "target": STATE.arm_angles}


@app.get("/armServo")
@app.get("/arm/servo")
async def arm_servo(id: str | None = None, joint: str | None = None, angle: float = 90, **_: Any) -> dict[str, Any]:
    name = id or joint
    if name not in STATE.arm_angles or not 5 <= angle <= 175:
        raise HTTPException(status_code=422, detail="invalid arm command")
    STATE.arm_angles[name] = angle
    STATE.arm_enabled = True
    return await arm_status()


@app.get("/armPose")
@app.get("/arm/pose")
async def arm_pose(base: float | None = None, joint1: float | None = None, joint2: float | None = None, gripper: float | None = None) -> dict[str, Any]:
    for name, value in (("base", base), ("joint1", joint1), ("joint2", joint2), ("gripper", gripper)):
        if value is not None:
            if not 5 <= value <= 175:
                raise HTTPException(status_code=422, detail=f"invalid {name}")
            STATE.arm_angles[name] = value
    STATE.arm_enabled = True
    return await arm_status()


@app.get("/armOff")
@app.get("/arm/off")
async def arm_off() -> dict[str, Any]:
    STATE.arm_enabled = False
    return await arm_status()


@app.get("/audio")
async def audio(**_: Any) -> dict[str, Any]:
    return {"ok": True, "dummy": True}


@app.get("/batteryCal")
async def battery_cal(side: str, actual: float) -> dict[str, Any]:
    if side.upper() not in {"R", "L"} or not 1 <= actual <= 20:
        raise HTTPException(status_code=422, detail="invalid calibration")
    if side.upper() == "R":
        STATE.battery_right_v = actual
    else:
        STATE.battery_left_v = actual
    return {"ok": True, "side": side.upper(), "actual": actual, "factor": 1.0}


@app.get("/sim/status")
async def sim_status() -> dict[str, Any]:
    return {
        "drop_commands": STATE.drop_commands,
        "telemetry_offline": STATE.telemetry_offline,
        "imu_fault": STATE.imu_fault,
        "tof_fault": STATE.tof_fault,
        "failsafe": STATE.failsafe,
        "failsafe_count": STATE.failsafe_count,
        "command_age_ms": STATE.command_age_ms(),
        "last_protocol_frame": STATE.last_protocol_frame,
    }


@app.get("/sim/fault")
async def sim_fault(kind: str, state: int = 1) -> dict[str, Any]:
    mapping = {
        "link_drop": "drop_commands",
        "telemetry_offline": "telemetry_offline",
        "imu": "imu_fault",
        "tof": "tof_fault",
    }
    attr = mapping.get(kind)
    if attr is None:
        raise HTTPException(status_code=400, detail=f"unknown fault; choose {sorted(mapping)}")
    setattr(STATE, attr, bool(state))
    return await sim_status()


@app.get("/sim/reset")
async def sim_reset() -> dict[str, Any]:
    async with STATE.lock:
        STATE.drop_commands = STATE.telemetry_offline = STATE.imu_fault = STATE.tof_fault = False
        STATE.failsafe = False
        STATE.joy_x = STATE.joy_y = STATE.joy_r = 0.0
        STATE.mode = "IDLE"
        STATE.last_command_mono = time.monotonic()
        STATE.pwm_enabled = [[True] * 3 for _ in range(6)]
    return await sim_status()
