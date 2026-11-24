"""APEX META-1 radio/SIL protocol v1.

Every encoded frame is exactly 32 bytes so the same bytes can travel over an
nRF24L01+ payload, a local simulation socket, or a recorded test fixture.
Values are explicitly little-endian; no Python/C++ struct layout is shared.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct

MAGIC = b"AX"
PROTOCOL_VERSION = 1
FRAME_SIZE = 32


class ProtocolError(ValueError):
    """Raised when a frame is malformed, stale, or outside protocol limits."""


class MessageType(IntEnum):
    MOTION_COMMAND = 0x10
    TELEMETRY_FAST = 0x80
    TELEMETRY_LEG = 0x81


class ControlMode(IntEnum):
    IDLE = 0
    MANUAL_SERVO = 1
    GAIT = 2
    AUTO = 3
    CALIBRATION = 4


class CommandFlags(IntFlag):
    NONE = 0
    REQUEST_ARM = 1 << 0
    CLEAR_FAILSAFE = 1 << 1
    GIMBAL_ENABLE = 1 << 2
    OUTPUT_DISABLE = 1 << 3


class LinkState(IntEnum):
    BOOT = 0
    DISARMED = 1
    ARMING = 2
    ACTIVE = 3
    FAILSAFE_HOLD = 4
    OUTPUT_DISABLED = 5
    FAULT = 6


class FaultFlags(IntFlag):
    NONE = 0
    LINK_TIMEOUT = 1 << 0
    BAD_PACKET = 1 << 1
    LOW_BATTERY = 1 << 2
    SENSOR_FAULT = 1 << 3
    CONFIG_FAULT = 1 << 4
    OUTPUT_DISABLED = 1 << 5


_MOTION_NO_CRC = struct.Struct("<2sBBHHBBhhhhHHHhhh")
_MOTION = struct.Struct("<2sBBHHBBhhhhHHHhhhH")
_FAST_NO_CRC = struct.Struct("<2sBBHHBBHHhhhHHHBBBB")
_FAST = struct.Struct("<2sBBHHBBHHhhhHHHBBBBH")
_LEG_NO_CRC = struct.Struct("<2sBBHHBBHHHHHHHHHH")
_LEG = struct.Struct("<2sBBHHBBHHHHHHHHHHH")

assert _MOTION.size == FRAME_SIZE
assert _FAST.size == FRAME_SIZE
assert _LEG.size == FRAME_SIZE


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""

    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _range(name: str, value: int, low: int, high: int) -> int:
    value = int(value)
    if not low <= value <= high:
        raise ProtocolError(f"{name}={value} outside [{low}, {high}]")
    return value


def _check_frame(frame: bytes, expected_type: MessageType) -> None:
    if len(frame) != FRAME_SIZE:
        raise ProtocolError(f"frame length {len(frame)} != {FRAME_SIZE}")
    if frame[:2] != MAGIC:
        raise ProtocolError("bad magic")
    if frame[2] != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {frame[2]}")
    if frame[3] != expected_type:
        raise ProtocolError(f"unexpected message type 0x{frame[3]:02x}")
    expected_crc = int.from_bytes(frame[-2:], "little")
    actual_crc = crc16_ccitt(frame[:-2])
    if actual_crc != expected_crc:
        raise ProtocolError(f"CRC mismatch: got 0x{expected_crc:04x}, expected 0x{actual_crc:04x}")


def is_newer_sequence(candidate: int, previous: int) -> bool:
    """Return True for a newer uint16 sequence, including wrap-around."""

    delta = (int(candidate) - int(previous)) & 0xFFFF
    return 0 < delta < 0x8000


@dataclass(frozen=True, slots=True)
class MotionCommand:
    sequence: int
    session_id: int
    flags: CommandFlags = CommandFlags.NONE
    mode: ControlMode = ControlMode.IDLE
    joy_x_milli: int = 0
    joy_y_milli: int = 0
    joy_r_milli: int = 0
    gait_z_mm: int = -100
    step_length_mm: int = 100
    step_lift_mm: int = 50
    cycle_ms: int = 1000
    body_pitch_cdeg: int = 0
    body_roll_cdeg: int = 0
    body_yaw_cdeg: int = 0

    def encode(self) -> bytes:
        values = (
            MAGIC,
            PROTOCOL_VERSION,
            MessageType.MOTION_COMMAND,
            _range("sequence", self.sequence, 0, 0xFFFF),
            _range("session_id", self.session_id, 0, 0xFFFF),
            _range("flags", int(self.flags), 0, 0xFF),
            _range("mode", int(self.mode), 0, 0xFF),
            _range("joy_x_milli", self.joy_x_milli, -1000, 1000),
            _range("joy_y_milli", self.joy_y_milli, -1000, 1000),
            _range("joy_r_milli", self.joy_r_milli, -1000, 1000),
            _range("gait_z_mm", self.gait_z_mm, -300, 0),
            _range("step_length_mm", self.step_length_mm, 0, 250),
            _range("step_lift_mm", self.step_lift_mm, 0, 180),
            _range("cycle_ms", self.cycle_ms, 250, 5000),
            _range("body_pitch_cdeg", self.body_pitch_cdeg, -3000, 3000),
            _range("body_roll_cdeg", self.body_roll_cdeg, -3000, 3000),
            _range("body_yaw_cdeg", self.body_yaw_cdeg, -4500, 4500),
        )
        raw = _MOTION_NO_CRC.pack(*values)
        return raw + struct.pack("<H", crc16_ccitt(raw))

    @classmethod
    def decode(cls, frame: bytes) -> "MotionCommand":
        _check_frame(frame, MessageType.MOTION_COMMAND)
        (
            _magic,
            _version,
            _kind,
            sequence,
            session_id,
            flags,
            mode,
            joy_x,
            joy_y,
            joy_r,
            gait_z,
            step_length,
            step_lift,
            cycle_ms,
            pitch,
            roll,
            yaw,
            _crc,
        ) = _MOTION.unpack(frame)
        try:
            mode_value = ControlMode(mode)
        except ValueError as exc:
            raise ProtocolError(f"unknown control mode {mode}") from exc
        return cls(
            sequence=sequence,
            session_id=session_id,
            flags=CommandFlags(flags),
            mode=mode_value,
            joy_x_milli=joy_x,
            joy_y_milli=joy_y,
            joy_r_milli=joy_r,
            gait_z_mm=gait_z,
            step_length_mm=step_length,
            step_lift_mm=step_lift,
            cycle_ms=cycle_ms,
            body_pitch_cdeg=pitch,
            body_roll_cdeg=roll,
            body_yaw_cdeg=yaw,
        )


@dataclass(frozen=True, slots=True)
class TelemetryFast:
    sequence: int
    session_id: int
    state: LinkState
    faults: FaultFlags
    acknowledged_sequence: int
    link_age_ms: int
    pitch_cdeg: int
    roll_cdeg: int
    yaw_cdeg: int
    tof_mm: int
    battery_right_mv: int
    battery_left_mv: int
    gait_phase_u8: int
    mode: ControlMode
    retransmit_count: int = 0
    packet_loss_pct: int = 0

    def encode(self) -> bytes:
        values = (
            MAGIC,
            PROTOCOL_VERSION,
            MessageType.TELEMETRY_FAST,
            _range("sequence", self.sequence, 0, 0xFFFF),
            _range("session_id", self.session_id, 0, 0xFFFF),
            _range("state", int(self.state), 0, 0xFF),
            _range("faults", int(self.faults), 0, 0xFF),
            _range("acknowledged_sequence", self.acknowledged_sequence, 0, 0xFFFF),
            _range("link_age_ms", self.link_age_ms, 0, 0xFFFF),
            _range("pitch_cdeg", self.pitch_cdeg, -32768, 32767),
            _range("roll_cdeg", self.roll_cdeg, -32768, 32767),
            _range("yaw_cdeg", self.yaw_cdeg, -32768, 32767),
            _range("tof_mm", self.tof_mm, 0, 0xFFFF),
            _range("battery_right_mv", self.battery_right_mv, 0, 0xFFFF),
            _range("battery_left_mv", self.battery_left_mv, 0, 0xFFFF),
            _range("gait_phase_u8", self.gait_phase_u8, 0, 0xFF),
            _range("mode", int(self.mode), 0, 0xFF),
            _range("retransmit_count", self.retransmit_count, 0, 0xFF),
            _range("packet_loss_pct", self.packet_loss_pct, 0, 100),
        )
        raw = _FAST_NO_CRC.pack(*values)
        return raw + struct.pack("<H", crc16_ccitt(raw))

    @classmethod
    def decode(cls, frame: bytes) -> "TelemetryFast":
        _check_frame(frame, MessageType.TELEMETRY_FAST)
        values = _FAST.unpack(frame)
        try:
            state = LinkState(values[5])
            mode = ControlMode(values[16])
        except ValueError as exc:
            raise ProtocolError("unknown telemetry state or mode") from exc
        return cls(
            sequence=values[3],
            session_id=values[4],
            state=state,
            faults=FaultFlags(values[6]),
            acknowledged_sequence=values[7],
            link_age_ms=values[8],
            pitch_cdeg=values[9],
            roll_cdeg=values[10],
            yaw_cdeg=values[11],
            tof_mm=values[12],
            battery_right_mv=values[13],
            battery_left_mv=values[14],
            gait_phase_u8=values[15],
            mode=mode,
            retransmit_count=values[17],
            packet_loss_pct=values[18],
        )


@dataclass(frozen=True, slots=True)
class TelemetryLeg:
    sequence: int
    session_id: int
    leg_index: int
    status_flags: int
    acknowledged_sequence: int
    current_cdeg: tuple[int, int, int]
    target_cdeg: tuple[int, int, int]
    home_cdeg: tuple[int, int, int]

    def encode(self) -> bytes:
        angles = (*self.current_cdeg, *self.target_cdeg, *self.home_cdeg)
        if len(angles) != 9:
            raise ProtocolError("leg telemetry requires 3 current, target, and home angles")
        values = (
            MAGIC,
            PROTOCOL_VERSION,
            MessageType.TELEMETRY_LEG,
            _range("sequence", self.sequence, 0, 0xFFFF),
            _range("session_id", self.session_id, 0, 0xFFFF),
            _range("leg_index", self.leg_index, 0, 5),
            _range("status_flags", self.status_flags, 0, 0xFF),
            _range("acknowledged_sequence", self.acknowledged_sequence, 0, 0xFFFF),
            *(_range("angle_cdeg", value, 0, 18000) for value in angles),
        )
        raw = _LEG_NO_CRC.pack(*values)
        return raw + struct.pack("<H", crc16_ccitt(raw))

    @classmethod
    def decode(cls, frame: bytes) -> "TelemetryLeg":
        _check_frame(frame, MessageType.TELEMETRY_LEG)
        values = _LEG.unpack(frame)
        return cls(
            sequence=values[3],
            session_id=values[4],
            leg_index=values[5],
            status_flags=values[6],
            acknowledged_sequence=values[7],
            current_cdeg=(values[8], values[9], values[10]),
            target_cdeg=(values[11], values[12], values[13]),
            home_cdeg=(values[14], values[15], values[16]),
        )
