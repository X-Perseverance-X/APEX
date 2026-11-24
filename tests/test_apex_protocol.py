from __future__ import annotations

import unittest

from shared.apex_protocol import (
    CommandFlags,
    ControlMode,
    FaultFlags,
    LinkState,
    MotionCommand,
    ProtocolError,
    TelemetryFast,
    TelemetryLeg,
    crc16_ccitt,
    is_newer_sequence,
)


class MotionCommandTests(unittest.TestCase):
    def test_round_trip_and_fixed_payload_size(self) -> None:
        command = MotionCommand(
            sequence=65535,
            session_id=0x1234,
            flags=CommandFlags.REQUEST_ARM | CommandFlags.GIMBAL_ENABLE,
            mode=ControlMode.GAIT,
            joy_x_milli=-1000,
            joy_y_milli=750,
            joy_r_milli=-125,
            gait_z_mm=-145,
            step_length_mm=90,
            step_lift_mm=45,
            cycle_ms=850,
            body_pitch_cdeg=-123,
            body_roll_cdeg=456,
            body_yaw_cdeg=-789,
        )
        frame = command.encode()
        self.assertEqual(len(frame), 32)
        self.assertEqual(MotionCommand.decode(frame), command)
        self.assertEqual(int.from_bytes(frame[-2:], "little"), crc16_ccitt(frame[:-2]))

    def test_corruption_is_rejected(self) -> None:
        frame = bytearray(MotionCommand(sequence=1, session_id=7).encode())
        frame[12] ^= 0x80
        with self.assertRaisesRegex(ProtocolError, "CRC mismatch"):
            MotionCommand.decode(bytes(frame))

    def test_cross_language_golden_vector(self) -> None:
        frame = MotionCommand(
            sequence=1,
            session_id=0x1234,
            flags=CommandFlags.REQUEST_ARM,
            mode=ControlMode.GAIT,
            joy_y_milli=500,
        ).encode()
        self.assertEqual(
            frame.hex(),
            "415801100100341201020000f40100009cff64003200e803000000000000ff6e",
        )

    def test_out_of_range_axis_is_rejected(self) -> None:
        with self.assertRaisesRegex(ProtocolError, "joy_x_milli"):
            MotionCommand(sequence=1, session_id=1, joy_x_milli=1001).encode()


class TelemetryTests(unittest.TestCase):
    def test_fast_round_trip(self) -> None:
        state = TelemetryFast(
            sequence=9,
            session_id=10,
            state=LinkState.FAILSAFE_HOLD,
            faults=FaultFlags.LINK_TIMEOUT,
            acknowledged_sequence=8,
            link_age_ms=201,
            pitch_cdeg=-120,
            roll_cdeg=35,
            yaw_cdeg=17999,
            tof_mm=65535,
            battery_right_mv=12340,
            battery_left_mv=12290,
            gait_phase_u8=127,
            mode=ControlMode.IDLE,
            retransmit_count=3,
            packet_loss_pct=2,
        )
        self.assertEqual(TelemetryFast.decode(state.encode()), state)

    def test_leg_page_round_trip(self) -> None:
        page = TelemetryLeg(
            sequence=100,
            session_id=3,
            leg_index=5,
            status_flags=1,
            acknowledged_sequence=99,
            current_cdeg=(9000, 4900, 5000),
            target_cdeg=(9100, 5000, 5100),
            home_cdeg=(9000, 4900, 5000),
        )
        self.assertEqual(TelemetryLeg.decode(page.encode()), page)


class SequenceTests(unittest.TestCase):
    def test_sequence_wrap(self) -> None:
        self.assertTrue(is_newer_sequence(0, 65535))
        self.assertTrue(is_newer_sequence(101, 100))
        self.assertFalse(is_newer_sequence(100, 100))
        self.assertFalse(is_newer_sequence(99, 100))


if __name__ == "__main__":
    unittest.main()
