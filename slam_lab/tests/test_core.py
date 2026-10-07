import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import yaml

from core.frames.axis_map import AxisMap
from core.frames.geometry import Pose2, Quaternion, polar_xy, wrap_angle
from core.mapping.gate import mapping_pause_reason
from core.mapping.occupancy import OccupancyGrid
from core.scan_matching.correlative import CorrelativeMatcher
from core.timing.clock import EspClock, ordered_ns
from drivers.imu_serial.protocol import parse_imu_line
from drivers.imu_serial.port import open_esp32_serial
from drivers.lidar.scan import LidarPoint, LidarScan, normalize_scan
from replay.session import merged_events
from slam_runtime.session import SessionWriter


class FrameTests(unittest.TestCase):
    def test_polar_axes(self):
        self.assertAlmostEqual(polar_xy(0, 2)[0], 2)
        self.assertAlmostEqual(polar_xy(math.pi/2, 2)[1], 2)
        self.assertAlmostEqual(polar_xy(-math.pi/2, 2)[1], -2)

    def test_yaw_transform(self):
        x, y = Pose2(1, 2, math.pi/2).transform_xy(1, 0)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 3)
        self.assertAlmostEqual(wrap_angle(3*math.pi), -math.pi)

    def test_quaternion_and_handedness(self):
        q = Quaternion(2, 0, 0, 0).normalized()
        self.assertAlmostEqual(q.w, 1)
        with self.assertRaises(ValueError):
            Quaternion(0, 0, 0, 0).normalized()
        self.assertEqual(AxisMap(["+y", "-x", "+z"]).apply((1, 2, 3)), (2, -1, 3))
        with self.assertRaises(ValueError):
            AxisMap(["+x", "-y", "+z"])

    def test_timestamp_rollover(self):
        clock = EspClock()
        clock.unwrap_us(0xFFFFFFFE)
        self.assertEqual(clock.unwrap_us(3), 0x100000003)
        self.assertFalse(ordered_ns(11, 11))
        with self.assertRaises(ValueError):
            EspClock().unwrap_us(-1)


class SensorTests(unittest.TestCase):
    def test_esp32_reset_lines_inactive_before_open(self):
        stream = Mock()

        def opening():
            self.assertFalse(stream.dtr)
            self.assertFalse(stream.rts)
            self.assertEqual(stream.port, "/dev/test-esp32")

        stream.open.side_effect = opening
        with patch("drivers.imu_serial.port.serial.Serial", return_value=stream) as constructor:
            self.assertIs(open_esp32_serial("/dev/test-esp32", 115200), stream)
        constructor.assert_called_once_with(port=None, baudrate=115200, timeout=0.5)
        stream.open.assert_called_once()

    def test_imu_parser(self):
        line = "IMU,100,0,0,9.80665,0,0,0,nan,nan,nan,23.5,0,0,8192,0,0,0,0,0,0"
        sample = parse_imu_line(line, 1234)
        self.assertAlmostEqual(sample.accel_norm_m_s2, 9.80665)
        self.assertEqual(sample.host_receive_ns, 1234)
        self.assertEqual(len(sample.raw_counts), 9)
        with self.assertRaises(ValueError):
            parse_imu_line("IMU,1,2", 1234)

    def test_lidar_filter_and_map_ray(self):
        payload = {"timestamp_start_ns": 1, "timestamp_end_ns": 2, "sequence": 0,
                   "points": [[0, 1, 10], [0, 0, 10], [0, 2, 0]]}
        scan = normalize_scan(payload, min_range_m=.1, max_range_m=3, min_quality=1)
        self.assertEqual(len(scan.points), 1)
        grid = OccupancyGrid(resolution_m=.5)
        grid.integrate(scan, Pose2())
        self.assertLess(grid.probability((1, 0)), .5)
        self.assertGreater(grid.probability((2, 0)), .5)

    def test_tilt_gate(self):
        kwargs = dict(max_roll_deg=5, max_pitch_deg=5,
                      imu_axes_verified=True, lidar_angle_verified=True)
        self.assertIsNone(mapping_pause_reason(0, 0, **kwargs))
        self.assertEqual(mapping_pause_reason(math.radians(6), 0, **kwargs), "EXCESSIVE TILT")
        self.assertEqual(mapping_pause_reason(0, 0, **{**kwargs, "imu_axes_verified": False}),
                         "UNVERIFIED SENSOR FRAMES")

    def test_matcher_stationary_does_not_drift_on_score_plateau(self):
        points = tuple(
            LidarPoint(angle, 2 + .3*math.sin(3*angle) + .2*math.cos(7*angle), 30)
            for angle in (2*math.pi*i/120 for i in range(120))
        )
        scan = LidarScan(1, 2, 0, points)
        result = CorrelativeMatcher(downsample=2).match(scan, scan, 0.0)
        self.assertTrue(result.accepted)
        self.assertEqual(result.delta, Pose2())

    def test_matcher_accepts_clear_translation(self):
        world = [
            (r*math.cos(angle), r*math.sin(angle))
            for angle in (2*math.pi*i/120 for i in range(120))
            for r in [2 + .3*math.sin(3*angle) + .2*math.cos(7*angle)]
        ]

        def scan_from(points):
            return LidarScan(1, 2, 0, tuple(
                LidarPoint(math.atan2(y, x) % (2*math.pi), math.hypot(x, y), 30)
                for x, y in points
            ))

        previous = scan_from(world)
        current = scan_from([(x-.2, y-.1) for x, y in world])
        result = CorrelativeMatcher(downsample=2).match(previous, current, 0.0)
        self.assertTrue(result.accepted)
        self.assertGreaterEqual(result.gain_over_static, .04)
        self.assertGreaterEqual(result.delta.x_m, .1)

    def test_replay_parser_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "imu_raw.csv").write_text(
                "host_receive_ns,esp_time_us,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s,mx_uT,my_uT,mz_uT,temperature_c,raw_counts_json,source_frame\n"
                '20,2,0,0,9.8,0,0,0,1,2,3,20,"[]",imu_sensor_raw\n', encoding="utf-8")
            (root / "lidar_scans.jsonl").write_text(json.dumps({
                "timestamp_start_ns": 10, "timestamp_end_ns": 30,
                "sequence": 1, "points": [[0, 1, 2]],
            }) + "\n", encoding="utf-8")
            events = list(merged_events(root))
            self.assertEqual([kind for _, kind, _ in events], ["imu", "lidar"])

    def test_session_sensor_identity_and_rates(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            hardware = {"lidar": {"expected_serial": "ABC"}, "imu": {"port": "/dev/esp", "sample_rate_hz": 100}}
            writer = SessionWriter(root, hardware, {"convention": "test"}, {})
            writer.update_sensor_metadata("lidar", {"serial": "ABC", "firmware": "1.29", "health_code": 0})
            writer.update_sensor_metadata("esp32", {"port": "/dev/esp", "who_am_i": "0x70"})
            writer.update_observed_rates({"imu": 94.0, "lidar": 7.4, "mapping": 2.5})
            writer.close()
            metadata = yaml.safe_load((writer.directory / "metadata.yaml").read_text(encoding="utf-8"))
            self.assertEqual(metadata["lidar_actual_serial"], "ABC")
            self.assertEqual(metadata["lidar_firmware"], "1.29")
            self.assertEqual(metadata["esp32_serial_info"]["who_am_i"], "0x70")
            self.assertEqual(metadata["sensor_rates_observed_hz"]["mapping"], 2.5)


if __name__ == "__main__":
    unittest.main()
