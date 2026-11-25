import ast
import pathlib
import threading
import time
import unittest
import cv2
import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]


class VisionRuntimeContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = (ROOT / "vision" / "detection_hud.py").read_text(encoding="utf-8")

    def test_red_cube_pipeline_is_removed(self):
        for removed in ("detect_red_cube", "draw_cube_overlay", "/cube/status", "RED XYZ CUBE"):
            self.assertNotIn(removed, self.source)

    def test_selection_control_is_independent_from_spine_telemetry(self):
        self.assertIn("APEX_VISION_CONTROL_URL", self.source)
        self.assertIn("def vision_control_poller", self.source)
        self.assertIn("threading.Thread(target=vision_control_poller", self.source)
        telemetry_block = self.source.split("def telemetry_poller", 1)[1].split("def vision_control_poller", 1)[0]
        self.assertNotIn("telemetry['sel']", telemetry_block)
        self.assertNotIn("telemetry['ai_track']", telemetry_block)

    def test_jpeg_is_encoded_once_and_shared_by_all_clients(self):
        render_block = self.source.split("def hud_render_worker", 1)[1].split(
            "class HUDStreamHandler", 1
        )[0]
        stream_block = self.source.split('if path == "/stream":', 1)[1].split(
            'elif path == "/tracking/status":', 1
        )[0]
        snapshot_block = self.source.split('elif path == "/snapshot":', 1)[1].split(
            'elif path == "/health":', 1
        )[0]
        self.assertEqual(render_block.count("cv2.imencode"), 1)
        self.assertNotIn("cv2.imencode", stream_block)
        self.assertNotIn("cv2.imencode", snapshot_block)
        self.assertIn("latest_jpeg[0]", stream_block)
        self.assertIn("latest_jpeg[0]", snapshot_block)

    def test_tracking_has_tolerant_click_and_verified_status(self):
        self.assertIn("pad = 0.04", self.source)
        self.assertIn("'state': 'locked'", self.source)
        self.assertIn('path == "/tracking/status"', self.source)
        self.assertIn("bbox_iou", self.source)

    def test_camera_hud_uses_real_power_tof_and_has_no_idle_radar_sweep(self):
        for field in ("data.get('batR')", "data.get('batL')", "data.get('tofMm')", "data.get('tofOnline'"):
            self.assertIn(field, self.source)
        self.assertIn("front_tof_readout(frame,cx,cy)", self.source)
        self.assertIn("BATTERY RIGHT", self.source)
        self.assertIn("BATTERY LEFT", self.source)
        self.assertNotIn("SCANNING...", self.source)
        self.assertNotIn("cv2.fillPoly(ov,[pts]", self.source)

    def test_ai_tracking_moves_only_camera_gimbal(self):
        self.assertIn("APEX_GIMBAL_URL", self.source)
        self.assertIn("def send_gimbal_target", self.source)
        self.assertIn("send_gimbal_target(command_pan, command_tilt)", self.source)
        self.assertIn("send_gimbal_mode('center')", self.source)
        self.assertNotIn("APEX_JOY_URL", self.source)
        self.assertNotIn("send_joy(", self.source)

    def test_new_camera_mount_is_not_rotated_or_mirrored_by_default(self):
        self.assertIn('CAMERA_HFLIP = os.environ.get("APEX_CAMERA_HFLIP", "0") == "1"', self.source)
        self.assertIn('CAMERA_VFLIP = os.environ.get("APEX_CAMERA_VFLIP", "0") == "1"', self.source)
        self.assertIn("Transform(hflip=CAMERA_HFLIP, vflip=CAMERA_VFLIP)", self.source)

    def test_adaptive_approach_is_close_smooth_and_tof_guarded(self):
        tree = ast.parse(self.source)
        wanted = {"compute_forward", "slew_towards"}
        functions = [
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name in wanted
        ]
        assignments = [
            node for node in tree.body
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id in {
                "TARGET_FOLLOW_DIST_M", "FORWARD_FULL_SPEED_M", "MAX_FORWARD_SPEED",
                "FORWARD_DEADBAND_M", "TOF_HARD_STOP_MM", "TOF_SLOW_END_MM",
            } for target in node.targets)
        ]
        namespace = {}
        exec(compile(ast.Module(body=assignments + functions, type_ignores=[]), "tracking-unit", "exec"), namespace)
        forward = namespace["compute_forward"]
        self.assertEqual(forward(0.90, 0.0), 0.0)
        self.assertGreater(forward(3.2, 0.0), forward(1.4, 0.0))
        self.assertGreater(forward(1.4, 0.0), 0.0)
        self.assertLess(forward(1.4, 0.65), forward(1.4, 0.0))
        self.assertEqual(forward(3.2, 0.0, 400), 0.0)
        self.assertLess(forward(3.2, 0.0, 700), forward(3.2, 0.0, 1200))
        slew = namespace["slew_towards"]
        self.assertAlmostEqual(slew(0.0, 1.0, 0.5, 1.0, 0.1), 0.05)
        self.assertAlmostEqual(slew(0.5, 0.0, 0.5, 1.0, 0.1), 0.4)
        self.assertIn("TRACK_FRAME_MAX_AGE_S", self.source)
        self.assertIn("CONTROL_MAX_AGE_S", self.source)
        self.assertIn("frame_fresh", self.source)
        self.assertIn("control_fresh", self.source)
        self.assertIn("PERSON_DISTANCE_K_M/bh_ratio", self.source)

    def test_click_selection_locks_clears_and_rejects_empty_space(self):
        tree = ast.parse(self.source)
        functions = [
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name in {"process_target_selection", "bbox_iou", "visual_frame", "initialize_visual_target", "follow_visual_target"}
        ]
        namespace = {
            "threading": threading,
            "time": time,
            "selected_track_id": None,
            "selected_bbox": None,
            "last_processed_sel_ts": 0.0,
            "selection_lock": threading.Lock(),
            "tracking_lock": threading.Lock(),
            "selection_status": {},
            "tracking_state": {},
            "visual_target": {"patch": None, "center": None, "last_seen": 0.0},
            "cv2": cv2,
            "np": np,
        }
        exec(compile(ast.Module(body=functions, type_ignores=[]), "selection-unit", "exec"), namespace)
        person = {"label": "person", "confidence": 0.92, "track_id": 7, "bbox": (0.30, 0.20, 0.60, 0.85)}
        selected, processed = namespace["process_target_selection"](
            [person], {"ts": 1.0, "x": 0.27, "y": 0.45, "clear": False}
        )
        self.assertTrue(processed)
        self.assertEqual(selected, 7)
        self.assertEqual(namespace["selection_status"]["state"], "locked")

        namespace["process_target_selection"]([], {"ts": 2.0, "x": 0.0, "y": 0.0, "clear": True})
        self.assertEqual(namespace["selection_status"]["state"], "cleared")
        self.assertIsNone(namespace["selected_bbox"])

        namespace["process_target_selection"]([person], {"ts": 3.0, "x": 0.95, "y": 0.05, "clear": False})
        self.assertEqual(namespace["selection_status"]["state"], "missed")

        bottle = {"label": "bottle", "confidence": 0.81, "track_id": None,
                  "bbox": (0.12, 0.25, 0.30, 0.70)}
        selected, processed = namespace["process_target_selection"](
            [bottle], {"ts": 4.0, "x": 0.20, "y": 0.50, "clear": False}
        )
        self.assertTrue(processed)
        self.assertIsNone(selected)
        self.assertEqual(namespace["selection_status"]["state"], "locked")
        self.assertEqual(namespace["selected_bbox"], bottle["bbox"])

        # Detector classes without persistent IDs (such as a cup) must keep
        # the selected image patch, otherwise the next detector frame loses it.
        rng = np.random.default_rng(9)
        image = rng.integers(0, 256, (180, 320, 3), dtype=np.uint8)
        namespace["process_target_selection"](
            [bottle], {"ts": 4.5, "x": 0.20, "y": 0.50, "clear": False}, image
        )
        self.assertIsNotNone(namespace["visual_target"]["patch"])
        moved = np.zeros_like(image)
        moved[:, 10:] = image[:, :-10]
        self.assertIsNotNone(namespace["follow_visual_target"](moved))

        # An object outside the detector's label set still locks via its
        # visible image patch and follows a small translation.
        namespace["process_target_selection"]([], {"ts": 5.0, "x": 0.40, "y": 0.50, "clear": False}, image)
        self.assertEqual(namespace["selection_status"]["state"], "locked")
        moved = np.zeros_like(image)
        moved[:, 10:] = image[:, :-10]
        match = namespace["follow_visual_target"](moved)
        self.assertIsNotNone(match)
        self.assertGreater(match[1], 0.90)
        self.assertAlmostEqual(sum(match[0][::2]) / 2, 0.40 + 10 / 320, places=2)
        # A single shaken frame can jump beyond the cheap local search.
        moved_far = np.zeros_like(image)
        moved_far[:, 115:] = image[:, :-115]
        recovered = namespace["follow_visual_target"](moved_far)
        self.assertIsNotNone(recovered)
        self.assertGreater(recovered[1], 0.90)


if __name__ == "__main__":
    unittest.main()
