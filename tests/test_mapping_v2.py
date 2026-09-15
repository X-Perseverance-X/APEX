import math
import pathlib
import tempfile
import time
import unittest
from unittest.mock import AsyncMock, patch

import httpx
import numpy as np

from mapping_v2 import GraphConfig, ImuSample, LidarOutlierFilter, LidarScan, MapArchive, MappingEngine, Pose2D, PoseGraph, TofSample
from mapping_v2.occupancy import GridConfig, OccupancyGrid, rle_encode
from mapping_v2.planner import plan_route
from mapping_v2.pointcloud import VoxelCloud, project_lidar_scan, rotation_matrix_rpy
from mapping_v2.scan_matcher import CorrelativeScanMatcher
import apex_server as server


ROOT = pathlib.Path(__file__).resolve().parents[1]


class MappingV2Tests(unittest.TestCase):
    def setUp(self):
        self.config = GridConfig(width=81, height=81, resolution_m=0.10, max_range_m=3.8)

    def test_lidar_scan_marks_ray_free_and_endpoint_occupied(self):
        grid = OccupancyGrid(self.config)
        scan = LidarScan(time.monotonic_ns(), 1, [(0.0, 2000.0)] * 3)
        self.assertTrue(grid.update_lidar(scan))
        states = grid.states()
        sx, sy = grid.world_to_grid(1.0, 0.0)
        ex, ey = grid.world_to_grid(2.0, 0.0)
        self.assertEqual(int(states[sy, sx]), 0)
        self.assertEqual(int(states[ey, ex]), 100)

    def test_lidar_filter_rejects_isolated_mirror_ray(self):
        filter_ = LidarOutlierFilter()
        points = [(float(angle), 1500.0) for angle in range(0, 21)]
        points[10] = (10.0, 5800.0)
        filtered = filter_.filter(LidarScan(time.monotonic_ns(), 1, points))
        self.assertNotIn((10.0, 5800.0), filtered.points)
        self.assertGreater(filter_.status()["rejected_points"], 0)

    def test_lidar_filter_requires_persistence_for_large_temporal_jump(self):
        filter_ = LidarOutlierFilter(persistence=3)
        now = time.monotonic_ns()
        base = [(float(angle), 1200.0) for angle in range(0, 40)]
        filter_.filter(LidarScan(now, 1, base))
        changed = [(float(angle), 3000.0) for angle in range(0, 40)]
        first = filter_.filter(LidarScan(now + 1, 2, changed))
        second = filter_.filter(LidarScan(now + 2, 3, changed))
        third = filter_.filter(LidarScan(now + 3, 4, changed))
        self.assertEqual(len(first.points), 0)
        self.assertEqual(len(second.points), 0)
        self.assertEqual(len(third.points), len(changed))

    def test_lidar_filter_cold_start_rejects_sparse_far_return(self):
        filter_ = LidarOutlierFilter(persistence=3)
        scan = LidarScan(time.monotonic_ns(), 1, [(0.0, 1000.0), (90.0, 5900.0)])
        filtered = filter_.filter(scan)
        self.assertIn((0.0, 1000.0), filtered.points)
        self.assertNotIn((90.0, 5900.0), filtered.points)

    def test_duplicate_scan_sequence_is_not_integrated_twice(self):
        grid = OccupancyGrid(self.config)
        scan = LidarScan(time.monotonic_ns(), 7, [(0.0, 1000.0)] * 3)
        self.assertTrue(grid.update_lidar(scan))
        before = grid.log_odds.copy()
        self.assertFalse(grid.update_lidar(scan))
        np.testing.assert_array_equal(before, grid.log_odds)

    def test_local_costmap_forgets_obstacles_that_are_no_longer_seen(self):
        config = GridConfig(width=41, height=41, resolution_m=0.1, max_range_m=1.8,
                            stale_grace_s=0.1, stale_half_life_s=0.2, forget_epsilon=0.08)
        grid = OccupancyGrid(config)
        start_ns = time.monotonic_ns()
        hit = [(0.0, 1000.0)] * 3
        grid.update_lidar(LidarScan(start_ns, 1, hit))
        gx, gy = grid.world_to_grid(1.0, 0.0)
        self.assertEqual(int(grid.states()[gy, gx]), 100)
        for sequence in range(2, 9):
            grid.update_lidar(LidarScan(start_ns + sequence * 200_000_000, sequence, []))
        self.assertEqual(int(grid.states()[gy, gx]), -1)

    def test_farther_return_clears_a_stale_near_endpoint_immediately(self):
        grid = OccupancyGrid(self.config)
        now = time.monotonic_ns()
        grid.update_lidar(LidarScan(now, 1, [(0.0, 1000.0)]))
        near = grid.world_to_grid(1.0, 0.0)
        self.assertEqual(int(grid.states()[near[1], near[0]]), 100)
        grid.update_lidar(LidarScan(now + 200_000_000, 2, [(0.0, 2000.0)]))
        self.assertNotEqual(int(grid.states()[near[1], near[0]]), 100)

    def test_each_cell_is_updated_only_once_per_scan(self):
        grid = OccupancyGrid(self.config)
        scan = LidarScan(time.monotonic_ns(), 1, [(0.0, 1000.0)] * 8)
        grid.update_lidar(scan)
        gx, gy = grid.world_to_grid(1.0, 0.0)
        self.assertAlmostEqual(float(grid.log_odds[gy, gx]), grid.config.occupied_delta, places=5)

    @staticmethod
    def _landmark_scan(pose_x, pose_y, pose_yaw, landmarks):
        points = []
        cosine, sine = math.cos(pose_yaw), math.sin(pose_yaw)
        for wx, wy in landmarks:
            dx, dy = wx - pose_x, wy - pose_y
            lx = dx * cosine + dy * sine
            ly = -dx * sine + dy * cosine
            distance = math.hypot(lx, ly)
            points.append((math.degrees(math.atan2(ly, lx)) % 360.0, distance * 1000.0))
        return points

    def test_correlative_matcher_recovers_rotation_without_scan_smearing(self):
        config = GridConfig(width=161, height=161, resolution_m=0.05, max_range_m=3.8)
        grid = OccupancyGrid(config)
        landmarks = []
        for x in np.linspace(-2.5, 2.5, 30):
            landmarks.extend(((float(x), -2.0), (float(x), 2.0)))
        for y in np.linspace(-1.8, 1.8, 24):
            landmarks.extend(((-2.5, float(y)), (2.5, float(y))))
        first = LidarScan(time.monotonic_ns(), 1, self._landmark_scan(0.0, 0.0, 0.0, landmarks))
        grid.update_lidar(first, Pose2D())
        matcher = CorrelativeScanMatcher()
        rotated = LidarScan(time.monotonic_ns(), 2, self._landmark_scan(0.0, 0.0, math.radians(7.0), landmarks))
        result = matcher.match(rotated, grid, Pose2D(), imu_yaw_delta_rad=math.radians(7.0))
        self.assertTrue(result.accepted, result)
        self.assertAlmostEqual(math.degrees(result.pose.yaw_rad), 7.0, delta=2.0)

    def test_engine_marks_pose_trusted_only_after_repeated_good_matches(self):
        config = GridConfig(width=161, height=161, resolution_m=0.05, max_range_m=3.8)
        engine = MappingEngine(config)
        landmarks = []
        for x in np.linspace(-2.5, 2.5, 30):
            landmarks.extend(((float(x), -2.0), (float(x), 2.0)))
        for y in np.linspace(-1.8, 1.8, 24):
            landmarks.extend(((-2.5, float(y)), (2.5, float(y))))
        start_ns = time.monotonic_ns()
        for sequence in range(1, 5):
            yaw_deg = float(sequence - 1) * 1.5
            engine.ingest_imu(ImuSample(start_ns + sequence, 0.0, 0.0, yaw_deg, True))
            engine.ingest_lidar(LidarScan(
                start_ns + sequence * 200_000_000,
                sequence,
                self._landmark_scan(0.0, 0.0, math.radians(yaw_deg), landmarks),
            ))
        snapshot = engine.snapshot(False)
        self.assertTrue(snapshot["pose"]["trusted"])
        self.assertEqual(snapshot["map_mode"], "LOCAL_SLAM")
        self.assertTrue(snapshot["fusion_readiness"]["local_slam"])
        self.assertFalse(snapshot["fusion_readiness"]["global_slam"])
        self.assertEqual(snapshot["local_slam"]["state"], "TRACKING")
        self.assertGreaterEqual(snapshot["local_slam"]["accepted_matches"], 3)
        self.assertEqual(snapshot["local_slam"]["imu_yaw_sign"], 1)
        self.assertGreaterEqual(snapshot["local_slam"]["imu_sign_votes"], 3)

    def test_correlative_matcher_recovers_small_translation_and_yaw(self):
        config = GridConfig(width=161, height=161, resolution_m=0.05, max_range_m=3.8)
        grid = OccupancyGrid(config)
        landmarks = [(2.4, y) for y in np.linspace(-1.6, 1.6, 35)]
        landmarks += [(x, -2.1) for x in np.linspace(-2.2, 2.2, 35)]
        grid.update_lidar(
            LidarScan(time.monotonic_ns(), 1, self._landmark_scan(0.0, 0.0, 0.0, landmarks)),
            Pose2D(),
        )
        moved = LidarScan(
            time.monotonic_ns(), 2,
            self._landmark_scan(0.06, -0.03, math.radians(4.0), landmarks),
        )
        result = CorrelativeScanMatcher().match(
            moved, grid, Pose2D(), imu_yaw_delta_rad=math.radians(4.0),
        )
        self.assertTrue(result.accepted, result)
        self.assertAlmostEqual(result.pose.x_m, 0.06, delta=0.04)
        self.assertAlmostEqual(result.pose.y_m, -0.03, delta=0.04)
        self.assertAlmostEqual(math.degrees(result.pose.yaw_rad), 4.0, delta=2.0)

    def test_pose_graph_loop_constraint_reduces_terminal_drift(self):
        graph = PoseGraph(GraphConfig(relaxation_iterations=120))
        scan = LidarScan(time.monotonic_ns(), 1, [(float(a), 1800.0) for a in range(0, 360, 3)])
        drifted = [
            Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0),
            Pose2D(1.0, 1.0, math.pi / 2), Pose2D(0.0, 1.0, math.pi),
            Pose2D(0.38, 0.12, math.radians(8.0)),
        ]
        for sequence, pose in enumerate(drifted, 1):
            graph.maybe_add_keyframe(pose, LidarScan(scan.timestamp_ns + sequence, sequence, scan.points), force=True)
        before = math.hypot(graph.keyframes[-1].pose[0], graph.keyframes[-1].pose[1])
        graph.add_loop_constraint(0, 4, np.zeros(3, dtype=float), weight=0.75)
        corrected = graph.optimize()
        after = math.hypot(float(corrected[-1][0]), float(corrected[-1][1]))
        self.assertTrue(graph.optimized)
        self.assertLess(after, before * 0.45)
        np.testing.assert_allclose(corrected[0], [0.0, 0.0, 0.0], atol=1e-9)

    def test_engine_global_finalize_replays_every_accepted_scan(self):
        engine = MappingEngine(self.config)
        engine.start_mapping_session("Tam Tarama")
        now = time.monotonic_ns()
        base_points = [(float(a), 1800.0) for a in range(0, 360, 4)]
        poses = [
            Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0),
            Pose2D(1.0, 1.0, math.pi / 2), Pose2D(0.0, 1.0, math.pi),
            Pose2D(0.30, 0.08, math.radians(5.0)),
        ]
        sequence = 1
        for index, pose in enumerate(poses):
            scan = LidarScan(now + sequence, sequence, base_points)
            engine.grid.update_lidar(scan, pose)
            observation = engine.pose_graph.record_scan(pose, scan)
            engine.pose_graph.maybe_add_keyframe(pose, scan, force=True, observation_index=observation)
            sequence += 1
            if index < len(poses) - 1:
                # This non-keyframe-only return must survive the global rebuild.
                intermediate = LidarScan(now + sequence, sequence, [(33.0, 730.0)] * 4)
                mid_pose = Pose2D((pose.x_m + poses[index + 1].x_m) / 2, (pose.y_m + poses[index + 1].y_m) / 2, pose.yaw_rad)
                engine.grid.update_lidar(intermediate, mid_pose)
                engine.pose_graph.record_scan(mid_pose, intermediate)
                sequence += 1
        expected_scans = len(engine.pose_graph.observations)
        engine.pose_graph.add_loop_constraint(0, len(poses) - 1, np.zeros(3), weight=0.75)
        status = engine.finish_mapping_session()
        self.assertTrue(status["global_slam"])
        self.assertEqual(engine.grid.scan_count, expected_scans)
        self.assertGreater(int(engine.grid.observed.sum()), 0)
        self.assertEqual(engine.pose.source, "global_pose_graph")

    def test_icp_validates_revisit_and_rejects_unrelated_scan(self):
        config = GraphConfig(min_loop_separation=2, min_loop_path_length_m=1.0, loop_search_radius_m=1.0, loop_min_inlier_ratio=0.45)
        graph = PoseGraph(config)
        asymmetric = [(float(a), 900.0 + 4.0 * a) for a in range(0, 270, 3)]
        first = LidarScan(time.monotonic_ns(), 1, asymmetric)
        graph.maybe_add_keyframe(Pose2D(), first, force=True)
        graph.maybe_add_keyframe(Pose2D(0.7, 0.0, 0.0), LidarScan(first.timestamp_ns + 1, 2, asymmetric), force=True)
        graph.maybe_add_keyframe(Pose2D(0.18, -0.06, math.radians(4.0)), LidarScan(first.timestamp_ns + 2, 3, asymmetric), force=True)
        self.assertGreaterEqual(graph.detect_loop_closures(), 1)
        self.assertGreaterEqual(graph.loop_count, 1)

        unrelated = PoseGraph(config)
        unrelated.maybe_add_keyframe(Pose2D(), first, force=True)
        unrelated.maybe_add_keyframe(Pose2D(0.7, 0.0, 0.0), LidarScan(first.timestamp_ns + 1, 2, asymmetric), force=True)
        circle = [(float(a), 2500.0) for a in range(0, 360, 3)]
        unrelated.maybe_add_keyframe(Pose2D(0.1, 0.0, 0.0), LidarScan(first.timestamp_ns + 2, 3, circle), force=True)
        self.assertEqual(unrelated.detect_loop_closures(), 0)

    def test_stationary_repeated_scans_do_not_claim_global_loop(self):
        graph = PoseGraph(GraphConfig(min_loop_separation=2, min_loop_path_length_m=1.0))
        points = [(float(a), 1200.0 + a) for a in range(0, 300, 3)]
        now = time.monotonic_ns()
        for sequence in range(1, 8):
            graph.maybe_add_keyframe(
                Pose2D(), LidarScan(now + sequence * 3_000_000_000, sequence, points), force=True,
            )
        self.assertEqual(graph.detect_loop_closures(), 0)
        self.assertEqual(graph.loop_count, 0)

    def test_observation_overflow_preserves_local_map_instead_of_partial_rebuild(self):
        graph = PoseGraph(GraphConfig(max_observations=2, min_loop_separation=1))
        points = [(float(a), 1200.0) for a in range(0, 360, 3)]
        now = time.monotonic_ns()
        for sequence in range(1, 4):
            scan = LidarScan(now + sequence, sequence, points)
            observation = graph.record_scan(Pose2D(float(sequence), 0.0, 0.0), scan)
            if observation is not None:
                graph.maybe_add_keyframe(Pose2D(float(sequence), 0.0, 0.0), scan, force=True, observation_index=observation)
        self.assertTrue(graph.observation_overflow)
        graph.add_loop_constraint(0, 1, np.zeros(3), weight=0.75)
        graph.finalize()
        self.assertFalse(graph.optimized)

    def test_loading_legacy_map_clears_stale_pose_graph_state(self):
        engine = MappingEngine(self.config)
        scan = LidarScan(time.monotonic_ns(), 1, [(float(a), 1000.0) for a in range(0, 360, 3)])
        for sequence, x_m in enumerate((0.0, 1.0), 1):
            frame = LidarScan(scan.timestamp_ns + sequence, sequence, scan.points)
            engine.pose_graph.maybe_add_keyframe(Pose2D(x_m, 0.0, 0.0), frame, force=True)
        engine.pose_graph.add_loop_constraint(0, 1, np.zeros(3))
        engine.pose_graph.optimize()
        self.assertTrue(engine.pose_graph.optimized)
        payload = {
            "metadata": {
                "resolution_m": self.config.resolution_m,
                "origin_x_m": engine.grid.origin_x_m,
                "origin_y_m": engine.grid.origin_y_m,
                "scan_count": 5, "map_id": "legacy", "display_name": "Legacy", "saved_at_utc": "old",
            },
            "log_odds": np.zeros((self.config.height, self.config.width), dtype=np.float32),
            "observed": np.zeros((self.config.height, self.config.width), dtype=np.bool_),
            "hit_count": np.zeros((self.config.height, self.config.width), dtype=np.uint16),
            "pose": np.zeros(3, dtype=np.float64),
        }
        engine.load_archive(payload)
        self.assertFalse(engine.pose_graph.optimized)
        self.assertFalse(engine.snapshot(False)["fusion_readiness"]["global_slam"])

    def test_weak_match_is_not_inserted_into_mature_submap(self):
        config = GridConfig(width=161, height=161, resolution_m=0.05, max_range_m=3.8)
        grid = OccupancyGrid(config)
        landmarks = [(2.0, y) for y in np.linspace(-1.5, 1.5, 60)]
        initial = LidarScan(time.monotonic_ns(), 1, self._landmark_scan(0.0, 0.0, 0.0, landmarks))
        grid.update_lidar(initial, Pose2D())
        before = grid.log_odds.copy()
        unrelated = LidarScan(time.monotonic_ns(), 2, [(float(a), 500.0) for a in range(0, 360, 5)])
        result = CorrelativeScanMatcher().match(unrelated, grid, Pose2D())
        self.assertFalse(result.accepted)
        np.testing.assert_array_equal(before, grid.log_odds)

    def test_tof_free_ray_does_not_erase_lidar_occupied_cell(self):
        grid = OccupancyGrid(self.config)
        now = time.monotonic_ns()
        grid.update_lidar(LidarScan(now, 1, [(0.0, 1000.0)]), Pose2D())
        gx, gy = grid.world_to_grid(1.0, 0.0)
        before = float(grid.log_odds[gy, gx])
        grid.update_tof(TofSample(now + 1, 2000.0, True), Pose2D())
        self.assertEqual(float(grid.log_odds[gy, gx]), before)

    def test_wall_becomes_confirmed_then_requires_three_clear_rays(self):
        grid = OccupancyGrid(self.config)
        start_ns = time.monotonic_ns()
        for sequence in range(1, 5):
            grid.update_lidar(LidarScan(start_ns + sequence, sequence, [(0.0, 1000.0)]))
        gx, gy = grid.world_to_grid(1.0, 0.0)
        self.assertTrue(bool(grid.confirmed_mask()[gy, gx]))
        for sequence in (5, 6):
            grid.update_lidar(LidarScan(start_ns + sequence, sequence, [(0.0, 2000.0)]))
            self.assertTrue(bool(grid.confirmed_mask()[gy, gx]))
        grid.update_lidar(LidarScan(start_ns + 7, 7, [(0.0, 2000.0)]))
        self.assertFalse(bool(grid.confirmed_mask()[gy, gx]))

    def test_mapping_session_is_atomically_saved_loaded_and_frozen(self):
        engine = MappingEngine(self.config)
        status = engine.start_mapping_session("Salon Testi")
        self.assertEqual(status["state"], "MAPPING")
        start_ns = time.monotonic_ns()
        points = [(float(angle), 1500.0) for angle in range(0, 360, 5)]
        for sequence in range(1, 5):
            engine.grid.update_lidar(LidarScan(start_ns + sequence, sequence, points), Pose2D())
        finished = engine.finish_mapping_session()
        self.assertEqual(finished["state"], "FROZEN")
        before = engine.grid.log_odds.copy()

        with tempfile.TemporaryDirectory() as directory:
            archive = MapArchive(directory)
            metadata = archive.save(engine.export_archive(), "Salon Testi")
            engine.mark_archive_saved(metadata)
            self.assertEqual(len(archive.list_maps()), 1)
            loaded_payload = archive.load(metadata["map_id"])
            restored = MappingEngine(self.config)
            loaded_status = restored.load_archive(loaded_payload)
            self.assertEqual(loaded_status["state"], "FROZEN")
            np.testing.assert_array_equal(restored.grid.log_odds, before)
            np.testing.assert_array_equal(restored.grid.hit_count, engine.grid.hit_count)

        # A frozen map still localizes against scans but never mutates its
        # archived occupancy evidence.
        engine.ingest_lidar(LidarScan(start_ns + 1_000_000, 99, points))
        np.testing.assert_array_equal(engine.grid.log_odds, before)

    def test_map_archive_rejects_path_traversal(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                MapArchive(directory).load("../outside")

    def test_astar_routes_around_inflated_wall_gap(self):
        grid = OccupancyGrid(self.config)
        # Mark the map observed/free, then add a wall with a wide upper gap.
        grid.observed[:, :] = True
        grid.log_odds[:, :] = -2.0
        wall_x, _ = grid.world_to_grid(1.0, 0.0)
        for gy in range(0, 50):
            grid.log_odds[gy, wall_x] = 4.0
        plan = plan_route(grid, (0.0, 0.0), (2.0, 0.0), footprint_radius_m=0.15)
        self.assertTrue(plan.ok, plan.message)
        self.assertGreater(plan.length_m, 2.0)
        self.assertGreaterEqual(len(plan.path_world), 3)

    def test_goal_inside_inflated_obstacle_is_rejected(self):
        grid = OccupancyGrid(self.config)
        grid.observed[:, :] = True
        grid.log_odds[:, :] = -2.0
        gx, gy = grid.world_to_grid(1.0, 0.0)
        grid.log_odds[gy, gx] = 4.0
        result = plan_route(grid, (0.0, 0.0), (1.0, 0.0), footprint_radius_m=0.2)
        self.assertFalse(result.ok)
        self.assertEqual(result.status, "GOAL_BLOCKED")

    def test_self_returns_inside_start_footprint_do_not_trap_planner(self):
        grid = OccupancyGrid(self.config)
        grid.observed[:, :] = True
        grid.log_odds[:, :] = -2.0
        # Simulate platform/chassis returns inside the space already occupied
        # by the robot. These must not inflate into a sealed start cell.
        for x_m, y_m in ((0.2, 0.0), (0.1, -0.2), (-0.2, -0.1)):
            gx, gy = grid.world_to_grid(x_m, y_m)
            grid.log_odds[gy, gx] = 4.0

        result = plan_route(grid, (0.0, 0.0), (2.0, 0.0), footprint_radius_m=0.38)

        self.assertTrue(result.ok, result.message)
        self.assertEqual(result.status, "PLANNED")
        self.assertGreater(result.expanded_nodes, 1)

    def test_rle_round_trip(self):
        values = np.array([[-1, -1, 0], [0, 100, 100]], dtype=np.int8)
        encoded = rle_encode(values)
        decoded = []
        for value, count in encoded:
            decoded.extend([value] * count)
        self.assertEqual(decoded, values.reshape(-1).tolist())

    def test_engine_guards_execution_and_manual_approval(self):
        engine = MappingEngine(self.config)
        engine.grid.observed[:, :] = True
        engine.grid.log_odds[:, :] = -2.0
        status = engine.set_goal(1.5, 0.0)
        self.assertTrue(status["route_ok"])
        self.assertEqual(status["decision"], "WAITING_OPERATOR_APPROVAL")
        approved = engine.approve_preview()
        self.assertEqual(approved["decision"], "APPROVED_FOR_EXECUTION")
        self.assertFalse(approved["execution_locked"])
        engine.ingest_lidar(LidarScan(time.monotonic_ns(), 11, []))
        self.assertEqual(engine.snapshot(False)["decision"], "APPROVED_FOR_EXECUTION")
        self.assertFalse(hasattr(engine, "execute"))

    def test_mapping_package_has_no_actuator_endpoint_literals(self):
        source = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "mapping_v2").glob("*.py")
        )
        for forbidden in ("/joy", "/manual", "/bodyIk", "/goHome", "/armPose", "/legServo"):
            self.assertNotIn(forbidden, source)

    def test_se3_projection_never_invents_height_at_zero_tilt(self):
        scan = LidarScan(time.monotonic_ns(), 1, [(0.0, 1000.0), (90.0, 1000.0)])
        xyz = project_lidar_scan(scan, 0.0, 0.0, 0.0)
        np.testing.assert_allclose(xyz[:, 2], [0.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(xyz[0], [1.0, 0.0, 0.0], atol=1e-9)

    def test_standard_positive_pitch_rotates_forward_ray_to_negative_z(self):
        scan = LidarScan(time.monotonic_ns(), 1, [(0.0, 1000.0)])
        xyz = project_lidar_scan(scan, 0.0, math.pi / 2, 0.0)
        np.testing.assert_allclose(xyz[0], [0.0, 0.0, -1.0], atol=1e-7)

    def test_voxel_cloud_is_bounded(self):
        cloud = VoxelCloud(voxel_size_m=0.01, max_points=3)
        cloud.integrate(np.array([[i, 0.0, 0.0] for i in range(8)], dtype=float))
        self.assertEqual(len(cloud), 3)

    def test_fusion_readiness_reports_real_blockers(self):
        engine = MappingEngine(self.config)
        readiness = engine.snapshot(False)["fusion_readiness"]
        self.assertFalse(readiness["global_slam"])
        self.assertFalse(readiness["sparse_3d"])
        self.assertIn("SENSÖR DIŞ KALİBRASYONU PROVISIONAL", readiness["blockers"])
        self.assertIn("DOĞRULANMIŞ GLOBAL LOOP CLOSURE YOK", readiness["blockers"])

    def test_archived_global_slam_status_round_trips(self):
        engine = MappingEngine(self.config)
        payload = {
            "metadata": {
                "resolution_m": self.config.resolution_m,
                "origin_x_m": engine.grid.origin_x_m,
                "origin_y_m": engine.grid.origin_y_m,
                "scan_count": 20,
                "map_id": "global-test",
                "display_name": "Global Test",
                "saved_at_utc": "2026-01-01T00:00:00+00:00",
                "global_slam": True,
                "pose_graph": {"keyframes": 18, "loop_closures": 1, "optimized": True, "state": "OPTIMIZED"},
            },
            "log_odds": np.zeros((self.config.height, self.config.width), dtype=np.float32),
            "observed": np.zeros((self.config.height, self.config.width), dtype=np.bool_),
            "hit_count": np.zeros((self.config.height, self.config.width), dtype=np.uint16),
            "pose": np.zeros(3, dtype=np.float64),
        }
        status = engine.load_archive(payload)
        snapshot = engine.snapshot(False)
        self.assertTrue(status["global_slam"])
        self.assertEqual(snapshot["map_mode"], "GLOBAL_SLAM")
        self.assertTrue(snapshot["fusion_readiness"]["global_slam"])
        self.assertEqual(snapshot["pose_graph"]["loop_closures"], 1)

    def test_mapping_revision_changes_only_for_publishable_state(self):
        engine = MappingEngine(self.config)
        first = engine.revision()
        scan = LidarScan(time.monotonic_ns(), 21, [(0.0, 1000.0)] * 3)
        self.assertTrue(engine.ingest_lidar(scan))
        after_scan = engine.revision()
        self.assertGreater(after_scan, first)
        self.assertFalse(engine.ingest_lidar(scan))
        self.assertEqual(engine.revision(), after_scan)
        engine.clear_goal()
        self.assertGreater(engine.revision(), after_scan)

    def test_snapshot_exposes_raw_imu_attitude_without_claiming_world_pose(self):
        engine = MappingEngine(self.config)
        engine.ingest_imu(ImuSample(time.monotonic_ns(), 12.5, -4.25, 7.0, True))
        snapshot = engine.snapshot(False)
        self.assertEqual(snapshot["sensors"]["imu"]["roll_deg"], 12.5)
        self.assertEqual(snapshot["sensors"]["imu"]["pitch_deg"], -4.25)
        self.assertEqual(snapshot["sensors"]["imu"]["yaw_deg"], 7.0)
        self.assertFalse(snapshot["pose"]["trusted"])

    def test_snapshot_exposes_real_imu_projected_sparse_cloud(self):
        engine = MappingEngine(self.config)
        now = time.monotonic_ns()
        engine.ingest_imu(ImuSample(now, 0.0, -15.0, 0.0, True))
        engine.ingest_lidar(LidarScan(now + 1, 1, [(float(a), 1000.0) for a in range(0, 90, 2)]))
        cloud = engine.snapshot(False)["sparse_cloud"]
        self.assertTrue(cloud["provisional"])
        self.assertGreater(len(cloud["points"]), 0)
        self.assertTrue(any(abs(point[2]) > 0.05 for point in cloud["points"]))


class MappingApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.engine = MappingEngine()
        self.engine_patch = patch.object(server, "_mapping_engine", self.engine)
        self.archive_patch = patch.object(server, "_map_archive", MapArchive(self.temporary.name))
        self.engine_patch.start()
        self.archive_patch.start()
        server._navigation_platform_status["active"] = False
        self.client = httpx.AsyncClient(
            transport=httpx.ASGITransport(app=server.app), base_url="http://test",
        )

    async def asyncTearDown(self):
        await self.client.aclose()
        self.archive_patch.stop()
        self.engine_patch.stop()
        self.temporary.cleanup()

    async def test_start_failure_keeps_existing_map_intact(self):
        self.engine.grid.update_lidar(LidarScan(time.monotonic_ns(), 1, [(0.0, 1000.0)]))
        with patch.object(
            server, "_set_lidar_motor",
            AsyncMock(return_value={"applied": False, "error": "test motor failure"}),
        ):
            response = await self.client.post("/api/navigation/mapping/start", json={"name": "Test"})
        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.engine.grid.scan_count, 1)

    async def test_start_finish_list_and_load_round_trip(self):
        with patch.object(
            server, "_set_lidar_motor",
            AsyncMock(return_value={"applied": True, "enabled": True, "online": True}),
        ):
            started = await self.client.post("/api/navigation/mapping/start", json={"name": "Salon"})
        self.assertEqual(started.status_code, 200)
        self.assertEqual(started.json()["mapping_session"]["state"], "MAPPING")

        now = time.monotonic_ns()
        points = [(float(angle), 1500.0) for angle in range(0, 360, 5)]
        for sequence in range(1, 5):
            self.engine.grid.update_lidar(LidarScan(now + sequence, sequence, points), Pose2D())

        finished = await self.client.post("/api/navigation/mapping/finish")
        self.assertEqual(finished.status_code, 200)
        map_id = finished.json()["mapping_session"]["archive_id"]
        self.assertTrue(map_id)
        listed = await self.client.get("/api/navigation/maps")
        self.assertEqual([item["map_id"] for item in listed.json()["maps"]], [map_id])
        loaded = await self.client.post("/api/navigation/mapping/load", json={"map_id": map_id})
        self.assertEqual(loaded.status_code, 200)
        self.assertEqual(loaded.json()["mapping_session"]["state"], "FROZEN")


if __name__ == "__main__":
    unittest.main()
