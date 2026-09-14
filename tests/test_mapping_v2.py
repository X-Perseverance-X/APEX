import math
import pathlib
import time
import unittest

import numpy as np

from mapping_v2 import ImuSample, LidarScan, MappingEngine
from mapping_v2.occupancy import GridConfig, OccupancyGrid, rle_encode
from mapping_v2.planner import plan_route
from mapping_v2.pointcloud import VoxelCloud, project_lidar_scan, rotation_matrix_rpy


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

    def test_duplicate_scan_sequence_is_not_integrated_twice(self):
        grid = OccupancyGrid(self.config)
        scan = LidarScan(time.monotonic_ns(), 7, [(0.0, 1000.0)] * 3)
        self.assertTrue(grid.update_lidar(scan))
        before = grid.log_odds.copy()
        self.assertFalse(grid.update_lidar(scan))
        np.testing.assert_array_equal(before, grid.log_odds)

    def test_local_costmap_forgets_obstacles_that_are_no_longer_seen(self):
        config = GridConfig(width=41, height=41, resolution_m=0.1, max_range_m=1.8,
                            temporal_decay=0.5, forget_epsilon=0.08)
        grid = OccupancyGrid(config)
        hit = [(0.0, 1000.0)] * 3
        grid.update_lidar(LidarScan(time.monotonic_ns(), 1, hit))
        gx, gy = grid.world_to_grid(1.0, 0.0)
        self.assertEqual(int(grid.states()[gy, gx]), 100)
        for sequence in range(2, 9):
            grid.update_lidar(LidarScan(time.monotonic_ns(), sequence, []))
        self.assertEqual(int(grid.states()[gy, gx]), -1)

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


if __name__ == "__main__":
    unittest.main()
