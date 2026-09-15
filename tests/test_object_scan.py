import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from object_scan.core import (
    ColoredVoxelCloud,
    estimate_rigid_transform,
    icp_register,
    lidar_polar_to_camera,
    transform_points,
)


class ObjectScanGeometryTests(unittest.TestCase):
    def test_kabsch_recovers_known_rigid_transform(self):
        source = np.asarray([
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0], [1.0, 1.0, 0.5],
        ])
        angle = math.radians(17.0)
        expected = np.eye(4)
        expected[:3, :3] = ((math.cos(angle), -math.sin(angle), 0.0),
                            (math.sin(angle), math.cos(angle), 0.0),
                            (0.0, 0.0, 1.0))
        expected[:3, 3] = (0.08, -0.04, 0.03)
        target = transform_points(source, expected)
        actual = estimate_rigid_transform(source, target)
        np.testing.assert_allclose(actual, expected, atol=1e-8)

    def test_icp_refines_small_pose_error(self):
        rng = np.random.default_rng(7)
        target = rng.uniform((-0.4, -0.3, -0.2), (0.5, 0.4, 0.25), size=(180, 3))
        expected = np.eye(4)
        expected[:3, 3] = (0.025, -0.018, 0.012)
        source = transform_points(target, np.linalg.inv(expected))
        result = icp_register(source, target, max_correspondence_m=0.10)
        self.assertTrue(result.accepted, result.reason)
        self.assertGreater(result.fitness, 0.95)
        self.assertLess(result.rmse_m, 0.005)
        np.testing.assert_allclose(result.transform, expected, atol=0.005)

    def test_lidar_conversion_keeps_only_front_metric_sector(self):
        camera_from_lidar = np.eye(4)
        points = [(0.0, 1000.0), (45.0, 800.0), (90.0, 900.0), (180.0, 700.0), (0.0, 100.0)]
        xyz = lidar_polar_to_camera(points, camera_from_lidar, forward_half_angle_deg=60.0)
        self.assertEqual(len(xyz), 2)
        np.testing.assert_allclose(xyz[0], [1.0, 0.0, 0.0], atol=1e-8)

    def test_voxel_cloud_averages_duplicate_observations_and_exports_rgb_ply(self):
        cloud = ColoredVoxelCloud(voxel_size_m=0.05)
        cloud.integrate(np.asarray([[0.01, 0.01, 0.01]]), np.asarray([[255, 0, 0]]))
        cloud.integrate(np.asarray([[0.02, 0.01, 0.01]]), np.asarray([[0, 0, 255]]))
        xyz, rgb, counts = cloud.arrays()
        self.assertEqual(len(cloud), 1)
        self.assertEqual(int(counts[0]), 2)
        np.testing.assert_allclose(xyz[0], [0.015, 0.01, 0.01], atol=1e-8)
        np.testing.assert_allclose(rgb[0], [127, 0, 127], atol=1)
        with tempfile.TemporaryDirectory() as directory:
            path = cloud.export_ply(Path(directory) / "scan.ply")
            text = path.read_text(encoding="ascii")
            self.assertIn("element vertex 1", text)
            self.assertIn("property uchar red", text)


if __name__ == "__main__":
    unittest.main()
