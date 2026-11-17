import math
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from urg_ground_geometry import (GroundConfig, scan_to_points, quaternion_rotation,
                                 transform_points, fit_local_ground, classify_points)


class UrgGroundTests(unittest.TestCase):
    def test_project_scan(self):
        idx, pts = scan_to_points([1.0, float('inf'), 2.0], 0, math.pi / 2, 0.05, 4.0)
        self.assertEqual(idx.tolist(), [0, 2])
        np.testing.assert_allclose(pts[1], [-2, 0, 0], atol=1e-10)

    def test_vertical_transform(self):
        # The central ray points down for (roll=0, pitch=pi/2, yaw=pi/2).
        cy, sy = math.cos(math.pi / 4), math.sin(math.pi / 4)
        cp, sp = math.cos(math.pi / 4), math.sin(math.pi / 4)
        q = (-sy * sp, sp * cy, cp * sy, cp * cy)  # qz*qy
        np.testing.assert_allclose(transform_points(np.array([[1, 0, 0]]),
            (0.4, 0.3, 0.35), q)[0], [0.4, 0.3, -0.65], atol=1e-9)

    def test_ramp_and_protrusion(self):
        cfg = GroundConfig()
        x = np.linspace(0, 1.0, 101)
        z = 0.1 * x + 0.005
        pts = np.column_stack((x, np.full_like(x, 0.30), z))
        extra = np.array([[0.8, 0.3, 0.8 * 0.1 + 0.105],
                          [0.805, 0.3, 0.805 * 0.1 + 0.105]])
        all_points = np.vstack((pts, extra))
        ids = np.arange(len(all_points))
        ground = fit_local_ground(all_points, cfg)
        self.assertIsNotNone(ground)
        self.assertAlmostEqual(ground[0], 0.1, delta=0.01)
        obstacles, floor = classify_points(ids, all_points, ground, cfg)
        self.assertEqual(len(obstacles), 2)
        self.assertGreater(len(floor), 70)

    def test_step_must_not_be_fitted_as_floor(self):
        cfg = GroundConfig()
        x = np.linspace(0.0, 1.0, 101)
        z = np.where(x < 0.5, 0.0, 0.10)
        pts = np.column_stack((x, np.full_like(x, 0.3), z))
        ground = fit_local_ground(pts, cfg)
        self.assertIsNotNone(ground)
        self.assertAlmostEqual(ground[1], 0.0, delta=0.02)
        obs, _ = classify_points(np.arange(len(pts)), pts, ground, cfg)
        self.assertGreater(len(obs), 40)

    def test_no_floor_returns_none(self):
        pts = np.array([[0.2, 0.3, 0.4], [0.3, 0.3, 0.5]])
        self.assertIsNone(fit_local_ground(pts, GroundConfig()))

    def test_single_noise_point_is_rejected(self):
        cfg = GroundConfig()
        x = np.linspace(0, 0.8, 81)
        pts = np.column_stack((x, np.zeros_like(x), np.zeros_like(x)))
        pts[30, 2] = 0.12
        ground = fit_local_ground(pts, cfg)
        obs, _ = classify_points(np.arange(len(pts)), pts, ground, cfg)
        self.assertEqual(len(obs), 0)

    def test_flat_to_ramp_not_obstacle(self):
        cfg = GroundConfig()
        x = np.linspace(0.0, 1.5, 151)
        z = 0.10 * np.maximum(x - 0.5, 0.0)
        points = np.column_stack((x, np.full_like(x, 0.3), z))
        indices = np.arange(len(points))

        ground = fit_local_ground(points, cfg)
        self.assertIsNotNone(ground)

        obstacles, floor = classify_points(indices, points, ground, cfg)
        self.assertEqual(len(obstacles), 0)
        self.assertGreater(len(floor), 140)

if __name__ == '__main__':
    unittest.main()
