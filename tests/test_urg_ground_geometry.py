import math
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from urg_ground_geometry import (ObstacleConfig, classify_near_field, scan_to_points,
                                 transform_points)


class UrgNearFieldTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ObstacleConfig()

    def cloud(self, x, z, indices=None):
        x = np.asarray(x)
        return (np.arange(len(x)) if indices is None else np.asarray(indices),
                np.column_stack((x, np.full_like(x, 0.3), np.asarray(z))))

    def test_projection(self):
        idx, pts = scan_to_points([1.0, float('inf'), 2.0], 0, math.pi / 2, 0.05, 4.0)
        self.assertEqual(idx.tolist(), [0, 2])
        np.testing.assert_allclose(pts[1], [-2, 0, 0], atol=1e-10)

    def test_mount_transform(self):
        cy, sy = math.cos(math.pi / 4), math.sin(math.pi / 4)
        cp, sp = math.cos(math.pi / 4), math.sin(math.pi / 4)
        q = (-sy * sp, sp * cy, cp * sy, cp * cy)
        np.testing.assert_allclose(transform_points(np.array([[1, 0, 0]]),
            (0.4, 0.3, 0.35), q)[0], [0.4, 0.3, -0.65], atol=1e-9)

    def test_flat_floor(self):
        x = np.linspace(0, 1.2, 121)
        ids, pts = self.cloud(x, np.zeros_like(x))
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        self.assertEqual(obs.shape, (0, 3))
        np.testing.assert_array_equal(clearing, pts)

    def test_ramp_fixed_height_detects_as_obstacle(self):
        ids, pts = self.cloud([0, 0.2, 0.4, 0.6, 0.8, 1.0],
                              [0, 0.02, 0.04, 0.06, 0.08, 0.10])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        np.testing.assert_array_equal(obs, pts[3:])
        np.testing.assert_array_equal(clearing, pts)

    def test_obstacle_and_clearing(self):
        ids, pts = self.cloud([0.60, 0.61, 0.62, 0.63], [0, 0.03, 0.10, 0.10])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        np.testing.assert_array_equal(obs, pts[2:])
        np.testing.assert_array_equal(clearing, pts)

    def test_single_obstacle_point_rejected(self):
        ids, pts = self.cloud([0.60, 0.61, 0.62], [0, 0.10, 0])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        self.assertEqual(obs.shape, (0, 3))
        np.testing.assert_array_equal(clearing, pts[[0, 2]])

    def test_near_field_boundaries(self):
        ids, pts = self.cloud([-0.051, -0.05, 1.20, 1.201], [0, 0, 0, 0])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        self.assertEqual(obs.shape, (0, 3))
        np.testing.assert_array_equal(clearing, pts[1:3])

    def test_detects_obstacles_without_any_floor(self):
        ids, pts = self.cloud([0.6, 0.61, 0.62], [0.08, 0.08, 0.08])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        np.testing.assert_array_equal(obs, pts)
        np.testing.assert_array_equal(clearing, pts)

    def test_continues_after_floor_disappears(self):
        x = [0.2, 0.3, 0.6, 0.7]
        for z, expected_start in (([0, 0, 0.1, 0.1], 2), ([0.1] * 4, 0)):
            with self.subTest(z=z):
                ids, pts = self.cloud(x, z)
                obs, clearing = classify_near_field(ids, pts, self.cfg)
                np.testing.assert_array_equal(obs, pts[expected_start:])
                np.testing.assert_array_equal(clearing, pts)

    def test_does_not_clear_unclassified_returns(self):
        ids, pts = self.cloud([0.6, 0.61, 0.62, 0.63], [-0.1, -0.06, 0.5, 0.6])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        self.assertEqual(obs.shape, (0, 3))
        self.assertEqual(clearing.shape, (0, 3))

    def test_requires_adjacent_original_obstacle_beams(self):
        ids, pts = self.cloud([0.6, 0.61], [0.1, 0.1], indices=[10, 12])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        self.assertEqual(obs.shape, (0, 3))
        self.assertEqual(clearing.shape, (0, 3))

    def test_three_centimeter_floor_offset_still_clears(self):
        for height in (-0.03, 0.03):
            with self.subTest(height=height):
                ids, pts = self.cloud([0.8, 0.81], [height, height])
                obs, clearing = classify_near_field(ids, pts, self.cfg)
                self.assertEqual(obs.shape, (0, 3))
                np.testing.assert_array_equal(clearing, pts)

    def test_height_boundaries(self):
        # Paired beams ensure the cluster rule does not hide height-boundary errors.
        cases = [(-0.050001, False, False), (-0.05, False, True),
                 (0.049999, False, True), (0.05, True, True),
                 (0.40, True, True), (0.400001, False, False)]
        for height, marked, cleared in cases:
            with self.subTest(height=height):
                ids, pts = self.cloud([0.6, 0.61], [height, height])
                obs, clearing = classify_near_field(ids, pts, self.cfg)
                np.testing.assert_array_equal(obs, pts if marked else pts[:0])
                np.testing.assert_array_equal(clearing, pts if cleared else pts[:0])

    def test_clearing_upper_bound_excludes_unconfirmed_obstacle(self):
        ids, pts = self.cloud([0.6], [0.05])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        self.assertEqual(obs.shape, (0, 3))
        self.assertEqual(clearing.shape, (0, 3))

    def test_rejects_empty_or_outside_region_observations(self):
        for x, z in (([], []), ([1.3, 1.4], [0.1, 0.1])):
            with self.subTest(x=x):
                ids, pts = self.cloud(x, z)
                self.assertIsNone(classify_near_field(ids, pts, self.cfg))

    def test_down_step_not_cleared(self):
        ids, pts = self.cloud([0.2, 0.3, 0.6, 0.7], [0, 0, -0.10, -0.10])
        obs, clearing = classify_near_field(ids, pts, self.cfg)
        self.assertEqual(obs.shape, (0, 3))
        np.testing.assert_array_equal(clearing, pts[:2])


if __name__ == '__main__':
    unittest.main()
