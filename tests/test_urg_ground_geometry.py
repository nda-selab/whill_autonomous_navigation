import math
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from urg_ground_geometry import (GroundConfig, classify_near_field, scan_to_points,
                                 transform_points)


class UrgNearFieldTests(unittest.TestCase):
    def setUp(self):
        self.cfg = GroundConfig()

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
        for mode in (True, False):
            obs, floor, clearing = classify_near_field(ids, pts, self.cfg, mode)
            self.assertEqual(len(obs), 0)
            self.assertEqual(len(floor), len(x))
            self.assertEqual(len(clearing), len(x))

    def test_ramp_lightweight(self):
        x = np.linspace(0, 1.2, 121)
        ids, pts = self.cloud(x, 0.10 * x)
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 0)
        self.assertGreater(len(floor), 100)

    def test_flat_to_ramp_lightweight(self):
        x = np.linspace(0, 1.2, 121)
        ids, pts = self.cloud(x, 0.1 * np.maximum(x - 0.5, 0))
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 0)
        self.assertGreater(len(floor), 100)

    def test_ramp_fixed_height_detects_as_obstacle(self):
        x = np.linspace(0, 1.2, 121)
        ids, pts = self.cloud(x, 0.10 * x)
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, False)
        self.assertGreater(len(obs), 40)

    def test_step_lightweight_not_absorbed_into_floor(self):
        x = np.linspace(0, 1.2, 121)
        ids, pts = self.cloud(x, np.where(x < 0.6, 0, 0.10))
        obs, floor, _ = classify_near_field(ids, pts, self.cfg, True)
        self.assertGreater(len(obs), 40)
        self.assertLessEqual(np.max(floor[:, 0]), 0.6)

    def test_two_beam_protrusion(self):
        x = np.linspace(0, 1.2, 121)
        z = 0.1 * x
        z[79:81] += 0.10
        ids, pts = self.cloud(x, z)
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 2)
        self.assertEqual(len(clearing), len(floor) + 2)

    def test_single_noise_ignored_and_not_cleared(self):
        x = np.linspace(0, 1.2, 121)
        z = np.zeros_like(x)
        z[80] = 0.12
        ids, pts = self.cloud(x, z)
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 0)
        self.assertEqual(len(clearing), len(floor))
        self.assertTrue(np.all(clearing[:, 2] == 0))

    def test_near_field_cutoff(self):
        x = np.linspace(0, 1.5, 151)
        z = np.zeros_like(x)
        z[135:140] = 0.2
        ids, pts = self.cloud(x, z)
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 0)
        self.assertTrue(np.all(clearing[:, 0] <= 1.2))

    def test_no_floor_fails_closed_with_ground_filter(self):
        ids, pts = self.cloud([0.1, 0.2], [0.4, 0.5])
        self.assertIsNone(classify_near_field(ids, pts, self.cfg, True))

    def test_fixed_height_detects_obstacles_without_any_floor(self):
        ids, pts = self.cloud([0.6, 0.61, 0.62], [0.08, 0.08, 0.08])
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, False)
        np.testing.assert_array_equal(obs, pts)
        self.assertEqual(floor.shape, (0, 3))
        np.testing.assert_array_equal(clearing, pts)

    def test_fixed_height_continues_after_floor_disappears(self):
        x = np.linspace(0, 1.2, 121)
        for z in (np.where(x < 0.6, 0, 0.1), np.full_like(x, 0.1)):
            ids, pts = self.cloud(x, z)
            obs, _, _ = classify_near_field(ids, pts, self.cfg, False)
            self.assertGreater(len(obs), 0)

    def test_fixed_height_does_not_clear_unclassified_returns(self):
        ids, pts = self.cloud([0.6, 0.61, 0.62, 0.63], [-0.1, 0.03, 0.5, 0.6])
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, False)
        for cloud in (obs, floor, clearing):
            self.assertEqual(cloud.shape, (0, 3))

    def test_fixed_height_still_requires_adjacent_obstacle_beams(self):
        ids, pts = self.cloud([0.6, 0.61], [0.1, 0.1], indices=[10, 12])
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, False)
        for cloud in (obs, floor, clearing):
            self.assertEqual(cloud.shape, (0, 3))

    def test_fixed_height_keeps_observed_floor_clearing_without_seed(self):
        ids, pts = self.cloud([0.8, 0.81], [0, 0])
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, False)
        self.assertEqual(obs.shape, (0, 3))
        np.testing.assert_array_equal(floor, pts)
        np.testing.assert_array_equal(clearing, pts)

    def test_fixed_height_rejects_empty_or_outside_region_observations(self):
        for x, z in (([], []), ([1.3, 1.4], [0.1, 0.1])):
            ids, pts = self.cloud(x, z)
            self.assertIsNone(classify_near_field(ids, pts, self.cfg, False))

    def test_obstacle_surface_is_not_accepted_as_floor(self):
        x = np.linspace(0, 1.2, 121)
        ids, pts = self.cloud(x, np.full_like(x, 0.10))
        self.assertIsNone(classify_near_field(ids, pts, self.cfg, True))

    def test_unclassified_high_returns_do_not_clear(self):
        x = np.linspace(0, 1.2, 121)
        z = np.zeros_like(x)
        z[70:75] = 0.65
        ids, pts = self.cloud(x, z)
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 0)
        self.assertTrue(np.all(clearing[:, 2] < 0.40))

    def test_distant_points_do_not_validate_floor(self):
        ids, pts = self.cloud([1.3, 1.4, 1.5], [0, 0, 0])
        self.assertIsNone(classify_near_field(ids, pts, self.cfg, True))

    def test_beam_index_gap_not_clustered(self):
        x = np.linspace(0, 1.2, 121)
        z = np.zeros_like(x)
        z[60:62] = 0.1
        ids = np.arange(len(x))
        ids[61:] += 2
        ids, pts = self.cloud(x, z, ids)
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 0)

    def test_down_step_not_cleared(self):
        x = np.linspace(0, 1.2, 121)
        ids, pts = self.cloud(x, np.where(x < 0.6, 0, -0.10))
        obs, floor, clearing = classify_near_field(ids, pts, self.cfg, True)
        self.assertEqual(len(obs), 0)
        self.assertTrue(np.all(clearing[:, 2] >= -0.03))


if __name__ == '__main__':
    unittest.main()
