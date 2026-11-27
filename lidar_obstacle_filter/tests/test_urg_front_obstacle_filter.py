"""Test front URG marking and clearing without starting a ROS node."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock

import numpy as np
import yaml
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from urg_front_obstacle_filter import UrgFrontObstacleFilter


class FrontUrgFilterTests(unittest.TestCase):
    def setUp(self):
        params = yaml.safe_load((ROOT / 'config/obstacle_filter_params.yaml').read_text())
        params = params['urg_front_obstacle_filter']['ros__parameters']
        self.node = SimpleNamespace(
            min_range=params['marking_min_range'],
            marking_max_range=params['marking_max_range'],
            clearing_max_range=params['clearing_max_range'],
            clear_on_inf=params['clear_on_inf'],
            marking_pub=Mock(),
            clearing_pub=Mock(),
            get_logger=Mock(),
        )
        self.scan = LaserScan()
        self.scan.header.frame_id = 'laser_front'
        self.scan.header.stamp.sec = 123
        self.scan.angle_min = -0.5
        self.scan.angle_increment = 0.1
        self.scan.range_min = 0.05
        self.scan.range_max = 10.0

    def process(self):
        UrgFrontObstacleFilter.process_scan(self.node, self.scan)
        clouds = []
        for publisher in (self.node.marking_pub, self.node.clearing_pub):
            publisher.publish.assert_called_once()
            msg = publisher.publish.call_args.args[0]
            self.assertEqual(msg.header, self.scan.header)
            clouds.append(point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z']))
        return clouds

    def test_invalid_beams_do_not_shift_remaining_angles(self):
        self.scan.ranges = [float('inf'), 1.0, float('nan'), 2.0, 0.02, 5.0]
        points, clearing = self.process()
        angles = np.array([self.scan.angle_min + i * self.scan.angle_increment for i in (1, 3)])
        distances = np.array([1.0, 2.0])
        expected = np.column_stack((distances * np.cos(angles), distances * np.sin(angles), [0, 0]))
        np.testing.assert_allclose(points, expected, atol=1e-6)
        clearing_angles = np.array([
            self.scan.angle_min + i * self.scan.angle_increment for i in (0, 1, 3, 5)
        ])
        clearing_ranges = np.array([4.0, 1.0, 2.0, 4.0])
        expected_clearing = np.column_stack((
            clearing_ranges * np.cos(clearing_angles),
            clearing_ranges * np.sin(clearing_angles), np.zeros(4),
        ))
        np.testing.assert_allclose(clearing, expected_clearing, atol=1e-6)

    def test_sensor_range_limits_also_apply(self):
        self.scan.range_min = 0.2
        self.scan.range_max = 3.0
        self.scan.ranges = [0.1, 0.5, 3.5]
        points, clearing = self.process()
        self.assertEqual(points.shape, (1, 3))
        self.assertAlmostEqual(float(np.linalg.norm(points[0])), 0.5, places=6)
        np.testing.assert_array_equal(clearing, points)

    def test_inf_only_usable_beam_publishes_empty_marking_and_clears(self):
        self.scan.ranges = [float('inf'), float('nan'), 0.01]
        marking, clearing = self.process()
        self.assertEqual(marking.shape, (0, 3))
        self.assertEqual(clearing.shape, (1, 3))
        self.assertAlmostEqual(float(np.linalg.norm(clearing[0])), 4.0, places=5)

    def test_inf_generates_clearing_endpoint(self):
        self.scan.ranges = [float('inf'), 2.0, float('nan')]
        marking_xyz, clearing_xyz = self.process()
        self.assertEqual(len(marking_xyz), 1)
        self.assertEqual(len(clearing_xyz), 2)
        self.assertAlmostEqual(
            np.linalg.norm(clearing_xyz[0]), 4.0, places=5)
        self.assertAlmostEqual(
            np.linalg.norm(clearing_xyz[1]), 2.0, places=5)

    def test_no_usable_beams_do_not_publish_fresh_observations(self):
        self.scan.ranges = [float('nan'), float('-inf'), 0.01, 11.0]
        UrgFrontObstacleFilter.process_scan(self.node, self.scan)
        self.node.marking_pub.publish.assert_not_called()
        self.node.clearing_pub.publish.assert_not_called()
        self.node.get_logger.return_value.warning.assert_called_once()

    def test_disabling_inf_clearing_keeps_only_finite_hits(self):
        self.node.clear_on_inf = False
        self.scan.ranges = [float('inf'), 2.0]
        marking, clearing = self.process()
        self.assertEqual(marking.shape, (1, 3))
        np.testing.assert_array_equal(clearing, marking)

    def test_inf_endpoint_respects_sensor_maximum_range(self):
        self.scan.range_max = 3.0
        self.scan.ranges = [float('inf')]
        marking, clearing = self.process()
        self.assertEqual(marking.shape, (0, 3))
        self.assertAlmostEqual(float(np.linalg.norm(clearing[0])), 3.0, places=5)

    def test_malformed_metadata_does_not_publish(self):
        cases = [
            ('frame', 'wrong_frame'), ('angle_increment', 0.0),
            ('angle_min', float('nan')), ('range_max', 0.01),
            ('ranges', []),
        ]
        for field, value in cases:
            with self.subTest(field=field):
                self.setUp()
                self.scan.ranges = [1.0]
                if field == 'frame':
                    self.scan.header.frame_id = value
                else:
                    setattr(self.scan, field, value)
                UrgFrontObstacleFilter.process_scan(self.node, self.scan)
                self.node.marking_pub.publish.assert_not_called()
                self.node.clearing_pub.publish.assert_not_called()
                self.node.get_logger.return_value.warning.assert_called_once()


if __name__ == '__main__':
    unittest.main()
