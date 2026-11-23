"""Test front URG marking; clearing publication is not implemented yet."""
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


class FrontUrgMarkingTests(unittest.TestCase):
    def setUp(self):
        params = yaml.safe_load((ROOT / 'config/obstacle_filter_params.yaml').read_text())
        params = params['urg_front_obstacle_filter']['ros__parameters']
        self.node = SimpleNamespace(
            min_range=params['marking_min_range'],
            max_range=params['marking_max_range'], publisher=Mock(),
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
        self.node.publisher.publish.assert_called_once()
        msg = self.node.publisher.publish.call_args.args[0]
        self.assertEqual(msg.header, self.scan.header)
        data = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'])
        return np.column_stack((data['x'], data['y'], data['z']))

    def test_invalid_beams_do_not_shift_remaining_angles(self):
        self.scan.ranges = [float('inf'), 1.0, float('nan'), 2.0, 0.02, 5.0]
        points = self.process()
        angles = np.array([self.scan.angle_min + i * self.scan.angle_increment for i in (1, 3)])
        distances = np.array([1.0, 2.0])
        expected = np.column_stack((distances * np.cos(angles), distances * np.sin(angles), [0, 0]))
        np.testing.assert_allclose(points, expected, atol=1e-6)

    def test_sensor_range_limits_also_apply(self):
        self.scan.range_min = 0.2
        self.scan.range_max = 3.0
        self.scan.ranges = [0.1, 0.5, 3.5]
        points = self.process()
        self.assertEqual(points.shape, (1, 3))
        self.assertAlmostEqual(float(np.linalg.norm(points[0])), 0.5, places=6)

    def test_invalid_scan_publishes_empty_marking_cloud(self):
        self.scan.ranges = [float('inf'), float('nan'), 0.01]
        self.assertEqual(self.process().shape, (0, 3))


if __name__ == '__main__':
    unittest.main()
