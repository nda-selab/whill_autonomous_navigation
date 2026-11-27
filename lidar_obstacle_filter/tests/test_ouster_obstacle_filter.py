"""Check Ouster filtering and ROS message handling without starting a ROS node."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock

import numpy as np
import yaml
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from ouster_filter_core import OusterConfig, split_points
from ouster_obstacle_filter import OusterObstacleFilter


def configured_filter():
    params = yaml.safe_load((ROOT / 'config/obstacle_filter_params.yaml').read_text())
    return OusterConfig(**params['ouster_obstacle_filter']['ros__parameters'])


def cloud_xyz(message):
    data = point_cloud2.read_points(message, field_names=['x', 'y', 'z'])
    return np.column_stack((data['x'], data['y'], data['z']))


class OusterGeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = configured_filter()

    def test_marking_and_clearing_height_and_range_limits(self):
        cases = [
            ([2, 0, -0.5], True, True),       # Obstacle surface.
            ([2, 0, -0.85], False, True),     # Low return, clearing only.
            ([2, 0, 0.2], False, True),       # Above marking band.
            ([2, 0, 0.5], False, False),      # Above both bands.
            ([9, 0, -0.5], False, True),      # Beyond marking range.
            ([11, 0, -0.5], False, False),    # Beyond clearing range.
            ([0.02, 0, -0.5], False, False),  # Too close.
        ]
        for point, mark, clear in cases:
            with self.subTest(point=point):
                pts = np.asarray([point], dtype=np.float32)
                marking, clearing = split_points(pts, self.cfg)
                np.testing.assert_array_equal(marking, pts if mark else pts[:0])
                np.testing.assert_array_equal(clearing, pts if clear else pts[:0])

    def test_same_direction_does_not_clear_through_near_obstacle(self):
        pts = np.array([[2, 0, -0.5], [1, 0, -0.5]], dtype=np.float32)
        marking, clearing = split_points(pts, self.cfg)
        np.testing.assert_array_equal(marking, pts)
        np.testing.assert_array_equal(clearing, pts[1:])

    def test_independent_directions_keep_their_returns(self):
        pts = np.array([[2, 0, -0.5], [0, 3, -0.5]], dtype=np.float32)
        marking, clearing = split_points(pts, self.cfg)
        np.testing.assert_array_equal(marking, pts)
        np.testing.assert_array_equal(clearing, pts)

    def test_empty_and_nonfinite_points_produce_empty_clouds(self):
        for pts in (np.empty((0, 3)), [[np.nan, 0, 0], [1, 0, np.inf]]):
            with self.subTest(points=pts):
                marking, clearing = split_points(pts, self.cfg)
                self.assertEqual(marking.shape, (0, 3))
                self.assertEqual(clearing.shape, (0, 3))


class OusterMessageTests(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            cfg=configured_filter(), marking_pub=Mock(), clearing_pub=Mock(),
            get_logger=Mock(),
        )
        self.header = Header(frame_id='os_lidar')
        self.header.stamp.sec = 123
        self.header.stamp.nanosec = 456

    def test_split_clouds_preserve_frame_and_timestamp(self):
        pts = np.array([[2, 0, -0.5], [0, 2, -0.85]], dtype=np.float32)
        msg = point_cloud2.create_cloud_xyz32(self.header, pts)
        OusterObstacleFilter.process_cloud(self.node, msg)
        self.node.marking_pub.publish.assert_called_once()
        self.node.clearing_pub.publish.assert_called_once()
        marking = self.node.marking_pub.publish.call_args.args[0]
        clearing = self.node.clearing_pub.publish.call_args.args[0]
        self.assertEqual(marking.header, self.header)
        self.assertEqual(clearing.header, self.header)
        np.testing.assert_array_equal(cloud_xyz(marking), pts[:1])
        np.testing.assert_array_equal(cloud_xyz(clearing), pts)
        self.node.get_logger.return_value.error.assert_not_called()

    def test_empty_scan_publishes_empty_marking_and_clearing(self):
        msg = point_cloud2.create_cloud_xyz32(self.header, np.empty((0, 3)))
        OusterObstacleFilter.process_cloud(self.node, msg)
        for publisher in (self.node.marking_pub, self.node.clearing_pub):
            publisher.publish.assert_called_once()
            self.assertEqual(cloud_xyz(publisher.publish.call_args.args[0]).shape, (0, 3))

    def test_unexpected_frame_is_rejected_without_publishing(self):
        self.header.frame_id = 'base_link'
        msg = point_cloud2.create_cloud_xyz32(self.header, [[2, 0, -0.5]])
        OusterObstacleFilter.process_cloud(self.node, msg)
        self.node.marking_pub.publish.assert_not_called()
        self.node.clearing_pub.publish.assert_not_called()
        self.node.get_logger.return_value.error.assert_called_once()


if __name__ == '__main__':
    unittest.main()
