"""Exercise scan handling and freshness without starting ROS nodes or hardware."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from urg_ground_geometry import GroundConfig
from urg_obstacle_filter import UrgObstacleFilter


class FixedHeightScanTests(unittest.TestCase):
    def setUp(self):
        self.scan = LaserScan()
        self.scan.header.frame_id = 'laser_left'
        self.scan.range_min = 0.05
        self.scan.range_max = 1.6
        self.scan.angle_increment = 0.01
        self.scan.ranges = [0.6, 0.61]
        transform = TransformStamped()
        transform.transform.rotation.w = 1.0
        transform.transform.translation.z = 0.1  # Obstacle returns, no floor.
        self.published = {}
        self.node = SimpleNamespace(
            cfg=GroundConfig(), enable_ground_filter=False, mode='fixed_height',
            tf_timeout=0.2, max_scan_range=1.6, sensor_timeout=0.6,
            tf_buffer=Mock(),
            clearing_pubs={'left': 'clearing'}, obstacle_pubs={'left': 'obstacles'},
            floor_pubs={'left': 'floor'},
            last_valid={'left': None, 'right': None},
            last_message={'left': 'waiting for scan', 'right': 'waiting for scan'},
            publish_cloud=lambda pub, stamp, points: self.published.update({pub: points}),
            diagnostic_pub=Mock(), get_clock=Mock(), get_logger=Mock(),
        )
        self.node.tf_buffer.lookup_transform.return_value = transform
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = self.scan.header.stamp

    def process(self, now):
        with patch('urg_obstacle_filter.time.monotonic', return_value=now):
            UrgObstacleFilter.process_scan(self.node, 'left', self.scan)

    def diagnostic(self, now):
        with patch('urg_obstacle_filter.time.monotonic', return_value=now):
            UrgObstacleFilter.publish_diagnostics(self.node)
        msg = self.node.diagnostic_pub.publish.call_args.args[0]
        return next(s for s in msg.status if s.name.endswith('/left'))

    def test_obstacle_only_scan_updates_cloud_and_health_then_times_out(self):
        self.process(10.0)
        self.assertEqual(len(self.published['obstacles']), 2)
        self.assertEqual(len(self.published['floor']), 0)
        np.testing.assert_array_equal(self.published['clearing'], self.published['obstacles'])
        self.assertEqual(self.node.last_valid['left'], 10.0)
        self.assertEqual(self.diagnostic(10.1).level, DiagnosticStatus.OK)
        self.assertEqual(self.diagnostic(10.7).level, DiagnosticStatus.ERROR)

    def test_tf_failure_does_not_refresh_cloud_or_health(self):
        self.node.last_valid['left'] = 9.0
        self.node.tf_buffer.lookup_transform.side_effect = TransformException('missing TF')
        self.process(10.0)
        self.assertFalse(self.published)
        self.assertEqual(self.node.last_valid['left'], 9.0)
        self.assertEqual(self.diagnostic(10.0).level, DiagnosticStatus.ERROR)

    def test_invalid_scan_does_not_refresh_cloud_or_health(self):
        self.scan.ranges = [float('inf'), float('nan')]
        self.process(10.0)
        self.assertFalse(self.published)
        self.assertIsNone(self.node.last_valid['left'])
        self.assertEqual(self.diagnostic(10.0).level, DiagnosticStatus.ERROR)

    def test_unclassified_scan_updates_health_without_clearing_unknown_space(self):
        self.node.tf_buffer.lookup_transform.return_value.transform.translation.z = 0.6
        self.process(10.0)
        for cloud in self.published.values():
            self.assertEqual(cloud.shape, (0, 3))
        self.assertEqual(self.node.last_valid['left'], 10.0)
        self.assertEqual(self.diagnostic(10.1).level, DiagnosticStatus.OK)


if __name__ == '__main__':
    unittest.main()
