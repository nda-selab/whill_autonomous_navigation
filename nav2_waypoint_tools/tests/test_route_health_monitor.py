"""Health diagnostics with controlled clocks; no ROS node or robot connection."""
import math
import pathlib
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from route_health_monitor import (
    LocalizationHealthGate, RobotMotionMonitor, RouteHealthGate,
    UrgHealthGate, require_healthy,
)


class FakeNode:
    now_ns = 10_000_000_000

    def create_subscription(self, *args):
        return None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))


def localization_status(level=DiagnosticStatus.OK, message='ok', **overrides):
    values = dict(has_converged='true', map_received='true', initialpose_received='true')
    values.update(overrides)
    return DiagnosticArray(status=[DiagnosticStatus(
        name='lidar_localization_ros2/alignment', level=level, message=message,
        values=[KeyValue(key=key, value=value) for key, value in values.items()],
    )])


def urg_status():
    return DiagnosticArray(status=[DiagnosticStatus(
        name=f'urg_obstacle_filter/{side}', level=DiagnosticStatus.OK, message='ok',
    ) for side in ('left', 'right')])


def odometry(sec=10, nanosec=0):
    msg = Odometry()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.child_frame_id = 'base_link'
    return msg


class HealthMonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('route_health_monitor.time.monotonic', return_value=100.0).start()
        self.addCleanup(patch.stopall)
        self.node = FakeNode()
        self.urg = UrgHealthGate(self.node)
        self.localization = LocalizationHealthGate(self.node)
        self.motion = RobotMotionMonitor(self.node)
        self.watchdog = SimpleNamespace(active=True, fault=None)
        self.route = RouteHealthGate(self.urg, self.localization, self.motion, self.watchdog)

    def make_healthy(self):
        self.urg.callback(urg_status())
        self.localization.callback(localization_status())
        self.motion.callback(odometry())

    def test_missing_sources_are_identified(self):
        reasons = '; '.join(self.route.failure_reasons())
        for topic in ('/urg_obstacle_filter/status', '/alignment_status', '/odom_nav'):
            self.assertIn(topic, reasons)
        self.assertFalse(self.route.healthy())

    def test_all_sources_healthy(self):
        self.make_healthy()
        self.assertTrue(self.route.healthy())
        self.assertEqual(self.route.failure_reasons(), [])
        require_healthy(self.route, 'navigation')

    def test_localization_failure_is_not_reported_as_urg_failure(self):
        self.make_healthy()
        self.localization.callback(localization_status(
            level=DiagnosticStatus.WARN, message='fitness_rejected', has_converged='false'))
        with self.assertRaisesRegex(RuntimeError, 'navigation: localization:.*fitness_rejected') as caught:
            require_healthy(self.route, 'navigation')
        self.assertNotIn('URG', str(caught.exception))
        self.assertIn('has_converged=false', str(caught.exception))

    def test_diagnostic_freshness_limits_unchanged(self):
        self.make_healthy()
        self.clock.return_value = 101.5
        self.assertTrue(self.urg.healthy())
        self.assertTrue(self.localization.healthy())
        self.clock.return_value = 101.501
        for gate in (self.urg, self.localization):
            self.assertFalse(gate.healthy())
            self.assertIn('age=1.501s, limit=1.500s', '; '.join(gate.failure_reasons()))

    def test_unrelated_localization_status_cannot_refresh_alignment(self):
        self.make_healthy()
        self.clock.return_value = 102.0
        self.localization.callback(DiagnosticArray(status=[DiagnosticStatus(name='other')]))
        self.assertFalse(self.localization.healthy())

    def test_urg_missing_side_and_error_include_sensor_details(self):
        msg = urg_status()
        msg.status.pop()
        msg.status[0].level = DiagnosticStatus.ERROR
        msg.status[0].message = 'scan timeout'
        self.urg.callback(msg)
        reasons = '; '.join(self.urg.failure_reasons())
        self.assertIn('URG left: level=2', reasons)
        self.assertIn('scan timeout', reasons)
        self.assertIn('URG right: diagnostic missing', reasons)

    def test_odometry_receipt_and_source_ages_are_distinguished(self):
        self.motion.callback(odometry())
        self.clock.return_value = 100.5
        self.node.now_ns = 10_500_000_000
        self.assertTrue(self.motion.healthy())
        self.clock.return_value = 100.501
        self.assertIn('reception stale', '; '.join(self.motion.failure_reasons()))
        self.motion.callback(odometry(nanosec=1))
        self.node.now_ns = 11_000_000_000
        self.assertIn('source timestamp', '; '.join(self.motion.failure_reasons()))
        self.assertNotIn('reception stale', '; '.join(self.motion.failure_reasons()))
        self.node.now_ns = 9_000_000_000
        self.assertFalse(self.motion.healthy())
        self.assertIn('age=-1.000s', '; '.join(self.motion.failure_reasons()))

    def test_invalid_and_repeated_odometry_do_not_confirm_stop(self):
        self.motion.callback(odometry())
        self.motion.callback(odometry())
        self.assertFalse(self.motion.stopped())
        self.assertIn('timestamp did not advance', '; '.join(self.motion.failure_reasons()))
        for kind in ('frame', 'velocity', 'stamp'):
            msg = odometry(sec=11)
            if kind == 'frame':
                msg.child_frame_id = 'odom'
            elif kind == 'velocity':
                msg.twist.twist.linear.x = math.nan
            else:
                msg.header.stamp.sec = 0
            self.motion.callback(msg)
            self.assertFalse(self.motion.healthy())
            self.assertIn('invalid /odom_nav', '; '.join(self.motion.failure_reasons()))
        self.node.now_ns = 11_000_000_000
        self.motion.callback(odometry(sec=11))
        self.assertTrue(self.motion.healthy())
        self.assertFalse(self.motion.stopped())

    def test_watchdog_fault_and_other_failures_are_all_preserved(self):
        self.make_healthy()
        self.watchdog.active = False
        self.watchdog.fault = 'heartbeat expired'
        self.localization.callback(localization_status(map_received='false'))
        reasons = '; '.join(self.route.failure_reasons())
        self.assertIn('watchdog: inactive', reasons)
        self.assertIn('heartbeat expired', reasons)
        self.assertIn('map_received=false', reasons)


if __name__ == '__main__':
    unittest.main()
