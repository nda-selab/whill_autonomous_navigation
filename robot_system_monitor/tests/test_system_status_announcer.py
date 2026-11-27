"""Exercise configured topic monitoring without ROS discovery or speech playback."""
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock, patch

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from system_status_announcer import SystemStatusAnnouncer


class OusterOutputMonitoringTests(unittest.TestCase):
    def setUp(self):
        stack = ExitStack()
        self.addCleanup(stack.close)
        parameters = {}
        self.subscriptions = {}

        def declare(name, default):
            parameters[name] = default
            return SimpleNamespace(value=default)

        def subscribe(message_type, topic, callback, qos):
            self.subscriptions[topic] = (message_type, callback)
            return Mock()

        stack.enter_context(patch.object(Node, '__init__', return_value=None))
        for method, kwargs in {
            'declare_parameter': {'side_effect': declare},
            'get_parameter': {'side_effect': lambda name: SimpleNamespace(value=parameters[name])},
            'create_subscription': {'side_effect': subscribe},
            'create_publisher': {}, 'create_client': {}, 'create_timer': {}, 'get_logger': {},
        }.items():
            stack.enter_context(patch.object(SystemStatusAnnouncer, method, **kwargs))
        stack.enter_context(patch('system_status_announcer.get_package_share_directory', return_value=str(ROOT)))
        stack.enter_context(patch('system_status_announcer.SpeechWorker'))
        stack.enter_context(patch('system_status_announcer.time.monotonic', return_value=100.0))
        self.node = SystemStatusAnnouncer()
        self.node.poll_lifecycle = Mock()
        self.node.whill_power_ok = True
        self.node.last_accept = self.node.last_status_time = 100.0
        self.node.last_status = 'ACCEPT'
        self.node.active_updates = dict.fromkeys(self.node.active_updates, 100.0)
        self.node.last_messages = dict.fromkeys(self.node.last_messages, 100.0)

    def test_uses_both_filtered_cloud_topics_instead_of_laserscan(self):
        self.assertNotIn('/scan_ouster', self.subscriptions)
        self.assertNotIn('scan_ouster', self.node.feed_states)
        for topic, feed in (
            ('/ouster/obstacles', 'ouster_obstacles'),
            ('/ouster/clearing', 'ouster_clearing'),
        ):
            with self.subTest(topic=topic):
                message_type, callback = self.subscriptions[topic]
                self.assertIs(message_type, PointCloud2)
                self.node.last_messages[feed] = None
                # An empty cloud is still a fresh observation of no returns.
                callback(PointCloud2())
                self.assertEqual(self.node.last_messages[feed], 100.0)

    def test_readiness_waits_for_both_outputs(self):
        self.node.last_messages['ouster_obstacles'] = None
        self.node.last_messages['ouster_clearing'] = None
        self.node.check_status()
        self.assertFalse(self.node.system_ready)
        self.subscriptions['/ouster/obstacles'][1](PointCloud2())
        self.node.check_status()
        self.assertFalse(self.node.system_ready)
        self.subscriptions['/ouster/clearing'][1](PointCloud2())
        self.node.check_status()
        self.assertTrue(self.node.system_ready)

    def test_either_output_timing_out_revokes_readiness_until_recovery(self):
        self.node.check_status()
        self.assertTrue(self.node.system_ready)
        for topic, feed in (
            ('/ouster/obstacles', 'ouster_obstacles'),
            ('/ouster/clearing', 'ouster_clearing'),
        ):
            with self.subTest(topic=topic):
                self.node.last_messages[feed] = 90.0
                self.node.check_status()
                self.assertFalse(self.node.system_ready)
                self.assertEqual(self.node.feed_states[feed], 'lost')
                self.subscriptions[topic][1](PointCloud2())
                self.node.check_status()
                self.assertTrue(self.node.system_ready)
                self.assertEqual(self.node.feed_states[feed], 'ok')


if __name__ == '__main__':
    unittest.main()
