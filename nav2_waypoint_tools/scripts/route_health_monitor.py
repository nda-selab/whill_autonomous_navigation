#!/usr/bin/env python3

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


def diagnostic_level(status):
    # ROS 2 Humble represents DiagnosticStatus.level (byte) as bytes.
    level = status.level
    return int.from_bytes(level, 'little') if isinstance(level, bytes) else level


def require_healthy(gate, context):
    reasons = gate.failure_reasons()
    if reasons:
        raise RuntimeError(f'{context}: {"; ".join(reasons)}')


class UrgHealthGate:
    """Prevent continuing when either processed vertical URG is stale/bad."""

    def __init__(self, node, timeout=1.5):
        self.last_at = None
        self.statuses = {}
        self.timeout = timeout
        self.subscription = node.create_subscription(
            DiagnosticArray, '/urg_obstacle_filter/status', self.callback, 10)

    def callback(self, msg):
        self.last_at = time.monotonic()
        self.statuses = {
            status.name.rsplit('/', 1)[-1]: status
            for status in msg.status
            if status.name in ('urg_obstacle_filter/left', 'urg_obstacle_filter/right')
        }

    def failure_reasons(self):
        if self.last_at is None:
            return ['URG: /urg_obstacle_filter/status not received']
        reasons = []
        age = time.monotonic() - self.last_at
        if age > self.timeout:
            reasons.append(f'URG: diagnostics stale (age={age:.3f}s, limit={self.timeout:.3f}s)')
        for side in ('left', 'right'):
            status = self.statuses.get(side)
            if status is None:
                reasons.append(f'URG {side}: diagnostic missing')
            elif status.level != DiagnosticStatus.OK:
                reasons.append(
                    f'URG {side}: level={diagnostic_level(status)}, message={status.message!r}')
        return reasons

    def healthy(self):
        return not self.failure_reasons()


class LocalizationHealthGate:

    def __init__(self, node, timeout=1.5):
        self.timeout = timeout
        self.last_received = None
        self.valid = False
        self.invalid_reason = 'alignment not received'

        self.subscription = node.create_subscription(
            DiagnosticArray, '/alignment_status', self.callback, 10,
        )

    def callback(self, msg):
        for status in msg.status:
            if status.name != 'lidar_localization_ros2/alignment':
                continue

            values = {item.key: item.value for item in status.values}

            self.valid = (
                status.level == DiagnosticStatus.OK
                and status.message == 'ok'
                and values.get('has_converged') == 'true'
                and values.get('map_received') == 'true'
                and values.get('initialpose_received') == 'true'
            )
            self.invalid_reason = (
                f'level={diagnostic_level(status)}, message={status.message!r}, '
                f'has_converged={values.get("has_converged", "missing")}, '
                f'map_received={values.get("map_received", "missing")}, '
                f'initialpose_received={values.get("initialpose_received", "missing")}'
            )
            self.last_received = time.monotonic()
            return

    def failure_reasons(self):
        if self.last_received is None:
            return ['localization: /alignment_status alignment not received']
        reasons = []
        age = time.monotonic() - self.last_received
        if age > self.timeout:
            reasons.append(
                f'localization: diagnostics stale (age={age:.3f}s, limit={self.timeout:.3f}s)')
        if not self.valid:
            reasons.append(f'localization: {self.invalid_reason}')
        return reasons

    def healthy(self):
        return not self.failure_reasons()


class RobotMotionMonitor:

    def __init__(self, node, timeout=0.5,
                 linear_threshold=0.02, angular_threshold=0.05,
                 stable_time=0.5):

        self.node = node
        self.timeout = timeout
        self.linear_threshold = linear_threshold
        self.angular_threshold = angular_threshold
        self.stable_time = stable_time

        self.last_received = None
        self.last_stamp = None
        self.stationary_since = None
        self.linear_speed = math.inf
        self.angular_speed = math.inf
        self.invalid_reason = '/odom_nav not received'

        self.subscription = node.create_subscription(
            Odometry, '/odom_nav', self.callback,
            qos_profile_sensor_data,
        )

    def callback(self, msg):
        now = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        twist = msg.twist.twist

        values = (
            twist.linear.x, twist.linear.y,
            twist.angular.z,
        )

        if (msg.child_frame_id != 'base_link'
                or not all(math.isfinite(v) for v in values)
                or stamp <= 0):
            self.stationary_since = None
            self.last_received = None
            self.invalid_reason = (
                f'invalid /odom_nav (child_frame_id={msg.child_frame_id!r}, '
                f'stamp_ns={stamp}, velocity={values})'
            )
            return

        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.stationary_since = None
            self.last_received = None
            self.invalid_reason = (
                f'/odom_nav timestamp did not advance '
                f'(stamp_ns={stamp}, previous_ns={self.last_stamp})'
            )
            return

        if (self.last_received is None
                or now - self.last_received > self.timeout):
            self.stationary_since = None

        self.last_stamp = stamp
        self.last_received = now
        self.invalid_reason = None

        self.linear_speed = math.hypot(
            twist.linear.x, twist.linear.y,
        )
        self.angular_speed = abs(twist.angular.z)

        if (self.linear_speed <= self.linear_threshold
                and self.angular_speed <= self.angular_threshold):

            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None

    def failure_reasons(self):
        if self.last_received is None or self.last_stamp is None:
            return [f'odometry: {self.invalid_reason}']

        age = (
            self.node.get_clock().now().nanoseconds - self.last_stamp
        ) * 1e-9

        reasons = []
        received_age = time.monotonic() - self.last_received
        if received_age > self.timeout:
            reasons.append(
                f'odometry: /odom_nav reception stale '
                f'(age={received_age:.3f}s, limit={self.timeout:.3f}s)')
        if not 0.0 <= age <= self.timeout:
            reasons.append(
                f'odometry: /odom_nav source timestamp outside freshness window '
                f'(age={age:.3f}s, allowed=0..{self.timeout:.3f}s)')
        return reasons

    def healthy(self):
        return not self.failure_reasons()

    def reset_stop_confirmation(self):
        self.stationary_since = None

    def stopped(self):
        return (
            self.healthy()
            and self.stationary_since is not None
            and time.monotonic() - self.stationary_since >= self.stable_time
        )


class RouteHealthGate:

    def __init__(self, urg_gate, localization_gate, motion, watchdog):
        self.urg_gate = urg_gate
        self.localization_gate = localization_gate
        self.motion = motion
        self.watchdog = watchdog

    def healthy(self):
        return not self.failure_reasons()

    def failure_reasons(self):
        reasons = []
        for gate in (self.urg_gate, self.localization_gate, self.motion):
            reasons.extend(gate.failure_reasons())
        if not self.watchdog.active:
            reasons.append('watchdog: inactive')
        if self.watchdog.fault is not None:
            reasons.append(f'watchdog: {self.watchdog.fault}')
        return reasons
