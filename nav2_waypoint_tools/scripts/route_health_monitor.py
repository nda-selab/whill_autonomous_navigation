#!/usr/bin/env python3

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


class LocalizationHealthGate:

    def __init__(self, node, timeout=1.5):
        self.timeout = timeout
        self.last_received = None
        self.valid = False

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
            self.last_received = time.monotonic()
            return

    def healthy(self):
        return (
            self.valid
            and self.last_received is not None
            and time.monotonic() - self.last_received <= self.timeout
        )


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
            return

        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.stationary_since = None
            self.last_received = None
            return

        if (self.last_received is None
                or now - self.last_received > self.timeout):
            self.stationary_since = None

        self.last_stamp = stamp
        self.last_received = now

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

    def healthy(self):
        if self.last_received is None or self.last_stamp is None:
            return False

        age = (
            self.node.get_clock().now().nanoseconds - self.last_stamp
        ) * 1e-9

        return (
            time.monotonic() - self.last_received <= self.timeout
            and 0.0 <= age <= self.timeout
        )

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
        return (
            self.urg_gate.healthy()
            and self.localization_gate.healthy()
            and self.motion.healthy()
            and self.watchdog.active
            and self.watchdog.fault is None
        )