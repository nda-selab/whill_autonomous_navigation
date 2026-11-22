#!/usr/bin/env python3
"""Classify near-field returns from two vertically mounted URGs."""
import time
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from urg_ground_geometry import GroundConfig, scan_to_points, transform_points, classify_near_field


class UrgObstacleFilter(Node):
    def __init__(self):
        super().__init__('urg_obstacle_filter')
        default = GroundConfig()
        for name, value in vars(default).items():
            self.declare_parameter(name, value)
        self.declare_parameter('sensor_timeout_sec', 0.6)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('max_scan_range', 1.6)
        self.declare_parameter('enable_ground_filter', True)
        self.cfg = GroundConfig(**{name: self.get_parameter(name).value for name in vars(default)})
        self.sensor_timeout = float(self.get_parameter('sensor_timeout_sec').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.max_scan_range = float(self.get_parameter('max_scan_range').value)
        self.enable_ground_filter = self.get_parameter('enable_ground_filter').value
        self.mode = 'lightweight' if self.enable_ground_filter else 'fixed_height'
        if type(self.enable_ground_filter) is not bool:
            raise ValueError('enable_ground_filter must be a boolean')
        if self.sensor_timeout <= 0 or self.tf_timeout <= 0 or self.max_scan_range <= 0:
            raise ValueError('Invalid timeout or range')
        if not (0 <= self.cfg.obstacle_min_height < self.cfg.obstacle_max_height and self.cfg.obstacle_x_min < self.cfg.obstacle_x_max):
            raise ValueError('Invalid obstacle limits')
        if self.cfg.min_cluster_points < 1 or self.cfg.floor_inlier_tolerance <= 0:
            raise ValueError('Invalid cluster count or floor-height tolerance')
        if self.enable_ground_filter:
            if self.cfg.floor_min_inliers < 2:
                raise ValueError('Invalid floor point count')
            if not (self.cfg.obstacle_x_min <= self.cfg.floor_fit_x_min < self.cfg.floor_fit_x_max <= self.cfg.obstacle_x_max):
                raise ValueError('Ground seed must lie within the obstacle detection window')
            if (self.cfg.floor_track_window <= 0 or self.cfg.floor_track_max_gap <= 0
                    or self.cfg.floor_max_slope <= 0):
                raise ValueError('Invalid ground filter configuration')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.obstacle_pubs = {}
        self.floor_pubs = {}
        self.clearing_pubs = {}
        self.scan_subscriptions = []
        self.last_valid = {side: None for side in ('left', 'right')}
        self.last_message = {side: 'waiting for scan' for side in ('left', 'right')}
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/urg_obstacle_filter/status', 10)
        for side in ('left', 'right'):
            self.obstacle_pubs[side] = self.create_publisher(
                PointCloud2, f'/urg_{side}/obstacles', qos_profile_sensor_data)
            self.floor_pubs[side] = self.create_publisher(
                PointCloud2, f'/urg_{side}/ground', qos_profile_sensor_data)
            self.clearing_pubs[side] = self.create_publisher(
                PointCloud2, f'/urg_{side}/clearing', qos_profile_sensor_data)
            self.scan_subscriptions.append(self.create_subscription(
                LaserScan, f'/urg_{side}/scan', lambda msg, s=side: self.process_scan(s, msg), qos_profile_sensor_data))
        self.timer = self.create_timer(0.25, self.publish_diagnostics)
        self.get_logger().info(f'Near-field URG classifier started (mode={self.mode}; not a safety stop)')

    def publish_cloud(self, publisher, stamp, points):
        header = Header(stamp=stamp, frame_id='base_footprint')
        publisher.publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32).tolist()))

    def process_scan(self, side, scan):
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_footprint', scan.header.frame_id, Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=self.tf_timeout))
            t = tf.transform.translation
            q = tf.transform.rotation
            ids, points = scan_to_points(
                scan.ranges, scan.angle_min, scan.angle_increment,
                max(float(scan.range_min), 0.05), min(float(scan.range_max), self.max_scan_range))
            if not len(points):
                self.last_message[side] = 'no finite scan points'
                return
            points = transform_points(points, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
            classified = classify_near_field(ids, points, self.cfg, self.enable_ground_filter)
            if classified is None:
                reason = ('near-field floor not observable' if self.enable_ground_filter
                          else 'no valid near-field points')
                self.last_message[side] = f'{self.mode}: {reason}'
                return  # Do not report a healthy scan or clear unknown costmap cells.
            obstacles, floor, clearing = classified
            self.publish_cloud(self.clearing_pubs[side], scan.header.stamp, clearing)
            self.publish_cloud(self.obstacle_pubs[side], scan.header.stamp, obstacles)
            self.publish_cloud(self.floor_pubs[side], scan.header.stamp, floor)
            self.last_valid[side] = time.monotonic()
            self.last_message[side] = f'{self.mode}: floor={len(floor)} obstacles={len(obstacles)}'
        except (TransformException, ValueError, TypeError, FloatingPointError) as error:
            self.last_message[side] = f'processing error: {error}'
            self.get_logger().warning(f'{side}: {error}', throttle_duration_sec=2.0)

    def publish_diagnostics(self):
        now = time.monotonic()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for side in ('left', 'right'):
            last = self.last_valid[side]
            fresh = last is not None and now - last <= self.sensor_timeout
            status = DiagnosticStatus()
            status.name = f'urg_obstacle_filter/{side}'
            status.hardware_id = f'urg_{side}'
            status.level = DiagnosticStatus.OK if fresh else DiagnosticStatus.ERROR
            status.message = self.last_message[side] if fresh else f'sensor/processing stale: {self.last_message[side]}'
            status.values = [
                KeyValue(key='fresh', value='true' if fresh else 'false'),
                KeyValue(key='ground_filter_mode', value=self.mode),
            ]
            msg.status.append(status)
        self.diagnostic_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UrgObstacleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
