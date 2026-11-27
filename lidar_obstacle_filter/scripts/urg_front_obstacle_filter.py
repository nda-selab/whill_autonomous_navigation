#!/usr/bin/env python3
"""Generate marking and clearing clouds from the front URG LaserScan."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def project_points(scan, indices, distances):
    """Project selected LaserScan beams into XYZ points in laser_front."""
    if len(indices) == 0:
        return np.empty((0, 3), dtype=np.float32)

    angles = scan.angle_min + indices * scan.angle_increment
    points = np.column_stack((
        distances * np.cos(angles),
        distances * np.sin(angles),
        np.zeros(len(distances)),
    ))
    return points.astype(np.float32)


class UrgFrontObstacleFilter(Node):
    def __init__(self):
        super().__init__('urg_front_obstacle_filter')

        self.min_range = float(
            self.declare_parameter('marking_min_range', 0.05).value)
        self.marking_max_range = float(
            self.declare_parameter('marking_max_range', 4.0).value)
        self.clearing_max_range = float(
            self.declare_parameter('clearing_max_range', 4.0).value)
        self.clear_on_inf = bool(
            self.declare_parameter('clear_on_inf', True).value)

        if not (0 < self.min_range < self.marking_max_range <= self.clearing_max_range):
            raise ValueError('Invalid front URG range parameters')

        self.marking_pub = self.create_publisher(
            PointCloud2, '/urg_front/obstacles', qos_profile_sensor_data)
        self.clearing_pub = self.create_publisher(
            PointCloud2, '/urg_front/clearing', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, '/urg_front/scan', self.process_scan,
            qos_profile_sensor_data)

    def process_scan(self, scan):
        # Reject malformed scans rather than interpreting them as free space.
        valid_metadata = (
            scan.header.frame_id == 'laser_front'
            and len(scan.ranges) > 0
            and np.isfinite([
                scan.angle_min, scan.angle_increment,
                scan.range_min, scan.range_max,
            ]).all()
            and scan.angle_increment > 0
            and scan.range_max > scan.range_min
        )
        if not valid_metadata:
            self.get_logger().warning('Invalid front URG scan metadata')
            return

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        min_range = max(self.min_range, float(scan.range_min))
        marking_max = min(self.marking_max_range, float(scan.range_max))
        clearing_max = min(self.clearing_max_range, float(scan.range_max))

        if clearing_max < min_range:
            self.get_logger().warning('Front URG valid range is too narrow')
            return

        # Finite returns inside the sensor's valid measurement range.
        valid_hits = (
            np.isfinite(ranges)
            & (ranges >= min_range)
            & (ranges <= scan.range_max)
        )

        # Marking: keep only returns within the configured obstacle range.
        marking_mask = valid_hits & (ranges <= marking_max)
        marking_ids = np.flatnonzero(marking_mask)
        marking_points = project_points(
            scan, marking_ids, ranges[marking_ids])

        # Clearing: finite returns, plus optional positive-infinity no-returns.
        no_returns = np.isposinf(ranges) & self.clear_on_inf
        clearing_mask = valid_hits | no_returns

        # Do not publish a fresh observation if every beam is invalid.
        if not np.any(clearing_mask):
            self.get_logger().warning('Front URG scan has no usable beams')
            return

        clearing_ids = np.flatnonzero(clearing_mask)
        clearing_distances = np.where(
            valid_hits[clearing_ids],
            np.minimum(ranges[clearing_ids], clearing_max),
            clearing_max,
        )
        clearing_points = project_points(
            scan, clearing_ids, clearing_distances)

        # Preserve the original frame and acquisition timestamp.
        self.marking_pub.publish(
            point_cloud2.create_cloud_xyz32(scan.header, marking_points))
        self.clearing_pub.publish(
            point_cloud2.create_cloud_xyz32(scan.header, clearing_points))


def main(args=None):
    rclpy.init(args=args)
    node = UrgFrontObstacleFilter()
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