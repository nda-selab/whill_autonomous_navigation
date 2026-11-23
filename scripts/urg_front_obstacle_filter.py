#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class UrgFrontObstacleFilter(Node):
    def __init__(self):
        super().__init__('urg_front_obstacle_filter')

        self.min_range = float(
            self.declare_parameter('marking_min_range', 0.05).value)
        self.max_range = float(
            self.declare_parameter('marking_max_range', 4.0).value)

        if not 0 < self.min_range < self.max_range:
            raise ValueError('Invalid front URG range parameters')

        self.publisher = self.create_publisher(
            PointCloud2, '/urg_front/obstacles', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, '/urg_front/scan', self.process_scan,
            qos_profile_sensor_data)

    def process_scan(self, scan):
        ranges = np.asarray(scan.ranges, dtype=np.float32)

        valid = (
            np.isfinite(ranges)
            & (ranges >= max(self.min_range, scan.range_min))
            & (ranges <= min(self.max_range, scan.range_max))
        )

        indices = np.flatnonzero(valid)
        distances = ranges[indices]
        angles = scan.angle_min + indices * scan.angle_increment

        points = np.column_stack((
            distances * np.cos(angles),
            distances * np.sin(angles),
            np.zeros(len(distances), dtype=np.float32),
        )).astype(np.float32)

        self.publisher.publish(
            point_cloud2.create_cloud_xyz32(scan.header, points))


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