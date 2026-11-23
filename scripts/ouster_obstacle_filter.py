#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from ouster_filter_core import OusterConfig, split_points


class OusterObstacleFilter(Node):
    def __init__(self):
        super().__init__('ouster_obstacle_filter')

        defaults = OusterConfig()
        values = {
            name: self.declare_parameter(name, value).value
            for name, value in vars(defaults).items()
        }
        self.cfg = OusterConfig(**values)

        if not (
            0 < self.cfg.min_range < self.cfg.marking_max_range <= self.cfg.clearing_max_range
            and self.cfg.clearing_min_z <= self.cfg.marking_min_z
            <= self.cfg.marking_max_z <= self.cfg.clearing_max_z
            and self.cfg.azimuth_bin_deg > 0
        ):
            raise ValueError('Invalid Ouster filter parameters')

        self.marking_pub = self.create_publisher(
            PointCloud2, '/ouster/obstacles', qos_profile_sensor_data)
        self.clearing_pub = self.create_publisher(
            PointCloud2, '/ouster/clearing', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, '/ouster/points', self.process_cloud,
            qos_profile_sensor_data)

    def process_cloud(self, msg):
        if msg.header.frame_id != 'os_lidar':
            self.get_logger().error(
                f'Unexpected point cloud frame: {msg.header.frame_id}')
            return

        try:
            data = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=False)
            xyz = np.column_stack(
                (data['x'], data['y'], data['z'])
            ).astype(np.float32, copy=False)

            obstacles, clearing = split_points(xyz, self.cfg)

            self.marking_pub.publish(
                point_cloud2.create_cloud_xyz32(msg.header, obstacles))
            self.clearing_pub.publish(
                point_cloud2.create_cloud_xyz32(msg.header, clearing))
        except (ValueError, AssertionError, TypeError) as error:
            self.get_logger().error(f'Point cloud processing failed: {error}')


def main(args=None):
    rclpy.init(args=args)
    node = OusterObstacleFilter()
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