#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult


class CmdVelWatchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/whill/controller/cmd_vel')
        self.declare_parameter('command_timeout_sec', 0.4)
        self.declare_parameter('localization_timeout_sec', 1.5)
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('max_forward_speed', 0.25)
        self.declare_parameter('max_reverse_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('route_speed_limit', float(self.get_parameter('max_forward_speed').value))
        self.declare_parameter('route_hold', False)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.timeout = float(self.get_parameter('command_timeout_sec').value)
        self.localization_timeout = float(self.get_parameter('localization_timeout_sec').value)
        frequency = float(self.get_parameter('publish_frequency').value)
        self.max_forward = float(self.get_parameter('max_forward_speed').value)
        self.max_reverse = float(self.get_parameter('max_reverse_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)

        if input_topic == output_topic:
            raise ValueError('Input and output topics must differ')

        values = [
            self.timeout,
            frequency,
            self.max_forward,
            self.max_reverse,
            self.max_angular,
        ]
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Invalid watchdog configuration')

        if not math.isfinite(self.localization_timeout) or self.localization_timeout <= 0:
            raise ValueError('Invalid localization timeout')

        self.last_received = None
        self.last_command = None
        self.last_alignment_received = None
        self.alignment_healthy = False

        self.publisher = self.create_publisher(
            Twist,
            output_topic,
            10,
        )
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.on_command,
            10,
        )
        self.alignment_subscription = self.create_subscription(
            DiagnosticArray,
            '/alignment_status',
            self.on_alignment_status,
            10,
        )
        self.timer = self.create_timer(
            1.0 / frequency, self.on_timer
        )

        self.add_on_set_parameters_callback(
            self.on_parameter_change
        )

    def on_command(self, msg):
        if self.get_parameter('route_hold').value:
            self.last_command = None
            self.last_received = None
            return
        
        values = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]

        if not all(math.isfinite(v) for v in values):
            self.last_command = None
            self.last_received = None
            self.get_logger().error('Invalid velocity command')
            return

        if any(abs(v) > 1e-6 for v in [
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
        ]):
            self.last_command = None
            self.last_received = None
            self.get_logger().error('Unsupported velocity command')
            return

        command = Twist()

        command.linear.x = min(
            self.max_forward,
            max(-self.max_reverse, msg.linear.x),
        )
        command.angular.z = min(
            self.max_angular,
            max(-self.max_angular, msg.angular.z),
        )

        self.last_command = command
        self.last_received = time.monotonic()

    def on_alignment_status(self, msg):
        for status in msg.status:
            if status.name != 'lidar_localization_ros2/alignment':
                continue
    
            values = {item.key: item.value for item in status.values}
    
            self.alignment_healthy = (
                status.level == DiagnosticStatus.OK
                and status.message == 'ok'
                and values.get('has_converged') == 'true'
                and values.get('map_received') == 'true'
                and values.get('initialpose_received') == 'true'
            )
    
            self.last_alignment_received = time.monotonic()
            break
    
    def on_timer(self):
        now = time.monotonic()
    
        localization_valid = (
            self.alignment_healthy
            and self.last_alignment_received is not None
            and now - self.last_alignment_received <= self.localization_timeout
        )
    
        command_valid = (
            self.last_command is not None
            and self.last_received is not None
            and now - self.last_received <= self.timeout
        )

        route_hold = self.get_parameter('route_hold').value
        route_limit = self.get_parameter('route_speed_limit').value

        if route_hold or not localization_valid or not command_valid:
            self.last_command = None
            self.last_received = None
            self.publisher.publish(Twist())
            return

        command = Twist()

        forward_limit = min(self.max_forward, route_limit)
        reverse_limit = min(self.max_reverse, route_limit)

        command.linear.x = min(
            forward_limit,
            max(-reverse_limit, self.last_command.linear.x),
        )

        command.angular.z = min(
            self.max_angular,
            max(-self.max_angular, self.last_command.angular.z),
        )

        self.publisher.publish(command)

    def on_parameter_change(self, params):
        for param in params:
            if param.name == 'route_speed_limit':
                value = param.value

                if (type(value) is not float
                        or not math.isfinite(value)
                        or not 0.0 < value <= self.max_forward):
                    return SetParametersResult(
                        successful=False,
                        reason='Invalid route speed limit',
                    )

            elif param.name == 'route_hold':
                if not isinstance(param.value, bool):
                    return SetParametersResult(
                        successful=False,
                        reason='route_hold must be boolean',
                    )

    return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = CmdVelWatchdog()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()