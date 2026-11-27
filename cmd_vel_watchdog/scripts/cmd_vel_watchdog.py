#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
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
        self.declare_parameter('require_route_control', False)
        self.declare_parameter('route_timeout_sec', 1.0)
        self.declare_parameter('route_session', '')
        self.declare_parameter('route_sequence', -1)

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

        route_limit = self.get_parameter('route_speed_limit').value
        if (type(route_limit) is not float
                or not math.isfinite(route_limit)
                or not 0.0 < route_limit <= self.max_forward):
            raise ValueError('Invalid initial route speed limit')

        self.last_received = None
        self.last_command = None
        self.last_alignment_received = None
        self.alignment_healthy = False

        self.require_route_control = self.get_parameter('require_route_control').value
        self.route_timeout = float(self.get_parameter('route_timeout_sec').value)

        if not math.isfinite(self.route_timeout) or self.route_timeout <= 0.0:
            raise ValueError('Invalid route timeout')

        if (self.get_parameter('route_session').value != ''
                or self.get_parameter('route_sequence').value != -1):
            raise ValueError('Invalid initial route session')

        self.route_update = None
        self.retired_sessions = set()
        self.route_expired_reported = False

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

        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.timer = self.create_timer(
            1.0 / frequency, self.on_timer,
            clock=self.steady_clock,
        )

        self.add_on_set_parameters_callback(
            self.on_parameter_change
        )

    def route_control_valid(self, now=None):
        if not self.require_route_control:
            return True

        if self.route_update is None:
            return False

        now = time.monotonic() if now is None else now
        session, sequence, received_at = self.route_update

        return (
            session == self.get_parameter('route_session').value
            and sequence == self.get_parameter('route_sequence').value
            and now - received_at <= self.route_timeout
        )

    def on_command(self, msg):
        if (self.get_parameter('route_hold').value
                or not self.route_control_valid()):
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
        route_valid = self.route_control_valid(now)

        if self.require_route_control and not route_valid:
            if not self.route_expired_reported:
                self.get_logger().warning('Route control unavailable or timed out')
                self.route_expired_reported = True
        else:
            self.route_expired_reported = False

        if route_hold or not route_valid or not localization_valid or not command_valid:
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
        updates = {p.name: p.value for p in params}

        fixed_params = {
            'input_topic', 'output_topic',
            'command_timeout_sec', 'localization_timeout_sec',
            'publish_frequency', 'max_forward_speed',
            'max_reverse_speed', 'max_angular_speed',
            'require_route_control', 'route_timeout_sec',
        }

        route_params = {
            'route_session', 'route_sequence',
            'route_speed_limit', 'route_hold',
        }

        def reject(reason):
            return SetParametersResult(successful=False, reason=reason)

        if fixed_params.intersection(updates):
            return reject('Fixed watchdog parameters cannot be changed at runtime')

        if 'route_speed_limit' in updates:
            value = updates['route_speed_limit']
            if (type(value) is not float
                    or not math.isfinite(value)
                    or not 0.0 < value <= self.max_forward):
                return reject('Invalid route speed limit')

        if 'route_hold' in updates and type(updates['route_hold']) is not bool:
            return reject('route_hold must be boolean')

        # Non-route parameters
        if not route_params.intersection(updates):
            return SetParametersResult(successful=True)

        # Legacy navigation mode
        if not self.require_route_control:
            if {'route_session', 'route_sequence'}.intersection(updates):
                return reject('Route communication monitoring is disabled')

            if 'route_hold' in updates:
                self.last_command = None
                self.last_received = None

            return SetParametersResult(successful=True)

        # All route parameters must be updated atomically
        if set(updates) != route_params or len(params) != len(route_params):
            return reject('All route parameters must be updated atomically')

        session = updates['route_session']
        sequence = updates['route_sequence']
        hold = updates['route_hold']
        now = time.monotonic()

        if not isinstance(session, str) or not session or len(session) > 64:
            return reject('Invalid route session')

        if type(sequence) is not int or sequence < 0:
            return reject('Invalid route sequence')

        current_session = self.get_parameter('route_session').value
        current_sequence = self.get_parameter('route_sequence').value

        if session == current_session:
            if not self.route_control_valid(now):
                return reject('Route session expired; start a new mission')

            if sequence != current_sequence + 1:
                return reject('Invalid route sequence')

        else:
            if session in self.retired_sessions:
                return reject('Retired route session')

            if self.route_control_valid(now):
                return reject('Another route session is active')

            if sequence != 0 or not hold:
                return reject('New session must start with sequence=0 and hold=true')

            if current_session:
                self.retired_sessions.add(current_session)

        # Discard cached commands at session/hold transitions
        if (session != current_session
                or hold != self.get_parameter('route_hold').value):
            self.last_command = None
            self.last_received = None

        # Only a committed matching session/sequence is valid
        self.route_update = (session, sequence, now)

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