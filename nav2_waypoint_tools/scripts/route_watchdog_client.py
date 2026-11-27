#!/usr/bin/env python3

import math
import time
import uuid

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically


class WatchdogClient:

    def __init__(self, node, initial_limit,
                 update_period=0.1, response_timeout=0.5):

        self.node = node
        self.session = uuid.uuid4().hex
        self.sequence = -1

        self.speed_limit = float(initial_limit)
        self.hold = True
        self.active = False
        self.updating = False
        self.fault = None

        self.response_timeout = response_timeout
        self.pending = None
        self.sent_at = None
        self.timer = None

        if not math.isfinite(self.speed_limit) or self.speed_limit <= 0.0:
            raise ValueError('Invalid initial speed limit')

        self.client = node.create_client(
            SetParametersAtomically,
            '/cmd_vel_watchdog/set_parameters_atomically',
        )

        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.update_period = update_period

    def _send(self):
        if self.pending is not None:
            raise RuntimeError('Previous watchdog request is pending')

        if self.fault is not None:
            raise RuntimeError(self.fault)

        self.sequence += 1

        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter('route_session', value=self.session).to_parameter_msg(),
            Parameter('route_sequence', value=self.sequence).to_parameter_msg(),
            Parameter('route_speed_limit', value=self.speed_limit).to_parameter_msg(),
            Parameter('route_hold', value=self.hold).to_parameter_msg(),
        ]

        self.sent_at = time.monotonic()
        try:
            self.pending = self.client.call_async(request)
        except Exception as error:
            self.fault = f'Watchdog request failed: {error}'
            raise RuntimeError(self.fault) from error

    def _poll(self):
        if self.pending is None:
            return

        if time.monotonic() - self.sent_at > self.response_timeout:
            self.fault = 'Watchdog response timeout'
            self.pending = None
            return

        if self.pending.done():
            try:
                response = self.pending.result()

                if response is None:
                    self.fault = 'Watchdog returned no response'
                elif not response.result.successful:
                    self.fault = response.result.reason

            except Exception as error:
                self.fault = f'Watchdog communication failed: {error}'

            self.pending = None

    def _wait_response(self):
        while rclpy.ok() and self.pending is not None:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            self._poll()

        if not rclpy.ok():
            raise RuntimeError('ROS shutdown during watchdog request')

        self.assert_healthy()

    def _tick(self):
        if not self.active or self.updating:
            return

        self._poll()

        if self.fault is not None:
            return

        if self.pending is None:
            try:
                self._send()
            except Exception as error:
                self.fault = str(error)

    def assert_healthy(self):
        if self.fault is not None:
            raise RuntimeError(self.fault)

    def start(self):
        if self.active or self.sequence != -1:
            raise RuntimeError('Watchdog session already started')

        if not self.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('Watchdog service unavailable')

        # New sessions always start with a stop hold
        self.hold = True
        self._send()
        self._wait_response()

        self.active = True
        self.timer = self.node.create_timer(
            self.update_period, self._tick, clock=self.clock,
        )

    def set_state(self, *, speed_limit=None, hold=None):
        if not self.active:
            raise RuntimeError('Watchdog session is not active')

        self.assert_healthy()
        self.updating = True

        try:
            self._wait_response()

            if speed_limit is not None:
                value = float(speed_limit)
                if not math.isfinite(value) or value <= 0.0:
                    raise ValueError('Invalid route speed limit')
                self.speed_limit = value

            if hold is not None:
                if type(hold) is not bool:
                    raise ValueError('hold must be boolean')
                self.hold = hold

            self._send()
            self._wait_response()

        finally:
            self.updating = False

    def set_hold(self, hold):
        self.set_state(hold=hold)

    def set_speed_limit(self, speed_limit):
        self.set_state(speed_limit=speed_limit)

    def close(self):
        if not self.active:
            return

        try:
            self.set_hold(True)
        finally:
            self.active = False
            if self.timer is not None:
                self.timer.cancel()