#!/usr/bin/env python3

from pathlib import Path
import sys
import time

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from route_watchdog_client import WatchdogClient


def main():
    rclpy.init()
    node = rclpy.create_node('route_watchdog_probe')
    watchdog = WatchdogClient(node, initial_limit=0.10)

    try:
        watchdog.start()
        node.get_logger().info('Watchdog session started in HOLD state')

        deadline = time.monotonic() + 30.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            watchdog.assert_healthy()

        node.get_logger().info('Communication test completed')

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if rclpy.ok():
                watchdog.close()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()