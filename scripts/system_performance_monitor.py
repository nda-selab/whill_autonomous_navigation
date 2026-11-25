#!/usr/bin/env python3
"""Lightweight, observation-only performance monitor for WHILL Nav2."""
import csv
import json
import math
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import psutil
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String


PROCESS_PATTERNS = {
    'controller_server': ('controller_server',),
    'planner_server': ('planner_server',),
    'bt_navigator': ('bt_navigator',),
    'fast_lio2': ('laser_mapping', 'lasermapping', 'fastlio_mapping', 'fast_lio'),
    'lidar_localization': ('lidar_localization_node',),
    'ouster_driver': ('os_sensor', 'ouster_sensor', 'ouster_driver'),
    'ouster_obstacle_filter': ('ouster_obstacle_filter.py',),
    'urg_front_obstacle_filter': ('urg_front_obstacle_filter.py',),
    'urg_side_obstacle_filter': ('urg_side_obstacle_filter.py',),
    'cmd_vel_watchdog': ('cmd_vel_watchdog.py',),
}

CSV_FIELDS = [
    'stamp_utc', 'cpu_pct', 'max_core_pct', 'memory_pct', 'memory_used_gib',
    'controller_cpu_pct', 'controller_rss_mib',
    'planner_cpu_pct', 'planner_rss_mib',
    'fast_lio2_cpu_pct', 'fast_lio2_rss_mib',
    'localization_cpu_pct', 'localization_rss_mib',
    'cmd_rate_hz', 'cmd_max_gap_ms', 'cmd_age_ms',
    'odom_rate_hz', 'odom_max_gap_ms', 'odom_age_ms',
]


def process_category(proc_info):
    """Identify common standalone ROS executables, not the ros2 launch process."""
    cmdline = proc_info.get('cmdline') or []
    candidates = [Path(proc_info.get('name') or '').name.lower()]
    candidates += [Path(arg).name.lower() for arg in cmdline[:2]]
    for category, patterns in PROCESS_PATTERNS.items():
        if any(pattern in candidate for pattern in patterns for candidate in candidates):
            return category
    return None


class TopicIntervals:
    def __init__(self):
        self.times = deque(maxlen=500)

    def received(self):
        self.times.append(time.monotonic())

    def snapshot(self, now, window_sec):
        recent = [t for t in self.times if now - t <= window_sec]
        age = (now - self.times[-1]) * 1000.0 if self.times else None
        if len(recent) < 2:
            return {'rate_hz': None, 'max_gap_ms': None, 'age_ms': age}
        gaps = [b - a for a, b in zip(recent, recent[1:])]
        span = recent[-1] - recent[0]
        return {
            'rate_hz': round((len(recent) - 1) / span, 2) if span > 0 else None,
            'max_gap_ms': round(max(gaps) * 1000.0, 1),
            'age_ms': round(age, 1) if age is not None else None,
        }


class SystemPerformanceMonitor(Node):
    def __init__(self):
        super().__init__('system_performance_monitor')
        self.declare_parameter('sample_period_sec', 1.0)
        self.declare_parameter('process_refresh_sec', 5.0)
        self.declare_parameter('topic_window_sec', 5.0)
        self.declare_parameter('enable_csv', True)
        self.declare_parameter('log_directory', '~/.ros/whill_performance')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('odom_topic', '/odom_nav')

        self.sample_period = float(self.get_parameter('sample_period_sec').value)
        self.refresh_period = float(self.get_parameter('process_refresh_sec').value)
        self.topic_window = float(self.get_parameter('topic_window_sec').value)
        if not all(math.isfinite(x) and x > 0 for x in
                   (self.sample_period, self.refresh_period, self.topic_window)):
            raise ValueError('Monitoring periods must be positive and finite')

        self.cmd_intervals = TopicIntervals()
        self.odom_intervals = TopicIntervals()
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            lambda _msg: self.cmd_intervals.received(), 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            lambda _msg: self.odom_intervals.received(), 10)
        self.publisher = self.create_publisher(String, '/system/performance', 10)

        self.processes = {}
        self.last_refresh = -float('inf')
        psutil.cpu_percent(interval=None, percpu=True)  # Prime CPU percentage counters.
        self.refresh_processes()

        self.csv_file = None
        self.csv_writer = None
        self.samples_since_flush = 0
        if bool(self.get_parameter('enable_csv').value):
            folder = Path(self.get_parameter('log_directory').value).expanduser()
            folder.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            self.csv_file = (folder / f'performance_{stamp}.csv').open(
                'w', newline='', encoding='utf-8')
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            self.csv_writer.writeheader()
            self.get_logger().info(f'CSV logging: {self.csv_file.name}')

        # STEADY_TIME continues sampling even when simulated /clock is paused.
        self.timer = self.create_timer(
            self.sample_period, self.sample,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def refresh_processes(self):
        selected = {name: {} for name in PROCESS_PATTERNS}
        for proc in psutil.process_iter(attrs=['pid', 'name', 'cmdline']):
            try:
                category = process_category(proc.info)
                if category is None:
                    continue
                existing = self.processes.get(category, {}).get(proc.pid)
                if existing is None:
                    proc.cpu_percent(interval=None)  # Initial reading is not meaningful.
                    existing = proc
                selected[category][proc.pid] = existing
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
        self.processes = selected
        self.last_refresh = time.monotonic()

    def process_snapshot(self):
        result = {}
        for category, processes in self.processes.items():
            cpu, rss, count = 0.0, 0, 0
            for proc in processes.values():
                try:
                    cpu += proc.cpu_percent(interval=None)
                    rss += proc.memory_info().rss
                    count += 1
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    continue
            result[category] = {
                'count': count,
                'cpu_pct': round(cpu, 1) if count else None,
                'rss_mib': round(rss / (1024 ** 2), 1) if count else None,
            }
        return result

    def sample(self):
        now = time.monotonic()
        if now - self.last_refresh >= self.refresh_period:
            self.refresh_processes()

        per_core = psutil.cpu_percent(interval=None, percpu=True)
        memory = psutil.virtual_memory()
        cmd = self.cmd_intervals.snapshot(now, self.topic_window)
        odom = self.odom_intervals.snapshot(now, self.topic_window)
        processes = self.process_snapshot()
        timestamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')

        system = {
            'cpu_pct': round(sum(per_core) / len(per_core), 1) if per_core else None,
            'max_core_pct': round(max(per_core), 1) if per_core else None,
            'memory_pct': round(memory.percent, 1),
            'memory_used_gib': round(memory.used / (1024 ** 3), 2),
        }
        snapshot = {
            'stamp_utc': timestamp,
            'system': system,
            'processes': processes,
            'topics': {'cmd_vel_nav': cmd, 'odom_nav': odom},
        }
        self.publisher.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))

        if self.csv_writer is not None:
            controller = processes['controller_server']
            planner = processes['planner_server']
            fast_lio = processes['fast_lio2']
            localization = processes['lidar_localization']
            row = {
                'stamp_utc': timestamp,
                **system,
                'controller_cpu_pct': controller['cpu_pct'],
                'controller_rss_mib': controller['rss_mib'],
                'planner_cpu_pct': planner['cpu_pct'],
                'planner_rss_mib': planner['rss_mib'],
                'fast_lio2_cpu_pct': fast_lio['cpu_pct'],
                'fast_lio2_rss_mib': fast_lio['rss_mib'],
                'localization_cpu_pct': localization['cpu_pct'],
                'localization_rss_mib': localization['rss_mib'],
                'cmd_rate_hz': cmd['rate_hz'],
                'cmd_max_gap_ms': cmd['max_gap_ms'],
                'cmd_age_ms': cmd['age_ms'],
                'odom_rate_hz': odom['rate_hz'],
                'odom_max_gap_ms': odom['max_gap_ms'],
                'odom_age_ms': odom['age_ms'],
            }
            self.csv_writer.writerow({key: row.get(key) for key in CSV_FIELDS})
            self.samples_since_flush += 1
            if self.samples_since_flush >= 10:
                self.csv_file.flush()
                self.samples_since_flush = 0

    def close(self):
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SystemPerformanceMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
