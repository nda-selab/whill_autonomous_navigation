#!/usr/bin/env python3

import os
import heapq
import json
import re
import shutil
import subprocess
import tempfile
import threading
import time
import yaml
import math
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from whill_msgs.msg import ModelCr2State
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ament_index_python.packages import get_package_share_directory


NAV2_LIFECYCLE_NODES = (
    'map_server',
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
)

class SpeechWorker:
    """Synthesize English speech with Piper and play it in a separate thread."""

    def __init__(self, logger, piper_python, model, rate, volume, queue_size, expiry_sec):
        self.logger = logger
        self.piper_python = piper_python
        self.model = model
        self.rate = rate
        self.volume = volume
        self.queue_size = queue_size
        self.expiry_sec = expiry_sec
        self.pending = []
        self.sequence = 0
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.thread = None

        model_path = Path(model)
        self.available = (
            Path(piper_python).is_file()
            and model_path.is_file()
            and Path(str(model_path) + '.json').is_file()
            and shutil.which('aplay') is not None
        )
        if not self.available:
            self.logger.error('Piper unavailable: verify Python, ONNX model, JSON and aplay')
            return

        self.thread = threading.Thread(target=self.run, daemon=True, name='speech_worker')
        self.thread.start()

    def say(self, message, valid_if=None, priority=1):
        if not self.available or self.stop_event.is_set() or not message:
            return

        with self.condition:
            if len(self.pending) >= self.queue_size:
                # Evict a lower-priority queued item to preserve critical alerts.
                worst = max(range(len(self.pending)), key=lambda i: self.pending[i][:2])
                if self.pending[worst][0] <= priority:
                    self.logger.warning(f'Speech queue full; dropped: {message}')
                    return
                self.logger.warning(f'Evicted queued speech: {self.pending[worst][3]}')
                self.pending.pop(worst)
                heapq.heapify(self.pending)

            self.sequence += 1
            heapq.heappush(
                self.pending, (priority, self.sequence, time.monotonic(), message, valid_if)
            )
            self.condition.notify()

    def run(self):
        with tempfile.TemporaryDirectory(prefix='whill_voice_') as directory:
            cache = {}
            while not self.stop_event.is_set():
                with self.condition:
                    while not self.pending and not self.stop_event.is_set():
                        self.condition.wait(timeout=0.2)
                    if self.stop_event.is_set():
                        break
                    priority, _, queued_at, message, valid_if = heapq.heappop(self.pending)

                try:
                    if time.monotonic() - queued_at > self.expiry_sec:
                        self.logger.warning(f'Announcement expired: {message}')
                        continue
                    if valid_if is not None and not valid_if():
                        continue

                    wav_path = cache.get(message)
                    if wav_path is None:
                        wav_path = os.path.join(directory, f'voice_{time.monotonic_ns()}.wav')
                        subprocess.run(
                            [self.piper_python, '-m', 'piper', '-m', self.model,
                             '--length-scale', str(1.0 / self.rate),
                             '--volume', str(self.volume), '-f', wav_path, '--', message],
                            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                            timeout=20, check=True,
                        )
                        # Keep the cache bounded for routes with many unique waypoint IDs.
                        if len(cache) >= 64:
                            old_path = cache.pop(next(iter(cache)))
                            try:
                                os.unlink(old_path)
                            except OSError:
                                pass
                        cache[message] = wav_path

                    # The queue prioritizes pending messages; playback already in progress
                    # is not interrupted. Navigation safety is never dependent on audio.
                    subprocess.run(
                        ['aplay', '-q', wav_path],
                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                        timeout=30, check=True,
                    )
                except (OSError, subprocess.SubprocessError) as error:
                    self.logger.error(f'Audio synthesis/playback failed: {error}')

    def close(self):
        self.stop_event.set()
        with self.condition:
            self.condition.notify_all()
        if self.thread is not None:
            self.thread.join(timeout=1.0)


class SystemStatusAnnouncer(Node):

    def __init__(self):
        super().__init__('system_status_announcer')

        self.declare_parameter('piper_python', '/home/selab/whill_tts_venv/bin/python')
        self.declare_parameter('piper_model', '/home/selab/whill_tts_voices/en_US-amy-medium.onnx')
        self.declare_parameter('speech_rate', 1.0)
        self.declare_parameter('speech_volume', 1.0)
        self.declare_parameter('speech_queue_size', 12)
        self.declare_parameter('speech_expiry_sec', 7.0)
        self.declare_parameter('sensor_timeout_sec', 4.0)
        self.declare_parameter('odom_timeout_sec', 3.0)
        self.declare_parameter('localization_timeout_sec', 5.0)
        self.declare_parameter('nav2_timeout_sec', 4.0)
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('messages_file', '')

        configured_path = self.get_parameter('messages_file').value
        default_path = (
            Path(get_package_share_directory('whill_nav2_bringup'))
            / 'config' / 'announcement_messages.yaml'
        )
        messages_path = Path(configured_path) if configured_path else default_path

        with messages_path.open('r', encoding='utf-8') as file:
            messages = yaml.safe_load(file)

        required_feeds = (
            'whill', 'ouster', 'urg_front', 'urg_left',
            'urg_right', 'odom', 'scan_ouster',
        )

        self.feed_messages = {
            name: tuple(
                messages['feeds'][name][state]
                for state in ('started', 'lost', 'recovered')
            )
            for name in required_feeds
        }
        self.event_messages = messages['events']
        self.waypoint_messages = messages['waypoints']
        self.last_waypoint_seq = {}

        def param(name):
            return self.get_parameter(name).value

        self.sensor_timeout = float(param('sensor_timeout_sec'))
        self.odom_timeout = float(param('odom_timeout_sec'))
        self.localization_timeout = float(
            param('localization_timeout_sec')
        )
        self.nav2_timeout = float(param('nav2_timeout_sec'))
        self.check_period = float(param('check_period_sec'))
        speech_rate = float(param('speech_rate'))
        speech_volume = float(param('speech_volume'))

        if min(
            self.sensor_timeout,
            self.odom_timeout,
            self.localization_timeout,
            self.nav2_timeout,
            self.check_period,
            speech_rate,
            speech_volume,
        ) <= 0.0:
            raise ValueError('Timeouts and speech rate must be positive')

        self.announcement_pub = self.create_publisher(
            String,
            '/system/announcement',
            10,
        )

        queue_size = int(param('speech_queue_size'))
        expiry_sec = float(param('speech_expiry_sec'))

        if queue_size <= 0 or not math.isfinite(expiry_sec) or expiry_sec <= 0:
            raise ValueError('Invalid speech queue configuration')

        self.speech = SpeechWorker(
            self.get_logger(),
            param('piper_python'),
            param('piper_model'),
            speech_rate,
            speech_volume,
            queue_size,
            expiry_sec,
        )

        self.last_messages = {
            name: None for name in self.feed_messages
        }
        self.feed_states = {
            name: 'waiting' for name in self.feed_messages
        }

        self.whill_power_ok = False
        self.whill_power_announced = None

        self.last_status = ''
        self.last_status_time = None
        self.last_accept = None

        self.ever_localized = False
        self.localized = False
        self.asked_initial_pose = False
        self.nav_active = False
        self.system_ready = False

        # -------------------------------------------------
        # Subscriptions
        # -------------------------------------------------

        self.monitor_subscriptions = [
            self.create_subscription(
                ModelCr2State,
                '/whill/states/model_cr2',
                self.on_whill,
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                PointCloud2,
                '/ouster/points',
                lambda msg: self.mark('ouster'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                LaserScan,
                '/urg_front/scan',
                lambda msg: self.mark('urg_front'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                LaserScan,
                '/urg_left/scan',
                lambda msg: self.mark('urg_left'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                LaserScan,
                '/urg_right/scan',
                lambda msg: self.mark('urg_right'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                '/odom_nav',
                lambda msg: self.mark('odom'),
                10,
            ),
            self.create_subscription(
                LaserScan,
                '/scan_ouster',
                lambda msg: self.mark('scan_ouster'),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                DiagnosticArray,
                '/alignment_status',
                self.on_localization_status,
                10,
            ),
            self.create_subscription(
                String,
                '/navigation/waypoint_event',
                self.on_waypoint_event,
                10,
            ),
        ]

        # -------------------------------------------------
        # Nav2 lifecycle monitoring
        # -------------------------------------------------

        self.lifecycle_clients = {
            name: self.create_client(
                GetState,
                f'/{name}/get_state',
            )
            for name in NAV2_LIFECYCLE_NODES
        }

        self.active_updates = {
            name: None for name in NAV2_LIFECYCLE_NODES
        }
        self.pending_requests = {}
        self.pending_since = {}

        # A steady clock remains usable when /clock is paused.
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self.timer = self.create_timer(
            self.check_period,
            self.check_status,
            clock=steady_clock,
        )

        self.announce_event('system_start')

    # -----------------------------------------------------
    # Announcements
    # -----------------------------------------------------

    def announce(self, message, valid_if=None, priority=1):
        self.get_logger().info(f'Announcement: {message}')
        self.announcement_pub.publish(String(data=message))
        self.speech.say(message, valid_if, priority=priority)

    def announce_event(self, key, valid_if=None):
        message = self.event_messages[key]
        if message:
            urgent = key in ('whill_error', 'localization_lost', 'navigation_error')
            self.announce(message, valid_if, priority=0 if urgent else 1)

    def on_waypoint_event(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('Waypoint event is not an object')
            mission_id = data.get('mission_id')
            sequence = data.get('sequence')
            event = data.get('event')
            if not isinstance(mission_id, str) or not mission_id or type(sequence) is not int:
                raise ValueError('Invalid mission ID or sequence')
            if event not in self.waypoint_messages:
                raise ValueError(f'Unknown waypoint event: {event}')
            last = self.last_waypoint_seq.get(mission_id, 0)
            if sequence <= last:
                return  # Ignore duplicate/replayed event IDs.
            self.last_waypoint_seq[mission_id] = sequence
            if len(self.last_waypoint_seq) > 32:
                self.last_waypoint_seq.pop(next(iter(self.last_waypoint_seq)))

            waypoint_id = str(data.get('waypoint_id', ''))
            match = re.fullmatch(r'wp0*(\d+)', waypoint_id, flags=re.IGNORECASE)
            spoken_id = str(int(match.group(1))) if match else waypoint_id.replace('_', ' ')
            template = self.waypoint_messages[event]
            message = template.replace('{waypoint}', spoken_id)
            priority = 0 if event in ('aborted', 'safety_warning') else (2 if event in ('reached', 'skipped', 'retrying') else 1)
            self.announce(message, priority=priority)
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f'Invalid waypoint event: {error}')

    def mark(self, name):
        self.last_messages[name] = time.monotonic()

    @staticmethod
    def is_fresh(stamp, now, timeout):
        return (
            stamp is not None
            and 0 <= now - stamp <= timeout
        )

    # -----------------------------------------------------
    # WHILL
    # -----------------------------------------------------

    def on_whill(self, msg):
        self.mark('whill')

        self.whill_power_ok = (
            bool(msg.power_on)
            and int(msg.error) == 0
        )

    # -----------------------------------------------------
    # GICP localization
    # -----------------------------------------------------

    def on_localization_status(self, msg):
        for status in msg.status:
            if status.name != 'lidar_localization_ros2/alignment':
                continue

            now = time.monotonic()
            values = {item.key: item.value for item in status.values}
            self.last_status_time = now

            if values.get('map_received') == 'false':
                self.last_status = 'WAIT_MAP'
                self.last_accept = None

            elif values.get('initialpose_received') == 'false':
                self.last_status = 'WAIT_INITIALPOSE'
                self.last_accept = None

            elif (
                status.level == DiagnosticStatus.OK
                and status.message == 'ok'
                and values.get('has_converged') == 'true'
            ):
                self.last_status = 'ACCEPT'
                self.last_accept = now

            else:
                self.last_status = status.message
                # 失敗時は成功時刻を更新しない。
                # 既存のタイムアウト判定で喪失を通知する。

            break

    # -----------------------------------------------------
    # Nav2 lifecycle
    # -----------------------------------------------------

    def on_lifecycle_state(self, name, future):
        if self.pending_requests.get(name) is not future:
            return

        self.pending_requests.pop(name, None)
        self.pending_since.pop(name, None)

        try:
            result = future.result()

            if (
                result is not None
                and result.current_state.id
                == State.PRIMARY_STATE_ACTIVE
            ):
                self.active_updates[name] = time.monotonic()
            else:
                self.active_updates[name] = None

        except Exception as error:
            self.active_updates[name] = None
            self.get_logger().debug(
                f'Lifecycle query failed for {name}: {error}'
            )

    def poll_lifecycle(self, now):
        for name, client in self.lifecycle_clients.items():

            pending = self.pending_requests.get(name)

            if pending is not None:
                if (
                    now - self.pending_since[name]
                    <= self.nav2_timeout
                ):
                    continue

                self.pending_requests.pop(name, None)
                self.pending_since.pop(name, None)
                self.active_updates[name] = None
                pending.cancel()

            if not client.service_is_ready():
                self.active_updates[name] = None
                continue

            try:
                future = client.call_async(GetState.Request())

                self.pending_requests[name] = future
                self.pending_since[name] = now

                future.add_done_callback(
                    lambda f, node_name=name:
                    self.on_lifecycle_state(node_name, f)
                )

            except Exception as error:
                self.active_updates[name] = None
                self.get_logger().debug(
                    f'Lifecycle request failed: {error}'
                )

    # -----------------------------------------------------
    # Periodic monitoring
    # -----------------------------------------------------

    def check_status(self):
        now = time.monotonic()

        self.poll_lifecycle(now)

        # Sensor and odometry freshness.
        for name, messages in self.feed_messages.items():
            started, lost, recovered = messages

            timeout = (
                self.odom_timeout
                if name == 'odom'
                else self.sensor_timeout
            )

            fresh = self.is_fresh(
                self.last_messages[name],
                now,
                timeout,
            )

            previous = self.feed_states[name]

            if fresh:
                state = 'ok'
            elif previous in ('ok', 'lost'):
                state = 'lost'
            else:
                state = 'waiting'

            if previous == state:
                continue

            self.feed_states[name] = state

            if started is None:
                continue

            if previous == 'waiting' and state == 'ok':
                self.announce(started)

            elif previous == 'ok' and state == 'lost':
                self.announce(lost, priority=0)

            elif previous == 'lost' and state == 'ok':
                self.announce(recovered)

        # WHILL power and error state.
        if self.feed_states['whill'] == 'ok':
            if self.whill_power_ok != self.whill_power_announced:

                if self.whill_power_ok:
                    self.announce_event('whill_ready')
                else:
                    self.announce_event('whill_error')

                self.whill_power_announced = self.whill_power_ok

        # Initial pose request.
        if (
            self.last_status.startswith('WAIT_INITIALPOSE')
            and self.feed_states['odom'] == 'ok'
        ):
            if not self.asked_initial_pose:
                self.announce_event('initial_pose_required')
                self.asked_initial_pose = True

        elif (
            self.last_status
            == 'INITIALPOSE_ACCEPTED_WAIT_REGISTRATION'
            or self.last_status.startswith('ACCEPT')
        ):
            self.asked_initial_pose = False

        # GICP localization freshness.
        localized_now = (
            self.is_fresh(
                self.last_accept,
                now,
                self.localization_timeout,
            )
            and self.is_fresh(
                self.last_status_time,
                now,
                self.localization_timeout,
            )
        )

        if localized_now and not self.localized:
            if self.ever_localized:
                self.announce_event('localization_recovered')
            else:
                self.announce_event('localization_ready')

            self.ever_localized = True

        elif self.localized and not localized_now:
            self.announce_event('localization_lost')

        self.localized = localized_now

        # Nav2 lifecycle state.
        nav_now = all(
            self.is_fresh(
                self.active_updates[name],
                now,
                self.nav2_timeout,
            )
            for name in NAV2_LIFECYCLE_NODES
        )

        if nav_now and not self.nav_active:
            self.announce_event('navigation_ready')

        elif self.nav_active and not nav_now:
            self.announce_event('navigation_error')

        self.nav_active = nav_now

        # Combined system readiness.
        ready = (
            self.whill_power_ok
            and all(
                state == 'ok'
                for state in self.feed_states.values()
            )
            and self.localized
            and self.nav_active
        )

        was_ready = self.system_ready
        self.system_ready = ready

        if ready and not was_ready:
            self.announce_event(
                'system_ready',
                valid_if=lambda: self.system_ready,
            )

    def close(self):
        self.speech.close()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = SystemStatusAnnouncer()
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