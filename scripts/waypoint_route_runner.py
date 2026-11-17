#!/usr/bin/env python3
"""Validate a saved waypoint route; issue Nav2 actions only with --execute.

Position-only intermediate goals; optional required yaw is handled as a separate
Nav2 Spin action after the XY target has been reached. No forced motion on load.
"""
import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from trajectory_waypoint_navigator import load_grid_map, load_footprint
from waypoint_route_core import load_route, route_heading, filter_unsafe_route


def yaw_quaternion(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def make_pose(nav, w, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = w.x
    pose.pose.position.y = w.y
    pose.pose.orientation.z, pose.pose.orientation.w = yaw_quaternion(yaw)
    return pose


class UrgHealthGate:
    """Prevent continuing the route when either processed vertical URG is stale/bad."""
    def __init__(self, navigator, timeout=1.5):
        self.last_at = None
        self.levels = {}
        self.timeout = timeout
        self.subscription = navigator.create_subscription(
            DiagnosticArray, '/urg_obstacle_filter/status', self.callback, 10)

    def callback(self, msg):
        self.last_at = time.monotonic()
        self.levels = {
            status.name.rsplit('/', 1)[-1]: status.level
            for status in msg.status
            if status.name in ('urg_obstacle_filter/left', 'urg_obstacle_filter/right')
        }

    def healthy(self):
        return (self.last_at is not None and time.monotonic() - self.last_at <= self.timeout
                and set(self.levels) == {'left', 'right'}
                and all(level == DiagnosticStatus.OK for level in self.levels.values()))


def wait_for_gate(nav, gate, max_wait=10.0):
    deadline = time.monotonic() + max_wait
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.1)
        if gate.healthy():
            return
    raise RuntimeError('URG filter status is missing/ERROR; route execution blocked')


def cancel_and_confirm(nav):
    nav.cancelTask()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if nav.isTaskComplete():
            return True
    return False


def wait_nav_task(nav, gate, timeout):
    started = time.monotonic()
    while not nav.isTaskComplete():
        if not gate.healthy():
            if not cancel_and_confirm(nav):
                raise RuntimeError('URG safety error: cancellation not confirmed')
            raise RuntimeError('URG filter unhealthy; navigation canceled, not skipped')
        if time.monotonic() - started > timeout:
            if not cancel_and_confirm(nav):
                raise RuntimeError('Goal timeout: cancellation not confirmed')
            return 'timeout'
    return 'succeeded' if nav.getResult() == TaskResult.SUCCEEDED else 'failed'


def robot_yaw(tf_buffer):
    transform = tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.4))
    q = transform.transform.rotation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def normalized_angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def ensure_heading(nav, tf_buffer, gate, target_yaw, tolerance, timeout):
    current = robot_yaw(tf_buffer)
    delta = normalized_angle(target_yaw - current)
    if abs(delta) <= tolerance:
        return True
    if not gate.healthy():
        raise RuntimeError('URG status unhealthy before yaw alignment')
    if not nav.spin(spin_dist=delta, time_allowance=math.ceil(timeout)):
        return False
    if wait_nav_task(nav, gate, timeout) != 'succeeded':
        return False
    return abs(normalized_angle(target_yaw - robot_yaw(tf_buffer))) <= tolerance


def check_position_goal_checker(params_file):
    with Path(params_file).open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    params = data['controller_server']['ros__parameters']
    checker_names = params['goal_checker_plugins']
    if len(checker_names) != 1 or params[checker_names[0]]['plugin'] != 'nav2_controller::PositionGoalChecker':
        raise ValueError('Execution requires controller_server PositionGoalChecker for yaw-free waypoints')


def parse_args():
    parser = argparse.ArgumentParser(description='Safe map-based waypoint route execution')
    parser.add_argument('--route', type=Path, required=True, help='Saved YAML route')
    parser.add_argument('--map-yaml', type=Path, required=True, help='Static occupancy map YAML')
    parser.add_argument('--nav2-params', type=Path, required=True, help='Nav2 parameters YAML')
    parser.add_argument('--clearance', type=float, default=0.05, help='Additional robot footprint clearance [m]')
    parser.add_argument('--plan-preview', action='store_true', help='Ask Nav2 Planner for each segment, no motion')
    parser.add_argument('--execute', action='store_true', help='Actually send NavigateToPose and Spin goals')
    parser.add_argument('--max-goal-time', type=float, default=120.0, help='Per-goal timeout [s]')
    parser.add_argument('--max-retries', type=int, default=1, help='Additional attempts for optional goals')
    parser.add_argument('--yaw-tolerance-deg', type=float, default=15.0, help='Required terminal yaw tolerance [deg]')
    parser.add_argument('--results-file', type=Path, help='Execution result log YAML')
    args = parser.parse_args()
    if args.max_goal_time <= 0 or not math.isfinite(args.max_goal_time) or args.max_retries < 0 or args.max_retries > 5:
        parser.error('Invalid goal timeout or retry count')
    if args.clearance < 0 or not math.isfinite(args.clearance):
        parser.error('Invalid clearance')
    if not 0.0 < args.yaw_tolerance_deg <= 180:
        parser.error('Invalid yaw tolerance')
    return args


def main():
    args = parse_args()
    map_file = args.map_yaml.expanduser().resolve()
    params_file = args.nav2_params.expanduser().resolve()
    route = load_route(args.route, str(map_file))
    route.validate(require_goals=True)
    grid = load_grid_map(map_file)
    footprint = load_footprint(params_file)
    filtered, skipped = filter_unsafe_route(route, grid, footprint, args.clearance)
    print(f'Route: {len(route.waypoints)} points; map-safe: {len(filtered.waypoints)}; skipped: {len(skipped)}')
    for entry in skipped:
        print(f'  SKIP {entry["id"]}: {entry["reason"]}')
    for i, wp in enumerate(filtered.waypoints):
        print(f'  {i+1}. {wp.id}: ({wp.x:.2f},{wp.y:.2f}), required={not wp.skippable}, yaw_required={wp.orientation_required}')
    if not args.plan_preview and not args.execute:
        print('OFFLINE PREVIEW ONLY. Add --plan-preview to query Nav2, --execute to move the robot.')
        return 0
    if args.execute:
        check_position_goal_checker(params_file)
    rclpy.init()
    nav = BasicNavigator(node_name='waypoint_route_runner')
    gate = UrgHealthGate(nav) if args.execute else None
    tf_buffer = Buffer() if args.execute else None
    tf_listener = TransformListener(tf_buffer, nav, spin_thread=False) if args.execute else None
    results = [{'id': s['id'], 'status': 'skipped', 'reason': s['reason']} for s in skipped]
    try:
        nav.waitUntilNav2Active(localizer='map_server')
        if args.execute:
            wait_for_gate(nav, gate)
        # Preflight all map-safe segments. If an optional segment is not plannable,
        # remove its target and plan the next segment from the last accepted pose.
        accepted = []
        previous = None
        for i, w in enumerate(filtered.waypoints):
            pose = make_pose(nav, w, route_heading(filtered, i))
            planned = nav.getPath(previous if previous else pose, pose, use_start=(previous is not None))
            if planned is None or not planned.poses:
                if not w.skippable:
                    raise RuntimeError(f'No valid plan to required waypoint {w.id}; mission aborted')
                results.append({'id': w.id, 'status': 'skipped', 'reason': 'preflight_planner'})
                print(f'SKIP {w.id}: planner returned no path')
                continue
            accepted.append((w, pose))
            previous = pose
        if not accepted or accepted[-1][0].skippable:
            raise RuntimeError('No reachable required final goal')
        print(f'Preflight complete: {len(accepted)} reachable waypoints')
        if not args.execute:
            print('PLAN PREVIEW ONLY: no movement commands sent')
            return 0

        for w, pose in accepted:
            if not gate.healthy():
                raise RuntimeError('URG diagnostics unhealthy; mission aborted')
            # Update pose stamp and re-plan from live pose before each goal.
            pose.header.stamp = nav.get_clock().now().to_msg()
            planned = nav.getPath(pose, pose, use_start=False)
            if planned is None or not planned.poses:
                if w.skippable:
                    results.append({'id': w.id, 'status': 'skipped', 'reason': 'live_planner'})
                    continue
                raise RuntimeError(f'Live path unavailable to required {w.id}')
            state = 'failed'
            for attempt in range(args.max_retries + 1):
                if not gate.healthy():
                    raise RuntimeError('URG diagnostics unhealthy; mission aborted')
                pose.header.stamp = nav.get_clock().now().to_msg()
                if not nav.goToPose(pose):
                    state = 'rejected'
                    break
                state = wait_nav_task(nav, gate, args.max_goal_time)
                print(f'{w.id}: attempt {attempt+1} -> {state}')
                if state == 'succeeded':
                    break
            if state == 'succeeded' and w.orientation_required:
                try:
                    rotated = ensure_heading(nav, tf_buffer, gate, w.yaw,
                                             math.radians(args.yaw_tolerance_deg), 30.0)
                except TransformException as error:
                    rotated = False
                    print(f'Yaw verification unavailable: {error}', file=sys.stderr)
                if not rotated:
                    state = 'yaw_failed'
            if state == 'succeeded':
                results.append({'id': w.id, 'status': 'reached'})
            elif w.skippable:
                results.append({'id': w.id, 'status': 'skipped', 'reason': state})
            else:
                raise RuntimeError(f'Required waypoint {w.id} failed: {state}')
        print('Mission complete; required goal reached.')
        return 0
    except KeyboardInterrupt:
        print('Interrupted; canceling navigation', file=sys.stderr)
        if args.execute and not cancel_and_confirm(nav):
            print('WARNING: navigation cancellation not confirmed', file=sys.stderr)
        return 130
    except (RuntimeError, TransformException) as error:
        print(f'ABORTED: {error}', file=sys.stderr)
        results.append({'id': 'mission', 'status': 'aborted', 'reason': str(error)})
        return 1
    finally:
        if args.execute:
            log = args.results_file or args.route.expanduser().with_suffix('.results.yaml')
            report = {'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                      'route': str(args.route), 'results': results}
            log = Path(log).expanduser().resolve()
            # Result file must never overwrite route or source map.
            if log not in {args.route.expanduser().resolve(), map_file}:
                log.parent.mkdir(parents=True, exist_ok=True)
                with log.open('w', encoding='utf-8') as file:
                    yaml.safe_dump(report, file, sort_keys=False, allow_unicode=True)
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
