#!/usr/bin/env python3
"""RViz2 waypoint editor: Publish Point to add; interactive markers to edit.

No movement/actions are ever sent from this node. Saving requires an explicit
service or interactive-marker menu operation.
"""
import copy
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from trajectory_waypoint_navigator import load_grid_map, load_footprint
from waypoint_route_core import Route, load_route, route_heading, save_route


def yaw_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class WaypointEditor(Node):
    def __init__(self):
        super().__init__('waypoint_editor')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('nav2_params_file', '')
        self.declare_parameter('clearance', 0.05)
        self.map_yaml = Path(self.get_parameter('map_yaml').value).expanduser().resolve()
        self.file = Path(self.get_parameter('waypoints_file').value).expanduser().resolve()
        params_path = Path(self.get_parameter('nav2_params_file').value).expanduser().resolve()
        self.clearance = float(self.get_parameter('clearance').value)
        if not self.map_yaml.is_file() or not params_path.is_file():
            raise ValueError('map_yaml and nav2_params_file must exist')
        if self.file == self.map_yaml or self.file.suffix.lower() != '.yaml':
            raise ValueError('waypoints_file must be a separate .yaml file')
        if not math.isfinite(self.clearance) or self.clearance < 0:
            raise ValueError('clearance must be finite and non-negative')
        self.grid = load_grid_map(self.map_yaml)
        self.footprint = load_footprint(params_path)
        self.route = load_route(self.file, str(self.map_yaml)) if self.file.is_file() else Route(map_yaml=str(self.map_yaml))
        self.undo_stack = []
        self.server = InteractiveMarkerServer(self, 'waypoint_editor')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(NavPath, '/waypoint_editor/path', qos)
        self.status_pub = self.create_publisher(String, '/waypoint_editor/status', qos)
        self.scan_sub = self.create_subscription(PointStamped, '/clicked_point', self.on_clicked, 10)
        self.create_service(Trigger, '/waypoint_editor/save', self.on_save)
        self.create_service(Trigger, '/waypoint_editor/reload', self.on_reload)
        self.create_service(Trigger, '/waypoint_editor/undo', self.on_undo)
        self.create_service(Trigger, '/waypoint_editor/clear', self.on_clear)
        self.refresh()
        self.get_logger().info('RViz waypoint editor ready: add via Publish Point (Fixed Frame=map)')

    def remember(self):
        self.undo_stack.append(copy.deepcopy(self.route))
        self.undo_stack = self.undo_stack[-30:]

    def safety(self):
        messages = {}
        for i, waypoint in enumerate(self.route.waypoints):
            yaw = route_heading(self.route, i)
            safe, reason = self.grid.clearance_check(
                waypoint.x, waypoint.y, yaw, self.footprint, self.clearance, True)
            if not safe:
                messages[waypoint.id] = reason
        return messages

    def on_clicked(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warning(f'Ignored click in {msg.header.frame_id!r}, set RViz Fixed Frame to map')
            return
        if not (math.isfinite(msg.point.x) and math.isfinite(msg.point.y)):
            return
        try:
            self.remember()
            previous_final_id = (self.route.waypoints[-1].id if self.route.waypoints and
                                 not self.route.waypoints[-1].skippable else None)
            new = self.route.add(msg.point.x, msg.point.y)
            if previous_final_id:
                self.route.shift(new.id, -1)
            self.refresh()
            self.get_logger().info(f'Added {new.id}: ({new.x:.2f}, {new.y:.2f}) (not saved)')
        except ValueError as error:
            self.get_logger().error(str(error))

    def on_save(self, request, response):
        try:
            # Saving drafts is allowed; route runner always validates before executing.
            save_route(self.file, self.route)
            response.success = True
            response.message = f'Saved {len(self.route.waypoints)} waypoints to {self.file}'
        except (OSError, ValueError) as error:
            response.success = False
            response.message = str(error)
        self.get_logger().info(response.message)
        return response

    def on_reload(self, request, response):
        try:
            route = load_route(self.file, str(self.map_yaml))
            self.remember()
            self.route = route
            self.refresh()
            response.success = True
            response.message = f'Reloaded {len(route.waypoints)} waypoints'
        except (OSError, ValueError, TypeError) as error:
            response.success = False
            response.message = str(error)
        return response

    def on_undo(self, request, response):
        if self.undo_stack:
            self.route = self.undo_stack.pop()
            self.refresh()
            response.success = True
            response.message = 'Reverted last edit (not saved)'
        else:
            response.success = False
            response.message = 'Nothing to undo'
        return response

    def on_clear(self, request, response):
        self.remember()
        self.route.waypoints.clear()
        self.refresh()
        response.success = True
        response.message = 'Cleared draft (saved file unchanged)'
        return response

    def on_menu(self, feedback, operation):
        id = feedback.marker_name.removeprefix('wp_')
        if id not in {w.id for w in self.route.waypoints}:
            return
        try:
            self.remember()
            if operation == 'delete':
                self.route.remove(id)
            elif operation == 'up':
                self.route.shift(id, -1)
            elif operation == 'down':
                self.route.shift(id, 1)
            elif operation == 'skippable':
                w = self.route.waypoints[self.route.index(id)]
                self.route.update(id, skippable=not w.skippable)
            elif operation == 'final':
                w = self.route.waypoints.pop(self.route.index(id))
                w.skippable = False
                self.route.waypoints.append(w)
            elif operation == 'yaw':
                i = self.route.index(id)
                w = self.route.waypoints[i]
                if w.orientation_required:
                    self.route.update(id, orientation_required=False, yaw=None)
                else:
                    self.route.update(id, orientation_required=True,
                                      yaw=route_heading(self.route, i))
            elif operation.startswith('wait:'):
                seconds = float(operation.split(':')[1])
                changes = {'wait_time_sec': seconds}
                if seconds > 0.0:
                    changes['skippable'] = False
                self.route.update(id, **changes)
            elif operation.startswith('speed:'):
                value = operation.split(':')[1]
                speed = None if value == 'None' else float(value)
                changes = {'speed_limit_mps': speed}
                if speed is not None:
                    changes['skippable'] = False
                self.route.update(id, **changes)
            elif operation == 'save':
                self.undo_stack.pop()
                self.on_save(None, Trigger.Response())
            elif operation == 'reload':
                self.undo_stack.pop()
                self.on_reload(None, Trigger.Response())
                return
            self.refresh()
        except (ValueError, IndexError) as error:
            self.get_logger().error(str(error))

    def on_marker_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        id = feedback.marker_name.removeprefix('wp_')
        if id not in {w.id for w in self.route.waypoints}:
            return
        pos = feedback.pose.position
        if not all(math.isfinite(n) for n in (pos.x, pos.y)):
            self.refresh()
            return
        self.remember()
        current = self.route.waypoints[self.route.index(id)]
        changes = {'x': float(pos.x), 'y': float(pos.y)}
        if current.orientation_required:
            changes['yaw'] = quaternion_yaw(feedback.pose.orientation)
        self.route.update(id, **changes)
        self.refresh()

    def marker(self, w, number, invalid):
        im = InteractiveMarker()
        im.header.frame_id = 'map'
        im.name = 'wp_' + w.id
        im.description = f'{number}. {w.id} | {"optional" if w.skippable else "required"}'
        im.scale = 0.9
        im.pose.position.x = w.x
        im.pose.position.y = w.y
        im.pose.orientation = yaw_quaternion(w.yaw if w.yaw is not None else 0.0)
        control = InteractiveMarkerControl()
        control.name = 'drag_xy'
        control.description = 'Drag in map XY plane'
        control.orientation.w = 0.70710678
        control.orientation.y = -0.70710678  # Rotate control X axis to +Z, so MOVE_PLANE is XY.
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        ball = Marker()
        ball.type = Marker.CYLINDER
        ball.scale.x = 0.28
        ball.scale.y = 0.28
        ball.scale.z = 0.06
        if invalid:
            ball.color.r, ball.color.g, ball.color.b = 0.95, 0.15, 0.12
        elif w.wait_time_sec > 0.0:
            ball.color.r, ball.color.g, ball.color.b = 0.55, 0.25, 0.85
        elif w.speed_limit_mps is not None:
            ball.color.r, ball.color.g, ball.color.b = 0.95, 0.60, 0.05
        elif w.skippable:
            ball.color.r, ball.color.g, ball.color.b = 0.08, 0.65, 0.52
        else:
            ball.color.r, ball.color.g, ball.color.b = 0.15, 0.35, 0.9
        ball.color.a = 0.9
        ball.pose.orientation.w = 1.0
        control.markers.append(ball)
        label = Marker()
        label.type = Marker.TEXT_VIEW_FACING
        labels = [str(number)]
        if w.wait_time_sec > 0.0:
            labels.append(f'WAIT {w.wait_time_sec:g}s')
        if w.speed_limit_mps is not None:
            labels.append(f'{w.speed_limit_mps:.2f}m/s')
        label.text = '\n'.join(labels)
        label.scale.z = 0.22
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.pose.position.z = 0.22
        label.pose.orientation.w = 1.0
        control.markers.append(label)
        im.controls.append(control)
        if w.orientation_required:
            rotate = InteractiveMarkerControl()
            rotate.name = 'rotate_yaw'
            rotate.description = 'Drag to change required yaw'
            rotate.orientation.w = 0.70710678
            rotate.orientation.y = -0.70710678
            rotate.orientation_mode = InteractiveMarkerControl.FIXED
            rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            im.controls.append(rotate)
            arrow = InteractiveMarkerControl()
            arrow.name = 'heading_indicator'
            arrow.always_visible = True
            heading = Marker()
            heading.type = Marker.ARROW
            heading.pose.position.z = 0.15
            heading.pose.orientation.w = 1.0
            heading.scale.x, heading.scale.y, heading.scale.z = 0.40, 0.08, 0.08
            heading.color.r, heading.color.g, heading.color.b, heading.color.a = 0.9, 0.5, 0.0, 0.9
            arrow.markers.append(heading)
            im.controls.append(arrow)
        return im

    def refresh(self):
        invalid = self.safety()
        self.server.clear()
        self.menu = MenuHandler()
        for name, action in (
            ('Move earlier', 'up'), ('Move later', 'down'), ('Delete', 'delete'),
            ('Toggle skippable / required', 'skippable'),
            ('Make final goal (required)', 'final'),
            ('Toggle fixed yaw / any yaw', 'yaw'),
            ('Save YAML', 'save'), ('Reload saved YAML', 'reload'),
        ):
            self.menu.insert(name, callback=lambda feedback, a=action: self.on_menu(feedback, a))
        wait_menu = self.menu.insert('Wait time')
        for seconds in (0, 3, 5, 10, 30, 60):
            self.menu.insert(
                f'{seconds} s', parent=wait_menu,
                callback=lambda fb, v=seconds: self.on_menu(fb, f'wait:{v}'),
            )
        speed_menu = self.menu.insert('Speed limit')
        for speed in (None, 0.05, 0.10, 0.15, 0.20, 0.25):
            label = 'Inherit previous limit' if speed is None else f'{speed:.2f} m/s'
            self.menu.insert(
                label, parent=speed_menu,
                callback=lambda fb, v=speed: self.on_menu(fb, f'speed:{v}'),
            )
        for i, w in enumerate(self.route.waypoints):
            self.server.insert(self.marker(w, i + 1, w.id in invalid),
                               feedback_callback=self.on_marker_feedback)
            self.menu.apply(self.server, 'wp_' + w.id)
        self.server.applyChanges()
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i, w in enumerate(self.route.waypoints):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = w.x
            pose.pose.position.y = w.y
            pose.pose.orientation = yaw_quaternion(route_heading(self.route, i))
            path.poses.append(pose)
        self.path_pub.publish(path)
        final_ok = bool(self.route.waypoints and not self.route.waypoints[-1].skippable)
        summary = (f'{len(path.poses)} waypoint(s), invalid={len(invalid)}, '
                   f'final_required={final_ok}, unsaved edits may exist')
        if invalid:
            summary += '; invalid: ' + ', '.join(f'{id}: {reason}' for id, reason in invalid.items())
        self.status_pub.publish(String(data=summary))
        self.get_logger().info(summary)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointEditor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
