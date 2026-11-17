#!/usr/bin/env python3
"""Idempotent local installer; does not touch GitHub or auto-launch WHILL.

Usage: python3 tools/apply_changes.py /path/to/whill_nav2_bringup
"""
from pathlib import Path
import shutil
import sys


def ensure_replace(text, old, new, file):
    if new in text:
        return text
    if old not in text:
        raise RuntimeError(f'{file}: expected anchor not found; do not modify automatically')
    return text.replace(old, new, 1)


def install(source, dest):
    if dest.exists() and dest.read_bytes() != source.read_bytes():
        raise RuntimeError(f'Existing target differs; backup/reconcile manually: {dest}')
    if not dest.exists():
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
    if dest.suffix == '.py' and dest.parent.name == 'scripts' and dest.name != 'waypoint_route_core.py':
        dest.chmod(dest.stat().st_mode | 0o111)
    print('installed', dest)


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: python3 tools/apply_changes.py /path/to/whill_nav2_bringup')
    root = Path(sys.argv[1]).expanduser().resolve()
    own = Path(__file__).resolve().parents[1]
    cmake = root / 'CMakeLists.txt'
    package = root / 'package.xml'
    nav = root / 'config/nav2_params.yaml'
    for file in (cmake, package, nav):
        if not file.is_file():
            raise RuntimeError(f'Missing existing repo file: {file}')
    files = {}
    c = cmake.read_text()
    c = ensure_replace(c, '  scripts/urg_obstacle_filter.py\n  DESTINATION lib/${PROJECT_NAME}',
                       '  scripts/urg_obstacle_filter.py\n  scripts/waypoint_editor.py\n'
                       '  scripts/waypoint_route_runner.py\n  DESTINATION lib/${PROJECT_NAME}', cmake)
    c = ensure_replace(c, '  scripts/urg_ground_geometry.py\n  DESTINATION lib/${PROJECT_NAME}',
                       '  scripts/urg_ground_geometry.py\n  scripts/waypoint_route_core.py\n'
                       '  DESTINATION lib/${PROJECT_NAME}', cmake)
    files[cmake] = c
    p = package.read_text()
    p = ensure_replace(p, '  <exec_depend>sensor_msgs_py</exec_depend>',
                       '  <exec_depend>sensor_msgs_py</exec_depend>\n'
                       '  <exec_depend>interactive_markers</exec_depend>\n'
                       '  <exec_depend>visualization_msgs</exec_depend>\n'
                       '  <exec_depend>std_srvs</exec_depend>', package)
    files[package] = p
    n = nav.read_text()
    n = ensure_replace(n, 'plugin: "nav2_controller::SimpleGoalChecker"',
                       'plugin: "nav2_controller::PositionGoalChecker"', nav)
    # yaw_goal_tolerance is not a PositionGoalChecker parameter.
    n = ensure_replace(n, '      yaw_goal_tolerance: 0.30\n', '', nav)
    files[nav] = n
    # Validate existing targets before touching any files.
    entries = [
        ('scripts/waypoint_route_core.py', 'scripts/waypoint_route_core.py'),
        ('scripts/waypoint_editor.py', 'scripts/waypoint_editor.py'),
        ('scripts/waypoint_route_runner.py', 'scripts/waypoint_route_runner.py'),
        ('launch/waypoint_editor.launch.py', 'launch/waypoint_editor.launch.py'),
    ]
    for src, dst in entries:
        target = root / dst
        if target.exists() and target.read_bytes() != (own / src).read_bytes():
            raise RuntimeError(f'Existing target differs; backup/reconcile manually: {target}')
    for src, dst in entries:
        install(own / src, root / dst)
    for file, content in files.items():
        if file.read_text() != content:
            file.write_text(content)
            print('updated', file)
        else:
            print('unchanged', file)
    print('Complete. Inspect git diff, run unit tests, then colcon build.')


if __name__ == '__main__':
    main()
