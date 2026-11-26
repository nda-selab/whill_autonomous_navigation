#!/usr/bin/env python3
"""Convert an ordered trajectory PLY into an editable WHILL waypoint YAML."""
import argparse
import math
import os
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
from plyfile import PlyData

from waypoint_route_core import MAX_WAYPOINTS, Route, Waypoint, save_route


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', type=Path, required=True, help='Input PLY (ordered vertices)')
    parser.add_argument('--map-yaml', type=Path, required=True, help='Nav2 occupancy map YAML')
    parser.add_argument('--output', type=Path, required=True, help='Output waypoint YAML')
    parser.add_argument('--waypoint-spacing', type=float, default=2.0, help='Regular spacing [m]')
    parser.add_argument('--raw-min-step', type=float, default=0.05, help='Minimum raw XY movement [m]')
    parser.add_argument('--corner-angle-deg', type=float, default=20.0, help='Corner threshold [deg]')
    parser.add_argument('--corner-window', type=float, default=0.5, help='Arc-length window on each side [m]')
    parser.add_argument('--corner-min-spacing', type=float, default=0.5, help='Minimum corner spacing [m]')
    parser.add_argument('--start-skip-distance', type=float, default=0.3, help='Skip initial route distance [m]')
    parser.add_argument('--start-index', type=int, default=0, help='First vertex index (inclusive)')
    parser.add_argument('--end-index', type=int, default=-1, help='Last vertex index (inclusive; -1 = last)')
    parser.add_argument('--reverse', action='store_true', help='Reverse selected trajectory')
    parser.add_argument('--overwrite', action='store_true', help='Allow replacing existing route YAML')
    args = parser.parse_args()

    for name in ('waypoint_spacing', 'raw_min_step', 'corner_window', 'corner_min_spacing'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f'--{name.replace("_", "-")} must be finite and positive')
    if not 0.0 < args.corner_angle_deg <= 180.0:
        parser.error('--corner-angle-deg must be in (0, 180]')
    if not math.isfinite(args.start_skip_distance) or args.start_skip_distance < 0.0:
        parser.error('--start-skip-distance must be finite and non-negative')
    if args.corner_min_spacing > args.waypoint_spacing:
        parser.error('--corner-min-spacing must not exceed --waypoint-spacing')
    return args


def load_trajectory(path, start_index, end_index, reverse):
    vertices = PlyData.read(str(path))['vertex'].data
    if not {'x', 'y', 'z'} <= set(vertices.dtype.names or ()):
        raise ValueError('PLY must contain vertex x, y, z properties')
    end_index = len(vertices) - 1 if end_index == -1 else end_index
    if not 0 <= start_index < end_index < len(vertices):
        raise ValueError('Invalid vertex range: at least two vertices are required')

    vertices = vertices[start_index:end_index + 1]
    xyz = np.column_stack([vertices[k] for k in ('x', 'y', 'z')]).astype(float)
    if not np.isfinite(xyz).all():
        raise ValueError('Selected trajectory contains non-finite coordinates')
    xy = xyz[:, :2]
    return xy[::-1].copy() if reverse else xy.copy()


def remove_stationary_points(xy, min_step):
    indices = [0]
    for i in range(1, len(xy) - 1):
        if np.linalg.norm(xy[i] - xy[indices[-1]]) >= min_step:
            indices.append(i)
    # Preserve the true endpoint, even if the last step is short.
    if np.linalg.norm(xy[-1] - xy[indices[-1]]) > 1e-9:
        indices.append(len(xy) - 1)
    if len(indices) < 2:
        raise ValueError('Trajectory has fewer than two distinct retained XY points')
    return xy[indices]


def cumulative_distance(xy):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]


def find_corners(xy, distances, window, threshold_deg, min_spacing):
    # Estimate headings over a physical distance rather than adjacent raw samples.
    angles = np.zeros(len(xy))
    for i in range(1, len(xy) - 1):
        if distances[i] < window or distances[-1] - distances[i] < window:
            continue
        left = np.searchsorted(distances, distances[i] - window, side='right') - 1
        right = np.searchsorted(distances, distances[i] + window, side='left')
        right = min(right, len(xy) - 1)
        incoming, outgoing = xy[i] - xy[left], xy[right] - xy[i]
        norm = np.linalg.norm(incoming) * np.linalg.norm(outgoing)
        if norm > 1e-12:
            cosine = np.clip(np.dot(incoming, outgoing) / norm, -1.0, 1.0)
            angles[i] = np.degrees(np.arccos(cosine))

    # Keep the strongest corner in each contiguous above-threshold region.
    candidates = np.flatnonzero(angles >= threshold_deg)
    if not len(candidates):
        return []
    groups = np.split(candidates, np.flatnonzero(np.diff(candidates) > 1) + 1)
    peaks = []
    for group in groups:
        peak = angles[group].max()
        tied = group[np.isclose(angles[group], peak, rtol=0.0, atol=1e-6)]
        center = (distances[tied[0]] + distances[tied[-1]]) / 2.0
        peaks.append(int(tied[np.argmin(np.abs(distances[tied] - center))]))
    corners = []
    for i in peaks:
        if not corners or distances[i] - distances[corners[-1]] >= min_spacing:
            corners.append(i)
        elif angles[i] > angles[corners[-1]]:
            corners[-1] = i
    return corners


def select_waypoints(xy, distances, corners, spacing, start_skip):
    # Preserve corners and endpoint; add regular samples between those anchors.
    anchors = [0, *corners, len(xy) - 1]
    selected = [0]
    for start, end in zip(anchors[:-1], anchors[1:]):
        for i in range(start + 1, end):
            if distances[i] - distances[selected[-1]] >= spacing:
                selected.append(i)
        selected.append(end)

    # Use traveled distance, not distance from the initial XY coordinate.
    # This preserves later visits near the start on a closed-loop route.
    selected = [i for i in selected if i != 0 and (distances[i] >= start_skip or i == anchors[-1])]
    goals = []
    for i in selected:
        if goals and np.linalg.norm(xy[i] - xy[goals[-1]]) <= 1e-9:
            goals[-1] = i
        else:
            goals.append(i)
    return xy[goals]


def write_route(output, route, overwrite):
    if overwrite:
        save_route(output, route)
        return
    # Atomic no-clobber publication, including if another process creates the file.
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix='.waypoints-', dir=output.parent) as directory:
        temporary = Path(directory) / 'route.yaml'
        save_route(temporary, route)
        os.link(temporary, output)


def main():
    args = parse_args()
    try:
        trajectory = args.trajectory.expanduser().resolve(strict=True)
        map_yaml = args.map_yaml.expanduser().resolve(strict=True)
        output = args.output.expanduser().resolve()
        if not trajectory.is_file() or trajectory.suffix.lower() != '.ply':
            raise ValueError('Input trajectory must be a .ply file')
        if not map_yaml.is_file():
            raise ValueError('map-yaml must be a file')
        if output.suffix.lower() != '.yaml' or output in (trajectory, map_yaml):
            raise ValueError('Output must be a separate .yaml file')
        if output.exists() and not args.overwrite:
            raise FileExistsError(f'{output} already exists; use another path or --overwrite')

        raw = load_trajectory(trajectory, args.start_index, args.end_index, args.reverse)
        xy = remove_stationary_points(raw, args.raw_min_step)
        distances = cumulative_distance(xy)
        corners = find_corners(xy, distances, args.corner_window, args.corner_angle_deg, args.corner_min_spacing)
        goals = select_waypoints(xy, distances, corners, args.waypoint_spacing, args.start_skip_distance)
        if len(goals) > MAX_WAYPOINTS:
            raise ValueError(f'{len(goals)} waypoints exceed limit {MAX_WAYPOINTS}; increase spacing or split the route')

        waypoints = [Waypoint(id=f'wp{i + 1:03d}', x=float(x), y=float(y),
                              skippable=i < len(goals) - 1, yaw=None, orientation_required=False)
                     for i, (x, y) in enumerate(goals)]
        route = Route(frame_id='map', map_yaml=str(map_yaml), waypoints=waypoints)
        route.validate(require_goals=True)
        write_route(output, route, args.overwrite)
        print(f'Raw vertices: {len(raw)} | Filtered: {len(xy)} | Corners: {len(corners)}')
        print(f'XY length: {distances[-1]:.2f} m | Waypoints: {len(goals)}')
        print(f'Saved: {output}\nDraft only: verify map alignment and route safety in RViz.')
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
