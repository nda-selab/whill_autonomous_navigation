#!/usr/bin/env python3
"""Pure Python data model and safe YAML persistence for editable WHILL routes."""
from dataclasses import dataclass, field, replace
from copy import deepcopy
from pathlib import Path
import math
import os
import re
import tempfile
import yaml

_ID = re.compile(r'^[A-Za-z][A-Za-z0-9_-]{0,47}$')
MAX_WAYPOINTS = 1000


@dataclass
class Waypoint:
    id: str
    x: float
    y: float
    skippable: bool = True
    yaw: float | None = None
    orientation_required: bool = False

    def validate(self):
        if not isinstance(self.id, str) or not _ID.fullmatch(self.id):
            raise ValueError(f'Invalid waypoint ID: {self.id!r}')
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (self.x, self.y)):
            raise ValueError(f'Waypoint {self.id}: coordinates must be finite')
        if not isinstance(self.skippable, bool) or not isinstance(self.orientation_required, bool):
            raise ValueError(f'Waypoint {self.id}: boolean property required')
        if self.yaw is not None and (not isinstance(self.yaw, (int, float)) or not math.isfinite(self.yaw)):
            raise ValueError(f'Waypoint {self.id}: yaw must be finite or null')
        if self.orientation_required and self.yaw is None:
            raise ValueError(f'Waypoint {self.id}: orientation_required needs yaw')


@dataclass
class Route:
    frame_id: str = 'map'
    map_yaml: str = ''
    waypoints: list[Waypoint] = field(default_factory=list)

    def validate(self, require_goals=False):
        if self.frame_id != 'map':
            raise ValueError('Only map frame routes are supported')
        if not isinstance(self.map_yaml, str) or not self.map_yaml:
            raise ValueError('map_yaml must refer to the original occupancy map')
        if len(self.waypoints) > MAX_WAYPOINTS:
            raise ValueError(f'Too many waypoints (max {MAX_WAYPOINTS})')
        if require_goals and (not self.waypoints or self.waypoints[-1].skippable):
            raise ValueError('Route needs a non-skippable final waypoint')
        ids = set()
        for w in self.waypoints:
            w.validate()
            if w.id in ids:
                raise ValueError(f'Duplicate waypoint ID: {w.id}')
            ids.add(w.id)
        return self

    def add(self, x: float, y: float):
        if len(self.waypoints) >= MAX_WAYPOINTS:
            raise ValueError('Waypoint limit reached')
        number = 1
        while f'wp{number:03d}' in {w.id for w in self.waypoints}:
            number += 1
        waypoint = Waypoint(f'wp{number:03d}', float(x), float(y))
        waypoint.validate()
        self.waypoints.append(waypoint)
        return waypoint

    def index(self, id: str):
        for i, waypoint in enumerate(self.waypoints):
            if waypoint.id == id:
                return i
        raise ValueError(f'Unknown waypoint: {id}')

    def remove(self, id: str):
        self.waypoints.pop(self.index(id))

    def shift(self, id: str, offset: int):
        i = self.index(id)
        j = i + offset
        if not (0 <= j < len(self.waypoints)):
            return False
        self.waypoints[i], self.waypoints[j] = self.waypoints[j], self.waypoints[i]
        return True

    def update(self, id: str, **changes):
        i = self.index(id)
        updated = replace(self.waypoints[i], **changes)
        updated.validate()
        self.waypoints[i] = updated
        return updated


def route_heading(route: Route, index: int):
    w = route.waypoints[index]
    if w.orientation_required and w.yaw is not None:
        return w.yaw
    if index + 1 < len(route.waypoints):
        other = route.waypoints[index + 1]
    elif index > 0:
        other = w
        w = route.waypoints[index - 1]
    else:
        return w.yaw if w.yaw is not None else 0.0
    dx, dy = other.x - w.x, other.y - w.y
    return math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0


def to_dict(route: Route):
    route.validate()
    return {
        'version': 1,
        'frame_id': route.frame_id,
        'map_yaml': route.map_yaml,
        'waypoints': [
            {'id': w.id, 'x': w.x, 'y': w.y, 'skippable': w.skippable,
             'yaw': w.yaw, 'orientation_required': w.orientation_required}
            for w in route.waypoints
        ],
    }


def from_dict(data, expected_map_yaml: str):
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('Expected waypoint route version 1')
    if not isinstance(data.get('waypoints'), list):
        raise ValueError('Missing waypoints array')
    allowed = {'id', 'x', 'y', 'skippable', 'yaw', 'orientation_required'}
    waypoints = []
    for row in data['waypoints']:
        if not isinstance(row, dict) or not set(row) <= allowed:
            raise ValueError('Invalid waypoint record')
        normalized = dict(row)
        for key in ('x', 'y', 'yaw'):
            value = normalized.get(key)
            if key == 'yaw' and value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'Waypoint {normalized.get("id")}: invalid {key}')
            try:
                value = float(value)
            except (ValueError, OverflowError) as error:
                raise ValueError(f'Waypoint {normalized.get("id")}: invalid {key}') from error
            if not math.isfinite(value):
                raise ValueError(f'Waypoint {normalized.get("id")}: non-finite {key}')
            normalized[key] = value
        waypoints.append(Waypoint(**normalized))
    route = Route(frame_id=data.get('frame_id', ''), map_yaml=str(data.get('map_yaml', '')), waypoints=waypoints)
    route.validate()
    actual = Path(route.map_yaml).expanduser().resolve()
    expected = Path(expected_map_yaml).expanduser().resolve()
    if actual != expected:
        raise ValueError(f'Route map mismatch: {actual} != {expected}')
    return route


def load_route(file: Path, map_yaml: str):
    with Path(file).expanduser().open('r', encoding='utf-8') as stream:
        return from_dict(yaml.safe_load(stream), map_yaml)


def save_route(file: Path, route: Route):
    """Atomic save. Route can be a draft; execution must revalidate each goal."""
    dest = Path(file).expanduser().resolve()
    if dest == Path(route.map_yaml).expanduser().resolve() or dest.suffix.lower() != '.yaml':
        raise ValueError('Waypoint file must be a separate .yaml file, not map.yaml')
    doc = to_dict(route)
    dest.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=dest.parent,
                                         prefix=f'.{dest.stem}.', suffix='.tmp', delete=False) as stream:
            name = stream.name
            yaml.safe_dump(doc, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, dest)
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


def filter_unsafe_route(route, grid, footprint, clearance):
    """Drop unsafe optional goals, checking adjusted tangent headings each round."""
    result = deepcopy(route)
    removed = []
    while result.waypoints:
        unsafe = []
        for i, wp in enumerate(result.waypoints):
            yaw = route_heading(result, i)
            safe, reason = grid.clearance_check(wp.x, wp.y, yaw, footprint, clearance, True)
            if not safe:
                if not wp.skippable:
                    raise RuntimeError(f'Required waypoint {wp.id} is unsafe: {reason}')
                unsafe.append((wp.id, reason))
        if not unsafe:
            break
        for id, reason in unsafe:
            result.remove(id)
            removed.append({'id': id, 'reason': 'static_map: ' + reason})
    result.validate(require_goals=True)
    return result, removed

