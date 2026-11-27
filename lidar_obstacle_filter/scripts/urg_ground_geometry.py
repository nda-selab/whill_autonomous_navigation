#!/usr/bin/env python3
"""Near-field geometry for two vertically mounted URG sensors (no ROS imports)."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ObstacleConfig:
    obstacle_x_min: float = -0.05
    obstacle_x_max: float = 1.20
    obstacle_min_height: float = 0.05
    obstacle_max_height: float = 0.40
    min_cluster_points: int = 2
    clearing_min_height: float = -0.05
    clearing_max_height: float = 0.05


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Return valid LaserScan beam indices and laser-frame XYZ points."""
    distances = np.asarray(ranges, dtype=float)
    valid = np.isfinite(distances) & (distances >= range_min) & (distances <= range_max)
    indices = np.flatnonzero(valid)
    angles = angle_min + indices * angle_increment
    distances = distances[indices]
    points = np.column_stack((distances * np.cos(angles), distances * np.sin(angles), np.zeros(len(indices))))
    return indices, points


def quaternion_rotation(qx, qy, qz, qw):
    q = np.asarray([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z-w*x)],
        [2 * (x*z-y*w), 2 * (y*z+x*w), 1 - 2 * (x*x+y*y)],
    ])


def transform_points(points, translation, quaternion):
    return points @ quaternion_rotation(*quaternion).T + np.asarray(translation, dtype=float)


def _cluster_mask(indices, candidate, min_cluster_points):
    """Accept only adjacent original laser beams, not adjacent filtered array slots."""
    selected = np.flatnonzero(candidate)
    keep = np.zeros(len(indices), dtype=bool)
    if len(selected):
        groups = np.split(selected, np.flatnonzero(np.diff(indices[selected]) != 1) + 1)
        for group in groups:
            if len(group) >= min_cluster_points:
                keep[group] = True
    return keep


def classify_near_field(indices, points, cfg):
    """Return (obstacles, clearing), or None if no valid near-field points."""
    if len(points) == 0:
        return None
    x, z = points[:, 0], points[:, 2]
    region = (
        np.isfinite(x) & np.isfinite(z)
        & (x >= cfg.obstacle_x_min)
        & (x <= cfg.obstacle_x_max)
    )
    indices, points = indices[region], points[region]
    if len(points) == 0:
        return None
    z = points[:, 2]
    candidates = (
        (z >= cfg.obstacle_min_height)
        & (z <= cfg.obstacle_max_height)
    )
    obstacles = _cluster_mask(
        indices, candidates, cfg.min_cluster_points
    )
    low_returns = (
        (z >= cfg.clearing_min_height)
        & (z < cfg.clearing_max_height)
    )
    clearing = low_returns | obstacles
    return points[obstacles], points[clearing]
