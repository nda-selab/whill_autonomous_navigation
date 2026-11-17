#!/usr/bin/env python3
"""Pure NumPy geometry for vertical URG scans (no ROS imports)."""
from dataclasses import dataclass
from itertools import combinations
import numpy as np


@dataclass(frozen=True)
class GroundConfig:
    floor_fit_x_min: float = -0.05
    floor_fit_x_max: float = 0.45
    floor_fit_z_abs_max: float = 0.18
    floor_inlier_tolerance: float = 0.025
    floor_min_inliers: int = 8
    floor_min_span: float = 0.15
    floor_anchor_x_max: float = 0.20
    floor_anchor_min_inliers: int = 4
    floor_max_slope: float = 0.25
    floor_max_intercept_abs: float = 0.12
    obstacle_x_min: float = -0.05
    obstacle_x_max: float = 1.50
    obstacle_min_height: float = 0.05
    obstacle_max_height: float = 0.40
    min_cluster_points: int = 2


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """Return valid LaserScan beam indices and laser-frame points."""
    distances = np.asarray(ranges, dtype=float)
    valid = np.isfinite(distances) & (distances >= range_min) & (distances <= range_max)
    indices = np.flatnonzero(valid)
    angles = angle_min + indices * angle_increment
    distances = distances[indices]
    points = np.column_stack((distances * np.cos(angles), distances * np.sin(angles), np.zeros(len(indices))))
    return indices, points


def quaternion_rotation(qx, qy, qz, qw):
    """Return a 3x3 rotation matrix for an (x, y, z, w) quaternion."""
    q = np.asarray([qx, qy, qz, qw], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def transform_points(points, translation, quaternion):
    return points @ quaternion_rotation(*quaternion).T + np.asarray(translation, dtype=float)


def fit_local_ground(points, cfg):
    """Robust near-robot linear ground z(x)=slope*x+intercept, or None.

    This is a limited-range baseline for one vertical scan plane; it is NOT a
    general terrain/step or negative-obstacle detector.
    """
    x, z = points[:, 0], points[:, 2]
    selected = (
        np.isfinite(x) & np.isfinite(z)
        & (x >= cfg.floor_fit_x_min) & (x <= cfg.floor_fit_x_max)
        & (np.abs(z) <= cfg.floor_fit_z_abs_max)
    )
    samples = points[selected]
    if len(samples) < cfg.floor_min_inliers:
        return None
    # Use x-spread candidates to avoid fitting only a dense near-origin cluster.
    samples = samples[np.argsort(samples[:, 0])]
    selected_ids = np.unique(np.linspace(0, len(samples) - 1, min(22, len(samples)), dtype=int))
    x, z = samples[:, 0], samples[:, 2]
    best_score = None
    best_inliers = None
    for i, j in combinations(selected_ids, 2):
        dx = x[j] - x[i]
        if dx < cfg.floor_min_span:
            continue
        slope = (z[j] - z[i]) / dx
        intercept = z[i] - slope * x[i]
        if abs(slope) > cfg.floor_max_slope or abs(intercept) > cfg.floor_max_intercept_abs:
            continue
        residual = np.abs(z - (slope * x + intercept))
        inliers = residual <= cfg.floor_inlier_tolerance
        count = int(inliers.sum())
        if (count < cfg.floor_min_inliers or np.ptp(x[inliers]) < cfg.floor_min_span
                or np.count_nonzero(inliers & (x <= cfg.floor_anchor_x_max)) < cfg.floor_anchor_min_inliers):
            continue
        anchor_count = int(np.count_nonzero(inliers & (x <= cfg.floor_anchor_x_max)))
        score = (anchor_count, count, -float(residual[inliers].mean()))
        if best_score is None or score > best_score:
            best_score, best_inliers = score, inliers
    if best_inliers is None:
        return None
    # Refine once on consensus floor points.
    slope, intercept = np.polyfit(x[best_inliers], z[best_inliers], 1)
    if abs(slope) > cfg.floor_max_slope or abs(intercept) > cfg.floor_max_intercept_abs:
        return None
    return float(slope), float(intercept)


def classify_points(indices, points, ground_model, cfg):
    """Return (obstacles, floor) in base_link; group contiguous obstacle beams."""
    slope, intercept = ground_model
    relative_height = points[:, 2] - (slope * points[:, 0] + intercept)
    in_range = (points[:, 0] >= cfg.obstacle_x_min) & (points[:, 0] <= cfg.obstacle_x_max)
    obstacles = in_range & (relative_height >= cfg.obstacle_min_height) & (relative_height <= cfg.obstacle_max_height)
    floor = in_range & (np.abs(relative_height) <= cfg.floor_inlier_tolerance)
    selected = np.flatnonzero(obstacles)
    keep = np.zeros(len(points), dtype=bool)
    if len(selected):
        groups = np.split(selected, np.flatnonzero(np.diff(indices[selected]) != 1) + 1)
        for group in groups:
            if len(group) >= cfg.min_cluster_points:
                keep[group] = True
    return points[keep], points[floor]
