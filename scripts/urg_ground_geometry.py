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
    floor_track_window: float = 0.25
    floor_track_max_gap: float = 0.12
    floor_track_max_step: float = 0.02


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
    """Track locally continuous ground and classify protrusions."""
    x, z = points[:, 0], points[:, 2]
    in_range = (
        np.isfinite(x) & np.isfinite(z)
        & (x >= cfg.obstacle_x_min) & (x <= cfg.obstacle_x_max)
    )

    slope, intercept = ground_model
    baseline = slope * x + intercept
    floor = (
        in_range & (x <= cfg.floor_fit_x_max)
        & (np.abs(z - baseline) <= cfg.floor_inlier_tolerance)
    )

    accepted = np.flatnonzero(floor).tolist()
    accepted.sort(key=lambda i: x[i])

    order = np.flatnonzero(in_range & (x > cfg.floor_fit_x_max))
    order = order[np.argsort(x[order], kind='stable')]
    last_line = np.array(ground_model, dtype=float)

    for i in order:
        if accepted:
            tail_x = x[accepted[-1]]
            recent = [
                j for j in accepted
                if x[j] >= tail_x - cfg.floor_track_window
            ]
            if len(recent) >= 3 and np.ptp(x[recent]) > 0.06:
                fitted = np.polyfit(x[recent], z[recent], 1)
                if abs(fitted[0]) <= cfg.floor_max_slope:
                    last_line = fitted

        predicted = last_line[0] * x[i] + last_line[1]
        baseline[i] = predicted

        if not accepted:
            continue

        dx = x[i] - x[accepted[-1]]
        if dx < 0 or dx > cfg.floor_track_max_gap:
            continue
        if abs(z[i] - predicted) > cfg.floor_inlier_tolerance:
            continue
        if abs(z[i] - z[accepted[-1]]) > (
            cfg.floor_max_slope * dx + cfg.floor_track_max_step
        ):
            continue

        floor[i] = True
        accepted.append(int(i))

    relative_height = z - baseline
    obstacles = (
        in_range & ~floor
        & (relative_height >= cfg.obstacle_min_height)
        & (relative_height <= cfg.obstacle_max_height)
    )

    selected = np.flatnonzero(obstacles)
    keep = np.zeros(len(points), dtype=bool)

    if len(selected):
        groups = np.split(
            selected,
            np.flatnonzero(np.diff(indices[selected]) != 1) + 1
        )
        for group in groups:
            if len(group) >= cfg.min_cluster_points:
                keep[group] = True

    return points[keep], points[floor]
