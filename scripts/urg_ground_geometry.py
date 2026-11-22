#!/usr/bin/env python3
"""Near-field geometry for two vertically mounted URG sensors (no ROS imports)."""
from collections import deque
from dataclasses import dataclass
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
    obstacle_x_max: float = 1.20
    obstacle_min_height: float = 0.05
    obstacle_max_height: float = 0.40
    min_cluster_points: int = 2
    floor_track_window: float = 0.15
    floor_track_max_gap: float = 0.12
    floor_track_max_step: float = 0.02


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


def _seed_floor(x, z, cfg):
    """Fit nearby floor once, then reject outliers and refit once (max two fits)."""
    near = np.flatnonzero(
        (x >= cfg.floor_fit_x_min) & (x <= cfg.floor_fit_x_max)
        & (np.abs(z) <= cfg.floor_fit_z_abs_max)
    )
    anchor = near[x[near] <= cfg.floor_anchor_x_max]
    if len(near) < cfg.floor_min_inliers or len(anchor) < cfg.floor_anchor_min_inliers:
        return None
    if abs(float(np.median(z[anchor]))) > cfg.obstacle_min_height:
        return None
    if np.ptp(x[near]) < cfg.floor_min_span:
        return None

    slope, intercept = np.polyfit(x[near], z[near], 1)
    residual = np.abs(z[near] - (slope * x[near] + intercept))
    inliers = near[residual <= cfg.floor_inlier_tolerance]
    if (len(inliers) < cfg.floor_min_inliers or np.ptp(x[inliers]) < cfg.floor_min_span
            or np.count_nonzero(x[inliers] <= cfg.floor_anchor_x_max) < cfg.floor_anchor_min_inliers):
        return None
    slope, intercept = np.polyfit(x[inliers], z[inliers], 1)
    if abs(slope) > cfg.floor_max_slope or abs(intercept) > cfg.floor_max_intercept_abs:
        return None
    # The second fit must still support the seed used for tracking.
    inliers = near[np.abs(z[near] - (slope * x[near] + intercept)) <= cfg.floor_inlier_tolerance]
    if (len(inliers) < cfg.floor_min_inliers or np.ptp(x[inliers]) < cfg.floor_min_span
            or np.count_nonzero(x[inliers] <= cfg.floor_anchor_x_max) < cfg.floor_anchor_min_inliers):
        return None
    return float(slope), float(intercept), inliers


def _lightweight_masks(x, z, cfg):
    model = _seed_floor(x, z, cfg)
    if model is None:
        return None
    slope, intercept, inliers = model
    floor = np.zeros(len(x), dtype=bool)
    floor[inliers] = True
    baseline = slope * x + intercept
    accepted = deque(sorted(inliers.tolist(), key=lambda i: x[i]))

    for i in np.argsort(x, kind='stable'):
        if x[i] <= cfg.floor_fit_x_max:
            continue
        if not accepted:
            break
        while len(accepted) > 2 and x[i] - x[accepted[1]] >= cfg.floor_track_window:
            accepted.popleft()
        anchor, last = accepted[0], accepted[-1]
        dx = x[i] - x[last]
        span = x[last] - x[anchor]
        local_slope = (z[last] - z[anchor]) / span if span >= 0.05 else slope
        local_slope = float(np.clip(local_slope, -cfg.floor_max_slope, cfg.floor_max_slope))
        predicted = z[last] + local_slope * dx
        baseline[i] = predicted
        if dx <= 0 or dx > cfg.floor_track_max_gap:
            continue
        if abs(z[i] - predicted) > cfg.floor_inlier_tolerance:
            continue
        if abs(z[i] - z[last]) > cfg.floor_max_slope * dx + cfg.floor_track_max_step:
            continue
        if abs(z[i] - z[anchor]) > cfg.floor_max_slope * (x[i] - x[anchor]) + cfg.floor_track_max_step:
            continue
        floor[i] = True
        accepted.append(int(i))

    obstacles = ~floor & (z - baseline >= cfg.obstacle_min_height) & (z - baseline <= cfg.obstacle_max_height)
    return floor, obstacles


def classify_near_field(indices, points, cfg, enable_ground_filter=True):
    """Return (obstacles, floor, clearing) for the near-field ROI, or None if invalid.

    Clearing endpoints are observed floor and confirmed obstacle points only;
    unclassified points never clear the costmap. This is not a drop-off detector.
    Fixed-height mode does not require a floor seed: the floor-height band is
    used only to select observed clearing endpoints, not to gate detection.
    """
    if not len(points):
        return None
    x, z = points[:, 0], points[:, 2]
    region = (np.isfinite(x) & np.isfinite(z)
              & (x >= cfg.obstacle_x_min) & (x <= cfg.obstacle_x_max))
    indices, points = indices[region], points[region]
    if len(points) == 0:
        return None
    x, z = points[:, 0], points[:, 2]

    if enable_ground_filter:
        result = _lightweight_masks(x, z, cfg)
        if result is None:
            return None
        floor, candidates = result
    else:
        # Detect independently of floor visibility. Only actual returns in the
        # floor-height band may clear free space; never synthesize floor points.
        floor = np.abs(z) <= cfg.floor_inlier_tolerance
        candidates = ~floor & (z >= cfg.obstacle_min_height) & (z <= cfg.obstacle_max_height)

    obstacles = _cluster_mask(indices, candidates, cfg.min_cluster_points)
    clearing = floor | obstacles
    return points[obstacles], points[floor], points[clearing]
