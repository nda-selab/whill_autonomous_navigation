from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class OusterConfig:
    marking_min_z: float = -0.70
    marking_max_z: float = 0.10
    clearing_min_z: float = -1.00
    clearing_max_z: float = 0.30
    min_range: float = 0.10
    marking_max_range: float = 8.0
    clearing_max_range: float = 10.0
    azimuth_bin_deg: float = 0.5


def split_points(xyz, cfg):
    pts = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    dist = np.hypot(pts[:, 0], pts[:, 1])

    valid = np.isfinite(pts).all(axis=1) & (dist >= cfg.min_range)

    marking_mask = (
        valid
        & (dist <= cfg.marking_max_range)
        & (pts[:, 2] >= cfg.marking_min_z)
        & (pts[:, 2] <= cfg.marking_max_z)
    )
    marking = pts[marking_mask]

    clearing_mask = valid & (dist <= cfg.clearing_max_range)
    candidates = pts[clearing_mask]

    if len(candidates) == 0:
        return marking, pts[:0].copy()

    ranges = dist[clearing_mask]
    angles = np.arctan2(candidates[:, 1], candidates[:, 0])
    bins = np.floor(
        (angles + np.pi) / np.deg2rad(cfg.azimuth_bin_deg)
    ).astype(np.int32)

    order = np.lexsort((ranges, bins))
    _, first = np.unique(bins[order], return_index=True)
    nearest = candidates[order[first]]

    clearing = nearest[
        (nearest[:, 2] >= cfg.clearing_min_z)
        & (nearest[:, 2] <= cfg.clearing_max_z)
    ]
    return marking, clearing