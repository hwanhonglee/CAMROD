"""Deterministic ground filtering for CARLA PointCloud2 samples.

The physical Vanjee ground segmenter is tuned for a denser scanner and is not
used here.  This module fits a local road plane to the lowest return in each
XY cell, rejects robust-fit outliers, and keeps only measured points above the
plane.  It contains no ROS dependencies so the numerical contract is directly
unit-testable.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class LidarFilterConfig:
    """Geometry and robust-fit limits expressed in the LiDAR frame."""

    roi_x_min_m: float = 0.0
    roi_x_max_m: float = 5.0
    roi_y_min_m: float = -3.0
    roi_y_max_m: float = 3.0
    roi_z_min_m: float = -1.2
    roi_z_max_m: float = 1.5
    expected_ground_z_m: float = -0.59538
    ground_seed_z_tolerance_m: float = 0.8
    ground_grid_size_m: float = 0.40
    ground_inlier_threshold_m: float = 0.045
    minimum_obstacle_height_m: float = 0.08
    maximum_ground_slope_deg: float = 25.0
    minimum_ground_seed_cells: int = 12
    minimum_ground_inliers: int = 24
    robust_fit_iterations: int = 4
    # Disabled by default so this numerical module remains neutral for other
    # CARLA actors and physical LiDARs.  The Ranger CARLA profile enables it
    # with bounds measured from that actor's query geometry.
    self_return_mask_enabled: bool = False
    self_return_x_min_m: float = 0.49
    self_return_x_max_m: float = 1.31
    self_return_abs_y_min_m: float = 0.59
    self_return_abs_y_max_m: float = 0.95
    self_return_z_min_m: float = -0.57
    self_return_z_max_m: float = 0.18


@dataclass(frozen=True)
class GroundPlaneResult:
    """Plane ``z = ax + by + c`` and evidence for its validity."""

    coefficients: tuple[float, float, float]
    seed_count: int
    inlier_count: int
    used_fallback: bool
    reason: str = ""


def _fit_plane(points: np.ndarray) -> np.ndarray | None:
    if points.shape[0] < 3:
        return None
    design = np.column_stack(
        (points[:, 0], points[:, 1], np.ones(points.shape[0]))
    )
    coefficients, _residuals, rank, _singular = np.linalg.lstsq(
        design, points[:, 2], rcond=None
    )
    if rank < 3 or not np.all(np.isfinite(coefficients)):
        return None
    return coefficients


def _fallback(config: LidarFilterConfig, reason: str, seeds: int = 0):
    return GroundPlaneResult(
        coefficients=(0.0, 0.0, config.expected_ground_z_m),
        seed_count=seeds,
        inlier_count=0,
        used_fallback=True,
        reason=reason,
    )


def estimate_ground_plane(
    roi_points: np.ndarray, config: LidarFilterConfig
) -> GroundPlaneResult:
    """Estimate a deterministic local road plane from finite ROI points."""
    points = np.asarray(roi_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("roi_points must have shape (N, 3)")
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < config.minimum_ground_inliers:
        return _fallback(config, "insufficient finite ROI points")

    cell_x = np.floor(
        (points[:, 0] - config.roi_x_min_m) / config.ground_grid_size_m
    ).astype(np.int64)
    cell_y = np.floor(
        (points[:, 1] - config.roi_y_min_m) / config.ground_grid_size_m
    ).astype(np.int64)
    y_span = max(1, int(cell_y.max() - cell_y.min() + 1))
    cell_key = cell_x * y_span + (cell_y - cell_y.min())
    # lexsort uses its last key as primary: group by cell, then take its
    # smallest Z return as the road seed.
    order = np.lexsort((points[:, 2], cell_key))
    ordered_keys = cell_key[order]
    _unique, first = np.unique(ordered_keys, return_index=True)
    seeds = points[order[first]]
    seed_z_min = (
        config.expected_ground_z_m - config.ground_seed_z_tolerance_m
    )
    seed_z_max = (
        config.expected_ground_z_m + config.ground_seed_z_tolerance_m
    )
    seeds = seeds[
        (seeds[:, 2] >= seed_z_min) & (seeds[:, 2] <= seed_z_max)
    ]
    if seeds.shape[0] < config.minimum_ground_seed_cells:
        return _fallback(
            config,
            "insufficient plausible ground seed cells",
            int(seeds.shape[0]),
        )

    coefficients = _fit_plane(seeds)
    if coefficients is None:
        return _fallback(config, "ground seed plane is degenerate", len(seeds))

    robust_seeds = seeds
    for _ in range(max(1, config.robust_fit_iterations)):
        predicted = (
            coefficients[0] * robust_seeds[:, 0]
            + coefficients[1] * robust_seeds[:, 1]
            + coefficients[2]
        )
        residual = robust_seeds[:, 2] - predicted
        residual_center = float(np.median(residual))
        mad = float(np.median(np.abs(residual - residual_center)))
        robust_limit = max(
            config.ground_inlier_threshold_m,
            2.5 * 1.4826 * mad,
        )
        keep = np.abs(residual - residual_center) <= robust_limit
        if int(np.count_nonzero(keep)) < config.minimum_ground_seed_cells:
            break
        robust_seeds = robust_seeds[keep]
        candidate = _fit_plane(robust_seeds)
        if candidate is None:
            break
        coefficients = candidate

    # Refine against all measured road returns, rather than cell minima only.
    inlier_count = 0
    for _ in range(2):
        predicted = (
            coefficients[0] * points[:, 0]
            + coefficients[1] * points[:, 1]
            + coefficients[2]
        )
        inliers = np.abs(points[:, 2] - predicted) <= (
            config.ground_inlier_threshold_m
        )
        inlier_count = int(np.count_nonzero(inliers))
        if inlier_count < config.minimum_ground_inliers:
            return _fallback(
                config,
                "too few measured plane inliers",
                int(seeds.shape[0]),
            )
        candidate = _fit_plane(points[inliers])
        if candidate is None:
            return _fallback(
                config,
                "measured ground plane is degenerate",
                int(seeds.shape[0]),
            )
        coefficients = candidate

    slope_deg = math.degrees(
        math.atan(math.hypot(float(coefficients[0]), float(coefficients[1])))
    )
    if slope_deg > config.maximum_ground_slope_deg:
        return _fallback(
            config,
            f"ground slope {slope_deg:.2f} deg exceeds limit",
            int(seeds.shape[0]),
        )
    if abs(float(coefficients[2]) - config.expected_ground_z_m) > (
        config.ground_seed_z_tolerance_m
    ):
        return _fallback(
            config,
            "ground intercept is outside the sensor-height envelope",
            int(seeds.shape[0]),
        )

    return GroundPlaneResult(
        coefficients=tuple(float(value) for value in coefficients),
        seed_count=int(seeds.shape[0]),
        inlier_count=inlier_count,
        used_fallback=False,
    )


def self_return_mask(
    xyz_points: np.ndarray, config: LidarFilterConfig
) -> np.ndarray:
    """Select Ranger CARLA query-geometry returns when explicitly enabled.

    ``abs(y)`` describes two symmetric left/right boxes while deliberately
    leaving the center corridor untouched.  This is narrower than a radial
    minimum-range filter, so a genuine obstacle close to the sensor at the
    center or outside the measured side boxes remains observable.
    """
    points = np.asarray(xyz_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz_points must have shape (N, 3)")
    if not config.self_return_mask_enabled:
        return np.zeros(points.shape[0], dtype=bool)
    finite = np.all(np.isfinite(points), axis=1)
    absolute_y = np.abs(points[:, 1])
    return (
        finite
        & (points[:, 0] >= config.self_return_x_min_m)
        & (points[:, 0] <= config.self_return_x_max_m)
        & (absolute_y >= config.self_return_abs_y_min_m)
        & (absolute_y <= config.self_return_abs_y_max_m)
        & (points[:, 2] >= config.self_return_z_min_m)
        & (points[:, 2] <= config.self_return_z_max_m)
    )


def nonground_mask_with_diagnostics(
    xyz_points: np.ndarray, config: LidarFilterConfig
) -> tuple[np.ndarray, GroundPlaneResult, int]:
    """Select nonground points and report self returns actually removed."""
    points = np.asarray(xyz_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("xyz_points must have shape (N, 3)")
    finite = np.all(np.isfinite(points), axis=1)
    roi = (
        finite
        & (points[:, 0] >= config.roi_x_min_m)
        & (points[:, 0] <= config.roi_x_max_m)
        & (points[:, 1] >= config.roi_y_min_m)
        & (points[:, 1] <= config.roi_y_max_m)
        & (points[:, 2] >= config.roi_z_min_m)
        & (points[:, 2] <= config.roi_z_max_m)
    )
    plane = estimate_ground_plane(points[roi], config)
    a, b, c = plane.coefficients
    ground_z = a * points[:, 0] + b * points[:, 1] + c
    height = points[:, 2] - ground_z
    nonground = roi & (height >= config.minimum_obstacle_height_m)
    self_returns = self_return_mask(points, config)
    removed_count = int(np.count_nonzero(nonground & self_returns))
    return nonground & ~self_returns, plane, removed_count


def nonground_mask(
    xyz_points: np.ndarray, config: LidarFilterConfig
) -> tuple[np.ndarray, GroundPlaneResult]:
    """Return a full-size mask selecting measured nonground obstacle points."""
    keep, plane, _removed_count = nonground_mask_with_diagnostics(
        xyz_points, config
    )
    return keep, plane
