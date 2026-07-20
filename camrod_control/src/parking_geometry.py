"""Shared geometry contract for drop-zone alignment and reverse parking."""

# HH_260720 - Keep station-yaw interpretation identical across control and parking nodes.

from __future__ import annotations

import math

# HH_260720 - Import the flattened shared control helper without package wrappers.
from control_support import normalize_angle


def body_yaw_for_reverse_axis(reverse_axis_yaw_rad: float) -> float:
    """Return ROS body-front yaw when the configured direction is the rear travel axis."""
    return normalize_angle(reverse_axis_yaw_rad + math.pi)


def reverse_axis_yaw_for_body(body_yaw_rad: float) -> float:
    """Return the map-frame direction traveled by a negative linear-x command."""
    return normalize_angle(body_yaw_rad + math.pi)


def signed_distance_along_axis_m(
    *,
    origin_x_m: float,
    origin_y_m: float,
    target_x_m: float,
    target_y_m: float,
    axis_yaw_rad: float,
) -> float:
    """Project the origin-to-target vector onto the supplied axis."""
    delta_x_m = target_x_m - origin_x_m
    delta_y_m = target_y_m - origin_y_m
    return math.cos(axis_yaw_rad) * delta_x_m + math.sin(axis_yaw_rad) * delta_y_m
