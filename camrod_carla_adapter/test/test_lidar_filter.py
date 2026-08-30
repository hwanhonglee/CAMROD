"""Numerical and PointCloud2 contracts for the real CARLA LiDAR filter."""

from dataclasses import replace
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

import camrod_carla_adapter.lidar_filter_node as lidar_filter_node
from camrod_carla_adapter.lidar_filter import (
    LidarFilterConfig,
    estimate_ground_plane,
    nonground_mask,
    nonground_mask_with_diagnostics,
)
from camrod_carla_adapter.lidar_filter_node import (
    pointcloud_xyz_and_records,
    selected_pointcloud,
)


class _StubLidarFilterNode:
    def __init__(self, calls):
        self._calls = calls

    def destroy_node(self):
        self._calls.append("destroy_node")


def test_main_skips_duplicate_shutdown_after_ctrl_c_stops_ros_context(
    monkeypatch,
):
    calls = []
    context = {"ok": True}
    node = _StubLidarFilterNode(calls)

    monkeypatch.setattr(
        lidar_filter_node.rclpy,
        "init",
        lambda args=None: calls.append(("init", args)),
    )
    monkeypatch.setattr(lidar_filter_node, "CarlaLidarFilterNode", lambda: node)

    def interrupt_spin(actual_node):
        assert actual_node is node
        calls.append("spin")
        context["ok"] = False
        raise KeyboardInterrupt

    monkeypatch.setattr(lidar_filter_node.rclpy, "spin", interrupt_spin)
    monkeypatch.setattr(lidar_filter_node.rclpy, "ok", lambda: context["ok"])
    monkeypatch.setattr(
        lidar_filter_node.rclpy,
        "shutdown",
        lambda: calls.append("shutdown"),
    )

    lidar_filter_node.main(args=["--ros-args"])

    assert calls == [("init", ["--ros-args"]), "spin", "destroy_node"]


def test_main_preserves_unexpected_runtime_exception_and_cleans_up(monkeypatch):
    calls = []
    node = _StubLidarFilterNode(calls)

    monkeypatch.setattr(
        lidar_filter_node.rclpy,
        "init",
        lambda args=None: calls.append(("init", args)),
    )
    monkeypatch.setattr(lidar_filter_node, "CarlaLidarFilterNode", lambda: node)

    def failed_spin(actual_node):
        assert actual_node is node
        calls.append("spin")
        raise RuntimeError("unexpected LiDAR runtime failure")

    monkeypatch.setattr(lidar_filter_node.rclpy, "spin", failed_spin)
    monkeypatch.setattr(lidar_filter_node.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        lidar_filter_node.rclpy,
        "shutdown",
        lambda: calls.append("shutdown"),
    )

    with pytest.raises(RuntimeError, match="unexpected LiDAR runtime failure"):
        lidar_filter_node.main()

    assert calls == [("init", None), "spin", "destroy_node", "shutdown"]


def _ground_and_obstacle(slope_x=0.01, slope_y=-0.005):
    grid_x, grid_y = np.meshgrid(
        np.linspace(0.2, 5.0, 25), np.linspace(-2.8, 2.8, 29)
    )
    ground_z = -0.59538 + slope_x * grid_x + slope_y * grid_y
    ground = np.column_stack(
        (grid_x.ravel(), grid_y.ravel(), ground_z.ravel())
    )
    obstacle_x, obstacle_y = np.meshgrid(
        np.linspace(1.8, 2.3, 8), np.linspace(0.2, 0.7, 8)
    )
    obstacle_ground = (
        -0.59538 + slope_x * obstacle_x + slope_y * obstacle_y
    )
    obstacle_z = obstacle_ground + np.linspace(0.12, 0.62, 64).reshape(8, 8)
    obstacle = np.column_stack(
        (obstacle_x.ravel(), obstacle_y.ravel(), obstacle_z.ravel())
    )
    return ground, obstacle


def _measured_ranger_self_returns():
    """Representative points inside both live Car-labelled side clusters."""
    x, absolute_y, z = np.meshgrid(
        np.array([0.541, 0.90, 1.255]),
        np.array([0.638, 0.76, 0.891]),
        np.array([-0.48, -0.19, 0.12]),
        indexing="ij",
    )
    left = np.column_stack((x.ravel(), absolute_y.ravel(), z.ravel()))
    right = left.copy()
    right[:, 1] *= -1.0
    return np.vstack((left, right))


def test_real_ground_plane_is_removed_and_elevated_hits_are_preserved():
    ground, obstacle = _ground_and_obstacle()
    points = np.vstack((ground, obstacle))
    mask, plane = nonground_mask(points, LidarFilterConfig())

    assert plane.used_fallback is False
    assert plane.inlier_count >= ground.shape[0]
    assert np.count_nonzero(mask[: ground.shape[0]]) == 0
    assert np.count_nonzero(mask[ground.shape[0] :]) == obstacle.shape[0]
    assert plane.coefficients == pytest.approx((0.01, -0.005, -0.59538), abs=1e-6)


def test_local_plane_fit_handles_a_sloped_carla_road_deterministically():
    slope = np.tan(np.deg2rad(8.0))
    ground, obstacle = _ground_and_obstacle(slope_x=slope, slope_y=0.02)
    points = np.vstack((ground, obstacle))
    first_mask, first_plane = nonground_mask(points, LidarFilterConfig())
    second_mask, second_plane = nonground_mask(points, LidarFilterConfig())

    assert first_plane.used_fallback is False
    assert first_plane.coefficients == pytest.approx(
        (slope, 0.02, -0.59538), abs=1e-6
    )
    assert np.array_equal(first_mask, second_mask)
    assert first_plane == second_plane
    assert np.count_nonzero(first_mask[: ground.shape[0]]) == 0
    assert np.count_nonzero(first_mask[ground.shape[0] :]) == obstacle.shape[0]


def test_insufficient_ground_evidence_uses_explicit_sensor_height_fallback():
    config = LidarFilterConfig()
    points = np.array([
        [1.0, 0.0, -0.59538],
        [1.0, 0.2, -0.40],
        [1.0, -0.2, 0.10],
    ])
    plane = estimate_ground_plane(points, config)
    mask, same_plane = nonground_mask(points, config)

    assert plane.used_fallback is True
    assert "insufficient" in plane.reason
    assert same_plane == plane
    assert mask.tolist() == [False, True, True]


def test_enabled_self_return_mask_removes_both_measured_side_clusters():
    ground, _obstacle = _ground_and_obstacle()
    self_returns = _measured_ranger_self_returns()
    points = np.vstack((ground, self_returns))
    config = replace(LidarFilterConfig(), self_return_mask_enabled=True)

    keep, plane, removed_count = nonground_mask_with_diagnostics(points, config)

    assert plane.used_fallback is False
    assert removed_count == self_returns.shape[0]
    assert np.count_nonzero(keep[-self_returns.shape[0] :]) == 0


def test_self_return_mask_preserves_close_center_and_outside_obstacles():
    ground, _obstacle = _ground_and_obstacle()
    self_returns = _measured_ranger_self_returns()
    close_real_obstacles = np.array(
        [
            [0.60, -0.40, -0.20],
            [0.85, 0.00, -0.10],
            [1.20, 0.40, 0.00],
            [0.85, 1.10, -0.20],
            [0.85, -1.10, -0.20],
            [1.55, 0.75, -0.20],
        ]
    )
    points = np.vstack((ground, self_returns, close_real_obstacles))
    config = replace(LidarFilterConfig(), self_return_mask_enabled=True)

    keep, _plane, removed_count = nonground_mask_with_diagnostics(points, config)

    assert removed_count == self_returns.shape[0]
    assert np.all(keep[-close_real_obstacles.shape[0] :])


def test_default_disabled_self_return_mask_is_production_neutral():
    ground, _obstacle = _ground_and_obstacle()
    self_returns = _measured_ranger_self_returns()
    points = np.vstack((ground, self_returns))

    keep, _plane, removed_count = nonground_mask_with_diagnostics(
        points, LidarFilterConfig()
    )

    assert LidarFilterConfig().self_return_mask_enabled is False
    assert removed_count == 0
    assert np.all(keep[-self_returns.shape[0] :])


def test_pointcloud_selection_preserves_original_carla_field_records():
    message = PointCloud2()
    message.header.frame_id = "lidar_link"
    message.height = 1
    message.width = 3
    message.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in (("x", 0), ("y", 4), ("z", 8), ("intensity", 12))
    ]
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = 48
    rows = (
        (1.0, -0.1, -0.60, 0.9),
        (2.0, 0.2, -0.20, 0.7),
        (3.0, 0.3, 0.10, 0.5),
    )
    message.data = b"".join(struct.pack("<ffff", *row) for row in rows)
    message.is_dense = True

    xyz, records = pointcloud_xyz_and_records(message)
    output = selected_pointcloud(message, records[[False, True, True]])

    assert xyz == pytest.approx(np.array([row[:3] for row in rows]))
    assert output.header.frame_id == "lidar_link"
    assert output.height == 1
    assert output.width == 2
    assert output.point_step == 16
    assert output.row_step == 32
    assert bytes(output.data) == bytes(message.data[16:])
