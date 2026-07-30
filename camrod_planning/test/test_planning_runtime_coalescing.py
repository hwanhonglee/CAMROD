"""Tests for latest-only planning visualization and obstacle monitoring inputs."""

import importlib.util
import math
import os
from pathlib import Path
import sys
import unittest
from unittest import mock

from avg_msgs.msg import AvgOccupancyGrid, AvgPath, AvgPoseStamped
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time


os.environ["ROS_DOMAIN_ID"] = "226"
SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"


def _load_script(stem):
    spec = importlib.util.spec_from_file_location(stem, SCRIPT_DIR / f"{stem}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[stem] = module
    spec.loader.exec_module(module)
    return module


PATH_VISUALIZER = _load_script("path_visualizer_node")
OBSTACLE_MONITOR = _load_script("obstacle_replan_monitor_node")


def _path(frame_id):
    message = AvgPath()
    message.header.frame_id = frame_id
    return message


def _grid(*, yaw=0.0):
    message = AvgOccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 4
    message.info.height = 3
    message.info.resolution = 0.5
    message.info.origin.position.x = 10.0
    message.info.origin.position.y = 20.0
    message.info.origin.orientation.z = math.sin(0.5 * yaw)
    message.info.origin.orientation.w = math.cos(0.5 * yaw)
    message.data = [0, 1, 2, 3, 4, -1, 6, 7, 8, 9, 10, 11]
    return message


def _legacy_grid_cost(grid, point_x, point_y, *, ignore_unknown):
    info = grid.info
    if info.width == 0 or info.height == 0 or info.resolution <= 0.0:
        return None
    quaternion = info.origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )
    dx = point_x - info.origin.position.x
    dy = point_y - info.origin.position.y
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    local_x = cos_yaw * dx - sin_yaw * dy
    local_y = sin_yaw * dx + cos_yaw * dy
    cell_x = int(math.floor(local_x / info.resolution))
    cell_y = int(math.floor(local_y / info.resolution))
    if cell_x < 0 or cell_y < 0 or cell_x >= int(info.width) or cell_y >= int(info.height):
        return None
    index = cell_y * int(info.width) + cell_x
    if index < 0 or index >= len(grid.data):
        return None
    cost = int(grid.data[index])
    if cost < 0 and ignore_unknown:
        return None
    return cost


class PlanningRuntimeCoalescingTest(unittest.TestCase):
    """Exercises raw subscriptions, coalescing, and cached grid transforms."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_path_visualizer_decodes_only_latest_path_when_due(self):
        node = PATH_VISUALIZER.PathVisualizerNode()
        try:
            path_subscriptions = [
                subscription
                for subscription in node.subscriptions
                if subscription.msg_type is AvgPath
            ]
            self.assertEqual(len(path_subscriptions), 2)
            self.assertTrue(all(subscription.raw for subscription in path_subscriptions))
            self.assertTrue(
                all(subscription.qos_profile.depth == 1 for subscription in path_subscriptions)
            )

            first = serialize_message(_path("first"))
            latest = serialize_message(_path("latest"))
            with mock.patch.object(
                PATH_VISUALIZER,
                "deserialize_message",
                wraps=deserialize_message,
            ) as decode:
                node._on_local_path(first)
                node._on_local_path(latest)
                self.assertEqual(decode.call_count, 0)

                node.publish_without_subscribers = True
                node._publish(force=True)

                self.assertEqual(decode.call_count, 1)
                self.assertEqual(node.local_path.header.frame_id, "latest")
                self.assertIsNone(node.pending_local_path)
        finally:
            node.destroy_node()

    def test_obstacle_monitor_decodes_latest_path_at_monitor_cadence(self):
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            path_subscriptions = [
                subscription
                for subscription in node.subscriptions
                if subscription.msg_type is AvgPath
            ]
            grid_subscriptions = [
                subscription
                for subscription in node.subscriptions
                if subscription.msg_type is AvgOccupancyGrid
            ]
            self.assertEqual(len(path_subscriptions), 1)
            self.assertEqual(len(grid_subscriptions), 2)
            self.assertTrue(path_subscriptions[0].raw)
            self.assertTrue(all(subscription.raw for subscription in grid_subscriptions))
            self.assertTrue(
                all(subscription.qos_profile.depth == 1 for subscription in grid_subscriptions)
            )
            self.assertTrue(node._selector_timer.is_canceled())

            first = serialize_message(_path("first"))
            latest = serialize_message(_path("latest"))
            node._latest_pose = AvgPoseStamped()
            with mock.patch.object(
                OBSTACLE_MONITOR,
                "deserialize_message",
                wraps=deserialize_message,
            ) as decode:
                node._on_path(first)
                node._on_path(latest)
                self.assertEqual(decode.call_count, 0)

                blockage = node._sample_dynamic_blockage()

                self.assertFalse(blockage.blocked)
                self.assertEqual(decode.call_count, 1)
                self.assertEqual(node._latest_path.header.frame_id, "latest")
                self.assertIsNone(node._latest_path_serialized)
        finally:
            node.destroy_node()

    def test_obstacle_monitor_decodes_only_latest_fresh_grid(self):
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            first = _grid()
            first.data[0] = 12
            latest = _grid()
            latest.data[0] = 34
            with mock.patch.object(
                OBSTACLE_MONITOR,
                "deserialize_message",
                wraps=deserialize_message,
            ) as decode:
                node._on_grid("/grid", serialize_message(first))
                node._on_grid("/grid", serialize_message(latest))
                self.assertEqual(decode.call_count, 0)

                fresh = node._fresh_grids()

                self.assertEqual(decode.call_count, 1)
                self.assertEqual(len(fresh), 1)
                self.assertEqual(fresh[0].grid.data[0], 34)
                self.assertIsNone(fresh[0].serialized_grid)
                node._fresh_grids()
                self.assertEqual(decode.call_count, 1)

                node._on_grid("/grid", serialize_message(first))
                node._grids["/grid"].received_time = Time(
                    clock_type=node.get_clock().clock_type
                )
                decode.reset_mock()
                self.assertEqual(node._fresh_grids(), [])
                self.assertEqual(decode.call_count, 0)
        finally:
            node.destroy_node()

    def test_cached_grid_lookup_matches_previous_formula(self):
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            local_samples = (
                (0.1, 0.1),
                (0.75, 0.75),
                (1.8, 1.2),
                (-0.01, 0.1),
                (2.1, 0.1),
            )
            for yaw in (0.0, math.pi / 2.0, -0.37):
                grid = _grid(yaw=yaw)
                cos_yaw = math.cos(yaw)
                sin_yaw = math.sin(yaw)
                world_samples = [
                    (
                        grid.info.origin.position.x
                        + cos_yaw * local_x
                        - sin_yaw * local_y,
                        grid.info.origin.position.y
                        + sin_yaw * local_x
                        + cos_yaw * local_y,
                    )
                    for local_x, local_y in local_samples
                ]
                record = OBSTACLE_MONITOR.GridRecord(
                    topic="/grid",
                    received_time=Time(),
                    serialized_grid=None,
                    grid=grid,
                )
                node._prepare_grid_record(record)
                for ignore_unknown in (False, True):
                    node._ignore_unknown_cells = ignore_unknown
                    for point_x, point_y in world_samples:
                        self.assertEqual(
                            node._grid_cost_at(record, point_x, point_y),
                            _legacy_grid_cost(
                                grid,
                                point_x,
                                point_y,
                                ignore_unknown=ignore_unknown,
                            ),
                        )
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
