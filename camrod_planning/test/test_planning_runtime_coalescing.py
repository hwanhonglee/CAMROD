"""Tests for latest-only planning visualization and obstacle monitoring inputs."""

import importlib.util
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

from action_msgs.msg import GoalStatus
from avg_msgs.msg import AvgOccupancyGrid, AvgPath, AvgPoseStamped
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.duration import Duration
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


def _lane_grid(*, half_width):
    message = AvgOccupancyGrid()
    message.header.frame_id = "map"
    message.info.width = 80
    message.info.height = 80
    message.info.resolution = 0.1
    message.info.origin.position.x = -4.0
    message.info.origin.position.y = -4.0
    message.info.origin.orientation.w = 1.0
    message.data = []
    for row in range(message.info.height):
        y = message.info.origin.position.y + (row + 0.5) * message.info.resolution
        cost = 70 if abs(y) < half_width else 100
        message.data.extend([cost] * message.info.width)
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
            self.assertEqual(len(grid_subscriptions), 3)
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

    def test_obstacle_fallback_is_allowed_only_on_a_wide_lane(self):
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            node._minimum_replan_lane_width_m = 2.5
            node._minimum_side_clearance_m = 0.6
            node._lanelet_blocked_cost_threshold = 100

            wide_record = OBSTACLE_MONITOR.GridRecord(
                topic="/map/cost_grid/lanelet",
                received_time=node.get_clock().now(),
                serialized_grid=None,
                grid=_lane_grid(half_width=1.5),
            )
            node._prepare_grid_record(wide_record)
            wide = node._measure_lane_width(wide_record, 0.0, 0.0, 0.0)
            self.assertTrue(wide.allowed)
            self.assertGreaterEqual(wide.total_width_m, 2.5)

            narrow_record = OBSTACLE_MONITOR.GridRecord(
                topic="/map/cost_grid/lanelet",
                received_time=node.get_clock().now(),
                serialized_grid=None,
                grid=_lane_grid(half_width=1.0),
            )
            node._prepare_grid_record(narrow_record)
            narrow = node._measure_lane_width(narrow_record, 0.0, 0.0, 0.0)
            self.assertFalse(narrow.allowed)
            self.assertEqual(narrow.reason, "lane_too_narrow")
        finally:
            node.destroy_node()

    def test_obstacle_fallback_waits_for_twenty_seconds_of_blockage(self):
        """Persistent replan timing must not weaken the separate command gate."""
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            self.assertEqual(node._block_hold_s, 20.0)
            node._require_navigation_active = False
            node._require_goal = False
            node._latest_path = _path("map")
            node._latest_pose = AvgPoseStamped()
            blockage = OBSTACLE_MONITOR.BlockageSample(
                blocked=True,
                blocked_count=3,
                total_count=10,
                max_cost=85,
                source_topic="/sensing/cost_grid/lidar",
                point_x=1.0,
                point_y=0.0,
            )

            with mock.patch.object(
                node, "_sample_dynamic_blockage", return_value=blockage
            ), mock.patch.object(node, "_publish_status"), mock.patch.object(
                node, "_maybe_trigger_fallback_replan"
            ) as trigger:
                node._blocked_since = node.get_clock().now() - Duration(seconds=19.5)
                node._on_monitor_timer()
                trigger.assert_not_called()

                node._blocked_since = node.get_clock().now() - Duration(seconds=20.1)
                node._on_monitor_timer()
                trigger.assert_called_once()
        finally:
            node.destroy_node()

    def test_obstacle_fallback_probes_path_before_navigation_preemption(self):
        """A no-path result must not replace the still-recoverable route mission."""
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            node._latest_goal = AvgPoseStamped()
            node._latest_goal.header.frame_id = "map"
            node._latest_goal.pose.orientation.w = 1.0
            node._latest_pose = AvgPoseStamped()
            node._latest_pose.header.frame_id = "map"
            node._latest_pose.pose.orientation.w = 1.0
            blockage = OBSTACLE_MONITOR.BlockageSample(
                blocked=True,
                blocked_count=8,
                total_count=20,
                max_cost=100,
                source_topic="/sensing/cost_grid/lidar",
                point_x=2.0,
                point_y=0.0,
            )
            lane_width = OBSTACLE_MONITOR.LaneWidthSample(
                allowed=True,
                total_width_m=3.0,
                left_clearance_m=1.5,
                right_clearance_m=1.5,
                reason="wide_lane",
            )
            send_future = mock.Mock()

            with mock.patch.object(
                node._compute_path_client, "wait_for_server", return_value=True
            ), mock.patch.object(
                node._compute_path_client,
                "send_goal_async",
                return_value=send_future,
            ) as compute_send, mock.patch.object(
                node._navigate_client, "send_goal_async"
            ) as navigate_send:
                node._probe_fallback_path(blockage, lane_width)

            self.assertTrue(node._fallback_probe_in_flight)
            compute_goal = compute_send.call_args.args[0]
            self.assertEqual(compute_goal.planner_id, "SmacLattice")
            self.assertTrue(compute_goal.use_start)
            send_future.add_done_callback.assert_called_once()
            navigate_send.assert_not_called()
        finally:
            node.destroy_node()

    def test_obstacle_failed_probe_latches_without_repeated_preemption(self):
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            node._fallback_probe_generation = 7
            node._fallback_probe_in_flight = True
            response = SimpleNamespace(
                status=GoalStatus.STATUS_ABORTED,
                result=SimpleNamespace(path=SimpleNamespace(poses=[])),
            )
            future = mock.Mock()
            future.result.return_value = response

            with mock.patch.object(node, "_restore_primary_selector"), mock.patch.object(
                node, "_publish_status"
            ), mock.patch.object(node, "_preempt_with_validated_fallback") as preempt:
                node._on_probe_result(future, 7)

            self.assertFalse(node._fallback_probe_in_flight)
            self.assertTrue(node._fallback_failed_latched)
            self.assertEqual(
                node._fallback_failure_reason, "compute_path_status_6_poses_0"
            )
            preempt.assert_not_called()

            node._preempt_enabled = True
            blockage = OBSTACLE_MONITOR.BlockageSample(
                True, 8, 20, 100, "/sensing/cost_grid/lidar", 2.0, 0.0
            )
            with mock.patch.object(node, "_publish_status") as publish_status, mock.patch.object(
                node, "_probe_fallback_path"
            ) as probe:
                node._maybe_trigger_fallback_replan(
                    blockage, node.get_clock().now(), 25.0, "BLOCKED_NO_PREEMPT"
                )
            probe.assert_not_called()
            self.assertIn("BLOCKED_REPLAN_FAILED_HOLD", publish_status.call_args.args[0])

            node._reset_fallback_probe()
            self.assertFalse(node._fallback_failed_latched)
            self.assertEqual(node._fallback_failure_reason, "")
        finally:
            node.destroy_node()

    def test_obstacle_validated_probe_is_the_only_preemption_path(self):
        node = OBSTACLE_MONITOR.ObstacleReplanMonitor()
        try:
            node._fallback_probe_generation = 3
            node._fallback_probe_in_flight = True
            response = SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=SimpleNamespace(path=SimpleNamespace(poses=[object(), object()])),
            )
            future = mock.Mock()
            future.result.return_value = response

            with mock.patch.object(node, "_preempt_with_validated_fallback") as preempt:
                node._on_probe_result(future, 3)

            self.assertFalse(node._fallback_probe_in_flight)
            self.assertFalse(node._fallback_failed_latched)
            preempt.assert_called_once_with()
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
