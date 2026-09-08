"""Tests for latest-only planning visualization and obstacle monitoring inputs."""

import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
import unittest
from unittest import mock

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_prefix
from avg_msgs.msg import AvgOccupancyGrid, AvgPath, AvgPoseStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.duration import Duration
from rclpy.time import Time
import yaml


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


class LocalPathTerminalRetentionTest(unittest.TestCase):
    """Exercise the native extractor with production YAML on isolated test topics.

    The executable is the workspace-built node, not a reimplementation of its
    completion condition. Domain 226 is set above and never shares robot topics.
    No Nav2 server, vehicle command, simulation, or map process is started.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.package = Path(__file__).resolve().parents[1]
        cls.config = cls.package / "config/local_path_extractor.yaml"
        cls.node = rclpy.create_node(f"terminal_path_test_{os.getpid()}")
        cls.topic = f"/terminal_path_test_{os.getpid()}"
        cls.samples = []
        cls.subscription = cls.node.create_subscription(
            AvgPath, cls.topic + "/local", lambda message: cls.samples.append(message), 10
        )
        cls.path_publisher = cls.node.create_publisher(RosPath, cls.topic + "/global", 10)
        cls.pose_publisher = cls.node.create_publisher(AvgPoseStamped, cls.topic + "/pose", 10)
        binary = Path(os.environ.get(
            "CAMROD_LOCAL_PATH_EXTRACTOR_TEST_EXECUTABLE",
            str(Path(get_package_prefix("camrod_planning")) / "lib/camrod_planning/local_path_extractor_node"),
        ))
        arguments = [str(binary), "--ros-args", "--params-file", str(cls.config),
                     "-r", "__ns:=/planning"]
        for name, suffix in (("global_path_topic", "global"), ("global_path_avg_topic", "global_avg"),
                             ("pose_topic", "pose"), ("output_topic", "local"),
                             ("output_topic_ros", "local_ros")):
            arguments += ["-p", f"{name}:={cls.topic}/{suffix}"]
        # Match this test's volatile publisher; all geometric/completion/stale
        # parameters continue to come from the deployed YAML under test.
        arguments += ["-p", "global_path_qos_transient_local:=false"]
        cls.process = subprocess.Popen(
            arguments, env=dict(os.environ, ROS_DOMAIN_ID="226"),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + 5.0
        while cls.path_publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            if cls.process.poll() is not None:
                raise RuntimeError("Native local_path_extractor exited before discovery")
            rclpy.spin_once(cls.node, timeout_sec=0.05)
        if cls.path_publisher.get_subscription_count() == 0:
            cls.tearDownClass()
            raise RuntimeError("Native local_path_extractor did not discover isolated test inputs")

    @classmethod
    def tearDownClass(cls):
        if cls.process.poll() is None:
            cls.process.send_signal(signal.SIGINT)
            try:
                cls.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                cls.process.terminate()
                cls.process.wait(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown()

    def publish_route(self, *, endpoint=10.0, empty=False, spacing=0.05, point_count=41):
        message = RosPath()
        message.header.frame_id = "map"
        message.header.stamp = self.node.get_clock().now().to_msg()
        if not empty:
            for index in range(point_count):
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = endpoint - (point_count - 1) * spacing + index * spacing
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
        self.path_publisher.publish(message)

    def observe_pose(self, x, *, y=0.0, duration=0.70, frame="map"):
        # Discard transition traffic. Require repeated fresh outputs over the
        # second half of the interval, not one cached nonempty path.
        started = time.monotonic()
        self.samples.clear()
        cleared = False
        while time.monotonic() - started < duration:
            elapsed = time.monotonic() - started
            if elapsed >= duration / 2.0 and not cleared:
                self.samples.clear()
                cleared = True
            pose = AvgPoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            self.pose_publisher.publish(pose)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.01)
        self.assertIsNone(self.process.poll())
        self.assertTrue(self.samples, "native extractor must keep publishing fresh path status")
        return [len(message.poses) for message in self.samples]

    def test_terminal_local_path_remains_available_at_observed_014m(self):
        self.publish_route()
        counts = self.observe_pose(9.86)
        self.assertTrue(all(count >= 2 for count in counts), counts)
        self.assertGreaterEqual(len(counts), 2, "recovery needs a fresh repeated path, not cached geometry")
        # An excursion outside 0.14 m also proves completion was not latched.
        self.assertTrue(all(count >= 2 for count in self.observe_pose(9.70)))

    def test_terminal_retention_matches_nav2_and_bringup_without_weaker_safety(self):
        canonical = yaml.safe_load(self.config.read_text())
        mirror = yaml.safe_load(
            (self.package.parent / "camrod_bringup/config/planning/local_path_extractor.yaml").read_text()
        )
        self.assertEqual(canonical, mirror)
        parameters = canonical["/planning/local_path_extractor"]["ros__parameters"]
        goal = yaml.safe_load((self.package / "config/nav2_base.yaml").read_text())
        nav2_tolerance = goal["controller_server"]["ros__parameters"]["goal_checker"]["xy_goal_tolerance"]
        self.assertEqual(nav2_tolerance, 0.10)
        self.assertEqual(parameters["goal_reached_distance_m"], 0.05)
        self.assertLess(parameters["goal_reached_distance_m"], nav2_tolerance)
        for flag in ("stop_after_goal_reached", "publish_empty_on_invalid", "clear_local_path_on_route_change"):
            self.assertTrue(parameters[flag])
        self.assertEqual(parameters["pose_timeout_s"], 2.5)
        self.assertEqual(parameters["max_segment_jump_m"], 3.0)

    def test_terminal_slice_keeps_two_points_when_closest_is_last(self):
        for spacing in (0.20, 0.2000000001, 0.25):
            with self.subTest(spacing=spacing):
                self.publish_route(spacing=spacing)
                # The closest route point is the last one, but the robot is
                # still 0.14 m laterally outside the 0.10 m Nav2 goal radius.
                counts = self.observe_pose(10.0, y=0.14)
                self.assertTrue(all(count >= 12 for count in counts), f"spacing={spacing}: {counts}")

    def test_terminal_retention_does_not_bridge_discontinuous_route(self):
        self.publish_route(spacing=3.01)
        self.assertTrue(all(count == 0 for count in self.observe_pose(10.0, y=0.14)))

    def test_short_terminal_route_retains_only_existing_three_points(self):
        self.publish_route(spacing=0.25, point_count=3)
        self.assertTrue(all(count == 3 for count in self.observe_pose(10.0, y=0.14)))

    def test_stale_pose_still_clears_retained_terminal_path(self):
        self.publish_route()
        self.assertTrue(all(count >= 2 for count in self.observe_pose(9.86)))
        deadline = time.monotonic() + 2.8
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.samples.clear()
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.samples)
        self.assertTrue(all(not message.poses for message in self.samples))

    def test_completion_latch_and_new_route_reset_are_preserved(self):
        self.publish_route()
        self.assertTrue(all(count == 0 for count in self.observe_pose(9.97)))
        self.assertTrue(all(count == 0 for count in self.observe_pose(9.70)))
        self.publish_route(endpoint=20.0)
        self.assertTrue(all(count >= 2 for count in self.observe_pose(19.86)))

    def test_empty_global_path_and_invalid_pose_still_clear_local_path(self):
        self.publish_route()
        self.assertTrue(all(count >= 2 for count in self.observe_pose(9.70)))
        self.publish_route(empty=True)
        self.assertTrue(all(count == 0 for count in self.observe_pose(9.70)))
        self.publish_route()
        self.assertTrue(all(count == 0 for count in self.observe_pose(9.70, frame="other")))
        self.assertTrue(all(count >= 2 for count in self.observe_pose(9.70)))


if __name__ == "__main__":
    unittest.main()
