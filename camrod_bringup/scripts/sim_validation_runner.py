#!/usr/bin/env python3
# HH_260630: Deterministic sim validation for planning, cmd gates, and fake obstacle sources.

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Callable

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
# HH_260720 - Validate generated CAMROD contracts and name ROS ecosystem boundaries explicitly.
from avg_msgs.msg import (
    AvgBool,
    AvgImu,
    AvgOccupancyGrid,
    AvgOdometry,
    AvgPath,
    AvgPlatformStatus,
    AvgPoseStamped,
    AvgRange,
    AvgServiceState,
    AvgString,
    AvgTwist,
    ModuleState,
    MotionOperation,
    PlanningMissionKey,
    PlanningState,
    UiDestinationCommand,
)
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped as RosPoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion as RosQuaternion
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2 as RosPointCloud2
from std_msgs.msg import String as RosString


RADAR_TOPICS = {
    "front": ["/sensing/radar/front1/range", "/sensing/radar/front2/range"],
    "left": ["/sensing/radar/left1/range", "/sensing/radar/left2/range"],
    "right": ["/sensing/radar/right1/range", "/sensing/radar/right2/range"],
    "rear": ["/sensing/radar/rear/range"],
}


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str
    metrics: dict[str, float | int | str | bool] = field(default_factory=dict)


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> RosQuaternion:
    q = RosQuaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def twist_abs(msg: AvgTwist) -> float:
    return (
        abs(msg.linear.x)
        + abs(msg.linear.y)
        + abs(msg.linear.z)
        + abs(msg.angular.x)
        + abs(msg.angular.y)
        + abs(msg.angular.z)
    )


class SimValidationRunner(Node):

    # HH_260721 - Separate the class declaration from its first method for lint readability.
    def __init__(self) -> None:
        super().__init__("sim_validation_runner")
        self.report_file = str(self.declare_parameter("report_file", "").value)
        self.quick = bool(self.declare_parameter("quick", False).value)
        self.manual_goal_timeout_s = float(
            self.declare_parameter("manual_goal_timeout_s", 45.0).value
        )
        self.camping_timeout_s = float(
            self.declare_parameter("camping_timeout_s", 240.0).value
        )
        self.run_camping = bool(self.declare_parameter("run_camping", False).value)
        self.camping_mission_key = str(
            self.declare_parameter("camping_mission_key", "camping_site_1").value
        )
        self.camping_sites_yaml = str(
            self.declare_parameter("camping_sites_yaml", "").value
        )
        self.camping_prepare_near_route = bool(
            self.declare_parameter("camping_prepare_near_route", True).value
        )
        self.camping_start_offset_m = float(
            self.declare_parameter("camping_start_offset_m", 0.0).value
        )
        self.camping_wait_drop_zone = bool(
            self.declare_parameter("camping_wait_drop_zone", False).value
        )
        # HH_260806 - Validate constrained roadside arrival without issuing an
        # unresolved return/turn command in a narrow real-world service area.
        self.camping_stop_at_wait_return = bool(
            self.declare_parameter("camping_stop_at_wait_return", False).value
        )
        # HH_260807 - Require the simulated raw CAN/BMS contract when charging
        # behavior is under test. The runner observes /platform/status but never
        # publishes it; ranger_platform_bridge remains the single owner.
        self.simulate_platform_status = bool(
            self.declare_parameter("simulate_platform_status", False).value
        )
        self.run_charging_recall = bool(
            self.declare_parameter("run_charging_recall", False).value
        )
        self.run_charging_recall_battery_gate = bool(
            self.declare_parameter("run_charging_recall_battery_gate", False).value
        )
        self.run_low_battery_finish_then_return = bool(
            self.declare_parameter("run_low_battery_finish_then_return", False).value
        )
        self.low_battery_mission_percentage = float(
            self.declare_parameter("low_battery_mission_percentage", 0.34).value
        )
        self.low_battery_return_manual_observe_s = float(
            self.declare_parameter("low_battery_return_manual_observe_s", 2.0).value
        )
        self.charging_recall_low_battery_percentage = float(
            self.declare_parameter(
                "charging_recall_low_battery_percentage", 0.34
            ).value
        )
        self.charging_recall_battery_gate_observe_s = float(
            self.declare_parameter(
                "charging_recall_battery_gate_observe_s", 2.0
            ).value
        )
        # HH_260721 - Optionally validate charging recall through the real UI command boundary.
        self.charging_recall_via_ui = bool(
            self.declare_parameter("charging_recall_via_ui", False).value
        )
        self.charging_recall_mission_key = str(
            self.declare_parameter(
                "charging_recall_mission_key", "camping_site_1"
            ).value
        )
        self.fake_platform_battery_percentage = float(
            self.declare_parameter("fake_platform_battery_percentage", 0.80).value
        )
        self.skip_manual_goal = bool(
            self.declare_parameter("skip_manual_goal", False).value
        )
        # HH_260720 - Allow long maneuver scenarios to reuse a previously validated gate matrix.
        self.run_gate_matrix = bool(
            self.declare_parameter("run_gate_matrix", True).value
        )
        self.run_obstacle_replan = bool(
            self.declare_parameter("run_obstacle_replan", False).value
        )
        self.obstacle_replan_expect_fallback = bool(
            self.declare_parameter("obstacle_replan_expect_fallback", False).value
        )
        self.obstacle_replan_expect_safe_hold = bool(
            self.declare_parameter("obstacle_replan_expect_safe_hold", False).value
        )
        self.obstacle_replan_start_x = float(
            self.declare_parameter("obstacle_replan_start_x", float("nan")).value
        )
        self.obstacle_replan_start_y = float(
            self.declare_parameter("obstacle_replan_start_y", float("nan")).value
        )
        self.obstacle_replan_start_yaw_deg = float(
            self.declare_parameter("obstacle_replan_start_yaw_deg", 0.0).value
        )
        # HH_260806 - Keep an absent LiDAR cost grid valid only when the launch
        # contract explicitly disables that optional component.
        self.expect_lidar_cost_grid = bool(
            # HH_260807 - Match the production launch default. ON validation
            # remains explicit with expect_lidar_cost_grid:=true.
            self.declare_parameter("expect_lidar_cost_grid", False).value
        )
        self.obstacle_replan_timeout_s = float(
            self.declare_parameter("obstacle_replan_timeout_s", 35.0).value
        )
        self.obstacle_replan_goal_distance_m = float(
            self.declare_parameter("obstacle_replan_goal_distance_m", 12.0).value
        )
        self.obstacle_replan_obstacle_offset_m = float(
            self.declare_parameter("obstacle_replan_obstacle_offset_m", 2.0).value
        )
        self.obstacle_replan_cluster_radius_m = float(
            self.declare_parameter("obstacle_replan_cluster_radius_m", 0.65).value
        )
        self.default_fake_obstacle_cluster_radius_m = float(
            self.declare_parameter("default_fake_obstacle_cluster_radius_m", 0.12).value
        )
        self.run_service_soak = bool(
            self.declare_parameter("run_service_soak", False).value
        )
        self.service_soak_mission_keys = [
            str(value).strip()
            for value in self.declare_parameter(
                "service_soak_mission_keys",
                ["camping_site_1", "camping_site_2", "camping_site_3"],
            ).value
            if str(value).strip()
        ]
        self.service_soak_cycle_timeout_s = float(
            self.declare_parameter("service_soak_cycle_timeout_s", 300.0).value
        )
        self.service_soak_obstacle_cycle = int(
            self.declare_parameter("service_soak_obstacle_cycle", 2).value
        )
        self.service_soak_obstacle_hold_s = float(
            self.declare_parameter("service_soak_obstacle_hold_s", 3.0).value
        )
        self.service_soak_obstacle_offset_m = float(
            self.declare_parameter("service_soak_obstacle_offset_m", 1.5).value
        )
        self.fake_sensor_node = str(
            self.declare_parameter("fake_sensor_node", "/bringup/fake_sensor_publisher").value
        )
        self.manual_goal_distance_m = float(
            self.declare_parameter("manual_goal_distance_m", 4.0).value
        )
        self.results: list[CheckResult] = []

        self.pub_goal = self.create_publisher(RosPoseStamped, "/goal_pose", 10)
        self.pub_prepare_goal = self.create_publisher(
            AvgPoseStamped, "/planning/auto_goal_raw", 10
        )
        self.pub_initialpose = self.create_publisher(
            RosPoseWithCovarianceStamped, "/initialpose", 10
        )
        self.pub_engage = self.create_publisher(AvgBool, "/planning/engage", 10)
        self.pub_mission_engage = self.create_publisher(
            AvgBool, "/planning/mission_engage", 10
        )
        self.pub_drive_enable = self.create_publisher(
            AvgBool, "/platform/drive_enable", 10
        )
        self.pub_mission_key = self.create_publisher(
            PlanningMissionKey, "/planning/mission_key", 10
        )
        # HH_260721 - Exercise the UI backend instead of bypassing station departure sequencing.
        self.pub_ui_destination = self.create_publisher(
            UiDestinationCommand, "/ui/selected_destination", 10
        )
        # HH_260720 - Drive each controller through its typed operation contract.
        self.pub_site_operation = self.create_publisher(
            MotionOperation, "/control/camping_site_maneuver_controller/operation", 10
        )
        self.pub_drop_maneuver_operation = self.create_publisher(
            MotionOperation, "/control/drop_zone_maneuver_controller/operation", 10
        )
        self.pub_parking_operation = self.create_publisher(
            MotionOperation, "/parking/operation", 10
        )
        self.pub_raw = self.create_publisher(AvgTwist, "/control/cmd_vel_raw", 10)

        self.param_client = self.create_client(
            SetParameters, f"{self.fake_sensor_node}/set_parameters"
        )
        self.gate_param_client = self.create_client(
            SetParameters, "/control/cmd_vel_safety_gate/set_parameters"
        )
        self.cancel_clients = {
            "follow_path": self.create_client(
                CancelGoal, "/planning/follow_path/_action/cancel_goal"
            ),
            "navigate_to_pose": self.create_client(
                CancelGoal, "/planning/navigate_to_pose/_action/cancel_goal"
            ),
        }

        self.counts: dict[str, int] = {}
        self.first_seen: dict[str, float] = {}
        self.last_seen: dict[str, float] = {}
        self.max_abs_since: dict[str, float] = {}
        self.latest_pose: AvgPoseStamped | None = None
        self.latest_lanelet_pose: AvgPoseStamped | None = None
        self.latest_state: PlanningState | None = None
        self.latest_mission_source: PlanningMissionKey | None = None
        self.latest_site_status: ModuleState | None = None
        self.latest_drop_maneuver_status: ModuleState | None = None
        self.latest_reverse_parking_controller_status: ModuleState | None = None
        self.latest_gate_status: ModuleState | None = None
        self.latest_route_recovery_status: ModuleState | None = None
        # HH_260721 - Validate the public service contract in addition to controller internals.
        self.latest_service_state: AvgServiceState | None = None
        self.latest_platform_status: AvgPlatformStatus | None = None
        self.service_state_names_seen: list[str] = []
        # HH_260807 - Preserve repeated phase transitions across service cycles;
        # latest-only status cannot prove a second return and parking sequence.
        self.site_status_events: list[str] = []
        self.drop_status_events: list[str] = []
        self.parking_status_events: list[str] = []
        self.gate_status_events: list[str] = []
        self.route_recovery_status_events: list[str] = []
        self.latest_control_cmd_abs = 0.0
        self.fake_platform_charging = False
        self.latest_replan_status = ""
        self.latest_planner_selector = ""
        self.replan_statuses_seen: set[str] = set()
        self.planner_selectors_seen: set[str] = set()
        self.latest_nav_status: GoalStatusArray | None = None
        self.latest_follow_status: GoalStatusArray | None = None
        self.latest_raw_goal: RosPoseStamped | None = None
        self.latest_route_goal: AvgPoseStamped | None = None
        self.latest_cost_grids: dict[str, AvgOccupancyGrid] = {}
        self.global_path_count = 0
        self.local_path_count = 0
        self.global_path_points = 0
        self.local_path_points = 0

        # HH_260720 - Observe the generated IMU contract instead of the hardware ROS boundary.
        self._subscribe_count("/sensing/imu/data", AvgImu)
        self._subscribe_count("/sensing/lidar/points_filtered", RosPointCloud2)
        self._subscribe_count("/perception/obstacles", RosPointCloud2)
        self._subscribe_cost_grid("/sensing/cost_grid/lidar")
        self._subscribe_cost_grid("/sensing/cost_grid/radar")
        self._subscribe_cost_grid("/planning/cost_grid/inflation")
        self._subscribe_count("/localization/input/wheel_odometry", AvgOdometry)
        for topics in RADAR_TOPICS.values():
            for topic in topics:
                self._subscribe_count(topic, AvgRange)

        self.create_subscription(AvgPoseStamped, "/localization/pose", self._on_pose, 10)
        self.create_subscription(RosPoseStamped, "/goal_pose", self._on_raw_goal, 10)
        self.create_subscription(
            AvgPoseStamped, "/planning/goal_pose_snapped", self._on_route_goal, 10
        )
        self.create_subscription(
            AvgPoseStamped, "/planning/lanelet_pose", self._on_lanelet_pose, 10
        )
        self.create_subscription(
            AvgPath, "/planning/global_path_avg", self._on_global_path, 10
        )
        self.create_subscription(AvgPath, "/planning/local_path", self._on_local_path, 10)
        self.create_subscription(AvgTwist, "/control/cmd_vel_raw", self._on_raw_cmd, 10)
        self.create_subscription(AvgTwist, "/control/cmd_vel", self._on_control_cmd, 10)
        self.create_subscription(
            PlanningState, "/planning/state_machine/state", self._on_state, 10
        )
        self.create_subscription(
            AvgServiceState, "/service/state", self._on_service_state, 10
        )
        self.create_subscription(
            AvgPlatformStatus, "/platform/status", self._on_platform_status, 10
        )
        self.create_subscription(
            PlanningMissionKey,
            "/planning/state_machine/mission_source",
            self._on_mission_source,
            10,
        )
        self.create_subscription(
            ModuleState,
            "/control/camping_site_maneuver_controller/status",
            self._on_site_status,
            10,
        )
        self.create_subscription(
            ModuleState,
            "/control/drop_zone_maneuver_controller/status",
            self._on_drop_maneuver_status,
            10,
        )
        self.create_subscription(
            ModuleState,
            "/parking/reverse_parking_controller/status",
            self._on_reverse_parking_controller_status,
            10,
        )
        self.create_subscription(
            ModuleState,
            "/control/cmd_vel_safety_gate/status",
            self._on_gate_status,
            10,
        )
        self.create_subscription(
            ModuleState,
            "/control/route_safety_recovery_controller/status",
            self._on_route_recovery_status,
            10,
        )
        self.create_subscription(
            AvgString, "/planning/obstacle_replan/status", self._on_replan_status, 10
        )
        self.create_subscription(
            # HH_260720 - Observe the explicit Nav2 std_msgs selector boundary.
            RosString, "/planning/planner_selector_ros", self._on_planner_selector, 10
        )
        self.create_subscription(
            GoalStatusArray,
            "/planning/navigate_to_pose/_action/status",
            self._on_nav_status,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            "/planning/follow_path/_action/status",
            self._on_follow_status,
            10,
        )

    def _subscribe_count(self, topic: str, msg_type) -> None:
        self.create_subscription(msg_type, topic, lambda _msg, t=topic: self._count(t), 10)

    def _subscribe_cost_grid(self, topic: str) -> None:
        self.create_subscription(
            AvgOccupancyGrid,
            topic,
            lambda msg, t=topic: self._on_cost_grid(t, msg),
            10,
        )

    def _count(self, topic: str) -> None:
        now_s = time.monotonic()
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.first_seen.setdefault(topic, now_s)
        self.last_seen[topic] = now_s

    def _record_twist(self, topic: str, msg: AvgTwist) -> None:
        self._count(topic)
        self.max_abs_since[topic] = max(self.max_abs_since.get(topic, 0.0), twist_abs(msg))

    def _on_cost_grid(self, topic: str, msg: AvgOccupancyGrid) -> None:
        self.latest_cost_grids[topic] = msg
        self._count(topic)

    def _on_pose(self, msg: AvgPoseStamped) -> None:
        self.latest_pose = msg
        self._count("/localization/pose")

    def _on_lanelet_pose(self, msg: AvgPoseStamped) -> None:
        self.latest_lanelet_pose = msg
        self._count("/planning/lanelet_pose")

    def _on_raw_goal(self, msg: RosPoseStamped) -> None:
        self.latest_raw_goal = msg
        self._count("/goal_pose")

    def _on_route_goal(self, msg: AvgPoseStamped) -> None:
        self.latest_route_goal = msg
        self._count("/planning/goal_pose_snapped")

    def _on_global_path(self, msg: AvgPath) -> None:
        self.global_path_count += 1
        self.global_path_points = len(msg.poses)
        self._count("/planning/global_path_avg")

    def _on_local_path(self, msg: AvgPath) -> None:
        self.local_path_count += 1
        self.local_path_points = len(msg.poses)
        self._count("/planning/local_path")

    def _on_raw_cmd(self, msg: AvgTwist) -> None:
        self._record_twist("/control/cmd_vel_raw", msg)

    def _on_control_cmd(self, msg: AvgTwist) -> None:
        self.latest_control_cmd_abs = twist_abs(msg)
        self._record_twist("/control/cmd_vel", msg)

    def _on_state(self, msg: PlanningState) -> None:
        self.latest_state = msg
        self._count("/planning/state_machine/state")

    def _on_service_state(self, msg: AvgServiceState) -> None:
        # HH_260721 - Preserve transition order while suppressing repeated state events.
        self.latest_service_state = msg
        state_name = str(msg.state_name).strip() or f"STATE_{int(msg.state)}"
        if not self.service_state_names_seen or self.service_state_names_seen[-1] != state_name:
            self.service_state_names_seen.append(state_name)
        self._count("/service/state")

    def _on_mission_source(self, msg: PlanningMissionKey) -> None:
        self.latest_mission_source = msg
        self._count("/planning/state_machine/mission_source")

    def _on_site_status(self, msg: ModuleState) -> None:
        self.latest_site_status = msg
        self._append_status_event(self.site_status_events, msg)
        self._count("/control/camping_site_maneuver_controller/status")

    def _on_drop_maneuver_status(self, msg: ModuleState) -> None:
        self.latest_drop_maneuver_status = msg
        self._append_status_event(self.drop_status_events, msg)
        self._count("/control/drop_zone_maneuver_controller/status")

    def _on_reverse_parking_controller_status(self, msg: ModuleState) -> None:
        self.latest_reverse_parking_controller_status = msg
        self._append_status_event(self.parking_status_events, msg)
        self._count("/parking/reverse_parking_controller/status")

    def _on_gate_status(self, msg: ModuleState) -> None:
        # HH_260721 - Observe CHARGING and DEPARTING_CHARGER transitions directly.
        self.latest_gate_status = msg
        self._append_status_event(self.gate_status_events, msg)
        self._count("/control/cmd_vel_safety_gate/status")

    def _on_route_recovery_status(self, msg: ModuleState) -> None:
        # HH_260807 - Record the automatic recovery owner separately from the
        # gate. A gate release alone does not prove reverse/crab motion occurred.
        self.latest_route_recovery_status = msg
        self._append_status_event(self.route_recovery_status_events, msg)
        self._count("/control/route_safety_recovery_controller/status")

    @staticmethod
    def _append_status_event(events: list[str], msg: ModuleState) -> None:
        state = str(msg.operating_state).strip()
        message = str(msg.message).strip()
        event = f"{state}|{message}"
        if not events or events[-1] != event:
            events.append(event)

    def _on_platform_status(self, msg: AvgPlatformStatus) -> None:
        # HH_260807 - Observe the bridge-owned normalized status. Publishing a
        # competing status here caused CHARGING/DROP_ZONE_WAIT oscillation.
        self.latest_platform_status = msg
        self.fake_platform_charging = bool(msg.is_charging)
        if msg.battery_state_available:
            self.fake_platform_battery_percentage = float(msg.battery_percentage)

    def _on_replan_status(self, msg: AvgString) -> None:
        self.latest_replan_status = msg.data
        if msg.data:
            self.replan_statuses_seen.add(msg.data.split(":", 1)[0])

    def _on_planner_selector(self, msg: RosString) -> None:
        self.latest_planner_selector = msg.data
        if msg.data:
            self.planner_selectors_seen.add(msg.data)

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        self.latest_nav_status = msg
        self._count("/planning/navigate_to_pose/_action/status")

    def _on_follow_status(self, msg: GoalStatusArray) -> None:
        self.latest_follow_status = msg
        self._count("/planning/follow_path/_action/status")

    def spin_for(self, duration_s: float) -> None:
        end = time.monotonic() + max(0.0, duration_s)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, pred: Callable[[], bool], timeout_s: float) -> bool:
        end = time.monotonic() + max(0.0, timeout_s)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        return bool(pred())

    def publish_bool(self, pub, value: bool, repeats: int = 4) -> None:
        msg = AvgBool()
        msg.data = bool(value)
        for _ in range(repeats):
            pub.publish(msg)
            self.spin_for(0.05)

    def publish_engage(self, value: bool) -> None:
        # HH_260720 - The final control gate requires both planning engage and operator arm.
        if value:
            self.publish_bool(self.pub_drive_enable, True)
        self.publish_bool(self.pub_engage, value)
        if not value:
            self.publish_bool(self.pub_drive_enable, False)

    def publish_mission_engage(self, value: bool) -> None:
        self.publish_bool(self.pub_mission_engage, value)

    def set_fake_params(self, **kwargs) -> bool:
        return self.set_node_params(self.param_client, "fake sensor", **kwargs)

    def set_simulated_battery_percentage(self, value: float) -> bool:
        # HH_260807 - Change the raw BMS simulation, then let the platform
        # bridge publish the canonical status exactly as hardware does.
        percentage = max(0.0, min(1.0, float(value)))
        self.fake_platform_battery_percentage = percentage
        return self.set_fake_params(simulated_battery_percentage=percentage)

    def set_gate_params(self, **kwargs) -> bool:
        return self.set_node_params(self.gate_param_client, "planning cmd_vel gate", **kwargs)

    def set_node_params(self, client, label: str, **kwargs) -> bool:
        if not client.wait_for_service(timeout_sec=2.0):
            self.results.append(
                CheckResult(
                    "set_parameters",
                    False,
                    f"{label} set_parameters unavailable",
                )
            )
            return False
        req = SetParameters.Request()
        for key, value in kwargs.items():
            if isinstance(value, bool):
                param = Parameter(key, Parameter.Type.BOOL, value)
            elif isinstance(value, int):
                param = Parameter(key, Parameter.Type.INTEGER, value)
            elif isinstance(value, float):
                param = Parameter(key, Parameter.Type.DOUBLE, value)
            else:
                param = Parameter(key, Parameter.Type.STRING, str(value))
            req.parameters.append(param.to_parameter_msg())
        future = client.call_async(req)
        end = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return False
        return all(result.successful for result in future.result().results)

    def clear_obstacle(self) -> None:
        self.set_fake_params(
            obstacle_reference_frame="robot",
            obstacle_offset=30.0,
            obstacle_lateral_offset=0.0,
            fake_obstacle_cluster_radius_m=self.default_fake_obstacle_cluster_radius_m,
            publish_fake_lidar_obstacle_cloud=True,
            publish_fake_radar_ranges=True,
        )
        self.spin_for(1.2)

    def cancel_all_actions(self) -> None:
        req = CancelGoal.Request()
        req.goal_info.goal_id.uuid = [0] * 16
        req.goal_info.stamp.sec = 0
        req.goal_info.stamp.nanosec = 0
        for client in self.cancel_clients.values():
            if not client.wait_for_service(timeout_sec=1.0):
                continue
            future = client.call_async(req)
            end = time.monotonic() + 2.0
            while rclpy.ok() and not future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
        zero = AvgTwist()
        for _ in range(8):
            self.pub_raw.publish(zero)
            self.spin_for(0.05)
        self.spin_for(0.5)

    def cancel_parking_maneuvers(self) -> None:
        # HH_260701 - Full-run validation executes manual-goal and camping
        # checks in one process. Clear any control/parking phase that may
        # have been triggered by stale /goal_pose during setup probes.
        # HH_260720 - Cancel through generated controller operations; legacy Bool topics are gone.
        self.publish_operation(
            self.pub_site_operation, MotionOperation.CANCEL, repeats=3
        )
        self.publish_operation(
            self.pub_drop_maneuver_operation, MotionOperation.CANCEL, repeats=3
        )
        self.publish_operation(
            self.pub_parking_operation, MotionOperation.CANCEL, repeats=3
        )
        self.spin_for(0.3)

    def publish_operation(self, publisher, operation: int, repeats: int = 3) -> None:
        message = MotionOperation()
        message.header.stamp = self.get_clock().now().to_msg()
        message.operation = int(operation)
        message.source = "sim_validation"
        for _ in range(repeats):
            publisher.publish(message)
            self.spin_for(0.05)

    def reset_cmd_metrics(self) -> None:
        # HH_260720 - Measure the raw candidate and single final control output separately.
        for topic in ("/control/cmd_vel_raw", "/control/cmd_vel"):
            self.max_abs_since[topic] = 0.0

    def cost_grid_max(self, topic: str) -> int:
        grid = self.latest_cost_grids.get(topic)
        if grid is None or not grid.data:
            return -1
        return max(int(v) for v in grid.data)

    def wait_for_cost_grid_update(
        self,
        topic: str,
        start_counts: dict[str, int],
        *,
        min_cost: int = 85,
        timeout_s: float = 2.5,
    ) -> bool:
        return self.wait_for(
            lambda: self.counts.get(topic, 0) > start_counts.get(topic, 0)
            and self.cost_grid_max(topic) >= min_cost,
            timeout_s,
        )

    def make_cmd(self, direction: str) -> AvgTwist:
        msg = AvgTwist()
        if direction == "front":
            msg.linear.x = 0.35
        elif direction == "rear":
            msg.linear.x = -0.18
        elif direction == "left":
            msg.linear.y = 0.16
        elif direction == "right":
            msg.linear.y = -0.16
        elif direction == "rotate":
            msg.angular.z = 0.35
        return msg

    def publish_raw_for(self, cmd: AvgTwist, duration_s: float) -> None:
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            self.pub_raw.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    def goal_ids(self, msg: GoalStatusArray | None) -> set[tuple[int, ...]]:
        if msg is None:
            return set()
        return {tuple(status.goal_info.goal_id.uuid) for status in msg.status_list}

    def terminal_success_seen(
        self,
        msg: GoalStatusArray | None,
        known_goal_ids: set[tuple[int, ...]] | None = None,
    ) -> bool:
        if msg is None:
            return False
        return any(
            status.status == GoalStatus.STATUS_SUCCEEDED
            and (
                known_goal_ids is None
                or tuple(status.goal_info.goal_id.uuid) not in known_goal_ids
            )
            for status in msg.status_list
        )

    def terminal_success_for(
        self,
        msg: GoalStatusArray | None,
        target_goal_ids: set[tuple[int, ...]],
    ) -> bool:
        if msg is None or not target_goal_ids:
            return False
        return any(
            status.status == GoalStatus.STATUS_SUCCEEDED
            and tuple(status.goal_info.goal_id.uuid) in target_goal_ids
            for status in msg.status_list
        )

    def navigation_active(self) -> bool:
        if self.latest_nav_status is None:
            return False
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        return any(status.status in active_states for status in self.latest_nav_status.status_list)

    def rate_for(self, topic: str, start_counts: dict[str, int], duration_s: float) -> float:
        return (self.counts.get(topic, 0) - start_counts.get(topic, 0)) / max(duration_s, 0.1)

    def check_baseline_rates(self) -> None:
        self.clear_obstacle()
        start_counts = dict(self.counts)
        duration_s = 3.0 if not self.quick else 1.5
        self.spin_for(duration_s)
        expected = {
            "/localization/pose": 7.0,
            # HH_260720 - Baseline checks observe the generated planning pose.
            "/planning/lanelet_pose": 7.0,
            "/sensing/imu/data": 7.0,
            "/sensing/lidar/points_filtered": 7.0,
            "/localization/input/wheel_odometry": 7.0,
            "/sensing/cost_grid/radar": 5.0,
            "/planning/cost_grid/inflation": 3.0,
        }
        if self.expect_lidar_cost_grid:
            expected["/sensing/cost_grid/lidar"] = 5.0
        bad = []
        metrics: dict[str, float] = {}
        for topic, min_hz in expected.items():
            hz = self.rate_for(topic, start_counts, duration_s)
            metrics[topic] = round(hz, 2)
            if hz < min_hz:
                bad.append(f"{topic}={hz:.1f}Hz<{min_hz:.1f}")
        self.results.append(
            CheckResult(
                "baseline_hz",
                not bad,
                (
                    "ok"
                    if not bad and self.expect_lidar_cost_grid
                    else (
                        "ok (optional lidar cost grid disabled)"
                        if not bad
                        else ", ".join(bad)
                    )
                ),
                metrics,
            )
        )

    def check_radar_ranges(self) -> None:
        all_ok = True
        detail_parts = []
        metrics: dict[str, float] = {}
        offsets = {"front": 1.0, "left": 0.6, "right": 0.6, "rear": 0.4}
        for direction, topics in RADAR_TOPICS.items():
            self.set_fake_params(
                obstacle_direction=direction,
                obstacle_offset=offsets[direction],
                # HH_260807 - This check measures radar frequency, not source
                # isolation. Keep the base LiDAR/perception health stream alive.
                publish_fake_lidar_obstacle_cloud=True,
                publish_fake_radar_ranges=True,
            )
            self.spin_for(0.4)
            start_counts = dict(self.counts)
            duration_s = 1.4 if not self.quick else 0.8
            self.spin_for(duration_s)
            for topic in topics:
                hz = self.rate_for(topic, start_counts, duration_s)
                metrics[topic] = round(hz, 2)
                if hz < 5.0:
                    all_ok = False
                    detail_parts.append(f"{topic}={hz:.1f}Hz")
        self.results.append(
            CheckResult(
                "radar_direction_hz",
                all_ok,
                "ok" if all_ok else ", ".join(detail_parts),
                metrics,
            )
        )
        self.clear_obstacle()

    def check_gate_stop_matrix(self) -> None:
        self.cancel_all_actions()
        # HH_260701 - This check drives raw body-direction commands directly into
        # the gate. Disable stale-route heading alignment here so the matrix tests
        # cost-stop corridors, not route-following yaw correction.
        self.set_gate_params(enable_route_heading_alignment=False)
        self.publish_engage(True)
        self.publish_mission_engage(False)
        offsets = {"front": 1.0, "left": 0.6, "right": 0.6, "rear": 0.4}
        # HH_260630: Keep LiDAR-only fake obstacles clearly outside ego-clear
        # while staying inside the side/rear lookahead corridors.
        lidar_offsets = {"front": 1.0, "left": 0.95, "right": 0.95, "rear": 0.95}
        directions = ["front", "left", "right", "rear"]
        sources = [
            ("lidar", True, False),
            ("radar", False, True),
            ("combined", True, True),
        ]
        all_ok = True
        detail_parts = []
        metrics: dict[str, float | str] = {}
        for direction in directions:
            for label, lidar, radar in sources:
                start_counts = dict(self.counts)
                self.set_fake_params(
                    obstacle_direction=direction,
                    obstacle_offset=(
                        lidar_offsets[direction] if label == "lidar" else offsets[direction]
                    ),
                    publish_fake_lidar_obstacle_cloud=lidar,
                    publish_fake_radar_ranges=radar,
                )
                wait_topics = []
                if lidar:
                    wait_topics.append("/sensing/cost_grid/lidar")
                if radar:
                    wait_topics.append("/sensing/cost_grid/radar")
                if not wait_topics:
                    wait_topics.append("/planning/cost_grid/inflation")
                for topic in wait_topics:
                    self.wait_for_cost_grid_update(topic, start_counts)
                self.spin_for(0.3)
                self.reset_cmd_metrics()
                self.publish_raw_for(self.make_cmd(direction), 1.2)
                out = self.max_abs_since.get("/control/cmd_vel", 0.0)
                metrics[f"{direction}_{label}_control_cmd_max"] = round(out, 3)
                ok = out <= 0.03
                if not ok:
                    all_ok = False
                    detail_parts.append(f"{direction}/{label} leaked {out:.3f}")
                self.clear_obstacle()
        self.set_gate_params(enable_route_heading_alignment=True)
        self.results.append(
            CheckResult(
                "directional_cost_stop",
                all_ok,
                "ok" if all_ok else "; ".join(detail_parts),
                metrics,
            )
        )

    def publish_initialpose(self, pose) -> None:
        # HH_260720 - Convert the internal Avg pose only at the RViz reset boundary.
        msg = RosPoseWithCovarianceStamped()
        msg.header.stamp = pose.header.stamp
        msg.header.frame_id = pose.header.frame_id
        self.copy_pose(msg.pose.pose, pose.pose)
        for _ in range(4):
            self.pub_initialpose.publish(msg)
            self.spin_for(0.05)

    def publish_goal_pose(self, pose: RosPoseStamped) -> None:
        for _ in range(4):
            msg = RosPoseStamped()
            msg.header.frame_id = pose.header.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            self.copy_pose(msg.pose, pose.pose)
            self.pub_goal.publish(msg)
            self.spin_for(0.05)

    def publish_prepare_goal_pose(self, pose: RosPoseStamped) -> None:
        for _ in range(4):
            msg = AvgPoseStamped()
            msg.header.frame_id = pose.header.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            self.copy_pose(msg.pose, pose.pose)
            self.pub_prepare_goal.publish(msg)
            self.spin_for(0.05)

    @staticmethod
    def copy_pose(target, source) -> None:
        target.position.x = source.position.x
        target.position.y = source.position.y
        target.position.z = source.position.z
        target.orientation.x = source.orientation.x
        target.orientation.y = source.orientation.y
        target.orientation.z = source.orientation.z
        target.orientation.w = source.orientation.w

    def prepare_camping_start_pose(self, goal_pose: RosPoseStamped) -> bool:
        if not self.camping_prepare_near_route:
            return True
        before = self.latest_route_goal
        self.publish_engage(False)
        self.publish_mission_engage(False)
        self.cancel_all_actions()
        # HH_260630: Probe goal_snapper through its auxiliary input so the
        # preparation step does not look like a user/UI camping goal to
        # camping_site_maneuver_controller or the state-machine raw-goal listener.
        self.publish_prepare_goal_pose(goal_pose)

        def route_goal_updated() -> bool:
            if self.latest_route_goal is None:
                return False
            if before is None:
                return True
            return math.hypot(
                self.latest_route_goal.pose.position.x - before.pose.position.x,
                self.latest_route_goal.pose.position.y - before.pose.position.y,
            ) > 0.05

        self.wait_for(route_goal_updated, 5.0)
        route_goal = self.latest_route_goal
        if route_goal is None:
            return False
        yaw = yaw_from_quat(route_goal.pose.orientation)
        start = RosPoseStamped()
        start.header.stamp = self.get_clock().now().to_msg()
        start.header.frame_id = route_goal.header.frame_id or "map"
        offset_m = max(0.0, abs(self.camping_start_offset_m))
        # HH_260630: Keep the camping end-to-end test deterministic by starting
        # at the snapped lanelet entry instead of from wherever a previous sim
        # run left the fake vehicle; nonzero offsets can land on the opposite
        # directed lanelet and turn a handoff test into a full route test.
        start.pose.position.x = route_goal.pose.position.x - offset_m * math.cos(yaw)
        start.pose.position.y = route_goal.pose.position.y - offset_m * math.sin(yaw)
        start.pose.position.z = route_goal.pose.position.z
        start.pose.orientation.x = route_goal.pose.orientation.x
        start.pose.orientation.y = route_goal.pose.orientation.y
        start.pose.orientation.z = route_goal.pose.orientation.z
        start.pose.orientation.w = route_goal.pose.orientation.w
        self.publish_initialpose(start)
        self.cancel_all_actions()
        prepared = self.wait_for(
            lambda: self.latest_pose is not None
            and math.hypot(
                self.latest_pose.pose.position.x - start.pose.position.x,
                self.latest_pose.pose.position.y - start.pose.position.y,
            )
            < 1.0,
            5.0,
        )
        self.cancel_parking_maneuvers()
        return prepared

    def _candidate_camping_site_files(self) -> list[str]:
        candidates = []
        if self.camping_sites_yaml.strip():
            candidates.append(self.camping_sites_yaml.strip())
        try:
            planning_share = get_package_share_directory("camrod_planning")
            candidates.extend(
                [
                    # HH_260720 - Follow the active park-map fallback before legacy profile files.
                    os.path.join(planning_share, "config", "camping_sites.yaml"),
                    os.path.join(planning_share, "config", "camping_sites (copy_park).yaml"),
                    os.path.join(planning_share, "config", "camping_sites (copy_c_track).yaml"),
                ]
            )
        except Exception:
            pass
        candidates.extend(
            [
                os.path.join(
                    os.getcwd(),
                    "camrod_planning",
                    "config",
                    "camping_sites (copy_c_track).yaml",
                ),
                os.path.join(os.getcwd(), "camrod_planning", "config", "camping_sites.yaml"),
            ]
        )
        return candidates

    # HH_260721 - Resolve the same operational campsite record consumed by UI, planning, and control.
    def load_camping_site_config(self, mission_key: str) -> dict | None:
        for path in self._candidate_camping_site_files():
            if not path or not os.path.exists(path):
                continue
            try:
                with open(path, "r", encoding="utf-8") as stream:
                    data = yaml.safe_load(stream) or {}
            except Exception as exc:
                self.get_logger().warn(f"failed to read camping site file {path}: {exc}")
                continue
            for site in data.get("camping_sites", []) or []:
                if str(site.get("type", "")).strip() != mission_key:
                    continue
                return site
        return None

    def load_camping_goal(self, mission_key: str) -> RosPoseStamped | None:
        site = self.load_camping_site_config(mission_key)
        if site is not None:
            # HH_260630: Sim UI validation mirrors ui_backend_node behavior:
            # publish semantic mission_key first, then its operational goal pose.
            # HH_260721 - Prefer a roadside service pose for map-annotated inaccessible sites.
            pose = RosPoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = str(site.get("frame_id", "map"))
            pose.pose.position.x = float(site.get("service_x", site.get("x", 0.0)))
            pose.pose.position.y = float(site.get("service_y", site.get("y", 0.0)))
            pose.pose.position.z = float(site.get("service_z", site.get("z", 0.0)))
            pose.pose.orientation = quat_from_yaw(
                math.radians(
                    float(site.get("service_yaw_deg", site.get("yaw_deg", 0.0)))
                )
            )
            return pose
        return None

    def publish_charging_recall_request(
        self,
        recall_key: str,
        goal_pose: RosPoseStamped,
        source: str,
    ) -> None:
        if self.charging_recall_via_ui:
            # HH_260724 - Exercise the UI battery dispatch gate before the route is released.
            site_number = recall_key.removeprefix("camping_site_")
            destination = UiDestinationCommand()
            destination.header.stamp = self.get_clock().now().to_msg()
            destination.site = f"B{site_number}"
            destination.run = True
            destination.mission_key = recall_key
            destination.source = source
            # HH_260807 - UiDestinationCommand uses a reliable local ROS topic.
            # Publish one user intent; retries are not separate missions and can
            # otherwise restart the bounded charger-exit handoff.
            self.pub_ui_destination.publish(destination)
            self.spin_for(0.10)
            return

        recall_msg = PlanningMissionKey()
        recall_msg.header.stamp = self.get_clock().now().to_msg()
        recall_msg.mission_key = recall_key
        recall_msg.source = source
        recall_msg.publish_route_goal = False
        self.publish_engage(True)
        self.publish_mission_engage(True)
        for _ in range(4):
            self.pub_mission_key.publish(recall_msg)
            self.spin_for(0.05)
        self.publish_goal_pose(goal_pose)

    def check_manual_goal(self) -> None:
        self.cancel_all_actions()
        self.clear_obstacle()
        self.publish_engage(True)
        if not self.wait_for(lambda: self.latest_lanelet_pose is not None, 5.0):
            self.results.append(CheckResult("manual_goal_nav", False, "missing lanelet pose"))
            return
        base = self.latest_lanelet_pose or self.latest_pose
        if base is None:
            self.results.append(CheckResult("manual_goal_nav", False, "missing pose"))
            return
        self.publish_initialpose(base)
        self.spin_for(0.8)
        yaw = yaw_from_quat(base.pose.orientation)
        goal = RosPoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = base.pose.position.x + self.manual_goal_distance_m * math.cos(yaw)
        goal.pose.position.y = base.pose.position.y + self.manual_goal_distance_m * math.sin(yaw)
        goal.pose.position.z = base.pose.position.z
        goal.pose.orientation = quat_from_yaw(yaw)
        base_global = self.global_path_count
        base_local = self.local_path_count
        known_nav_goals = self.goal_ids(self.latest_nav_status)
        start_pose = self.latest_pose
        self.reset_cmd_metrics()
        self.publish_goal_pose(goal)
        target_goal_ids: set[tuple[int, ...]] = set()

        def observe_new_goal() -> bool:
            new_ids = self.goal_ids(self.latest_nav_status) - known_nav_goals
            if new_ids:
                target_goal_ids.update(new_ids)
                return True
            return False

        if not self.wait_for(observe_new_goal, 5.0):
            self.publish_engage(False)
            self.cancel_all_actions()
            self.results.append(
                CheckResult(
                    "manual_goal_nav",
                    False,
                    "new Nav2 goal id was not observed",
                    {
                        "global_path_points": self.global_path_points,
                        "local_path_points": self.local_path_points,
                        "cmd_max": round(self.max_abs_since.get("/control/cmd_vel", 0.0), 3),
                        "moved_m": 0.0,
                        "succeeded": False,
                    },
                )
            )
            return

        start = time.monotonic()
        succeeded = False
        moved_m = 0.0
        while rclpy.ok() and (time.monotonic() - start) < self.manual_goal_timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if start_pose is not None and self.latest_pose is not None:
                moved_m = math.hypot(
                    self.latest_pose.pose.position.x - start_pose.pose.position.x,
                    self.latest_pose.pose.position.y - start_pose.pose.position.y,
                )
            new_ids = self.goal_ids(self.latest_nav_status) - known_nav_goals
            if new_ids:
                target_goal_ids.update(new_ids)
            if self.terminal_success_for(self.latest_nav_status, target_goal_ids):
                succeeded = True
                break
        self.publish_engage(False)
        self.cancel_all_actions()
        global_new = self.global_path_count > base_global
        local_new = self.local_path_count > base_local
        cmd_max = self.max_abs_since.get("/control/cmd_vel", 0.0)
        ok = bool(global_new and local_new and cmd_max > 0.03 and moved_m > 0.5 and succeeded)
        self.results.append(
            CheckResult(
                "manual_goal_nav",
                ok,
                "ok"
                if ok
                else (
                    f"global={global_new} local={local_new} cmd={cmd_max:.3f} "
                    f"moved={moved_m:.2f} succeeded={succeeded}"
                ),
                {
                    "global_path_points": self.global_path_points,
                    "local_path_points": self.local_path_points,
                    "cmd_max": round(cmd_max, 3),
                    "moved_m": round(moved_m, 2),
                    "succeeded": succeeded,
                },
            )
        )

    def check_obstacle_replan(self) -> None:
        self.cancel_all_actions()
        self.cancel_parking_maneuvers()
        self.clear_obstacle()
        self.publish_engage(True)
        self.publish_mission_engage(False)
        if not self.wait_for(lambda: self.latest_lanelet_pose is not None, 5.0):
            self.results.append(CheckResult("obstacle_replan", False, "missing lanelet pose"))
            return
        base = self.latest_lanelet_pose or self.latest_pose
        if base is None:
            self.results.append(CheckResult("obstacle_replan", False, "missing pose"))
            return

        if math.isfinite(self.obstacle_replan_start_x) and math.isfinite(
            self.obstacle_replan_start_y
        ):
            # HH_260807 - Reproduce the fallback on a map-measured wide lane
            # instead of inheriting whichever narrow pose a previous run left.
            seeded = RosPoseStamped()
            seeded.header.stamp = self.get_clock().now().to_msg()
            seeded.header.frame_id = "map"
            seeded.pose.position.x = self.obstacle_replan_start_x
            seeded.pose.position.y = self.obstacle_replan_start_y
            seeded.pose.orientation = quat_from_yaw(
                math.radians(self.obstacle_replan_start_yaw_deg)
            )
            self.publish_initialpose(seeded)
            self.spin_for(1.0)
            base = self.latest_pose or seeded

        self.publish_initialpose(base)
        self.spin_for(0.8)
        yaw = yaw_from_quat(base.pose.orientation)
        goal = RosPoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = (
            base.pose.position.x + self.obstacle_replan_goal_distance_m * math.cos(yaw)
        )
        goal.pose.position.y = (
            base.pose.position.y + self.obstacle_replan_goal_distance_m * math.sin(yaw)
        )
        goal.pose.position.z = base.pose.position.z
        goal.pose.orientation = quat_from_yaw(yaw)

        self.latest_replan_status = ""
        self.latest_planner_selector = ""
        self.replan_statuses_seen.clear()
        self.planner_selectors_seen.clear()
        base_global = self.global_path_count
        base_local = self.local_path_count
        self.reset_cmd_metrics()
        self.publish_goal_pose(goal)
        route_ready = self.wait_for(
            lambda: self.global_path_count > base_global and self.navigation_active(),
            12.0,
        )
        obstacle_world_x = 0.0
        obstacle_world_y = 0.0
        if route_ready:
            obstacle_pose = self.latest_pose or base
            obstacle_yaw = yaw_from_quat(obstacle_pose.pose.orientation)
            obstacle_world_x = (
                obstacle_pose.pose.position.x
                + self.obstacle_replan_obstacle_offset_m * math.cos(obstacle_yaw)
            )
            obstacle_world_y = (
                obstacle_pose.pose.position.y
                + self.obstacle_replan_obstacle_offset_m * math.sin(obstacle_yaw)
            )
            # HH_260807 - Keep the obstacle fixed in map coordinates so an
            # alternate SmacLattice path can pass it instead of chasing a point
            # that moves with the simulated robot.
            self.set_fake_params(
                obstacle_reference_frame="map",
                obstacle_world_x=obstacle_world_x,
                obstacle_world_y=obstacle_world_y,
                obstacle_direction="front",
                obstacle_offset=self.obstacle_replan_obstacle_offset_m,
                obstacle_lateral_offset=0.0,
                fake_obstacle_cluster_radius_m=self.obstacle_replan_cluster_radius_m,
                publish_fake_lidar_obstacle_cloud=True,
                publish_fake_radar_ranges=True,
            )
        start = time.monotonic()
        seen_blocked = False
        seen_fallback = False
        seen_fallback_path = False
        seen_stop = False
        seen_motion_around_obstacle = False
        seen_failed_hold = False
        resumed_after_clear = False
        obstacle_clear_requested = False
        pose_at_clear = None
        blocked_seen_at: float | None = None
        fallback_global_base: int | None = None
        start_pose = self.latest_pose
        while route_ready and rclpy.ok() and (time.monotonic() - start) < self.obstacle_replan_timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_replan_status.startswith("BLOCKED"):
                seen_blocked = True
                seen_stop = seen_stop or self.latest_control_cmd_abs <= 0.03
                if blocked_seen_at is None:
                    blocked_seen_at = time.monotonic()
            if self.latest_replan_status.startswith("BLOCKED_REPLAN_FAILED_HOLD"):
                seen_failed_hold = True
                if self.obstacle_replan_expect_safe_hold and not obstacle_clear_requested:
                    # HH_260807 - A failed Smac preflight must leave LaneletRoute
                    # active. Remove the obstacle only after observing the latch,
                    # then prove that the original mission resumes without a new goal.
                    pose_at_clear = self.latest_pose
                    self.clear_obstacle()
                    obstacle_clear_requested = True
            fallback_now = (
                seen_fallback
                or self.latest_planner_selector == "Smac2D"
                or "Smac2D" in self.planner_selectors_seen
                or self.latest_planner_selector == "SmacLattice"
                or "SmacLattice" in self.planner_selectors_seen
            )
            if fallback_now and not seen_fallback:
                fallback_global_base = self.global_path_count
            seen_fallback = fallback_now
            if fallback_global_base is not None and self.global_path_count > fallback_global_base:
                seen_fallback_path = True
            moved_m = (
                math.hypot(
                    self.latest_pose.pose.position.x - start_pose.pose.position.x,
                    self.latest_pose.pose.position.y - start_pose.pose.position.y,
                )
                if self.latest_pose is not None and start_pose is not None
                else 0.0
            )
            seen_motion_around_obstacle = (
                seen_motion_around_obstacle
                or (
                    seen_fallback_path
                    and self.latest_control_cmd_abs > 0.03
                    and moved_m > 0.20
                )
            )
            if obstacle_clear_requested and pose_at_clear is not None and self.latest_pose is not None:
                resumed_after_clear = resumed_after_clear or (
                    self.latest_control_cmd_abs > 0.03
                    and math.hypot(
                        self.latest_pose.pose.position.x - pose_at_clear.pose.position.x,
                        self.latest_pose.pose.position.y - pose_at_clear.pose.position.y,
                    )
                    > 0.20
                )
            if self.obstacle_replan_expect_fallback and seen_motion_around_obstacle:
                break
            if self.obstacle_replan_expect_safe_hold and resumed_after_clear:
                break
            if (
                not self.obstacle_replan_expect_fallback
                and not self.obstacle_replan_expect_safe_hold
                and blocked_seen_at is not None
                and (time.monotonic() - blocked_seen_at) >= 2.0
            ):
                break

        cmd_max = self.max_abs_since.get("/control/cmd_vel", 0.0)
        moved_m = (
            math.hypot(
                self.latest_pose.pose.position.x - start_pose.pose.position.x,
                self.latest_pose.pose.position.y - start_pose.pose.position.y,
            )
            if self.latest_pose is not None and start_pose is not None
            else 0.0
        )
        self.clear_obstacle()
        self.publish_engage(False)
        self.cancel_all_actions()
        self.cancel_parking_maneuvers()
        if self.obstacle_replan_expect_safe_hold:
            ok = bool(
                route_ready
                and seen_blocked
                and seen_stop
                and seen_failed_hold
                and not seen_fallback
                and resumed_after_clear
            )
        elif self.obstacle_replan_expect_fallback:
            ok = bool(
                route_ready
                and seen_blocked
                and seen_stop
                and seen_fallback
                and seen_fallback_path
                and seen_motion_around_obstacle
            )
        else:
            ok = bool(route_ready and seen_blocked and not seen_fallback)
        self.results.append(
            CheckResult(
                "obstacle_replan",
                ok,
                "ok"
                if ok
                else (
                    f"route_ready={route_ready} blocked={seen_blocked} "
                    f"stopped={seen_stop} fallback={seen_fallback} "
                    f"fallback_path={seen_fallback_path} moved={moved_m:.2f} "
                    f"status={self.latest_replan_status}"
                ),
                {
                    "route_ready": route_ready,
                    "global_path_points": self.global_path_points,
                    "local_path_points": self.local_path_points,
                    "blocked_seen": seen_blocked,
                    "stop_seen": seen_stop,
                    "fallback_expected": self.obstacle_replan_expect_fallback,
                    "safe_hold_expected": self.obstacle_replan_expect_safe_hold,
                    "fallback_selector_seen": seen_fallback,
                    "fallback_path_seen": seen_fallback_path,
                    "fallback_failed_hold_seen": seen_failed_hold,
                    "motion_around_obstacle_seen": seen_motion_around_obstacle,
                    "resumed_after_obstacle_clear": resumed_after_clear,
                    "moved_m": round(moved_m, 3),
                    "obstacle_world_x": round(obstacle_world_x, 3),
                    "obstacle_world_y": round(obstacle_world_y, 3),
                    "latest_replan_status": self.latest_replan_status,
                    "latest_planner_selector": self.latest_planner_selector,
                    "replan_statuses_seen": ",".join(sorted(self.replan_statuses_seen)),
                    "planner_selectors_seen": ",".join(sorted(self.planner_selectors_seen)),
                    "cmd_max": round(cmd_max, 3),
                    "global_path_updates": self.global_path_count - base_global,
                    "local_path_updates": self.local_path_count - base_local,
                },
            )
        )

    def check_camping_site_smoke(self) -> None:
        self.cancel_all_actions()
        self.cancel_parking_maneuvers()
        self.clear_obstacle()
        self.publish_engage(True)
        self.publish_mission_engage(True)
        mission_key = self.camping_mission_key.strip() or "camping_site_1"
        camping_site_config = self.load_camping_site_config(mission_key) or {}
        # HH_260721 - Assert the phase contract selected by the campsite service policy.
        camping_service_mode = (
            str(camping_site_config.get("service_mode", "turnaround")).strip().lower()
            or "turnaround"
        )
        goal_pose = self.load_camping_goal(mission_key)
        if goal_pose is None:
            self.results.append(
                CheckResult(
                    "camping_site_smoke",
                    False,
                    f"missing camping goal for {mission_key}",
                )
            )
            return
        if not self.prepare_camping_start_pose(goal_pose):
            self.results.append(
                CheckResult(
                    "camping_site_smoke",
                    False,
                    "failed to prepare camping start pose near snapped route goal",
                )
            )
            return

        # HH_260630: prepare_camping_start_pose intentionally disarms planning
        # while it probes the snapped route and seeds /initialpose. Re-arm both
        # gates before mirroring the UI mission_key + goal publish sequence.
        self.cancel_parking_maneuvers()
        self.publish_engage(True)
        self.publish_mission_engage(True)
        self.spin_for(0.3)

        msg = PlanningMissionKey()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_key = mission_key
        msg.source = "sim_validation"
        msg.publish_route_goal = False
        start = time.monotonic()
        base_global = self.global_path_count
        base_local = self.local_path_count
        known_nav_goals = self.goal_ids(self.latest_nav_status)
        self.reset_cmd_metrics()
        for _ in range(4):
            self.pub_mission_key.publish(msg)
            self.spin_for(0.05)
        self.publish_goal_pose(goal_pose)
        seen_site_phase = False
        # HH_260721 - Track the retrace-heading verification separately from the site turn.
        seen_align_retrace_yaw = False
        # HH_260721 - Roadside stops must rotate only after returning to the lanelet snap pose.
        seen_align_return_route_yaw = False
        seen_rotate_180 = False
        seen_crab_out = False
        seen_done = False
        seen_drop_zone_return = False
        seen_drop_maneuver_alignment = False
        seen_reverse_parking_controller_started = False
        seen_reverse_parking_controller_parked = False
        seen_drop_sequence_error = False
        seen_reverse_wait_for_charging = False
        seen_gate_charging_state = False
        seen_service_waiting_for_return_request = False
        seen_service_waiting_for_charging = False
        seen_service_charging = False
        charging_recall_requested = False
        charging_recall_request_time = 0.0
        charging_recall_goal_pose: AvgPoseStamped | None = None
        charging_recall_key_seen = False
        charging_recall_departure_state_seen = False
        charging_recall_service_departure_seen = False
        charging_recall_exit_straight_seen = False
        charging_recall_exit_alignment_seen = False
        charging_recall_cmd_released = False
        charging_recall_disconnect_seen = False
        charging_recall_site_arrived = False
        charging_recall_global_path_base = 0
        charging_recall_local_path_base = 0
        charging_recall_battery_gate_enabled = (
            self.run_charging_recall
            and self.run_charging_recall_battery_gate
        )
        charging_recall_low_battery_requested = False
        charging_recall_low_battery_done = False
        charging_recall_low_battery_departure_seen = False
        charging_recall_low_battery_cmd_released = False
        charging_recall_low_battery_global_base = 0
        charging_recall_low_battery_local_base = 0
        charging_recall_low_battery_global_delta = 0
        charging_recall_low_battery_local_delta = 0
        charging_recall_low_battery_request_time = 0.0
        charging_recall_low_battery_ok = not charging_recall_battery_gate_enabled
        charging_recall_recovered_battery_percentage = self.fake_platform_battery_percentage
        low_battery_finish_return_enabled = (
            self.run_low_battery_finish_then_return
            and self.simulate_platform_status
            and self.camping_wait_drop_zone
        )
        low_battery_finish_triggered = False
        low_battery_finish_wait_seen = False
        low_battery_finish_wait_start = 0.0
        low_battery_finish_no_auto_return = True
        low_battery_finish_manual_return_sent = False
        low_battery_finish_return_started = False
        low_battery_finish_recovered = False
        reached_nav = False
        seen_goal_reached_state = False
        seen_site_key = False
        state_labels: set[str] = set()
        scenario_labels: set[str] = set()
        site_status_messages: set[str] = set()
        # HH_260806 - Retain transition order for auditable campsite sequence assets.
        site_phase_sequence: list[str] = []
        drop_maneuver_status_messages: set[str] = set()
        reverse_parking_controller_status_messages: set[str] = set()
        while rclpy.ok() and (time.monotonic() - start) < self.camping_timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            site_msg = self.latest_site_status.message if self.latest_site_status else ""
            drop_maneuver_msg = (
                self.latest_drop_maneuver_status.message
                if self.latest_drop_maneuver_status
                else ""
            )
            reverse_parking_controller_msg = (
                self.latest_reverse_parking_controller_status.message
                if self.latest_reverse_parking_controller_status
                else ""
            )
            gate_status_msg = (
                self.latest_gate_status.message if self.latest_gate_status else ""
            )
            service_state_name = (
                str(self.latest_service_state.state_name).strip()
                if self.latest_service_state
                else ""
            )
            state_label = self.latest_state.label if self.latest_state else ""
            active_key = self.latest_state.active_mission_key if self.latest_state else ""
            scenario_label = self.latest_state.scenario_label if self.latest_state else ""
            if state_label:
                state_labels.add(state_label)
            if scenario_label:
                scenario_labels.add(scenario_label)
            if site_msg:
                site_status_messages.add(site_msg)
            site_phase = (
                str(self.latest_site_status.operating_state).strip()
                if self.latest_site_status
                else ""
            )
            if (
                site_phase
                and site_phase != "IDLE"
                and (not site_phase_sequence or site_phase_sequence[-1] != site_phase)
            ):
                site_phase_sequence.append(site_phase)
            if drop_maneuver_msg:
                drop_maneuver_status_messages.add(drop_maneuver_msg)
            if reverse_parking_controller_msg:
                reverse_parking_controller_status_messages.add(reverse_parking_controller_msg)
            seen_goal_reached_state = seen_goal_reached_state or state_label == "GOAL_REACHED"
            seen_site_key = seen_site_key or active_key.startswith("camping_site_")
            seen_drop_zone_return = (
                seen_drop_zone_return
                or active_key == "drop_zone"
                or scenario_label == "RETURN_TO_DROP_ZONE"
            )
            if any(token in site_msg for token in ("CRAB_IN", "ROTATE_180", "WAIT_RETURN")):
                seen_site_phase = True
            if (
                low_battery_finish_return_enabled
                and not low_battery_finish_triggered
                and (
                    seen_site_phase
                    or service_state_name
                    in {"MOVING_TO_SITE", "SITE_ENTRY", "UNLOAD_WAIT"}
                )
            ):
                self.set_simulated_battery_percentage(
                    self.low_battery_mission_percentage
                )
                self.spin_for(0.25)
                low_battery_finish_triggered = True
            if (
                "WAIT_RETURN" in site_msg
                and service_state_name == "WAITING_FOR_RETURN_REQUEST"
            ):
                # HH_260721 - Do not leave the site before the public return-wait state is observable.
                seen_service_waiting_for_return_request = True
                if self.camping_stop_at_wait_return:
                    # HH_260806 - B11-B13 return geometry remains field-pending;
                    # end the arrival-only probe before any departure motion.
                    break
                if low_battery_finish_return_enabled:
                    if not low_battery_finish_wait_seen:
                        low_battery_finish_wait_seen = True
                        low_battery_finish_wait_start = time.monotonic()
                    observe_s = max(0.0, self.low_battery_return_manual_observe_s)
                    if (
                        not low_battery_finish_manual_return_sent
                        and time.monotonic() >= low_battery_finish_wait_start + observe_s
                    ):
                        self.publish_operation(
                            self.pub_site_operation,
                            MotionOperation.RETURN,
                            repeats=1,
                        )
                        low_battery_finish_manual_return_sent = True
                else:
                    self.publish_operation(
                        self.pub_site_operation, MotionOperation.RETURN, repeats=1
                    )
            # HH_260701 - RETURNING can be emitted before the campsite exit is
            # actually complete. Require the concrete site maneuver phases so
            # the smoke test catches engage/gate issues that stop CRAB_OUT.
            # HH_260721 - Capture only the initial site's turn policy before a charging recall starts.
            if not seen_done:
                seen_rotate_180 = seen_rotate_180 or "ROTATE_180" in site_msg
                seen_align_retrace_yaw = (
                    seen_align_retrace_yaw or "ALIGN_RETRACE_YAW" in site_msg
                )
                # HH_260721 - Capture the on-lane turn used by constrained roadside sites.
                seen_align_return_route_yaw = (
                    seen_align_return_route_yaw or "ALIGN_RETURN_ROUTE_YAW" in site_msg
                )
            seen_crab_out = seen_crab_out or "CRAB_OUT" in site_msg
            seen_done = seen_done or "DONE" in site_msg
            low_battery_return_started_now = (
                "CRAB_OUT" in site_msg
                or service_state_name == "RETURN_WITH_CARGO"
            )
            if (
                low_battery_finish_return_enabled
                and low_battery_finish_wait_seen
                and low_battery_return_started_now
                and not low_battery_finish_manual_return_sent
            ):
                low_battery_finish_no_auto_return = False
            low_battery_finish_return_started = (
                low_battery_finish_return_started
                or (
                    low_battery_finish_manual_return_sent
                    and low_battery_return_started_now
                )
            )
            # HH_260720 - Validate drop-zone alignment and reverse parking as
            # separate control and parking responsibilities.
            seen_drop_maneuver_alignment = (
                seen_drop_maneuver_alignment
                or "ALIGN_PARKING_YAW" in drop_maneuver_msg
            )
            seen_reverse_parking_controller_started = (
                seen_reverse_parking_controller_started
                or "REVERSE_APPROACH" in reverse_parking_controller_msg
            )
            seen_reverse_parking_controller_parked = (
                seen_reverse_parking_controller_parked
                or "PARKED" in reverse_parking_controller_msg
            )
            # HH_260721 - Emulate charger contact only after reverse parking reaches its stop pose.
            if "WAIT_FOR_CHARGING" in reverse_parking_controller_msg:
                seen_reverse_wait_for_charging = True
            seen_gate_charging_state = (
                seen_gate_charging_state or "state=CHARGING" in gate_status_msg
            )
            # HH_260721 - Assert externally visible wait and charging states, not only node phases.
            seen_service_waiting_for_return_request = (
                seen_service_waiting_for_return_request
                or service_state_name == "WAITING_FOR_RETURN_REQUEST"
            )
            seen_service_waiting_for_charging = (
                seen_service_waiting_for_charging
                or service_state_name == "WAITING_FOR_CHARGING"
            )
            seen_service_charging = (
                seen_service_charging or service_state_name == "CHARGING"
            )
            if (
                low_battery_finish_return_enabled
                and seen_service_charging
                and not low_battery_finish_recovered
            ):
                self.set_simulated_battery_percentage(
                    charging_recall_recovered_battery_percentage
                )
                self.spin_for(0.25)
                low_battery_finish_recovered = True
            # HH_260721 - Capture UI-visible charger departure and both bounded exit phases.
            charging_recall_service_departure_seen = (
                charging_recall_service_departure_seen
                or service_state_name == "DEPARTING_CHARGER"
            )
            charging_recall_exit_straight_seen = (
                charging_recall_exit_straight_seen
                or "EXIT_STRAIGHT" in drop_maneuver_msg
            )
            charging_recall_exit_alignment_seen = (
                charging_recall_exit_alignment_seen
                or "ALIGN_EXIT_YAW" in drop_maneuver_msg
            )
            seen_drop_sequence_error = (
                seen_drop_sequence_error
                or "ERROR" in drop_maneuver_msg
                or "ERROR" in reverse_parking_controller_msg
            )

            if (
                charging_recall_battery_gate_enabled
                and seen_reverse_parking_controller_parked
                and seen_gate_charging_state
                and not charging_recall_low_battery_requested
            ):
                recall_key = self.charging_recall_mission_key.strip() or mission_key
                charging_recall_goal_pose = self.load_camping_goal(recall_key)
                if charging_recall_goal_pose is None:
                    seen_drop_sequence_error = True
                    break
                self.set_simulated_battery_percentage(
                    self.charging_recall_low_battery_percentage
                )
                self.spin_for(0.25)
                self.reset_cmd_metrics()
                charging_recall_low_battery_global_base = self.global_path_count
                charging_recall_low_battery_local_base = self.local_path_count
                self.publish_charging_recall_request(
                    recall_key,
                    charging_recall_goal_pose,
                    "sim_charging_recall_low_battery",
                )
                charging_recall_low_battery_requested = True
                charging_recall_low_battery_request_time = time.monotonic()

            if (
                charging_recall_low_battery_requested
                and not charging_recall_low_battery_done
            ):
                charging_recall_low_battery_departure_seen = (
                    charging_recall_low_battery_departure_seen
                    or service_state_name == "DEPARTING_CHARGER"
                    or "state=DEPARTING_CHARGER" in gate_status_msg
                )
                charging_recall_low_battery_cmd_released = (
                    charging_recall_low_battery_cmd_released
                    or self.max_abs_since.get("/control/cmd_vel", 0.0) > 0.03
                )
                observe_s = max(0.1, self.charging_recall_battery_gate_observe_s)
                if time.monotonic() > charging_recall_low_battery_request_time + observe_s:
                    charging_recall_low_battery_global_delta = (
                        self.global_path_count - charging_recall_low_battery_global_base
                    )
                    charging_recall_low_battery_local_delta = (
                        self.local_path_count - charging_recall_low_battery_local_base
                    )
                    route_goal_blocked = charging_recall_low_battery_global_delta == 0
                    ui_boundary_ok = self.charging_recall_via_ui and route_goal_blocked
                    motion_boundary_ok = (
                        not charging_recall_low_battery_departure_seen
                        and not charging_recall_low_battery_cmd_released
                    )
                    charging_recall_low_battery_ok = bool(
                        motion_boundary_ok
                        and (
                            ui_boundary_ok
                            or not self.charging_recall_via_ui
                        )
                    )
                    self.set_simulated_battery_percentage(
                        charging_recall_recovered_battery_percentage
                    )
                    self.spin_for(0.25)
                    self.reset_cmd_metrics()
                    charging_recall_low_battery_done = True

            # HH_260721 - Recall only after both parking and the gate confirm the simulated charge.
            if (
                self.run_charging_recall
                and seen_reverse_parking_controller_parked
                and seen_gate_charging_state
                and not charging_recall_requested
                and (
                    not charging_recall_battery_gate_enabled
                    or charging_recall_low_battery_done
                )
            ):
                recall_key = self.charging_recall_mission_key.strip() or mission_key
                charging_recall_goal_pose = self.load_camping_goal(recall_key)
                if charging_recall_goal_pose is None:
                    seen_drop_sequence_error = True
                    break
                self.reset_cmd_metrics()
                charging_recall_global_path_base = self.global_path_count
                charging_recall_local_path_base = self.local_path_count
                self.publish_charging_recall_request(
                    recall_key,
                    charging_recall_goal_pose,
                    "sim_charging_recall_ui"
                    if self.charging_recall_via_ui
                    else "sim_charging_recall",
                )
                charging_recall_requested = True
                charging_recall_request_time = time.monotonic()

            if charging_recall_requested:
                recall_key = self.charging_recall_mission_key.strip() or mission_key
                charging_recall_key_seen = (
                    charging_recall_key_seen or active_key == recall_key
                )
                charging_recall_departure_state_seen = (
                    charging_recall_departure_state_seen
                    or "state=DEPARTING_CHARGER" in gate_status_msg
                )
                # HH_260721 - The first admitted command represents physical charger separation.
                if (
                    self.fake_platform_charging
                    and self.max_abs_since.get("/control/cmd_vel", 0.0) > 0.03
                ):
                    charging_recall_cmd_released = True
                charging_recall_disconnect_seen = (
                    charging_recall_disconnect_seen
                    or (charging_recall_cmd_released and not self.fake_platform_charging)
                )
                post_recall_site_phase = any(
                    token in site_msg
                    for token in ("CRAB_IN", "REVERSE_IN", "ROTATE_180", "WAIT_RETURN")
                )
                charging_recall_site_arrived = (
                    charging_recall_site_arrived
                    or (
                        time.monotonic() > charging_recall_request_time + 1.0
                        and charging_recall_key_seen
                        and post_recall_site_phase
                    )
                )
            if self.terminal_success_seen(self.latest_nav_status, known_nav_goals):
                reached_nav = True
            if seen_site_phase and seen_crab_out and seen_done:
                if not self.camping_wait_drop_zone:
                    break
                if seen_drop_sequence_error:
                    break
                if (
                    not self.run_charging_recall
                    and seen_reverse_parking_controller_parked
                    and (
                        not self.simulate_platform_status
                        or seen_service_charging
                    )
                ):
                    # HH_260721 - Wait for the externally visible CHARGING event before ending the run.
                    break
                if self.run_charging_recall and charging_recall_site_arrived:
                    break
        self.publish_engage(False)
        self.publish_mission_engage(False)
        self.cancel_all_actions()
        # HH_260720 - Always stop site/drop-zone/parking controllers after a
        # focused camping run, including timeout and failed-sequence cases.
        self.cancel_parking_maneuvers()
        cmd_max = self.max_abs_since.get("/control/cmd_vel", 0.0)
        global_new = self.global_path_count > base_global
        local_new = self.local_path_count > base_local
        route_reached = bool(reached_nav or seen_goal_reached_state)
        # HH_260721 - Roadside sites rotate on-lane after exit;
        # regular sites retain their in-site turn.
        # HH_260806 - Arrival-only roadside validation deliberately ends before
        # the unresolved lane-return rotation is requested.
        service_mode_ok = (
            not seen_rotate_180
            and not seen_align_retrace_yaw
            and (
                not seen_align_return_route_yaw
                if self.camping_stop_at_wait_return
                else seen_align_return_route_yaw
            )
            if camping_service_mode == "roadside_stop"
            else seen_rotate_180
            and seen_align_retrace_yaw
            and not seen_align_return_route_yaw
        )
        site_departure_ok = (
            seen_service_waiting_for_return_request
            if self.camping_stop_at_wait_return
            else seen_crab_out and seen_done
        )
        # HH_260806 - The arrival-only probe intentionally seeds the robot at
        # the snapped route endpoint. Nav2 may publish only its terminal local
        # path there, so require goal-reached plus either path surface.
        route_path_ok = global_new or (
            self.camping_stop_at_wait_return and local_new
        )
        site_ok = bool(
            route_path_ok
            and cmd_max > 0.03
            and route_reached
            and seen_site_phase
            and site_departure_ok
            and service_mode_ok
            and seen_service_waiting_for_return_request
        )
        drop_zone_ok = (
            not self.camping_wait_drop_zone
            or (
                seen_drop_zone_return
                and seen_drop_maneuver_alignment
                and seen_reverse_parking_controller_started
                and seen_reverse_parking_controller_parked
                and (
                    not self.simulate_platform_status
                    or (
                        seen_service_waiting_for_charging
                        and seen_service_charging
                    )
                )
                and not seen_drop_sequence_error
            )
        )
        # HH_260721 - Require the complete charging recall contract only when requested.
        charging_recall_ok = (
            not self.run_charging_recall
            or (
                self.simulate_platform_status
                and seen_reverse_wait_for_charging
                and seen_gate_charging_state
                and charging_recall_requested
                and charging_recall_key_seen
                and charging_recall_departure_state_seen
                and (
                    not self.charging_recall_via_ui
                    or (
                        charging_recall_service_departure_seen
                        and charging_recall_exit_straight_seen
                        and charging_recall_exit_alignment_seen
                    )
                )
                and charging_recall_cmd_released
                and charging_recall_disconnect_seen
                and self.global_path_count > charging_recall_global_path_base
                and self.local_path_count > charging_recall_local_path_base
                and charging_recall_site_arrived
            )
        )
        low_battery_finish_return_ok = (
            not low_battery_finish_return_enabled
            or (
                low_battery_finish_triggered
                and low_battery_finish_wait_seen
                and low_battery_finish_no_auto_return
                and low_battery_finish_manual_return_sent
                and low_battery_finish_return_started
                and low_battery_finish_recovered
            )
        )
        ok = bool(
            site_ok
            and drop_zone_ok
            and low_battery_finish_return_ok
            and charging_recall_low_battery_ok
            and charging_recall_ok
        )
        route_dist = (
            math.hypot(
                self.latest_pose.pose.position.x - self.latest_route_goal.pose.position.x,
                self.latest_pose.pose.position.y - self.latest_route_goal.pose.position.y,
            )
            if self.latest_pose is not None and self.latest_route_goal is not None
            else -1.0
        )
        site_route_dist = (
            math.hypot(
                self.latest_raw_goal.pose.position.x - self.latest_route_goal.pose.position.x,
                self.latest_raw_goal.pose.position.y - self.latest_route_goal.pose.position.y,
            )
            if self.latest_raw_goal is not None and self.latest_route_goal is not None
            else -1.0
        )
        site_dist = (
            math.hypot(
                self.latest_pose.pose.position.x - self.latest_raw_goal.pose.position.x,
                self.latest_pose.pose.position.y - self.latest_raw_goal.pose.position.y,
            )
            if self.latest_pose is not None and self.latest_raw_goal is not None
            else -1.0
        )
        latest_state_label = self.latest_state.label if self.latest_state else ""
        latest_state_key = self.latest_state.active_mission_key if self.latest_state else ""
        latest_scenario = self.latest_state.scenario_label if self.latest_state else ""
        latest_site_status = self.latest_site_status.message if self.latest_site_status else ""
        latest_drop_maneuver_status = (
            self.latest_drop_maneuver_status.message
            if self.latest_drop_maneuver_status
            else ""
        )
        latest_reverse_parking_controller_status = (
            self.latest_reverse_parking_controller_status.message
            if self.latest_reverse_parking_controller_status
            else ""
        )
        self.results.append(
            CheckResult(
                "camping_site_smoke",
                ok,
                "ok"
                if ok
                else (
                    f"global={global_new} local={local_new} cmd={cmd_max:.3f} "
                    f"nav_success={reached_nav} site_phase={seen_site_phase} "
                    f"service_mode={camping_service_mode} service_mode_ok={service_mode_ok} "
                    f"rotate_180={seen_rotate_180} "
                    f"align_retrace_yaw={seen_align_retrace_yaw} "
                    f"align_return_route_yaw={seen_align_return_route_yaw} "
                    f"crab_out={seen_crab_out} done={seen_done} "
                    f"drop_return={seen_drop_zone_return} "
                    f"drop_alignment={seen_drop_maneuver_alignment} "
                    f"reverse_started={seen_reverse_parking_controller_started} "
                    f"reverse_parked={seen_reverse_parking_controller_parked} "
                    f"service_wait_return={seen_service_waiting_for_return_request} "
                    f"service_wait_charging={seen_service_waiting_for_charging} "
                    f"service_charging={seen_service_charging} "
                    f"low_battery_finish_return={low_battery_finish_return_ok} "
                    f"charging_recall_battery_gate={charging_recall_low_battery_ok} "
                    f"charging_recall={charging_recall_ok} "
                    f"state={latest_state_label} "
                    f"key={latest_state_key} route_dist={route_dist:.2f}"
                ),
                {
                    "cmd_max": round(cmd_max, 3),
                    "nav_success": reached_nav,
                    "route_reached": route_reached,
                    "site_phase": seen_site_phase,
                    "service_mode": camping_service_mode,
                    "service_mode_ok": service_mode_ok,
                    "arrival_only": self.camping_stop_at_wait_return,
                    "rotate_180": seen_rotate_180,
                    "align_retrace_yaw": seen_align_retrace_yaw,
                    "align_return_route_yaw": seen_align_return_route_yaw,
                    "crab_out": seen_crab_out,
                    "done": seen_done,
                    "wait_drop_zone": self.camping_wait_drop_zone,
                    "drop_zone_return": seen_drop_zone_return,
                    "drop_zone_alignment": seen_drop_maneuver_alignment,
                    "reverse_parking_controller_started": seen_reverse_parking_controller_started,
                    "reverse_parking_controller_parked": seen_reverse_parking_controller_parked,
                    "drop_zone_sequence_error": seen_drop_sequence_error,
                    "reverse_wait_for_charging": seen_reverse_wait_for_charging,
                    "gate_charging_state": seen_gate_charging_state,
                    "service_waiting_for_return_request": (
                        seen_service_waiting_for_return_request
                    ),
                    "service_waiting_for_charging": seen_service_waiting_for_charging,
                    "service_charging": seen_service_charging,
                    "service_state_sequence": " -> ".join(self.service_state_names_seen),
                    "low_battery_finish_return_enabled": (
                        low_battery_finish_return_enabled
                    ),
                    "low_battery_mission_percentage": round(
                        self.low_battery_mission_percentage, 3
                    ),
                    "low_battery_finish_triggered": low_battery_finish_triggered,
                    "low_battery_finish_wait_seen": low_battery_finish_wait_seen,
                    "low_battery_return_manual_observe_s": round(
                        max(0.0, self.low_battery_return_manual_observe_s), 3
                    ),
                    "low_battery_finish_no_auto_return": (
                        low_battery_finish_no_auto_return
                    ),
                    "low_battery_finish_manual_return_sent": (
                        low_battery_finish_manual_return_sent
                    ),
                    "low_battery_finish_return_started": (
                        low_battery_finish_return_started
                    ),
                    "low_battery_finish_recovered": low_battery_finish_recovered,
                    "low_battery_finish_return_ok": low_battery_finish_return_ok,
                    "charging_recall_requested": charging_recall_requested,
                    "charging_recall_via_ui": self.charging_recall_via_ui,
                    "charging_recall_battery_gate_enabled": (
                        charging_recall_battery_gate_enabled
                    ),
                    "charging_recall_low_battery_percentage": round(
                        self.charging_recall_low_battery_percentage, 3
                    ),
                    "charging_recall_low_battery_requested": (
                        charging_recall_low_battery_requested
                    ),
                    "charging_recall_low_battery_block": (
                        charging_recall_low_battery_ok
                    ),
                    "charging_recall_low_battery_departure_seen": (
                        charging_recall_low_battery_departure_seen
                    ),
                    "charging_recall_low_battery_cmd_released": (
                        charging_recall_low_battery_cmd_released
                    ),
                    "charging_recall_low_battery_global_paths": (
                        charging_recall_low_battery_global_delta
                    ),
                    "charging_recall_low_battery_local_paths": (
                        charging_recall_low_battery_local_delta
                    ),
                    "charging_recall_key_seen": charging_recall_key_seen,
                    "charging_recall_departure_state": charging_recall_departure_state_seen,
                    "charging_recall_service_departure": (
                        charging_recall_service_departure_seen
                    ),
                    "charging_recall_exit_straight": charging_recall_exit_straight_seen,
                    "charging_recall_exit_alignment": charging_recall_exit_alignment_seen,
                    "charging_recall_cmd_released": charging_recall_cmd_released,
                    "charging_recall_disconnect": charging_recall_disconnect_seen,
                    "charging_recall_site_arrived": charging_recall_site_arrived,
                    "charging_recall_ok": charging_recall_ok,
                    "goal_reached_state_seen": seen_goal_reached_state,
                    "site_key_seen": seen_site_key,
                    "latest_state": latest_state_label,
                    "latest_active_mission_key": latest_state_key,
                    "latest_scenario": latest_scenario,
                    "latest_site_status": latest_site_status,
                    "latest_drop_maneuver_status": latest_drop_maneuver_status,
                    "latest_reverse_parking_controller_status": (
                        latest_reverse_parking_controller_status
                    ),
                    "state_labels_seen": ",".join(sorted(state_labels)),
                    "scenario_labels_seen": ",".join(sorted(scenario_labels)),
                    "site_statuses_seen": " | ".join(sorted(site_status_messages)),
                    "site_phase_sequence": " -> ".join(site_phase_sequence),
                    "drop_maneuver_statuses_seen": " | ".join(
                        sorted(drop_maneuver_status_messages)
                    ),
                    "reverse_parking_controller_statuses_seen": " | ".join(
                        sorted(reverse_parking_controller_status_messages)
                    ),
                    "route_distance_m": round(route_dist, 2),
                    "site_route_distance_m": round(site_route_dist, 2),
                    "site_distance_m": round(site_dist, 2),
                },
            )
        )

    @staticmethod
    def _event_contains(events: list[str], start_index: int, token: str) -> bool:
        return any(token in event for event in events[start_index:])

    @staticmethod
    def _route_recovery_progress(events: list[str]) -> tuple[bool, bool]:
        motion_prefixes = (
            "CRAB_LEFT|",
            "CRAB_RIGHT|",
            "REVERSE|",
            "REVERSE_YAW_LEFT|",
            "REVERSE_YAW_RIGHT|",
        )
        motion_index = next(
            (index for index, event in enumerate(events) if event.startswith(motion_prefixes)),
            None,
        )
        if motion_index is None:
            return False, False
        # HH_260807 - The owner publishes IDLE only after the gate removes its
        # route hold. Requiring IDLE after the motion event avoids counting the
        # initial pre-contact IDLE sample as a completed recovery.
        released = any(event.startswith("IDLE|") for event in events[motion_index + 1:])
        return True, released

    def check_repeated_service_soak(self) -> None:
        """Run multiple site-return-charge cycles without restarting bringup."""
        self.cancel_all_actions()
        self.cancel_parking_maneuvers()
        self.clear_obstacle()
        mission_keys = self.service_soak_mission_keys
        if len(mission_keys) < 2:
            self.results.append(
                CheckResult(
                    "repeated_service_soak",
                    False,
                    "service_soak_mission_keys requires at least two sites",
                )
            )
            return

        site_configs = [self.load_camping_site_config(key) for key in mission_keys]
        invalid = [key for key, config in zip(mission_keys, site_configs) if config is None]
        roadside = [
            key
            for key, config in zip(mission_keys, site_configs)
            if config is not None
            and str(config.get("service_mode", "turnaround")).strip().lower()
            != "turnaround"
        ]
        if invalid or roadside:
            self.results.append(
                CheckResult(
                    "repeated_service_soak",
                    False,
                    f"invalid_sites={invalid} roadside_return_not_approved={roadside}",
                )
            )
            return

        first_goal = self.load_camping_goal(mission_keys[0])
        if first_goal is None or not self.prepare_camping_start_pose(first_goal):
            self.results.append(
                CheckResult(
                    "repeated_service_soak",
                    False,
                    "failed to prepare first service route",
                )
            )
            return

        # HH_260807 - Only the first cycle is seeded near the route endpoint.
        # Every later cycle departs naturally from the simulated charger and
        # must return, park, charge, and accept the next destination in one
        # uninterrupted bringup process.
        self.cancel_parking_maneuvers()
        self.publish_engage(True)
        self.publish_mission_engage(True)
        self.spin_for(0.3)

        cycle_reports: list[dict[str, object]] = []
        cycles_completed = 0
        overall_ok = True
        soak_start = time.monotonic()

        for cycle_number, mission_key in enumerate(mission_keys, start=1):
            goal_pose = self.load_camping_goal(mission_key)
            if goal_pose is None:
                overall_ok = False
                cycle_reports.append(
                    {"cycle": cycle_number, "mission_key": mission_key, "error": "missing goal"}
                )
                break

            service_base = len(self.service_state_names_seen)
            site_base = len(self.site_status_events)
            drop_base = len(self.drop_status_events)
            parking_base = len(self.parking_status_events)
            gate_base = len(self.gate_status_events)
            recovery_base = len(self.route_recovery_status_events)
            global_base = self.global_path_count
            local_base = self.local_path_count
            self.latest_replan_status = ""
            self.replan_statuses_seen.clear()
            self.reset_cmd_metrics()

            starts_charging = cycle_number > 1
            if starts_charging:
                # HH_260807 - The preceding PARKED cycle must leave the raw BMS
                # simulator charging; do not synthesize a second status source.
                self.spin_for(0.25)
            self.publish_charging_recall_request(
                mission_key,
                goal_pose,
                f"sim_service_soak_cycle_{cycle_number}",
            )

            cycle_start = time.monotonic()
            return_sent = False
            departure_seen = not starts_charging
            charger_disconnected = not starts_charging
            route_motion_seen = False
            obstacle_injected = False
            obstacle_stop_seen = False
            obstacle_cleared = False
            obstacle_resume_seen = False
            obstacle_started_at = 0.0
            obstacle_world_x = 0.0
            obstacle_world_y = 0.0
            cycle_complete = False
            boundary_hold_seen = False
            boundary_recovery_motion_seen = False
            boundary_recovery_released = False
            boundary_retry_latched = False

            while (
                rclpy.ok()
                and time.monotonic() - cycle_start < self.service_soak_cycle_timeout_s
            ):
                rclpy.spin_once(self, timeout_sec=0.05)
                service_events = self.service_state_names_seen[service_base:]
                site_events = self.site_status_events[site_base:]
                parking_events = self.parking_status_events[parking_base:]
                gate_events = self.gate_status_events[gate_base:]
                recovery_events = self.route_recovery_status_events[recovery_base:]

                boundary_hold_seen = boundary_hold_seen or any(
                    "ROUTE_SAFETY_HOLD" in event or "route_safety_hold=" in event
                    for event in gate_events
                )
                boundary_retry_latched = boundary_retry_latched or any(
                    "route_safety_retry_latched=" in event
                    or "rapid_recontact_latched" in event
                    for event in gate_events
                )
                recovery_motion, recovery_released = self._route_recovery_progress(
                    recovery_events
                )
                boundary_recovery_motion_seen = (
                    boundary_recovery_motion_seen or recovery_motion
                )
                boundary_recovery_released = (
                    boundary_recovery_released or recovery_released
                )

                route_motion_seen = route_motion_seen or self.latest_control_cmd_abs > 0.03
                departure_seen = departure_seen or any(
                    event == "DEPARTING_CHARGER" for event in service_events
                ) or any("DEPARTING_CHARGER" in event for event in gate_events)
                if (
                    starts_charging
                    and departure_seen
                    and not self.fake_platform_charging
                ):
                    charger_disconnected = True

                should_inject_obstacle = (
                    cycle_number == self.service_soak_obstacle_cycle
                    and not obstacle_injected
                    and route_motion_seen
                    and self.navigation_active()
                    and self.global_path_count > global_base
                    and self.latest_pose is not None
                    and time.monotonic() - cycle_start > 1.0
                )
                if should_inject_obstacle:
                    yaw = yaw_from_quat(self.latest_pose.pose.orientation)
                    obstacle_world_x = (
                        self.latest_pose.pose.position.x
                        + self.service_soak_obstacle_offset_m * math.cos(yaw)
                    )
                    obstacle_world_y = (
                        self.latest_pose.pose.position.y
                        + self.service_soak_obstacle_offset_m * math.sin(yaw)
                    )
                    self.set_fake_params(
                        obstacle_reference_frame="map",
                        obstacle_world_x=obstacle_world_x,
                        obstacle_world_y=obstacle_world_y,
                        obstacle_direction="front",
                        obstacle_offset=self.service_soak_obstacle_offset_m,
                        obstacle_lateral_offset=0.0,
                        fake_obstacle_cluster_radius_m=0.35,
                        publish_fake_lidar_obstacle_cloud=True,
                        publish_fake_radar_ranges=True,
                    )
                    obstacle_injected = True
                    obstacle_started_at = time.monotonic()

                if obstacle_injected and not obstacle_cleared:
                    obstacle_stop_seen = obstacle_stop_seen or (
                        self.latest_control_cmd_abs <= 0.03
                        and (
                            self.latest_replan_status.startswith("BLOCKED")
                            or any(
                                token in event.lower()
                                for event in gate_events
                                for token in ("obstacle", "radar", "lidar", "cost")
                            )
                        )
                    )
                    if (
                        obstacle_stop_seen
                        and time.monotonic() - obstacle_started_at
                        >= max(0.5, self.service_soak_obstacle_hold_s)
                    ):
                        self.clear_obstacle()
                        obstacle_cleared = True

                if obstacle_cleared:
                    obstacle_resume_seen = (
                        obstacle_resume_seen or self.latest_control_cmd_abs > 0.03
                    )

                waiting_for_return = (
                    any("WAIT_RETURN" in event for event in site_events)
                    and "WAITING_FOR_RETURN_REQUEST" in service_events
                )
                obstacle_cycle_ready = (
                    cycle_number != self.service_soak_obstacle_cycle
                    or (obstacle_stop_seen and obstacle_cleared and obstacle_resume_seen)
                )
                if waiting_for_return and obstacle_cycle_ready and not return_sent:
                    # HH_260807 - Mirror a human RETURN only after the public
                    # unload-wait state; never auto-return on first arrival.
                    self.publish_operation(
                        self.pub_site_operation,
                        MotionOperation.RETURN,
                        repeats=1,
                    )
                    return_sent = True

                cycle_complete = bool(
                    return_sent
                    and any("CRAB_IN" in event for event in site_events)
                    and any("ROTATE_180" in event for event in site_events)
                    and any("CRAB_OUT" in event for event in site_events)
                    and any("DONE" in event for event in site_events)
                    and any("ALIGN_PARKING_YAW" in event for event in self.drop_status_events[drop_base:])
                    and any("PARKED" in event for event in parking_events)
                    and "WAITING_FOR_CHARGING" in service_events
                    and "CHARGING" in service_events
                )
                if cycle_complete:
                    break

            if obstacle_injected and not obstacle_cleared:
                self.clear_obstacle()

            service_events = self.service_state_names_seen[service_base:]
            site_events = self.site_status_events[site_base:]
            drop_events = self.drop_status_events[drop_base:]
            parking_events = self.parking_status_events[parking_base:]
            gate_events = self.gate_status_events[gate_base:]
            recovery_events = self.route_recovery_status_events[recovery_base:]
            obstacle_required = cycle_number == self.service_soak_obstacle_cycle
            boundary_recovery_ok = bool(
                not boundary_retry_latched
                and (
                    not boundary_hold_seen
                    or (boundary_recovery_motion_seen and boundary_recovery_released)
                )
            )
            cycle_ok = bool(
                cycle_complete
                and route_motion_seen
                and departure_seen
                and charger_disconnected
                and (self.global_path_count > global_base or self.local_path_count > local_base)
                and (
                    not obstacle_required
                    or (obstacle_stop_seen and obstacle_cleared and obstacle_resume_seen)
                )
                and boundary_recovery_ok
            )
            cycle_reports.append(
                {
                    "cycle": cycle_number,
                    "mission_key": mission_key,
                    "success": cycle_ok,
                    "elapsed_s": round(time.monotonic() - cycle_start, 3),
                    "service_states": service_events,
                    "site_phases": site_events,
                    "drop_phases": drop_events,
                    "parking_phases": parking_events,
                    "gate_phases": gate_events,
                    "route_recovery_phases": recovery_events,
                    "global_path_updates": self.global_path_count - global_base,
                    "local_path_updates": self.local_path_count - local_base,
                    "route_motion_seen": route_motion_seen,
                    "return_sent_after_wait": return_sent,
                    "departure_seen": departure_seen,
                    "charger_disconnected": charger_disconnected,
                    "obstacle_required": obstacle_required,
                    "obstacle_world_x": round(obstacle_world_x, 3),
                    "obstacle_world_y": round(obstacle_world_y, 3),
                    "obstacle_stop_seen": obstacle_stop_seen,
                    "obstacle_cleared": obstacle_cleared,
                    "obstacle_resume_seen": obstacle_resume_seen,
                    "boundary_hold_seen": boundary_hold_seen,
                    "boundary_recovery_motion_seen": boundary_recovery_motion_seen,
                    "boundary_recovery_released": boundary_recovery_released,
                    "boundary_retry_latched": boundary_retry_latched,
                    "boundary_recovery_ok": boundary_recovery_ok,
                    "replan_statuses": sorted(self.replan_statuses_seen),
                }
            )
            if not cycle_ok:
                overall_ok = False
                break
            cycles_completed += 1

        self.clear_obstacle()
        self.publish_engage(False)
        self.publish_mission_engage(False)
        self.cancel_all_actions()
        self.cancel_parking_maneuvers()
        overall_ok = overall_ok and cycles_completed == len(mission_keys)
        self.results.append(
            CheckResult(
                "repeated_service_soak",
                overall_ok,
                "ok"
                if overall_ok
                else f"completed={cycles_completed}/{len(mission_keys)}",
                {
                    "cycles_requested": len(mission_keys),
                    "cycles_completed": cycles_completed,
                    "mission_keys": mission_keys,
                    # HH_260807 - Keep the endurance claim explicit: cycle 1
                    # isolates the site handoff when preparation is enabled;
                    # later cycles depart from the charger and run full routes.
                    "first_cycle_seeded_near_route": self.camping_prepare_near_route,
                    "full_outbound_cycles": max(
                        0,
                        cycles_completed - (1 if self.camping_prepare_near_route else 0),
                    ),
                    "elapsed_s": round(time.monotonic() - soak_start, 3),
                    "bringup_restart_count": 0,
                    "cycles": cycle_reports,
                },
            )
        )

    def run(self) -> int:
        self.spin_for(1.0)
        self.publish_mission_engage(False)
        self.publish_engage(False)
        self.cancel_all_actions()
        self.check_baseline_rates()
        self.check_radar_ranges()
        # HH_260720 - Keep the default full suite while supporting focused long-duration runs.
        if self.run_gate_matrix:
            self.check_gate_stop_matrix()
        if not self.skip_manual_goal:
            self.check_manual_goal()
        if self.run_obstacle_replan:
            self.check_obstacle_replan()
        if self.run_camping:
            self.check_camping_site_smoke()
        if self.run_service_soak:
            self.check_repeated_service_soak()
        self.clear_obstacle()
        self.publish_engage(False)
        self.publish_mission_engage(False)
        ok = all(result.ok for result in self.results)
        self.print_report(ok)
        self.write_report(ok)
        return 0 if ok else 2

    def print_report(self, overall_ok: bool) -> None:
        print("\n=== CAMROD_SIM_VALIDATION_REPORT ===")
        print(f"OVERALL={'PASS' if overall_ok else 'FAIL'}")
        for idx, result in enumerate(self.results, 1):
            print(f"CHECK_{idx:02d}_NAME={result.name}")
            print(f"CHECK_{idx:02d}_RESULT={'PASS' if result.ok else 'FAIL'}")
            print(f"CHECK_{idx:02d}_DETAIL={result.detail}")
            if result.metrics:
                print(
                    f"CHECK_{idx:02d}_METRICS="
                    + json.dumps(result.metrics, sort_keys=True, ensure_ascii=False)
                )
        print("=== CAMROD_SIM_VALIDATION_REPORT_END ===")

    def write_report(self, overall_ok: bool) -> None:
        if not self.report_file.strip():
            return
        path = os.path.abspath(self.report_file.strip())
        report_dir = os.path.dirname(path)
        if report_dir:
            os.makedirs(report_dir, exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            json.dump(
                {
                    "overall_pass": overall_ok,
                    "checks": [
                        {
                            "name": r.name,
                            "success": r.ok,
                            "detail": r.detail,
                            "metrics": r.metrics,
                        }
                        for r in self.results
                    ],
                },
                stream,
                indent=2,
                ensure_ascii=False,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimValidationRunner()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
