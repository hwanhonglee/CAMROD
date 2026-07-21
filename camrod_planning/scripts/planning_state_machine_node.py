#!/usr/bin/env python3
# HH_260312 Planning state machine with keypoint mapping based on /status.

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Dict, Optional, Set

import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from avg_msgs.msg import (
    AvgAmrServiceState,
    AvgBool,
    AvgPoseStamped,
    PlanningMissionKey,
    PlanningRecallRequest,
    PlanningScenario,
    PlanningState,
)
from avg_msgs.srv import RequestGoalByKey
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from rclpy.node import Node

# HH_260528 Prefer status_msgs, fallback to diagnostic_msgs for robustness.
try:
    from status_msgs.msg import KeyValue, StatusArray, StatusStatus
except Exception:  # noqa: BLE001
    from diagnostic_msgs.msg import (
        DiagnosticArray as StatusArray,
        DiagnosticStatus as StatusStatus,
        KeyValue,
    )


# Implements `_diag_level` behavior.
def _diag_level(value: object) -> bytes:
    # HH_260311 Humble uint8 constants may be exposed as bytes.
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])


@dataclass
class Keypoint:
    name: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw_deg: float = 0.0
    source_id: str = ""


class PlanningStateMachineNode(Node):
    # HH_260528 Scenario IDs for UI/state contracts.
    SCENARIO_WAIT_DROP_ZONE = 0
    SCENARIO_DELIVERY_TO_SITE = 1
    SCENARIO_RETURN_TO_DROP_ZONE = 2
    SCENARIO_RECALL_TO_SITE = 3
    # HH_260622 - Mirror non-Nav2 parking/site phases into the central planning state output.
    SCENARIO_SITE_ENTRY = int(PlanningScenario.SITE_ENTRY)
    SCENARIO_UNLOAD_WAIT = int(PlanningScenario.UNLOAD_WAIT)
    SCENARIO_RECALL_TO_SITE_ROAD = int(PlanningScenario.RECALL_TO_SITE_ROAD)
    SCENARIO_GUEST_LOADING_WAIT = int(PlanningScenario.GUEST_LOADING_WAIT)
    SCENARIO_RETURN_WITH_CARGO = int(PlanningScenario.RETURN_WITH_CARGO)
    SCENARIO_DROP_ZONE_PARKING = int(PlanningScenario.DROP_ZONE_PARKING)

    _SCENARIO_LABELS = {
        SCENARIO_WAIT_DROP_ZONE: "WAIT_DROP_ZONE",
        SCENARIO_DELIVERY_TO_SITE: "DELIVERY_TO_SITE",
        SCENARIO_RETURN_TO_DROP_ZONE: "RETURN_TO_DROP_ZONE",
        SCENARIO_RECALL_TO_SITE: "RECALL_TO_SITE",
        SCENARIO_SITE_ENTRY: "SITE_ENTRY",
        SCENARIO_UNLOAD_WAIT: "UNLOAD_WAIT",
        SCENARIO_RECALL_TO_SITE_ROAD: "RECALL_TO_SITE_ROAD",
        SCENARIO_GUEST_LOADING_WAIT: "GUEST_LOADING_WAIT",
        SCENARIO_RETURN_WITH_CARGO: "RETURN_WITH_CARGO",
        SCENARIO_DROP_ZONE_PARKING: "DROP_ZONE_PARKING",
    }

    _STATE_IDS = {
        "INIT": PlanningState.INIT,
        "READY": PlanningState.READY,
        "WAIT_DZ": PlanningState.WAIT_DZ,
        "RUNNING": PlanningState.RUNNING,
        "GOAL_REACHED": PlanningState.GOAL_REACHED,
        "RECALLED": PlanningState.RECALLED,
        "RETURNING": PlanningState.RETURNING,
        "WARN_RECOVERY": PlanningState.WARN_RECOVERY,
        "ERROR_STOP": PlanningState.ERROR_STOP,
    }

    # Implements `__init__` behavior.
    def __init__(self) -> None:
        super().__init__("planning_state_machine")

        self.enabled = bool(self.declare_parameter("enabled", True).value)
        self.state_status_topic = str(
            self.declare_parameter("state_status_topic", "/system/diagnostics_agg").value
        )
        self.state_stale_timeout_s = float(
            self.declare_parameter("state_stale_timeout_s", 3.0).value
        )
        ignored_state_names = self.declare_parameter(
            "state_status_ignored_names", [""]
        ).value
        ignored_state_prefixes = self.declare_parameter(
            "state_status_ignored_prefixes", [""]
        ).value
        self.state_status_ignored_names = self._param_string_set(ignored_state_names)
        self.state_status_ignored_prefixes = self._param_string_set(ignored_state_prefixes)
        # HH_260720 - Consume canonical generated CAMROD poses for internal state decisions.
        self.pose_topic = str(self.declare_parameter("pose_topic", "/planning/lanelet_pose").value)
        # HH_260721 - Delay a return route until the lanelet snap has caught up
        # with the vehicle after a campsite maneuver. This prevents Nav2 from
        # planning from a stale pre-maneuver lanelet pose.
        self.return_start_vehicle_pose_topic = str(
            self.declare_parameter(
                "return_start_vehicle_pose_topic", "/localization/pose"
            ).value
        )
        self.return_start_pose_max_distance_m = float(
            self.declare_parameter("return_start_pose_max_distance_m", 1.5).value
        )
        self.return_start_pose_stale_timeout_s = float(
            self.declare_parameter("return_start_pose_stale_timeout_s", 1.0).value
        )
        self.goal_topic = str(
            self.declare_parameter("goal_topic", "/planning/goal_pose_snapped").value
        )
        # HH_260319 Mirror auto-goals to Nav2 ROS-facing goal topic if configured.
        self.goal_topic_ros = str(
            self.declare_parameter("goal_topic_ros", "/planning/goal_pose_snapped_ros").value
        )
        # HH_260623 - Auto return/drop-zone goals are station-center poses, not
        # lanelet route poses. Publish those raw poses through a private goal_snapper
        # input so Nav2 receives a valid lanelet-snapped route goal while parking
        # keeps the original station pose for reverse parking.
        self.auto_goal_snapper_input_topic = str(
            self.declare_parameter("auto_goal_snapper_input_topic", "").value
        )
        self.state_topic = str(
            self.declare_parameter("state_topic", "/planning/state_machine/state").value
        )
        self.estop_topic = str(
            self.declare_parameter("estop_topic", "/planning/state_machine/estop").value
        )
        self.return_topic = str(
            self.declare_parameter(
                "return_to_drop_zone_topic", "/planning/state_machine/return_to_drop_zone"
            ).value
        )
        self.recall_topic = str(
            self.declare_parameter(
                "camping_site_recall_topic", "/planning/state_machine/camping_site_recall"
            ).value
        )
        self.mission_key_topic = str(
            self.declare_parameter("mission_key_topic", "/planning/mission_key").value
        )
        # HH_260616: UI destination requests publish both mission_key and site_goal.
        # This switch controls whether a mission_key publishes a route_goal directly, while the
        # actual site-center site_goal still passes through goal_snapper before Nav2.
        self.mission_key_publish_route_goal = bool(
            self.declare_parameter("mission_key_publish_route_goal", True).value
        )
        self.request_mission_service = str(
            self.declare_parameter("request_mission_service", "/planning/request_mission").value
        )

        # HH_260528 Scenario contract topics for UI.
        self.scenario_id_topic = str(
            self.declare_parameter("scenario_id_topic", "/planning/state_machine/scenario_id").value
        )
        self.scenario_command_topic = str(
            self.declare_parameter("scenario_command_topic", "/planning/state_machine/scenario_command").value
        )
        # HH_260720 - Control maneuvers and parking publish phase telemetry on
        # /AMR_service_state; planning mirrors only the explicit phase owners.
        self.enable_maneuver_phase_state_override = bool(
            self.declare_parameter("enable_maneuver_phase_state_override", True).value
        )
        self.maneuver_phase_state_topic = str(
            self.declare_parameter("maneuver_phase_state_topic", "/AMR_service_state").value
        )
        self.maneuver_phase_override_timeout_s = float(
            self.declare_parameter("maneuver_phase_override_timeout_s", 0.0).value
        )

        self.mission_source_topic = str(
            self.declare_parameter("mission_source_topic", "/planning/state_machine/mission_source").value
        )

        self.diag_topic = str(
            self.declare_parameter("status_stream_topic", "/status_stream").value
        )
        self.keypoints_yaml = str(self.declare_parameter("keypoints_yaml", "").value)
        self.camping_sites_yaml = str(self.declare_parameter("camping_sites_yaml", "").value)
        self.startup_mission_key = str(self.declare_parameter("startup_mission_key", "drop_zone").value)
        self.warn_mission_key = str(self.declare_parameter("warn_mission_key", "garage").value)
        self.return_mission_key = str(self.declare_parameter("return_mission_key", "drop_zone").value)
        # HH_260624 - Keep planning and parking on the same semantic drop-zone selector.
        self.drop_zone_id = str(self.declare_parameter("drop_zone_id", "drop_zone").value)
        # HH_260624 - Publish the raw station-center return target for RViz/debug only.
        self.drop_zone_goal_raw_topic = str(
            self.declare_parameter("drop_zone_goal_raw_topic", "/planning/drop_zone_goal_raw").value
        )
        # HH_260720 - Expose the semantic drop-zone target to RViz on a named ROS boundary.
        self.drop_zone_goal_raw_ros_topic = str(
            self.declare_parameter(
                "drop_zone_goal_raw_ros_topic", "/planning/drop_zone_goal_raw_ros"
            ).value
        )
        # HH_260624 - RViz/UI manual /goal_pose clicks lose semantic meaning after
        # goal_snapper moves them to a lanelet. Watch the raw goal too so a manual
        # drop-zone click still starts RETURN_TO_DROP_ZONE and reverse parking.
        self.raw_goal_topic = str(self.declare_parameter("raw_goal_topic", "/goal_pose").value)
        self.enable_raw_drop_zone_goal_match = bool(
            self.declare_parameter("enable_raw_drop_zone_goal_match", True).value
        )
        self.drop_zone_raw_goal_match_distance_m = float(
            self.declare_parameter("drop_zone_raw_goal_match_distance_m", 3.0).value
        )
        raw_auto_snap_keys = self.declare_parameter(
            "auto_goal_snapper_keys", [self.return_mission_key]
        ).value
        if isinstance(raw_auto_snap_keys, str):
            self.auto_goal_snapper_keys = {
                item.strip() for item in raw_auto_snap_keys.split(",") if item.strip()
            }
        else:
            self.auto_goal_snapper_keys = {
                str(item).strip() for item in raw_auto_snap_keys if str(item).strip()
            }
        self.default_recall_mission_key = str(
            self.declare_parameter("default_recall_mission_key", "camping_site_1").value
        )

        self.auto_startup_goal = bool(self.declare_parameter("auto_startup_goal", True).value)
        self.auto_warn_recovery_goal = bool(
            self.declare_parameter("auto_warn_recovery_goal", True).value
        )
        self.auto_estop_on_error = bool(self.declare_parameter("auto_estop_on_error", True).value)
        # HH_260703 - Diagnostics WARN must stay visible, but it should not
        # mask the GOAL_REACHED handoff used by campsite/drop-zone parking.
        self.allow_goal_reached_handoff_in_warn = bool(
            self.declare_parameter("allow_goal_reached_handoff_in_warn", True).value
        )

        # HH_260528 Optional auto-return behavior for scenario 1/3 when site reached.
        self.enable_auto_return_on_site_goal = bool(
            self.declare_parameter("enable_auto_return_on_site_goal", False).value
        )
        self.site_mission_key_prefix = str(
            self.declare_parameter("site_mission_key_prefix", "camping_site_").value
        )
        self.goal_reached_dwell_s = float(
            self.declare_parameter("goal_reached_dwell_s", 10.0).value
        )
        self.mission_key_match_distance_m = float(
            self.declare_parameter("mission_key_match_distance_m", 1.5).value
        )
        # HH_260618: Return-to-drop-zone goals can be lanelet-snap poses several
        # meters away from the station keypoint. Preserve the semantic drop_zone
        # key when an echoed/snap goal is still near the active return goal.
        self.return_goal_key_preserve_distance_m = float(
            self.declare_parameter("return_goal_key_preserve_distance_m", 5.0).value
        )
        # HH_260618: UI sends a semantic mission_key and a raw campsite center,
        # then goal_snapper publishes the lanelet-snapped route_goal. The route
        # goal is several meters from the campsite keypoint, so repeated/smoothed
        # route_goal echoes must not clear active_mission_key before parking
        # starts.
        self.site_goal_key_preserve_distance_m = float(
            self.declare_parameter("site_goal_key_preserve_distance_m", 2.0).value
        )
        # HH_260619 - UI destination dispatch publishes mission_key first and
        # raw campsite center immediately after. The snapped route goal may be
        # closer to another semantic key during stale component transitions, so
        # the recent mission_key must remain authoritative for one route update.
        self.pending_mission_key_preserve_s = float(
            self.declare_parameter("pending_mission_key_preserve_s", 5.0).value
        )
        self.pending_mission_key_overrides_goal_match = bool(
            self.declare_parameter("pending_mission_key_overrides_goal_match", True).value
        )

        self.min_goal_publish_interval_s = float(
            self.declare_parameter("min_goal_publish_interval_s", 1.0).value
        )
        self.goal_reached_distance_m = float(
            self.declare_parameter("goal_reached_distance_m", 0.2).value
        )
        # HH_260622 - Parking handoff must wait for Nav2 terminal success, not
        # only geometric distance, otherwise camping_site_maneuver_controller can race bt_navigator.
        self.require_nav2_success_for_goal_reached = bool(
            self.declare_parameter("require_nav2_success_for_goal_reached", True).value
        )
        self.nav_status_topic = str(
            self.declare_parameter(
                "nav_status_topic", "/planning/navigate_to_pose/_action/status"
            ).value
        )
        self.nav_status_goal_match_tolerance_s = float(
            self.declare_parameter("nav_status_goal_match_tolerance_s", 0.5).value
        )
        self.nav_success_latch_s = float(
            self.declare_parameter("nav_success_latch_s", 3.0).value
        )
        # HH_260618 - Return-to-drop-zone Nav2 success can stop slightly outside
        # the generic route tolerance because centerline/lanelet pose and Nav2
        # goal-checker pose are not identical. Keep campsite arrival tight while
        # allowing the parking handoff to start after the return route succeeds.
        self.return_goal_reached_distance_m = float(
            self.declare_parameter("return_goal_reached_distance_m", 0.3).value
        )
        # HH_260720 - Keep GOAL_REACHED visible long enough for drop-zone
        # alignment to receive its handoff before reverse parking starts.
        self.reverse_parking_controller_handoff_hold_s = float(
            self.declare_parameter("reverse_parking_controller_handoff_hold_s", 1.0).value
        )
        self.loop_rate_hz = float(self.declare_parameter("loop_rate_hz", 5.0).value)

        self.keypoints: Dict[str, Keypoint] = {}
        self.drop_zone_keypoints: list[Keypoint] = []
        self.drop_zone_polygons: Dict[str, list[tuple[float, float]]] = {}
        self.selected_return_keypoint: Optional[Keypoint] = None
        self._load_keypoints()

        self.last_state_stamp: Optional[rclpy.time.Time] = None
        self.module_levels: Dict[str, int] = {}
        self.last_pose: Optional[AvgPoseStamped] = None
        # HH_260721 - Keep independent receipt times because message stamps can
        # use hardware or simulation clocks that differ during startup.
        self.last_pose_received_time: Optional[rclpy.time.Time] = None
        self.last_return_start_vehicle_pose: Optional[AvgPoseStamped] = None
        self.last_return_start_vehicle_pose_received_time: Optional[rclpy.time.Time] = None
        self._last_return_start_wait_log_time: Optional[rclpy.time.Time] = None
        self.last_manual_goal: Optional[AvgPoseStamped] = None
        self.active_goal: Optional[AvgPoseStamped] = None
        self.active_goal_source: str = "none"
        self.active_mission_key: str = ""

        self.state: str = "INIT"
        self.scenario_id: int = self.SCENARIO_WAIT_DROP_ZONE
        self.startup_goal_sent = False
        self.warn_goal_sent = False
        self.return_requested = False
        self.recall_requested = False
        self.recall_target_key: str = ""
        self.recall_site_name: str = ""
        self.last_recalled_mission_key: str = ""

        self.prev_state_level: Optional[int] = None
        self._ok_level = self._status_level_int(StatusStatus.OK)
        self._warn_level = self._status_level_int(StatusStatus.WARN)
        self._error_level = self._status_level_int(StatusStatus.ERROR)

        self._last_goal_publish_time = self.get_clock().now()
        self._last_self_goal: Optional[AvgPoseStamped] = None
        self._pending_mission_key_request: str = ""
        self._pending_mission_key_time = self.get_clock().now()
        self._active_goal_time = self.get_clock().now()
        self._nav2_goal_succeeded = False
        self._nav2_terminal_status = 0
        self._nav2_terminal_time: Optional[rclpy.time.Time] = None
        self._maneuver_phase_override_state: str = ""
        self._maneuver_phase_override_scenario_id: Optional[int] = None
        self._maneuver_phase_override_source: str = ""
        self._maneuver_phase_override_time: Optional[rclpy.time.Time] = None

        self._goal_reached_since: Optional[rclpy.time.Time] = None
        self._goal_reached_latched = False
        # HH_260720 - Publish drop-zone route completion once before WAIT_DROP_ZONE
        # so control alignment and reverse parking can take ownership.
        self._drop_zone_arrival_notified = False
        self._drop_zone_arrival_notified_time: Optional[rclpy.time.Time] = None

        self.pub_goal = self.create_publisher(AvgPoseStamped, self.goal_topic, 10)
        self.pub_goal_ros = None
        if self.goal_topic_ros and self.goal_topic_ros != self.goal_topic:
            # HH_260720 - Keep only the Nav2-facing goal output on geometry_msgs.
            self.pub_goal_ros = self.create_publisher(RosPoseStamped, self.goal_topic_ros, 10)
        self.pub_auto_goal_snapper = None
        if self.auto_goal_snapper_input_topic:
            self.pub_auto_goal_snapper = self.create_publisher(
                AvgPoseStamped, self.auto_goal_snapper_input_topic, 10
            )
        self.pub_drop_zone_goal_raw = None
        if self.drop_zone_goal_raw_topic:
            # HH_260720 - Publish the exact station pose as a generated internal contract.
            self.pub_drop_zone_goal_raw = self.create_publisher(
                AvgPoseStamped, self.drop_zone_goal_raw_topic, 10
            )
        self.pub_drop_zone_goal_raw_ros = None
        if self.drop_zone_goal_raw_ros_topic:
            # HH_260720 - Keep the standard drop-zone pose isolated to visualization consumers.
            self.pub_drop_zone_goal_raw_ros = self.create_publisher(
                RosPoseStamped, self.drop_zone_goal_raw_ros_topic, 10
            )

        # HH_260617: Publish CAMROD semantic state/status messages instead of
        # std_msgs/String/Int32 wrappers.
        self.pub_state = self.create_publisher(PlanningState, self.state_topic, 10)
        self.pub_estop = self.create_publisher(AvgBool, self.estop_topic, 10)
        self.pub_diag = self.create_publisher(StatusArray, self.diag_topic, 10)
        self.pub_mission_source = self.create_publisher(
            PlanningMissionKey, self.mission_source_topic, 10
        )
        self.pub_scenario_id = self.create_publisher(PlanningScenario, self.scenario_id_topic, 10)

        self.create_subscription(StatusArray, self.state_status_topic, self._on_state, 10)
        self.create_subscription(AvgPoseStamped, self.pose_topic, self._on_pose, 10)
        if self.return_start_vehicle_pose_topic == self.pose_topic:
            # HH_260721 - A shared topic is valid when an integrator explicitly
            # disables the independent localization-versus-lanelet comparison.
            self.last_return_start_vehicle_pose = self.last_pose
        elif self.return_start_vehicle_pose_topic:
            self.create_subscription(
                AvgPoseStamped,
                self.return_start_vehicle_pose_topic,
                self._on_return_start_vehicle_pose,
                10,
            )
        self.create_subscription(AvgPoseStamped, self.goal_topic, self._on_goal, 10)
        if self.enable_raw_drop_zone_goal_match and self.raw_goal_topic:
            # HH_260720 - RViz raw goal remains an explicit ROS boundary.
            self.create_subscription(RosPoseStamped, self.raw_goal_topic, self._on_raw_goal, 10)
        self.create_subscription(GoalStatusArray, self.nav_status_topic, self._on_nav_status, 10)
        if self.enable_maneuver_phase_state_override:
            self.create_subscription(
                AvgAmrServiceState,
                self.maneuver_phase_state_topic,
                self._on_maneuver_phase_state,
                10,
            )
        self.create_subscription(
            PlanningRecallRequest, self.return_topic, self._on_return_to_drop_zone, 10
        )
        self.create_subscription(
            PlanningRecallRequest, self.recall_topic, self._on_camping_site_recall, 10
        )
        self.create_subscription(
            PlanningMissionKey, self.mission_key_topic, self._on_mission_key_request, 10
        )
        self.create_subscription(
            PlanningScenario, self.scenario_command_topic, self._on_scenario_command, 10
        )
        self.create_service(RequestGoalByKey, self.request_mission_service, self._on_mission_key_service)

        period = 1.0 / max(0.5, self.loop_rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            "planning_state_machine: "
            f"state={self.state_status_topic} "
            f"pose={self.pose_topic} "
            f"return_start_vehicle_pose={self.return_start_vehicle_pose_topic or '(disabled)'} "
            f"return_start_pose_max_distance_m={self.return_start_pose_max_distance_m:.2f} "
            f"goal={self.goal_topic} "
            f"raw_goal={self.raw_goal_topic if self.enable_raw_drop_zone_goal_match else '(disabled)'} "
            f"goal_ros={self.goal_topic_ros} "
            f"auto_goal_snapper_input={self.auto_goal_snapper_input_topic or '(disabled)'} "
            f"auto_goal_snapper_keys={','.join(sorted(self.auto_goal_snapper_keys)) or '(none)'} "
            f"drop_zone_id={self.drop_zone_id} "
            f"drop_zone_goal_raw={self.drop_zone_goal_raw_topic or '(disabled)'} "
            f"drop_zone_goal_raw_ros={self.drop_zone_goal_raw_ros_topic or '(disabled)'} "
            f"nav_status={self.nav_status_topic} "
            f"maneuver_phase_state={self.maneuver_phase_state_topic if self.enable_maneuver_phase_state_override else '(disabled)'} "
            f"return={self.return_topic} "
            f"recall={self.recall_topic} "
            f"scenario_cmd={self.scenario_command_topic} "
            f"scenario_id={self.scenario_id_topic} "
            f"mission_key_topic={self.mission_key_topic} "
            f"request_mission_service={self.request_mission_service} "
            f"keypoints={','.join(sorted(self.keypoints.keys()))}"
        )

    # Implements `_load_keypoints` behavior.
    def _load_keypoints(self) -> None:
        self.keypoints.clear()
        self.drop_zone_keypoints.clear()
        self.drop_zone_polygons.clear()
        self.selected_return_keypoint = None
        if not self.keypoints_yaml:
            return
        if not os.path.exists(self.keypoints_yaml):
            self.get_logger().warn(f"keypoints_yaml not found: {self.keypoints_yaml}")
            return
        try:
            with open(self.keypoints_yaml, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"failed to read keypoints_yaml {self.keypoints_yaml}: {e}"
            )
            return

        raw_kp = data.get("keypoints", {})
        if isinstance(raw_kp, dict):
            for name, v in raw_kp.items():
                if not isinstance(v, dict):
                    continue
                self.keypoints[name] = Keypoint(
                    name=name,
                    frame_id=str(v.get("frame_id", "map")),
                    x=float(v.get("x", 0.0)),
                    y=float(v.get("y", 0.0)),
                    z=float(v.get("z", 0.0)),
                    yaw_deg=float(v.get("yaw_deg", 0.0)),
                    source_id=str(v.get("id", name)),
                )

        # HH_260624 - Directly consume map/drop_zones.yaml so return goals use exported station centers.
        self._merge_drop_zones(data)

        self._merge_camping_sites(data)

        if self.camping_sites_yaml:
            if not os.path.exists(self.camping_sites_yaml):
                self.get_logger().warn(
                    f"camping_sites_yaml not found: {self.camping_sites_yaml}"
                )
            else:
                try:
                    with open(self.camping_sites_yaml, "r", encoding="utf-8") as f:
                        camping_data = yaml.safe_load(f) or {}
                    self._merge_camping_sites(camping_data)
                except Exception as e:  # noqa: BLE001
                    self.get_logger().error(
                        f"failed to read camping_sites_yaml {self.camping_sites_yaml}: {e}"
                    )

        if "garage" not in self.keypoints and "drop_zone" in self.keypoints:
            dz = self.keypoints["drop_zone"]
            self.keypoints["garage"] = Keypoint(
                name="garage",
                frame_id=dz.frame_id,
                x=dz.x,
                y=dz.y,
                z=dz.z,
                yaw_deg=dz.yaw_deg,
                source_id=dz.source_id,
            )

    # HH_260624 - Preserve exported drop-zone id/yaw and select the configured return station.
    def _merge_drop_zones(self, data: dict) -> None:
        raw_zones = data.get("drop_zones", [])
        if not isinstance(raw_zones, list):
            return

        selected: Optional[Keypoint] = None
        first_zone: Optional[Keypoint] = None
        for index, zone in enumerate(raw_zones, start=1):
            if not isinstance(zone, dict):
                continue
            zone_id = str(zone.get("id", f"drop_zone_{index}")).strip() or f"drop_zone_{index}"
            zone_type = str(zone.get("type", "drop_zone")).strip() or "drop_zone"
            keypoint = Keypoint(
                name=zone_id,
                frame_id=str(zone.get("frame_id", "map")),
                x=float(zone.get("x", 0.0)),
                y=float(zone.get("y", 0.0)),
                z=float(zone.get("z", 0.0)),
                yaw_deg=float(zone.get("yaw_deg", 0.0)),
                source_id=zone_id,
            )
            self.drop_zone_keypoints.append(keypoint)
            self.keypoints.setdefault(zone_id, keypoint)
            self.keypoints.setdefault(f"drop_zone_{index}", keypoint)
            corners = self._extract_xy_polygon(zone.get("corners"))
            if corners:
                self.drop_zone_polygons[zone_id] = corners
            if first_zone is None:
                first_zone = keypoint
            if selected is None and (zone_id == self.drop_zone_id or zone_type == self.drop_zone_id):
                selected = keypoint

        selected = selected or first_zone
        if selected is None:
            return

        self._select_return_keypoint(selected, "configured_selector")

    def _drop_zone_selector_is_generic(self) -> bool:
        selector = self.drop_zone_id.strip()
        return selector in {"", self.return_mission_key, "drop_zone"}

    def _select_return_keypoint(self, selected: Keypoint, reason: str) -> None:
        # HH_260624 - Keep the canonical drop_zone key mapped to the selected
        # physical area so planning, RViz raw target, and parking station agree.
        canonical = Keypoint(
            name=self.return_mission_key,
            frame_id=selected.frame_id,
            x=selected.x,
            y=selected.y,
            z=selected.z,
            yaw_deg=selected.yaw_deg,
            source_id=selected.source_id,
        )
        self.selected_return_keypoint = canonical
        self.keypoints[self.return_mission_key] = canonical
        self.get_logger().info(
            "selected drop-zone return keypoint: "
            f"selector={self.drop_zone_id} reason={reason} "
            f"id={canonical.source_id or canonical.name} "
            f"xy=({canonical.x:.2f},{canonical.y:.2f}) yaw={canonical.yaw_deg:.1f}deg"
        )

    def _select_nearest_return_keypoint_for_current_pose(self, reason: str) -> None:
        if not self._drop_zone_selector_is_generic() or len(self.drop_zone_keypoints) <= 1:
            return
        if self.last_pose is None:
            return
        pose_frame = str(self.last_pose.header.frame_id).strip()
        pose_x = float(self.last_pose.pose.position.x)
        pose_y = float(self.last_pose.pose.position.y)
        nearest: Optional[Keypoint] = None
        nearest_dist = float("inf")
        for keypoint in self.drop_zone_keypoints:
            kp_frame = str(keypoint.frame_id).strip()
            if pose_frame and kp_frame and pose_frame != kp_frame:
                continue
            dist = math.hypot(pose_x - keypoint.x, pose_y - keypoint.y)
            if dist < nearest_dist:
                nearest = keypoint
                nearest_dist = dist
        if nearest is None:
            return
        current_id = self.selected_return_keypoint.source_id if self.selected_return_keypoint else ""
        if current_id != nearest.source_id:
            self._select_return_keypoint(nearest, f"{reason}:nearest_current_pose")

    @staticmethod
    def _extract_xy_polygon(raw_corners: object) -> list[tuple[float, float]]:
        if not isinstance(raw_corners, list):
            return []
        polygon: list[tuple[float, float]] = []
        for corner in raw_corners:
            if not isinstance(corner, dict):
                continue
            try:
                polygon.append((float(corner["x"]), float(corner["y"])))
            except (KeyError, TypeError, ValueError):
                continue
        return polygon if len(polygon) >= 3 else []

    @staticmethod
    def _point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
        inside = False
        count = len(polygon)
        j = count - 1
        for i in range(count):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            denom = yj - yi
            if abs(denom) < 1e-12:
                denom = 1e-12
            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / denom + xi
            ):
                inside = not inside
            j = i
        return inside

    # Implements `_merge_camping_sites` behavior.
    def _merge_camping_sites(self, data: dict) -> None:
        raw_sites = data.get("camping_sites", [])
        if not isinstance(raw_sites, list):
            return
        for index, site in enumerate(raw_sites, start=1):
            if not isinstance(site, dict):
                continue

            key_name = str(site.get("type", "")).strip() or f"camping_site_{index}"
            frame_id = str(site.get("frame_id", "map"))
            if key_name not in self.keypoints:
                # HH_260721 - Route inaccessible campsites to their map-authored roadside service pose.
                self.keypoints[key_name] = Keypoint(
                    name=key_name,
                    frame_id=frame_id,
                    x=float(site.get("service_x", site.get("x", 0.0))),
                    y=float(site.get("service_y", site.get("y", 0.0))),
                    z=float(site.get("service_z", site.get("z", 0.0))),
                    yaw_deg=float(
                        site.get("service_yaw_deg", site.get("yaw_deg", 0.0))
                    ),
                )

            # HH_260528 Recall should target lanelet-snapped road point when provided.
            road_key = f"{key_name}_road"
            if road_key not in self.keypoints and "recall_x" in site:
                self.keypoints[road_key] = Keypoint(
                    name=road_key,
                    frame_id=frame_id,
                    x=float(site.get("recall_x", site.get("x", 0.0))),
                    y=float(site.get("recall_y", site.get("y", 0.0))),
                    z=float(site.get("recall_z", site.get("z", 0.0))),
                )

    @staticmethod
    def _status_level_int(level_value: object) -> int:
        if isinstance(level_value, (bytes, bytearray)):
            if len(level_value) > 0:
                return int(level_value[0])
            return int(StatusStatus.OK)
        return int(level_value)

    @staticmethod
    def _extract_module(st: StatusStatus) -> str:
        for kv in st.values:
            if kv.key == "category" and kv.value:
                return kv.value
        if st.name:
            parts = [p for p in st.name.strip("/").split("/") if p]
            if parts:
                return parts[0]
        if st.hardware_id:
            return st.hardware_id
        return ""

    @staticmethod
    def _param_string_set(value: object) -> Set[str]:
        if isinstance(value, str):
            raw_items = value.split(",")
        else:
            try:
                raw_items = list(value)
            except TypeError:
                raw_items = [value]
        return {str(item).strip() for item in raw_items if str(item).strip()}

    def _ignore_state_status(self, st: StatusStatus) -> bool:
        status_name = str(getattr(st, "name", "") or "")
        if status_name in self.state_status_ignored_names:
            return True
        return any(
            status_name.startswith(prefix)
            for prefix in self.state_status_ignored_prefixes
        )

    def _on_state(self, msg: StatusArray) -> None:
        self.last_state_stamp = self.get_clock().now()
        current_levels: Dict[str, int] = {}
        for st in msg.status:
            if self._ignore_state_status(st):
                continue
            module = self._extract_module(st)
            if not module:
                continue
            level = self._status_level_int(st.level)
            # HH_260617: Always record OK levels too. The previous logic only
            # stored levels greater than OK, so once an ERROR was observed it
            # never cleared when /system/diagnostics_agg returned to all OK.
            prev = current_levels.get(module)
            if prev is None or level > prev:
                current_levels[module] = level
        self.module_levels = current_levels

    def _on_pose(self, msg: AvgPoseStamped) -> None:
        self.last_pose = msg
        self.last_pose_received_time = self.get_clock().now()
        # HH_260721 - Preserve explicit same-topic configurations without a
        # duplicate subscription or callback ordering dependency.
        if self.return_start_vehicle_pose_topic == self.pose_topic:
            self.last_return_start_vehicle_pose = msg
            self.last_return_start_vehicle_pose_received_time = self.last_pose_received_time

    def _on_return_start_vehicle_pose(self, msg: AvgPoseStamped) -> None:
        # HH_260721 - The unsnapped vehicle pose is used only to reject a stale
        # lanelet start pose before a drop-zone return route is created.
        self.last_return_start_vehicle_pose = msg
        self.last_return_start_vehicle_pose_received_time = self.get_clock().now()

    def _return_start_pose_ready(self) -> bool:
        # HH_260721 - A non-positive distance explicitly disables this guard for
        # specialized deployments while preserving the existing route behavior.
        if self.return_start_pose_max_distance_m <= 0.0:
            return True
        if self.last_pose is None or self.last_return_start_vehicle_pose is None:
            self._log_return_start_pose_wait("waiting for lanelet and vehicle poses")
            return False
        if (
            self.last_pose.header.frame_id
            and self.last_return_start_vehicle_pose.header.frame_id
            and self.last_pose.header.frame_id
            != self.last_return_start_vehicle_pose.header.frame_id
        ):
            self._log_return_start_pose_wait(
                "frame mismatch "
                f"lanelet={self.last_pose.header.frame_id} "
                f"vehicle={self.last_return_start_vehicle_pose.header.frame_id}"
            )
            return False

        now = self.get_clock().now()
        stale_timeout_s = max(0.0, self.return_start_pose_stale_timeout_s)
        if stale_timeout_s > 0.0:
            if (
                self.last_pose_received_time is None
                or self.last_return_start_vehicle_pose_received_time is None
            ):
                self._log_return_start_pose_wait("waiting for fresh pose timestamps")
                return False
            lanelet_age_s = (now - self.last_pose_received_time).nanoseconds / 1e9
            vehicle_age_s = (
                now - self.last_return_start_vehicle_pose_received_time
            ).nanoseconds / 1e9
            if lanelet_age_s > stale_timeout_s or vehicle_age_s > stale_timeout_s:
                self._log_return_start_pose_wait(
                    f"stale pose lanelet_age={lanelet_age_s:.2f}s "
                    f"vehicle_age={vehicle_age_s:.2f}s"
                )
                return False

        distance_m = self._dist_xy(self.last_pose, self.last_return_start_vehicle_pose)
        if distance_m > self.return_start_pose_max_distance_m:
            self._log_return_start_pose_wait(
                f"lanelet/vehicle distance={distance_m:.2f}m "
                f"limit={self.return_start_pose_max_distance_m:.2f}m"
            )
            return False
        return True

    def _log_return_start_pose_wait(self, reason: str) -> None:
        # HH_260721 - Return retries run at 5 Hz, so rate-limit this operational
        # message while retaining the exact reason in logs.
        now = self.get_clock().now()
        if self._last_return_start_wait_log_time is not None:
            elapsed_s = (
                now - self._last_return_start_wait_log_time
            ).nanoseconds / 1e9
            if elapsed_s < 2.0:
                return
        self._last_return_start_wait_log_time = now
        self.get_logger().warn(f"delaying drop-zone return route: {reason}")

    def _on_raw_goal(self, msg: RosPoseStamped) -> None:
        matched = self._match_drop_zone_raw_goal(msg)
        if matched is None:
            return

        self._select_return_keypoint(matched, "manual_raw_goal")
        self._pending_mission_key_request = self.return_mission_key
        self._pending_mission_key_time = self.get_clock().now()
        self._publish_drop_zone_goal_raw_for_keypoint(matched)
        self.get_logger().info(
            "raw drop-zone goal matched: "
            f"id={matched.source_id or matched.name} "
            f"xy=({matched.x:.2f},{matched.y:.2f}) yaw={matched.yaw_deg:.1f}deg"
        )

    @staticmethod
    def _dist_xy(a: AvgPoseStamped, b: AvgPoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)

    def _match_mission_key(self, goal: AvgPoseStamped) -> str:
        best_name = ""
        best_dist = float("inf")
        goal_frame = str(goal.header.frame_id).strip()
        for name, kp in self.keypoints.items():
            kp_frame = str(kp.frame_id).strip()
            if goal_frame and kp_frame and goal_frame != kp_frame:
                continue
            d = math.hypot(goal.pose.position.x - kp.x, goal.pose.position.y - kp.y)
            if d < best_dist:
                best_dist = d
                best_name = name
        if best_name and best_dist <= self.mission_key_match_distance_m:
            return best_name
        return ""

    def _match_drop_zone_raw_goal(self, goal: RosPoseStamped) -> Optional[Keypoint]:
        if not self.drop_zone_keypoints:
            return None
        goal_frame = str(goal.header.frame_id).strip()
        x = float(goal.pose.position.x)
        y = float(goal.pose.position.y)
        nearest: Optional[Keypoint] = None
        nearest_dist = float("inf")
        for keypoint in self.drop_zone_keypoints:
            kp_frame = str(keypoint.frame_id).strip()
            if goal_frame and kp_frame and goal_frame != kp_frame:
                continue
            polygon = self.drop_zone_polygons.get(keypoint.source_id or keypoint.name, [])
            if polygon and self._point_in_polygon(x, y, polygon):
                return keypoint
            dist = math.hypot(x - keypoint.x, y - keypoint.y)
            if dist < nearest_dist:
                nearest = keypoint
                nearest_dist = dist
        if nearest is not None and nearest_dist <= max(0.0, self.drop_zone_raw_goal_match_distance_m):
            return nearest
        return None

    # HH_260624 - Use the selected drop-zone keypoint for every return-to-drop-zone path.
    def _return_keypoint(self) -> Optional[Keypoint]:
        return self.selected_return_keypoint or self.keypoints.get(self.return_mission_key)

    # HH_260624 - Resolve semantic keys through the drop-zone selector when needed.
    def _goal_keypoint(self, key_name: str) -> Optional[Keypoint]:
        if key_name == self.return_mission_key:
            return self._return_keypoint()
        return self.keypoints.get(key_name)

    def _publish_drop_zone_goal_raw_for_keypoint(self, keypoint: Keypoint) -> None:
        if self.pub_drop_zone_goal_raw is None:
            return
        msg = AvgPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = keypoint.frame_id
        msg.pose.position.x = keypoint.x
        msg.pose.position.y = keypoint.y
        msg.pose.position.z = keypoint.z
        yaw = math.radians(float(keypoint.yaw_deg))
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        self.pub_drop_zone_goal_raw.publish(msg)
        if self.pub_drop_zone_goal_raw_ros is not None:
            # HH_260720 - Convert the generated semantic target only at the RViz boundary.
            ros_msg = RosPoseStamped()
            ros_msg.header.stamp = msg.header.stamp
            ros_msg.header.frame_id = msg.header.frame_id
            ros_msg.pose.position.x = msg.pose.position.x
            ros_msg.pose.position.y = msg.pose.position.y
            ros_msg.pose.position.z = msg.pose.position.z
            ros_msg.pose.orientation.x = msg.pose.orientation.x
            ros_msg.pose.orientation.y = msg.pose.orientation.y
            ros_msg.pose.orientation.z = msg.pose.orientation.z
            ros_msg.pose.orientation.w = msg.pose.orientation.w
            self.pub_drop_zone_goal_raw_ros.publish(ros_msg)

    def _is_site_key(self, key: str) -> bool:
        return bool(key) and key.startswith(self.site_mission_key_prefix) and not key.endswith("_road")

    def _default_site_key(self) -> str:
        if self.default_recall_mission_key in self.keypoints:
            return self.default_recall_mission_key
        site_keys = sorted(
            [k for k in self.keypoints.keys() if self._is_site_key(k)]
        )
        if site_keys:
            return site_keys[0]
        return self.return_mission_key

    def _resolve_recall_target_key(self, site_name: str) -> str:
        site_name = site_name.strip()
        if site_name:
            road_key = f"{site_name}_road"
            if road_key in self.keypoints:
                return road_key
            if site_name in self.keypoints:
                return site_name
        fallback = self._default_site_key()
        road_fallback = f"{fallback}_road"
        if road_fallback in self.keypoints:
            return road_fallback
        return fallback

    def _set_scenario(self, scenario_id: int, reason: str) -> None:
        scenario_id = int(scenario_id)
        if scenario_id != self.SCENARIO_WAIT_DROP_ZONE:
            self._drop_zone_arrival_notified = False
            self._drop_zone_arrival_notified_time = None
        if scenario_id == self.scenario_id:
            return
        self.scenario_id = scenario_id
        self.get_logger().info(f"scenario_id -> {self.scenario_id} ({reason})")

    def _on_goal(self, msg: AvgPoseStamped) -> None:
        if self._last_self_goal is not None and self._dist_xy(self._last_self_goal, msg) < 0.02:
            return

        self._clear_maneuver_phase_override("new_route_goal")
        raw_matched_mission_key = self._match_mission_key(msg)
        matched_mission_key = raw_matched_mission_key
        goal_source = "manual"
        # HH_260616: A camping-site center can be several meters away from the
        # lanelet-snapped route_goal. If a mission_key request just preceded this
        # route_goal, keep that semantic key instead of clearing active_mission_key.
        if self._pending_mission_key_request:
            pending_age_s = (
                self.get_clock().now() - self._pending_mission_key_time
            ).nanoseconds / 1e9
            if pending_age_s <= self.pending_mission_key_preserve_s and (
                self.pending_mission_key_overrides_goal_match or not matched_mission_key
            ):
                if (
                    raw_matched_mission_key
                    and raw_matched_mission_key != self._pending_mission_key_request
                ):
                    self.get_logger().warn(
                        "pending mission_key overrides snapped-goal key match: "
                        f"pending={self._pending_mission_key_request} "
                        f"matched={raw_matched_mission_key}"
                    )
                matched_mission_key = self._pending_mission_key_request
                goal_source = f"auto_snapper:{self._pending_mission_key_request}"
            self._pending_mission_key_request = ""
        if (
            not matched_mission_key
            and self.scenario_id == self.SCENARIO_RETURN_TO_DROP_ZONE
            and self.active_mission_key == self.return_mission_key
            and self.active_goal is not None
            and self._dist_xy(self.active_goal, msg) <= self.return_goal_key_preserve_distance_m
        ):
            # HH_260618: Do not let a near-identical route-goal echo clear the
            # HH_260720 - Preserve the drop_zone semantic key for alignment and parking.
            matched_mission_key = self.return_mission_key
        if (
            not matched_mission_key
            and self._is_site_key(self.active_mission_key)
            and self.active_goal is not None
            and self._dist_xy(self.active_goal, msg) <= self.site_goal_key_preserve_distance_m
        ):
            # HH_260720 - Preserve campsite ownership for duplicate snapped-goal echoes;
            # the camping-site controller starts from GOAL_REACHED plus the mission key.
            matched_mission_key = self.active_mission_key

        self.last_manual_goal = msg
        self.active_goal = msg
        self.active_goal_source = goal_source
        self.active_mission_key = matched_mission_key
        self._reset_nav2_goal_status()
        self.startup_goal_sent = True
        self.return_requested = False
        self.recall_requested = False
        self._goal_reached_since = None
        self._goal_reached_latched = False
        self._drop_zone_arrival_notified = False

        if self._is_site_key(self.active_mission_key):
            self._set_scenario(self.SCENARIO_DELIVERY_TO_SITE, "manual_site_goal")
        elif self.active_mission_key == self.return_mission_key:
            reason = "auto_return_goal" if goal_source.startswith("auto_snapper:") else "manual_return_goal"
            self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, reason)
        else:
            # HH_260618: A fresh unmatched manual/UI goal must enter a driving
            # scenario instead of leaving stale WAIT_DZ/RETURN_TO_DROP_ZONE state.
            # Otherwise the state output and route ownership can disagree with
            # the operator's latest goal.
            self._set_scenario(self.SCENARIO_DELIVERY_TO_SITE, "manual_goal")

    def _reset_nav2_goal_status(self) -> None:
        # HH_260622 - Treat every route-goal update as a new Nav2 handoff.
        self._active_goal_time = self.get_clock().now()
        self._nav2_goal_succeeded = False
        self._nav2_terminal_status = 0
        self._nav2_terminal_time = None

    def _goal_status_time(self, status: GoalStatus) -> Optional[rclpy.time.Time]:
        stamp = status.goal_info.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None
        return rclpy.time.Time.from_msg(stamp)

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        if self.active_goal is None:
            return
        active_time = self._active_goal_time
        tolerance_ns = int(max(0.0, self.nav_status_goal_match_tolerance_s) * 1e9)
        latest_terminal: Optional[GoalStatus] = None
        latest_terminal_time: Optional[rclpy.time.Time] = None
        for status in msg.status_list:
            if status.status not in {
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            }:
                continue
            status_time = self._goal_status_time(status)
            if status_time is not None:
                if (status_time - active_time).nanoseconds < -tolerance_ns:
                    continue
            if latest_terminal_time is None or (
                status_time is not None and status_time > latest_terminal_time
            ):
                latest_terminal = status
                latest_terminal_time = status_time

        if latest_terminal is None:
            return

        self._nav2_terminal_status = int(latest_terminal.status)
        self._nav2_terminal_time = self.get_clock().now()
        self._nav2_goal_succeeded = latest_terminal.status == GoalStatus.STATUS_SUCCEEDED

    def _clear_maneuver_phase_override(self, reason: str) -> None:
        if self._maneuver_phase_override_scenario_id is None:
            return
        self.get_logger().info(
            "maneuver phase override cleared: "
            f"scenario={self._maneuver_phase_override_scenario_id} reason={reason}"
        )
        self._maneuver_phase_override_state = ""
        self._maneuver_phase_override_scenario_id = None
        self._maneuver_phase_override_source = ""
        self._maneuver_phase_override_time = None

    def _on_maneuver_phase_state(self, msg: AvgAmrServiceState) -> None:
        description = str(msg.description).strip()
        source = description.split(":", 1)[0] if ":" in description else ""
        # HH_260720 - Accept phase updates only from explicit control/parking owners.
        if source not in {"camping_site_maneuver_controller", "drop_zone_maneuver_controller", "reverse_parking_controller"}:
            return

        state_id = int(msg.state)
        if (
            source == "camping_site_maneuver_controller"
            and state_id == int(AvgAmrServiceState.RETURN_WITH_CARGO)
            and "DONE" in description
            and self.scenario_id == self.SCENARIO_RETURN_TO_DROP_ZONE
            and self.active_mission_key == self.return_mission_key
        ):
            # HH_260701 - camping_site_maneuver_controller DONE can arrive just after it requests
            # the drop-zone route. Do not let that late phase event mask the
            # RETURN_TO_DROP_ZONE GOAL_REACHED handoff used by drop_zone_maneuver_controller.
            self.get_logger().info(
                "ignored camping_site_maneuver_controller DONE override during active drop-zone return"
            )
            return
        override_state = ""
        override_scenario: Optional[int] = None
        if state_id == int(AvgAmrServiceState.SITE_ENTRY):
            override_state = "RUNNING"
            override_scenario = self.SCENARIO_SITE_ENTRY
        elif state_id == int(AvgAmrServiceState.UNLOAD_WAIT):
            override_state = "GOAL_REACHED"
            override_scenario = self.SCENARIO_UNLOAD_WAIT
        elif state_id == int(AvgAmrServiceState.RECALL_TO_SITE_ROAD):
            override_state = "RUNNING"
            override_scenario = self.SCENARIO_RECALL_TO_SITE_ROAD
        elif state_id == int(AvgAmrServiceState.GUEST_LOADING_WAIT):
            override_state = "GOAL_REACHED"
            override_scenario = self.SCENARIO_GUEST_LOADING_WAIT
        elif state_id == int(AvgAmrServiceState.RETURN_WITH_CARGO):
            override_state = "RETURNING"
            override_scenario = self.SCENARIO_RETURN_WITH_CARGO
        elif state_id == int(AvgAmrServiceState.DROP_ZONE_PARKING):
            override_state = "RUNNING"
            override_scenario = self.SCENARIO_DROP_ZONE_PARKING
        elif state_id == int(AvgAmrServiceState.DROP_ZONE_WAIT):
            override_state = "WAIT_DZ"
            override_scenario = self.SCENARIO_WAIT_DROP_ZONE

        if override_scenario is None:
            return

        self._maneuver_phase_override_state = override_state
        self._maneuver_phase_override_scenario_id = int(override_scenario)
        self._maneuver_phase_override_source = description
        self._maneuver_phase_override_time = self.get_clock().now()
        self.get_logger().info(
            "maneuver phase override: "
            f"state={override_state} scenario={override_scenario} source={description}"
        )

    def _active_maneuver_phase_override(self) -> tuple[str, Optional[int], str]:
        if self._maneuver_phase_override_scenario_id is None or self._maneuver_phase_override_time is None:
            return "", None, ""
        # HH_260720 - Maneuver phase messages are state-transition events, not
        # periodic telemetry. A non-positive timeout keeps the latest phase
        # authoritative until a new route/mission/return command explicitly
        # clears it.
        if self.maneuver_phase_override_timeout_s <= 0.0:
            return (
                self._maneuver_phase_override_state,
                self._maneuver_phase_override_scenario_id,
                self._maneuver_phase_override_source,
            )
        age_s = (self.get_clock().now() - self._maneuver_phase_override_time).nanoseconds / 1e9
        if age_s > max(0.1, self.maneuver_phase_override_timeout_s):
            self._clear_maneuver_phase_override("timeout")
            return "", None, ""
        return (
            self._maneuver_phase_override_state,
            self._maneuver_phase_override_scenario_id,
            self._maneuver_phase_override_source,
        )

    def _on_return_to_drop_zone(self, msg: PlanningRecallRequest) -> None:
        # HH_260720 - A recall request carries its origin instead of a context-free Bool.
        # HH_260623 - Do not publish another drop-zone route when the robot is
        # already at the drop-zone keypoint or in the return-arrival handoff.
        # The duplicate route looked like a small forward goal in RViz and could
        # interrupt the reverse-parking sequence.
        if (
            self.scenario_id == self.SCENARIO_WAIT_DROP_ZONE
            and self._return_keypoint_reached()
        ) or (
            self.scenario_id == self.SCENARIO_RETURN_TO_DROP_ZONE
            and self._drop_zone_arrived_with_condition()
        ):
            self.return_requested = False
            self.get_logger().info("ignored return_to_drop_zone: already at drop_zone")
            return
        self._clear_maneuver_phase_override("return_to_drop_zone")
        self.return_requested = True
        if self._publish_auto_goal(self.return_mission_key, "return_request", force=True):
            self.return_requested = False
            self.warn_goal_sent = False
            self._set_scenario(
                self.SCENARIO_RETURN_TO_DROP_ZONE,
                msg.source.strip() or "return_topic",
            )

    def _on_camping_site_recall(self, msg: PlanningRecallRequest) -> None:
        site_name = msg.site_name.strip()
        target_key = self._resolve_recall_target_key(site_name)
        self.recall_target_key = target_key
        self.recall_site_name = site_name
        self.last_recalled_mission_key = site_name or self.last_recalled_mission_key
        self.recall_requested = True

        if self._publish_auto_goal(target_key, f"recall:{site_name or 'unspecified'}", force=True):
            self.recall_requested = False
            self.warn_goal_sent = False
            self._set_scenario(self.SCENARIO_RECALL_TO_SITE, "recall_topic")

    def _on_scenario_command(self, msg: PlanningScenario) -> None:
        requested = int(msg.scenario_id)
        self._clear_maneuver_phase_override("scenario_command")
        if requested not in (
            self.SCENARIO_WAIT_DROP_ZONE,
            self.SCENARIO_DELIVERY_TO_SITE,
            self.SCENARIO_RETURN_TO_DROP_ZONE,
            self.SCENARIO_RECALL_TO_SITE,
        ):
            self.get_logger().warn(f"unsupported scenario command: {requested}")
            return

        if requested == self.SCENARIO_DELIVERY_TO_SITE:
            site_key = self.active_mission_key if self._is_site_key(self.active_mission_key) else self._default_site_key()
            if self._publish_auto_goal(site_key, "scenario:1", force=True):
                self._set_scenario(self.SCENARIO_DELIVERY_TO_SITE, "scenario_command")
                self.return_requested = False
                self.recall_requested = False
            return

        if requested == self.SCENARIO_RECALL_TO_SITE:
            site_name = self.last_recalled_mission_key or self._default_site_key()
            target_key = self._resolve_recall_target_key(site_name)
            if self._publish_auto_goal(target_key, f"scenario:3:{site_name}", force=True):
                self._set_scenario(self.SCENARIO_RECALL_TO_SITE, "scenario_command")
                self.return_requested = False
                self.recall_requested = False
            return

        if requested == self.SCENARIO_RETURN_TO_DROP_ZONE:
            self.return_requested = True
            if self._publish_auto_goal(self.return_mission_key, "scenario:2", force=True):
                self.return_requested = False
                self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, "scenario_command")
            return

        # requested == SCENARIO_WAIT_DROP_ZONE
        if self._drop_zone_arrived_with_condition():
            self._set_scenario(self.SCENARIO_WAIT_DROP_ZONE, "scenario_command_idle")
            self.return_requested = False
            self.recall_requested = False
        else:
            # HH_260528 If not at drop-zone yet, command 0 means return first.
            self.return_requested = True
            if self._publish_auto_goal(self.return_mission_key, "scenario:0_to_return", force=True):
                self.return_requested = False
                self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, "scenario0_requires_return")

    def _state_level(self) -> int:
        if self.last_state_stamp is None:
            return self._ok_level

        age = (self.get_clock().now() - self.last_state_stamp).nanoseconds / 1e9
        if age > self.state_stale_timeout_s:
            return self._warn_level

        if not self.module_levels:
            return self._ok_level
        return max(self.module_levels.values())

    def _publish_auto_goal(self, key_name: str, source: str, force: bool = False) -> bool:
        if key_name == self.return_mission_key:
            # HH_260721 - A campsite maneuver moves outside the lanelet planner.
            # Wait for centerline snapping to reflect the completed crab exit
            # before Nav2 captures its route start pose.
            if not self._return_start_pose_ready():
                return False
            self._select_nearest_return_keypoint_for_current_pose(source)
        kp = self._goal_keypoint(key_name)
        if kp is None:
            self.get_logger().warn(f"keypoint '{key_name}' not found")
            return False

        now = self.get_clock().now()
        dt = (now - self._last_goal_publish_time).nanoseconds / 1e9
        if not force and dt < self.min_goal_publish_interval_s:
            return False

        msg = AvgPoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = kp.frame_id
        msg.pose.position.x = kp.x
        msg.pose.position.y = kp.y
        msg.pose.position.z = kp.z
        yaw = math.radians(float(kp.yaw_deg))
        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)

        publish_via_snapper = (
            self.pub_auto_goal_snapper is not None
            and key_name in self.auto_goal_snapper_keys
        )
        if publish_via_snapper:
            self.pub_auto_goal_snapper.publish(msg)
            if key_name == self.return_mission_key and self.pub_drop_zone_goal_raw is not None:
                self._publish_drop_zone_goal_raw_for_keypoint(kp)
            self._pending_mission_key_request = key_name
            self._pending_mission_key_time = now
            self._last_self_goal = None
            self.get_logger().info(
                "published raw auto-goal through goal_snapper: "
                f"key={key_name} source={source} topic={self.auto_goal_snapper_input_topic} "
                f"drop_zone_id={kp.source_id or kp.name} yaw={kp.yaw_deg:.1f}deg "
                f"xy=({kp.x:.2f},{kp.y:.2f})"
            )
        else:
            self.pub_goal.publish(msg)
            if self.pub_goal_ros is not None:
                # HH_260720 - Convert the generated goal only at the Nav2 ROS boundary.
                ros_goal = RosPoseStamped()
                ros_goal.header.stamp = msg.header.stamp
                ros_goal.header.frame_id = msg.header.frame_id
                ros_goal.pose.position.x = msg.pose.position.x
                ros_goal.pose.position.y = msg.pose.position.y
                ros_goal.pose.position.z = msg.pose.position.z
                ros_goal.pose.orientation.x = msg.pose.orientation.x
                ros_goal.pose.orientation.y = msg.pose.orientation.y
                ros_goal.pose.orientation.z = msg.pose.orientation.z
                ros_goal.pose.orientation.w = msg.pose.orientation.w
                self.pub_goal_ros.publish(ros_goal)
            self._last_self_goal = msg

        self._last_goal_publish_time = now
        self.active_goal = msg
        self.active_goal_source = f"{source}:snap_request" if publish_via_snapper else source
        self.active_mission_key = key_name
        self._reset_nav2_goal_status()
        self.startup_goal_sent = True
        self._goal_reached_since = None
        self._goal_reached_latched = False
        self._drop_zone_arrival_notified = False
        return True

    def _on_mission_key_request(self, msg: PlanningMissionKey) -> None:
        key_name = msg.mission_key.strip()
        if not key_name:
            self.get_logger().warn("received empty mission-key request on topic")
            return

        self._clear_maneuver_phase_override("mission_key_request")
        if not self.mission_key_publish_route_goal:
            if self._goal_keypoint(key_name) is None:
                self.get_logger().warn(f"keypoint '{key_name}' not found")
                return
            self.active_mission_key = key_name
            self._pending_mission_key_request = key_name
            self._pending_mission_key_time = self.get_clock().now()
            self.return_requested = False
            self.recall_requested = False
            self.warn_goal_sent = False
            if self._is_site_key(key_name):
                self._set_scenario(self.SCENARIO_DELIVERY_TO_SITE, "mission_key_topic")
            elif key_name == self.return_mission_key:
                self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, "mission_key_return")
            self.get_logger().info(
                f"accepted mission key without direct route_goal publish: {key_name}"
            )
            return

        if self._publish_auto_goal(key_name, f"mission_key:{key_name}", force=True):
            self.return_requested = False
            self.recall_requested = False
            self.warn_goal_sent = False
            if self._is_site_key(key_name):
                self._set_scenario(self.SCENARIO_DELIVERY_TO_SITE, "mission_key_topic")
            elif key_name == self.return_mission_key:
                self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, "mission_key_return")
            self.get_logger().info(f"published route_goal from mission key request: {key_name}")

    def _on_mission_key_service(
        self, request: RequestGoalByKey.Request, response: RequestGoalByKey.Response
    ) -> RequestGoalByKey.Response:
        key_name = request.key.strip()
        if not key_name:
            response.accepted = False
            response.message = "empty key"
            return response

        keypoint = self._goal_keypoint(key_name)
        if keypoint is None:
            response.accepted = False
            response.message = f"unknown key: {key_name}"
            return response

        goal_pose = AvgPoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = keypoint.frame_id
        goal_pose.pose.position.x = keypoint.x
        goal_pose.pose.position.y = keypoint.y
        goal_pose.pose.position.z = keypoint.z
        goal_pose.pose.orientation.w = 1.0

        published = self._publish_auto_goal(key_name, f"key_service:{key_name}", force=True)
        response.accepted = bool(published)
        if published:
            self.return_requested = False
            self.recall_requested = False
            self.warn_goal_sent = False
            if self._is_site_key(key_name):
                self._set_scenario(self.SCENARIO_DELIVERY_TO_SITE, "mission_key_service")
            elif key_name == self.return_mission_key:
                self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, "mission_key_service_return")
            response.message = f"route_goal published for mission_key: {key_name}"
            response.goal_pose = goal_pose
            self.get_logger().info(f"published route_goal from mission_key service: {key_name}")
        else:
            response.message = f"route_goal publish throttled for mission_key: {key_name}"
        return response

    def _active_goal_distance(self) -> Optional[float]:
        if self.last_pose is None or self.active_goal is None:
            return None
        if self.last_pose.header.frame_id and self.active_goal.header.frame_id:
            if self.last_pose.header.frame_id != self.active_goal.header.frame_id:
                return None
        return self._dist_xy(self.last_pose, self.active_goal)

    def _goal_reached(self, distance_m: Optional[float] = None) -> bool:
        distance = self._active_goal_distance()
        if distance is None:
            return False
        if distance_m is None:
            distance_m = self.goal_reached_distance_m
        if distance > float(distance_m):
            return False
        if not self.require_nav2_success_for_goal_reached:
            return True
        if not self._nav2_goal_succeeded:
            return False
        if self._nav2_terminal_time is None:
            return False
        age_s = (self.get_clock().now() - self._nav2_terminal_time).nanoseconds / 1e9
        return age_s <= max(0.1, self.nav_success_latch_s)

    def _active_goal_reached_distance_m(self) -> float:
        # HH_260618 - Use a wider handoff threshold only for the return route.
        if self.scenario_id == self.SCENARIO_RETURN_TO_DROP_ZONE:
            return self.return_goal_reached_distance_m
        return self.goal_reached_distance_m

    def _return_keypoint_reached(self) -> bool:
        if self.last_pose is None:
            return False
        return_kp = self._return_keypoint()
        if return_kp is None:
            return False
        pseudo_goal = AvgPoseStamped()
        pseudo_goal.header.frame_id = return_kp.frame_id
        pseudo_goal.pose.position.x = return_kp.x
        pseudo_goal.pose.position.y = return_kp.y
        pseudo_goal.pose.position.z = return_kp.z
        if self.last_pose.header.frame_id != pseudo_goal.header.frame_id:
            return False
        return self._dist_xy(self.last_pose, pseudo_goal) <= self.return_goal_reached_distance_m

    def _drop_zone_arrived_with_condition(self) -> bool:
        if self.active_mission_key != self.return_mission_key:
            # HH_260618 - If the semantic key was lost, still allow drop-zone
            # completion by geometry near the configured return keypoint.
            if not self._return_keypoint_reached():
                return False
        else:
            if not self._goal_reached(self.return_goal_reached_distance_m):
                return False

        # HH_260720 - Planning reports arrival only; reverse parking and control own charging state.
        return True

    def _publish_state_outputs(self, estop: bool) -> None:
        now_msg = self.get_clock().now().to_msg()
        effective_state = self.state
        effective_scenario_id = int(self.scenario_id)
        maneuver_override_state, maneuver_override_scenario_id, maneuver_override_source = (
            self._active_maneuver_phase_override()
        )
        if (
            maneuver_override_scenario_id is not None
            and not estop
            and self.state not in {"ERROR_STOP", "WARN_RECOVERY"}
        ):
            effective_state = maneuver_override_state
            effective_scenario_id = int(maneuver_override_scenario_id)

        state_msg = PlanningState()
        state_msg.header.stamp = now_msg
        state_msg.header.frame_id = "map"
        state_msg.state = int(self._STATE_IDS.get(effective_state, PlanningState.READY))
        state_msg.label = effective_state
        state_msg.scenario_id = int(effective_scenario_id)
        state_msg.scenario_label = self._SCENARIO_LABELS.get(effective_scenario_id, "UNKNOWN")
        state_msg.active_mission_key = self.active_mission_key
        state_msg.active_goal_source = self.active_goal_source
        state_msg.estop = bool(estop)
        state_msg.return_requested = bool(self.return_requested)
        state_msg.recall_requested = bool(self.recall_requested)
        self.pub_state.publish(state_msg)

        mission_msg = PlanningMissionKey()
        mission_msg.header.stamp = now_msg
        mission_msg.mission_key = self.active_mission_key
        mission_msg.source = self.active_goal_source
        mission_msg.publish_route_goal = bool(self.mission_key_publish_route_goal)
        self.pub_mission_source.publish(mission_msg)

        scenario_msg = PlanningScenario()
        scenario_msg.header.stamp = now_msg
        scenario_msg.scenario_id = int(effective_scenario_id)
        scenario_msg.label = self._SCENARIO_LABELS.get(effective_scenario_id, "UNKNOWN")
        scenario_msg.source = self.active_goal_source
        self.pub_scenario_id.publish(scenario_msg)

        b = AvgBool()
        b.data = bool(estop)
        self.pub_estop.publish(b)

        diag = StatusArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = StatusStatus()
        st.name = "planning/state_machine"
        st.hardware_id = "planning"
        st.level = _diag_level(StatusStatus.OK)
        if estop:
            st.level = _diag_level(StatusStatus.ERROR)
            st.message = "estop_asserted"
        elif effective_state.startswith("WARN"):
            st.level = _diag_level(StatusStatus.WARN)
            st.message = "warn_recovery"
        else:
            st.message = effective_state.lower()

        st.values.append(KeyValue(key="state", value=effective_state))
        st.values.append(KeyValue(key="raw_state", value=self.state))
        st.values.append(KeyValue(key="scenario_id", value=str(effective_scenario_id)))
        st.values.append(KeyValue(key="raw_scenario_id", value=str(self.scenario_id)))
        st.values.append(KeyValue(key="active_goal_source", value=self.active_goal_source))
        st.values.append(KeyValue(key="active_mission_key", value=self.active_mission_key))
        st.values.append(KeyValue(key="maneuver_phase_override", value=maneuver_override_source))
        active_goal_distance = self._active_goal_distance()
        st.values.append(
            KeyValue(
                key="active_goal_distance_m",
                value="none" if active_goal_distance is None else f"{active_goal_distance:.2f}",
            )
        )
        st.values.append(
            KeyValue(
                key="active_goal_reached_distance_m",
                value=f"{self._active_goal_reached_distance_m():.2f}",
            )
        )
        st.values.append(
            KeyValue(
                key="nav2_goal_succeeded",
                value=str(self._nav2_goal_succeeded).lower(),
            )
        )
        st.values.append(KeyValue(key="nav2_terminal_status", value=str(self._nav2_terminal_status)))
        st.values.append(KeyValue(key="return_requested", value=str(self.return_requested).lower()))
        st.values.append(KeyValue(key="recall_requested", value=str(self.recall_requested).lower()))
        st.values.append(KeyValue(key="return_mission_key", value=self.return_mission_key))
        st.values.append(
            KeyValue(
                key="active_goal_xy",
                value=(
                    "none"
                    if self.active_goal is None
                    else f"{self.active_goal.pose.position.x:.2f},{self.active_goal.pose.position.y:.2f}"
                ),
            )
        )
        diag.status.append(st)
        self.pub_diag.publish(diag)

    def _tick(self) -> None:
        if not self.enabled:
            return

        now = self.get_clock().now()
        level = self._state_level()
        estop = False

        if self.prev_state_level != self._warn_level and level == self._warn_level:
            self.warn_goal_sent = False
        if level != self._warn_level:
            self.warn_goal_sent = False

        if level >= self._error_level:
            self.state = "ERROR_STOP"
            estop = self.auto_estop_on_error
        elif self.recall_requested:
            self.state = "RECALLED"
            if self._publish_auto_goal(self.recall_target_key, self.active_goal_source):
                self.recall_requested = False
                self.warn_goal_sent = False
                self._set_scenario(self.SCENARIO_RECALL_TO_SITE, "recall_retry")
        elif self.return_requested:
            self.state = "RETURNING"
            if self._publish_auto_goal(self.return_mission_key, "return_request"):
                self.return_requested = False
                self.warn_goal_sent = False
                self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, "return_retry")
        else:
            if level != self._warn_level and self.auto_startup_goal and not self.startup_goal_sent:
                if self._publish_auto_goal(self.startup_mission_key, "startup"):
                    self.startup_goal_sent = True
                    if self.startup_mission_key == self.return_mission_key:
                        self._set_scenario(self.SCENARIO_WAIT_DROP_ZONE, "startup_goal")

            goal_reached = self._goal_reached(self._active_goal_reached_distance_m())
            if goal_reached:
                if not self._goal_reached_latched:
                    self._goal_reached_latched = True
                if self._goal_reached_since is None:
                    self._goal_reached_since = now
            else:
                self._goal_reached_latched = False
                self._goal_reached_since = None

            # HH_260528 Scenario 1/3 optional auto-return.
            if (
                goal_reached
                and self.enable_auto_return_on_site_goal
                and self.scenario_id in (self.SCENARIO_DELIVERY_TO_SITE, self.SCENARIO_RECALL_TO_SITE)
                and self._is_site_key(self.active_mission_key)
            ):
                dwell_elapsed_s = 0.0
                if self._goal_reached_since is not None:
                    dwell_elapsed_s = (now - self._goal_reached_since).nanoseconds / 1e9
                if dwell_elapsed_s >= self.goal_reached_dwell_s:
                    if self._publish_auto_goal(self.return_mission_key, "auto_return"):
                        self._set_scenario(self.SCENARIO_RETURN_TO_DROP_ZONE, "auto_return")
                        self.state = "RETURNING"

            # HH_260720 - Drop-zone arrival hands motion ownership to control and parking.
            drop_zone_arrival_announced = False
            if self.scenario_id == self.SCENARIO_RETURN_TO_DROP_ZONE and self._drop_zone_arrived_with_condition():
                if not self._drop_zone_arrival_notified:
                    # HH_260720 - Hold route arrival before drop-zone alignment
                    # and automatic reverse parking take ownership.
                    self.state = "GOAL_REACHED"
                    self._drop_zone_arrival_notified = True
                    self._drop_zone_arrival_notified_time = now
                    drop_zone_arrival_announced = True
                else:
                    hold_s = max(0.0, self.reverse_parking_controller_handoff_hold_s)
                    if self._drop_zone_arrival_notified_time is None:
                        elapsed_s = hold_s
                    else:
                        elapsed_s = (
                            now - self._drop_zone_arrival_notified_time
                        ).nanoseconds / 1e9
                    if elapsed_s < hold_s:
                        self.state = "GOAL_REACHED"
                        drop_zone_arrival_announced = True
                    else:
                        self._set_scenario(self.SCENARIO_WAIT_DROP_ZONE, "drop_zone_arrived")

            if drop_zone_arrival_announced:
                pass
            elif self.scenario_id == self.SCENARIO_WAIT_DROP_ZONE:
                self.state = "WAIT_DZ"
            elif goal_reached:
                self.state = "GOAL_REACHED"
            elif self.active_goal is not None:
                self.state = "RUNNING"
            else:
                self.state = "READY"

            goal_reached_handoff = self.state == "GOAL_REACHED" and self.scenario_id in {
                self.SCENARIO_DELIVERY_TO_SITE,
                self.SCENARIO_RECALL_TO_SITE,
                self.SCENARIO_RETURN_TO_DROP_ZONE,
            }
            if (
                level == self._warn_level
                and not (
                    self.allow_goal_reached_handoff_in_warn
                    and goal_reached_handoff
                )
            ):
                self.state = "WARN_RECOVERY"
                if self.auto_warn_recovery_goal and not self.warn_goal_sent:
                    if self._publish_auto_goal(self.warn_mission_key, "warn_recovery"):
                        self.warn_goal_sent = True

        self.prev_state_level = level
        self._publish_state_outputs(estop)


def main() -> None:
    rclpy.init()
    node = PlanningStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:  # noqa: BLE001
        node.get_logger().error(f"planning_state_machine runtime exception: {e}")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
