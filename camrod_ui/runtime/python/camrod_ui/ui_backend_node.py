#!/usr/bin/env python3
# HH_260421: UI backend simplified to direct destination-driven engage/goal dispatch.
# HH_260520: Migrated HTTP server to FastAPI+uvicorn with WebSocket support.
#            Added /battery_percentage and /service/state sub/pub.

from __future__ import annotations

import asyncio
import json
import math
import os
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import rclpy
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from avg_msgs.msg import (
    AvgServiceState,
    AvgBool,
    AvgLocalizationMode,
    AvgPlatformStatus,
    AvgPoseStamped,
    CampsiteOccupancy,
    ModuleState,
    MotionOperation,
    PlanningMissionKey,
    PlanningState,
    SystemStatus,
    UiDestinationCommand,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
# HH_260721 - Keep only the FastAPI symbols used by the runtime backend.
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

import uvicorn

from camrod_ui.api_common import to_diag_level_int
from camrod_ui.ui_state_policy import UiStatePolicy

# HH_260721 - Keep symbolic service names stable across ROS, REST, and WebSocket clients.
SERVICE_STATE_NAMES = {
    AvgServiceState.DROP_ZONE_WAIT: "DROP_ZONE_WAIT",
    AvgServiceState.MOVING_TO_SITE: "MOVING_TO_SITE",
    AvgServiceState.SITE_ARRIVED: "SITE_ARRIVED",
    AvgServiceState.RETURNING_TO_DROP_ZONE: "RETURNING_TO_DROP_ZONE",
    AvgServiceState.GUEST_RECALL_SERVICE: "GUEST_RECALL_SERVICE",
    AvgServiceState.SITE_ENTRY: "SITE_ENTRY",
    AvgServiceState.UNLOAD_WAIT: "UNLOAD_WAIT",
    AvgServiceState.RECALL_TO_SITE_ROAD: "RECALL_TO_SITE_ROAD",
    AvgServiceState.GUEST_LOADING_WAIT: "GUEST_LOADING_WAIT",
    AvgServiceState.RETURN_WITH_CARGO: "RETURN_WITH_CARGO",
    AvgServiceState.DROP_ZONE_PARKING: "DROP_ZONE_PARKING",
    # HH_260721 - Mirror explicit normal wait, charging, and departure states.
    AvgServiceState.WAITING_FOR_RETURN_REQUEST: "WAITING_FOR_RETURN_REQUEST",
    AvgServiceState.WAITING_FOR_CHARGING: "WAITING_FOR_CHARGING",
    AvgServiceState.CHARGING: "CHARGING",
    AvgServiceState.DEPARTING_CHARGER: "DEPARTING_CHARGER",
    AvgServiceState.DEPARTING_DROP_ZONE: "DEPARTING_DROP_ZONE",
    AvgServiceState.OPERATOR_STOPPED: "OPERATOR_STOPPED",
}


@dataclass
class ApiState:
    """In-memory snapshot exposed by the HTTP/WebSocket API."""

    engaged: bool = False
    ready: bool = False
    # 260708: Operator headlight toggle state (relay via light MCU bridge).
    headlight: bool = False
    operation_mode: str = "STOP"
    ready_message: str = ""
    module_states: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics_agg_count: int = 0
    diagnostics_agg_error_count: int = 0
    system_health: str = "STARTING"
    mission_phase: str = UiStatePolicy.INITIALIZING
    mission_source: str = "none"
    service_state: int = -1
    service_state_name: str = "PREPARING"
    service_state_description: str = "Waiting for service state"
    destination: Dict[str, Any] = field(
        default_factory=lambda: {"site": "", "run": False}
    )
    battery_percentage: int = -1
    ws_site_states: Dict[str, bool] = field(default_factory=dict)
    occupied_sites: List[str] = field(default_factory=list)


@dataclass
class MissionKeypoint:
    """Resolved keypoint for destination dispatch."""

    key: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw_deg: float
    # HH_260721 - Keep the maneuver policy attached to the operational campsite target.
    service_mode: str = "turnaround"
    corners: List[tuple[float, float]] = field(default_factory=list)


class UiBackendNode(Node):
    """FastAPI backend that bridges UI destination commands to planning topics."""

    def __init__(self) -> None:
        super().__init__("ui_backend")

        # HTTP server parameters.
        self.host = str(self.declare_parameter("host", "0.0.0.0").value)
        self.port = int(self.declare_parameter("port", 8000).value)
        self.enable_http_server = bool(self.declare_parameter("enable_http_server", True).value)
        self.frontend_dir = Path(str(self.declare_parameter("frontend_dir", "").value))

        # Topic and destination dispatch parameters.
        self.ui_destination_topic = str(
            self.declare_parameter("ui_destination_topic", "/ui/selected_destination").value
        )
        self.planning_engage_topic = str(
            self.declare_parameter("planning_engage_topic", "/planning/engage").value
        )
        self.planning_mission_engage_topic = str(
            self.declare_parameter("planning_mission_engage_topic", "/planning/mission_engage").value
        )
        self.platform_drive_enable_topic = str(
            self.declare_parameter("platform_drive_enable_topic", "/platform/drive_enable").value
        )
        # 260708: Headlight ON/OFF button target, consumed by light_controller.
        self.headlight_command_topic = str(
            self.declare_parameter("headlight_command_topic", "/platform/headlight/command").value
        )
        self.planning_mission_key_topic = str(
            self.declare_parameter("planning_mission_key_topic", "/planning/mission_key").value
        )
        # HH_260727 - Keep regulated site dispatch separate from RViz /goal_pose.
        self.planning_goal_pose_topic = str(
            self.declare_parameter(
                "planning_goal_pose_topic", "/planning/site_goal_pose_ros"
            ).value
        )
        self.platform_status_topic = str(
            self.declare_parameter("platform_status_topic", "/platform/status").value
        )
        self.ranger_base_node_name = str(
            self.declare_parameter("ranger_base_node_name", "/ranger_base_node").value
        ).rstrip("/")
        self.steering_transition_parameter = str(
            self.declare_parameter(
                "steering_transition_parameter",
                "steering_transition_rate_radps",
            ).value
        )
        self.service_state_topic = str(
            self.declare_parameter("service_state_topic", "/service/state").value
        )
        # HH_260803 - Robot and guest frontends submit one semantic UI request;
        # this backend remains the only owner of service-state and engage changes.
        self.ui_camping_site_operation_request_topic = str(
            self.declare_parameter(
                "ui_camping_site_operation_request_topic",
                "/ui/camping_site_operation_request",
            ).value
        )
        self.camping_site_maneuver_controller_operation_topic = str(
            # HH_260720 - UI commands the control-owned campsite maneuver directly.
            self.declare_parameter(
                "camping_site_maneuver_controller_operation_topic",
                "/control/camping_site_maneuver_controller/operation",
            ).value
        )
        self.camping_site_maneuver_controller_adopt_topic = str(
            self.declare_parameter("camping_site_maneuver_controller_adopt_topic", "/control/camping_site_maneuver_controller/adopt").value
        )
        # HH_260721 - Route site requests through the explicit drop-zone departure handoff.
        self.drop_zone_maneuver_controller_operation_topic = str(
            self.declare_parameter(
                "drop_zone_maneuver_controller_operation_topic",
                "/control/drop_zone_maneuver_controller/operation",
            ).value
        )
        # HH_260721 - Release the final parking controller before station departure starts.
        self.parking_operation_topic = str(
            self.declare_parameter("parking_operation_topic", "/parking/operation").value
        )
        self.drop_zone_exit_complete_topic = str(
            self.declare_parameter(
                "drop_zone_exit_complete_topic", "/control/drop_zone/exit_complete"
            ).value
        )
        self.nav2_cancel_action_topics = [
            str(topic)
            for topic in self.declare_parameter(
                "nav2_cancel_action_topics",
                [
                    "/planning/follow_path/_action/cancel_goal",
                    "/planning/navigate_to_pose/_action/cancel_goal",
                ],
            ).value
        ]
        self.arrival_pose_topic = str(
            self.declare_parameter("arrival_pose_topic", "/localization/pose").value
        )
        self.immediate_site_arrival_enabled = bool(
            self.declare_parameter("immediate_site_arrival_enabled", True).value
        )
        self.site_arrival_center_radius_m = abs(
            float(self.declare_parameter("site_arrival_center_radius_m", 2.5).value)
        )
        self.site_arrival_pose_timeout_s = float(
            self.declare_parameter("site_arrival_pose_timeout_s", 2.0).value
        )
        self.publish_mission_key = bool(self.declare_parameter("publish_mission_key", True).value)
        self.publish_goal_pose = bool(self.declare_parameter("publish_goal_pose", True).value)
        self.publish_engage_from_destination = bool(
            self.declare_parameter("publish_engage_from_destination", True).value
        )
        self.publish_mission_engage_from_destination = bool(
            self.declare_parameter("publish_mission_engage_from_destination", False).value
        )
        # HH_260724 - New campsite departures are admitted only with enough battery margin.
        self.require_battery_for_mission_dispatch = bool(
            self.declare_parameter("require_battery_for_mission_dispatch", True).value
        )
        self.minimum_mission_dispatch_battery_percent = max(
            0,
            min(
                100,
                int(
                    round(
                        float(
                            self.declare_parameter(
                                "minimum_mission_dispatch_battery_percent", 35.0
                            ).value
                        )
                    )
                ),
            ),
        )
        self.low_battery_return_after_current_mission = bool(
            self.declare_parameter(
                "low_battery_return_after_current_mission", True
            ).value
        )
        self.low_battery_return_threshold_percent = max(
            0,
            min(
                100,
                int(
                    round(
                        float(
                            self.declare_parameter(
                                "low_battery_return_threshold_percent", 35.0
                            ).value
                        )
                    )
                ),
            ),
        )
        self.publish_platform_drive_enable_with_engage = bool(
            self.declare_parameter("publish_platform_drive_enable_with_engage", True).value
        )
        self.default_goal_frame_id = str(self.declare_parameter("default_goal_frame_id", "map").value)
        self.fallback_mission_key = str(self.declare_parameter("fallback_mission_key", "camping_site_1").value)
        self.fallback_to_first_known_goal = bool(
            self.declare_parameter("fallback_to_first_known_goal", True).value
        )

        self.camping_sites_yaml = str(self.declare_parameter("camping_sites_yaml", "").value)
        self.campsite_occupancy_topic = str(
            self.declare_parameter(
                "campsite_occupancy_topic", "/perception/camping_sites/occupancy"
            ).value
        )
        self.site_names = [
            str(site)
            for site in self.declare_parameter("site_names", [f"B{i}" for i in range(1, 14)]).value
        ]
        if not self.site_names:
            self.site_names = [f"B{i}" for i in range(1, 14)]

        raw_site_to_mission = self.declare_parameter("site_to_mission_key_map", []).value
        self.site_to_mission_key_map = self._parse_site_mission_map(raw_site_to_mission)

        self.diagnostics_agg_topic = str(
            self.declare_parameter("diagnostics_agg_topic", "/system/diagnostics_agg").value
        )
        self.system_status_topic = str(
            self.declare_parameter("system_status_topic", "/system/status").value
        )
        self.planning_state_topic = str(
            self.declare_parameter(
                "planning_state_topic", "/planning/state_machine/state"
            ).value
        )
        self.planning_route_goal_topic = str(
            self.declare_parameter(
                "planning_route_goal_topic", "/planning/goal_pose_snapped_ros"
            ).value
        )
        self.planning_goal_source_topic = str(
            self.declare_parameter(
                "planning_goal_source_topic", "/planning/goal_source"
            ).value
        )
        self.planning_nav_status_topic = str(
            self.declare_parameter(
                "planning_nav_status_topic",
                "/planning/navigate_to_pose/_action/status",
            ).value
        )
        self.planning_engaged_state_topic = str(
            self.declare_parameter(
                "planning_engaged_state_topic", "/control/planning_engaged"
            ).value
        )
        self.localization_mode_topic = str(
            self.declare_parameter(
                "localization_mode_topic", "/localization/mode"
            ).value
        )
        self.control_gate_status_topic = str(
            self.declare_parameter(
                "control_gate_status_topic",
                "/control/cmd_vel_safety_gate/status",
            ).value
        )
        self.navigate_to_pose_action = str(
            self.declare_parameter(
                "navigate_to_pose_action", "/planning/navigate_to_pose"
            ).value
        )
        self.readiness_required_modules = [
            str(name)
            for name in self.declare_parameter(
                "readiness_required_modules",
                [
                    "map",
                    "sensing",
                    "localization",
                    "planning",
                    "control",
                    "platform",
                    "system",
                ],
            ).value
        ]
        self.readiness_map_frame = str(
            self.declare_parameter("readiness_map_frame", "map").value
        )
        # HH_260803 - UI readiness follows the axle-midpoint navigation frame.
        self.readiness_base_frame = str(
            self.declare_parameter(
                "readiness_base_frame", "robot_center_link"
            ).value
        )
        self.readiness_check_period_s = max(
            0.1,
            float(
                self.declare_parameter(
                    "readiness_check_period_s", 0.5
                ).value
            ),
        )
        self.max_ready_localization_mode = int(
            self.declare_parameter("max_ready_localization_mode", 0).value
        )
        self._keypoints_by_mission_key = self._load_camping_site_keypoints(self.camping_sites_yaml)
        self._lock = threading.Lock()
        self._runtime_policy = UiStatePolicy(
            self.readiness_required_modules,
            max_ready_localization_mode=self.max_ready_localization_mode,
        )
        self._state = ApiState(
            ws_site_states={s: False for s in self.site_names},
        )
        self._latest_arrival_pose: Optional[AvgPoseStamped] = None
        self._latest_arrival_pose_time_s = 0.0
        # HH_260721 - Keep only the latest requested site while drop-zone exit owns motion.
        self._latest_platform_is_charging = False
        self._latest_service_state: Optional[int] = None
        self._pending_site_after_drop_zone_exit: Optional[tuple[str, str, str]] = None
        self._drop_zone_exit_active = False
        # HJ_260804 - Destination state may be cleared by a departure ack
        # before campsite arrival. Preserve the active mission site so the
        # Robot and Guest UIs still receive the matching arrival notification.
        self._active_mission_site: str = ""
        self._low_battery_return_pending = False
        self._low_battery_return_started = False
        self._low_battery_return_wait_notified = False
        # HH_260706 - HTTP site selection dispatches immediately; ignore the
        # local topic echo so the same camping-site command is not applied twice.
        self._last_direct_destination_echo: Optional[tuple[str, bool, str, float]] = None

        # WebSocket client management.
        self._ws_clients: Set[WebSocket] = set()
        self._ws_clients_lock = threading.Lock()
        self._main_loop: Optional[asyncio.AbstractEventLoop] = None
        self._uvicorn_server: Optional[uvicorn.Server] = None
        self._server_thread: Optional[threading.Thread] = None
        self._server_stop_requested = threading.Event()

        # Subscriptions.
        self.sub_destination = self.create_subscription(
            UiDestinationCommand,
            self.ui_destination_topic,
            self._on_destination_command,
            10,
        )
        self.sub_ui_camping_site_operation_request = self.create_subscription(
            MotionOperation,
            self.ui_camping_site_operation_request_topic,
            self._on_ui_camping_site_operation_request,
            10,
        )
        self.sub_diagnostics_agg = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_agg_topic,
            self._on_diagnostics_agg,
            10,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub_system_status = self.create_subscription(
            SystemStatus,
            self.system_status_topic,
            self._on_system_status,
            10,
        )
        self.sub_planning_state = self.create_subscription(
            PlanningState,
            self.planning_state_topic,
            self._on_planning_state,
            10,
        )
        self.sub_planning_route_goal = self.create_subscription(
            PoseStamped,
            self.planning_route_goal_topic,
            self._on_planning_route_goal,
            10,
        )
        self.sub_planning_goal_source = self.create_subscription(
            String,
            self.planning_goal_source_topic,
            self._on_planning_goal_source,
            state_qos,
        )
        self.sub_planning_nav_status = self.create_subscription(
            GoalStatusArray,
            self.planning_nav_status_topic,
            self._on_planning_nav_status,
            10,
        )
        self.sub_planning_engaged_state = self.create_subscription(
            AvgBool,
            self.planning_engaged_state_topic,
            self._on_planning_engaged_state,
            state_qos,
        )
        self.sub_localization_mode = self.create_subscription(
            AvgLocalizationMode,
            self.localization_mode_topic,
            self._on_localization_mode,
            10,
        )
        self.sub_control_gate_status = self.create_subscription(
            ModuleState,
            self.control_gate_status_topic,
            self._on_control_gate_status,
            state_qos,
        )
        self.sub_platform_status = self.create_subscription(
            AvgPlatformStatus,
            self.platform_status_topic,
            self._on_platform_status,
            10,
        )
        self.sub_service_state = self.create_subscription(
            AvgServiceState,
            self.service_state_topic,
            self._on_service_state,
            10,
        )
        self.sub_arrival_pose = self.create_subscription(
            AvgPoseStamped,
            self.arrival_pose_topic,
            self._on_arrival_pose,
            10,
        )
        self.sub_drop_zone_exit_complete = self.create_subscription(
            AvgBool,
            self.drop_zone_exit_complete_topic,
            self._on_drop_zone_exit_complete,
            10,
        )
        self.sub_campsite_occupancy = self.create_subscription(
            CampsiteOccupancy,
            self.campsite_occupancy_topic,
            self._on_campsite_occupancy,
            10,
        )

        # Publishers.
        # HH_260617: UI destination and planning mission-key topics now use
        # generated avg_msgs semantic messages instead of JSON/String wrappers.
        self.pub_destination = self.create_publisher(
            UiDestinationCommand, self.ui_destination_topic, 10
        )
        self.pub_engage = self.create_publisher(AvgBool, self.planning_engage_topic, 10)
        self.pub_mission_engage = self.create_publisher(
            AvgBool, self.planning_mission_engage_topic, 10
        )
        self.pub_platform_drive_enable = self.create_publisher(
            AvgBool, self.platform_drive_enable_topic, 10
        )
        # 260708: Headlight button publisher (light_controller passes it to the MCU).
        self.pub_headlight = self.create_publisher(AvgBool, self.headlight_command_topic, 10)
        self.pub_camping_site_maneuver_controller_operation = self.create_publisher(
            MotionOperation, self.camping_site_maneuver_controller_operation_topic, 10
        )
        self.pub_camping_site_maneuver_controller_adopt = self.create_publisher(
            UiDestinationCommand, self.camping_site_maneuver_controller_adopt_topic, 10
        )
        self.pub_drop_zone_maneuver_controller_operation = self.create_publisher(
            MotionOperation, self.drop_zone_maneuver_controller_operation_topic, 10
        )
        self.pub_parking_operation = self.create_publisher(
            MotionOperation, self.parking_operation_topic, 10
        )
        self.pub_mission_key = self.create_publisher(
            PlanningMissionKey, self.planning_mission_key_topic, 10
        )
        self.pub_goal_pose = self.create_publisher(PoseStamped, self.planning_goal_pose_topic, 10)
        self.pub_service_state = self.create_publisher(AvgServiceState, self.service_state_topic, 10)
        # HH_260727 - Runtime tuning uses the standard ROS parameter services, so the UI
        # changes the driver immediately without restarting the platform.
        self.get_ranger_parameters_client = self.create_client(
            GetParameters, f"{self.ranger_base_node_name}/get_parameters"
        )
        self.set_ranger_parameters_client = self.create_client(
            SetParameters, f"{self.ranger_base_node_name}/set_parameters"
        )
        # HH_260724 - UI cancel/stop must cancel the active Nav2 actions, not only close engage.
        self.nav2_cancel_clients = [
            self.create_client(CancelGoal, topic) for topic in self.nav2_cancel_action_topics
        ]
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._navigate_action_client = ActionClient(
            self, NavigateToPose, self.navigate_to_pose_action
        )
        self._readiness_timer = self.create_timer(
            self.readiness_check_period_s, self._on_readiness_timer
        )
        if self.enable_http_server:
            self._start_fastapi_server()

        self.get_logger().info(
            "ui_backend ready: "
            f"host={self.host} port={self.port} "
            f"frontend_dir={str(self.frontend_dir) if self.frontend_dir else '(builtin)'} "
            f"destination_topic={self.ui_destination_topic} "
            f"engage_topic={self.planning_engage_topic} "
            f"mission_engage_topic={self.planning_mission_engage_topic} "
            f"platform_drive_enable_topic={self.platform_drive_enable_topic} "
            f"camping_site_maneuver_controller_operation_topic={self.camping_site_maneuver_controller_operation_topic} "
            f"camping_site_maneuver_controller_adopt_topic={self.camping_site_maneuver_controller_adopt_topic} "
            f"drop_zone_operation_topic={self.drop_zone_maneuver_controller_operation_topic} "
            f"parking_operation_topic={self.parking_operation_topic} "
            f"drop_zone_exit_complete_topic={self.drop_zone_exit_complete_topic} "
            f"mission_key_topic={self.planning_mission_key_topic} "
            f"goal_pose_topic={self.planning_goal_pose_topic} "
            f"arrival_pose_topic={self.arrival_pose_topic} "
            f"campsite_occupancy_topic={self.campsite_occupancy_topic} "
            f"ranger_base_node={self.ranger_base_node_name} "
            f"camping_sites_yaml={self.camping_sites_yaml if self.camping_sites_yaml else '(none)'}"
        )

    def _parse_site_mission_map(self, raw_value: object) -> Dict[str, str]:
        parsed: Dict[str, str] = {}
        if not isinstance(raw_value, list):
            return parsed
        for item in raw_value:
            entry = str(item).strip()
            if not entry or ":" not in entry:
                continue
            site, mission_key = entry.split(":", maxsplit=1)
            site = site.strip()
            mission_key = mission_key.strip()
            if site and mission_key:
                parsed[site] = mission_key
        return parsed

    def _load_camping_site_keypoints(self, yaml_path: str) -> Dict[str, MissionKeypoint]:
        keypoints: Dict[str, MissionKeypoint] = {}
        path = Path(yaml_path).expanduser() if yaml_path else None
        if path is None or not str(path):
            self.get_logger().warn("camping_sites_yaml is empty: goal_pose dispatch will use mission-key only")
            return keypoints
        if not path.exists():
            self.get_logger().warn(
                f"camping_sites_yaml not found: {str(path)} (goal_pose dispatch may be unavailable)"
            )
            return keypoints

        try:
            with path.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to read camping_sites_yaml {str(path)}: {exc}")
            return keypoints

        raw_sites = data.get("camping_sites", [])
        if not isinstance(raw_sites, list):
            self.get_logger().warn("camping_sites_yaml has no list key 'camping_sites'")
            return keypoints

        for idx, site in enumerate(raw_sites, start=1):
            if not isinstance(site, dict):
                continue
            key = str(site.get("type", "")).strip() or f"camping_site_{idx}"
            if key in keypoints:
                continue
            corners = []
            raw_corners = site.get("corners", [])
            if isinstance(raw_corners, list):
                for corner in raw_corners:
                    if not isinstance(corner, dict):
                        continue
                    try:
                        corners.append((float(corner["x"]), float(corner["y"])))
                    except (KeyError, TypeError, ValueError):
                        continue
            # HH_260721 - Dispatch an alternate roadside pose when the map marks a site inaccessible.
            service_mode = str(site.get("service_mode", "turnaround")).strip().lower()
            service_mode = service_mode or "turnaround"
            keypoints[key] = MissionKeypoint(
                key=key,
                frame_id=str(site.get("frame_id", self.default_goal_frame_id)).strip()
                or self.default_goal_frame_id,
                x=float(site.get("service_x", site.get("x", 0.0))),
                y=float(site.get("service_y", site.get("y", 0.0))),
                z=float(site.get("service_z", site.get("z", 0.0))),
                yaw_deg=float(site.get("service_yaw_deg", site.get("yaw_deg", 0.0))),
                service_mode=service_mode,
                corners=corners,
            )

        self.get_logger().info(
            f"loaded {len(keypoints)} camping-site keypoints from {str(path)}"
        )
        return keypoints

    def destroy_node(self) -> bool:
        # HH_260805 - Stop the HTTP event loop before ROS destroys callbacks and
        # publishers that in-flight FastAPI/WebSocket handlers may still access.
        self._stop_fastapi_server()
        return super().destroy_node()

    def _stop_fastapi_server(self, join_timeout_s: float = 3.0) -> None:
        self._server_stop_requested.set()
        server = self._uvicorn_server
        if server is not None:
            server.should_exit = True

        loop = self._main_loop
        if loop is not None and loop.is_running():
            try:
                loop.call_soon_threadsafe(lambda: None)
            except RuntimeError:
                pass

        thread = self._server_thread
        if (
            thread is not None
            and thread is not threading.current_thread()
            and thread.is_alive()
        ):
            thread.join(timeout=max(0.0, float(join_timeout_s)))
            if thread.is_alive():
                self.get_logger().warn(
                    "ui backend fastapi thread did not stop before timeout"
                )

    def _compute_operation_mode(self, engaged: bool, ready: bool) -> str:
        if engaged and ready:
            return "AUTO"
        if engaged and not ready:
            return "WAITING_FOR_READY"
        return "STOP"

    def _extract_module_name(self, status: DiagnosticStatus) -> str:
        for kv in status.values:
            if kv.key == "category" and kv.value:
                return str(kv.value)
        if status.hardware_id:
            return str(status.hardware_id)
        if status.name:
            parts = [p for p in status.name.strip("/").split("/") if p]
            if parts:
                return parts[0]
        return "unknown"

    def _yaw_deg_to_quaternion(self, yaw_deg: float) -> tuple[float, float, float, float]:
        yaw_rad = math.radians(float(yaw_deg))
        half = yaw_rad * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _point_in_polygon(x: float, y: float, polygon: List[tuple[float, float]]) -> bool:
        inside = False
        count = len(polygon)
        if count < 3:
            return False
        j = count - 1
        for i in range(count):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            crosses = (yi > y) != (yj > y)
            if crosses:
                denom = yj - yi
                if abs(denom) < 1.0e-9:
                    j = i
                    continue
                x_at_y = (xj - xi) * (y - yi) / denom + xi
                if x < x_at_y:
                    inside = not inside
            j = i
        return inside

    def _site_arrival_match(self, site: str) -> tuple[bool, str, float, str]:
        mission_key = self._resolve_mission_key_for_site(site) or ""
        keypoint = self._keypoints_by_mission_key.get(mission_key)
        if not self.immediate_site_arrival_enabled:
            return False, mission_key, float("inf"), "disabled"
        if keypoint is None:
            return False, mission_key, float("inf"), "missing_keypoint"
        if self._latest_arrival_pose is None:
            return False, mission_key, float("inf"), "missing_pose"
        age_s = self._now_s() - self._latest_arrival_pose_time_s
        if self.site_arrival_pose_timeout_s > 0.0 and age_s > self.site_arrival_pose_timeout_s:
            return False, mission_key, float("inf"), f"stale_pose:{age_s:.2f}s"

        pose_frame = str(self._latest_arrival_pose.header.frame_id or self.default_goal_frame_id)
        goal_frame = str(keypoint.frame_id or self.default_goal_frame_id)
        if pose_frame and goal_frame and pose_frame != goal_frame:
            return False, mission_key, float("inf"), f"frame_mismatch:{pose_frame}!={goal_frame}"

        px = float(self._latest_arrival_pose.pose.position.x)
        py = float(self._latest_arrival_pose.pose.position.y)
        center_distance = math.hypot(px - keypoint.x, py - keypoint.y)
        if keypoint.corners and self._point_in_polygon(px, py, keypoint.corners):
            return True, mission_key, center_distance, "inside_site_polygon"
        if center_distance <= self.site_arrival_center_radius_m:
            return True, mission_key, center_distance, "near_site_center"
        return False, mission_key, center_distance, "outside_site"

    def _mission_dispatch_battery_block(self, site: str) -> Optional[Dict[str, Any]]:
        if not self.require_battery_for_mission_dispatch:
            return None
        already_arrived, _, _, _ = self._site_arrival_match(site)
        if already_arrived:
            return None
        with self._lock:
            battery_percentage = int(self._state.battery_percentage)
        if battery_percentage < 0:
            message = (
                "battery state unavailable; campsite dispatch requires "
                f">={self.minimum_mission_dispatch_battery_percent}%"
            )
        elif battery_percentage >= self.minimum_mission_dispatch_battery_percent:
            return None
        else:
            message = (
                f"battery {battery_percentage}% is below mission minimum "
                f"{self.minimum_mission_dispatch_battery_percent}%"
            )
        return {
            "error": "battery_below_mission_minimum",
            "site": site,
            "battery_percentage": battery_percentage,
            "minimum_battery_percentage": self.minimum_mission_dispatch_battery_percent,
            "message": message,
        }

    def _low_battery_mission_state(self, state: Optional[int]) -> bool:
        if state is None:
            return False
        return int(state) in {
            int(AvgServiceState.DEPARTING_CHARGER),
            int(AvgServiceState.DEPARTING_DROP_ZONE),
            int(AvgServiceState.MOVING_TO_SITE),
            int(AvgServiceState.SITE_ARRIVED),
            int(AvgServiceState.SITE_ENTRY),
            int(AvgServiceState.UNLOAD_WAIT),
            int(AvgServiceState.RECALL_TO_SITE_ROAD),
            int(AvgServiceState.GUEST_LOADING_WAIT),
            int(AvgServiceState.WAITING_FOR_RETURN_REQUEST),
        }

    def _low_battery_station_state(self, state: Optional[int]) -> bool:
        if state is None:
            return False
        return int(state) in {
            int(AvgServiceState.DROP_ZONE_WAIT),
            int(AvgServiceState.WAITING_FOR_CHARGING),
            int(AvgServiceState.CHARGING),
        }

    def _low_battery_return_payload(
        self,
        battery_percentage: int,
        *,
        started: bool = False,
        waiting_for_user: bool = False,
        pending: bool = True,
    ) -> Dict[str, Any]:
        message = (
            "battery below mission return threshold; finish current campsite "
            "mission, wait for user return request, then return to drop zone"
        )
        if started:
            message = "battery low; user return request accepted; returning to drop zone"
        elif waiting_for_user:
            message = "battery low; waiting for user return request before drop-zone return"
        return {
            "battery_return_pending": bool(pending),
            "battery_return_started": bool(started),
            "battery_return_waiting_for_user": bool(waiting_for_user),
            "battery_percentage": int(battery_percentage),
            "minimum_battery_percentage": self.low_battery_return_threshold_percent,
            "message": message,
        }

    def _clear_low_battery_return_if_stationary(self, state: Optional[int]) -> None:
        if not self._low_battery_station_state(state):
            return
        if (
            not self._low_battery_return_pending
            and not self._low_battery_return_started
            and not self._low_battery_return_wait_notified
        ):
            return
        self._low_battery_return_pending = False
        self._low_battery_return_started = False
        self._low_battery_return_wait_notified = False
        self._schedule_broadcast(
            {
                "battery_return_pending": False,
                "battery_return_started": False,
                "battery_return_waiting_for_user": False,
            }
        )

    def _maybe_notify_low_battery_waiting_for_user(self) -> None:
        if (
            not self._low_battery_return_pending
            or self._low_battery_return_started
            or self._low_battery_return_wait_notified
        ):
            return
        if self._latest_service_state != int(AvgServiceState.WAITING_FOR_RETURN_REQUEST):
            return
        self._low_battery_return_wait_notified = True
        with self._lock:
            battery_percentage = int(self._state.battery_percentage)
        self._schedule_broadcast(
            self._low_battery_return_payload(
                battery_percentage,
                waiting_for_user=True,
                pending=True,
            )
        )

    def _mark_low_battery_return_started_if_needed(self) -> None:
        if not self._low_battery_return_pending or self._low_battery_return_started:
            return
        self._low_battery_return_started = True
        with self._lock:
            battery_percentage = int(self._state.battery_percentage)
        self._schedule_broadcast(
            self._low_battery_return_payload(
                battery_percentage,
                started=True,
                pending=True,
            )
        )

    def _update_low_battery_return_policy(self, battery_percentage: int, source: str) -> None:
        if not self.low_battery_return_after_current_mission:
            return
        self._clear_low_battery_return_if_stationary(self._latest_service_state)
        if battery_percentage < 0:
            return
        if battery_percentage >= self.low_battery_return_threshold_percent:
            return
        if not self._low_battery_mission_state(self._latest_service_state):
            return
        if not self._low_battery_return_pending:
            self._low_battery_return_pending = True
            self._low_battery_return_started = False
            self._low_battery_return_wait_notified = False
            self.get_logger().warn(
                "low battery return pending: "
                f"battery={battery_percentage}% "
                f"threshold={self.low_battery_return_threshold_percent}%"
            )
            self._schedule_broadcast(
                self._low_battery_return_payload(
                    battery_percentage,
                    started=False,
                    pending=True,
                )
            )
        self._maybe_notify_low_battery_waiting_for_user()

    def _notify_site_arrival(
        self,
        site: str,
        state: int,
        source: str,
        *,
        already_at_site: bool = False,
    ) -> None:
        payload = {"arrived": site, "site": site, "service_state": int(state)}
        if already_at_site:
            payload["already_at_site"] = True
        self._schedule_broadcast(payload)
        self.get_logger().info(
            f"site arrival notify ({source}): site={site} state={int(state)} "
            f"already_at_site={str(already_at_site).lower()}"
        )

    # ── WebSocket broadcast helpers ──────────────────────────────────────────

    def _schedule_broadcast(self, payload: dict) -> None:
        """Schedule a broadcast from a ROS2 callback thread into the asyncio loop."""
        if self._main_loop is None:
            return
        with self._ws_clients_lock:
            if not self._ws_clients:
                return
        asyncio.run_coroutine_threadsafe(
            self._broadcast(payload), self._main_loop
        )

    async def _broadcast(self, payload: dict) -> None:
        with self._ws_clients_lock:
            clients = list(self._ws_clients)
        for client in clients:
            try:
                await client.send_json(payload)
            except Exception:
                with self._ws_clients_lock:
                    self._ws_clients.discard(client)

    # ── ROS2 subscription callbacks ─────────────────────────────────────────

    def _update_runtime_state(
        self,
        update: Any,
        *,
        force_broadcast: bool = False,
    ) -> None:
        with self._lock:
            previous = (
                self._state.ready,
                self._state.ready_message,
                self._state.engaged,
                self._state.operation_mode,
                self._state.mission_phase,
                self._state.mission_source,
            )
            update()
            reasons = self._runtime_policy.readiness_reasons(
                require_idle=False
            )
            self._state.ready = self._runtime_policy.ready
            self._state.ready_message = (
                "ready" if not reasons else ", ".join(reasons)
            )
            self._state.engaged = self._runtime_policy.engaged
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
            )
            self._state.mission_phase = self._runtime_policy.mission_phase
            self._state.mission_source = self._runtime_policy.mission_source
            current = (
                self._state.ready,
                self._state.ready_message,
                self._state.engaged,
                self._state.operation_mode,
                self._state.mission_phase,
                self._state.mission_source,
            )
        if force_broadcast or current != previous:
            self._schedule_broadcast(
                {
                    "ready": current[0],
                    "ready_message": current[1],
                    "engaged": current[2],
                    "operation_mode": current[3],
                    "mission_phase": current[4],
                    "mission_source": current[5],
                }
            )

    def _on_readiness_timer(self) -> None:
        try:
            tf_ready = self._tf_buffer.can_transform(
                self.readiness_map_frame,
                self.readiness_base_frame,
                Time(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"UI readiness TF check failed: {exc}")
            tf_ready = False

        def update() -> None:
            self._runtime_policy.update_tf(bool(tf_ready))
            self._runtime_policy.update_action_server(
                self._navigate_action_client.server_is_ready()
            )

        # HH_260810 - Send an authoritative readiness heartbeat.  The first
        # WebSocket snapshot can otherwise race the ROS readiness transition
        # and leave a stale INITIALIZING frame visible until a goal event.
        self._update_runtime_state(update, force_broadcast=True)

    def _on_system_status(self, msg: SystemStatus) -> None:
        modules = {
            str(module.module_name): (
                int(module.level),
                str(module.operating_state),
            )
            for module in msg.modules
        }
        self._update_runtime_state(
            lambda: self._runtime_policy.update_system(modules)
        )

    def _on_planning_state(self, msg: PlanningState) -> None:
        self._update_runtime_state(
            lambda: self._runtime_policy.update_planning(
                state=msg.label,
                scenario=msg.scenario_label,
                active_mission_key=msg.active_mission_key,
                active_goal_source=msg.active_goal_source,
            )
        )

    def _on_planning_route_goal(self, _msg: PoseStamped) -> None:
        self._update_runtime_state(
            self._runtime_policy.update_goal_received
        )

    def _on_planning_goal_source(self, msg: String) -> None:
        self._update_runtime_state(
            lambda: self._runtime_policy.update_goal_source(msg.data)
        )

    def _on_planning_nav_status(self, msg: GoalStatusArray) -> None:
        statuses = [int(status.status) for status in msg.status_list]
        nav_status = GoalStatus.STATUS_UNKNOWN
        for candidate in (
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_ACCEPTED,
        ):
            if candidate in statuses:
                nav_status = candidate
                break
        else:
            if statuses:
                nav_status = statuses[-1]
        self._update_runtime_state(
            lambda: self._runtime_policy.update_nav_status(nav_status)
        )

    def _on_planning_engaged_state(self, msg: AvgBool) -> None:
        self._update_runtime_state(
            lambda: self._runtime_policy.update_engaged(bool(msg.data))
        )

    def _on_localization_mode(self, msg: AvgLocalizationMode) -> None:
        self._update_runtime_state(
            lambda: self._runtime_policy.update_localization(int(msg.value))
        )

    def _on_control_gate_status(self, msg: ModuleState) -> None:
        self._update_runtime_state(
            lambda: self._runtime_policy.update_gate(
                level=int(msg.level),
                operating_state=msg.operating_state,
                message=msg.message,
            )
        )

    def _on_diagnostics_agg(self, msg: DiagnosticArray) -> None:
        diagnostics: List[Dict[str, Any]] = []
        module_levels: Dict[str, int] = {}
        module_messages: Dict[str, str] = {}

        error_count = 0
        warning_count = 0
        error_level = to_diag_level_int(DiagnosticStatus.ERROR)
        warning_level = to_diag_level_int(DiagnosticStatus.WARN)
        stale_level = to_diag_level_int(DiagnosticStatus.STALE)

        for status in msg.status:
            raw_level = to_diag_level_int(status.level)
            # HH_260721 - A diagnostic stream with no fresh data is an operator-visible error.
            level = error_level if raw_level == stale_level else raw_level
            message = (
                f"diagnostic update stale: {status.message}"
                if raw_level == stale_level else status.message
            )
            module = self._extract_module_name(status)
            prev = module_levels.get(module, -1)
            if level >= prev:
                module_levels[module] = level
                module_messages[module] = message

            if level == error_level:
                error_count += 1
            elif level == warning_level:
                warning_count += 1

            diagnostics.append(
                {
                    "name": status.name,
                    "module": module,
                    "level": level,
                    "message": message,
                    "values": [{"key": kv.key, "value": kv.value} for kv in status.values],
                }
            )

        module_states = [
            {
                "name": module,
                "level": module_levels[module],
                "message": module_messages.get(module, ""),
            }
            for module in sorted(module_levels.keys())
        ]

        system_health = (
            "ERROR" if error_count > 0 else
            "WARNING" if warning_count > 0 else
            "OK"
        )

        with self._lock:
            health_changed = self._state.system_health != system_health
            self._state.diagnostics = diagnostics
            self._state.module_states = module_states
            self._state.diagnostics_agg_count = len(msg.status)
            self._state.diagnostics_agg_error_count = error_count
            self._state.system_health = system_health
        if health_changed:
            # HH_260721 - Broadcast health transitions without mixing them with service progress.
            self._schedule_broadcast({"system_health": system_health})

    def _on_platform_status(self, msg: AvgPlatformStatus) -> None:
        # HH_260720 - UI battery state comes from the canonical generated platform status.
        # HH_260721 - Charging state also decides whether a campsite goal must wait for departure.
        self._update_runtime_state(
            lambda: self._runtime_policy.update_platform(
                estop=bool(msg.estop),
                error_code=int(msg.error_code),
            )
        )
        charging = bool(msg.is_charging)
        charging_changed = charging != self._latest_platform_is_charging
        self._latest_platform_is_charging = charging
        if charging_changed and charging:
            departure_active = self._drop_zone_exit_active or self._latest_service_state in {
                int(AvgServiceState.DEPARTING_CHARGER),
                int(AvgServiceState.DEPARTING_DROP_ZONE),
            }
            if departure_active:
                # HH_260807 - Charger contact remains true until the platform
                # physically moves clear. Preserve the authorized departure
                # state instead of closing drive-enable on that expected edge.
                self.get_logger().info(
                    "charging contact still active during drop-zone departure; "
                    "preserving departure authorization"
                )
            else:
                # HH_260721 - Display confirmed CAN charging independently from parking completion.
                self._publish_service_state(
                    AvgServiceState.CHARGING,
                    source="platform_status:charging_started",
                )
        elif (
            charging_changed
            and not charging
            and self._latest_service_state == int(AvgServiceState.CHARGING)
        ):
            # HH_260721 - Return to uncharged standby only when no departure state replaced charging.
            self._publish_service_state(
                AvgServiceState.DROP_ZONE_WAIT,
                source="platform_status:charging_stopped",
            )
        if not msg.battery_state_available:
            return
        battery_fraction = float(msg.battery_percentage)
        if not math.isfinite(battery_fraction):
            return
        pct = max(0, min(100, int(math.floor(battery_fraction * 100.0))))
        with self._lock:
            battery_changed = self._state.battery_percentage != pct
            self._state.battery_percentage = pct
        if battery_changed:
            self._schedule_broadcast({"battery": pct})
        self._update_low_battery_return_policy(pct, source="platform_status")

    def _on_arrival_pose(self, msg: AvgPoseStamped) -> None:
        self._latest_arrival_pose = msg
        self._latest_arrival_pose_time_s = self._now_s()

    # HH_260721 - Release the pending Nav2 site goal only after vertical exit and yaw alignment.
    def _on_drop_zone_exit_complete(self, msg: AvgBool) -> None:
        if not self._drop_zone_exit_active:
            return
        pending = self._pending_site_after_drop_zone_exit
        self._drop_zone_exit_active = False
        self._pending_site_after_drop_zone_exit = None
        if pending is None:
            return
        site, mission_key, source = pending
        if not bool(msg.data):
            self.get_logger().error(
                f"drop-zone departure failed; campsite goal cancelled: site={site}"
            )
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source="drop_zone_exit_failed")
            self._schedule_broadcast(
                {"departure_failed": True, "site": site, "message": "drop-zone exit failed"}
            )
            # HH_260721 - Do not leave the UI in a departure state after a failed exit.
            self._publish_service_state(
                AvgServiceState.CHARGING
                if self._latest_platform_is_charging
                else AvgServiceState.DROP_ZONE_WAIT,
                source="drop_zone_exit_failed",
            )
            return
        # HH_260727 - The state-machine mission-key latch expires while the
        # station-exit maneuver runs, so refresh it immediately before the goal.
        release_source = f"{source}:drop_zone_exit_complete"
        self._publish_site_mission_key(mission_key, release_source)
        pose_published = self._publish_site_goal_pose(site, mission_key, source)
        if pose_published:
            self._publish_service_state(
                AvgServiceState.MOVING_TO_SITE,
                source=f"{source}:drop_zone_exit_complete",
            )
        self.get_logger().info(
            "drop-zone departure complete; released campsite goal: "
            f"site={site} mission_key={mission_key}"
        )

    def _on_service_state(self, msg: AvgServiceState) -> None:
        state = int(msg.state)
        # HH_260721 - DROP_ZONE_WAIT is the semantic parked state used by departure sequencing.
        self._latest_service_state = state
        state_name = str(msg.state_name).strip() or SERVICE_STATE_NAMES.get(
            state, f"UNKNOWN_{state}"
        )
        description = str(msg.description).strip() or state_name
        with self._lock:
            self._state.service_state = state
            self._state.service_state_name = state_name
            self._state.service_state_description = description
        # HH_260721 - Every client receives explicit operational state, not a health warning surrogate.
        self._schedule_broadcast({
            "service_state": state,
            "service_state_name": state_name,
            "service_state_description": description,
        })
        self.get_logger().info(
            f"Service state received: {state_name}({state}) ({description})"
        )
        arrival_states = {
            int(AvgServiceState.SITE_ARRIVED),
            int(AvgServiceState.UNLOAD_WAIT),
            int(AvgServiceState.GUEST_LOADING_WAIT),
            int(AvgServiceState.WAITING_FOR_RETURN_REQUEST),
        }
        returning_states = {
            int(AvgServiceState.RETURNING_TO_DROP_ZONE),
            int(AvgServiceState.RETURN_WITH_CARGO),
            int(AvgServiceState.DROP_ZONE_PARKING),
        }
        operator_stop_states = {
            int(AvgServiceState.OPERATOR_STOPPED),
        }
        stationary_drop_zone_states = {
            int(AvgServiceState.DROP_ZONE_WAIT),
            int(AvgServiceState.CHARGING),
        }
        departure_states = {
            int(AvgServiceState.DEPARTING_CHARGER),
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        }
        if state in arrival_states:
            with self._lock:
                site = (
                    self._state.destination.get("site", "")
                    or self._active_mission_site
                )
            if site:
                self._notify_site_arrival(site, state, source=f"service_state:{state}")
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source=f"service_state:{state}")
            self._publish_engage(False, source=f"service_state:{state}")
        elif state in stationary_drop_zone_states:
            # HH_260721 - Charged and uncharged standby are both stopped, selectable states.
            self._schedule_broadcast({"service_state": state})
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source=f"service_state:{state}")
            self._publish_engage(False, source=f"service_state:{state}")
        elif state == int(AvgServiceState.WAITING_FOR_CHARGING):
            # HH_260721 - Charger confirmation wait holds zero command and remains healthy.
            self._schedule_broadcast({"service_state": state, "returning": True})
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source="service_state:WAITING_FOR_CHARGING")
            self._publish_engage(False, source="service_state:WAITING_FOR_CHARGING")
        elif state in departure_states:
            # HH_260721 - Keep command authorization open until drop-zone exit completion.
            self._schedule_broadcast({"service_state": state})
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(True, source=f"service_state:{state}")
        elif state in returning_states:
            self._mark_low_battery_return_started_if_needed()
            self._schedule_broadcast({"service_state": state, "returning": True})
            # HH_260630 - Return-to-drop-zone must re-open the mission/platform
            # gates after the site arrival hold closed them.
            if state == int(AvgServiceState.RETURNING_TO_DROP_ZONE):
                self._publish_camping_site_maneuver_controller_return(source="service_state:RETURNING_TO_DROP_ZONE")
            # HH_260701 - Clear only the manual engage latch during return states.
            # Mission engage owns the platform drive gate here; dropping platform
            # enable on RETURN_WITH_CARGO can stop CRAB_OUT before the campsite
            # maneuver reaches DONE.
            self._publish_engage(
                False,
                source=f"service_state:{state}:manual_clear",
                sync_drive_enable=False,
            )
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(True, source=f"service_state:{state}")
        elif state in operator_stop_states:
            # HH_260724 - Operator cancel/stop is an explicit stopped state, separate from diagnostics WARN.
            with self._lock:
                self._state.ws_site_states = {s: False for s in self.site_names}
                self._state.destination = {"site": "", "run": False}
            self._schedule_broadcast({
                "service_state": state,
                "returning": False,
                "states": {s: False for s in self.site_names},
                "engage": False,
            })
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source=f"service_state:{state}")
            self._publish_engage(False, source=f"service_state:{state}")
        elif state == AvgServiceState.GUEST_RECALL_SERVICE:
            # HJ_260601: Notify robot UI that guest requested a recall.
            self._schedule_broadcast({"guest_recall": True})
        with self._lock:
            battery_percentage = int(self._state.battery_percentage)
        self._update_low_battery_return_policy(
            battery_percentage,
            source=f"service_state:{state_name}",
        )

    def _publish_service_state(self, state: int, source: str) -> None:
        # HH_260706 - Keep ROS state descriptions and logs ASCII/English; UI
        # localization should be handled in the frontend display layer.
        desc_map = {
            AvgServiceState.DROP_ZONE_WAIT:         "Waiting at drop zone",
            AvgServiceState.MOVING_TO_SITE:         "Moving from drop zone to site",
            AvgServiceState.SITE_ARRIVED:           "Arrived at site",
            AvgServiceState.RETURNING_TO_DROP_ZONE: "Returning from site to drop zone",
            AvgServiceState.GUEST_RECALL_SERVICE:   "Guest recall requested",
            AvgServiceState.SITE_ENTRY:             "Entering site",
            AvgServiceState.UNLOAD_WAIT:            "Waiting for unload at site",
            AvgServiceState.RECALL_TO_SITE_ROAD:    "Moving to the site road recall point",
            AvgServiceState.GUEST_LOADING_WAIT:     "Waiting for guest loading",
            AvgServiceState.RETURN_WITH_CARGO:      "Leaving site with cargo",
            AvgServiceState.DROP_ZONE_PARKING:      "Parking at drop zone",
            # HH_260721 - Keep API descriptions explicit and English-only.
            AvgServiceState.WAITING_FOR_RETURN_REQUEST: "Waiting for return request",
            AvgServiceState.WAITING_FOR_CHARGING:   "Waiting for charger connection",
            AvgServiceState.CHARGING:               "Charging",
            AvgServiceState.DEPARTING_CHARGER:      "Departing charger",
            AvgServiceState.DEPARTING_DROP_ZONE:    "Departing drop zone",
            AvgServiceState.OPERATOR_STOPPED:        "Stopped by operator",
        }
        msg = AvgServiceState()
        msg.state = state
        msg.state_name = SERVICE_STATE_NAMES.get(state, f"UNKNOWN_{state}")
        msg.description = desc_map.get(state, f"unknown state {state}")
        # HH_260721 - Update local intent synchronously so CAN edges cannot overwrite departure.
        self._latest_service_state = int(state)
        self.pub_service_state.publish(msg)
        self.get_logger().info(
            f"Service state ({source}) -> {self.service_state_topic}: "
            f"{state} ({msg.description})"
        )

    def _on_destination_command(self, msg: UiDestinationCommand) -> None:
        site = str(msg.site).strip()
        if not site:
            self.get_logger().warn("destination command has empty site")
            return
        run = bool(msg.run)
        source = str(msg.source).strip() if msg.source else "destination_topic"
        if self._is_recent_direct_destination_echo(site, run, source):
            self.get_logger().info(
                "destination echo ignored after direct HTTP dispatch: "
                f"site={site} run={str(run).lower()} source={source}"
            )
            return
        result = self._apply_destination_command(site=site, run=run, source=source)
        # Broadcast guest-origin destination calls to the robot-side UI.
        if source == "guest" and run and not result.get("blocked", False):
            self._schedule_broadcast({"guest_navigate": site})
        self.get_logger().info(
            "destination dispatch summary: "
            f"site={site} run={str(run).lower()} source={source} "
            f"mission_key={result.get('mission_key', '')} "
            f"site_goal={str(bool(result.get('goal_pose_published', False))).lower()}"
        )

    def _on_campsite_occupancy(self, msg: CampsiteOccupancy) -> None:
        # HH_260723 - Disable occupied destinations and cancel an active
        # dispatch before the controller can enter the confirmed campsite.
        occupied_keys = {str(key).strip() for key in msg.occupied_mission_keys if key}
        occupied_sites = sorted(
            site
            for site in self.site_names
            if (self._resolve_mission_key_for_site(site) or "") in occupied_keys
        )
        active_occupied_site = ""
        with self._lock:
            occupancy_changed = self._state.occupied_sites != occupied_sites
            self._state.occupied_sites = occupied_sites
            for site in occupied_sites:
                self._state.ws_site_states[site] = False
            active_site = str(self._state.destination.get("site", "")).strip()
            active_run = bool(self._state.destination.get("run", False))
            if active_run and active_site in occupied_sites:
                active_occupied_site = active_site
                self._state.destination = {"site": active_site, "run": False}

        if occupancy_changed:
            self._schedule_broadcast({"occupied_sites": occupied_sites})
        if active_occupied_site:
            self.get_logger().warn(
                f"active campsite became occupied; stopping dispatch: {active_occupied_site}"
            )
            self._apply_destination_command(
                site=active_occupied_site,
                run=False,
                source="perception_occupancy",
            )
            self._schedule_broadcast(
                {
                    "states": {s: False for s in self.site_names},
                    "engage": False,
                    "error": "campsite_occupied",
                    "site": active_occupied_site,
                }
            )

    # ── Goal and engage publishing ────────────────────────────────────────────

    def _publish_camping_site_maneuver_controller_return(self, source: str) -> None:
        # HH_260720 - Publish a semantic RETURN operation instead of a context-free Bool.
        msg = MotionOperation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = MotionOperation.RETURN
        msg.source = source
        self.pub_camping_site_maneuver_controller_operation.publish(msg)
        self.get_logger().info(
            f"site maneuver return ({source}) -> {self.camping_site_maneuver_controller_operation_topic}"
        )

    def _on_ui_camping_site_operation_request(self, msg: MotionOperation) -> None:
        """Translate frontend intent into the shared operational state machine."""
        source = str(msg.source).strip() or "ui_operation_request"
        if int(msg.operation) != int(MotionOperation.RETURN):
            self.get_logger().warn(
                "unsupported UI campsite operation request: "
                f"operation={int(msg.operation)} source={source}"
            )
            return
        self._request_return_to_drop_zone(source=source)

    def _request_return_to_drop_zone(self, source: str) -> None:
        # HH_260803 - Publishing RETURNING_TO_DROP_ZONE is sufficient: the
        # service-state callback below performs RETURN, manual-clear, and mission
        # engage exactly once for robot UI, guest UI, and any future frontend.
        self._publish_service_state(
            AvgServiceState.RETURNING_TO_DROP_ZONE,
            source=source,
        )

    def _publish_camping_site_operation(self, operation: int, source: str) -> None:
        msg = MotionOperation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = int(operation)
        msg.source = source
        self.pub_camping_site_maneuver_controller_operation.publish(msg)
        self.get_logger().info(
            f"site maneuver operation ({source}) -> "
            f"{self.camping_site_maneuver_controller_operation_topic}: {int(operation)}"
        )

    def _publish_camping_site_maneuver_controller_adopt(self, site: str, mission_key: str, source: str) -> None:
        msg = UiDestinationCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.site = site
        msg.run = True
        msg.mission_key = mission_key
        msg.source = source
        self.pub_camping_site_maneuver_controller_adopt.publish(msg)
        self.get_logger().info(
            f"site maneuver adopt ({source}) -> {self.camping_site_maneuver_controller_adopt_topic}: "
            f"site={site} mission_key={mission_key}"
        )

    # HH_260721 - Start or cancel the bounded drop-zone departure controller with a typed command.
    def _publish_drop_zone_operation(self, operation: int, source: str) -> None:
        msg = MotionOperation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = int(operation)
        msg.source = source
        self.pub_drop_zone_maneuver_controller_operation.publish(msg)
        self.get_logger().info(
            f"drop-zone operation ({source}) -> "
            f"{self.drop_zone_maneuver_controller_operation_topic}: {int(operation)}"
        )

    # HH_260721 - Prevent a parked controller from publishing standby during charger release.
    def _publish_parking_operation(self, operation: int, source: str) -> None:
        msg = MotionOperation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.operation = int(operation)
        msg.source = source
        self.pub_parking_operation.publish(msg)
        self.get_logger().info(
            f"parking operation ({source}) -> {self.parking_operation_topic}: "
            f"{int(operation)}"
        )

    def _cancel_active_motion(self, source: str) -> None:
        # HH_260724 - Operator cancel/stop should leave no stale Nav2 or maneuver owner active.
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        sent_topics: List[str] = []
        for topic, client in zip(self.nav2_cancel_action_topics, self.nav2_cancel_clients):
            if not client.service_is_ready():
                continue
            # HH_260804 - rclpy service clients expose call_async(); using the
            # rclcpp-style async_send_request() made /ui/stop return HTTP 500
            # before it could cancel Nav2 or publish the stopped service state.
            client.call_async(request)
            sent_topics.append(topic)
        self._publish_camping_site_operation(
            MotionOperation.CANCEL, source=f"{source}:operator_stop"
        )
        self._publish_drop_zone_operation(
            MotionOperation.CANCEL, source=f"{source}:operator_stop"
        )
        self._publish_parking_operation(
            MotionOperation.CANCEL, source=f"{source}:operator_stop"
        )
        if sent_topics:
            self.get_logger().info(
                f"Nav2 cancel requested ({source}): {', '.join(sent_topics)}"
            )
        else:
            self.get_logger().warn(
                f"Nav2 cancel skipped ({source}): cancel services not ready"
            )

    def _stop_active_service(self, source: str) -> None:
        # HH_260724 - Stop/cancel is a state transition, not only a command-gate update.
        self._drop_zone_exit_active = False
        self._pending_site_after_drop_zone_exit = None
        self._cancel_active_motion(source=source)
        if self.publish_mission_engage_from_destination:
            self._publish_mission_engage(False, source=source)
        self._publish_engage(False, source=source)
        with self._lock:
            self._active_mission_site = ""
            self._state.ws_site_states = {s: False for s in self.site_names}
            self._state.destination = {"site": "", "run": False}
        self._publish_service_state(
            AvgServiceState.OPERATOR_STOPPED,
            source=f"{source}:operator_stop",
        )
        self._schedule_broadcast({
            "states": {s: False for s in self.site_names},
            "engage": False,
            "returning": False,
        })

    def _publish_platform_drive_enable(self, enabled: bool, source: str) -> None:
        if not self.publish_platform_drive_enable_with_engage:
            return
        msg = AvgBool()
        msg.data = bool(enabled)
        self.pub_platform_drive_enable.publish(msg)
        self.get_logger().info(
            f"platform drive-enable ({source}) -> {self.platform_drive_enable_topic}: "
            f"{str(bool(enabled)).lower()}"
        )

    def _resolve_mission_key_for_site(self, site: str) -> Optional[str]:
        if site in self.site_to_mission_key_map:
            mapped = self.site_to_mission_key_map[site]
            if mapped in self._keypoints_by_mission_key:
                return mapped

        site_text = str(site).strip().upper()
        if site_text.startswith("B") and site_text[1:].isdigit():
            candidate = f"camping_site_{int(site_text[1:])}"
            if candidate in self._keypoints_by_mission_key:
                return candidate

        if self.fallback_mission_key in self._keypoints_by_mission_key:
            self.get_logger().warn(
                f"no exact mission key for site={site}; fallback to {self.fallback_mission_key}"
            )
            return self.fallback_mission_key

        if self.fallback_to_first_known_goal and self._keypoints_by_mission_key:
            fallback = sorted(self._keypoints_by_mission_key.keys())[0]
            self.get_logger().warn(
                f"no exact mission key for site={site}; fallback to first known key={fallback}"
            )
            return fallback

        if site_text.startswith("B") and site_text[1:].isdigit():
            return f"camping_site_{int(site_text[1:])}"
        return None

    def _is_site_occupied(self, site: str) -> bool:
        mission_key = self._resolve_mission_key_for_site(site) or ""
        with self._lock:
            occupied_sites = set(self._state.occupied_sites)
        return bool(mission_key) and site in occupied_sites

    def _publish_engage(
        self, enabled: bool, source: str, *, sync_drive_enable: bool = True
    ) -> None:
        msg = AvgBool()
        msg.data = bool(enabled)
        self.pub_engage.publish(msg)
        if sync_drive_enable:
            self._publish_platform_drive_enable(enabled, source=source)

        self._update_runtime_state(
            lambda: self._runtime_policy.update_engaged(bool(enabled))
        )

        self.get_logger().info(
            f"engage command ({source}) -> {self.planning_engage_topic}: "
            f"{str(bool(enabled)).lower()}"
        )

    def _publish_mission_engage(self, enabled: bool, source: str) -> None:
        msg = AvgBool()
        msg.data = bool(enabled)
        self.pub_mission_engage.publish(msg)
        self._publish_platform_drive_enable(enabled, source=source)

        self._update_runtime_state(
            lambda: self._runtime_policy.update_engaged(bool(enabled))
        )

        self.get_logger().info(
            f"mission engage command ({source}) -> {self.planning_mission_engage_topic}: "
            f"{str(bool(enabled)).lower()}"
        )

    # HH_260721 - Publish mission intent separately so charging departure can open before Nav2.
    def _publish_site_mission_key(self, mission_key: str, source: str) -> None:
        if self.publish_mission_key:
            key_msg = PlanningMissionKey()
            key_msg.header.stamp = self.get_clock().now().to_msg()
            key_msg.mission_key = mission_key
            key_msg.source = source
            key_msg.publish_route_goal = False
            self.pub_mission_key.publish(key_msg)
            self.get_logger().info(
                f"mission key ({source}) -> {self.planning_mission_key_topic}: {mission_key}"
            )

    # HH_260721 - Publish the map-selected operational campsite target for route and control use.
    def _publish_site_goal_pose(self, site: str, mission_key: str, source: str) -> bool:
        pose_published = False
        goal = self._keypoints_by_mission_key.get(mission_key)
        if self.publish_goal_pose and goal is not None:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = goal.frame_id
            pose.pose.position.x = goal.x
            pose.pose.position.y = goal.y
            pose.pose.position.z = goal.z
            qx, qy, qz, qw = self._yaw_deg_to_quaternion(goal.yaw_deg)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pub_goal_pose.publish(pose)
            pose_published = True
            self.get_logger().info(
                f"site goal ({source}) -> {self.planning_goal_pose_topic}: "
                f"mission_key={mission_key} site={site} service_mode={goal.service_mode} "
                f"xy=({goal.x:.2f},{goal.y:.2f})"
            )

        if self.publish_goal_pose and goal is None:
            self.get_logger().warn(
                f"site goal dispatch skipped: mission key '{mission_key}' is missing in camping_sites_yaml"
            )
        return pose_published

    def _publish_goal_for_site(self, site: str, source: str) -> Dict[str, Any]:
        mission_key = self._resolve_mission_key_for_site(site)
        if not mission_key:
            return {
                "mission_key": "",
                "goal_pose_published": False,
                "message": f"no mission key resolved for site={site}",
            }

        self._publish_site_mission_key(mission_key, source)
        pose_published = self._publish_site_goal_pose(site, mission_key, source)

        return {
            "mission_key": mission_key,
            "goal_pose_published": pose_published,
            "message": "ok",
        }

    def _parse_destination_payload(self, raw: str) -> Optional[tuple[str, bool]]:
        text = str(raw).strip()
        if not text:
            return None
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            self.get_logger().warn(f"invalid destination JSON: {text}")
            return None

        if not isinstance(payload, dict):
            self.get_logger().warn(f"destination payload must be JSON object: {text}")
            return None

        site = str(payload.get("site", "")).strip()
        run = bool(payload.get("run", False))
        if not site:
            with self._lock:
                site = str(self._state.destination.get("site", "")).strip()

        if not site:
            self.get_logger().warn("destination payload has empty site")
            return None

        return (site, run)

    def _apply_destination_command(self, site: str, run: bool, source: str) -> Dict[str, Any]:
        if not run:
            # HH_260724 - Destination OFF is an operator stop, so cancel Nav2 and service progress.
            self._stop_active_service(source=f"{source}:destination_stop")
            return {
                "site": site,
                "run": False,
                "mission_key": "",
                "goal_pose_published": False,
                "message": "run=false -> operator stop, engage off, goal cancelled",
            }

        pending_departure = getattr(self, "_pending_site_after_drop_zone_exit", None)
        if (
            getattr(self, "_drop_zone_exit_active", False)
            and pending_departure is not None
            and pending_departure[0] == site
        ):
            # HH_260807 - A reliable ROS topic or websocket retry may deliver the
            # same selection more than once. Keep one motion owner and one state
            # transition while the already accepted station exit is in progress.
            self.get_logger().info(
                f"duplicate destination ignored during drop-zone departure: site={site}"
            )
            return {
                "site": site,
                "run": True,
                "mission_key": pending_departure[1],
                "goal_pose_published": False,
                "message": "destination already pending drop-zone departure",
            }

        mission_key = self._resolve_mission_key_for_site(site) or ""
        if self._is_site_occupied(site):
            self.get_logger().warn(
                f"occupied campsite selection blocked: site={site} mission_key={mission_key}"
            )
            self._schedule_broadcast(
                {"error": "campsite_occupied", "site": site, "occupied": True}
            )
            return {
                "site": site,
                "run": False,
                "mission_key": mission_key,
                "goal_pose_published": False,
                "blocked": True,
                "message": "campsite occupied by detected tent",
            }

        already_arrived, mission_key, distance_m, match_reason = self._site_arrival_match(site)
        if already_arrived:
            with self._lock:
                self._active_mission_site = site
            # HH_260701 - If the robot was manually driven into a campsite,
            # selecting that site in the UI should adopt the parked state instead
            # of dispatching a fresh Nav2 goal back through the lanelet route.
            self._publish_camping_site_maneuver_controller_adopt(site, mission_key, source=f"{source}:already_at_site")
            self._publish_service_state(
                AvgServiceState.UNLOAD_WAIT,
                source=f"{source}:already_at_site:{match_reason}",
            )
            self._notify_site_arrival(
                site,
                int(AvgServiceState.UNLOAD_WAIT),
                source=f"{source}:already_at_site:{match_reason}",
                already_at_site=True,
            )
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source=f"{source}:already_at_site")
            self._publish_engage(False, source=f"{source}:already_at_site")
            return {
                "site": site,
                "run": True,
                "mission_key": mission_key,
                "goal_pose_published": False,
                "message": (
                    f"already at site ({match_reason}, distance={distance_m:.2f}m) "
                    "-> arrival adopted"
                ),
            }

        battery_block = self._mission_dispatch_battery_block(site)
        if battery_block is not None:
            self.get_logger().warn(
                "campsite dispatch blocked by battery gate: "
                f"site={site} battery={battery_block.get('battery_percentage')} "
                f"minimum={battery_block.get('minimum_battery_percentage')}"
            )
            self._schedule_broadcast(battery_block)
            return {
                "site": site,
                "run": False,
                "mission_key": mission_key,
                "goal_pose_published": False,
                "blocked": True,
                "message": str(battery_block["message"]),
                "error": "battery_below_mission_minimum",
                "battery_percentage": battery_block["battery_percentage"],
                "minimum_battery_percentage": battery_block["minimum_battery_percentage"],
            }

        # HJ_260804 - Only accepted/adopted destinations become the fallback
        # arrival identity. A battery-rejected request must not replace it.
        with self._lock:
            self._active_mission_site = site

        # HH_260730 - Record accepted UI intent before engage so regulated and
        # manual goals expose the same goal-received -> path-preparing order.
        self._update_runtime_state(
            lambda: self._runtime_policy.update_goal_received("regulated")
        )
        if self.publish_engage_from_destination:
            self._publish_engage(True, source=f"{source}:destination")
        if self.publish_mission_engage_from_destination:
            self._publish_mission_engage(True, source=f"{source}:destination")

        # HH_260721 - A parked/charging robot must leave the station before Nav2 gets a site goal.
        departure_required = (
            self._latest_platform_is_charging
            or self._latest_service_state
            in {
                int(AvgServiceState.DROP_ZONE_WAIT),
                int(AvgServiceState.CHARGING),
            }
        )
        mission_key = self._resolve_mission_key_for_site(site) or ""
        if departure_required and mission_key:
            self._publish_site_mission_key(mission_key, source)
            self._pending_site_after_drop_zone_exit = (site, mission_key, source)
            if not self._drop_zone_exit_active:
                self._drop_zone_exit_active = True
                # HH_260721 - Transfer motion ownership from final parking to station departure.
                self._publish_parking_operation(
                    MotionOperation.CANCEL, source=f"{source}:site_departure"
                )
                self._publish_drop_zone_operation(
                    MotionOperation.EXIT, source=f"{source}:site_departure"
                )
            # HH_260721 - Show physical departure before the Nav2 site route is released.
            departure_state = (
                AvgServiceState.DEPARTING_CHARGER
                if self._latest_platform_is_charging
                else AvgServiceState.DEPARTING_DROP_ZONE
            )
            self._publish_service_state(
                departure_state,
                source=f"{source}:drop_zone_departure",
            )
            return {
                "site": site,
                "run": True,
                "mission_key": mission_key,
                "goal_pose_published": False,
                "message": "site goal pending drop-zone straight exit and yaw alignment",
            }

        self._publish_service_state(AvgServiceState.MOVING_TO_SITE, source=f"{source}:start")
        goal_result = self._publish_goal_for_site(site=site, source=source)
        return {
            "site": site,
            "run": True,
            "mission_key": goal_result.get("mission_key", ""),
            "goal_pose_published": bool(goal_result.get("goal_pose_published", False)),
            "message": str(goal_result.get("message", "ok")),
        }

    def _is_recent_direct_destination_echo(self, site: str, run: bool, source: str) -> bool:
        recent = self._last_direct_destination_echo
        if recent is None:
            return False
        recent_site, recent_run, recent_source, recent_time_s = recent
        if site != recent_site or bool(run) != recent_run or source != recent_source:
            return False
        age_s = self.get_clock().now().nanoseconds * 1e-9 - recent_time_s
        if 0.0 <= age_s <= 1.0:
            return True
        self._last_direct_destination_echo = None
        return False

    def _publish_destination_command(self, site: str, run: bool, source: str) -> Dict[str, Any]:
        payload = {"site": site, "run": bool(run)}

        msg = UiDestinationCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.site = site
        msg.run = bool(run)
        msg.mission_key = self._resolve_mission_key_for_site(site) or ""
        msg.source = source
        self.pub_destination.publish(msg)

        with self._lock:
            self._state.destination = dict(payload)

        self.get_logger().info(
            f"destination command ({source}) -> {self.ui_destination_topic}: "
            f"site={site}, run={str(bool(run)).lower()}"
        )
        return payload

    def _snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "engaged": self._state.engaged,
                "ready": self._state.ready,
                "headlight": self._state.headlight,
                "operation_mode": self._state.operation_mode,
                "ready_message": self._state.ready_message,
                "module_states": list(self._state.module_states),
                "diagnostics": list(self._state.diagnostics),
                "diagnostics_agg_count": self._state.diagnostics_agg_count,
                "diagnostics_agg_error_count": self._state.diagnostics_agg_error_count,
                "system_health": self._state.system_health,
                "mission_phase": self._state.mission_phase,
                "mission_source": self._state.mission_source,
                "service_state": self._state.service_state,
                "service_state_name": self._state.service_state_name,
                "service_state_description": self._state.service_state_description,
                "destination": dict(self._state.destination),
                "battery_percentage": self._state.battery_percentage,
                # HH_260724 - Initial UI snapshots carry the active battery policy state,
                # not only edge-triggered websocket updates.
                "battery_return_pending": self._low_battery_return_pending,
                "battery_return_started": self._low_battery_return_started,
                "battery_return_waiting_for_user": self._low_battery_return_wait_notified,
                "minimum_battery_percentage": self.low_battery_return_threshold_percent,
                "occupied_sites": list(self._state.occupied_sites),
            }

    # ── Public API methods (called by HTTP handlers) ──────────────────────────

    def set_engage(self, value: bool) -> Dict[str, Any]:
        self._publish_engage(bool(value), source="http")
        self._schedule_broadcast({"engage": bool(value)})
        return {"success": True, "message": "engage command published", "value": bool(value)}

    # HH_260720 - Headlight toggle publishes the generated control command contract.
    def set_headlight(self, value: bool) -> Dict[str, Any]:
        msg = AvgBool()
        msg.data = bool(value)
        self.pub_headlight.publish(msg)
        with self._lock:
            self._state.headlight = bool(value)
        self._schedule_broadcast({"headlight": bool(value)})
        self.get_logger().info(f"headlight command (http): {bool(value)}")
        return {"success": True, "message": "headlight command published", "value": bool(value)}

    def set_operation_mode(self, value: bool) -> Dict[str, Any]:
        self._publish_engage(bool(value), source="http_operation_mode")
        self._schedule_broadcast({"engage": bool(value)})
        return {
            "success": True,
            "message": "operation_mode command forwarded as engage",
            "auto": bool(value),
        }

    def set_auto(self) -> Dict[str, Any]:
        self._publish_engage(True, source="http_auto")
        self._schedule_broadcast({"engage": True})
        return {"success": True, "message": "auto command published"}

    def set_stop(self) -> Dict[str, Any]:
        self._stop_active_service(source="http_stop")
        return {"success": True, "message": "stop command published"}

    def set_destination(self, site: str, run: bool) -> Dict[str, Any]:
        normalized_site = str(site).strip()
        if not normalized_site:
            return {"success": False, "message": "site is required"}
        if normalized_site not in self.site_names:
            return {
                "success": False,
                "message": f"unknown site: {normalized_site}",
                "valid_sites": list(self.site_names),
            }
        if run and self._is_site_occupied(normalized_site):
            return {
                "success": False,
                "message": "campsite occupied by detected tent",
                "site": normalized_site,
                "error": "campsite_occupied",
            }
        if run:
            battery_block = self._mission_dispatch_battery_block(normalized_site)
            if battery_block is not None:
                return {"success": False, **battery_block}

        with self._lock:
            self._state.ws_site_states = {s: (s == normalized_site and run) for s in self.site_names}
        self._schedule_broadcast({
            "states": {s: (s == normalized_site and run) for s in self.site_names}
        })

        self._last_direct_destination_echo = (
            normalized_site,
            bool(run),
            "http_ui_destination",
            self.get_clock().now().nanoseconds * 1e-9,
        )
        payload = self._publish_destination_command(
            site=normalized_site,
            run=bool(run),
            source="http_ui_destination",
        )
        dispatch = self._apply_destination_command(
            site=normalized_site,
            run=bool(run),
            source="http_ui_destination",
        )
        return {"success": True, "destination": payload, "dispatch": dispatch}

    async def _await_ros_future(self, future: Any, timeout_s: float = 1.5) -> Any:
        deadline = asyncio.get_running_loop().time() + timeout_s
        while not future.done():
            if asyncio.get_running_loop().time() >= deadline:
                raise TimeoutError("ROS parameter service timed out")
            await asyncio.sleep(0.02)
        return future.result()

    async def get_platform_tuning(self) -> Dict[str, Any]:
        if not self.get_ranger_parameters_client.service_is_ready():
            return {
                "success": False,
                "available": False,
                "message": f"{self.ranger_base_node_name} parameter service is unavailable",
            }
        request = GetParameters.Request()
        request.names = [self.steering_transition_parameter]
        try:
            response = await self._await_ros_future(
                self.get_ranger_parameters_client.call_async(request)
            )
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "available": False, "message": str(exc)}
        if not response.values:
            return {"success": False, "available": True, "message": "parameter not returned"}
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_DOUBLE:
            return {
                "success": False,
                "available": True,
                "message": "driver parameter is not a double",
            }
        return {
            "success": True,
            "available": True,
            "steering_transition_rate_radps": float(value.double_value),
            # HH_260727 - UI ratio is relative to the documented 1.0 rad/s reference.
            "steering_transition_ratio": float(value.double_value),
            "min_radps": 0.05,
            "max_radps": 2.0,
        }

    async def set_platform_tuning(
        self, steering_transition_rate_radps: float
    ) -> Dict[str, Any]:
        requested = float(steering_transition_rate_radps)
        if not math.isfinite(requested) or not 0.05 <= requested <= 2.0:
            return {
                "success": False,
                "available": True,
                "message": "steering_transition_rate_radps must be in [0.05, 2.0]",
            }
        if not self.set_ranger_parameters_client.service_is_ready():
            return {
                "success": False,
                "available": False,
                "message": f"{self.ranger_base_node_name} parameter service is unavailable",
            }

        parameter = Parameter()
        parameter.name = self.steering_transition_parameter
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=requested,
        )
        request = SetParameters.Request()
        request.parameters = [parameter]
        try:
            response = await self._await_ros_future(
                self.set_ranger_parameters_client.call_async(request)
            )
        except Exception as exc:  # noqa: BLE001
            return {"success": False, "available": False, "message": str(exc)}
        if not response.results or not response.results[0].successful:
            reason = response.results[0].reason if response.results else "no result"
            return {"success": False, "available": True, "message": reason}

        payload = {
            "success": True,
            "available": True,
            "steering_transition_rate_radps": requested,
            "steering_transition_ratio": requested,
            "message": "steering transition rate updated dynamically",
        }
        self._schedule_broadcast({"platform_tuning": payload})
        return payload

    # ── FastAPI server ────────────────────────────────────────────────────────

    def _resolve_frontend_dir(self) -> Optional[Path]:
        def realdir(p: Path) -> Path:
            # Resolve symlinks so Starlette's security check (commonprefix) passes.
            return Path(os.path.realpath(str(p)))

        if self.frontend_dir and self.frontend_dir.exists():
            return realdir(self.frontend_dir)
        try:
            share = Path(get_package_share_directory("camrod_ui"))
            candidate = share / "camrod_ui_robot" / "assets" / "frontend" / "build"
            if candidate.exists():
                return realdir(candidate)
        except PackageNotFoundError:
            pass
        # parents[3] = package root (runtime/python/camrod_ui → runtime/python → runtime → pkg)
        source_candidate = Path(__file__).resolve().parents[3] / "camrod_ui_robot" / "assets" / "frontend" / "build"
        if source_candidate.exists():
            return realdir(source_candidate)
        return None

    def _start_fastapi_server(self) -> None:
        node = self
        app = FastAPI(title="camrod_ui_backend")
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # ── WebSocket endpoint ────────────────────────────────────────────────

        @app.websocket("/ws")
        async def websocket_endpoint(ws: WebSocket) -> None:
            await ws.accept()
            with node._ws_clients_lock:
                node._ws_clients.add(ws)

            # Send initial state on connect.
            with node._lock:
                states = dict(node._state.ws_site_states)
                engage = node._state.engaged
                ready = node._state.ready
                ready_message = node._state.ready_message
                battery = node._state.battery_percentage
                system_health = node._state.system_health
                mission_phase = node._state.mission_phase
                mission_source = node._state.mission_source
                service_state = node._state.service_state
                service_state_name = node._state.service_state_name
                service_state_description = node._state.service_state_description
                occupied_sites = list(node._state.occupied_sites)
            await ws.send_json({"states": states})
            await ws.send_json({"occupied_sites": occupied_sites})
            await ws.send_json({"engage": engage})
            await ws.send_json({
                "ready": ready,
                "ready_message": ready_message,
                "mission_phase": mission_phase,
                "mission_source": mission_source,
            })
            if battery >= 0:
                await ws.send_json({"battery": battery})
            await ws.send_json({"system_health": system_health})
            await ws.send_json({
                "service_state": service_state,
                "service_state_name": service_state_name,
                "service_state_description": service_state_description,
            })

            try:
                while True:
                    data = await ws.receive_text()
                    try:
                        payload = json.loads(data)
                    except json.JSONDecodeError as exc:
                        # HH_260616: Keep malformed WebSocket frames from tearing down
                        # the UI bridge; browser/UI retries should not leave stale goals.
                        node.get_logger().warn(f"invalid websocket JSON ignored: {exc}")
                        continue

                    if not isinstance(payload, dict):
                        # HH_260616: The UI protocol is object-based. Ignore other
                        # payload shapes instead of raising inside Starlette.
                        node.get_logger().warn("websocket payload must be a JSON object")
                        continue

                    # {"site": "B1", "state": true/false}
                    if "site" in payload and "state" in payload:
                        site = str(payload["site"])
                        new_state = bool(payload["state"])
                        if site not in node.site_names:
                            node.get_logger().warn(f"unknown websocket site ignored: {site}")
                            await ws.send_json({
                                "error": "unknown_site",
                                "site": site,
                                "valid_sites": list(node.site_names),
                            })
                            continue
                        if new_state and node._is_site_occupied(site):
                            await ws.send_json({
                                "error": "campsite_occupied",
                                "site": site,
                                "occupied": True,
                            })
                            continue
                        if new_state:
                            battery_block = node._mission_dispatch_battery_block(site)
                            if battery_block is not None:
                                await ws.send_json(battery_block)
                                continue
                        if new_state:
                            # Deactivate all other sites, activate this one.
                            with node._lock:
                                node._state.ws_site_states = {
                                    s: (s == site) for s in node.site_names
                                }
                            # HH_260616: Publish one destination command and let the
                            # /ui/selected_destination subscriber dispatch engage/goal.
                            # This keeps WebSocket behavior identical to REST and prevents
                            # duplicate mission_key/site_goal publications for one button tap.
                            node._publish_destination_command(site, run=True, source="ws")
                            await node._broadcast(
                                {"states": {s: (s == site) for s in node.site_names}}
                            )
                            await node._broadcast({"engage": True})
                        else:
                            with node._lock:
                                node._state.ws_site_states = {s: False for s in node.site_names}
                            # HH_260724 - Let the destination subscriber perform the full operator-stop transition.
                            node._publish_destination_command(site, run=False, source="ws_toggle_off")
                            await node._broadcast(
                                {"states": {s: False for s in node.site_names}}
                            )
                            await node._broadcast({"engage": False})

                    # {"engage": true/false}
                    if "engage" in payload:
                        new_engage = bool(payload["engage"])
                        node._publish_engage(new_engage, source="ws_engage")
                        await node._broadcast({"engage": new_engage})

                    # HH_260617: usage_complete is return-to-drop-zone state=3.
                    # Guest recall request is state=4 and is published by ui_guest_node.
                    if payload.get("usage_complete"):
                        node._request_return_to_drop_zone(source="ws:usage_complete")
                        with node._lock:
                            node._state.ws_site_states = {s: False for s in node.site_names}
                        await node._broadcast({"states": {s: False for s in node.site_names}})
                        await node._broadcast({"engage": False})

            except WebSocketDisconnect:
                pass
            except KeyError as exc:
                # HH_260616: Some non-browser test clients disconnect without a close
                # code; Starlette can surface that as KeyError('code').
                node.get_logger().debug(f"websocket disconnected without close code: {exc}")
            finally:
                with node._ws_clients_lock:
                    node._ws_clients.discard(ws)

        # ── REST API endpoints ────────────────────────────────────────────────

        @app.get("/ui/state")
        def get_state() -> JSONResponse:
            return JSONResponse(node._snapshot())

        @app.get("/ui/health")
        async def get_health() -> JSONResponse:
            return JSONResponse({"ok": True, "node": node.get_name()})

        @app.get("/ui/destination")
        def get_destination() -> JSONResponse:
            snap = node._snapshot()
            return JSONResponse({
                "destination": snap.get("destination", {"site": "", "run": False}),
                "valid_sites": list(node.site_names),
            })

        @app.get("/ui/diagnostics")
        def get_diagnostics() -> JSONResponse:
            snap = node._snapshot()
            return JSONResponse({"status": snap.get("diagnostics", [])})

        @app.get("/api/diagnostics")
        def get_api_diagnostics() -> JSONResponse:
            with node._lock:
                diags = list(node._state.diagnostics)
            return JSONResponse({"status": diags})

        @app.get("/ui/platform_tuning")
        async def get_platform_tuning() -> JSONResponse:
            result = await node.get_platform_tuning()
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/platform_tuning")
        async def post_platform_tuning(
            steering_transition_rate_radps: float = 0.5,
        ) -> JSONResponse:
            result = await node.set_platform_tuning(steering_transition_rate_radps)
            status = 200 if result.get("success") else (
                503 if not result.get("available", False) else 400
            )
            return JSONResponse(result, status_code=status)

        @app.post("/ui/engage")
        def post_engage(value: str = "false") -> JSONResponse:
            enabled = value.lower() in {"1", "true", "yes", "on"}
            result = node.set_engage(enabled)
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/headlight")
        def post_headlight(value: str = "false") -> JSONResponse:
            enabled = value.lower() in {"1", "true", "yes", "on"}
            result = node.set_headlight(enabled)
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/operation_mode")
        def post_operation_mode(auto: str = "false") -> JSONResponse:
            enabled = auto.lower() in {"1", "true", "yes", "on"}
            result = node.set_operation_mode(enabled)
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/auto")
        def post_auto() -> JSONResponse:
            result = node.set_auto()
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/stop")
        def post_stop() -> JSONResponse:
            result = node.set_stop()
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/destination")
        def post_destination(site: str = "", run: str = "false") -> JSONResponse:
            run_bool = run.lower() in {"1", "true", "yes", "on"}
            result = node.set_destination(site=site, run=run_bool)
            return JSONResponse(result, status_code=200 if result.get("success") else 400)

        # ── Static frontend serving ───────────────────────────────────────────
        # Starlette 0.18 StaticFiles rejects symlinked files (commonprefix check
        # fails when realpath(file) points outside realpath(directory) due to
        # --symlink-install chains).  Serve all static assets manually so symlinks
        # are followed transparently.

        frontend_dir = node._resolve_frontend_dir()
        if frontend_dir and frontend_dir.exists():
            index_html = frontend_dir / "index.html"

            @app.get("/{full_path:path}")
            async def serve_spa(full_path: str) -> FileResponse:
                candidate = frontend_dir / full_path
                real = Path(os.path.realpath(str(candidate)))
                if real.is_file():
                    return FileResponse(str(real))
                return FileResponse(str(os.path.realpath(str(index_html))))
        else:
            @app.get("/")
            async def serve_fallback() -> JSONResponse:
                return JSONResponse({"message": "frontend not available"}, status_code=503)

        # ── Server thread ─────────────────────────────────────────────────────

        def _run() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            node._main_loop = loop
            config = uvicorn.Config(
                app,
                host=node.host,
                port=node.port,
                log_level="warning",
                loop="asyncio",
            )
            server = uvicorn.Server(config)
            node._uvicorn_server = server
            if node._server_stop_requested.is_set():
                server.should_exit = True
            node.get_logger().info(f"ui backend fastapi listening: http://{node.host}:{node.port}")
            try:
                loop.run_until_complete(server.serve())
            finally:
                node._uvicorn_server = None
                node._main_loop = None
                loop.close()

        self._server_thread = threading.Thread(target=_run, name="camrod_ui_fastapi", daemon=True)
        self._server_thread.start()


def main() -> None:
    rclpy.init()
    node = UiBackendNode()
    try:
        rclpy.spin(node)
    # HH_260805 - rclpy may report launch-driven SIGINT as either exception;
    # both are normal shutdown paths and must leave launch with exit code 0.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    # HH_260807 - Humble can race context shutdown with take_message conversion
    # and raise RuntimeError instead of ExternalShutdownException. Suppress it
    # only after the ROS context is down; operational RuntimeError still fails.
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
