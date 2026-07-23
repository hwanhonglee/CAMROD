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
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from avg_msgs.msg import (
    AvgServiceState,
    AvgBool,
    AvgPlatformStatus,
    AvgPoseStamped,
    CampsiteOccupancy,
    MotionOperation,
    PlanningMissionKey,
    UiDestinationCommand,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
# HH_260721 - Keep only the FastAPI symbols used by the runtime backend.
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

import uvicorn

from camrod_ui.api_common import to_diag_level_int

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
        self.planning_goal_pose_topic = str(
            self.declare_parameter("planning_goal_pose_topic", "/goal_pose").value
        )
        self.platform_status_topic = str(
            self.declare_parameter("platform_status_topic", "/platform/status").value
        )
        self.service_state_topic = str(
            self.declare_parameter("service_state_topic", "/service/state").value
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
        self._keypoints_by_mission_key = self._load_camping_site_keypoints(self.camping_sites_yaml)
        self._lock = threading.Lock()
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
        # HH_260706 - HTTP site selection dispatches immediately; ignore the
        # local topic echo so the same camping-site command is not applied twice.
        self._last_direct_destination_echo: Optional[tuple[str, bool, str, float]] = None

        # WebSocket client management.
        self._ws_clients: Set[WebSocket] = set()
        self._ws_clients_lock = threading.Lock()
        self._main_loop: Optional[asyncio.AbstractEventLoop] = None

        # Subscriptions.
        self.sub_destination = self.create_subscription(
            UiDestinationCommand,
            self.ui_destination_topic,
            self._on_destination_command,
            10,
        )
        self.sub_diagnostics_agg = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_agg_topic,
            self._on_diagnostics_agg,
            10,
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
        self._server_thread: Optional[threading.Thread] = None
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
        return super().destroy_node()

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
        if self._main_loop is not None:
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

        ready = bool(msg.status) and error_count == 0
        if not msg.status:
            ready_message = "no diagnostics yet"
        elif error_count > 0:
            ready_message = f"diagnostics errors: {error_count}"
        else:
            ready_message = "ready"

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
            self._state.ready = ready
            self._state.ready_message = ready_message
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
            )
        if health_changed:
            # HH_260721 - Broadcast health transitions without mixing them with service progress.
            self._schedule_broadcast({"system_health": system_health})

    def _on_platform_status(self, msg: AvgPlatformStatus) -> None:
        # HH_260720 - UI battery state comes from the canonical generated platform status.
        # HH_260721 - Charging state also decides whether a campsite goal must wait for departure.
        charging = bool(msg.is_charging)
        charging_changed = charging != self._latest_platform_is_charging
        self._latest_platform_is_charging = charging
        if charging_changed and charging:
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
        pct = max(0, min(100, int(round(float(msg.battery_percentage) * 100.0))))
        with self._lock:
            self._state.battery_percentage = pct
        self._schedule_broadcast({"battery": pct})

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
                site = self._state.destination.get("site", "")
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
        elif state == AvgServiceState.GUEST_RECALL_SERVICE:
            # HJ_260601: Notify robot UI that guest requested a recall.
            self._schedule_broadcast({"guest_recall": True})

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
        occupied_keys = {str(key).strip() for key in msg.occupied_mission_keys if key}
        occupied_sites = sorted(
            site
            for site in self.site_names
            if (self._resolve_mission_key_for_site(site) or "") in occupied_keys
        )
        active_occupied_site = ""
        with self._lock:
            self._state.occupied_sites = occupied_sites
            for site in occupied_sites:
                self._state.ws_site_states[site] = False
            active_site = str(self._state.destination.get("site", "")).strip()
            active_run = bool(self._state.destination.get("run", False))
            if active_run and active_site in occupied_sites:
                active_occupied_site = active_site
                self._state.destination = {"site": active_site, "run": False}

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

        with self._lock:
            self._state.engaged = bool(enabled)
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
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

        with self._lock:
            self._state.engaged = bool(enabled)
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
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
            # HH_260721 - A stop request cancels any goal waiting behind drop-zone departure.
            if self._drop_zone_exit_active:
                self._publish_drop_zone_operation(
                    MotionOperation.CANCEL, source=f"{source}:destination_stop"
                )
            self._drop_zone_exit_active = False
            self._pending_site_after_drop_zone_exit = None
            if self.publish_engage_from_destination:
                self._publish_engage(False, source=f"{source}:destination")
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source=f"{source}:destination")
            return {
                "site": site,
                "run": False,
                "mission_key": "",
                "goal_pose_published": False,
                "message": "run=false -> engage off, goal dispatch skipped",
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
                "service_state": self._state.service_state,
                "service_state_name": self._state.service_state_name,
                "service_state_description": self._state.service_state_description,
                "destination": dict(self._state.destination),
                "battery_percentage": self._state.battery_percentage,
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
        self._publish_engage(False, source="http_stop")
        self._schedule_broadcast({"engage": False})
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
                battery = node._state.battery_percentage
                system_health = node._state.system_health
                service_state = node._state.service_state
                service_state_name = node._state.service_state_name
                service_state_description = node._state.service_state_description
                occupied_sites = list(node._state.occupied_sites)
            await ws.send_json({"states": states})
            await ws.send_json({"occupied_sites": occupied_sites})
            await ws.send_json({"engage": engage})
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
                                node._state.ws_site_states[site] = False
                            if node.publish_mission_engage_from_destination:
                                node._publish_mission_engage(False, source="ws_toggle_off")
                            node._publish_engage(False, source="ws_toggle_off")
                            node._publish_destination_command(site, run=False, source="ws_toggle_off")
                            await node._broadcast({"site": site, "state": False})
                            await node._broadcast({"engage": False})

                    # {"engage": true/false}
                    if "engage" in payload:
                        new_engage = bool(payload["engage"])
                        node._publish_engage(new_engage, source="ws_engage")
                        await node._broadcast({"engage": new_engage})

                    # HH_260617: usage_complete is return-to-drop-zone state=3.
                    # Guest recall request is state=4 and is published by ui_guest_node.
                    if payload.get("usage_complete"):
                        node._publish_service_state(AvgServiceState.RETURNING_TO_DROP_ZONE, source="ws:usage_complete")
                        node._publish_camping_site_maneuver_controller_return(source="ws:usage_complete")
                        node._publish_engage(
                            False,
                            source="ws:usage_complete:manual_clear",
                            sync_drive_enable=False,
                        )
                        if node.publish_mission_engage_from_destination:
                            node._publish_mission_engage(True, source="ws:usage_complete")
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
        async def get_state() -> JSONResponse:
            return JSONResponse(node._snapshot())

        @app.get("/ui/health")
        async def get_health() -> JSONResponse:
            return JSONResponse({"ok": True, "node": node.get_name()})

        @app.get("/ui/destination")
        async def get_destination() -> JSONResponse:
            snap = node._snapshot()
            return JSONResponse({
                "destination": snap.get("destination", {"site": "", "run": False}),
                "valid_sites": list(node.site_names),
            })

        @app.get("/ui/diagnostics")
        async def get_diagnostics() -> JSONResponse:
            snap = node._snapshot()
            return JSONResponse({"status": snap.get("diagnostics", [])})

        @app.get("/api/diagnostics")
        async def get_api_diagnostics() -> JSONResponse:
            with node._lock:
                diags = list(node._state.diagnostics)
            return JSONResponse({"status": diags})

        @app.post("/ui/engage")
        async def post_engage(value: str = "false") -> JSONResponse:
            enabled = value.lower() in {"1", "true", "yes", "on"}
            result = node.set_engage(enabled)
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/headlight")
        async def post_headlight(value: str = "false") -> JSONResponse:
            enabled = value.lower() in {"1", "true", "yes", "on"}
            result = node.set_headlight(enabled)
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/operation_mode")
        async def post_operation_mode(auto: str = "false") -> JSONResponse:
            enabled = auto.lower() in {"1", "true", "yes", "on"}
            result = node.set_operation_mode(enabled)
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/auto")
        async def post_auto() -> JSONResponse:
            result = node.set_auto()
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/stop")
        async def post_stop() -> JSONResponse:
            result = node.set_stop()
            return JSONResponse(result, status_code=200 if result.get("success") else 503)

        @app.post("/ui/destination")
        async def post_destination(site: str = "", run: str = "false") -> JSONResponse:
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
            node._main_loop = loop
            config = uvicorn.Config(
                app,
                host=node.host,
                port=node.port,
                log_level="warning",
                loop="asyncio",
            )
            server = uvicorn.Server(config)
            node.get_logger().info(f"ui backend fastapi listening: http://{node.host}:{node.port}")
            loop.run_until_complete(server.serve())

        self._server_thread = threading.Thread(target=_run, name="camrod_ui_fastapi", daemon=True)
        self._server_thread.start()


def main() -> None:
    rclpy.init()
    node = UiBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
