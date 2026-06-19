#!/usr/bin/env python3
# HH_260421: UI backend simplified to direct destination-driven engage/goal dispatch.
# HH_260520: Migrated HTTP server to FastAPI+uvicorn with WebSocket support.
#            Added /battery_percentage and /AMR_service_state sub/pub.

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
from avg_msgs.msg import AvgAmrServiceState, PlanningMissionKey, UiDestinationCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

import uvicorn

from camrod_ui.api_common import to_diag_level_int


@dataclass
class ApiState:
    """In-memory snapshot exposed by the HTTP/WebSocket API."""

    engaged: bool = False
    ready: bool = False
    operation_mode: str = "STOP"
    ready_message: str = ""
    module_states: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics: List[Dict[str, Any]] = field(default_factory=list)
    diagnostics_agg_count: int = 0
    diagnostics_agg_error_count: int = 0
    destination: Dict[str, Any] = field(
        default_factory=lambda: {"site": "", "run": False}
    )
    battery_percentage: int = -1
    ws_site_states: Dict[str, bool] = field(default_factory=dict)


@dataclass
class MissionKeypoint:
    """Resolved keypoint for destination dispatch."""

    key: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw_deg: float


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
        self.planning_mission_key_topic = str(
            self.declare_parameter("planning_mission_key_topic", "/planning/mission_key").value
        )
        self.planning_goal_pose_topic = str(
            self.declare_parameter("planning_goal_pose_topic", "/goal_pose").value
        )
        self.battery_topic = str(
            self.declare_parameter("battery_topic", "/battery_percentage").value
        )
        self.amr_service_state_topic = str(
            self.declare_parameter("amr_service_state_topic", "/AMR_service_state").value
        )
        self.publish_mission_key = bool(self.declare_parameter("publish_mission_key", True).value)
        self.publish_goal_pose = bool(self.declare_parameter("publish_goal_pose", True).value)
        self.publish_engage_from_destination = bool(
            self.declare_parameter("publish_engage_from_destination", True).value
        )
        self.default_goal_frame_id = str(self.declare_parameter("default_goal_frame_id", "map").value)
        self.fallback_mission_key = str(self.declare_parameter("fallback_mission_key", "camping_site_1").value)
        self.fallback_to_first_known_goal = bool(
            self.declare_parameter("fallback_to_first_known_goal", True).value
        )

        self.camping_sites_yaml = str(self.declare_parameter("camping_sites_yaml", "").value)
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
            ws_site_states={s: False for s in self.site_names}
        )

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
        self.sub_battery = self.create_subscription(
            Int32,
            self.battery_topic,
            self._on_battery,
            10,
        )
        self.sub_amr_service_state = self.create_subscription(
            AvgAmrServiceState,
            self.amr_service_state_topic,
            self._on_amr_service_state,
            10,
        )

        # Publishers.
        # HH_260617: UI destination and planning mission-key topics now use
        # generated avg_msgs semantic messages instead of JSON/String wrappers.
        self.pub_destination = self.create_publisher(
            UiDestinationCommand, self.ui_destination_topic, 10
        )
        self.pub_engage = self.create_publisher(Bool, self.planning_engage_topic, 10)
        self.pub_mission_key = self.create_publisher(
            PlanningMissionKey, self.planning_mission_key_topic, 10
        )
        self.pub_goal_pose = self.create_publisher(PoseStamped, self.planning_goal_pose_topic, 10)
        self.pub_amr_service_state = self.create_publisher(AvgAmrServiceState, self.amr_service_state_topic, 10)

        self._server_thread: Optional[threading.Thread] = None
        if self.enable_http_server:
            self._start_fastapi_server()

        self.get_logger().info(
            "ui_backend ready: "
            f"host={self.host} port={self.port} "
            f"frontend_dir={str(self.frontend_dir) if self.frontend_dir else '(builtin)'} "
            f"destination_topic={self.ui_destination_topic} "
            f"engage_topic={self.planning_engage_topic} "
            f"mission_key_topic={self.planning_mission_key_topic} "
            f"goal_pose_topic={self.planning_goal_pose_topic} "
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
            keypoints[key] = MissionKeypoint(
                key=key,
                frame_id=str(site.get("frame_id", self.default_goal_frame_id)).strip()
                or self.default_goal_frame_id,
                x=float(site.get("x", 0.0)),
                y=float(site.get("y", 0.0)),
                z=float(site.get("z", 0.0)),
                yaw_deg=float(site.get("yaw_deg", 0.0)),
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
        error_level = to_diag_level_int(DiagnosticStatus.ERROR)

        for status in msg.status:
            level = to_diag_level_int(status.level)
            module = self._extract_module_name(status)
            prev = module_levels.get(module, -1)
            if level >= prev:
                module_levels[module] = level
                module_messages[module] = status.message

            if level >= error_level:
                error_count += 1

            diagnostics.append(
                {
                    "name": status.name,
                    "module": module,
                    "level": level,
                    "message": status.message,
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

        ready = bool(msg.status) and (error_count == 0)
        if not msg.status:
            ready_message = "no diagnostics yet"
        elif error_count > 0:
            ready_message = f"diagnostics errors: {error_count}"
        else:
            ready_message = "ready"

        with self._lock:
            self._state.diagnostics = diagnostics
            self._state.module_states = module_states
            self._state.diagnostics_agg_count = len(msg.status)
            self._state.diagnostics_agg_error_count = error_count
            self._state.ready = ready
            self._state.ready_message = ready_message
            self._state.operation_mode = self._compute_operation_mode(
                self._state.engaged,
                self._state.ready,
            )

    def _on_battery(self, msg: Int32) -> None:
        pct = max(0, min(100, int(msg.data)))
        with self._lock:
            self._state.battery_percentage = pct
        self._schedule_broadcast({"battery": pct})

    def _on_amr_service_state(self, msg: AvgAmrServiceState) -> None:
        state = int(msg.state)
        self.get_logger().info(
            f"AMR service state received: {state} ({msg.description})"
        )
        if state == AvgAmrServiceState.SITE_ARRIVED:
            with self._lock:
                site = self._state.destination.get("site", "")
            if site:
                self._schedule_broadcast({"arrived": site})
            self._publish_engage(False, source="amr_service_state:SITE_ARRIVED")
        elif state == AvgAmrServiceState.DROP_ZONE_WAIT:
            self._schedule_broadcast({"amr_state": 0})
        elif state == AvgAmrServiceState.GUEST_RECALL_SERVICE:
            # HJ_260601: Notify robot UI that guest requested a recall.
            self._schedule_broadcast({"guest_recall": True})

    def _publish_amr_service_state(self, state: int, source: str) -> None:
        desc_map = {
            AvgAmrServiceState.DROP_ZONE_WAIT:         "Drop Zone 대기 중",
            AvgAmrServiceState.MOVING_TO_SITE:         "Drop Zone → Site 이동 중",
            AvgAmrServiceState.SITE_ARRIVED:           "Site 도착",
            AvgAmrServiceState.RETURNING_TO_DROP_ZONE: "Site → Drop Zone 복귀 중",
            AvgAmrServiceState.GUEST_RECALL_SERVICE:   "Guest 호출 요청",
        }
        msg = AvgAmrServiceState()
        msg.state = state
        msg.description = desc_map.get(state, f"unknown state {state}")
        self.pub_amr_service_state.publish(msg)
        self.get_logger().info(
            f"AMR service state ({source}) -> {self.amr_service_state_topic}: "
            f"{state} ({msg.description})"
        )

    def _on_destination_command(self, msg: UiDestinationCommand) -> None:
        site = str(msg.site).strip()
        if not site:
            self.get_logger().warn("destination command has empty site")
            return
        run = bool(msg.run)
        result = self._apply_destination_command(site=site, run=run, source="destination_topic")
        self.get_logger().info(
            "destination dispatch summary: "
            f"site={site} run={str(run).lower()} "
            f"mission_key={result.get('mission_key', '')} "
            f"site_goal={str(bool(result.get('goal_pose_published', False))).lower()}"
        )

    # ── Goal and engage publishing ────────────────────────────────────────────

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

    def _publish_engage(self, enabled: bool, source: str) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_engage.publish(msg)

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

    def _publish_goal_for_site(self, site: str, source: str) -> Dict[str, Any]:
        mission_key = self._resolve_mission_key_for_site(site)
        if not mission_key:
            return {
                "mission_key": "",
                "goal_pose_published": False,
                "message": f"no mission key resolved for site={site}",
            }

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
                f"mission_key={mission_key} site={site} xy=({goal.x:.2f},{goal.y:.2f})"
            )

        if self.publish_goal_pose and goal is None:
            self.get_logger().warn(
                f"site goal dispatch skipped: mission key '{mission_key}' is missing in camping_sites_yaml"
            )

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
        if self.publish_engage_from_destination:
            self._publish_engage(run, source=f"{source}:destination")

        if not run:
            return {
                "site": site,
                "run": False,
                "mission_key": "",
                "goal_pose_published": False,
                "message": "run=false -> engage off, goal dispatch skipped",
            }

        self._publish_amr_service_state(AvgAmrServiceState.MOVING_TO_SITE, source=f"{source}:start")
        goal_result = self._publish_goal_for_site(site=site, source=source)
        return {
            "site": site,
            "run": True,
            "mission_key": goal_result.get("mission_key", ""),
            "goal_pose_published": bool(goal_result.get("goal_pose_published", False)),
            "message": str(goal_result.get("message", "ok")),
        }

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
                "operation_mode": self._state.operation_mode,
                "ready_message": self._state.ready_message,
                "module_states": list(self._state.module_states),
                "diagnostics": list(self._state.diagnostics),
                "diagnostics_agg_count": self._state.diagnostics_agg_count,
                "diagnostics_agg_error_count": self._state.diagnostics_agg_error_count,
                "destination": dict(self._state.destination),
                "battery_percentage": self._state.battery_percentage,
            }

    # ── Public API methods (called by HTTP handlers) ──────────────────────────

    def set_engage(self, value: bool) -> Dict[str, Any]:
        self._publish_engage(bool(value), source="http")
        self._schedule_broadcast({"engage": bool(value)})
        return {"success": True, "message": "engage command published", "value": bool(value)}

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

        with self._lock:
            self._state.ws_site_states = {s: (s == normalized_site and run) for s in self.site_names}
        self._schedule_broadcast({
            "states": {s: (s == normalized_site and run) for s in self.site_names}
        })

        payload = self._publish_destination_command(
            site=normalized_site,
            run=bool(run),
            source="http_ui_destination",
        )
        return {"success": True, "destination": payload}

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
            await ws.send_json({"states": states})
            await ws.send_json({"engage": engage})
            if battery >= 0:
                await ws.send_json({"battery": battery})

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
                            node._publish_engage(False, source="ws_toggle_off")
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
                        node._publish_amr_service_state(AvgAmrServiceState.RETURNING_TO_DROP_ZONE, source="ws:usage_complete")
                        node._publish_engage(False, source="ws:usage_complete")
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
