#!/usr/bin/env python3
"""Live CAMROD UI simulator.

The node publishes the minimum production ROS contracts required by camrod_ui,
observes commands emitted by the real backend, and exposes an intentionally
small HTTP control surface for interactive fault and lifecycle injection.
"""

from __future__ import annotations

import asyncio
import copy
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, Optional

import rclpy
import uvicorn
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from avg_msgs.msg import (
    AvgBool,
    AvgLocalizationMode,
    AvgPlatformStatus,
    AvgPoseStamped,
    AvgServiceState,
    ModuleState,
    MotionOperation,
    PlanningMissionKey,
    PlanningRecallRequest,
    PlanningState,
    SystemStatus,
    UiDestinationCommand,
)

from camrod_ui_tester.simulation_state import (
    PLANNING_STATES,
    SERVICE_STATES,
    Pose2D,
    ScenarioEngine,
    parking_method_for_battery,
)


SERVICE_NAMES_BY_VALUE = {value: name for name, value in SERVICE_STATES.items()}
OPERATION_NAMES = {
    MotionOperation.START: "START",
    MotionOperation.CANCEL: "CANCEL",
    MotionOperation.RETURN: "RETURN",
    MotionOperation.ALIGN_FOR_PARKING: "ALIGN_FOR_PARKING",
    MotionOperation.EXIT: "EXIT",
}


def _yaw_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(float(yaw_deg)) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_from_quaternion(z: float, w: float) -> float:
    return math.degrees(math.atan2(2.0 * float(w) * float(z), 1.0 - 2.0 * float(z) ** 2))


def _clamp_fraction(value: Any) -> float:
    parsed = float(value)
    if parsed > 1.0:
        parsed /= 100.0
    return max(0.0, min(1.0, parsed))


class UiSimulatorNode(Node):
    """Publish a virtual runtime and react to commands from the real UI backend."""

    REQUIRED_MODULES = ("map", "sensing", "localization", "planning", "control", "platform", "system")

    def __init__(self) -> None:
        super().__init__("ui_simulator")

        self.control_host = str(self.declare_parameter("control_host", "127.0.0.1").value)
        self.control_port = int(self.declare_parameter("control_port", 8020).value)
        self.mode = str(self.declare_parameter("mode", "closed_loop").value)
        self.initial_scenario = str(self.declare_parameter("initial_scenario", "ready").value)
        self.speed_scale = float(self.declare_parameter("speed_scale", 1.0).value)
        self.route_duration_s = float(self.declare_parameter("route_duration_s", 12.0).value)
        self.maneuver_duration_s = float(self.declare_parameter("maneuver_duration_s", 2.0).value)
        self.publish_rate_hz = max(2.0, min(30.0, float(self.declare_parameter("publish_rate_hz", 10.0).value)))
        self.allow_default_ros_domain = bool(
            self.declare_parameter("allow_default_ros_domain", False).value
        )
        self.camping_sites_yaml = str(self.declare_parameter("camping_sites_yaml", "").value)
        self.drop_zones_yaml = str(self.declare_parameter("drop_zones_yaml", "").value)
        self.scenarios_yaml = str(self.declare_parameter("scenarios_yaml", "").value)
        self.control_panel_dir = str(self.declare_parameter("control_panel_dir", "").value)
        # HH_260908 - A reset restores the simulated world, but the backend
        # separately owns the live mission identity.  Without an explicit
        # operator stop it keeps site/generation active and rejects the
        # post-reset DROP_ZONE_WAIT as an uncorrelated return state, so no
        # service-state frame reaches the UIs and their latched guest
        # notifications never clear.  Empty disables the hook.
        self.backend_stop_url = str(
            self.declare_parameter("backend_stop_url", "http://127.0.0.1:8010/ui/stop").value
        ).strip()
        self.backend_stop_timeout_s = max(
            0.1,
            min(10.0, float(self.declare_parameter("backend_stop_timeout_s", 2.0).value)),
        )

        self._validate_isolated_domain()
        sites = self._load_sites(self.camping_sites_yaml)
        station = self._load_station(self.drop_zones_yaml)
        self._scenario_profiles = self._load_scenarios(self.scenarios_yaml)
        self._lock = threading.RLock()
        self._engine = ScenarioEngine(
            station_pose=station,
            sites=sites,
            mode=self.mode,
            speed_scale=self.speed_scale,
            route_duration_s=self.route_duration_s,
            maneuver_duration_s=self.maneuver_duration_s,
        )
        self._engine.apply_scenario(self.initial_scenario)
        self._events: Deque[dict] = deque(maxlen=500)
        self._signals: Deque[str] = deque()
        self._event_sequence = 0
        self._started_monotonic = time.monotonic()
        self._backend_stop_suppress_until = 0.0
        self._last_tick = time.monotonic()
        self._last_slow_publish = 0.0
        self._last_goal: Optional[PoseStamped] = None
        self._last_path: Optional[NavPath] = None
        self._server: Optional[uvicorn.Server] = None
        self._server_thread: Optional[threading.Thread] = None

        self._state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._service_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._create_publishers()
        self._create_subscriptions()
        self._tf_broadcaster = TransformBroadcaster(self)
        self._navigate_server = ActionServer(
            self,
            NavigateToPose,
            "/planning/navigate_to_pose",
            execute_callback=self._execute_navigate,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self._follow_path_server = ActionServer(
            self,
            FollowPath,
            "/planning/follow_path",
            execute_callback=self._execute_follow_path,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
        )
        self._tick_timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self._start_control_server()
        self._record_event("simulator", "started", {
            "mode": self._engine.state.mode,
            "scenario": self.initial_scenario,
            "domain": os.environ.get("ROS_DOMAIN_ID", "0"),
            "sites": sorted(sites),
        })
        self.get_logger().info(
            "CAMROD UI simulator ready: "
            f"control=http://{self.control_host}:{self.control_port} "
            f"mode={self._engine.state.mode} sites={len(sites)}"
        )

    def _validate_isolated_domain(self) -> None:
        domain = os.environ.get("ROS_DOMAIN_ID", "").strip()
        if domain in {"", "0"} and not self.allow_default_ros_domain:
            raise RuntimeError(
                "ui_simulator refuses ROS_DOMAIN_ID 0/unset; export an isolated "
                "domain (for example ROS_DOMAIN_ID=91) or explicitly set "
                "allow_default_ros_domain:=true"
            )

    @staticmethod
    def _load_sites(path: str) -> Dict[str, Pose2D]:
        source = Path(path).expanduser()
        with source.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        result: Dict[str, Pose2D] = {}
        for index, entry in enumerate(data.get("camping_sites", []), start=1):
            if not isinstance(entry, dict):
                continue
            mission_key = str(entry.get("type", f"camping_site_{index}"))
            suffix = mission_key.removeprefix("camping_site_")
            site = f"B{suffix}"
            result[site] = Pose2D(
                float(entry.get("service_x", entry.get("x", 0.0))),
                float(entry.get("service_y", entry.get("y", 0.0))),
                float(entry.get("service_yaw_deg", entry.get("yaw_deg", 0.0))),
            )
        if not result:
            raise ValueError(f"no camping_sites found in {source}")
        return result

    @staticmethod
    def _load_station(path: str) -> Pose2D:
        source = Path(path).expanduser()
        with source.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        zones = data.get("drop_zones", [])
        if not zones or not isinstance(zones[0], dict):
            raise ValueError(f"no drop_zones found in {source}")
        zone = zones[0]
        return Pose2D(
            float(zone.get("x", 0.0)),
            float(zone.get("y", 0.0)),
            float(zone.get("yaw_deg", 0.0)),
        )

    @staticmethod
    def _load_scenarios(path: str) -> Dict[str, dict]:
        if not path:
            return {}
        with Path(path).expanduser().open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        profiles = data.get("scenarios", {})
        return profiles if isinstance(profiles, dict) else {}

    def _create_publishers(self) -> None:
        self.pub_system = self.create_publisher(SystemStatus, "/system/status", 10)
        self.pub_diagnostics = self.create_publisher(DiagnosticArray, "/system/diagnostics_agg", 10)
        self.pub_planning = self.create_publisher(PlanningState, "/planning/state_machine/state", 10)
        self.pub_goal_source = self.create_publisher(String, "/planning/goal_source", self._state_qos)
        self.pub_engaged = self.create_publisher(AvgBool, "/control/planning_engaged", self._state_qos)
        self.pub_localization = self.create_publisher(AvgLocalizationMode, "/localization/mode", 10)
        self.pub_pose = self.create_publisher(AvgPoseStamped, "/localization/pose", 10)
        self.pub_lanelet_pose = self.create_publisher(AvgPoseStamped, "/planning/lanelet_pose", 10)
        self.pub_gate = self.create_publisher(ModuleState, "/control/cmd_vel_safety_gate/status", self._state_qos)
        self.pub_platform = self.create_publisher(AvgPlatformStatus, "/platform/status", 10)
        self.pub_service = self.create_publisher(AvgServiceState, "/service/state", self._service_qos)
        self.pub_drop_zone_exit = self.create_publisher(AvgBool, "/control/drop_zone/exit_complete", 10)
        self.pub_drop_zone_status = self.create_publisher(ModuleState, "/control/drop_zone_maneuver_controller/status", self._service_qos)
        self.pub_campsite_status = self.create_publisher(ModuleState, "/control/camping_site_maneuver_controller/status", self._service_qos)
        self.pub_parking_status = self.create_publisher(ModuleState, "/parking/status", self._service_qos)
        self.pub_reverse_parking_status = self.create_publisher(ModuleState, "/parking/reverse_parking_controller/status", self._service_qos)
        self.pub_apriltag_parking_status = self.create_publisher(ModuleState, "/parking/apriltag_parking_controller/status", self._service_qos)
        self.pub_route_goal = self.create_publisher(PoseStamped, "/planning/goal_pose_snapped_ros", 10)
        self.pub_global_path = self.create_publisher(NavPath, "/planning/global_path", 10)
        self.pub_local_path = self.create_publisher(NavPath, "/planning/local_path_ros", 10)

    def _create_subscriptions(self) -> None:
        self.create_subscription(UiDestinationCommand, "/ui/selected_destination", self._on_destination, 10)
        self.create_subscription(PlanningMissionKey, "/planning/mission_key", self._on_mission_key, 10)
        self.create_subscription(PoseStamped, "/planning/site_goal_pose_ros", self._on_site_goal, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_manual_goal, 10)
        self.create_subscription(AvgBool, "/planning/engage", self._on_planning_engage, 10)
        self.create_subscription(AvgBool, "/planning/mission_engage", self._on_mission_engage, 10)
        self.create_subscription(AvgBool, "/platform/drive_enable", self._on_drive_enable, 10)
        self.create_subscription(MotionOperation, "/control/drop_zone_maneuver_controller/operation", self._on_drop_zone_operation, 10)
        self.create_subscription(MotionOperation, "/control/camping_site_maneuver_controller/operation", self._on_campsite_operation, 10)
        self.create_subscription(MotionOperation, "/parking/operation", self._on_parking_operation, 10)
        self.create_subscription(PlanningRecallRequest, "/planning/state_machine/return_to_drop_zone", self._on_return_request, 10)
        self.create_subscription(PlanningRecallRequest, "/planning/state_machine/camping_site_recall", self._on_recall_request, 10)
        self.create_subscription(AvgBool, "/platform/headlight/command", self._on_headlight, 10)
        self.create_subscription(AvgServiceState, "/service/state", self._on_service_state, self._service_qos)

    def _request_backend_stop(self, reason: str) -> bool:
        """Release the backend's active mission before restoring the world.

        A reset is an operator-level stop from the backend's point of view:
        it is the only path that clears the active site and generation
        without the matching Return that the mission-identity guard demands.
        """
        if not self.backend_stop_url:
            return False
        # The backend answers this stop by publishing OPERATOR_STOPPED.
        # That echo must not overwrite the state the reset is about to
        # install, so ignore it for the same bounded window the bootstrap
        # barrier already uses.
        self._backend_stop_suppress_until = time.monotonic() + 4.0
        request = urllib.request.Request(self.backend_stop_url, data=b"", method="POST")
        try:
            with urllib.request.urlopen(
                request, timeout=self.backend_stop_timeout_s
            ) as response:
                accepted = 200 <= int(response.status) < 300
        except (urllib.error.URLError, OSError, ValueError) as exc:
            self.get_logger().warn(
                f"backend stop request failed ({reason}); the backend may keep "
                f"the previous mission active: url={self.backend_stop_url} error={exc}"
            )
            self._record_event(
                "control", "backend stop failed",
                {"reason": reason, "error": str(exc)},
            )
            return False
        self._record_event(
            "control", "backend stop requested",
            {"reason": reason, "accepted": accepted},
        )
        return accepted

    def _record_event(self, topic: str, summary: str, detail: Optional[dict] = None) -> None:
        with self._lock:
            self._event_sequence += 1
            self._events.append({
                "sequence": self._event_sequence,
                "wall_time": time.strftime("%H:%M:%S"),
                "sim_time_s": round(self._engine.state.sim_time_s, 3),
                "topic": topic,
                "summary": summary,
                "detail": detail or {},
            })

    def _on_destination(self, msg: UiDestinationCommand) -> None:
        self._record_event("/ui/selected_destination", f"{msg.site} run={msg.run} source={msg.source}")
        with self._lock:
            if msg.run:
                self._engine.select_destination(msg.site, msg.mission_key, msg.source)
            elif self._engine.state.active_site == str(msg.site).strip().upper():
                self._engine.state.active_site = ""

    def _on_mission_key(self, msg: PlanningMissionKey) -> None:
        self._record_event("/planning/mission_key", msg.mission_key, {"source": msg.source})
        with self._lock:
            self._engine.state.active_mission_key = msg.mission_key

    def _pose_from_message(self, msg: PoseStamped) -> Pose2D:
        return Pose2D(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            _yaw_from_quaternion(msg.pose.orientation.z, msg.pose.orientation.w),
        )

    def _on_site_goal(self, msg: PoseStamped) -> None:
        pose = self._pose_from_message(msg)
        self._record_event("/planning/site_goal_pose_ros", f"x={pose.x:.2f} y={pose.y:.2f}")
        with self._lock:
            self._last_goal = copy.deepcopy(msg)
            self._last_path = self._make_path(self._engine.state.pose, pose)
            self._engine.receive_goal(pose, source="ui_site_goal")
        self.pub_route_goal.publish(copy.deepcopy(msg))

    def _on_manual_goal(self, msg: PoseStamped) -> None:
        pose = self._pose_from_message(msg)
        self._record_event("/goal_pose", f"manual x={pose.x:.2f} y={pose.y:.2f}")
        with self._lock:
            self._last_goal = copy.deepcopy(msg)
            self._last_path = self._make_path(self._engine.state.pose, pose)
            self._engine.receive_goal(pose, source="manual_ui")
        self.pub_route_goal.publish(copy.deepcopy(msg))

    def _on_planning_engage(self, msg: AvgBool) -> None:
        self._record_event("/planning/engage", str(bool(msg.data)).lower())
        with self._lock:
            self._engine.set_engaged(planning=bool(msg.data))

    def _on_mission_engage(self, msg: AvgBool) -> None:
        self._record_event("/planning/mission_engage", str(bool(msg.data)).lower())
        with self._lock:
            self._engine.set_engaged(mission=bool(msg.data))

    def _on_drive_enable(self, msg: AvgBool) -> None:
        self._record_event("/platform/drive_enable", str(bool(msg.data)).lower())
        with self._lock:
            self._engine.set_drive_enabled(bool(msg.data))
            can_start_site_goal = self._engine.state.service_state in {
                "DROP_ZONE_WAIT",
                "DEPARTING_CHARGER",
                "DEPARTING_DROP_ZONE",
                "MOVING_TO_SITE",
            }
            if (
                bool(msg.data)
                and can_start_site_goal
                and self._last_goal is not None
                and self._engine.state.motion is None
            ):
                self._engine.receive_goal(self._pose_from_message(self._last_goal), source="ui_goal")

    def _on_drop_zone_operation(self, msg: MotionOperation) -> None:
        operation = OPERATION_NAMES.get(int(msg.operation), str(int(msg.operation)))
        self._record_event("/control/drop_zone_maneuver_controller/operation", operation, {"source": msg.source})
        with self._lock:
            if msg.operation == MotionOperation.EXIT:
                self._engine.begin_departure()
            elif msg.operation == MotionOperation.CANCEL:
                self._engine.state.velocity_mps = 0.0
            elif msg.operation == MotionOperation.ALIGN_FOR_PARKING:
                self._engine.state.service_state = "DROP_ZONE_PARKING"

    def _on_campsite_operation(self, msg: MotionOperation) -> None:
        operation = OPERATION_NAMES.get(int(msg.operation), str(int(msg.operation)))
        self._record_event("/control/camping_site_maneuver_controller/operation", operation, {"source": msg.source})
        with self._lock:
            if msg.operation in {MotionOperation.RETURN, MotionOperation.EXIT}:
                # HH_260908 - A guest recall completes in two authorized
                # commands: the first runs the in-site clearance turn and
                # arms RECALL_RETURN_WAIT; only the confirmed second one
                # releases the exit and the cargo return.
                if not self._engine.begin_recall_site_turn():
                    self._engine.state.service_state = "RETURN_WITH_CARGO"
                    if self._engine.state.mode == "closed_loop":
                        self._engine.begin_return()
            elif msg.operation == MotionOperation.CANCEL:
                self._engine.state.velocity_mps = 0.0

    def _on_parking_operation(self, msg: MotionOperation) -> None:
        operation = OPERATION_NAMES.get(int(msg.operation), str(int(msg.operation)))
        self._record_event("/parking/operation", operation, {"source": msg.source})
        with self._lock:
            if msg.operation == MotionOperation.START:
                self._engine.state.service_state = "DROP_ZONE_PARKING"
            elif msg.operation == MotionOperation.CANCEL and self._engine.state.service_state == "DROP_ZONE_PARKING":
                self._engine.state.service_state = "DROP_ZONE_WAIT"

    def _on_return_request(self, msg: PlanningRecallRequest) -> None:
        self._record_event("/planning/state_machine/return_to_drop_zone", msg.site_name or "drop_zone", {"source": msg.source})
        with self._lock:
            self._engine.begin_return(msg.site_name)

    def _on_recall_request(self, msg: PlanningRecallRequest) -> None:
        self._record_event("/planning/state_machine/camping_site_recall", msg.site_name, {"source": msg.source})
        with self._lock:
            site = str(msg.site_name).strip().upper()
            if site.startswith("CAMPING_SITE_"):
                site = "B" + site.removeprefix("CAMPING_SITE_")
            self._engine.begin_recall(site)

    def _on_headlight(self, msg: AvgBool) -> None:
        self._record_event("/platform/headlight/command", str(bool(msg.data)).lower())
        with self._lock:
            self._engine.state.headlight = bool(msg.data)

    def _on_service_state(self, msg: AvgServiceState) -> None:
        name = SERVICE_NAMES_BY_VALUE.get(int(msg.state))
        if name is None:
            return
        # The production backend deliberately publishes OPERATOR_STOPPED while
        # cancelling stale owners during startup.  It is a bootstrap barrier,
        # not an operator action, so it must not poison the simulator's READY
        # planning snapshot.
        if name == "OPERATOR_STOPPED" and (
            "backend_startup" in str(msg.description)
            or time.monotonic() - self._started_monotonic < 4.0
            or time.monotonic() < self._backend_stop_suppress_until
        ):
            return
        with self._lock:
            current = self._engine.state.service_state
            if name == current:
                return
            self._engine.state.service_state = name
            if name == "OPERATOR_STOPPED":
                self._engine.stop()
        self._record_event("/service/state", name, {"description": msg.description})

    def _tick(self) -> None:
        if not rclpy.ok():
            return
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        with self._lock:
            signal = self._engine.tick(dt)
            self._drain_signals_locked()
            state = copy.deepcopy(self._engine.state)
        if signal == "drop_zone_exit_complete":
            message = AvgBool()
            message.data = True
            self.pub_drop_zone_exit.publish(message)
            self._record_event("simulator", "drop_zone_exit_complete")
        elif signal == "site_arrived":
            self._record_event("simulator", f"arrived {state.active_site}")
        elif signal == "recall_arrived":
            self._record_event("simulator", f"recall arrived {state.active_site}")
        elif signal == "returned_drop_zone":
            self._record_event("simulator", "returned drop zone")
        elif signal == "recall_loading_wait":
            self._record_event("simulator", f"loading wait {state.active_site}")
        elif signal == "parking_complete":
            self._record_event("simulator", "parking complete")

        self._publish_fast_state(state)
        if now - self._last_slow_publish >= 0.5:
            self._last_slow_publish = now
            self._publish_slow_state(state)

    def _drain_signals_locked(self) -> None:
        while self._signals:
            signal = self._signals.popleft()
            if signal == "drop_zone_exit_complete":
                message = AvgBool()
                message.data = True
                self.pub_drop_zone_exit.publish(message)
            elif signal == "arrive":
                self._engine.arrive(self._engine.state.active_site or "B1")
            elif signal == "return":
                self._engine.begin_return()
            elif signal == "parking_complete":
                self._engine.apply_scenario("parking_complete")

    def _publish_fast_state(self, state: Any) -> None:
        stamp = self.get_clock().now().to_msg()
        pose = self._make_avg_pose(state.pose, stamp)
        self.pub_pose.publish(pose)
        self.pub_lanelet_pose.publish(copy.deepcopy(pose))

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "robot_center_link"
        transform.transform.translation.x = state.pose.x
        transform.transform.translation.y = state.pose.y
        qx, qy, qz, qw = _yaw_quaternion(state.pose.yaw_deg)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)

        platform = AvgPlatformStatus()
        platform.stamp = stamp
        platform.header.stamp = stamp
        platform.header.frame_id = "robot_center_link"
        platform.state.stamp = stamp
        platform.state.module_name = "platform"
        platform.state.level = ModuleState.ERROR if state.platform_error_code else ModuleState.OK
        platform.state.operating_state = "CHARGING" if state.charging else "READY"
        platform.state.message = "simulated platform"
        platform.estop = state.estop
        platform.vehicle_state = 0
        platform.control_mode = 1
        platform.error_code = state.platform_error_code
        platform.battery_voltage = 48.0
        platform.battery_percentage = state.battery_percentage
        platform.battery_current_a = -5.0 if state.charging else 1.5
        platform.battery_temperature_c = 27.0
        platform.battery_power_supply_status = (
            4
            if state.battery_available and state.battery_percentage >= 1.0
            else (1 if state.charging else 2)
        )
        platform.battery_state_available = state.battery_available
        platform.is_charging = state.charging
        platform.motion_mode = 1 if state.velocity_mps > 0.001 else 0
        platform.velocity.header.stamp = stamp
        platform.velocity.header.frame_id = "robot_center_link"
        platform.velocity.twist.linear.x = state.velocity_mps
        self.pub_platform.publish(platform)

        if self._last_path is not None:
            path = copy.deepcopy(self._last_path)
            path.header.stamp = stamp
            self.pub_global_path.publish(path)
            self.pub_local_path.publish(copy.deepcopy(path))

    def _publish_slow_state(self, state: Any) -> None:
        stamp = self.get_clock().now().to_msg()
        system = SystemStatus()
        system.stamp = stamp
        system.system_ok = not state.system_error
        system.message = "simulated system healthy" if system.system_ok else "simulated system fault"
        for name in self.REQUIRED_MODULES:
            module = ModuleState()
            module.stamp = stamp
            module.module_name = name
            module.level = ModuleState.ERROR if state.system_error and name == "system" else ModuleState.OK
            module.operating_state = "ERROR" if module.level == ModuleState.ERROR else "READY"
            module.message = "simulated"
            system.modules.append(module)
        self.pub_system.publish(system)

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = stamp
        status = DiagnosticStatus()
        status.name = "/camrod_ui_tester/runtime"
        status.hardware_id = "system"
        status.level = DiagnosticStatus.ERROR if state.system_error else DiagnosticStatus.OK
        status.message = system.message
        diagnostics.status.append(status)
        self.pub_diagnostics.publish(diagnostics)

        planning = PlanningState()
        planning.header.stamp = stamp
        planning.header.frame_id = "map"
        planning.state = PLANNING_STATES[state.planning_state]
        planning.label = state.planning_state
        planning.scenario_id = 1
        planning.scenario_label = "UI_SIMULATOR"
        planning.active_mission_key = state.active_mission_key
        planning.active_goal_source = state.goal_source
        planning.estop = state.estop
        planning.return_requested = state.planning_state == "RETURNING"
        planning.recall_requested = state.service_state == "GUEST_RECALL_SERVICE"
        self.pub_planning.publish(planning)

        localization = AvgLocalizationMode()
        localization.value = state.localization_mode
        localization.label = "NORMAL" if state.localization_mode == 0 else "INVALID"
        self.pub_localization.publish(localization)

        engaged = AvgBool()
        engaged.data = bool(state.planning_engaged or state.mission_engaged)
        self.pub_engaged.publish(engaged)

        goal_source = String()
        goal_source.data = state.goal_source
        self.pub_goal_source.publish(goal_source)

        gate = self._module_state(
            "cmd_vel_safety_gate",
            "SAFETY_HOLD" if state.safety_hold or state.estop or state.platform_error_code else state.gate_state,
            ModuleState.ERROR if state.estop or state.platform_error_code else ModuleState.OK,
            stamp,
        )
        self.pub_gate.publish(gate)

        service = AvgServiceState()
        service.state = SERVICE_STATES[state.service_state]
        service.state_name = state.service_state
        service.description = (
            f"simulator:{state.workflow_type}:{state.workflow_phase}"
            if state.workflow_type
            else f"simulator:{state.scenario}"
        )
        self.pub_service.publish(service)

        dz_state = "EXIT_STRAIGHT" if state.motion and state.motion.kind == "drop_zone_exit" else "IDLE"
        self.pub_drop_zone_status.publish(self._module_state("drop_zone_maneuver_controller", dz_state, ModuleState.OK, stamp))

        campsite_state = "IDLE"
        if state.service_state == "SITE_ENTRY":
            campsite_state = "SITE_ENTRY"
        elif state.service_state == "GUEST_LOADING_WAIT" and state.recall_turn_complete:
            campsite_state = "RECALL_RETURN_WAIT"
        campsite_status = self._module_state("camping_site_maneuver_controller", campsite_state, ModuleState.OK, stamp)
        campsite_status.message = f"site={state.active_site} simulated"
        self.pub_campsite_status.publish(campsite_status)

        if state.workflow_phase == "docking_complete":
            parking_state = "PARKED"
        elif state.service_state == "DROP_ZONE_PARKING":
            parking_state = "ACTIVE"
        else:
            parking_state = "IDLE"
        parking = self._module_state("parking", parking_state, ModuleState.OK, stamp)
        parking_method = parking_method_for_battery(
            state.battery_percentage,
            available=state.battery_available,
        )
        parking.message = (
            f"parking_method={parking_method} "
            f"battery_percent={state.battery_percentage * 100.0:.1f} simulated"
        )
        self.pub_parking_status.publish(parking)
        self.pub_reverse_parking_status.publish(self._module_state("reverse_parking", parking_state, ModuleState.OK, stamp))
        self.pub_apriltag_parking_status.publish(self._module_state("apriltag_parking", parking_state, ModuleState.OK, stamp))

    def _module_state(self, name: str, operating_state: str, level: int, stamp: Any) -> ModuleState:
        message = ModuleState()
        message.stamp = stamp
        message.module_name = name
        message.level = level
        message.operating_state = operating_state
        message.message = "simulated"
        return message

    def _make_avg_pose(self, pose: Pose2D, stamp: Any) -> AvgPoseStamped:
        message = AvgPoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        qx, qy, qz, qw = _yaw_quaternion(pose.yaw_deg)
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        return message

    def _make_path(self, start: Pose2D, target: Pose2D) -> NavPath:
        path = NavPath()
        path.header.frame_id = "map"
        for index in range(31):
            ratio = index / 30.0
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = start.x + (target.x - start.x) * ratio
            pose.pose.position.y = start.y + (target.y - start.y) * ratio
            qx, qy, qz, qw = _yaw_quaternion(start.yaw_deg + (target.yaw_deg - start.yaw_deg) * ratio)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        return path

    def snapshot(self) -> dict:
        with self._lock:
            payload = self._engine.state.snapshot()
            payload["events"] = list(self._events)[-30:]
            payload["available_sites"] = sorted(self._engine.sites)
            payload["available_scenarios"] = sorted(set(self._builtin_scenarios()) | set(self._scenario_profiles))
            return payload

    @staticmethod
    def _builtin_scenarios() -> tuple[str, ...]:
        return (
            "ready", "low_battery", "urgent_battery", "charging",
            "safety_hold", "estop", "platform_fault", "localization_lost",
            "clear_faults", "loading_wait", "returned_drop_zone", "parking_complete",
        )

    RESET_SCENARIOS = ("reset", "ready", "boot")

    def apply_scenario(self, name: str) -> dict:
        with self._lock:
            profile = self._scenario_profiles.get(name, {})
            base = str(profile.get("base", name)) if isinstance(profile, dict) else name
        if base in self.RESET_SCENARIOS:
            self._request_backend_stop(f"scenario={name}")
        with self._lock:
            self._engine.apply_scenario(base)
            if isinstance(profile, dict):
                self._apply_patch_locked(profile.get("state", {}))
            result = self._engine.state.snapshot()
        self._record_event("control", f"scenario={name}")
        return result

    def apply_patch(self, values: dict) -> dict:
        with self._lock:
            self._apply_patch_locked(values)
            result = self._engine.state.snapshot()
        self._record_event("control", "state patched", values)
        return result

    def _apply_patch_locked(self, values: Any) -> None:
        if not isinstance(values, dict):
            raise ValueError("state patch must be an object")
        state = self._engine.state
        for key, value in values.items():
            if key in {"battery", "battery_percent", "battery_percentage"}:
                state.battery_percentage = _clamp_fraction(value)
            elif key == "battery_available":
                state.battery_available = bool(value)
            elif key == "charging":
                state.charging = bool(value)
                if state.charging:
                    state.service_state = "CHARGING"
                elif state.service_state == "CHARGING":
                    state.service_state = "DROP_ZONE_WAIT"
            elif key == "estop":
                state.estop = bool(value)
            elif key == "platform_error_code":
                state.platform_error_code = max(0, int(value))
            elif key == "system_error":
                state.system_error = bool(value)
            elif key == "safety_hold":
                state.safety_hold = bool(value)
            elif key == "localization_mode":
                state.localization_mode = max(0, min(3, int(value)))
            elif key == "service_state":
                normalized = str(value).strip().upper()
                if normalized not in SERVICE_STATES:
                    raise ValueError(f"invalid service_state: {value}")
                state.service_state = normalized
            elif key == "planning_state":
                normalized = str(value).strip().upper()
                if normalized not in PLANNING_STATES:
                    raise ValueError(f"invalid planning_state: {value}")
                state.planning_state = normalized
            elif key == "paused":
                state.paused = bool(value)
            elif key == "speed_scale":
                self._engine.set_speed_scale(float(value))
            elif key == "mode":
                self._engine.set_mode(str(value))
            elif key in {"x", "y", "yaw_deg"}:
                setattr(state.pose, key, float(value))
            else:
                raise ValueError(f"unsupported state field: {key}")

        if state.estop or state.platform_error_code or state.safety_hold:
            state.gate_state = "SAFETY_HOLD"
            state.velocity_mps = 0.0
        elif state.planning_engaged or state.mission_engaged:
            state.gate_state = "ENABLED"
        else:
            state.gate_state = "STANDBY"

    def queue_signal(self, signal: str) -> None:
        with self._lock:
            self._signals.append(signal)
        self._record_event("control", f"signal={signal}")

    def set_mode(self, mode: str) -> dict:
        with self._lock:
            self._engine.set_mode(mode)
            return self._engine.state.snapshot()

    def set_speed(self, scale: float) -> dict:
        with self._lock:
            self._engine.set_speed_scale(scale)
            return self._engine.state.snapshot()

    def control_workflow(self, workflow: str, phase: str, site: str) -> dict:
        with self._lock:
            normalized_site = str(site).strip().upper()
            target = self._engine.sites.get(normalized_site)
            if target is not None and phase in {"move_to_site", "return_to_drop_zone"}:
                start = self._engine.state.pose
                route_target = target if phase == "move_to_site" else self._engine.station_pose
                self._last_path = self._make_path(start, route_target)
            self._engine.control_workflow(workflow, normalized_site, phase)
            result = self._engine.state.snapshot()
        self._record_event(
            "control/workflow",
            f"{workflow}:{phase} site={normalized_site}",
        )
        return result

    def _accept_goal(self, _request: Any) -> GoalResponse:
        self._record_event("action", "goal accepted")
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle: Any) -> CancelResponse:
        self._record_event("action", "cancel accepted")
        with self._lock:
            self._engine.stop()
        return CancelResponse.ACCEPT

    async def _execute_navigate(self, goal_handle: Any) -> NavigateToPose.Result:
        goal_handle.succeed()
        return NavigateToPose.Result()

    async def _execute_follow_path(self, goal_handle: Any) -> FollowPath.Result:
        goal_handle.succeed()
        return FollowPath.Result()

    def _start_control_server(self) -> None:
        app = self._build_control_app()
        config = uvicorn.Config(
            app,
            host=self.control_host,
            port=self.control_port,
            log_level="warning",
            access_log=False,
        )
        self._server = uvicorn.Server(config)
        self._server_thread = threading.Thread(
            target=self._server.run,
            name="camrod-ui-simulator-control",
            daemon=True,
        )
        self._server_thread.start()

    def _build_control_app(self) -> FastAPI:
        node = self
        app = FastAPI(title="CAMROD UI Simulator", version="0.1.0")
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=False,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.get("/")
        def index() -> FileResponse:
            path = Path(node.control_panel_dir) / "index.html"
            if not path.is_file():
                raise HTTPException(status_code=404, detail=f"control panel missing: {path}")
            return FileResponse(path, headers={"Cache-Control": "no-store"})

        @app.get("/app.js")
        def javascript() -> FileResponse:
            return FileResponse(Path(node.control_panel_dir) / "app.js", media_type="application/javascript")

        @app.get("/style.css")
        def stylesheet() -> FileResponse:
            return FileResponse(Path(node.control_panel_dir) / "style.css", media_type="text/css")

        @app.get("/api/sim/state")
        def state() -> JSONResponse:
            return JSONResponse(node.snapshot())

        @app.get("/api/sim/events")
        def events() -> JSONResponse:
            with node._lock:
                return JSONResponse({"events": list(node._events)})

        @app.post("/api/sim/reset")
        def reset() -> JSONResponse:
            node._request_backend_stop("reset")
            with node._lock:
                node._engine.reset(ready=True)
            node._record_event("control", "reset")
            return JSONResponse(node.snapshot())

        @app.post("/api/sim/scenario/{name}")
        def scenario(name: str) -> JSONResponse:
            try:
                return JSONResponse(node.apply_scenario(name))
            except (ValueError, TypeError) as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc

        @app.patch("/api/sim/state")
        def patch_state(payload: dict) -> JSONResponse:
            try:
                return JSONResponse(node.apply_patch(payload))
            except (ValueError, TypeError) as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc

        @app.post("/api/sim/mode/{mode}")
        def mode(mode: str) -> JSONResponse:
            return JSONResponse(node.set_mode(mode))

        @app.post("/api/sim/speed/{scale}")
        def speed(scale: float) -> JSONResponse:
            return JSONResponse(node.set_speed(scale))

        @app.post("/api/sim/workflow/{workflow}/{phase}")
        def workflow_step(workflow: str, phase: str, payload: dict) -> JSONResponse:
            try:
                return JSONResponse(
                    node.control_workflow(workflow, phase, str(payload.get("site", "")))
                )
            except (ValueError, TypeError) as exc:
                raise HTTPException(status_code=400, detail=str(exc)) from exc

        @app.post("/api/sim/signal/{signal}")
        def signal(signal: str) -> JSONResponse:
            if signal not in {"drop_zone_exit_complete", "arrive", "return", "parking_complete"}:
                raise HTTPException(status_code=400, detail=f"unknown signal: {signal}")
            node.queue_signal(signal)
            return JSONResponse({"accepted": True, "signal": signal})

        @app.websocket("/ws/sim")
        async def websocket(websocket: WebSocket) -> None:
            await websocket.accept()
            try:
                while True:
                    await websocket.send_text(json.dumps(node.snapshot(), ensure_ascii=False))
                    await asyncio.sleep(0.25)
            except (WebSocketDisconnect, RuntimeError):
                return

        return app

    def destroy_node(self) -> bool:
        if self._server is not None:
            self._server.should_exit = True
        if self._server_thread is not None and self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        self._navigate_server.destroy()
        self._follow_path_server.destroy()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[UiSimulatorNode] = None
    executor: Optional[MultiThreadedExecutor] = None
    try:
        node = UiSimulatorNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
