#!/usr/bin/env python3
# HH_260421: UI backend simplified to direct destination-driven engage/goal dispatch.
# HH_260520: Migrated HTTP server to FastAPI+uvicorn with WebSocket support.
#            Added /battery_percentage and /service/state sub/pub.
# HH_260810 - Add an operator-map manual Goal Pose path so the managed UI can
#             replace RViz for normal field operation while retaining /goal_pose.

from __future__ import annotations

import asyncio
import copy
import io
import json
import math
import os
import struct
import threading
import time
from collections import defaultdict, deque
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
    AvgLocalizationStatus,
    AvgLocalizationMode,
    AvgOccupancyGrid,
    AvgPolygonStamped,
    AvgPlatformStatus,
    AvgPoseStamped,
    AvgString,
    AvgTrackingError,
    AvgTwistStamped,
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
from fastapi.responses import FileResponse, JSONResponse, Response
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image as RosImage, Imu, NavSatFix, PointCloud2, Range
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from ublox_msgs.msg import NavCOV, NavPVT, NavRELPOSNED9
from visualization_msgs.msg import Marker, MarkerArray

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

# HH_260810 - A manual map click may preempt another manual Nav2 goal, but it
# must not steal ownership from a campsite, return, parking, or charger maneuver.
MANUAL_GOAL_BLOCKED_SERVICE_STATES = frozenset({
    AvgServiceState.MOVING_TO_SITE,
    AvgServiceState.SITE_ARRIVED,
    AvgServiceState.RETURNING_TO_DROP_ZONE,
    AvgServiceState.GUEST_RECALL_SERVICE,
    AvgServiceState.SITE_ENTRY,
    AvgServiceState.UNLOAD_WAIT,
    AvgServiceState.RECALL_TO_SITE_ROAD,
    AvgServiceState.GUEST_LOADING_WAIT,
    AvgServiceState.RETURN_WITH_CARGO,
    AvgServiceState.DROP_ZONE_PARKING,
    AvgServiceState.WAITING_FOR_RETURN_REQUEST,
    AvgServiceState.WAITING_FOR_CHARGING,
    AvgServiceState.CHARGING,
    AvgServiceState.DEPARTING_CHARGER,
    AvgServiceState.DEPARTING_DROP_ZONE,
})


# HH_260813 - The occupancy guard exists to stop a dispatch *before* the robot
# enters an occupied campsite. Once entry has begun it must stay quiet: the
# delivery target legitimately holds the guest's tent, and the robot only sees
# that tent up close after it has crabbed in and turned around. Cancelling there
# ended the run at the exact moment the operator was being asked to send it home.
# Currently unused: the whole occupancy guard is disabled in
# _on_campsite_occupancy. Kept here as the contract to restore it with.
OCCUPANCY_CANCEL_BLOCKED_SERVICE_STATES = frozenset({
    AvgServiceState.SITE_ENTRY,
    AvgServiceState.SITE_ARRIVED,
    AvgServiceState.UNLOAD_WAIT,
    AvgServiceState.WAITING_FOR_RETURN_REQUEST,
    AvgServiceState.GUEST_LOADING_WAIT,
    AvgServiceState.RETURNING_TO_DROP_ZONE,
    AvgServiceState.RETURN_WITH_CARGO,
    AvgServiceState.DROP_ZONE_PARKING,
})


# HH_260810 - One bounded JSON contract replaces the separate Tk/RViz operator
# viewers without changing any control or sensor-authority topic.
TELEMETRY_SCHEMA_VERSION = 2
TELEMETRY_VIEWS = frozenset({
    "gnss",
    "proximity",
    "camera",
    "trajectory",
    "perception",
    "safety",
    "all",
})
TELEMETRY_RADAR_CHANNELS = (
    "front1",
    "front2",
    "left1",
    "left2",
    "right1",
    "right2",
    "rear",
)


def _bounded_telemetry_rate_hz(value: Any) -> float:
    """Clamp a browser refresh request to the ARM64-safe runtime envelope."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        parsed = 10.0
    if not math.isfinite(parsed):
        parsed = 10.0
    return min(20.0, max(1.0, parsed))


async def _wait_for_websocket_disconnect(
    ws: WebSocket, on_activity: Optional[Any] = None
) -> None:
    """Consume telemetry heartbeat events until the client disconnects."""
    # HH_260810 - A send-only WebSocket does not reliably observe a browser or
    # network disconnect in uvicorn. Reading the disconnect event in parallel
    # prevents lazy ROS sensor subscriptions from remaining active indefinitely.
    try:
        while True:
            message = await ws.receive()
            if message.get("type") == "websocket.disconnect":
                return
            if message.get("type") == "websocket.receive" and on_activity:
                on_activity()
    except (WebSocketDisconnect, RuntimeError, KeyError):
        return

# HH_260810 - Keep every operator-view source in one reviewable contract. The
# per-view ROS subscriptions remain lazy, so listing a topic here does not add
# idle CPU or DDS traffic on the 8-core ARM64 target.
TELEMETRY_TOPIC_DEFAULTS = {
    "gnss_fix": "/sensing/gnss/ublox_gps_node/fix",
    "gnss_navpvt": "/sensing/gnss/ublox_gps_node/navpvt",
    "gnss_navcov": "/sensing/gnss/navcov",
    "gnss_relpos": "/sensing/gnss/navrelposned",
    "imu": "/sensing/imu/data_ros",
    "localization_status": "/localization/status",
    "tracking_error": "/planning/tracking_error",
    "global_path": "/planning/global_path",
    "local_path": "/planning/local_path_ros",
    "lidar": "/sensing/lidar/points_filtered",
    "lidar_raw": "/sensing/lidar/vanjee/points_raw",
    "obstacle_cloud": "/perception/obstacles",
    "obstacle_boxes": "/perception/lidar/bboxes",
    "front_camera": "/sensing/camera/econ_front/image_rect/compressed",
    "front_camera_raw": "/sensing/camera/econ_front/image_raw",
    "rear_camera": "/sensing/camera/econ_rear/image_raw/compressed",
    "rear_camera_raw": "/sensing/camera/econ_rear/image_raw",
    "map_markers": "/map/markers",
    "lanelet_cost_grid": "/map/cost_grid/lanelet",
    "lidar_cost_grid": "/sensing/cost_grid/lidar",
    "radar_cost_grid": "/sensing/cost_grid/radar",
    "inflation_cost_grid": "/planning/cost_grid/inflation",
    "planning_boundary": "/platform/robot/planning_boundary",
    "robot_markers": "/platform/robot/markers",
    "radar_evidence": "/sensing/radar/obstacle_evidence",
    "obstacle_replan": "/planning/obstacle_replan/status",
}


def _finite_or_none(value: Any) -> Optional[float]:
    """Return a JSON-safe finite float, or None for NaN/inf/unset values."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _quaternion_yaw_deg(orientation: Any) -> Optional[float]:
    """Extract ENU yaw in degrees from either ROS or generated quaternion fields."""
    values = [
        _finite_or_none(getattr(orientation, axis, None))
        for axis in ("x", "y", "z", "w")
    ]
    if any(value is None for value in values):
        return None
    x, y, z, w = values
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9:
        return None
    x, y, z, w = (value / norm for value in (x, y, z, w))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(yaw)


def _decimate_xy(points: List[tuple[float, float]], limit: int) -> List[List[float]]:
    """Bound browser payload size while preserving both path endpoints."""
    if not points:
        return []
    target = max(2, int(limit))
    if len(points) <= target:
        return [[float(x), float(y)] for x, y in points]
    step = (len(points) - 1) / float(target - 1)
    return [
        [float(points[min(len(points) - 1, round(index * step))][0]),
         float(points[min(len(points) - 1, round(index * step))][1])]
        for index in range(target)
    ]


def _sample_pointcloud_xy(
    message: PointCloud2,
    *,
    max_points: int,
    max_abs_xy_m: float,
    map_to_local_pose: Optional[tuple[float, float, float]] = None,
) -> List[List[float]]:
    """Read a bounded x/y subset without materializing the complete point cloud."""
    fields = {field.name: field for field in message.fields}
    if "x" not in fields or "y" not in fields or message.point_step <= 0:
        return []
    x_field = fields["x"]
    y_field = fields["y"]
    if x_field.datatype != 7 or y_field.datatype != 7:  # PointField.FLOAT32
        return []

    width = int(message.width)
    height = max(1, int(message.height))
    total = width * height
    if width <= 0 or total <= 0:
        return []
    limit = max(1, int(max_points))
    stride = max(1, math.ceil(total / limit))
    endian = ">" if message.is_bigendian else "<"
    data = memoryview(message.data)
    samples: List[List[float]] = []
    for flat_index in range(0, total, stride):
        row = flat_index // width
        column = flat_index % width
        offset = row * int(message.row_step) + column * int(message.point_step)
        try:
            x = struct.unpack_from(f"{endian}f", data, offset + int(x_field.offset))[0]
            y = struct.unpack_from(f"{endian}f", data, offset + int(y_field.offset))[0]
        except (struct.error, ValueError):
            continue
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if map_to_local_pose is not None:
            origin_x, origin_y, yaw_rad = map_to_local_pose
            delta_x = x - origin_x
            delta_y = y - origin_y
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            x = cos_yaw * delta_x + sin_yaw * delta_y
            y = -sin_yaw * delta_x + cos_yaw * delta_y
        if abs(x) > max_abs_xy_m or abs(y) > max_abs_xy_m:
            continue
        samples.append([round(float(x), 3), round(float(y), 3)])
        if len(samples) >= limit:
            break
    return samples


def _sample_occupancy_grid(
    message: Any,
    *,
    max_cells: int,
    minimum_cost: int = 1,
) -> List[List[float]]:
    """Extract a bounded set of occupied map cells with NumPy-backed indexing."""
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    total = width * height
    if width <= 0 or height <= 0 or resolution <= 0.0 or total <= 0:
        return []

    try:
        import numpy as np

        values = np.frombuffer(memoryview(message.data), dtype=np.int8, count=total)
        occupied = np.flatnonzero(values >= int(minimum_cost))
        if occupied.size == 0:
            return []
        limit = max(1, int(max_cells))
        if occupied.size > limit:
            selection = np.linspace(0, occupied.size - 1, limit, dtype=np.int64)
            occupied = occupied[selection]
        rows = occupied // width
        columns = occupied % width
        costs = values[occupied]
    except (ImportError, TypeError, ValueError, BufferError):
        values = list(message.data[:total])
        occupied_list = [
            index for index, value in enumerate(values)
            if int(value) >= int(minimum_cost)
        ]
        if not occupied_list:
            return []
        limit = max(1, int(max_cells))
        step = max(1, math.ceil(len(occupied_list) / limit))
        occupied = occupied_list[::step][:limit]
        rows = [index // width for index in occupied]
        columns = [index % width for index in occupied]
        costs = [values[index] for index in occupied]

    origin = message.info.origin
    yaw_deg = _quaternion_yaw_deg(origin.orientation)
    yaw = math.radians(yaw_deg) if yaw_deg is not None else 0.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    origin_x = float(origin.position.x)
    origin_y = float(origin.position.y)
    samples: List[List[float]] = []
    for row, column, cost in zip(rows, columns, costs):
        local_x = (float(column) + 0.5) * resolution
        local_y = (float(row) + 0.5) * resolution
        x = origin_x + cos_yaw * local_x - sin_yaw * local_y
        y = origin_y + sin_yaw * local_x + cos_yaw * local_y
        samples.append([round(x, 3), round(y, 3), int(cost)])
    return samples


def _marker_array_geometry(
    message: MarkerArray,
    *,
    max_points: int,
) -> Dict[str, Any]:
    """Flatten RViz marker geometry into bounded browser points and outlines."""
    points: List[List[float]] = []
    polylines: List[Dict[str, Any]] = []
    frame_id = "map"
    deleted_all = False
    limit = max(1, int(max_points))

    for marker in message.markers:
        if marker.action == Marker.DELETEALL:
            points = []
            polylines = []
            deleted_all = True
            continue
        if marker.action == Marker.DELETE:
            continue
        if marker.header.frame_id:
            frame_id = marker.header.frame_id
        yaw_deg = _quaternion_yaw_deg(marker.pose.orientation)
        yaw = math.radians(yaw_deg) if yaw_deg is not None else 0.0
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        tx = float(marker.pose.position.x)
        ty = float(marker.pose.position.y)

        def transform(x: float, y: float) -> List[float]:
            return [
                round(tx + cos_yaw * x - sin_yaw * y, 3),
                round(ty + sin_yaw * x + cos_yaw * y, 3),
            ]

        transformed = [
            transform(float(point.x), float(point.y))
            for point in marker.points
            if math.isfinite(float(point.x)) and math.isfinite(float(point.y))
        ]
        if marker.type in {Marker.POINTS, Marker.SPHERE_LIST, Marker.CUBE_LIST}:
            remaining = limit - len(points)
            points.extend(transformed[:remaining])
        elif marker.type == Marker.CUBE:
            half_x = max(0.0, float(marker.scale.x)) * 0.5
            half_y = max(0.0, float(marker.scale.y)) * 0.5
            if half_x > 0.0 and half_y > 0.0:
                outline = [
                    transform(-half_x, -half_y),
                    transform(half_x, -half_y),
                    transform(half_x, half_y),
                    transform(-half_x, half_y),
                    transform(-half_x, -half_y),
                ]
                polylines.append({"kind": marker.ns, "points": outline})
        elif marker.type == Marker.LINE_STRIP and len(transformed) >= 2:
            polylines.append({
                "kind": marker.ns,
                "points": _decimate_xy(
                    [(point[0], point[1]) for point in transformed],
                    min(300, limit),
                ),
            })
        elif marker.type == Marker.LINE_LIST:
            for index in range(0, len(transformed) - 1, 2):
                polylines.append({
                    "kind": marker.ns,
                    "points": [transformed[index], transformed[index + 1]],
                })
        if len(points) >= limit:
            break

    return {
        "frame_id": frame_id,
        "points": points,
        "polylines": polylines[:limit],
        "point_count": len(points),
        "outline_count": len(polylines),
        "cleared": deleted_all and not points and not polylines,
    }


def _encode_raw_image_jpeg(
    message: RosImage,
    *,
    max_width: int,
    quality: int,
) -> Optional[bytes]:
    """Encode common ROS raw image layouts only for the low-rate UI fallback."""
    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0 or int(message.step) <= 0:
        return None
    encoding = str(message.encoding).lower()
    raw_modes = {
        "rgb8": ("RGB", "RGB"),
        "bgr8": ("RGB", "BGR"),
        "rgba8": ("RGBA", "RGBA"),
        "bgra8": ("RGBA", "BGRA"),
        "mono8": ("L", "L"),
    }
    if encoding not in raw_modes:
        return None
    try:
        from PIL import Image as PillowImage

        mode, raw_mode = raw_modes[encoding]
        image = PillowImage.frombuffer(
            mode,
            (width, height),
            bytes(message.data),
            "raw",
            raw_mode,
            int(message.step),
            1,
        )
        if image.width > max_width:
            resized_height = max(1, round(image.height * max_width / image.width))
            image = image.resize((max_width, resized_height), PillowImage.Resampling.BILINEAR)
        if image.mode not in {"RGB", "L"}:
            image = image.convert("RGB")
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=max(30, min(90, int(quality))))
        return output.getvalue()
    except (ImportError, OSError, ValueError):
        return None


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
        # HH_260810 - Keep arbitrary operator goals on the same raw topic that
        # goal_snapper already treats as the manual/RViz input contract.
        self.manual_goal_pose_topic = str(
            self.declare_parameter("manual_goal_pose_topic", "/goal_pose").value
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
        # HH_260810 - Operator telemetry remains dormant until the authenticated
        # diagnostics modal sends a heartbeat. This avoids permanent camera and
        # point-cloud subscribers on the production Jetson.
        self.enable_operator_telemetry = bool(
            self.declare_parameter("enable_operator_telemetry", True).value
        )
        self.telemetry_session_timeout_s = max(
            5.0,
            float(self.declare_parameter("telemetry_session_timeout_s", 12.0).value),
        )
        # HH_260810 - Stream only the selected bounded view at a browser-smooth
        # rate. The 20 Hz ceiling protects the 8-core ARM64 target from a client
        # requesting unbounded JSON serialization and React redraw work.
        self.telemetry_stream_rate_hz = _bounded_telemetry_rate_hz(
            self.declare_parameter("telemetry_stream_rate_hz", 10.0).value
        )
        self.telemetry_max_path_points = max(
            50,
            int(self.declare_parameter("telemetry_max_path_points", 500).value),
        )
        self.telemetry_max_trace_points = max(
            100,
            int(self.declare_parameter("telemetry_max_trace_points", 800).value),
        )
        self.telemetry_max_lidar_points = max(
            50,
            int(self.declare_parameter("telemetry_max_lidar_points", 480).value),
        )
        self.telemetry_lidar_min_period_s = max(
            0.2,
            float(self.declare_parameter("telemetry_lidar_min_period_s", 0.5).value),
        )
        self.telemetry_lidar_max_abs_xy_m = max(
            2.0,
            float(self.declare_parameter("telemetry_lidar_max_abs_xy_m", 12.0).value),
        )
        self.telemetry_max_map_points = max(
            500,
            int(self.declare_parameter("telemetry_max_map_points", 6000).value),
        )
        self.telemetry_max_grid_cells = max(
            100,
            int(self.declare_parameter("telemetry_max_grid_cells", 800).value),
        )
        self.telemetry_grid_min_period_s = max(
            0.5,
            float(self.declare_parameter("telemetry_grid_min_period_s", 1.0).value),
        )
        self.telemetry_camera_min_period_s = max(
            0.1,
            float(self.declare_parameter("telemetry_camera_min_period_s", 0.1).value),
        )
        self.telemetry_camera_fallback_max_width = max(
            320,
            int(self.declare_parameter(
                "telemetry_camera_fallback_max_width", 960
            ).value),
        )
        self.telemetry_camera_fallback_jpeg_quality = min(
            90,
            max(30, int(self.declare_parameter(
                "telemetry_camera_fallback_jpeg_quality", 72
            ).value)),
        )
        self.telemetry_topics = {
            key: str(self.declare_parameter(
                f"telemetry_{key}_topic", default_topic
            ).value)
            for key, default_topic in TELEMETRY_TOPIC_DEFAULTS.items()
        }
        self.telemetry_radar_topic_prefix = str(
            self.declare_parameter(
                "telemetry_radar_topic_prefix", "/sensing/radar"
            ).value
        ).rstrip("/")
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
        self._telemetry = self._new_telemetry_snapshot()
        self._telemetry_source_rx: Dict[str, float] = {}
        self._telemetry_source_history: Dict[str, deque[float]] = defaultdict(
            lambda: deque(maxlen=20)
        )
        self._telemetry_images: Dict[str, Dict[str, Any]] = {
            "front": {"data": b"", "media_type": "image/jpeg", "sequence": 0},
            "rear": {"data": b"", "media_type": "image/jpeg", "sequence": 0},
        }
        self._telemetry_map: Dict[str, Any] = {
            "frame_id": "map",
            "polylines": [],
            "point_count": 0,
        }
        self._telemetry_session_deadline = 0.0
        self._telemetry_requested_view = "all"
        self._telemetry_active_view = ""
        self._telemetry_capture_active = False
        self._telemetry_subscriptions: List[Any] = []
        self._telemetry_last_cloud_decode: Dict[str, float] = defaultdict(float)
        self._telemetry_last_grid_decode: Dict[str, float] = defaultdict(float)
        self._telemetry_last_camera_encode: Dict[str, float] = defaultdict(float)
        self._telemetry_last_trace_sample = 0.0
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
        self.pub_manual_goal_pose = self.create_publisher(
            PoseStamped, self.manual_goal_pose_topic, 10
        )
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
        self._telemetry_session_timer = self.create_timer(
            # HH_260810 - A newly authenticated tab should acquire its lazy ROS
            # subscriptions within one display frame group, not after 500 ms.
            0.1, self._maintain_telemetry_session
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
            f"manual_goal_pose_topic={self.manual_goal_pose_topic} "
            f"arrival_pose_topic={self.arrival_pose_topic} "
            f"campsite_occupancy_topic={self.campsite_occupancy_topic} "
            f"ranger_base_node={self.ranger_base_node_name} "
            f"camping_sites_yaml={self.camping_sites_yaml if self.camping_sites_yaml else '(none)'} "
            f"telemetry_stream_rate_hz={self.telemetry_stream_rate_hz:.1f}"
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

    @staticmethod
    def _normalize_manual_goal(
        x: Any, y: Any, yaw_deg: Any
    ) -> tuple[float, float, float]:
        """Validate a browser-selected map goal and normalize its ENU yaw."""
        try:
            values = (float(x), float(y), float(yaw_deg))
        except (TypeError, ValueError) as exc:
            raise ValueError("manual goal x, y, and yaw_deg must be numbers") from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError("manual goal x, y, and yaw_deg must be finite")
        normalized_yaw = ((values[2] + 180.0) % 360.0) - 180.0
        return values[0], values[1], normalized_yaw

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _new_telemetry_snapshot() -> Dict[str, Any]:
        """Create the stable browser contract before any ROS sample arrives."""
        return {
            "schema_version": TELEMETRY_SCHEMA_VERSION,
            "stream_rate_hz": None,
            "session_active": False,
            "active_view": "",
            "sources": {},
            "gnss": {
                "fix": {},
                "navpvt": {},
                "covariance": {},
                "relative_heading": {},
            },
            "imu": {},
            "radar": {"channels": {}},
            "lidar": {
                "frame_id": "",
                "points": [],
                "point_count": 0,
                "streams": {"filtered": {}, "raw": {}, "obstacles": {}},
            },
            "cameras": {
                "front": {"available": False, "sequence": 0},
                "rear": {"available": False, "sequence": 0},
            },
            "localization": {"pose": {}, "status": {}, "trace": []},
            "motion": {"velocity": {}, "tracking_error": {}},
            "paths": {"global": {}, "local": {}, "maneuvers": {}},
            "footprint": {
                "frame_id": "",
                "points": [],
                "physical_points": [],
                "planning_points": [],
            },
            "perception": {
                "obstacle_cloud": {},
                "obstacle_boxes": {},
                "cost_layers": {
                    "lanelet": {},
                    "lidar": {},
                    "radar": {},
                    "inflation": {},
                },
            },
            "safety": {
                "gate": {},
                "controllers": {},
                "radar_evidence": "",
                "obstacle_replan": "",
            },
            "mission": {},
        }

    def _touch_telemetry_locked(self, source: str, now: Optional[float] = None) -> None:
        received = time.monotonic() if now is None else float(now)
        self._telemetry_source_rx[source] = received
        self._telemetry_source_history[source].append(received)

    def _reset_telemetry_source_timing_locked(self) -> None:
        """Start each leased view with a fresh receive-rate window."""
        # HH_260810 - Do not count time spent unsubscribed as a sensor period;
        # that made healthy 10 Hz streams appear near 0.1 Hz after reopening.
        self._telemetry_source_rx.clear()
        self._telemetry_source_history.clear()

    @staticmethod
    def _normalize_telemetry_view(view: str) -> str:
        normalized = str(view or "all").strip().lower()
        return normalized if normalized in TELEMETRY_VIEWS else "all"

    def _request_telemetry_session(
        self, active: bool, view: str = "all"
    ) -> Dict[str, Any]:
        """Refresh or close the bounded operator telemetry lease."""
        now = time.monotonic()
        requested_view = self._normalize_telemetry_view(view)
        with self._lock:
            if active and self.enable_operator_telemetry:
                self._telemetry_session_deadline = now + self.telemetry_session_timeout_s
                self._telemetry_requested_view = requested_view
            elif (
                requested_view == "all"
                or requested_view == self._telemetry_requested_view
            ):
                # HH_260810 - Ignore a delayed release from the previous tab
                # after a newer tab has already acquired the lease.
                self._telemetry_session_deadline = 0.0
            session_active = self._telemetry_capture_active
            active_view = self._telemetry_active_view
        return {
            "enabled": self.enable_operator_telemetry,
            "requested": bool(active),
            "requested_view": requested_view,
            "active_view": active_view,
            "session_active": session_active,
            "timeout_s": self.telemetry_session_timeout_s,
        }

    def _maintain_telemetry_session(self) -> None:
        now = time.monotonic()
        with self._lock:
            should_be_active = (
                self.enable_operator_telemetry
                and self._telemetry_session_deadline > now
            )
            is_active = self._telemetry_capture_active
            requested_view = self._telemetry_requested_view
            active_view = self._telemetry_active_view
        if should_be_active and is_active and requested_view != active_view:
            self._stop_telemetry_subscriptions()
            self._start_telemetry_subscriptions(requested_view)
            return
        if should_be_active == is_active:
            return
        if should_be_active:
            self._start_telemetry_subscriptions(requested_view)
        else:
            self._stop_telemetry_subscriptions()

    def _start_telemetry_subscriptions(self, view: str = "all") -> None:
        if self._telemetry_subscriptions:
            return
        active_view = self._normalize_telemetry_view(view)
        with self._lock:
            self._telemetry_capture_active = True
            self._telemetry_active_view = active_view
            self._telemetry["session_active"] = True
            self._telemetry["active_view"] = active_view
            self._reset_telemetry_source_timing_locked()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        subscriptions: List[Any] = []

        def wants(*views: str) -> bool:
            return active_view == "all" or active_view in views

        if wants("gnss"):
            subscriptions.extend([
                self.create_subscription(
                    NavSatFix, self.telemetry_topics["gnss_fix"],
                    self._on_telemetry_gnss_fix, sensor_qos,
                ),
                self.create_subscription(
                    NavPVT, self.telemetry_topics["gnss_navpvt"],
                    self._on_telemetry_navpvt, sensor_qos,
                ),
                self.create_subscription(
                    NavCOV, self.telemetry_topics["gnss_navcov"],
                    self._on_telemetry_navcov, sensor_qos,
                ),
                self.create_subscription(
                    NavRELPOSNED9, self.telemetry_topics["gnss_relpos"],
                    self._on_telemetry_relpos, sensor_qos,
                ),
                self.create_subscription(
                    Imu, self.telemetry_topics["imu"],
                    self._on_telemetry_imu, sensor_qos,
                ),
                self.create_subscription(
                    AvgLocalizationStatus,
                    self.telemetry_topics["localization_status"],
                    self._on_telemetry_localization_status, 10,
                ),
            ])

        if wants("proximity"):
            subscriptions.extend([
                self.create_subscription(
                    PointCloud2, self.telemetry_topics["lidar"],
                    lambda message: self._on_telemetry_cloud("filtered", message),
                    sensor_qos,
                ),
                self.create_subscription(
                    PointCloud2, self.telemetry_topics["lidar_raw"],
                    lambda message: self._on_telemetry_cloud("raw", message),
                    sensor_qos,
                ),
            ])
            for channel in TELEMETRY_RADAR_CHANNELS:
                topic = f"{self.telemetry_radar_topic_prefix}/{channel}/range_ros"
                subscriptions.append(self.create_subscription(
                    Range,
                    topic,
                    lambda message, name=channel: self._on_telemetry_radar(
                        name, message
                    ),
                    sensor_qos,
                ))

        if wants("camera"):
            subscriptions.extend([
                self.create_subscription(
                    CompressedImage, self.telemetry_topics["front_camera"],
                    lambda message: self._on_telemetry_camera("front", message),
                    sensor_qos,
                ),
                self.create_subscription(
                    CompressedImage, self.telemetry_topics["rear_camera"],
                    lambda message: self._on_telemetry_camera("rear", message),
                    sensor_qos,
                ),
                self.create_subscription(
                    RosImage, self.telemetry_topics["front_camera_raw"],
                    lambda message: self._on_telemetry_raw_camera("front", message),
                    sensor_qos,
                ),
                self.create_subscription(
                    RosImage, self.telemetry_topics["rear_camera_raw"],
                    lambda message: self._on_telemetry_raw_camera("rear", message),
                    sensor_qos,
                ),
            ])

        if wants("trajectory", "perception"):
            subscriptions.append(self.create_subscription(
                MarkerArray, self.telemetry_topics["map_markers"],
                self._on_telemetry_map, transient_qos,
            ))

        if wants("proximity", "perception"):
            subscriptions.extend([
                self.create_subscription(
                    AvgPolygonStamped, self.telemetry_topics["planning_boundary"],
                    self._on_telemetry_footprint, transient_qos,
                ),
                self.create_subscription(
                    MarkerArray, self.telemetry_topics["robot_markers"],
                    self._on_telemetry_robot_markers, transient_qos,
                ),
            ])

        if wants("trajectory"):
            subscriptions.extend([
                self.create_subscription(
                    AvgTrackingError, self.telemetry_topics["tracking_error"],
                    self._on_telemetry_tracking_error, 10,
                ),
                self.create_subscription(
                    NavPath, self.telemetry_topics["global_path"],
                    lambda message: self._on_telemetry_path("global", message),
                    transient_qos,
                ),
                self.create_subscription(
                    NavPath, self.telemetry_topics["local_path"],
                    lambda message: self._on_telemetry_path("local", message), 10,
                ),
            ])

        if wants("perception"):
            subscriptions.extend([
                self.create_subscription(
                    PointCloud2, self.telemetry_topics["obstacle_cloud"],
                    lambda message: self._on_telemetry_cloud("obstacles", message),
                    sensor_qos,
                ),
                self.create_subscription(
                    MarkerArray, self.telemetry_topics["obstacle_boxes"],
                    self._on_telemetry_obstacle_boxes, sensor_qos,
                ),
            ])
            for layer_name in ("lanelet", "lidar", "radar", "inflation"):
                subscriptions.append(self.create_subscription(
                    AvgOccupancyGrid,
                    self.telemetry_topics[f"{layer_name}_cost_grid"],
                    lambda message, name=layer_name: self._on_telemetry_cost_grid(
                        name, message
                    ),
                    sensor_qos,
                ))

        if wants("safety"):
            subscriptions.extend([
                self.create_subscription(
                    AvgString, self.telemetry_topics["radar_evidence"],
                    lambda message: self._on_telemetry_text(
                        "radar_evidence", message.data
                    ), 10,
                ),
                self.create_subscription(
                    AvgString, self.telemetry_topics["obstacle_replan"],
                    lambda message: self._on_telemetry_text(
                        "obstacle_replan", message.data
                    ), 10,
                ),
            ])

        maneuver_topics = {
            "camping_site": "/control/camping_site_maneuver_controller/path_ros",
            "drop_zone_exit": "/control/drop_zone_maneuver_controller/exit_path_ros",
            "reverse_parking": "/parking/reverse_parking_controller/path_ros",
        }
        if wants("trajectory"):
            for name, topic in maneuver_topics.items():
                subscriptions.append(
                    self.create_subscription(
                        NavPath,
                        topic,
                        lambda message, path_name=name: self._on_telemetry_path(
                            path_name, message
                        ),
                        10,
                    )
                )

        controller_topics = {
            "camping_site": "/control/camping_site_maneuver_controller/status",
            "drop_zone": "/control/drop_zone_maneuver_controller/status",
            "route_recovery": "/control/route_safety_recovery_controller/status",
            "reverse_parking": "/parking/reverse_parking_controller/status",
            "apriltag_parking": "/parking/apriltag_parking_controller/status",
        }
        if wants("safety"):
            for name, topic in controller_topics.items():
                subscriptions.append(
                    self.create_subscription(
                        ModuleState,
                        topic,
                        lambda message, controller=name: self._on_telemetry_controller(
                            controller, message
                        ),
                        10,
                    )
                )

        self._telemetry_subscriptions = subscriptions
        self.get_logger().info(
            f"operator telemetry session active: view={active_view} "
            f"dynamic_subscriptions={len(subscriptions)}"
        )

    def _stop_telemetry_subscriptions(self) -> None:
        subscriptions = self._telemetry_subscriptions
        self._telemetry_subscriptions = []
        for subscription in subscriptions:
            try:
                self.destroy_subscription(subscription)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(
                    f"operator telemetry subscription cleanup ignored: {exc}"
                )
        with self._lock:
            was_active = self._telemetry_capture_active
            self._telemetry_capture_active = False
            self._telemetry_active_view = ""
            self._telemetry["session_active"] = False
            self._telemetry["active_view"] = ""
        if was_active:
            self.get_logger().info(
                "operator telemetry session idle: high-bandwidth subscriptions released"
            )

    @classmethod
    def _prune_telemetry_snapshot(
        cls, snapshot: Dict[str, Any], active_view: str
    ) -> Dict[str, Any]:
        """Keep each bounded browser stream scoped to the selected tab."""
        view = cls._normalize_telemetry_view(active_view) if active_view else ""
        if view == "all":
            return snapshot

        # HH_260810 - Cost grids, traces, and point clouds are bounded at their
        # callbacks, then omitted entirely from unrelated tabs. This avoids
        # repeatedly serializing stale high-volume data on an 8-core ARM host.
        allowed_sections = {
            "gnss": {"gnss", "imu", "localization"},
            "proximity": {"radar", "lidar", "footprint"},
            "camera": {"cameras"},
            "trajectory": {"localization", "motion", "paths", "footprint"},
            "perception": {"localization", "perception", "footprint"},
            "safety": {"safety"},
        }.get(view, set())
        template = cls._new_telemetry_snapshot()
        for section in (
            "gnss", "imu", "radar", "lidar", "cameras", "localization",
            "motion", "paths", "footprint", "perception", "safety",
        ):
            if section not in allowed_sections:
                snapshot[section] = template[section]

        # Only the trajectory tab draws history. GNSS and perception require
        # the latest localization pose but not the accumulated trace array.
        if "localization" in allowed_sections and view != "trajectory":
            snapshot["localization"]["trace"] = []
        return snapshot

    def _snapshot_telemetry(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            snapshot = copy.deepcopy(self._telemetry)
            sources: Dict[str, Dict[str, Any]] = {}
            for source, received in self._telemetry_source_rx.items():
                stamps = self._telemetry_source_history.get(source, ())
                rate_hz = None
                if len(stamps) >= 2 and stamps[-1] > stamps[0]:
                    rate_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
                sources[source] = {
                    "age_s": round(max(0.0, now - received), 3),
                    "rate_hz": round(rate_hz, 2) if rate_hz is not None else None,
                }
            snapshot["sources"] = sources
            snapshot["stream_rate_hz"] = self.telemetry_stream_rate_hz
            snapshot["session_active"] = self._telemetry_capture_active
            snapshot["active_view"] = self._telemetry_active_view
            snapshot["mission"] = {
                "phase": self._state.mission_phase,
                "source": self._state.mission_source,
                "service_state": self._state.service_state,
                "service_state_name": self._state.service_state_name,
                "service_state_description": self._state.service_state_description,
                "engaged": self._state.engaged,
                "ready": self._state.ready,
                "system_health": self._state.system_health,
            }
        snapshot = self._prune_telemetry_snapshot(
            snapshot, snapshot.get("active_view", "")
        )
        snapshot["generated_unix_s"] = round(time.time(), 3)
        return snapshot

    def _snapshot_telemetry_map(self) -> Dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._telemetry_map)

    def _telemetry_camera_response(self, camera: str) -> Response:
        with self._lock:
            image = dict(self._telemetry_images.get(camera, {}))
            received = self._telemetry_source_rx.get(f"camera.{camera}")
        data = image.get("data", b"")
        if not data:
            return Response(status_code=204, headers={"Cache-Control": "no-store"})
        age_s = max(0.0, time.monotonic() - received) if received is not None else -1.0
        return Response(
            content=data,
            media_type=str(image.get("media_type", "image/jpeg")),
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "X-Camrod-Frame-Age": f"{age_s:.3f}",
                "X-Camrod-Frame-Sequence": str(image.get("sequence", 0)),
            },
        )

    def _on_telemetry_gnss_fix(self, message: NavSatFix) -> None:
        covariance = list(message.position_covariance)
        east_std = math.sqrt(max(0.0, covariance[0])) if len(covariance) >= 1 else None
        north_std = math.sqrt(max(0.0, covariance[4])) if len(covariance) >= 5 else None
        up_std = math.sqrt(max(0.0, covariance[8])) if len(covariance) >= 9 else None
        with self._lock:
            self._telemetry["gnss"]["fix"] = {
                "frame_id": message.header.frame_id,
                "status": int(message.status.status),
                "service": int(message.status.service),
                "latitude": _finite_or_none(message.latitude),
                "longitude": _finite_or_none(message.longitude),
                "altitude_m": _finite_or_none(message.altitude),
                "east_std_m": _finite_or_none(east_std),
                "north_std_m": _finite_or_none(north_std),
                "up_std_m": _finite_or_none(up_std),
                "covariance_type": int(message.position_covariance_type),
            }
            self._touch_telemetry_locked("gnss.fix")

    def _on_telemetry_navpvt(self, message: NavPVT) -> None:
        carrier_bits = int(message.flags) & int(NavPVT.FLAGS_CARRIER_PHASE_MASK)
        carrier = {
            int(NavPVT.CARRIER_PHASE_NO_SOLUTION): "NONE",
            int(NavPVT.CARRIER_PHASE_FLOAT): "FLOAT",
            int(NavPVT.CARRIER_PHASE_FIXED): "FIXED",
        }.get(carrier_bits, "UNKNOWN")
        fix_name = {
            int(NavPVT.FIX_TYPE_NO_FIX): "NO_FIX",
            int(NavPVT.FIX_TYPE_DEAD_RECKONING_ONLY): "DR_ONLY",
            int(NavPVT.FIX_TYPE_2D): "2D",
            int(NavPVT.FIX_TYPE_3D): "3D",
            int(NavPVT.FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED): "GNSS_DR",
            int(NavPVT.FIX_TYPE_TIME_ONLY): "TIME_ONLY",
        }.get(int(message.fix_type), "UNKNOWN")
        with self._lock:
            self._telemetry["gnss"]["navpvt"] = {
                "fix_type": int(message.fix_type),
                "fix_name": fix_name,
                "gnss_fix_ok": bool(int(message.flags) & int(NavPVT.FLAGS_GNSS_FIX_OK)),
                "carrier_solution": carrier,
                "satellites": int(message.num_sv),
                "latitude": float(message.lat) * 1.0e-7,
                "longitude": float(message.lon) * 1.0e-7,
                "height_m": float(message.height) * 1.0e-3,
                "horizontal_accuracy_m": float(message.h_acc) * 1.0e-3,
                "vertical_accuracy_m": float(message.v_acc) * 1.0e-3,
                "ground_speed_mps": float(message.g_speed) * 1.0e-3,
                "motion_heading_deg": float(message.heading) * 1.0e-5,
                "vehicle_heading_deg": float(message.head_veh) * 1.0e-5,
                "heading_accuracy_deg": float(message.head_acc) * 1.0e-5,
                "position_dop": float(message.p_dop) * 0.01,
            }
            self._touch_telemetry_locked("gnss.navpvt")

    def _on_telemetry_navcov(self, message: NavCOV) -> None:
        with self._lock:
            self._telemetry["gnss"]["covariance"] = {
                "position_valid": bool(message.pos_cov_valid),
                "velocity_valid": bool(message.vel_cov_valid),
                "north_std_m": math.sqrt(max(0.0, float(message.pos_cov_nn))),
                "east_std_m": math.sqrt(max(0.0, float(message.pos_cov_ee))),
                "down_std_m": math.sqrt(max(0.0, float(message.pos_cov_dd))),
                "velocity_north_std_mps": math.sqrt(max(0.0, float(message.vel_cov_nn))),
                "velocity_east_std_mps": math.sqrt(max(0.0, float(message.vel_cov_ee))),
            }
            self._touch_telemetry_locked("gnss.navcov")

    def _on_telemetry_relpos(self, message: NavRELPOSNED9) -> None:
        flags = int(message.flags)
        carrier_bits = flags & int(NavRELPOSNED9.FLAGS_CARR_SOLN_MASK)
        carrier = {
            int(NavRELPOSNED9.FLAGS_CARR_SOLN_NONE): "NONE",
            int(NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT): "FLOAT",
            int(NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED): "FIXED",
        }.get(carrier_bits, "UNKNOWN")
        with self._lock:
            self._telemetry["gnss"]["relative_heading"] = {
                "valid": bool(flags & int(NavRELPOSNED9.FLAGS_REL_POS_VALID)),
                "heading_valid": bool(
                    flags & int(NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID)
                ),
                "moving": bool(flags & int(NavRELPOSNED9.FLAGS_IS_MOVING)),
                "carrier_solution": carrier,
                "heading_deg": float(message.rel_pos_heading) * 1.0e-5,
                "heading_accuracy_deg": float(message.acc_heading) * 1.0e-5,
                "baseline_m": (
                    float(message.rel_pos_length)
                    + float(message.rel_pos_hp_length) * 0.01
                ) * 0.01,
            }
            self._touch_telemetry_locked("gnss.relpos")

    def _on_telemetry_imu(self, message: Imu) -> None:
        with self._lock:
            self._telemetry["imu"] = {
                "frame_id": message.header.frame_id,
                "yaw_deg": _quaternion_yaw_deg(message.orientation),
                "angular_velocity_rps": {
                    "x": _finite_or_none(message.angular_velocity.x),
                    "y": _finite_or_none(message.angular_velocity.y),
                    "z": _finite_or_none(message.angular_velocity.z),
                },
                "linear_acceleration_mps2": {
                    "x": _finite_or_none(message.linear_acceleration.x),
                    "y": _finite_or_none(message.linear_acceleration.y),
                    "z": _finite_or_none(message.linear_acceleration.z),
                },
                "orientation_covariance": [
                    _finite_or_none(value) for value in message.orientation_covariance
                ],
            }
            self._touch_telemetry_locked("imu")

    def _on_telemetry_localization_status(
        self, message: AvgLocalizationStatus
    ) -> None:
        with self._lock:
            self._telemetry["localization"]["status"] = {
                "mode": int(message.mode.value),
                "mode_label": message.mode.label,
                "confidence": _finite_or_none(message.confidence),
                "gnss_ok": bool(message.gnss_ok),
                "imu_ok": bool(message.imu_ok),
                "wheel_ok": bool(message.wheel_ok),
                "gnss_innovation_norm": _finite_or_none(
                    message.gnss_innovation_norm
                ),
                "wheel_innovation_norm": _finite_or_none(
                    message.wheel_innovation_norm
                ),
            }
            self._touch_telemetry_locked("localization.status")

    def _on_telemetry_tracking_error(self, message: AvgTrackingError) -> None:
        with self._lock:
            self._telemetry["motion"]["tracking_error"] = {
                "local_valid": bool(message.local_valid),
                "local_lateral_m": _finite_or_none(message.local_lateral_deviation),
                "local_heading_deg": _finite_or_none(
                    math.degrees(float(message.local_heading_error))
                ),
                "local_distance_m": _finite_or_none(message.local_distance_to_path),
                "global_valid": bool(message.global_valid),
                "global_lateral_m": _finite_or_none(message.global_lateral_deviation),
                "global_heading_deg": _finite_or_none(
                    math.degrees(float(message.global_heading_error))
                ),
                "active_path_source": message.active_path_source,
                "active_lateral_m": _finite_or_none(message.active_lateral_deviation),
                "active_heading_deg": _finite_or_none(
                    math.degrees(float(message.active_heading_error))
                ),
                "active_distance_m": _finite_or_none(message.active_distance_to_path),
            }
            self._touch_telemetry_locked("planning.tracking_error")

    def _on_telemetry_radar(self, channel: str, message: Range) -> None:
        measured = _finite_or_none(message.range)
        no_target = (
            math.isinf(float(message.range)) and float(message.range) > 0.0
        ) or (
            measured is not None
            and math.isfinite(float(message.max_range))
            and measured > float(message.max_range)
        )
        with self._lock:
            self._telemetry["radar"]["channels"][channel] = {
                "frame_id": message.header.frame_id,
                "range_m": measured,
                "no_target": no_target,
                "min_range_m": _finite_or_none(message.min_range),
                "max_range_m": _finite_or_none(message.max_range),
                "field_of_view_rad": _finite_or_none(message.field_of_view),
            }
            self._touch_telemetry_locked(f"radar.{channel}")

    def _on_telemetry_lidar(self, message: PointCloud2) -> None:
        """Compatibility callback for callers using the original filtered name."""
        self._on_telemetry_cloud("filtered", message)

    def _on_telemetry_cloud(self, stream: str, message: PointCloud2) -> None:
        now = time.monotonic()
        if (
            now - self._telemetry_last_cloud_decode[stream]
            < self.telemetry_lidar_min_period_s
        ):
            return
        self._telemetry_last_cloud_decode[stream] = now
        map_to_local_pose = None
        with self._lock:
            current_pose = dict(self._telemetry["localization"]["pose"])
        if stream != "obstacles" and message.header.frame_id == "map":
            pose_x = _finite_or_none(current_pose.get("x"))
            pose_y = _finite_or_none(current_pose.get("y"))
            pose_yaw = _finite_or_none(current_pose.get("yaw_deg"))
            if pose_x is not None and pose_y is not None and pose_yaw is not None:
                map_to_local_pose = (pose_x, pose_y, math.radians(pose_yaw))
        points = _sample_pointcloud_xy(
            message,
            max_points=self.telemetry_max_lidar_points,
            max_abs_xy_m=self.telemetry_lidar_max_abs_xy_m,
            map_to_local_pose=map_to_local_pose,
        )
        payload = {
            "frame_id": (
                "robot_center_link" if map_to_local_pose is not None
                else message.header.frame_id
            ),
            "source_frame_id": message.header.frame_id,
            "points": points,
            "point_count": int(message.width) * max(1, int(message.height)),
            "sample_count": len(points),
        }
        with self._lock:
            self._telemetry["lidar"]["streams"][stream] = payload
            if stream == "filtered":
                self._telemetry["lidar"].update(payload)
                self._touch_telemetry_locked("lidar", now)
                self._touch_telemetry_locked("lidar.filtered", now)
            elif stream == "raw":
                self._touch_telemetry_locked("lidar.raw", now)
            else:
                self._telemetry["perception"]["obstacle_cloud"] = payload
                self._touch_telemetry_locked("perception.obstacles", now)

    def _on_telemetry_camera(
        self, camera: str, message: CompressedImage
    ) -> None:
        format_name = str(message.format).lower()
        media_type = "image/png" if "png" in format_name else "image/jpeg"
        with self._lock:
            current = self._telemetry_images[camera]
            sequence = int(current.get("sequence", 0)) + 1
            self._telemetry_images[camera] = {
                "data": bytes(message.data),
                "media_type": media_type,
                "sequence": sequence,
                "source": "compressed",
                "received": time.monotonic(),
            }
            self._telemetry["cameras"][camera] = {
                "available": bool(message.data),
                "format": message.format,
                "frame_id": message.header.frame_id,
                "bytes": len(message.data),
                "sequence": sequence,
                "source": "compressed",
                "endpoint": f"/api/camera/{camera}",
            }
            self._touch_telemetry_locked(f"camera.{camera}")

    def _on_telemetry_raw_camera(self, camera: str, message: RosImage) -> None:
        now = time.monotonic()
        if (
            now - self._telemetry_last_camera_encode[camera]
            < self.telemetry_camera_min_period_s
        ):
            return
        with self._lock:
            current = dict(self._telemetry_images.get(camera, {}))
        compressed_received = float(current.get("received", 0.0))
        if current.get("source") == "compressed" and now - compressed_received < 1.5:
            return
        self._telemetry_last_camera_encode[camera] = now
        encoded = _encode_raw_image_jpeg(
            message,
            max_width=self.telemetry_camera_fallback_max_width,
            quality=self.telemetry_camera_fallback_jpeg_quality,
        )
        if not encoded:
            with self._lock:
                self._touch_telemetry_locked(f"camera.{camera}.raw_unsupported", now)
            return
        with self._lock:
            current = self._telemetry_images[camera]
            sequence = int(current.get("sequence", 0)) + 1
            self._telemetry_images[camera] = {
                "data": encoded,
                "media_type": "image/jpeg",
                "sequence": sequence,
                "source": "raw_fallback",
                "received": now,
            }
            self._telemetry["cameras"][camera] = {
                "available": True,
                "format": f"jpeg fallback from {message.encoding}",
                "frame_id": message.header.frame_id,
                "width": int(message.width),
                "height": int(message.height),
                "bytes": len(encoded),
                "sequence": sequence,
                "source": "raw_fallback",
                "endpoint": f"/api/camera/{camera}",
            }
            self._touch_telemetry_locked(f"camera.{camera}", now)
            self._touch_telemetry_locked(f"camera.{camera}.raw", now)

    def _on_telemetry_obstacle_boxes(self, message: MarkerArray) -> None:
        geometry = _marker_array_geometry(
            message,
            max_points=self.telemetry_max_grid_cells,
        )
        with self._lock:
            self._telemetry["perception"]["obstacle_boxes"] = geometry
            self._touch_telemetry_locked("perception.boxes")

    def _on_telemetry_cost_grid(
        self, layer: str, message: AvgOccupancyGrid
    ) -> None:
        now = time.monotonic()
        if (
            now - self._telemetry_last_grid_decode[layer]
            < self.telemetry_grid_min_period_s
        ):
            return
        self._telemetry_last_grid_decode[layer] = now
        cells = _sample_occupancy_grid(
            message,
            max_cells=self.telemetry_max_grid_cells,
        )
        payload = {
            "frame_id": message.header.frame_id,
            "resolution_m": _finite_or_none(message.info.resolution),
            "width": int(message.info.width),
            "height": int(message.info.height),
            "occupied_samples": cells,
            "sample_count": len(cells),
        }
        with self._lock:
            self._telemetry["perception"]["cost_layers"][layer] = payload
            self._touch_telemetry_locked(f"cost.{layer}", now)

    def _on_telemetry_path(self, name: str, message: NavPath) -> None:
        raw_points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
            if math.isfinite(float(pose.pose.position.x))
            and math.isfinite(float(pose.pose.position.y))
        ]
        payload = {
            "frame_id": message.header.frame_id,
            "points": _decimate_xy(raw_points, self.telemetry_max_path_points),
            "raw_point_count": len(raw_points),
        }
        with self._lock:
            if name in {"global", "local"}:
                self._telemetry["paths"][name] = payload
            else:
                self._telemetry["paths"]["maneuvers"][name] = payload
            self._touch_telemetry_locked(f"path.{name}")

    def _on_telemetry_footprint(self, message: AvgPolygonStamped) -> None:
        points = [
            [round(float(point.x), 4), round(float(point.y), 4)]
            for point in message.polygon.points
            if math.isfinite(float(point.x)) and math.isfinite(float(point.y))
        ]
        with self._lock:
            footprint = self._telemetry["footprint"]
            footprint["frame_id"] = message.header.frame_id
            footprint["points"] = points
            footprint["planning_points"] = points
            self._touch_telemetry_locked("platform.planning_boundary")

    def _on_telemetry_robot_markers(self, message: MarkerArray) -> None:
        physical_points: List[List[float]] = []
        planning_points: List[List[float]] = []
        for marker in message.markers:
            if marker.type != Marker.LINE_STRIP or "chassis" not in marker.ns:
                continue
            points = [
                [round(float(point.x), 4), round(float(point.y), 4)]
                for point in marker.points
                if math.isfinite(float(point.x)) and math.isfinite(float(point.y))
            ]
            if len(points) < 3:
                continue
            if float(marker.color.r) > 0.8 and float(marker.color.g) > 0.65:
                planning_points = points
            elif float(marker.color.b) > 0.65:
                physical_points = points
        if not physical_points and not planning_points:
            return
        with self._lock:
            footprint = self._telemetry["footprint"]
            if physical_points:
                footprint["physical_points"] = physical_points
            if planning_points:
                footprint["planning_points"] = planning_points
            self._touch_telemetry_locked("platform.robot_markers")

    def _on_telemetry_controller(
        self, controller: str, message: ModuleState
    ) -> None:
        with self._lock:
            self._telemetry["safety"]["controllers"][controller] = {
                "level": int(message.level),
                "operating_state": message.operating_state,
                "message": message.message,
            }
            self._touch_telemetry_locked(f"controller.{controller}")

    def _on_telemetry_text(self, key: str, value: str) -> None:
        with self._lock:
            self._telemetry["safety"][key] = str(value)
            self._touch_telemetry_locked(f"safety.{key}")

    def _on_telemetry_map(self, message: MarkerArray) -> None:
        polylines: List[Dict[str, Any]] = []
        point_count = 0
        frame_id = "map"
        for marker in message.markers:
            if marker.action == Marker.DELETEALL:
                polylines = []
                point_count = 0
                continue
            if marker.action == Marker.DELETE or marker.type not in {
                Marker.LINE_STRIP,
                Marker.LINE_LIST,
            }:
                continue
            if marker.header.frame_id:
                frame_id = marker.header.frame_id
            yaw = _quaternion_yaw_deg(marker.pose.orientation)
            yaw_rad = math.radians(yaw) if yaw is not None else 0.0
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            tx = float(marker.pose.position.x)
            ty = float(marker.pose.position.y)
            transformed = [
                (
                    tx + cos_yaw * float(point.x) - sin_yaw * float(point.y),
                    ty + sin_yaw * float(point.x) + cos_yaw * float(point.y),
                )
                for point in marker.points
                if math.isfinite(float(point.x)) and math.isfinite(float(point.y))
            ]
            if marker.type == Marker.LINE_STRIP:
                remaining = self.telemetry_max_map_points - point_count
                if remaining < 2:
                    break
                line = _decimate_xy(transformed, min(remaining, 300))
                if len(line) >= 2:
                    polylines.append({"kind": marker.ns, "points": line})
                    point_count += len(line)
            else:
                for index in range(0, len(transformed) - 1, 2):
                    if point_count + 2 > self.telemetry_max_map_points:
                        break
                    first, second = transformed[index], transformed[index + 1]
                    polylines.append({
                        "kind": marker.ns,
                        "points": [
                            [float(first[0]), float(first[1])],
                            [float(second[0]), float(second[1])],
                        ],
                    })
                    point_count += 2
            if point_count >= self.telemetry_max_map_points:
                break
        with self._lock:
            self._telemetry_map = {
                "frame_id": frame_id,
                "polylines": polylines,
                "point_count": point_count,
            }
            self._touch_telemetry_locked("map.markers")

    def _record_telemetry_pose(self, message: AvgPoseStamped) -> None:
        if not self._telemetry_capture_active:
            return
        now = time.monotonic()
        x = float(message.pose.position.x)
        y = float(message.pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return
        pose = {
            "frame_id": message.header.frame_id,
            "x": x,
            "y": y,
            "z": _finite_or_none(message.pose.position.z),
            "yaw_deg": _quaternion_yaw_deg(message.pose.orientation),
        }
        with self._lock:
            self._telemetry["localization"]["pose"] = pose
            self._touch_telemetry_locked("localization.pose", now)
            if now - self._telemetry_last_trace_sample >= 0.25:
                trace = self._telemetry["localization"]["trace"]
                trace.append([round(x, 3), round(y, 3)])
                if len(trace) > self.telemetry_max_trace_points:
                    del trace[:len(trace) - self.telemetry_max_trace_points]
                self._telemetry_last_trace_sample = now

    def _record_telemetry_platform(self, message: AvgPlatformStatus) -> None:
        if not self._telemetry_capture_active:
            return
        vx = float(message.velocity.twist.linear.x)
        vy = float(message.velocity.twist.linear.y)
        speed_mps = math.hypot(vx, vy)
        with self._lock:
            self._telemetry["motion"]["velocity"] = {
                "frame_id": message.velocity.header.frame_id,
                "vx_mps": _finite_or_none(vx),
                "vy_mps": _finite_or_none(vy),
                "speed_mps": _finite_or_none(speed_mps),
                "speed_kph": _finite_or_none(speed_mps * 3.6),
                "yaw_rate_rps": _finite_or_none(message.velocity.twist.angular.z),
                "motion_mode": int(message.motion_mode),
                "control_mode": int(message.control_mode),
                "vehicle_state": int(message.vehicle_state),
                "estop": bool(message.estop),
                "is_charging": bool(message.is_charging),
                "battery_percentage": (
                    _finite_or_none(float(message.battery_percentage) * 100.0)
                    if message.battery_state_available else None
                ),
            }
            self._touch_telemetry_locked("platform.velocity")

    def _record_telemetry_gate(self, message: ModuleState) -> None:
        if not self._telemetry_capture_active:
            return
        with self._lock:
            self._telemetry["safety"]["gate"] = {
                "level": int(message.level),
                "operating_state": message.operating_state,
                "message": message.message,
                "missing_nodes": list(message.missing_nodes),
                "missing_topics": list(message.missing_topics),
            }
            self._touch_telemetry_locked("control.safety_gate")

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

    def _manual_goal_dispatch_block(self) -> Optional[Dict[str, Any]]:
        """Return the authoritative reason a map-selected manual goal cannot start."""
        with self._lock:
            ready = bool(self._state.ready)
            ready_message = str(self._state.ready_message)
            battery_percentage = int(self._state.battery_percentage)
            service_state = int(self._state.service_state)
        if not ready:
            return {
                "error": "system_not_ready",
                "message": ready_message or "system is not ready for a manual goal",
            }
        if service_state in MANUAL_GOAL_BLOCKED_SERVICE_STATES:
            state_name = SERVICE_STATE_NAMES.get(service_state, str(service_state))
            return {
                "error": "service_state_busy",
                "service_state": service_state,
                "service_state_name": state_name,
                "message": f"manual goal is blocked while service state is {state_name}",
            }
        if bool(getattr(self, "_latest_platform_is_charging", False)):
            return {
                "error": "charger_departure_required",
                "message": "manual goal is blocked while charging; use a campsite departure",
            }
        if not self.require_battery_for_mission_dispatch:
            return None
        if battery_percentage >= self.minimum_mission_dispatch_battery_percent:
            return None
        battery_text = "unavailable" if battery_percentage < 0 else f"{battery_percentage}%"
        return {
            "error": "battery_below_mission_minimum",
            "battery_percentage": battery_percentage,
            "minimum_battery_percentage": self.minimum_mission_dispatch_battery_percent,
            "message": (
                f"battery {battery_text} does not satisfy manual-goal minimum "
                f"{self.minimum_mission_dispatch_battery_percent}%"
            ),
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
        if getattr(self, "_telemetry_capture_active", False):
            UiBackendNode._record_telemetry_gate(self, msg)

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
        if getattr(self, "_telemetry_capture_active", False):
            UiBackendNode._record_telemetry_platform(self, msg)
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
        if getattr(self, "_telemetry_capture_active", False):
            UiBackendNode._record_telemetry_pose(self, msg)

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
        # HH_260813 - DISABLED. Every delivery target legitimately holds the
        # guest's tent, so a confirmed tent disabled the very destination the
        # operator needed. Leaving _state.occupied_sites empty keeps
        # _is_site_occupied() false everywhere and broadcasts an empty list, so
        # the frontend toggles stay enabled. Restore this body to re-enable.
        return
        # occupied_keys = {str(key).strip() for key in msg.occupied_mission_keys if key}
        # occupied_sites = sorted(
        #     site
        #     for site in self.site_names
        #     if (self._resolve_mission_key_for_site(site) or "") in occupied_keys
        # )
        # active_occupied_site = ""
        # with self._lock:
        #     occupancy_changed = self._state.occupied_sites != occupied_sites
        #     self._state.occupied_sites = occupied_sites
        #     active_site = str(self._state.destination.get("site", "")).strip()
        #     active_run = bool(self._state.destination.get("run", False))
        #     # The site being serviced right now keeps its selection: the robot
        #     # is already inside it, so deselecting it only hides an active
        #     # mission.
        #     service_state = int(self._state.service_state)
        #     committed = service_state in OCCUPANCY_CANCEL_BLOCKED_SERVICE_STATES
        #     protected_site = active_site if (active_run and committed) else ""
        #     for site in occupied_sites:
        #         if site != protected_site:
        #             self._state.ws_site_states[site] = False
        #     if active_run and active_site in occupied_sites and not committed:
        #         active_occupied_site = active_site
        #         self._state.destination = {"site": active_site, "run": False}
        #
        # if occupancy_changed:
        #     self._schedule_broadcast({"occupied_sites": occupied_sites})
        # if active_occupied_site:
        #     self.get_logger().warn(
        #         f"active campsite became occupied; stopping dispatch: {active_occupied_site}"
        #     )
        #     self._apply_destination_command(
        #         site=active_occupied_site,
        #         run=False,
        #         source="perception_occupancy",
        #     )
        #     self._schedule_broadcast(
        #         {
        #             "states": {s: False for s in self.site_names},
        #             "engage": False,
        #             "error": "campsite_occupied",
        #             "site": active_occupied_site,
        #         }
        #     )

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
        # HH_260813 - Always false while _on_campsite_occupancy stays disabled,
        # so every campsite-occupied gate below it is inert.
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

    def set_manual_goal(self, x: Any, y: Any, yaw_deg: Any) -> Dict[str, Any]:
        """Publish one UI-selected manual goal, then authorize manual motion."""
        try:
            goal_x, goal_y, goal_yaw_deg = self._normalize_manual_goal(
                x, y, yaw_deg
            )
        except ValueError as exc:
            return {
                "success": False,
                "error": "invalid_manual_goal",
                "message": str(exc),
            }

        blocked = self._manual_goal_dispatch_block()
        if blocked is not None:
            return {"success": False, **blocked}

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.default_goal_frame_id
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = self._yaw_deg_to_quaternion(goal_yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # HH_260810 - Clear campsite presentation before publishing the raw goal.
        # goal_snapper remains the sole authority that accepts/snaps /goal_pose.
        with self._lock:
            self._active_mission_site = ""
            self._state.ws_site_states = {site: False for site in self.site_names}
            self._state.destination = {"site": "", "run": False}
        self._update_runtime_state(
            lambda: self._runtime_policy.update_goal_received("manual")
        )
        self.pub_manual_goal_pose.publish(pose)
        self._publish_engage(True, source="http_manual_goal")
        broadcast = {
            "states": {site: False for site in self.site_names},
            "engage": True,
            "manual_goal": {
                "frame_id": self.default_goal_frame_id,
                "x": goal_x,
                "y": goal_y,
                "yaw_deg": goal_yaw_deg,
            },
        }
        self._schedule_broadcast(broadcast)
        self.get_logger().info(
            "manual goal (operator_map) -> "
            f"{self.manual_goal_pose_topic}: frame={self.default_goal_frame_id} "
            f"xy=({goal_x:.3f},{goal_y:.3f}) yaw={goal_yaw_deg:.1f}deg engage=true"
        )
        return {
            "success": True,
            "message": "manual goal published and motion engaged",
            "goal": broadcast["manual_goal"],
            "engaged": True,
        }

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

        # HH_260810 - Dedicated latest-value telemetry transport. Keeping this
        # separate from the mission command socket prevents high-rate operator
        # drawings from delaying safety/service control messages. Each socket
        # renews exactly one lazy ROS subscription view and releases it on exit.
        @app.websocket("/ws/telemetry")
        async def telemetry_websocket(ws: WebSocket) -> None:
            view = node._normalize_telemetry_view(
                str(ws.query_params.get("view", "all"))
            )
            period_s = 1.0 / node.telemetry_stream_rate_hz
            await ws.accept()
            node._request_telemetry_session(True, view)
            disconnect_task = asyncio.create_task(
                _wait_for_websocket_disconnect(
                    ws,
                    lambda: node._request_telemetry_session(True, view),
                )
            )
            try:
                while (
                    not node._server_stop_requested.is_set()
                    and not disconnect_task.done()
                ):
                    await ws.send_json(node._snapshot_telemetry())
                    try:
                        await asyncio.wait_for(
                            asyncio.shield(disconnect_task), timeout=period_s
                        )
                    except asyncio.TimeoutError:
                        pass
            except (WebSocketDisconnect, RuntimeError):
                pass
            finally:
                disconnect_task.cancel()
                try:
                    await disconnect_task
                except asyncio.CancelledError:
                    pass
                node._request_telemetry_session(False, view)

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

        # HH_260810 - The browser renews this lease only while the administrator
        # telemetry modal is open. The ROS timer owns subscription creation and
        # teardown so FastAPI never mutates the node graph from its server thread.
        @app.post("/api/telemetry/session")
        def post_telemetry_session(
            active: bool = True, view: str = "all"
        ) -> JSONResponse:
            return JSONResponse(node._request_telemetry_session(active, view))

        @app.get("/api/telemetry")
        def get_telemetry() -> JSONResponse:
            return JSONResponse(node._snapshot_telemetry())

        @app.get("/api/telemetry/map")
        def get_telemetry_map() -> JSONResponse:
            return JSONResponse(node._snapshot_telemetry_map())

        @app.get("/api/camera/front")
        def get_front_camera() -> Response:
            return node._telemetry_camera_response("front")

        @app.get("/api/camera/rear")
        def get_rear_camera() -> Response:
            return node._telemetry_camera_response("rear")

        @app.get("/ui/platform_tuning")
        async def get_platform_tuning() -> JSONResponse:
            result = await node.get_platform_tuning()
            # HH_260810 - Driver absence is an expected read state in simulation, not an HTTP failure.
            return JSONResponse(result, status_code=200)

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

        # HH_260810 - This is the managed operator-map equivalent of RViz 2D
        # Goal Pose. The backend owns validation and engage ordering; deployment
        # still relies on the trusted robot-LAN boundary documented by camrod_ui.
        @app.post("/ui/manual_goal")
        def post_manual_goal(
            x: float, y: float, yaw_deg: float = 0.0
        ) -> JSONResponse:
            result = node.set_manual_goal(x=x, y=y, yaw_deg=yaw_deg)
            if result.get("success"):
                status = 200
            elif result.get("error") == "invalid_manual_goal":
                status = 400
            else:
                status = 409
            return JSONResponse(result, status_code=status)

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
