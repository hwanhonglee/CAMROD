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
    AvgAprilTagPose,
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
    PlanningRecallRequest,
    PlanningState,
    SystemStatus,
    UiDestinationCommand,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
# HH_260721 - Keep only the FastAPI symbols used by the runtime backend.
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from geometry_msgs.msg import PoseStamped, Twist
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
from tf2_ros import Buffer, ExtrapolationException, TransformListener
from ublox_msgs.msg import NavCOV, NavPVT, NavRELPOSNED9
from visualization_msgs.msg import Marker, MarkerArray

import uvicorn

from camrod_ui.api_common import to_diag_level_int
from camrod_ui.manual_drive_policy import (
    MANUAL_DRIVE_DEADMAN_TIMEOUT_S,
    ManualDriveCommand,
    ManualDriveLease,
    ManualDriveLimits,
    ManualDrivePolicy,
    ManualDriveProtocolError,
)
from camrod_ui.service_metrics import (
    ServiceMetricsTracker,
    default_service_metrics_path,
)
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


# HH_260818 - The optional occupancy guard stops a dispatch *before* the robot
# enters an occupied campsite. Once entry has begun it must stay quiet: the
# delivery target legitimately holds the guest's tent, and the robot only sees
# that tent up close after it has crabbed in and turned around. Cancelling there
# ended the run at the exact moment the operator was being asked to send it home.
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


# HH_260824 - A roadside service is intentionally stopped beside the lanelet,
# not at the authored campsite center.  The minimum lateral separation mirrors
# the campsite controller's goal-pair contract.  B13's authored center is more
# than 7 m from its snap, so roadside matching must apply the 0.30 m cap after
# projecting the raw pair rather than rejecting the authored center distance.
ROADSIDE_MINIMUM_SITE_LATERAL_M = 0.20
MAX_PENDING_SITE_ROUTE_GOALS = 32


# HH_260810 - One bounded JSON contract replaces the separate Tk/RViz operator
# viewers without changing any control or sensor-authority topic.
TELEMETRY_SCHEMA_VERSION = 3
TELEMETRY_VIEWS = frozenset({
    "gnss",
    "proximity",
    "camera",
    "trajectory",
    "perception",
    "safety",
    "docking",
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
    # HH_260818 - Docking sources are lazy and only add image/DDS load while
    # the operator has the docking tab open.
    "docking_debug_camera": (
        "/perception/apriltag_parking_detector/debug_image/compressed"
    ),
    "docking_tag_pose": "/perception/apriltag_parking_detector/tag_pose",
    "docking_tag_detected": "/perception/apriltag_parking_detector/tag_detected",
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


def _ros_stamp_seconds(stamp: Any) -> Optional[float]:
    """Convert a generated/builtin ROS timestamp to finite seconds."""
    try:
        seconds = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return seconds if math.isfinite(seconds) else None


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


def _transform_xy(
    x: float,
    y: float,
    source_to_target_transform: tuple[float, float, float],
) -> tuple[float, float]:
    """Apply a planar TF transform expressed as target translation and yaw."""
    tx, ty, yaw_rad = source_to_target_transform
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        tx + cos_yaw * x - sin_yaw * y,
        ty + sin_yaw * x + cos_yaw * y,
    )


def _sample_pointcloud_xy(
    message: PointCloud2,
    *,
    max_points: int,
    max_abs_xy_m: float,
    map_to_local_pose: Optional[tuple[float, float, float]] = None,
    source_to_target_transform: Optional[tuple[float, float, float]] = None,
    filter_before_transform: bool = False,
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
        if filter_before_transform and (
            abs(x) > max_abs_xy_m or abs(y) > max_abs_xy_m
        ):
            continue
        if map_to_local_pose is not None:
            origin_x, origin_y, yaw_rad = map_to_local_pose
            delta_x = x - origin_x
            delta_y = y - origin_y
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            x = cos_yaw * delta_x + sin_yaw * delta_y
            y = -sin_yaw * delta_x + cos_yaw * delta_y
        if source_to_target_transform is not None:
            x, y = _transform_xy(x, y, source_to_target_transform)
        if not filter_before_transform and (
            abs(x) > max_abs_xy_m or abs(y) > max_abs_xy_m
        ):
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
    source_to_target_transforms: Optional[
        Dict[str, tuple[float, float, float]]
    ] = None,
    target_frame_id: Optional[str] = None,
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
        marker_frame_id = str(marker.header.frame_id or "").strip()

        def transform(x: float, y: float) -> List[float]:
            marker_x = tx + cos_yaw * x - sin_yaw * y
            marker_y = ty + sin_yaw * x + cos_yaw * y
            if source_to_target_transforms is not None:
                marker_x, marker_y = _transform_xy(
                    marker_x,
                    marker_y,
                    source_to_target_transforms[marker_frame_id],
                )
            return [round(marker_x, 3), round(marker_y, 3)]

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
        "frame_id": target_frame_id or frame_id,
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
            # HH_260814 - Image.Resampling only exists from Pillow 9.1; the
            # Ubuntu 22.04 python3-pil exec_depend resolves to 9.0.1, where the
            # filter constants live on the module itself. Every camera frame is
            # wider than max_width, so the enum lookup raised AttributeError on
            # the first raw frame and killed the whole operator backend.
            resampling = getattr(PillowImage, "Resampling", PillowImage)
            image = image.resize((max_width, resized_height), resampling.BILINEAR)
        if image.mode not in {"RGB", "L"}:
            image = image.convert("RGB")
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=max(30, min(90, int(quality))))
        return output.getvalue()
    # HH_260814 - This runs inside a subscription callback, so an escaping
    # exception terminates rclpy.spin and takes readiness, destination
    # dispatch, and voice events down with it. A frame the fallback cannot
    # encode must degrade to "no preview", never to a dead backend.
    except (ImportError, AttributeError, OSError, ValueError):
        return None


@dataclass
class _TelemetryPlanarTransform:
    """Planar TF result plus browser-visible provenance and degradation state."""

    xy_yaw: Optional[tuple[float, float, float]]
    mode: str
    age_s: Optional[float]
    warning: str
    error: str


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
class ManualDriveTransport:
    """One generation-qualified WebSocket and its serialized send boundary."""

    websocket: WebSocket
    lease: ManualDriveLease
    loop: asyncio.AbstractEventLoop
    send_lock: asyncio.Lock


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
        # HH_260819 - Persist proof-of-operation evidence outside the build and
        # install trees so an ARM64 rebuild cannot erase accumulated history.
        self.service_metrics_enabled = bool(
            self.declare_parameter("service_metrics_enabled", True).value
        )
        self.service_metrics_database_path = str(
            self.declare_parameter(
                "service_metrics_database_path",
                str(default_service_metrics_path()),
            ).value
        )
        self.service_metrics_timezone = str(
            self.declare_parameter(
                "service_metrics_timezone", "Asia/Seoul"
            ).value
        )
        self.service_metrics_minimum_speed_mps = max(
            0.0,
            float(
                self.declare_parameter(
                    "service_metrics_minimum_speed_mps", 0.03
                ).value
            ),
        )
        self.service_metrics_maximum_speed_mps = max(
            self.service_metrics_minimum_speed_mps,
            float(
                self.declare_parameter(
                    "service_metrics_maximum_speed_mps", 3.0
                ).value
            ),
        )
        self.service_metrics_maximum_sample_gap_s = max(
            0.05,
            float(
                self.declare_parameter(
                    "service_metrics_maximum_sample_gap_s", 2.0
                ).value
            ),
        )
        self.service_metrics_checkpoint_interval_s = max(
            0.1,
            float(
                self.declare_parameter(
                    "service_metrics_checkpoint_interval_s", 5.0
                ).value
            ),
        )

        # Topic and destination dispatch parameters.
        self.ui_destination_topic = str(
            self.declare_parameter("ui_destination_topic", "/ui/selected_destination").value
        )
        self.ui_destination_dispatch_status_topic = str(
            self.declare_parameter(
                "ui_destination_dispatch_status_topic",
                "/ui/destination_dispatch_status",
            ).value
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
        # Dedicated manual Twist is opt-in. Ordinary CAMROD/develop launches
        # leave this empty; the CARLA composition explicitly supplies the topic.
        self.manual_cmd_vel_ros_topic = str(
            self.declare_parameter("manual_cmd_vel_ros_topic", "").value
        ).strip()
        self.manual_drive_available = bool(self.manual_cmd_vel_ros_topic)
        # The browser only sends a normalized scale.  The ROS backend owns the
        # physical command envelope, so a CARLA launch can expose an adjustable
        # Ranger-safe range without letting a client forge an arbitrary Twist.
        self.manual_drive_limits = ManualDriveLimits(
            linear_x_mps=float(
                self.declare_parameter(
                    "manual_drive_linear_limit_mps", 0.20
                ).value
            ),
            lateral_y_mps=float(
                self.declare_parameter(
                    "manual_drive_lateral_limit_mps", 0.20
                ).value
            ),
            angular_z_radps=float(
                self.declare_parameter(
                    "manual_drive_angular_limit_radps", 0.20
                ).value
            ),
        )
        self.manual_drive_deadman_timeout_s = float(
            self.declare_parameter(
                "manual_drive_deadman_timeout_s",
                MANUAL_DRIVE_DEADMAN_TIMEOUT_S,
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
        self.planning_return_to_drop_zone_topic = str(
            self.declare_parameter(
                "planning_return_to_drop_zone_topic",
                "/planning/state_machine/return_to_drop_zone",
            ).value
        )
        self.planning_camping_site_recall_topic = str(
            self.declare_parameter(
                "planning_camping_site_recall_topic",
                "/planning/state_machine/camping_site_recall",
            ).value
        )
        # HH_260819 - Close the old Nav2 command gate long enough for its final
        # velocity sample to expire before a manual Return publishes a new route.
        self.manual_return_preempt_hold_s = max(
            0.0,
            min(
                5.0,
                float(
                    self.declare_parameter(
                        "manual_return_preempt_hold_s", 0.5
                    ).value
                ),
            ),
        )
        # Re-arming before a campsite return was introduced for the CARLA
        # mission harness. Keep the develop lifecycle unchanged unless that
        # simulator profile opts in explicitly.
        self.return_site_exit_rearm_enabled = bool(
            self.declare_parameter(
                "return_site_exit_rearm_enabled", False
            ).value
        )
        # Give the selected parking controller one bounded heartbeat interval
        # to consume CANCEL before drop-zone alignment reopens the drive gate.
        self.parking_rearm_hold_s = max(
            0.0,
            min(
                2.0,
                float(
                    self.declare_parameter(
                        "parking_rearm_hold_s", 0.6
                    ).value
                ),
            ),
        )
        self.redock_request_timeout_s = max(
            3.0,
            min(
                30.0,
                float(
                    self.declare_parameter(
                        "redock_request_timeout_s", 10.0
                    ).value
                ),
            ),
        )
        # Physical Ranger motion is accepted only in CAN command mode.  Full
        # simulation explicitly disables this guard at the launch boundary;
        # an unavailable/unknown hardware status must never be treated as CAN.
        self.redock_require_can_control_mode = bool(
            self.declare_parameter(
                "redock_require_can_control_mode", True
            ).value
        )
        # HH_260825 - A campsite selection made while physically charging is
        # accepted immediately but motion remains disabled for this one-shot
        # dwell.  The timer is created only for a pending departure, avoiding a
        # permanent polling callback on the constrained ARM64 deployment.
        self.charging_departure_delay_s = max(
            0.0,
            min(
                30.0,
                float(
                    self.declare_parameter(
                        "charging_departure_delay_s", 7.0
                    ).value
                ),
            ),
        )
        self.drop_zone_exit_complete_topic = str(
            self.declare_parameter(
                "drop_zone_exit_complete_topic", "/control/drop_zone/exit_complete"
            ).value
        )
        self.drop_zone_maneuver_status_topic = str(
            self.declare_parameter(
                "drop_zone_maneuver_status_topic",
                "/control/drop_zone_maneuver_controller/status",
            ).value
        )
        self.reverse_parking_status_topic = str(
            self.declare_parameter(
                "reverse_parking_status_topic",
                "/parking/reverse_parking_controller/status",
            ).value
        )
        self.apriltag_parking_status_topic = str(
            self.declare_parameter(
                "apriltag_parking_status_topic",
                "/parking/apriltag_parking_controller/status",
            ).value
        )
        self.parking_method = str(
            self.declare_parameter("parking_method", "reverse").value
        ).strip().lower()
        if self.parking_method not in {"reverse", "apriltag"}:
            self.parking_method = "reverse"
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
        # HH_260824 - B11-B13 stop at a signed, lane-relative service point;
        # their semantic polygon centers are 3.3-9.0 m away and are not an
        # arrival predicate. Mirror the controller's requested offset and
        # existing entry/route tolerances without widening the global radius.
        self.site_arrival_roadside_offset_m = abs(
            float(
                self.declare_parameter(
                    "site_arrival_roadside_offset_m", 0.30
                ).value
            )
        )
        self.site_arrival_roadside_lateral_tolerance_m = abs(
            float(
                self.declare_parameter(
                    "site_arrival_roadside_lateral_tolerance_m", 0.15
                ).value
            )
        )
        self.site_arrival_roadside_forward_tolerance_m = abs(
            float(
                self.declare_parameter(
                    "site_arrival_roadside_forward_tolerance_m", 0.60
                ).value
            )
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
        default_drop_zones_yaml = ""
        try:
            default_drop_zones_yaml = str(
                Path(get_package_share_directory("camrod_map"))
                / "config"
                / "drop_zones.yaml"
            )
        except PackageNotFoundError:
            source_candidate = (
                Path(__file__).resolve().parents[4]
                / "camrod_map"
                / "config"
                / "drop_zones.yaml"
            )
            if source_candidate.is_file():
                default_drop_zones_yaml = str(source_candidate)
        self.drop_zones_yaml = str(
            self.declare_parameter(
                "drop_zones_yaml", default_drop_zones_yaml
            ).value
        )
        self.drop_zone_arrival_radius_m = max(
            0.1,
            float(
                self.declare_parameter("drop_zone_arrival_radius_m", 4.0).value
            ),
        )
        # HH_260818 - Keep the previous field behavior by default. Operators
        # can enable semantic tent occupancy from the top-level bringup config.
        self.enable_campsite_occupancy_guard = bool(
            self.declare_parameter("enable_campsite_occupancy_guard", False).value
        )
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
        self.planning_lanelet_pose_topic = str(
            self.declare_parameter(
                "planning_lanelet_pose_topic", "/planning/lanelet_pose"
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
        self.telemetry_tf_transform_enabled = bool(
            self.declare_parameter(
                "telemetry_tf_transform_enabled", False
            ).value
        )
        # Develop keeps the diagnostic Euclidean boxes visible by default.
        # External-simulator compositions may disable the subscription so a
        # classless raw cluster cannot be mistaken for semantic stop evidence.
        self.telemetry_raw_lidar_bbox_overlay_enabled = bool(
            self.declare_parameter(
                "telemetry_raw_lidar_bbox_overlay_enabled", True
            ).value
        )
        tf_latest_tolerance = float(
            self.declare_parameter(
                "telemetry_tf_latest_fallback_tolerance_s", 0.0
            ).value
        )
        # Exact-time TF remains the production default.  Simulator launch
        # compositions may opt into a small, explicit bound when independently
        # reception-stamped sensor and odometry callbacks arrive out of order.
        self.telemetry_tf_latest_fallback_tolerance_s = (
            min(0.25, tf_latest_tolerance)
            if math.isfinite(tf_latest_tolerance) and tf_latest_tolerance > 0.0
            else 0.0
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
        # Compressed images are the primary operator-UI transport.  Keep the
        # raw-image fallback enabled for ordinary CAMROD deployments whose
        # camera driver may not publish JPEG, while allowing simulator
        # compositions with guaranteed compressed relays to avoid receiving
        # and copying the same high-bandwidth frame twice.
        self.telemetry_camera_raw_fallback_enabled = bool(
            self.declare_parameter(
                "telemetry_camera_raw_fallback_enabled", True
            ).value
        )
        self.telemetry_docking_rear_camera_fallback_enabled = bool(
            self.declare_parameter(
                "telemetry_docking_rear_camera_fallback_enabled", False
            ).value
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
        self._drop_zone_keypoint = self._load_drop_zone_keypoint(
            self.drop_zones_yaml
        )
        self._lock = threading.Lock()
        # ROS callbacks and FastAPI requests can submit destination commands
        # from different threads. Serialize the complete admission/dispatch
        # transaction so one B-site cannot replace another while station exit
        # is pending.
        # One re-entrant lock orders destination mutation, service-terminal
        # reconciliation, and the authoritative cross-UI acknowledgement.
        # Re-entrancy is required because run=false dispatch calls the shared
        # stop path while already holding this lock.
        self._destination_dispatch_lock = threading.RLock()
        # FastAPI runs in its own thread while the deadman timer runs on the ROS
        # executor. One re-entrant transition lock keeps arm/STOP/mission
        # takeover and delayed WebSocket cleanup strictly ordered.
        self._manual_drive_transition_lock = threading.RLock()
        self._manual_drive_policy = ManualDrivePolicy(
            available=self.manual_drive_available,
            limits=self.manual_drive_limits,
            deadman_timeout_s=self.manual_drive_deadman_timeout_s,
        )
        self._manual_drive_transport: Optional[ManualDriveTransport] = None
        self._manual_drive_shutdown_done = False
        self._runtime_policy = UiStatePolicy(
            self.readiness_required_modules,
            max_ready_localization_mode=self.max_ready_localization_mode,
        )
        self._state = ApiState(
            ws_site_states={s: False for s in self.site_names},
        )
        self._service_metrics = ServiceMetricsTracker(
            self.service_metrics_database_path
            if self.service_metrics_enabled else None,
            timezone_name=self.service_metrics_timezone,
            minimum_speed_mps=self.service_metrics_minimum_speed_mps,
            maximum_speed_mps=self.service_metrics_maximum_speed_mps,
            maximum_sample_gap_s=self.service_metrics_maximum_sample_gap_s,
            checkpoint_interval_s=self.service_metrics_checkpoint_interval_s,
        )
        if self._service_metrics.persistence_error:
            self.get_logger().warn(
                "service metrics persistence unavailable; continuing with "
                f"in-memory evidence: {self._service_metrics.persistence_error}"
            )
        self._telemetry = self._new_telemetry_snapshot()
        self._telemetry["options"].update({
            "docking_rear_camera_fallback_enabled": (
                self.telemetry_docking_rear_camera_fallback_enabled
            ),
            "raw_lidar_bbox_overlay_enabled": (
                self.telemetry_raw_lidar_bbox_overlay_enabled
            ),
        })
        self._telemetry_source_rx: Dict[str, float] = {}
        self._telemetry_source_history: Dict[str, deque[float]] = defaultdict(
            lambda: deque(maxlen=20)
        )
        self._telemetry_images: Dict[str, Dict[str, Any]] = {
            "front": {"data": b"", "media_type": "image/jpeg", "sequence": 0},
            "rear": {"data": b"", "media_type": "image/jpeg", "sequence": 0},
            "docking": {"data": b"", "media_type": "image/jpeg", "sequence": 0},
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
        self._telemetry_tf_warning_last: Dict[str, float] = defaultdict(float)
        self._telemetry_last_trace_sample = 0.0
        self._readiness_log_last_sec = 0.0
        self._latest_arrival_pose: Optional[AvgPoseStamped] = None
        self._latest_arrival_pose_time_s = 0.0
        self._latest_planning_lanelet_pose: Optional[AvgPoseStamped] = None
        self._latest_planning_lanelet_pose_time_s = 0.0
        # HH_260824 - Correlate the UI's raw campsite goal with goal_snapper's
        # output using the preserved ROS timestamp.  A mission-scoped real snap
        # is required before roadside parked-state adoption; the live vehicle
        # pose is never promoted into a synthetic return anchor.
        self._pending_site_route_goal_stamps: Dict[tuple[int, int], str] = {}
        self._site_route_anchors: Dict[str, PoseStamped] = {}
        # HH_260721 - Keep only the latest requested site while drop-zone exit owns motion.
        self._latest_platform_is_charging = False
        self._latest_platform_control_mode = -1
        self._latest_service_state: Optional[int] = None
        # A terminal heartbeat is also present while a newly accepted recall
        # waits for station departure.  Arm cleanup only after that recall has
        # actually entered its return/parking leg.
        self._recall_terminal_clear_armed = False
        self._pending_site_after_drop_zone_exit: Optional[tuple[str, str, str]] = None
        self._drop_zone_exit_active = False
        self._drop_zone_exit_handoff_ready = False
        self._drop_zone_exit_failure_latched = False
        self._drop_zone_exit_waiting_for_fresh_status = False
        self._drop_zone_exit_cancel_suppressed = False
        # HH_260825 - Serialize the one-shot charger departure timer against
        # operator Stop without re-entering the general UI state lock.
        self._charging_departure_transition_lock = threading.Lock()
        self._charging_departure_delay_pending = False
        self._charging_departure_transition_timer: Optional[Any] = None
        self._charging_departure_from_charger = False
        # HJ_260804 - Destination state may be cleared by a departure ack
        # before campsite arrival. Preserve the active mission site so the
        # Robot and Guest UIs still receive the matching arrival notification.
        self._active_mission_site: str = ""
        # Preserve the accepted command origin independently from presentation
        # state. An occupied tent is expected for guest roadside recall and must
        # not trigger the delivery-only perception auto-cancel while en route.
        self._active_mission_source: str = ""
        # Service-state messages do not carry a mission id. Bind return and
        # terminal cleanup to the generation that accepted the UI command so a
        # delayed terminal from a stopped mission cannot erase its successor.
        # Start each backend incarnation in a disjoint JavaScript-safe range.
        # A Guest browser can survive a backend restart; reusing generation 1
        # would otherwise let an old B-site operation match the new process.
        self._mission_generation = (time.time_ns() // 1_000_000) * 1000
        self._active_mission_generation = 0
        self._return_requested_generation = 0
        self._return_progress_generation = 0
        self._return_operation_token = ""
        self._return_operation_sequence = 0
        self._terminal_clear_armed_generation = 0
        self._active_mission_retryable = False
        # HH_260819 - Return uses one transient timer only while changing Nav2
        # ownership. Repeated buttons share this latch instead of creating work.
        self._manual_return_transition_lock = threading.Lock()
        self._manual_return_transition_pending = False
        self._manual_return_transition_timer: Optional[Any] = None
        self._manual_return_transition_source = ""
        self._manual_return_transition_generation = 0
        self._manual_return_transition_token = ""
        # HH_260904 - A Return press during charger-release debounce is an
        # explicit re-dock request, not a no-op. Keep it until canonical CAN
        # charging falls, then reset the old PARKED owner before realignment.
        # One lock and monotonic generation serialize charger release, CAN
        # handoff, timer completion, Stop, and destination supersession.  A
        # callback may carry a generation outside the lock, but it may publish
        # motion only while that generation is still current.
        self._redock_after_disconnect_lock = threading.RLock()
        self._redock_generation = 0
        self._redock_after_disconnect_pending = False
        self._redock_after_disconnect_source = ""
        self._redock_after_disconnect_requested_at_s = 0.0
        self._redock_after_disconnect_generation = 0
        self._redock_after_disconnect_timer: Optional[Any] = None
        self._parking_rearm_transition_lock = self._redock_after_disconnect_lock
        self._parking_rearm_transition_pending = False
        self._parking_rearm_transition_timer: Optional[Any] = None
        self._parking_rearm_transition_source = ""
        self._parking_rearm_transition_generation = 0
        self._parking_rearm_waiting_for_can = False
        self._parking_rearm_waiting_source = ""
        self._parking_rearm_waiting_generation = 0
        self._parking_controller_operating_states: Dict[str, str] = {}
        # Persist asynchronous re-dock progress across websocket reconnects.
        # The booleans describe the current wait gates; this explicit status
        # distinguishes terminal outcomes such as expiry from a CAN handoff.
        self._redock_status = "idle"
        self._redock_status_message = ""
        self._low_battery_return_pending = False
        self._low_battery_return_started = False
        self._low_battery_return_wait_notified = False
        # HH_260706 - HTTP site selection dispatches immediately; consume every
        # local topic echo exactly once. The random per-process source marker is
        # authoritative even if this bounded debug FIFO has already wrapped.
        self._direct_destination_echoes: deque[tuple[str, bool, str, float]] = deque(
            maxlen=64
        )
        self._direct_destination_echo_sequence = 0
        self._direct_destination_echo_prefix = (
            f"{os.getpid()}-{time.monotonic_ns()}"
        )
        # Compatibility/debug mirror of the newest unconsumed entry.
        self._last_direct_destination_echo: Optional[tuple[str, bool, str, float]] = None
        # `/service/state` has no header, source, or mission id and Humble does
        # not expose the publisher GID to Python callbacks.  Correlate our own
        # messages with a per-process token embedded in description; identical
        # external emergency messages therefore remain authoritative.
        self._service_state_echo_prefix = (
            f"{os.getpid()}-{time.monotonic_ns()}"
        )
        self._service_state_echo_sequence = 0
        # ServiceState has no header or publisher identity. This epoch covers
        # generation-0 authorities too (manual map, manual drive, standalone
        # Return), while mission_generation covers campsite ownership.
        self._command_epoch_lock = threading.Lock()
        self._command_epoch = (time.time_ns() // 1_000_000) * 1000
        self._generation_zero_authority = ""
        self._generation_zero_authority_epoch = 0
        self._standalone_return_progress = False
        self._standalone_return_parking_seen = False
        # UI command admission remains closed until the post-discovery Nav2
        # cancel futures complete. This is the restart ownership barrier.
        self._startup_recovery_pending = True
        self._startup_fail_closed_attempts = 0
        self._startup_nav2_cancel_futures: Dict[str, Any] = {}
        self._startup_nav2_cancel_completed: Set[str] = set()

        # WebSocket client management.
        self._ws_clients: Set[WebSocket] = set()
        self._ws_clients_lock = threading.Lock()
        self._ws_client_send_locks: Dict[WebSocket, asyncio.Lock] = {}
        # New clients receive one atomic snapshot followed by every broadcast
        # queued after that snapshot before joining the live set.
        self._ws_initializing_clients: Dict[WebSocket, List[dict]] = {}
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
        self.sub_planning_lanelet_pose = self.create_subscription(
            AvgPoseStamped,
            self.planning_lanelet_pose_topic,
            self._on_planning_lanelet_pose,
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
        # HH_260819 - AprilTag parking republishes its current service state as
        # a 2 Hz heartbeat.  Keep QoS compatible with the existing volatile
        # publishers; the periodic value, rather than a latched historical
        # event, restores departure_required after a UI restart.
        #
        # HH_260906 - RETURN ownership is acknowledged by the campsite
        # controller's first token-bearing RETURN_WITH_CARGO sample.  The HTTP
        # dispatch transaction deliberately holds _destination_dispatch_lock,
        # so ALIGN_RETRACE_YAW, CRAB_OUT, and DONE can queue before the executor
        # enters this subscription callback.  A depth-one reader replaced the
        # unique token-bearing edge with the following heartbeat and left the
        # mission unable to accept otherwise valid parking/charging terminal
        # states.  Preserve the bounded transition burst without changing the
        # reliable/volatile wire contract or weakening generation, token, and
        # physical-pose validation below.
        service_heartbeat_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_service_state = self.create_subscription(
            AvgServiceState,
            self.service_state_topic,
            self._on_service_state,
            service_heartbeat_qos,
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
        self.sub_drop_zone_maneuver_status = self.create_subscription(
            ModuleState,
            self.drop_zone_maneuver_status_topic,
            self._on_drop_zone_maneuver_status,
            service_heartbeat_qos,
        )
        # Parking ERROR has no service-state enum of its own.  Keep the
        # controller health heartbeat available even when the telemetry tab is
        # closed so an explicit Return can distinguish a live parking attempt
        # from a failed one and safely retry it.
        self.sub_reverse_parking_status = self.create_subscription(
            ModuleState,
            self.reverse_parking_status_topic,
            lambda message: self._on_parking_controller_status(
                "reverse_parking", message
            ),
            service_heartbeat_qos,
        )
        self.sub_apriltag_parking_status = self.create_subscription(
            ModuleState,
            self.apriltag_parking_status_topic,
            lambda message: self._on_parking_controller_status(
                "apriltag_parking", message
            ),
            service_heartbeat_qos,
        )
        self.sub_campsite_occupancy = None
        if self.enable_campsite_occupancy_guard:
            self.sub_campsite_occupancy = self.create_subscription(
                CampsiteOccupancy,
                self.campsite_occupancy_topic,
                self._on_campsite_occupancy,
                state_qos,
            )

        # Publishers.
        # HH_260617: UI destination and planning mission-key topics now use
        # generated avg_msgs semantic messages instead of JSON/String wrappers.
        self.pub_destination = self.create_publisher(
            UiDestinationCommand, self.ui_destination_topic, 10
        )
        self.pub_destination_dispatch_status = self.create_publisher(
            String, self.ui_destination_dispatch_status_topic, state_qos
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
        self.pub_planning_return_to_drop_zone = self.create_publisher(
            PlanningRecallRequest, self.planning_return_to_drop_zone_topic, 10
        )
        self.pub_planning_camping_site_recall = self.create_publisher(
            PlanningRecallRequest, self.planning_camping_site_recall_topic, 10
        )
        self.pub_mission_key = self.create_publisher(
            PlanningMissionKey, self.planning_mission_key_topic, 10
        )
        self.pub_goal_pose = self.create_publisher(PoseStamped, self.planning_goal_pose_topic, 10)
        self.pub_manual_goal_pose = self.create_publisher(
            PoseStamped, self.manual_goal_pose_topic, 10
        )
        self.pub_manual_cmd_vel_ros = None
        if self.manual_drive_available:
            self.pub_manual_cmd_vel_ros = self.create_publisher(
                Twist, self.manual_cmd_vel_ros_topic, 10
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
        # HH_260819 - FastAPI wakes the ROS executor immediately through a guard
        # condition. The 1 Hz timer only expires abandoned leases, replacing the
        # previous permanent 10 Hz poll on the 8-core ARM64 deployment.
        self._telemetry_session_guard = self.create_guard_condition(
            self._maintain_telemetry_session
        )
        self._telemetry_session_timer = self.create_timer(
            1.0, self._maintain_telemetry_session
        )
        self._manual_drive_deadman_timer = None
        if self.manual_drive_available:
            # Check faster than the configured UI lease. Commands are not
            # replayed here, so the independent 0.35 s control/adapter watchdogs
            # remain effective if this callback ever stops running.
            self._manual_drive_deadman_timer = self.create_timer(
                0.05, self._on_manual_drive_deadman
            )
        # A backend restart cannot recover the exact controller/mission owner
        # from volatile ROS state.  Fail closed before accepting a new UI
        # command: revoke every possible motion writer, close both engage
        # gates, publish OPERATOR_STOPPED, and clear the transient-local Guest
        # identity.  Publishing only an empty dispatch status would leave a
        # surviving Nav2/controller process moving with no UI owner.
        UiBackendNode._stop_active_service_serialized(
            self, "backend_startup_fail_closed"
        )
        # DDS discovery is asynchronous. The constructor-time volatile CANCEL
        # frames and Nav2 service request can precede subscriber/service
        # discovery, so reassert the fail-closed boundary after spin begins.
        # New commands remain rejected until every sent cancel future has
        # completed, preventing a late cancel-all from erasing a new goal.
        self._startup_fail_closed_timer = self.create_timer(
            0.5, self._reassert_startup_fail_closed
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
            f"manual_cmd_vel_ros_topic={self.manual_cmd_vel_ros_topic or '(disabled)'} "
            f"manual_drive_limits={self.manual_drive_limits.as_dict()} "
            f"arrival_pose_topic={self.arrival_pose_topic} "
            f"planning_lanelet_pose_topic={self.planning_lanelet_pose_topic} "
            f"roadside_arrival_offset={self.site_arrival_roadside_offset_m:.2f}m "
            f"roadside_arrival_lateral_tolerance={self.site_arrival_roadside_lateral_tolerance_m:.2f}m "
            f"roadside_arrival_forward_tolerance={self.site_arrival_roadside_forward_tolerance_m:.2f}m "
            f"campsite_occupancy_topic={self.campsite_occupancy_topic} "
            f"campsite_occupancy_guard={str(self.enable_campsite_occupancy_guard).lower()} "
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

    def _load_drop_zone_keypoint(self, yaml_path: str) -> Optional[MissionKeypoint]:
        """Load the physical station area used to validate return terminals."""
        path = Path(yaml_path).expanduser() if yaml_path else None
        if path is None or not path.is_file():
            self.get_logger().warn(
                "drop_zones_yaml unavailable: return terminal states will "
                "fail closed until station geometry is configured"
            )
            return None
        try:
            with path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to read drop_zones_yaml {path}: {exc}")
            return None
        raw_zones = data.get("drop_zones", [])
        if not isinstance(raw_zones, list) or not raw_zones:
            self.get_logger().error(f"drop_zones_yaml has no drop_zones list: {path}")
            return None
        zone = next(
            (
                item
                for item in raw_zones
                if isinstance(item, dict)
                and str(item.get("type", "")).strip() == "drop_zone"
            ),
            None,
        )
        if zone is None:
            self.get_logger().error(f"drop_zones_yaml has no type=drop_zone: {path}")
            return None
        corners: List[tuple[float, float]] = []
        for corner in zone.get("corners", []):
            if not isinstance(corner, dict):
                continue
            try:
                corners.append((float(corner["x"]), float(corner["y"])))
            except (KeyError, TypeError, ValueError):
                continue
        try:
            return MissionKeypoint(
                key="drop_zone",
                frame_id=str(
                    zone.get("frame_id", self.default_goal_frame_id)
                ).strip()
                or self.default_goal_frame_id,
                x=float(zone["x"]),
                y=float(zone["y"]),
                z=float(zone.get("z", 0.0)),
                yaw_deg=float(zone.get("yaw_deg", 0.0)),
                service_mode="drop_zone",
                corners=corners,
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"invalid drop-zone geometry in {path}: {exc}"
            )
            return None

    def _fresh_arrival_xy(self) -> tuple[Optional[tuple[float, float]], str]:
        """Return a finite, fresh map-frame physical position or a reason."""
        pose = getattr(self, "_latest_arrival_pose", None)
        if pose is None:
            return None, "missing_pose"
        age_s = self._now_s() - float(
            getattr(self, "_latest_arrival_pose_time_s", 0.0)
        )
        timeout_s = float(getattr(self, "site_arrival_pose_timeout_s", 0.0))
        if timeout_s > 0.0 and age_s > timeout_s:
            return None, f"stale_pose:{age_s:.2f}s"
        frame = str(
            pose.header.frame_id or getattr(self, "default_goal_frame_id", "map")
        )
        expected_frame = str(getattr(self, "default_goal_frame_id", "map"))
        if frame and expected_frame and frame != expected_frame:
            return None, f"frame_mismatch:{frame}!={expected_frame}"
        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            return None, "nonfinite_pose"
        return (x, y), "fresh_pose"

    def _drop_zone_arrival_match(self) -> tuple[bool, float, str]:
        """Validate the return terminal against the authored station area."""
        xy, pose_reason = UiBackendNode._fresh_arrival_xy(self)
        if xy is None:
            return False, float("inf"), pose_reason
        target = getattr(self, "_drop_zone_keypoint", None)
        if target is None:
            return False, float("inf"), "missing_drop_zone_geometry"
        x, y = xy
        distance = math.hypot(x - float(target.x), y - float(target.y))
        # An exported station polygon is the authoritative physical boundary.
        # Do not re-admit an outside point through the legacy center-radius
        # fallback: the CARLA road-front spawn is intentionally close to the
        # station center while already sitting on its lanelet handoff.  Such a
        # false match starts an unnecessary bounded EXIT whose nearest-lanelet
        # target is correctly rejected as being behind/too close.  Radius-only
        # deployments retain the historical behavior when no polygon exists.
        if target.corners:
            if self._point_in_polygon(x, y, target.corners):
                return True, distance, "inside_drop_zone_polygon"
            return False, distance, "outside_drop_zone_polygon"
        if distance <= float(getattr(self, "drop_zone_arrival_radius_m", 4.0)):
            return True, distance, "near_drop_zone_center"
        return False, distance, "outside_drop_zone"

    def _arm_stationary_drop_zone_return_progress(self) -> bool:
        """Bind an accepted re-dock alignment to the current mission token."""
        active_site, _, active_generation, return_generation = (
            UiBackendNode._active_mission_identity(self)
        )
        token = str(getattr(self, "_return_operation_token", "")).strip()
        matched, _, _ = UiBackendNode._drop_zone_arrival_match(self)
        if (
            not active_site
            and active_generation == 0
            and str(getattr(self, "_generation_zero_authority", ""))
            == "standalone_return"
            and token
            and matched
        ):
            self._standalone_return_progress = True
            return True
        if not (
            active_site
            and active_generation > 0
            and return_generation == active_generation
            and token
            and matched
        ):
            return False
        self._return_progress_generation = active_generation
        return True

    def _manual_drive_state_payload(self) -> Dict[str, Any]:
        with self._manual_drive_transition_lock:
            state = self._manual_drive_policy.snapshot()
        return {"type": "state", "manual_drive": state}

    def _publish_manual_drive_command(
        self, command: ManualDriveCommand
    ) -> None:
        publisher = self.pub_manual_cmd_vel_ros
        if publisher is None:
            return
        message = Twist()
        message.linear.x = float(command.linear_x)
        message.linear.y = float(command.linear_y)
        message.angular.z = float(command.angular_z)
        publisher.publish(message)

    def _publish_manual_drive_zero(self) -> None:
        self._publish_manual_drive_command(ManualDriveCommand())

    async def _send_manual_drive_payload(
        self, transport: ManualDriveTransport, payload: Dict[str, Any]
    ) -> None:
        """Serialize all handler and ROS-timer sends for one WebSocket."""
        async with transport.send_lock:
            await transport.websocket.send_json(payload)

    def _schedule_manual_drive_payload(
        self, transport: Optional[ManualDriveTransport], payload: Dict[str, Any]
    ) -> None:
        if transport is None or transport.loop.is_closed():
            return
        try:
            future = asyncio.run_coroutine_threadsafe(
                self._send_manual_drive_payload(transport, payload),
                transport.loop,
            )
        except RuntimeError:
            return

        def consume_result(completed: Any) -> None:
            try:
                completed.result()
            except (RuntimeError, WebSocketDisconnect):
                pass
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(
                    f"manual-drive websocket state send failed: {exc}"
                )

        future.add_done_callback(consume_result)

    def _revoke_manual_drive(
        self, reason: str, *, notify: bool = True
    ) -> None:
        """Publish zero and revoke manual authorization state, if enabled."""
        if not self.manual_drive_available:
            return
        with self._manual_drive_transition_lock:
            self._manual_drive_policy.revoke(reason)
            transport = self._manual_drive_transport if notify else None
            payload = {
                "type": "state",
                "manual_drive": self._manual_drive_policy.snapshot(),
            }
            self._publish_manual_drive_zero()
        self._schedule_manual_drive_payload(transport, payload)

    def _arm_manual_drive(
        self, lease: ManualDriveLease, payload: Any
    ) -> Dict[str, Any]:
        """Order manual preemption after the destination transaction lock."""
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._arm_manual_drive_serialized(self, lease, payload)
        with dispatch_lock:
            return UiBackendNode._arm_manual_drive_serialized(self, lease, payload)

    def _arm_manual_drive_serialized(
        self, lease: ManualDriveLease, payload: Any
    ) -> Dict[str, Any]:
        """Atomically preempt all motion owners, then arm the manual boundary."""
        startup_block = UiBackendNode._startup_recovery_block(self)
        if startup_block is not None:
            raise ManualDriveProtocolError(
                "backend_startup_recovery", startup_block["message"]
            )
        with self._manual_drive_transition_lock:
            # Validate before changing the running system, but consume seq only
            # after the complete STOP/cancel transition has succeeded.
            self._manual_drive_policy.validate_arm(lease, payload)
            self._stop_active_service(source="ws_manual_drive_arm")
            # STOP and the newly armed generation-0 manual owner are distinct
            # epochs. A delayed self echo of STOP must not disarm this lease.
            UiBackendNode._advance_command_epoch(self)
            UiBackendNode._set_generation_zero_authority(self, "manual_drive")
            # Full STOP normally releases mission engage in the production UI
            # profile. Publish the false boundary explicitly so opt-in standalone
            # compositions have the same deterministic manual ownership order.
            self._publish_mission_engage(
                False, source="ws_manual_drive_arm:mission_release"
            )
            self._publish_manual_drive_zero()
            self._manual_drive_policy.arm(lease, payload, time.monotonic())
            self._publish_engage(True, source="ws_manual_drive_arm")
            return {
                "type": "state",
                "manual_drive": self._manual_drive_policy.snapshot(),
            }

    def _apply_manual_drive(
        self, lease: ManualDriveLease, payload: Any
    ) -> Dict[str, Any]:
        with self._manual_drive_transition_lock:
            command = self._manual_drive_policy.drive(
                lease, payload, time.monotonic()
            )
            self._publish_manual_drive_command(command)
            return {
                "type": "state",
                "manual_drive": self._manual_drive_policy.snapshot(),
            }

    def _disarm_manual_drive(
        self, lease: ManualDriveLease, payload: Any
    ) -> Dict[str, Any]:
        with self._manual_drive_transition_lock:
            previous_state = self._manual_drive_policy.snapshot()
            was_active = bool(
                previous_state["armed"] or previous_state["holding"]
            )
            self._manual_drive_policy.disarm(lease, payload)
            if was_active:
                UiBackendNode._clear_generation_zero_authority(
                    self, "manual_drive"
                )
                self._publish_manual_drive_zero()
                self._publish_engage(False, source="ws_manual_drive_disarm")
            return {
                "type": "state",
                "manual_drive": self._manual_drive_policy.snapshot(),
            }

    def _fail_closed_manual_drive(
        self, lease: ManualDriveLease, error: ManualDriveProtocolError
    ) -> Dict[str, Any]:
        """An invalid or expired owner frame cannot preserve authorization."""
        with self._manual_drive_transition_lock:
            if self._manual_drive_policy.lease_matches(lease):
                reason = (
                    "deadman_timeout"
                    if error.error == "manual_drive_deadman_expired"
                    else f"protocol_error:{error.error}"
                )
                self._manual_drive_policy.revoke(reason)
                UiBackendNode._clear_generation_zero_authority(
                    self, "manual_drive"
                )
                self._publish_manual_drive_zero()
                self._publish_engage(
                    False, source=f"ws_manual_drive_error:{error.error}"
                )
            return {
                "type": "error",
                "error": error.error,
                "message": str(error),
                "manual_drive": self._manual_drive_policy.snapshot(),
            }

    def _disconnect_manual_drive(self, lease: ManualDriveLease) -> bool:
        """Stop only the matching generation; stale finally blocks are inert."""
        with self._manual_drive_transition_lock:
            previous_state = self._manual_drive_policy.snapshot()
            was_active = bool(
                previous_state["armed"] or previous_state["holding"]
            )
            if not self._manual_drive_policy.disconnect(lease):
                return False
            if (
                self._manual_drive_transport is not None
                and self._manual_drive_transport.lease == lease
            ):
                self._manual_drive_transport = None
            if was_active:
                UiBackendNode._clear_generation_zero_authority(
                    self, "manual_drive"
                )
                self._publish_manual_drive_zero()
                self._publish_engage(False, source="ws_manual_drive_disconnect")
            return True

    def _on_manual_drive_deadman(self) -> None:
        if not self.manual_drive_available:
            return
        with self._manual_drive_transition_lock:
            if not self._manual_drive_policy.expire(time.monotonic()):
                return
            transport = self._manual_drive_transport
            self._publish_manual_drive_zero()
            UiBackendNode._clear_generation_zero_authority(
                self, "manual_drive"
            )
            self._publish_engage(False, source="manual_drive_deadman")
            payload = {
                "type": "state",
                "manual_drive": self._manual_drive_policy.snapshot(),
            }
        self._schedule_manual_drive_payload(transport, payload)
        self.get_logger().warn(
            "manual-drive deadman expired after %.2fs; zero and disengage published"
            % self._manual_drive_policy.deadman_timeout_s
        )

    def _shutdown_manual_drive(self) -> None:
        if not self.manual_drive_available:
            return
        with self._manual_drive_transition_lock:
            if self._manual_drive_shutdown_done:
                return
            self._manual_drive_shutdown_done = True
            transport = self._manual_drive_transport
            self._manual_drive_transport = None
            previous_state = self._manual_drive_policy.snapshot()
            was_active = bool(
                previous_state["armed"] or previous_state["holding"]
            )
            self._manual_drive_policy.shutdown()
            # Publish the final boundary while the ROS context is available.
            # If shutdown has already invalidated it, the independent command
            # watchdog remains the fail-safe backstop.
            context = getattr(self, "context", None)
            context_ok = context is None or rclpy.ok(context=context)
            if was_active and context_ok:
                try:
                    self._publish_manual_drive_zero()
                    self._publish_engage(False, source="manual_drive_shutdown")
                except RuntimeError:
                    if context is None or rclpy.ok(context=context):
                        raise
                    self.get_logger().warn(
                        "ROS context closed during manual-drive shutdown; "
                        "safety-gate watchdog will hold zero"
                    )
            elif was_active:
                self.get_logger().warn(
                    "ROS context already closed during manual-drive shutdown; "
                    "safety-gate watchdog will hold zero"
                )
            payload = {
                "type": "state",
                "manual_drive": self._manual_drive_policy.snapshot(),
            }
        self._schedule_manual_drive_payload(transport, payload)

    def destroy_node(self) -> bool:
        # HH_260805 - Stop the HTTP event loop before ROS destroys callbacks and
        # publishers that in-flight FastAPI/WebSocket handlers may still access.
        self._cancel_pending_manual_return_transition("node_shutdown")
        self._cancel_pending_charging_departure_transition("node_shutdown")
        self._cancel_pending_redock_after_disconnect("node_shutdown")
        self._cancel_pending_parking_rearm_transition("node_shutdown")
        self._shutdown_manual_drive()
        self._stop_fastapi_server()
        service_metrics = getattr(self, "_service_metrics", None)
        if service_metrics is not None:
            service_metrics.close()
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
            "options": {
                "docking_rear_camera_fallback_enabled": False,
                "raw_lidar_bbox_overlay_enabled": True,
            },
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
                "docking": {"available": False, "sequence": 0},
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
            "docking": {
                "tag_detected": False,
                "tag": {},
                "is_charging": False,
                "battery_percentage": None,
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
        # HH_260819 - GuardCondition.trigger() is the cross-thread handoff; ROS
        # subscription graph mutation remains on the executor callback.
        guard = getattr(self, "_telemetry_session_guard", None)
        if guard is not None:
            guard.trigger()
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
            subscriptions.append(
                self.create_subscription(
                    CompressedImage, self.telemetry_topics["front_camera"],
                    lambda message: self._on_telemetry_camera("front", message),
                    sensor_qos,
                )
            )
            if self.telemetry_camera_raw_fallback_enabled:
                subscriptions.append(self.create_subscription(
                    RosImage, self.telemetry_topics["front_camera_raw"],
                    lambda message: self._on_telemetry_raw_camera("front", message),
                    sensor_qos,
                ))

        # The camera tab retains develop's front/rear pair. A docking-only rear
        # subscription is added only when an external-simulator profile opts
        # into the rear-camera fallback.
        if wants("camera") or (
            wants("docking")
            and getattr(
                self,
                "telemetry_docking_rear_camera_fallback_enabled",
                False,
            )
        ):
            subscriptions.append(
                self.create_subscription(
                    CompressedImage, self.telemetry_topics["rear_camera"],
                    lambda message: self._on_telemetry_camera("rear", message),
                    sensor_qos,
                )
            )
            if self.telemetry_camera_raw_fallback_enabled:
                subscriptions.append(self.create_subscription(
                    RosImage, self.telemetry_topics["rear_camera_raw"],
                    lambda message: self._on_telemetry_raw_camera("rear", message),
                    sensor_qos,
                ))

        if wants("docking"):
            subscriptions.extend([
                self.create_subscription(
                    CompressedImage,
                    self.telemetry_topics["docking_debug_camera"],
                    lambda message: self._on_telemetry_camera("docking", message),
                    sensor_qos,
                ),
                self.create_subscription(
                    AvgAprilTagPose,
                    self.telemetry_topics["docking_tag_pose"],
                    self._on_telemetry_docking_tag_pose,
                    sensor_qos,
                ),
                self.create_subscription(
                    AvgBool,
                    self.telemetry_topics["docking_tag_detected"],
                    self._on_telemetry_docking_tag_detected,
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
            subscriptions.append(self.create_subscription(
                PointCloud2, self.telemetry_topics["obstacle_cloud"],
                lambda message: self._on_telemetry_cloud("obstacles", message),
                sensor_qos,
            ))
            if getattr(
                self, "telemetry_raw_lidar_bbox_overlay_enabled", True
            ):
                subscriptions.append(self.create_subscription(
                    MarkerArray, self.telemetry_topics["obstacle_boxes"],
                    self._on_telemetry_obstacle_boxes, sensor_qos,
                ))
            for layer_name in ("lanelet", "lidar", "radar", "inflation"):
                subscriptions.append(self.create_subscription(
                    AvgOccupancyGrid,
                    self.telemetry_topics[f"{layer_name}_cost_grid"],
                    lambda message, name=layer_name: self._on_telemetry_cost_grid(
                        name, message
                    ),
                    sensor_qos,
                ))

        # HH_260904 - The proximity UI needs one lightweight evidence string
        # to distinguish a raw radar echo from a cost-producing return. Keep
        # the heavier safety-only replan stream scoped to its original tab.
        if wants("safety", "proximity"):
            subscriptions.append(self.create_subscription(
                AvgString, self.telemetry_topics["radar_evidence"],
                lambda message: self._on_telemetry_text(
                    "radar_evidence", message.data
                ), 10,
            ))
        if wants("safety"):
            subscriptions.extend([
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
            "drop_zone_parking": "/control/drop_zone_maneuver_controller/parking_approach_path_ros",
            "reverse_parking": "/parking/reverse_parking_controller/path_ros",
            # HH_260814 - Only the selected parking controller publishes, so both
            # entries can stay subscribed without drawing two parking paths.
            "apriltag_parking": "/parking/apriltag_parking_controller/path_ros",
        }
        if wants("trajectory", "docking"):
            for name, topic in maneuver_topics.items():
                if active_view == "docking" and name not in {
                    "drop_zone_parking", "reverse_parking", "apriltag_parking"
                }:
                    continue
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
        if wants("safety", "docking"):
            for name, topic in controller_topics.items():
                if active_view == "docking" and name not in {
                    "drop_zone", "reverse_parking", "apriltag_parking"
                }:
                    continue
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
            "proximity": {"radar", "lidar", "footprint", "safety"},
            "camera": {"cameras"},
            "trajectory": {"localization", "motion", "paths", "footprint"},
            "perception": {"localization", "perception", "footprint"},
            "safety": {"safety"},
            "docking": {"cameras", "motion", "paths", "safety", "docking"},
        }.get(view, set())
        template = cls._new_telemetry_snapshot()
        for section in (
            "gnss", "imu", "radar", "lidar", "cameras", "localization",
            "motion", "paths", "footprint", "perception", "safety", "docking",
        ):
            if section not in allowed_sections:
                snapshot[section] = template[section]

        if view == "proximity":
            # Only the short authoritative radar-cost evidence is needed to
            # classify ECHO versus COST; omit unrelated controller payloads.
            radar_evidence = snapshot["safety"].get("radar_evidence", "")
            snapshot["safety"] = template["safety"]
            snapshot["safety"]["radar_evidence"] = radar_evidence

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

    def _lookup_telemetry_xy_transform(
        self,
        *,
        source_frame_id: str,
        target_frame_id: str,
        stamp: Any,
    ) -> _TelemetryPlanarTransform:
        """Resolve exact planar TF, with an opt-in bounded future fallback."""
        source_frame_id = str(source_frame_id or "").strip()
        target_frame_id = str(target_frame_id or "").strip()

        def unavailable(error: str) -> _TelemetryPlanarTransform:
            return _TelemetryPlanarTransform(
                xy_yaw=None,
                mode="unavailable",
                age_s=None,
                warning="",
                error=error,
            )

        def planar(transform: Any) -> Optional[tuple[float, float, float]]:
            translation = transform.transform.translation
            yaw_deg = _quaternion_yaw_deg(transform.transform.rotation)
            tx = _finite_or_none(translation.x)
            ty = _finite_or_none(translation.y)
            if tx is None or ty is None or yaw_deg is None:
                return None
            return (tx, ty, math.radians(yaw_deg))

        if not source_frame_id:
            return unavailable("source frame_id is empty")
        if not target_frame_id:
            return unavailable("target frame_id is empty")
        if source_frame_id == target_frame_id:
            return _TelemetryPlanarTransform(
                xy_yaw=(0.0, 0.0, 0.0),
                mode="exact",
                age_s=0.0,
                warning="",
                error="",
            )
        try:
            requested_time = Time.from_msg(stamp)
        except Exception as exc:  # noqa: BLE001 - malformed ROS stamps vary.
            return unavailable(f"invalid source timestamp: {type(exc).__name__}: {exc}")
        if requested_time.nanoseconds <= 0:
            return unavailable("source timestamp is zero; exact TF is required")
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame_id,
                source_frame_id,
                requested_time,
            )
            xy_yaw = planar(transform)
            if xy_yaw is None:
                return unavailable("exact TF contains a non-finite planar transform")
            return _TelemetryPlanarTransform(
                xy_yaw=xy_yaw,
                mode="exact",
                age_s=0.0,
                warning="",
                error="",
            )
        except ExtrapolationException as exact_exc:
            tolerance_s = float(
                getattr(self, "telemetry_tf_latest_fallback_tolerance_s", 0.0)
            )
            if tolerance_s <= 0.0:
                return unavailable(f"{type(exact_exc).__name__}: {exact_exc}")

            # Time() requests the latest common time in the complete chain.
            # It is used only to classify an exact-time extrapolation as the
            # observed CARLA future-stamp race; past/stale data is never
            # promoted to current map geometry.
            try:
                latest_transform = self._tf_buffer.lookup_transform(
                    target_frame_id,
                    source_frame_id,
                    Time(),
                )
            except Exception as latest_exc:  # noqa: BLE001 - TF2 subclasses vary.
                return unavailable(
                    f"{type(exact_exc).__name__}: {exact_exc}; latest TF failed: "
                    f"{type(latest_exc).__name__}: {latest_exc}"
                )

            try:
                latest_time = Time.from_msg(latest_transform.header.stamp)
            except Exception as latest_stamp_exc:  # noqa: BLE001
                return unavailable(
                    "latest TF has an invalid timestamp: "
                    f"{type(latest_stamp_exc).__name__}: {latest_stamp_exc}"
                )
            if latest_time.nanoseconds <= 0:
                return unavailable("latest TF timestamp is zero; fallback refused")

            lag_s = (
                requested_time.nanoseconds - latest_time.nanoseconds
            ) / 1_000_000_000.0
            if lag_s <= 0.0:
                return unavailable(
                    "exact TF extrapolation is not a future-stamp race: latest "
                    f"TF is {-lag_s:.6f} s newer than the source timestamp"
                )
            if lag_s > tolerance_s:
                return unavailable(
                    f"latest TF lags the source timestamp by {lag_s:.6f} s, "
                    f"exceeding the {tolerance_s:.6f} s fallback tolerance"
                )

            xy_yaw = planar(latest_transform)
            if xy_yaw is None:
                return unavailable("latest TF contains a non-finite planar transform")
            age_s = round(lag_s, 6)
            return _TelemetryPlanarTransform(
                xy_yaw=xy_yaw,
                mode="latest_fallback",
                age_s=age_s,
                warning=(
                    "exact-time TF was ahead of the latest dynamic transform; "
                    f"using bounded latest TF {age_s:.6f} s behind"
                ),
                error="",
            )
        except Exception as exc:  # noqa: BLE001 - TF2 exceptions vary by distro.
            return unavailable(f"{type(exc).__name__}: {exc}")

    def _warn_telemetry_tf_failure(
        self,
        *,
        stream: str,
        source_frame_id: str,
        target_frame_id: str,
        error: str,
    ) -> None:
        """Log actionable TF failures without flooding the ROS console."""
        now = time.monotonic()
        key = f"{stream}:{source_frame_id}->{target_frame_id}"
        warning_times = getattr(self, "_telemetry_tf_warning_last", None)
        if warning_times is None:
            warning_times = defaultdict(float)
            self._telemetry_tf_warning_last = warning_times
        if now - warning_times[key] < 5.0:
            return
        warning_times[key] = now
        self.get_logger().warning(
            "operator telemetry TF unavailable: "
            f"stream={stream} source={source_frame_id or '<empty>'} "
            f"target={target_frame_id or '<empty>'} error={error}"
        )

    def _on_telemetry_cloud(self, stream: str, message: PointCloud2) -> None:
        now = time.monotonic()
        if (
            now - self._telemetry_last_cloud_decode[stream]
            < self.telemetry_lidar_min_period_s
        ):
            return
        self._telemetry_last_cloud_decode[stream] = now
        if not getattr(self, "telemetry_tf_transform_enabled", False):
            # Preserve origin/develop's map-to-local fallback and payload
            # contract for ordinary CAMROD deployments.
            map_to_local_pose = None
            with self._lock:
                current_pose = dict(self._telemetry["localization"]["pose"])
            if stream != "obstacles" and message.header.frame_id == "map":
                pose_x = _finite_or_none(current_pose.get("x"))
                pose_y = _finite_or_none(current_pose.get("y"))
                pose_yaw = _finite_or_none(current_pose.get("yaw_deg"))
                if (
                    pose_x is not None
                    and pose_y is not None
                    and pose_yaw is not None
                ):
                    map_to_local_pose = (
                        pose_x, pose_y, math.radians(pose_yaw)
                    )
            points = _sample_pointcloud_xy(
                message,
                max_points=self.telemetry_max_lidar_points,
                max_abs_xy_m=self.telemetry_lidar_max_abs_xy_m,
                map_to_local_pose=map_to_local_pose,
            )
            payload = {
                "frame_id": (
                    "robot_center_link"
                    if map_to_local_pose is not None
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
            return

        source_frame_id = str(message.header.frame_id or "").strip()
        target_frame_id = (
            "map" if stream == "obstacles" else "robot_center_link"
        )
        transform_result = self._lookup_telemetry_xy_transform(
            source_frame_id=source_frame_id,
            target_frame_id=target_frame_id,
            stamp=message.header.stamp,
        )
        source_to_target = transform_result.xy_yaw
        if source_to_target is None:
            self._warn_telemetry_tf_failure(
                stream=f"cloud.{stream}",
                source_frame_id=source_frame_id,
                target_frame_id=target_frame_id,
                error=transform_result.error,
            )
            points: List[List[float]] = []
        else:
            points = _sample_pointcloud_xy(
                message,
                max_points=self.telemetry_max_lidar_points,
                max_abs_xy_m=self.telemetry_lidar_max_abs_xy_m,
                source_to_target_transform=source_to_target,
                # Map coordinates are usually much larger than the local
                # display radius. Bound obstacle samples in their sensor frame
                # before applying map<-sensor, then publish the map result.
                filter_before_transform=(target_frame_id == "map"),
            )
        payload = {
            # Never label untransformed sensor coordinates as a browser target
            # frame. An empty frame plus the explicit target/error keeps the UI
            # from rendering plausible-looking but spatially false evidence.
            "frame_id": target_frame_id if source_to_target is not None else "",
            "source_frame_id": source_frame_id,
            "target_frame_id": target_frame_id,
            "transform_available": source_to_target is not None,
            "transform_mode": transform_result.mode,
            "transform_age_s": transform_result.age_s,
            "transform_warning": transform_result.warning,
            "transform_error": transform_result.error,
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
        if not getattr(self, "telemetry_tf_transform_enabled", False):
            geometry = _marker_array_geometry(
                message,
                max_points=self.telemetry_max_grid_cells,
            )
            with self._lock:
                self._telemetry["perception"]["obstacle_boxes"] = geometry
                self._touch_telemetry_locked("perception.boxes")
            return

        active_markers = [
            marker for marker in message.markers
            if marker.action not in {Marker.DELETE, Marker.DELETEALL}
        ]
        source_frames = sorted({
            str(marker.header.frame_id or "").strip()
            for marker in active_markers
        })
        transforms: Dict[str, tuple[float, float, float]] = {}
        transform_results: Dict[str, _TelemetryPlanarTransform] = {}
        failed_transform: Optional[_TelemetryPlanarTransform] = None
        for source_frame_id in source_frames:
            marker = next(
                marker for marker in active_markers
                if str(marker.header.frame_id or "").strip() == source_frame_id
            )
            transform_result = self._lookup_telemetry_xy_transform(
                source_frame_id=source_frame_id,
                target_frame_id="map",
                stamp=marker.header.stamp,
            )
            source_to_map = transform_result.xy_yaw
            if source_to_map is None:
                failed_transform = transform_result
                self._warn_telemetry_tf_failure(
                    stream="obstacle_boxes",
                    source_frame_id=source_frame_id,
                    target_frame_id="map",
                    error=transform_result.error,
                )
                break
            transforms[source_frame_id] = source_to_map
            transform_results[source_frame_id] = transform_result

        transform_available = not active_markers or (
            failed_transform is None and len(transforms) == len(source_frames)
        )
        if transform_available:
            geometry = _marker_array_geometry(
                message,
                max_points=self.telemetry_max_grid_cells,
                source_to_target_transforms=transforms if active_markers else None,
                target_frame_id="map",
            )
        else:
            geometry = {
                "frame_id": "",
                "points": [],
                "polylines": [],
                "point_count": 0,
                "outline_count": 0,
                "cleared": False,
            }
        if not active_markers:
            transform_mode = "not_required"
            transform_age_s: Optional[float] = 0.0
            transform_warning = ""
            transform_error = ""
        elif failed_transform is not None:
            transform_mode = failed_transform.mode
            transform_age_s = failed_transform.age_s
            transform_warning = failed_transform.warning
            transform_error = failed_transform.error
        else:
            transform_mode = (
                "latest_fallback"
                if any(
                    result.mode == "latest_fallback"
                    for result in transform_results.values()
                )
                else "exact"
            )
            transform_age_s = max(
                result.age_s or 0.0 for result in transform_results.values()
            )
            transform_warning = "; ".join(
                result.warning
                for result in transform_results.values()
                if result.warning
            )
            transform_error = ""

        geometry.update({
            "source_frame_id": source_frames[0] if len(source_frames) == 1 else "",
            "source_frame_ids": source_frames,
            "target_frame_id": "map",
            "transform_available": transform_available,
            "transform_mode": transform_mode,
            "transform_age_s": transform_age_s,
            "transform_warning": transform_warning,
            "transform_error": transform_error,
        })
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

    def _on_telemetry_docking_tag_pose(
        self, message: AvgAprilTagPose
    ) -> None:
        """Record the camera-relative tag vector used by docking control."""
        # HH_260818 - Publish the exact controller input instead of deriving a
        # distance from the debug image, so the UI and controller remain auditable.
        position = message.pose.pose.position
        x = float(position.x)
        y = float(position.y)
        z = float(position.z)
        distance_m = math.sqrt(x * x + y * y + z * z)
        with self._lock:
            self._telemetry["docking"]["tag"] = {
                "family": str(message.family),
                "id": int(message.id),
                "tag_frame": str(message.tag_frame),
                "camera_frame": str(message.pose.header.frame_id),
                "x_m": _finite_or_none(x),
                "y_m": _finite_or_none(y),
                "z_m": _finite_or_none(z),
                "distance_m": _finite_or_none(distance_m),
                "yaw_deg": _quaternion_yaw_deg(message.pose.pose.orientation),
            }
            self._touch_telemetry_locked("docking.tag_pose")

    def _on_telemetry_docking_tag_detected(self, message: AvgBool) -> None:
        # HH_260818 - Keep detection validity separate from the most recent pose;
        # a stale pose must never look like a currently visible docking target.
        with self._lock:
            self._telemetry["docking"]["tag_detected"] = bool(message.data)
            self._touch_telemetry_locked("docking.tag_detected")

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
            self._telemetry["docking"]["is_charging"] = bool(message.is_charging)
            self._telemetry["docking"]["battery_percentage"] = (
                _finite_or_none(float(message.battery_percentage) * 100.0)
                if message.battery_state_available else None
            )
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

    @staticmethod
    def _route_goal_stamp_key(message: Any) -> tuple[int, int]:
        stamp = message.header.stamp
        return (int(stamp.sec), int(stamp.nanosec))

    @staticmethod
    def _finite_planar_pose(message: Any) -> bool:
        try:
            values = (
                float(message.pose.position.x),
                float(message.pose.position.y),
            )
        except (AttributeError, TypeError, ValueError):
            return False
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _planar_pose_yaw(message: Any) -> Optional[float]:
        try:
            quaternion = tuple(
                float(value)
                for value in (
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                )
            )
        except (AttributeError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in quaternion):
            return None
        norm_sq = sum(component * component for component in quaternion)
        if not math.isfinite(norm_sq) or norm_sq <= 1.0e-12:
            return None
        inverse_norm = 1.0 / math.sqrt(norm_sq)
        qx, qy, qz, qw = (
            component * inverse_norm for component in quaternion
        )
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return yaw if math.isfinite(yaw) else None

    def _remember_pending_site_route_goal(
        self, raw_goal: PoseStamped, mission_key: str
    ) -> None:
        stamp_key = self._route_goal_stamp_key(raw_goal)
        with self._lock:
            if stamp_key not in self._pending_site_route_goal_stamps:
                self._pending_site_route_goal_stamps[stamp_key] = mission_key
            elif self._pending_site_route_goal_stamps[stamp_key] != mission_key:
                # A paused simulation clock can produce duplicate stamps. An
                # ambiguous stamp authorizes neither mission.
                previous_mission = self._pending_site_route_goal_stamps[
                    stamp_key
                ]
                self._site_route_anchors.pop(previous_mission, None)
                self._site_route_anchors.pop(mission_key, None)
                self._pending_site_route_goal_stamps[stamp_key] = ""
            while (
                len(self._pending_site_route_goal_stamps)
                > MAX_PENDING_SITE_ROUTE_GOALS
            ):
                oldest = next(iter(self._pending_site_route_goal_stamps))
                self._pending_site_route_goal_stamps.pop(oldest, None)

    def _roadside_operational_arrival_match(
        self,
        mission_key: str,
        keypoint: MissionKeypoint,
        px: float,
        py: float,
        pose_frame: str,
    ) -> tuple[bool, float, str]:
        with self._lock:
            route_goal = self._site_route_anchors.get(mission_key)
            lanelet_pose = self._latest_planning_lanelet_pose
            lanelet_pose_time_s = self._latest_planning_lanelet_pose_time_s
        if route_goal is None:
            return False, float("inf"), "missing_mission_route_anchor"
        if lanelet_pose is None:
            return False, float("inf"), "missing_fresh_lanelet_anchor"
        lanelet_age_s = self._now_s() - lanelet_pose_time_s
        if (
            not math.isfinite(lanelet_age_s)
            or lanelet_age_s < 0.0
            or not math.isfinite(self.site_arrival_pose_timeout_s)
            or self.site_arrival_pose_timeout_s <= 0.0
        ):
            return False, float("inf"), "invalid_lanelet_anchor_freshness"
        if lanelet_age_s > self.site_arrival_pose_timeout_s:
            return (
                False,
                float("inf"),
                f"stale_lanelet_anchor:{lanelet_age_s:.2f}s",
            )

        route_frame = str(route_goal.header.frame_id or self.default_goal_frame_id)
        lanelet_frame = str(
            lanelet_pose.header.frame_id or self.default_goal_frame_id
        )
        goal_frame = str(keypoint.frame_id or self.default_goal_frame_id)
        frames = {
            frame
            for frame in (pose_frame, route_frame, lanelet_frame, goal_frame)
            if frame
        }
        if len(frames) > 1:
            return False, float("inf"), "roadside_frame_mismatch"

        if (
            not self._finite_planar_pose(route_goal)
            or not self._finite_planar_pose(lanelet_pose)
            or not all(
                math.isfinite(value)
                for value in (px, py, float(keypoint.x), float(keypoint.y))
            )
        ):
            return False, float("inf"), "invalid_roadside_geometry"
        route_yaw = self._planar_pose_yaw(route_goal)
        if route_yaw is None:
            return False, float("inf"), "invalid_roadside_route_yaw"

        requested_offset = float(self.site_arrival_roadside_offset_m)
        lateral_tolerance = float(
            self.site_arrival_roadside_lateral_tolerance_m
        )
        forward_tolerance = float(
            self.site_arrival_roadside_forward_tolerance_m
        )
        if (
            not all(
                math.isfinite(value)
                for value in (
                    requested_offset,
                    lateral_tolerance,
                    forward_tolerance,
                )
            )
            or requested_offset <= 0.0
            or lateral_tolerance < 0.0
            or forward_tolerance < 0.0
        ):
            return False, float("inf"), "invalid_roadside_arrival_policy"

        anchor_x = float(route_goal.pose.position.x)
        anchor_y = float(route_goal.pose.position.y)
        cos_yaw = math.cos(route_yaw)
        sin_yaw = math.sin(route_yaw)

        def relative_axes(x: float, y: float) -> tuple[float, float]:
            dx = x - anchor_x
            dy = y - anchor_y
            return (
                cos_yaw * dx + sin_yaw * dy,
                -sin_yaw * dx + cos_yaw * dy,
            )

        raw_forward, raw_lateral = relative_axes(
            float(keypoint.x), float(keypoint.y)
        )
        if (
            not math.isfinite(raw_forward)
            or not math.isfinite(raw_lateral)
            or abs(raw_forward) > forward_tolerance + 1.0e-6
            or abs(raw_lateral) < ROADSIDE_MINIMUM_SITE_LATERAL_M
        ):
            return False, float("inf"), "invalid_roadside_goal_pair"

        # The live lanelet projection must still agree with this mission's
        # timestamp-correlated snap.  This rejects a stale snap from another
        # campsite even if its raw center happens to be nearby.
        lanelet_forward, lanelet_lateral = relative_axes(
            float(lanelet_pose.pose.position.x),
            float(lanelet_pose.pose.position.y),
        )
        if (
            abs(lanelet_forward) > forward_tolerance + 1.0e-6
            or abs(lanelet_lateral) > lateral_tolerance + 1.0e-6
        ):
            return False, float("inf"), "lanelet_anchor_disagrees_with_route"

        operational_offset = min(abs(raw_lateral), requested_offset)
        signed_target_lateral = math.copysign(
            operational_offset, raw_lateral
        )
        current_forward, current_lateral = relative_axes(px, py)
        lateral_error = current_lateral - signed_target_lateral
        distance = math.hypot(current_forward, lateral_error)
        matches = (
            abs(current_forward) <= forward_tolerance + 1.0e-6
            and abs(lateral_error) <= lateral_tolerance + 1.0e-6
        )
        return (
            matches,
            distance,
            "near_roadside_operational_target"
            if matches
            else "outside_roadside_operational_target",
        )

    def _site_arrival_match(
        self,
        site: str,
        *,
        force_roadside: bool = False,
        respect_immediate_arrival_policy: bool = True,
    ) -> tuple[bool, str, float, str]:
        mission_key = self._resolve_mission_key_for_site(site) or ""
        keypoint = self._keypoints_by_mission_key.get(mission_key)
        # ``immediate_site_arrival_enabled`` controls only the dispatch-time
        # fast-adopt path (selecting a site while the robot is already there).
        # A controller's later arrival heartbeat must always be checked against
        # the physical pose; disabling fast-adopt must not make every genuine
        # controller arrival impossible.
        if (
            respect_immediate_arrival_policy
            and not self.immediate_site_arrival_enabled
        ):
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
        if not math.isfinite(px) or not math.isfinite(py):
            return False, mission_key, float("inf"), "nonfinite_pose"
        if force_roadside or str(keypoint.service_mode).strip().lower() == "roadside_stop":
            matched, distance, reason = self._roadside_operational_arrival_match(
                mission_key, keypoint, px, py, pose_frame
            )
            return matched, mission_key, distance, reason
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
            # A newly connected socket first receives one atomic snapshot and
            # lives in ``_ws_initializing_clients`` until that frame is sent.
            # Do not drop state transitions in that interval: ``_broadcast``
            # appends them to the socket's initialization queue.
            if not self._ws_clients and not getattr(
                self, "_ws_initializing_clients", {}
            ):
                return
        asyncio.run_coroutine_threadsafe(
            self._broadcast(payload), self._main_loop
        )

    async def _broadcast(self, payload: dict) -> None:
        with self._ws_clients_lock:
            clients = list(self._ws_clients)
            for queued in getattr(
                self, "_ws_initializing_clients", {}
            ).values():
                queued.append(copy.deepcopy(payload))
        for client in clients:
            try:
                await UiBackendNode._send_ws_json(self, client, payload)
            except Exception:
                with self._ws_clients_lock:
                    self._ws_clients.discard(client)
                    getattr(self, "_ws_client_send_locks", {}).pop(
                        client, None
                    )

    async def _send_ws_json(self, client: WebSocket, payload: dict) -> None:
        """Serialize every control-socket frame for one Robot UI client."""
        with self._ws_clients_lock:
            send_lock = getattr(self, "_ws_client_send_locks", {}).get(client)
        if send_lock is None:
            # Focused unit fixtures predating live WebSocket registration.
            await client.send_json(payload)
            return
        async with send_lock:
            await client.send_json(payload)

    def _mission_dispatch_snapshot(
        self, service_state: Optional[int] = None
    ) -> Dict[str, Any]:
        """Return the authoritative active mission shown after UI reconnect."""
        with self._lock:
            site = str(getattr(self, "_active_mission_site", "")).strip()
            source = str(getattr(self, "_active_mission_source", "")).strip()
            generation = int(
                getattr(self, "_active_mission_generation", 0)
            )
        retryable = bool(
            site and getattr(self, "_active_mission_retryable", False)
        )
        # Mission identity is committed synchronously before the service-state
        # publication loops back through DDS. Treat that exact site+generation
        # claim as authority for the immediate HTTP/WebSocket ACK; otherwise a
        # fast response can transiently report gen0/empty while the ROS echo is
        # still queued. Terminal and Stop paths clear identity before taking a
        # snapshot, so this cannot resurrect a completed mission.
        active = bool(site) and generation > 0
        intent = (
            "recall"
            if active and UiBackendNode._is_guest_recall_source(source)
            else ("delivery" if active else "")
        )
        return {
            "mission_dispatch_active": active,
            "mission_dispatch_site": site if active else "",
            "mission_dispatch_intent": intent,
            "mission_dispatch_owner": (
                UiBackendNode._destination_request_owner(source)
                if active
                else ""
            ),
            "mission_dispatch_generation": generation if active else 0,
            "mission_retryable": retryable,
            "mission_retry_site": site if retryable else "",
            "mission_retry_owner": (
                UiBackendNode._destination_request_owner(source)
                if retryable
                else ""
            ),
            "departure_failed": retryable,
        }

    def _active_mission_identity(self) -> tuple[str, str, int, int]:
        """Read mission identity from production nodes or lightweight tests."""
        state_lock = getattr(self, "_lock", None)

        def snapshot() -> tuple[str, str, int, int]:
            return (
                str(getattr(self, "_active_mission_site", "")).strip(),
                str(getattr(self, "_active_mission_source", "")).strip(),
                int(getattr(self, "_active_mission_generation", 0)),
                int(getattr(self, "_return_requested_generation", 0)),
            )

        if state_lock is None:
            return snapshot()
        with state_lock:
            return snapshot()

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
        if current[:2] != previous[:2]:
            # HH_260814 - The blocking reasons previously reached the operator
            # only over the telemetry WebSocket, so a console-only session had
            # no way to see why the mission display stayed in INITIALIZING.
            # Throttle it: a flapping sensor rate can rewrite the reason set
            # several times per second.
            self._log_readiness_transition(current[0], current[1])
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

    def _log_readiness_transition(self, ready: bool, message: str) -> None:
        """Surface readiness blockers on the console, at most once per second."""
        now = time.monotonic()
        if ready:
            self.get_logger().info("UI readiness: ready")
            self._readiness_log_last_sec = now
            return
        if now - self._readiness_log_last_sec < 1.0:
            return
        self._readiness_log_last_sec = now
        self.get_logger().warning(f"UI readiness blocked by: {message}")

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

    def _on_planning_route_goal(self, msg: PoseStamped) -> None:
        # goal_snapper preserves the raw goal timestamp.  Only that exact
        # correspondence may populate a campsite return anchor; manual/RViz or
        # another campsite's latest route must never authorize roadside adopt.
        stamp_key = self._route_goal_stamp_key(msg)
        mission_key = ""
        with self._lock:
            mission_key = self._pending_site_route_goal_stamps.pop(
                stamp_key, ""
            )
            if mission_key:
                self._site_route_anchors[mission_key] = copy.deepcopy(msg)
        if mission_key:
            self.get_logger().info(
                "cached timestamp-correlated campsite route anchor: "
                f"mission_key={mission_key} "
                f"xy=({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})"
            )
        self._update_runtime_state(
            self._runtime_policy.update_goal_received
        )

    def _on_planning_lanelet_pose(self, msg: AvgPoseStamped) -> None:
        with self._lock:
            # Match the existing localization-pose cache: rclpy hands this
            # callback an owned message object, so retaining it avoids a 10 Hz
            # deep-copy allocation on ARM64.
            self._latest_planning_lanelet_pose = msg
            self._latest_planning_lanelet_pose_time_s = self._now_s()

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
        """Serialize charger lifecycle edges against mission admission/Stop."""
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._on_platform_status_serialized(self, msg)
        with dispatch_lock:
            return UiBackendNode._on_platform_status_serialized(self, msg)

    def _on_platform_status_serialized(self, msg: AvgPlatformStatus) -> None:
        # HH_260720 - UI battery state comes from the canonical generated platform status.
        # HH_260721 - Charging state also decides whether a campsite goal must wait for departure.
        control_mode = int(msg.control_mode)
        charging = bool(msg.is_charging)
        redock_lock = getattr(self, "_redock_after_disconnect_lock", None)
        if redock_lock is None:
            redock_lock = threading.RLock()
            self._redock_after_disconnect_lock = redock_lock
        # The release edge and request-side recheck use this same lock.  This
        # closes the check/queue window where a false edge could otherwise be
        # consumed immediately before the pending request became visible.
        with redock_lock:
            previous_control_mode = int(
                getattr(self, "_latest_platform_control_mode", -1)
            )
            previous_charging = bool(
                getattr(self, "_latest_platform_is_charging", False)
            )
            self._latest_platform_control_mode = control_mode
            self._latest_platform_is_charging = charging
            charging_changed = charging != previous_charging
            can_resume_redock = (
                control_mode == 1
                and previous_control_mode != 1
                and bool(getattr(self, "_parking_rearm_waiting_for_can", False))
            )
        self._update_runtime_state(
            lambda: self._runtime_policy.update_platform(
                estop=bool(msg.estop),
                error_code=int(msg.error_code),
            )
        )
        if getattr(self, "_telemetry_capture_active", False):
            UiBackendNode._record_telemetry_platform(self, msg)
        service_metrics = getattr(self, "_service_metrics", None)
        if service_metrics is not None:
            sample_time_s = _ros_stamp_seconds(msg.velocity.header.stamp)
            if sample_time_s is None or sample_time_s <= 0.0:
                sample_time_s = _ros_stamp_seconds(msg.stamp)
            if sample_time_s is None or sample_time_s <= 0.0:
                sample_time_s = self._now_s()
            service_metrics.observe_velocity(
                msg.velocity.twist.linear.x,
                msg.velocity.twist.linear.y,
                sample_time_s,
            )
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
        if charging_changed and not charging:
            redock_request = UiBackendNode._take_pending_redock_after_disconnect(self)
            if redock_request:
                redock_source, redock_generation = redock_request
                alignment_started = UiBackendNode._start_drop_zone_parking_alignment(
                    self,
                    source=f"{redock_source}:charging_released",
                    generation=redock_generation,
                )
                if alignment_started:
                    UiBackendNode._arm_stationary_drop_zone_return_progress(self)
        if can_resume_redock or (charging_changed and not charging):
            can_request = UiBackendNode._take_parking_rearm_waiting_for_can(self)
            if can_request:
                redock_source, redock_generation = can_request
                alignment_started = UiBackendNode._start_drop_zone_parking_alignment(
                    self,
                    source=f"{redock_source}:can_restored",
                    generation=redock_generation,
                )
                if alignment_started:
                    UiBackendNode._arm_stationary_drop_zone_return_progress(self)
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
    def _charging_departure_delay_required(self) -> bool:
        delay_s = float(getattr(self, "charging_departure_delay_s", 0.0))
        if delay_s <= 0.0:
            return False
        state = getattr(self, "_latest_service_state", None)
        if state in {
            int(AvgServiceState.DEPARTING_CHARGER),
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        }:
            return False
        return bool(getattr(self, "_latest_platform_is_charging", False)) or state in {
            int(AvgServiceState.CHARGING),
            int(AvgServiceState.WAITING_FOR_CHARGING),
        }

    def _hold_charging_departure_motion(self, source: str) -> None:
        # HH_260825 - Close manual, mission, and platform gates independently.
        # This prevents a stale manual latch from defeating the visible dwell.
        if getattr(self, "publish_mission_engage_from_destination", False):
            self._publish_mission_engage(
                False, source=f"{source}:charging_departure_delay"
            )
        self._publish_engage(
            False,
            source=f"{source}:charging_departure_delay",
            sync_drive_enable=False,
        )
        self._publish_platform_drive_enable(
            False, source=f"{source}:charging_departure_delay"
        )

    def _schedule_charging_departure_transition(self, source: str) -> bool:
        delay_s = float(getattr(self, "charging_departure_delay_s", 0.0))
        if delay_s <= 0.0:
            return UiBackendNode._begin_drop_zone_departure(self, source)
        lock = getattr(self, "_charging_departure_transition_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._charging_departure_transition_lock = lock
        with lock:
            if getattr(self, "_charging_departure_delay_pending", False):
                return True
            self._charging_departure_delay_pending = True
            self._charging_departure_transition_timer = self.create_timer(
                delay_s,
                lambda: UiBackendNode._complete_charging_departure_transition(self),
            )
        UiBackendNode._hold_charging_departure_motion(self, source)
        self._schedule_broadcast(
            {
                "departure_delay_active": True,
                "departure_delay_seconds": delay_s,
                "message": f"Charging departure starts after {delay_s:.1f} s safety dwell",
            }
        )
        self.get_logger().info(
            "charging departure safety dwell started: "
            f"delay={delay_s:.1f}s source={source}"
        )
        return True

    def _complete_charging_departure_transition(self) -> None:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._complete_charging_departure_transition_serialized(
                self
            )
        with dispatch_lock:
            return UiBackendNode._complete_charging_departure_transition_serialized(
                self
            )

    def _complete_charging_departure_transition_serialized(self) -> None:
        lock = getattr(self, "_charging_departure_transition_lock", None)
        if lock is None:
            return
        with lock:
            if not getattr(self, "_charging_departure_delay_pending", False):
                return
            timer = getattr(self, "_charging_departure_transition_timer", None)
            self._charging_departure_delay_pending = False
            self._charging_departure_transition_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        pending = getattr(self, "_pending_site_after_drop_zone_exit", None)
        if pending is None:
            return
        UiBackendNode._begin_drop_zone_departure(self, pending[2])

    def _cancel_pending_charging_departure_transition(self, reason: str) -> None:
        lock = getattr(self, "_charging_departure_transition_lock", None)
        if lock is None:
            self._charging_departure_delay_pending = False
            self._charging_departure_transition_timer = None
            self._charging_departure_from_charger = False
            return
        with lock:
            timer = getattr(self, "_charging_departure_transition_timer", None)
            was_pending = bool(
                getattr(self, "_charging_departure_delay_pending", False)
            )
            self._charging_departure_delay_pending = False
            self._charging_departure_transition_timer = None
            self._charging_departure_from_charger = False
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        if was_pending:
            self.get_logger().info(
                f"pending charging departure cancelled: reason={reason}"
            )

    def _begin_drop_zone_departure(self, source: str) -> bool:
        pending = getattr(self, "_pending_site_after_drop_zone_exit", None)
        if pending is None:
            return False
        if getattr(self, "_drop_zone_exit_active", False):
            return True

        UiBackendNode._cancel_pending_redock_after_disconnect(
            self, "drop_zone_departure"
        )
        UiBackendNode._cancel_pending_parking_rearm_transition(
            self, "drop_zone_departure"
        )

        self._drop_zone_exit_active = True
        self._drop_zone_exit_handoff_ready = False
        self._drop_zone_exit_failure_latched = False
        self._drop_zone_exit_cancel_suppressed = False
        state = getattr(self, "_latest_service_state", None)
        resumed_active_departure = state in {
            int(AvgServiceState.DEPARTING_CHARGER),
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        }
        if not resumed_active_departure:
            # CANCEL and EXIT share this UI publisher, so DDS preserves their
            # order.  Always clear a just-started re-dock owner here: its first
            # DROP_ZONE_PARKING service heartbeat may race the destination
            # request and need not have updated `state` yet.
            self._publish_drop_zone_operation(
                MotionOperation.CANCEL,
                source=f"{source}:site_departure_preempt_alignment",
            )
        if not resumed_active_departure:
            self._publish_parking_operation(
                MotionOperation.CANCEL, source=f"{source}:site_departure"
            )
            self._drop_zone_exit_waiting_for_fresh_status = True

        # HH_260825 - Open authorization only after the dwell has expired, then
        # start the departure owner. Dynamic radar/fusion cost checks stay active
        # in EXIT_STRAIGHT and ALIGN_EXIT_YAW; only static lanelet cost is bypassed.
        if getattr(self, "publish_engage_from_destination", False):
            self._publish_engage(True, source=f"{source}:site_departure")
        if getattr(self, "publish_mission_engage_from_destination", False):
            self._publish_mission_engage(
                True, source=f"{source}:site_departure"
            )
        if not resumed_active_departure:
            self._publish_drop_zone_operation(
                MotionOperation.EXIT, source=f"{source}:site_departure"
            )
        departure_state = (
            AvgServiceState.DEPARTING_CHARGER
            if bool(getattr(self, "_charging_departure_from_charger", False))
            or bool(getattr(self, "_latest_platform_is_charging", False))
            else AvgServiceState.DEPARTING_DROP_ZONE
        )
        self._schedule_broadcast(
            {
                "departure_delay_active": False,
                "departure_delay_seconds": 0.0,
            }
        )
        self._publish_service_state(
            departure_state, source=f"{source}:drop_zone_departure"
        )
        self.get_logger().info(
            f"drop-zone departure released after safety dwell: source={source}"
        )
        return True

    def _mark_drop_zone_exit_failed(self, source: str) -> None:
        safe_state = (
            AvgServiceState.CHARGING
            if self._latest_platform_is_charging
            else AvgServiceState.DROP_ZONE_WAIT
        )
        already_safe = (
            getattr(self, "_drop_zone_exit_failure_latched", False)
            and getattr(self, "_latest_service_state", None) == int(safe_state)
        )
        self._drop_zone_exit_failure_latched = True
        self._drop_zone_exit_active = False
        self._drop_zone_exit_handoff_ready = False
        self._charging_departure_from_charger = False
        if already_safe:
            return
        if getattr(self, "publish_mission_engage_from_destination", False):
            self._publish_mission_engage(False, source=source)
        active_site, active_source, active_generation, _ = (
            UiBackendNode._active_mission_identity(self)
        )
        if active_site:
            self._active_mission_retryable = True
        site_names = list(getattr(self, "site_names", []))
        cleared_states = {site: False for site in site_names}
        state = getattr(self, "_state", None)
        state_lock = getattr(self, "_lock", None)
        if state is not None and state_lock is not None:
            with state_lock:
                state.ws_site_states = dict(cleared_states)
                state.destination = {"site": "", "run": False}
        elif state is not None:
            state.ws_site_states = dict(cleared_states)
            state.destination = {"site": "", "run": False}
        self._schedule_broadcast({
            "departure_failed": True,
            "mission_retryable": bool(active_site),
            "mission_retry_site": active_site,
            "mission_retry_owner": UiBackendNode._destination_request_owner(
                active_source
            ),
            "states": cleared_states,
            "message": "Drop-zone exit failed; select the destination again to retry",
        })
        if active_site:
            UiBackendNode._publish_destination_dispatch_status(
                self,
                active_site,
                True,
                active_source,
                {
                    "blocked": True,
                    "error": "drop_zone_exit_failed",
                    "message": "drop-zone exit failed; same mission may be retried",
                    "retryable": True,
                },
            )
        self._publish_service_state(safe_state, source=source)

    def _on_drop_zone_maneuver_status(self, msg: ModuleState) -> None:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._on_drop_zone_maneuver_status_serialized(
                self, msg
            )
        with dispatch_lock:
            return UiBackendNode._on_drop_zone_maneuver_status_serialized(
                self, msg
            )

    def _on_drop_zone_maneuver_status_serialized(
        self, msg: ModuleState
    ) -> None:
        operating_state = str(msg.operating_state).strip()
        if operating_state in {"EXIT_STRAIGHT", "ALIGN_EXIT_YAW"}:
            if getattr(self, "_drop_zone_exit_cancel_suppressed", False):
                # Operator stop owns the motion gate until a new human request.
                # Queued controller heartbeats cannot reconstruct departure.
                return
            if getattr(self, "_drop_zone_exit_failure_latched", False):
                if not getattr(
                    self, "_drop_zone_exit_waiting_for_fresh_status", False
                ):
                    # ERROR/failure is terminal for this attempt.  A delayed
                    # pre-error heartbeat cannot reconstruct DEPARTING afterward.
                    return
                # A human retry can race an old false/ERROR from the prior
                # attempt. The first later EXIT status on this ordered writer
                # proves that the controller accepted the new operation.
                self._drop_zone_exit_failure_latched = False
                self._drop_zone_exit_active = True
                active_site, active_source, _, _ = (
                    UiBackendNode._active_mission_identity(self)
                )
                self._active_mission_retryable = False
                restored_states = {
                    site: site == active_site for site in getattr(self, "site_names", [])
                }
                state = getattr(self, "_state", None)
                state_lock = getattr(self, "_lock", None)
                if state is not None and state_lock is not None:
                    with state_lock:
                        state.ws_site_states = dict(restored_states)
                        state.destination = {
                            "site": active_site,
                            "run": bool(active_site),
                        }
                elif state is not None:
                    state.ws_site_states = dict(restored_states)
                    state.destination = {
                        "site": active_site,
                        "run": bool(active_site),
                    }
                self._schedule_broadcast({
                    "departure_failed": False,
                    "mission_retryable": False,
                    "mission_retry_site": "",
                    "mission_retry_owner": "",
                    "states": restored_states,
                })
                if active_site:
                    UiBackendNode._publish_destination_dispatch_status(
                        self,
                        active_site,
                        True,
                        active_source,
                        {"message": "drop-zone exit retry accepted by controller"},
                    )
            self._drop_zone_exit_waiting_for_fresh_status = False
            if getattr(self, "_drop_zone_exit_handoff_ready", False):
                # This status writer's pre-terminal phase can be delivered
                # after exit_complete.  A new UI-owned exit clears the flag
                # before publishing EXIT, so only an old heartbeat is ignored.
                return
            self._drop_zone_exit_handoff_ready = False
            if self._latest_service_state not in {
                int(AvgServiceState.DEPARTING_CHARGER),
                int(AvgServiceState.DEPARTING_DROP_ZONE),
            }:
                recovered = AvgServiceState()
                recovered.state = (
                    AvgServiceState.DEPARTING_CHARGER
                    if self._latest_platform_is_charging
                    else AvgServiceState.DEPARTING_DROP_ZONE
                )
                recovered.state_name = (
                    "DEPARTING_CHARGER"
                    if self._latest_platform_is_charging
                    else "DEPARTING_DROP_ZONE"
                )
                recovered.description = (
                    f"drop_zone_maneuver_controller:{operating_state}:status recovery"
                )
                self._on_service_state(recovered)
            return
        if operating_state == "ERROR":
            if getattr(self, "_drop_zone_exit_cancel_suppressed", False):
                return
            # ERROR is the controller's persistent, ordered failure terminal.
            # It is authoritative even after a UI restart; Bool(false) is not.
            self._mark_drop_zone_exit_failed(source="drop_zone_status:error")
            return
        if operating_state == "IDLE":
            # IDLE is both the normal pre-start heartbeat and the post-cancel
            # state, so it is never a departure terminal.
            return
        if operating_state == "ROAD_HANDOFF_READY":
            if getattr(self, "_drop_zone_exit_cancel_suppressed", False):
                return
            if getattr(self, "_drop_zone_exit_waiting_for_fresh_status", False):
                self.get_logger().warn(
                    "stale ROAD_HANDOFF_READY ignored before current EXIT status"
                )
                return
            if (
                getattr(self, "_drop_zone_exit_failure_latched", False)
                and not getattr(
                    self, "_drop_zone_exit_waiting_for_fresh_status", False
                )
            ):
                return
            self._drop_zone_exit_failure_latched = False
            self._drop_zone_exit_waiting_for_fresh_status = False
            terminal = AvgServiceState()
            terminal.state = AvgServiceState.DROP_ZONE_WAIT
            terminal.state_name = "ROAD_HANDOFF_READY"
            terminal.description = "Drop-zone exit complete; ready for route dispatch"
            self._on_service_state(terminal)

    def _on_parking_controller_status(
        self, controller: str, msg: ModuleState
    ) -> None:
        """Remember retryable parking ERROR independently from service state."""
        lock = getattr(self, "_redock_after_disconnect_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._redock_after_disconnect_lock = lock
        with lock:
            states = getattr(self, "_parking_controller_operating_states", None)
            if states is None:
                states = {}
                self._parking_controller_operating_states = states
            states[str(controller)] = str(msg.operating_state).strip().upper()

    def _on_drop_zone_exit_complete(self, msg: AvgBool) -> None:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._on_drop_zone_exit_complete_serialized(self, msg)
        with dispatch_lock:
            return UiBackendNode._on_drop_zone_exit_complete_serialized(self, msg)

    def _on_drop_zone_exit_complete_serialized(self, msg: AvgBool) -> None:
        if not bool(msg.data):
            # This topic is intentionally observation-only for failures:
            # startExit() also emits false when an exit is already active, and
            # cross-topic delivery can carry an old false into a new attempt.
            # The ordered, persistent ModuleState ERROR owns failure handling.
            self.get_logger().warn(
                "drop-zone exit_complete=false observed; awaiting authoritative "
                "drop-zone ModuleState ERROR"
            )
            return
        if getattr(self, "_drop_zone_exit_waiting_for_fresh_status", False):
            # Cross-topic completion from a failed prior attempt cannot release
            # a newly retried site. The ordered ModuleState writer must first
            # acknowledge this attempt with EXIT_STRAIGHT/ALIGN_EXIT_YAW.
            self.get_logger().warn(
                "drop-zone exit_complete=true ignored while waiting for fresh "
                "current-attempt status"
            )
            return
        self._drop_zone_exit_failure_latched = False
        self._drop_zone_exit_waiting_for_fresh_status = False
        if not self._drop_zone_exit_active:
            departure_states = {
                int(AvgServiceState.DEPARTING_CHARGER),
                int(AvgServiceState.DEPARTING_DROP_ZONE),
            }
            if getattr(self, "_latest_service_state", None) not in departure_states:
                return
            # HH_260819 - The UI may restart and recover only the controller's
            # DEPARTING heartbeat before this completion.  Remember a
            # successful road handoff even without a pending site; otherwise a
            # stale DEPARTING value would make the next selection wait forever.
            self._drop_zone_exit_handoff_ready = True
            self._latest_service_state = None
            with self._lock:
                self._state.service_state = int(AvgServiceState.DROP_ZONE_WAIT)
                self._state.service_state_name = "ROAD_HANDOFF_READY"
                self._state.service_state_description = (
                    "Drop-zone exit complete; ready for route dispatch"
                )
            self._schedule_broadcast({
                "departure_complete": True,
                "service_state": int(AvgServiceState.DROP_ZONE_WAIT),
                "service_state_name": "ROAD_HANDOFF_READY",
                "service_state_description": (
                    "Drop-zone exit complete; ready for route dispatch"
                ),
            })
            return
        pending = self._pending_site_after_drop_zone_exit
        self._drop_zone_exit_active = False
        self._pending_site_after_drop_zone_exit = None
        self._drop_zone_exit_handoff_ready = True
        self._charging_departure_from_charger = False
        if pending is None:
            return
        site, mission_key, source = pending
        if UiBackendNode._is_guest_recall_source(source):
            recall_published = UiBackendNode._publish_planning_camping_site_recall(
                self, mission_key, source
            )
            self.get_logger().info(
                "drop-zone departure complete; released campsite recall: "
                f"site={site} mission_key={mission_key} "
                f"published={str(recall_published).lower()}"
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
        """Serialize terminal reconciliation against destination admission."""
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._on_service_state_serialized(self, msg)
        with dispatch_lock:
            return UiBackendNode._on_service_state_serialized(self, msg)

    def _on_service_state_serialized(self, msg: AvgServiceState) -> None:
        state = int(msg.state)
        state_name = str(msg.state_name).strip() or SERVICE_STATE_NAMES.get(
            state, f"UNKNOWN_{state}"
        )
        raw_description = str(msg.description).strip() or state_name
        description, self_echo_generation, self_echo_epoch = (
            UiBackendNode._decode_service_state_echo(self, raw_description)
        )
        current_generation = int(
            getattr(self, "_active_mission_generation", 0)
        )
        current_epoch = UiBackendNode._current_command_epoch(self)
        if self_echo_epoch is not None and (
            self_echo_epoch != current_epoch
            or self_echo_generation != current_generation
        ):
            # This check must precede every visible-state, engage, destination,
            # and `_latest_service_state` mutation. A Stop request clears the
            # old generation before publishing OPERATOR_STOPPED; if a new
            # mission is admitted before DDS delivers that self echo, it is old.
            self.get_logger().warn(
                "stale self-published service state ignored: "
                f"published_generation={self_echo_generation} "
                f"active_generation={current_generation} "
                f"published_epoch={self_echo_epoch} "
                f"active_epoch={current_epoch} state={state_name}"
            )
            return
        if (
            state == int(AvgServiceState.OPERATOR_STOPPED)
            and self_echo_epoch is None
        ):
            # An OPERATOR_STOPPED message from any other writer is global
            # fail-safe authority, including while a generation-0 manual goal,
            # manual-drive lease, or standalone Return owns motion.  Handle it
            # before the generation-0 heartbeat filter so a pending Return
            # timer cannot re-engage after the stop.  A current self echo is
            # excluded because the originating stop already performed this
            # full cancellation.
            UiBackendNode._stop_active_service_serialized(
                self,
                "service_state:OPERATOR_STOPPED",
                publish_service_state=False,
            )
        if (
            bool(getattr(self, "_startup_recovery_pending", False))
            and state != int(AvgServiceState.OPERATOR_STOPPED)
        ):
            self.get_logger().warn(
                "service-state heartbeat ignored during backend startup "
                f"recovery: state={state_name}"
            )
            return
        generation_zero_authority = str(
            getattr(self, "_generation_zero_authority", "")
        ).strip()
        if generation_zero_authority == "standalone_return":
            own_current_echo = bool(
                self_echo_epoch == current_epoch
                and self_echo_generation == 0
            )
            parking_state = state == int(AvgServiceState.DROP_ZONE_PARKING)
            terminal_state = state in {
                int(AvgServiceState.DROP_ZONE_WAIT),
                int(AvgServiceState.WAITING_FOR_CHARGING),
                int(AvgServiceState.CHARGING),
            }
            physical_match = False
            if parking_state or terminal_state:
                physical_match, _, _ = UiBackendNode._drop_zone_arrival_match(
                    self
                )
            if (
                parking_state
                and getattr(self, "_standalone_return_progress", False)
                and physical_match
            ):
                self._standalone_return_parking_seen = True
            correlated_terminal = bool(
                terminal_state
                and getattr(self, "_standalone_return_progress", False)
                and getattr(self, "_standalone_return_parking_seen", False)
                and physical_match
            )
            if correlated_terminal:
                UiBackendNode._clear_generation_zero_authority(
                    self, "standalone_return"
                )
                generation_zero_authority = ""
            elif not own_current_echo and not (
                parking_state
                and getattr(self, "_standalone_return_parking_seen", False)
            ):
                self.get_logger().warn(
                    "uncorrelated service-state heartbeat ignored during "
                    f"standalone Return: state={state_name}"
                )
                return
        if (
            generation_zero_authority
            and generation_zero_authority != "standalone_return"
            and state != int(AvgServiceState.OPERATOR_STOPPED)
        ):
            # Campsite/parking writers publish lifecycle heartbeats without a
            # mission ID. They cannot own a raw map/manual-drive command. In
            # particular, a queued pre-CANCEL DROP_ZONE_WAIT must not close the
            # engage gate after a generation-0 manual authority is accepted.
            self.get_logger().warn(
                "service-state heartbeat ignored during generation-0 "
                f"authority={generation_zero_authority}: state={state_name}"
            )
            return
        return_completion_states = {
            int(AvgServiceState.RETURNING_TO_DROP_ZONE),
            int(AvgServiceState.RETURN_WITH_CARGO),
            int(AvgServiceState.DROP_ZONE_PARKING),
        }
        terminal_states = {
            int(AvgServiceState.DROP_ZONE_WAIT),
            int(AvgServiceState.WAITING_FOR_CHARGING),
            int(AvgServiceState.CHARGING),
        }
        road_handoff_ready = (
            state == int(AvgServiceState.DROP_ZONE_WAIT)
            and state_name == "ROAD_HANDOFF_READY"
        )
        if road_handoff_ready and getattr(
            self, "_drop_zone_exit_waiting_for_fresh_status", False
        ):
            self.get_logger().warn(
                "stale road handoff ignored while waiting for current EXIT status"
            )
            return
        (
            active_site,
            active_source,
            active_generation,
            return_generation,
        ) = UiBackendNode._active_mission_identity(self)
        arrival_states = {
            int(AvgServiceState.SITE_ARRIVED),
            int(AvgServiceState.UNLOAD_WAIT),
            int(AvgServiceState.GUEST_LOADING_WAIT),
            int(AvgServiceState.WAITING_FOR_RETURN_REQUEST),
        }
        if (
            active_site
            and active_generation > 0
            and return_generation == active_generation
            and state in arrival_states
        ):
            # Once this generation accepted Return, an old campsite heartbeat
            # must never roll RETURN_WITH_CARGO back to an arrival wait and
            # close mission engage while CRAB_OUT is still moving.
            self.get_logger().warn(
                "stale arrival service state ignored after return request: "
                f"site={active_site} generation={active_generation} "
                f"state={state_name}"
            )
            return
        if (
            self_echo_generation is None
            and active_site
            and active_generation > 0
            and state in arrival_states
        ):
            # Controller lifecycle messages also lack a mission id. Require a
            # fresh physical pose at the current mission's target before an
            # arrival may close command authorization. Guest/Robot recall uses
            # the bounded roadside target for every B1-B13 site, independently
            # from each site's normal delivery service mode.
            try:
                matched, _mission_key, distance_m, reason = (
                    self._site_arrival_match(
                        active_site,
                        force_roadside=UiBackendNode._is_guest_recall_source(
                            active_source
                        ),
                        respect_immediate_arrival_policy=False,
                    )
                )
            except TypeError:
                # Compatibility for focused unit fixtures that provide the
                # historical one-argument matcher.
                matched, _mission_key, distance_m, reason = (
                    self._site_arrival_match(active_site)
                )
            if not matched:
                self.get_logger().warn(
                    "stale/unmatched arrival service state ignored for active "
                    f"mission: site={active_site} generation={active_generation} "
                    f"state={state_name} distance_m={distance_m} reason={reason}"
                )
                return
        return_progress_generation = int(
            getattr(self, "_return_progress_generation", 0)
        )
        return_operation_token = str(
            getattr(self, "_return_operation_token", "")
        ).strip()
        if (
            self_echo_generation is None
            and active_site
            and active_generation > 0
            and return_generation == active_generation
            and state == int(AvgServiceState.RETURN_WITH_CARGO)
            and return_progress_generation != active_generation
        ):
            # The campsite controller preserves the RETURN operation source in
            # its phase detail. Only the exact token minted for this mission
            # generation can acknowledge physical site-exit ownership.
            token_acknowledged = bool(
                return_operation_token
                and f"ui_return_token={return_operation_token}"
                in raw_description
            )
            try:
                still_at_departure_site, _, distance_m, reason = (
                    self._site_arrival_match(
                        active_site,
                        force_roadside=UiBackendNode._is_guest_recall_source(
                            active_source
                        ),
                        respect_immediate_arrival_policy=False,
                    )
                )
            except TypeError:
                still_at_departure_site, _, distance_m, reason = (
                    self._site_arrival_match(active_site)
                )
            if not token_acknowledged or not still_at_departure_site:
                self.get_logger().warn(
                    "uncorrelated return-progress state ignored: "
                    f"site={active_site} generation={active_generation} "
                    f"token_ack={str(token_acknowledged).lower()} "
                    f"distance_m={distance_m} reason={reason}"
                )
                return
            self._return_progress_generation = active_generation
            return_progress_generation = active_generation
        if (
            active_site
            and active_generation > 0
            and return_generation == active_generation
            and state
            in {
                int(AvgServiceState.DROP_ZONE_PARKING),
                *terminal_states,
            }
            and not road_handoff_ready
        ):
            if return_progress_generation != active_generation:
                self.get_logger().warn(
                    "return completion ignored before correlated progress: "
                    f"site={active_site} generation={active_generation} "
                    f"state={state_name}"
                )
                return
            drop_zone_match, distance_m, reason = (
                UiBackendNode._drop_zone_arrival_match(self)
            )
            if not drop_zone_match:
                self.get_logger().warn(
                    "return completion ignored outside authored drop zone: "
                    f"site={active_site} generation={active_generation} "
                    f"state={state_name} distance_m={distance_m} reason={reason}"
                )
                return
        if (
            active_site
            and active_generation > 0
            and return_generation != active_generation
            and not road_handoff_ready
            and not getattr(self, "_drop_zone_exit_failure_latched", False)
            and (
                state in return_completion_states
                or state in terminal_states
            )
        ):
            # AvgServiceState has no mission id. A return/terminal callback is
            # valid for the current mission only after that generation received
            # an explicit RETURN request. Otherwise it is queued lifecycle data
            # from the mission that STOP/new admission just superseded.
            self.get_logger().warn(
                "stale uncorrelated service state ignored for active mission: "
                f"site={active_site} generation={active_generation} "
                f"state={state_name}"
            )
            return
        # HH_260721 - DROP_ZONE_WAIT is the semantic parked state used by departure sequencing.
        previous_state = getattr(self, "_latest_service_state", None)
        if (
            state
            in {
                int(AvgServiceState.DEPARTING_CHARGER),
                int(AvgServiceState.DEPARTING_DROP_ZONE),
            }
            and (
                getattr(self, "_drop_zone_exit_handoff_ready", False)
                or getattr(self, "_drop_zone_exit_failure_latched", False)
                or getattr(self, "_drop_zone_exit_cancel_suppressed", False)
            )
        ):
            # A pre-terminal DEPARTING heartbeat from another writer cannot
            # roll back a completed road handoff.
            return
        if (
            road_handoff_ready
            and getattr(self, "_drop_zone_exit_handoff_ready", False)
            and previous_state
            not in {
                None,
                int(AvgServiceState.DROP_ZONE_WAIT),
                int(AvgServiceState.DEPARTING_CHARGER),
                int(AvgServiceState.DEPARTING_DROP_ZONE),
            }
        ):
            # A separate exit_complete writer may already have released and
            # synchronously recorded MOVING_TO_SITE.  Its later terminal
            # service-state companion is an acknowledgement, not a rollback.
            return
        state_changed = previous_state != state
        self._latest_service_state = state
        if state in {
            int(AvgServiceState.DROP_ZONE_PARKING),
            int(AvgServiceState.WAITING_FOR_CHARGING),
            int(AvgServiceState.CHARGING),
            int(AvgServiceState.DEPARTING_CHARGER),
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        }:
            self._drop_zone_exit_handoff_ready = False
        with self._lock:
            visible_changed = (
                self._state.service_state != state
                or self._state.service_state_name != state_name
                or self._state.service_state_description != description
            )
            self._state.service_state = state
            self._state.service_state_name = state_name
            self._state.service_state_description = description
        if state_changed:
            service_metrics = getattr(self, "_service_metrics", None)
            if service_metrics is not None:
                service_metrics.observe_service_state(
                    state,
                    state_name,
                    now_s=time.time(),
                )
        if visible_changed:
            self.get_logger().info(
                f"Service state received: {state_name}({state}) ({description})"
            )
        if road_handoff_ready:
            self._schedule_broadcast({
                "service_state": state,
                "service_state_name": state_name,
                "service_state_description": description,
            })
            # The terminal service state is ordered after every DEPARTING
            # heartbeat on one DataWriter.  It therefore wins regardless of
            # how the separate exit_complete topic is interleaved.
            self._drop_zone_exit_handoff_ready = True
            if getattr(self, "_drop_zone_exit_active", False):
                completion = AvgBool()
                completion.data = True
                self._on_drop_zone_exit_complete(completion)
            return
        if (
            active_site
            and active_generation > 0
            and return_generation == active_generation
            and int(getattr(self, "_return_progress_generation", 0))
            == active_generation
            and (
            state in return_completion_states
            or (
                state in terminal_states
                and previous_state in return_completion_states
            )
            )
        ):
            self._recall_terminal_clear_armed = True
            self._terminal_clear_armed_generation = active_generation
        recall_departure_pending = (
            getattr(self, "_drop_zone_exit_active", False)
            or getattr(self, "_pending_site_after_drop_zone_exit", None) is not None
            or getattr(self, "_charging_departure_delay_pending", False)
        )
        terminal_mission_completed = bool(
            state in terminal_states
            and getattr(self, "_recall_terminal_clear_armed", False)
            and int(getattr(self, "_terminal_clear_armed_generation", 0))
            == active_generation
            and active_generation > 0
            and not recall_departure_pending
        )
        completed_site = str(
            getattr(self, "_active_mission_site", "")
        ).strip()
        completed_source = str(
            getattr(self, "_active_mission_source", "")
        ).strip()
        if terminal_mission_completed and UiBackendNode._is_guest_recall_source(
            completed_source
        ):
            # Guest/Robot recall mirrors its Bx into ws_site_states so the
            # Robot UI can render the active roadside request.  That mirror is
            # transient, unlike an operator delivery site's usage state, and
            # must not survive completed terminal parking as an `anyOn` lock.
            UiBackendNode._clear_recall_owned_transient_site_state(self)
        elif terminal_mission_completed:
            # Delivery occupancy/site presentation is intentionally retained,
            # but the completed command is no longer an active mission owner.
            with self._lock:
                self._active_mission_site = ""
                self._active_mission_source = ""
                self._active_mission_generation = 0
                self._return_requested_generation = 0
                self._return_progress_generation = 0
                self._return_operation_token = ""
                self._terminal_clear_armed_generation = 0
                self._active_mission_retryable = False
                self._recall_terminal_clear_armed = False
        if terminal_mission_completed or state == int(AvgServiceState.OPERATOR_STOPPED):
            UiBackendNode._publish_destination_dispatch_status(
                self,
                completed_site,
                False,
                "service_terminal",
                {"message": "mission ownership cleared at terminal state"},
            )
        elif visible_changed:
            # Mirror every *accepted* lifecycle transition on the same
            # transient-local DataWriter as mission identity. Guest UI consumes
            # this stream instead of raw /service/state, so a heartbeat that
            # failed generation/token/physical-pose validation above can never
            # be paired with the current B-site and expose a false Return.
            lifecycle_site, _, lifecycle_generation, _ = (
                UiBackendNode._active_mission_identity(self)
            )
            UiBackendNode._publish_destination_dispatch_status(
                self,
                lifecycle_site,
                lifecycle_generation > 0,
                "service_lifecycle",
                {"message": f"authoritative service state: {state_name}"},
            )
        if visible_changed:
            lifecycle_payload = {
                "service_state": state,
                "service_state_name": state_name,
                "service_state_description": description,
            }
            lifecycle_payload.update(
                UiBackendNode._mission_dispatch_snapshot(self, state)
            )
            self._schedule_broadcast(lifecycle_payload)
        if not state_changed:
            # HH_260819 - A controller heartbeat restores the latest value
            # after restart, but repeating that same value must not re-run
            # engage transitions or count another service metric edge.
            return
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
        public_description = desc_map.get(state, f"unknown state {state}")
        # AvgServiceState has no Header/source field and Humble's rclpy does
        # not expose DataWriter GIDs to this callback.  Add a private,
        # per-process correlation suffix so only this node's own delayed echo
        # can be associated with its publish-time mission generation.  Tuple
        # matching is unsafe: an external emergency stop can have identical
        # state/name/description and must never be consumed as our own echo.
        generation = int(getattr(self, "_active_mission_generation", 0))
        msg.description = UiBackendNode._encode_service_state_echo(
            self, public_description, generation
        )
        service_metrics = getattr(self, "_service_metrics", None)
        if service_metrics is not None:
            service_metrics.observe_service_state(
                int(state),
                msg.state_name,
                now_s=time.time(),
            )
        # HH_260721 - Update local intent synchronously so CAN edges cannot overwrite departure.
        self._latest_service_state = int(state)
        self.pub_service_state.publish(msg)
        self.get_logger().info(
            f"Service state ({source}) -> {self.service_state_topic}: "
            f"{state} ({public_description})"
        )

    def _on_destination_command(self, msg: UiDestinationCommand) -> None:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._on_destination_command_serialized(self, msg)
        with dispatch_lock:
            return UiBackendNode._on_destination_command_serialized(self, msg)

    def _on_destination_command_serialized(self, msg: UiDestinationCommand) -> None:
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
        if not run:
            # Destination OFF is a mission-scoped cancel, not the global
            # emergency-stop API. Require the caller to echo the authoritative
            # generation published in destination_dispatch_status; otherwise
            # a queued OFF from an older visit can cancel a newer mission.
            source_site, source_generation = (
                UiBackendNode._parse_guest_operation_identity(source)
            )
            active_site, active_source, active_generation, _ = (
                UiBackendNode._active_mission_identity(self)
            )
            request_owner = UiBackendNode._destination_request_owner(source)
            active_owner = UiBackendNode._destination_request_owner(active_source)
            if not (
                site
                and site == source_site == active_site
                and source_generation > 0
                and source_generation == active_generation
                and request_owner == active_owner
            ):
                self.get_logger().warn(
                    "stale/unbound destination OFF ignored: "
                    f"site={site} source={source} active_site={active_site} "
                    f"active_generation={active_generation}"
                )
                UiBackendNode._publish_destination_dispatch_status(
                    self,
                    site,
                    False,
                    source,
                    {
                        "blocked": True,
                        "error": "stale_or_unowned_destination_stop",
                        "message": (
                            "destination stop does not own the active mission; "
                            "use the global /ui/stop endpoint for operator stop"
                        ),
                    },
                )
                return
        result = self._apply_destination_command(site=site, run=run, source=source)
        # Broadcast every guest-origin call (including versioned/kiosk suffixes)
        # to the robot-side UI.  The same prefix is the semantic split between a
        # roadside recall and an operator delivery into the campsite.
        if (
            UiBackendNode._is_guest_recall_source(source)
            and run
            and not result.get("blocked", False)
        ):
            # A Guest UI call uses the same typed roadside-recall path as the
            # Robot UI. Mirror its selected site as well as the notification;
            # otherwise a live Robot UI has no recall identity and falls back
            # to the generic idle preview until it reconnects.
            states = {
                known_site: known_site == site for known_site in self.site_names
            }
            with self._lock:
                self._state.ws_site_states = dict(states)
            self._schedule_broadcast({
                "states": states,
                "robot_recall_site": site,
                "guest_navigate": site,
            })
        self.get_logger().info(
            "destination dispatch summary: "
            f"site={site} run={str(run).lower()} source={source} "
            f"mission_key={result.get('mission_key', '')} "
            f"site_goal={str(bool(result.get('goal_pose_published', False))).lower()}"
        )

    def _on_campsite_occupancy(self, msg: CampsiteOccupancy) -> None:
        # HH_260818 - This callback is also directly unit-tested, so retain an
        # explicit disabled guard even though production does not subscribe in
        # that mode.
        if not self.enable_campsite_occupancy_guard:
            return

        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._on_campsite_occupancy_serialized(self, msg)
        with dispatch_lock:
            return UiBackendNode._on_campsite_occupancy_serialized(self, msg)

    def _on_campsite_occupancy_serialized(
        self, msg: CampsiteOccupancy
    ) -> None:

        occupied_keys = {
            str(key).strip() for key in msg.occupied_mission_keys if key
        }
        occupied_sites = sorted(
            site
            for site in self.site_names
            if (self._resolve_mission_key_for_site(site) or "") in occupied_keys
        )
        active_occupied_site = ""
        with self._lock:
            occupancy_changed = self._state.occupied_sites != occupied_sites
            self._state.occupied_sites = occupied_sites
            active_site = str(self._state.destination.get("site", "")).strip()
            active_run = bool(self._state.destination.get("run", False))
            service_state = int(self._state.service_state)
            committed = service_state in OCCUPANCY_CANCEL_BLOCKED_SERVICE_STATES
            guest_recall_active = (
                active_run
                and active_site == str(getattr(self, "_active_mission_site", ""))
                and UiBackendNode._is_guest_recall_source(
                    getattr(self, "_active_mission_source", "")
                )
            )
            protected_site = (
                active_site if (active_run and (committed or guest_recall_active)) else ""
            )
            for site in occupied_sites:
                if site != protected_site:
                    self._state.ws_site_states[site] = False
            if (
                active_run
                and active_site in occupied_sites
                and not committed
                and not guest_recall_active
            ):
                active_occupied_site = active_site

        if occupancy_changed:
            self._schedule_broadcast({"occupied_sites": occupied_sites})
        if active_occupied_site:
            active_identity_site, _, active_generation, _ = (
                UiBackendNode._active_mission_identity(self)
            )
            if (
                active_identity_site != active_occupied_site
                or active_generation <= 0
            ):
                self.get_logger().warn(
                    "stale occupancy cancellation ignored after mission change: "
                    f"observed_site={active_occupied_site} "
                    f"active_site={active_identity_site}"
                )
                return
            self.get_logger().warn(
                "active campsite became occupied; stopping dispatch: "
                f"{active_occupied_site}"
            )
            self._apply_destination_command(
                site=active_occupied_site,
                run=False,
                source="perception_occupancy",
            )
            self._schedule_broadcast(
                {
                    "states": {site: False for site in self.site_names},
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
        """Serialize frontend operations against destination ownership."""
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._on_ui_camping_site_operation_request_serialized(
                self, msg
            )
        with dispatch_lock:
            return UiBackendNode._on_ui_camping_site_operation_request_serialized(
                self, msg
            )

    @staticmethod
    def _parse_guest_operation_identity(source: str) -> tuple[str, int]:
        """Read ``site`` and mission generation from a Guest operation."""
        site = ""
        generation = 0
        for component in str(source).strip().split(":"):
            if component.startswith("site="):
                site = component.split("=", 1)[1].strip()
            elif component.startswith("g="):
                try:
                    generation = int(component.split("=", 1)[1])
                except ValueError:
                    generation = 0
        return site, generation

    def _reject_guest_operation(
        self, source: str, site: str, error: str, message: str
    ) -> None:
        self.get_logger().warn(
            "guest operation rejected: "
            f"source={source} site={site or '<missing>'} error={error}"
        )
        UiBackendNode._publish_destination_dispatch_status(
            self,
            site,
            True,
            source,
            {"blocked": True, "error": error, "message": message},
        )

    def _on_ui_camping_site_operation_request_serialized(
        self, msg: MotionOperation
    ) -> None:
        """Translate an authority-bound frontend intent into service control."""
        source = str(msg.source).strip() or "ui_operation_request"
        operation = int(msg.operation)
        if UiBackendNode._destination_request_owner(source) == "guest":
            requested_site, requested_generation = (
                UiBackendNode._parse_guest_operation_identity(source)
            )
            active_site, active_source, active_generation, _ = (
                UiBackendNode._active_mission_identity(self)
            )
            identity_matches = bool(
                requested_site
                and requested_generation > 0
                and requested_site == active_site
                and requested_generation == active_generation
                and UiBackendNode._destination_request_owner(active_source)
                == "guest"
                and UiBackendNode._is_guest_recall_source(active_source)
            )
            if not identity_matches:
                UiBackendNode._reject_guest_operation(
                    self,
                    source,
                    requested_site,
                    "stale_or_unowned_guest_operation",
                    "guest operation does not own the active mission identity",
                )
                return
            if (
                operation == int(MotionOperation.RETURN)
                and int(getattr(self, "_latest_service_state", -1))
                != int(AvgServiceState.GUEST_LOADING_WAIT)
            ):
                UiBackendNode._reject_guest_operation(
                    self,
                    source,
                    requested_site,
                    "guest_return_not_at_loading_wait",
                    "guest return is allowed only after roadside arrival",
                )
                return
        if operation == int(MotionOperation.CANCEL):
            # A guest/robot UI cancel must stop every possible owner of the
            # return sequence, including Nav2, campsite exit, drop-zone
            # approach, and final parking/docking.
            UiBackendNode._stop_active_service_serialized(self, source=source)
            return
        if operation != int(MotionOperation.RETURN):
            self.get_logger().warn(
                "unsupported UI campsite operation request: "
                f"operation={operation} source={source}"
            )
            return
        UiBackendNode._request_return_to_drop_zone_serialized(self, source=source)

    def _request_return_to_drop_zone(self, source: str) -> str:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._request_return_to_drop_zone_serialized(self, source)
        with dispatch_lock:
            return UiBackendNode._request_return_to_drop_zone_serialized(self, source)

    def _request_return_to_drop_zone_serialized(self, source: str) -> str:
        if UiBackendNode._startup_recovery_block(self) is not None:
            self.get_logger().warn(
                "return request rejected during backend startup recovery"
            )
            return "backend_startup_recovery"
        if bool(getattr(self, "_manual_return_transition_pending", False)):
            return "return_preempting"
        if bool(getattr(self, "_parking_rearm_transition_pending", False)):
            return "parking_alignment"
        if bool(getattr(self, "_parking_rearm_waiting_for_can", False)):
            return "parking_alignment_waiting_for_can"
        active_site, _, active_generation, return_generation = (
            UiBackendNode._active_mission_identity(self)
        )
        if active_generation > 0 and active_site:
            current_token = str(
                getattr(self, "_return_operation_token", "")
            ).strip()
            new_return_identity = bool(
                return_generation != active_generation or not current_token
            )
            if new_return_identity:
                sequence = int(
                    getattr(self, "_return_operation_sequence", 0)
                ) + 1
                self._return_operation_sequence = sequence
                current_token = (
                    f"g{active_generation}-s{sequence}-"
                    f"{time.monotonic_ns():x}"
                )
            state_lock = getattr(self, "_lock", None)
            if state_lock is None:
                self._return_requested_generation = active_generation
                if new_return_identity:
                    self._return_progress_generation = 0
                self._return_operation_token = current_token
            else:
                with state_lock:
                    self._return_requested_generation = active_generation
                    if new_return_identity:
                        self._return_progress_generation = 0
                    self._return_operation_token = current_token
            if f"ui_return_token={current_token}" not in source:
                source = f"{source}:ui_return_token={current_token}"
        else:
            # A raw/manual Nav2 goal intentionally has no campsite mission
            # identity. Give its Return request a standalone nonce that Stop or
            # any later destination claim clears, without inventing a B-site.
            UiBackendNode._clear_generation_zero_authority(self)
            UiBackendNode._advance_command_epoch(self)
            UiBackendNode._set_generation_zero_authority(
                self, "standalone_return"
            )
            self._standalone_return_progress = False
            self._standalone_return_parking_seen = False
            sequence = int(getattr(self, "_return_operation_sequence", 0)) + 1
            self._return_operation_sequence = sequence
            current_token = f"manual-s{sequence}-{time.monotonic_ns():x}"
            self._return_operation_token = current_token
            self._return_progress_generation = 0
            if f"ui_return_token={current_token}" not in source:
                source = f"{source}:ui_return_token={current_token}"
        # HH_260818 - A return request is state-independent, but route planning
        # must not start from inside a campsite. Latch physical CRAB_OUT first;
        # that controller publishes the planning recall at the shared snap anchor.

        stationary_parking_states = {
            int(AvgServiceState.DROP_ZONE_WAIT),
            int(AvgServiceState.CHARGING),
            int(AvgServiceState.WAITING_FOR_CHARGING),
        }
        parking_states = getattr(self, "_parking_controller_operating_states", {})
        selected_parking_controller = (
            "apriltag_parking"
            if str(getattr(self, "parking_method", "reverse")).strip().lower()
            == "apriltag"
            else "reverse_parking"
        )
        parking_failed = (
            str(parking_states.get(selected_parking_controller, ""))
            .strip()
            .upper()
            == "ERROR"
        )
        retryable_drop_zone_parking = (
            self._latest_service_state == int(AvgServiceState.DROP_ZONE_PARKING)
            and parking_failed
        )
        physically_at_drop_zone = False
        drop_zone_matcher = getattr(self, "_drop_zone_arrival_match", None)
        if callable(drop_zone_matcher):
            physically_at_drop_zone, _, _ = drop_zone_matcher()
        parking_context = (
            self._latest_service_state in stationary_parking_states
            or retryable_drop_zone_parking
            # Startup fail-closed intentionally publishes OPERATOR_STOPPED and
            # cancels the old parking heartbeat.  A fresh authored drop-zone
            # pose remains valid physical authority: re-dock locally instead
            # of sending an already-stationary robot around a Nav2 return loop.
            or physically_at_drop_zone
        )
        if parking_context and not self._latest_platform_is_charging:
            # HH_260818 - A stationary robot already at the drop zone needs no
            # synthetic Nav2 loop. This also recovers a stale CHARGING service
            # state or a timed-out charging wait after CAN contact is lost.
            # Reset the former PARKED/ERROR owner first: an AprilTag PARKED
            # heartbeat would otherwise close drive-enable during alignment.
            alignment_started = UiBackendNode._start_drop_zone_parking_alignment(
                self,
                source=f"{source}:already_at_drop_zone"
            )
            if alignment_started:
                UiBackendNode._arm_stationary_drop_zone_return_progress(self)
            control_mode = int(
                getattr(self, "_latest_platform_control_mode", -1)
            )
            require_can = bool(
                getattr(self, "redock_require_can_control_mode", True)
            )
            return (
                "parking_alignment_waiting_for_can"
                if require_can and control_mode != 1
                else "parking_alignment"
            )
        if parking_context and self._latest_platform_is_charging:
            # Never drive against an authoritative charger contact. Preserve
            # the explicit request through the release debounce and execute it
            # exactly once when the canonical charging status falls.
            queued, generation = UiBackendNode._queue_redock_after_disconnect(
                self, source
            )
            if queued:
                return "waiting_for_disconnect"
            # The canonical false edge won the race immediately before this
            # request acquired the generation lock.  Execute this same request
            # now; do not wait for a second edge that may never arrive.
            alignment_started = UiBackendNode._start_drop_zone_parking_alignment(
                self,
                source=f"{source}:charging_already_released",
                generation=generation,
            )
            if alignment_started:
                UiBackendNode._arm_stationary_drop_zone_return_progress(self)
            control_mode = int(
                getattr(self, "_latest_platform_control_mode", -1)
            )
            require_can = bool(
                getattr(self, "redock_require_can_control_mode", True)
            )
            return (
                "parking_alignment_waiting_for_can"
                if require_can and control_mode != 1
                else "parking_alignment"
            )

        if self._latest_service_state in {
            int(AvgServiceState.SITE_ARRIVED),
            int(AvgServiceState.SITE_ENTRY),
            int(AvgServiceState.UNLOAD_WAIT),
            int(AvgServiceState.GUEST_LOADING_WAIT),
            int(AvgServiceState.WAITING_FOR_RETURN_REQUEST),
            int(AvgServiceState.RETURN_WITH_CARGO),
        }:
            if getattr(self, "return_site_exit_rearm_enabled", False):
                # CARLA may disarm both mission engage and drive enable at an
                # arrival boundary. The simulator profile can opt into an
                # explicit re-arm before asking its campsite controller to
                # start CRAB_OUT; develop keeps the original lifecycle.
                if self.publish_mission_engage_from_destination:
                    self._publish_mission_engage(
                        True, source=f"{source}:site_exit_resume"
                    )
                else:
                    self._publish_platform_drive_enable(
                        True, source=f"{source}:site_exit_resume"
                    )
            self._publish_camping_site_maneuver_controller_return(
                source=f"{source}:site_exit_first"
            )
            if self._latest_service_state != int(AvgServiceState.RETURN_WITH_CARGO):
                # HH_260818 - This public intent re-opens mission authorization.
                # The site controller immediately replaces it with its exact
                # exit phase.
                self._publish_service_state(
                    AvgServiceState.RETURNING_TO_DROP_ZONE,
                    source=f"{source}:site_exit_latched",
                )
            return "site_exit_then_return"

        if self._latest_service_state in {
            int(AvgServiceState.DROP_ZONE_PARKING),
        }:
            # HH_260818 - Final parking already owns velocity; restarting
            # alignment here can pull the chassis away from a valid charger
            # approach.
            return "parking_in_progress"

        if self._latest_service_state == int(
            AvgServiceState.RETURNING_TO_DROP_ZONE
        ):
            # HH_260819 - Once the fresh return route owns motion, either UI
            # button is idempotent. Do not cancel and restart a valid return.
            return "return_in_progress"

        # HH_260819 - Ordinary Nav2 travel must first lose command ownership.
        # Publishing a new route without this barrier leaves the outbound path
        # active until Nav2 processes preemption and was visible as continued
        # campsite motion after either Return button was pressed.
        self._request_nav2_cancel(source=f"{source}:return_preempt")
        if self.publish_mission_engage_from_destination:
            self._publish_mission_engage(
                False, source=f"{source}:return_preempt_hold"
            )
        else:
            self._publish_platform_drive_enable(
                False, source=f"{source}:return_preempt_hold"
            )
        self._publish_engage(
            False,
            source=f"{source}:return_preempt_hold",
            sync_drive_enable=False,
        )
        self._schedule_manual_return_transition(source)
        return "return_preempting"

    def _redock_lock(self):
        lock = getattr(self, "_redock_after_disconnect_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._redock_after_disconnect_lock = lock
        return lock

    def _redock_can_ready(self) -> bool:
        return (
            not bool(getattr(self, "redock_require_can_control_mode", True))
            or int(getattr(self, "_latest_platform_control_mode", -1)) == 1
        )

    def _set_redock_status_locked(self, status: str, message: str) -> None:
        """Record one operator-facing re-dock state while its lock is held."""
        self._redock_status = str(status).strip().lower() or "idle"
        self._redock_status_message = str(message).strip()

    def _redock_status_snapshot(self) -> Dict[str, Any]:
        """Return reconnect-safe re-dock state without nesting the UI lock."""
        lock = UiBackendNode._redock_lock(self)
        with lock:
            return {
                "redock_pending": bool(
                    getattr(self, "_redock_after_disconnect_pending", False)
                ),
                "redock_waiting_for_can": bool(
                    getattr(self, "_parking_rearm_waiting_for_can", False)
                ),
                "redock_status": str(
                    getattr(self, "_redock_status", "idle")
                ),
                "redock_message": str(
                    getattr(self, "_redock_status_message", "")
                ),
            }

    def _arm_redock_expiry_locked(self, generation: int) -> None:
        """Start one non-extendable deadline for an explicit re-dock request."""
        previous_timer = getattr(self, "_redock_after_disconnect_timer", None)
        if previous_timer is not None:
            previous_timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(previous_timer)
        self._redock_after_disconnect_requested_at_s = time.monotonic()
        self._redock_after_disconnect_generation = int(generation)
        self._redock_after_disconnect_timer = None
        timeout_s = float(getattr(self, "redock_request_timeout_s", 10.0))
        create_timer = getattr(self, "create_timer", None)
        if callable(create_timer) and timeout_s > 0.0:
            self._redock_after_disconnect_timer = create_timer(
                timeout_s,
                lambda request_generation=int(generation):
                UiBackendNode._expire_pending_redock_after_disconnect(
                    self, request_generation
                ),
            )

    def _start_drop_zone_parking_alignment(
        self, source: str, generation: Optional[int] = None
    ) -> bool:
        """Cancel the completed owner, then align under one request generation."""
        lock = UiBackendNode._redock_lock(self)
        complete_immediately = False
        with lock:
            # Re-check coalescing under the generation lock.  HTTP and ROS
            # Return requests can pass the optimistic caller-side check at the
            # same time; the loser must not invalidate the winner's timer.
            if (
                getattr(self, "_parking_rearm_transition_pending", False)
                or getattr(self, "_parking_rearm_waiting_for_can", False)
            ):
                return False
            current = int(getattr(self, "_redock_generation", 0))
            if generation is None:
                generation = current + 1
                self._redock_generation = generation
                UiBackendNode._arm_redock_expiry_locked(self, int(generation))
            elif int(generation) != current:
                return False
            elif int(
                getattr(self, "_redock_after_disconnect_generation", 0)
            ) != int(generation):
                # Compatibility for a caller carrying a valid generation from
                # before request metadata was initialized.
                UiBackendNode._arm_redock_expiry_locked(self, int(generation))
            self._redock_after_disconnect_pending = False
            self._redock_after_disconnect_source = ""
            self._parking_rearm_waiting_for_can = False
            self._parking_rearm_waiting_source = ""
            self._parking_rearm_waiting_generation = 0
            # Keep generation serialization across both CANCEL publications.
            # A Stop/destination can then only run wholly before or wholly
            # after them; a stale callback cannot cancel a newer EXIT command.
            self._publish_parking_operation(
                MotionOperation.CANCEL, source=f"{source}:parking_rearm"
            )
            self._publish_drop_zone_operation(
                MotionOperation.CANCEL, source=f"{source}:drop_zone_rearm"
            )

            self._parking_rearm_transition_pending = True
            self._parking_rearm_transition_source = source
            self._parking_rearm_transition_generation = int(generation)
            UiBackendNode._set_redock_status_locked(
                self,
                "alignment_preparing",
                "Re-dock alignment handoff is being prepared",
            )
            hold_s = float(getattr(self, "parking_rearm_hold_s", 0.6))
            if hold_s <= 0.0:
                complete_immediately = True
                self._parking_rearm_transition_timer = None
            else:
                create_timer = getattr(self, "create_timer", None)
                if callable(create_timer):
                    self._parking_rearm_transition_timer = create_timer(
                        hold_s,
                        lambda request_generation=int(generation):
                        UiBackendNode._complete_parking_rearm_transition(
                            self, request_generation
                        ),
                    )
                else:
                    complete_immediately = True
                    self._parking_rearm_transition_timer = None
        if complete_immediately:
            UiBackendNode._complete_parking_rearm_transition(
                self, int(generation)
            )
        return True

    def _complete_parking_rearm_transition(
        self, generation: Optional[int] = None
    ) -> bool:
        # Every command-owner transition uses destination -> redock -> manual
        # lock order. Without this outer serialization the timer path held the
        # redock lock and then revoked manual drive while manual arm held the
        # manual lock and attempted a redock cancellation: a real AB/BA
        # deadlock.
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._complete_parking_rearm_transition_serialized(
                self, generation
            )
        with dispatch_lock:
            return UiBackendNode._complete_parking_rearm_transition_serialized(
                self, generation
            )

    def _complete_parking_rearm_transition_serialized(
        self, generation: Optional[int] = None
    ) -> bool:
        lock = UiBackendNode._redock_lock(self)
        timer = None
        result = False
        with lock:
            expected = int(
                getattr(self, "_parking_rearm_transition_generation", 0)
                if generation is None else generation
            )
            if (
                not getattr(self, "_parking_rearm_transition_pending", False)
                or expected <= 0
                or expected != int(getattr(self, "_redock_generation", 0))
                or expected != int(
                    getattr(self, "_parking_rearm_transition_generation", 0)
                )
            ):
                return False
            timer = getattr(self, "_parking_rearm_transition_timer", None)
            source = str(
                getattr(self, "_parking_rearm_transition_source", "")
            ).strip() or "manual_return"
            self._parking_rearm_transition_pending = False
            self._parking_rearm_transition_timer = None
            self._parking_rearm_transition_source = ""
            self._parking_rearm_transition_generation = 0

            if bool(getattr(self, "_latest_platform_is_charging", False)):
                # Contact can reassert during the heartbeat hold.  Preserve
                # this generation and wait for the next canonical release.
                UiBackendNode._queue_redock_after_disconnect_locked(
                    self, source, expected
                )
                result = False
            elif not UiBackendNode._redock_can_ready(self):
                # Do not start the controller while RC owns the chassis: its
                # 12 s exact-position timeout would continue behind the gate.
                self._parking_rearm_waiting_for_can = True
                self._parking_rearm_waiting_source = source
                self._parking_rearm_waiting_generation = expected
                status_message = (
                    "Switch Ranger control mode from RC to CAN for re-dock"
                )
                UiBackendNode._set_redock_status_locked(
                    self, "waiting_for_can", status_message
                )
                schedule_broadcast = getattr(self, "_schedule_broadcast", None)
                if callable(schedule_broadcast):
                    schedule_broadcast(
                        {
                            "redock_waiting_for_can": True,
                            "redock_status": "waiting_for_can",
                            "redock_message": status_message,
                            "message": status_message,
                        }
                    )
                result = False
            else:
                self._parking_rearm_waiting_for_can = False
                self._parking_rearm_waiting_source = ""
                self._parking_rearm_waiting_generation = 0
                expiry_timer = getattr(
                    self, "_redock_after_disconnect_timer", None
                )
                self._redock_after_disconnect_timer = None
                self._redock_after_disconnect_requested_at_s = 0.0
                self._redock_after_disconnect_generation = 0
                self._redock_after_disconnect_pending = False
                self._redock_after_disconnect_source = ""
                status_message = "Re-dock alignment started"
                UiBackendNode._set_redock_status_locked(
                    self, "alignment_started", status_message
                )
                if expiry_timer is not None:
                    expiry_timer.cancel()
                    destroy_timer = getattr(self, "destroy_timer", None)
                    if callable(destroy_timer):
                        destroy_timer(expiry_timer)
                schedule_broadcast = getattr(self, "_schedule_broadcast", None)
                if callable(schedule_broadcast):
                    schedule_broadcast(
                        {
                            "redock_pending": False,
                            "redock_waiting_for_can": False,
                            "redock_status": "alignment_started",
                            "redock_message": status_message,
                            "message": status_message,
                        }
                    )
                # Keep the generation lock through the final publications.
                # Stop/destination invalidation therefore orders either wholly
                # before these commands or wholly after them.
                if getattr(self, "publish_engage_from_destination", False):
                    self._publish_engage(True, source=f"{source}:parking_rearm")
                if getattr(self, "publish_mission_engage_from_destination", False):
                    self._publish_mission_engage(
                        True, source=f"{source}:parking_rearm"
                    )
                elif not getattr(self, "publish_engage_from_destination", False):
                    self._publish_platform_drive_enable(
                        True, source=f"{source}:parking_rearm"
                    )
                self._publish_drop_zone_operation(
                    MotionOperation.ALIGN_FOR_PARKING,
                    source=f"{source}:parking_rearm",
                )
                result = True
        if timer is not None:
            timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(timer)
        return result

    def _take_parking_rearm_waiting_for_can(
        self,
    ) -> Optional[tuple[str, int]]:
        lock = UiBackendNode._redock_lock(self)
        with lock:
            generation = int(
                getattr(self, "_parking_rearm_waiting_generation", 0)
            )
            if (
                not getattr(self, "_parking_rearm_waiting_for_can", False)
                or not UiBackendNode._redock_can_ready(self)
                or bool(getattr(self, "_latest_platform_is_charging", False))
                or generation <= 0
                or generation != int(getattr(self, "_redock_generation", 0))
            ):
                return None
            source = str(
                getattr(self, "_parking_rearm_waiting_source", "")
            ).strip() or "manual_return"
            self._parking_rearm_waiting_for_can = False
            self._parking_rearm_waiting_source = ""
            self._parking_rearm_waiting_generation = 0
            status_message = (
                "CAN control restored; re-dock alignment is starting"
            )
            UiBackendNode._set_redock_status_locked(
                self, "can_restored", status_message
            )
            schedule_broadcast = getattr(self, "_schedule_broadcast", None)
            if callable(schedule_broadcast):
                schedule_broadcast(
                    {
                        "redock_waiting_for_can": False,
                        "redock_status": "can_restored",
                        "redock_message": status_message,
                        "message": status_message,
                    }
                )
            return source, generation

    def _cancel_pending_parking_rearm_transition(self, reason: str) -> None:
        lock = UiBackendNode._redock_lock(self)
        with lock:
            self._redock_generation = int(
                getattr(self, "_redock_generation", 0)
            ) + 1
            timer = getattr(self, "_parking_rearm_transition_timer", None)
            was_pending = bool(
                getattr(self, "_parking_rearm_transition_pending", False)
            )
            was_waiting_for_can = bool(
                getattr(self, "_parking_rearm_waiting_for_can", False)
            )
            self._parking_rearm_transition_pending = False
            self._parking_rearm_transition_timer = None
            self._parking_rearm_transition_source = ""
            self._parking_rearm_transition_generation = 0
            self._parking_rearm_waiting_for_can = False
            self._parking_rearm_waiting_source = ""
            self._parking_rearm_waiting_generation = 0
            if was_pending or was_waiting_for_can:
                status_message = f"Re-dock request cancelled: {reason}"
                UiBackendNode._set_redock_status_locked(
                    self, "cancelled", status_message
                )
                schedule_broadcast = getattr(self, "_schedule_broadcast", None)
                if callable(schedule_broadcast):
                    schedule_broadcast(
                        {
                            "redock_pending": False,
                            "redock_waiting_for_can": False,
                            "redock_status": "cancelled",
                            "redock_message": status_message,
                            "message": status_message,
                        }
                    )
        if timer is not None:
            timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(timer)
        if was_pending or was_waiting_for_can:
            get_logger = getattr(self, "get_logger", None)
            if callable(get_logger):
                get_logger().info(f"pending parking rearm cancelled: reason={reason}")

    def _queue_redock_after_disconnect_locked(
        self, source: str, generation: int
    ) -> None:
        """Install a charger-release request while the redock lock is held."""
        if (
            int(getattr(self, "_redock_after_disconnect_generation", 0))
            != int(generation)
            or float(
                getattr(self, "_redock_after_disconnect_requested_at_s", 0.0)
            ) <= 0.0
        ):
            UiBackendNode._arm_redock_expiry_locked(self, int(generation))
        self._redock_after_disconnect_pending = True
        self._redock_after_disconnect_source = source
        status_message = (
            "Charging release pending; re-dock will start automatically"
        )
        UiBackendNode._set_redock_status_locked(
            self, "waiting_for_disconnect", status_message
        )
        schedule_broadcast = getattr(self, "_schedule_broadcast", None)
        if callable(schedule_broadcast):
            schedule_broadcast(
                {
                    "redock_pending": True,
                    "redock_status": "waiting_for_disconnect",
                    "redock_message": status_message,
                    "message": status_message,
                }
            )

    def _queue_redock_after_disconnect(
        self, source: str, generation: Optional[int] = None
    ) -> tuple[bool, int]:
        lock = UiBackendNode._redock_lock(self)
        with lock:
            current = int(getattr(self, "_redock_generation", 0))
            if generation is None:
                generation = current + 1
                self._redock_generation = int(generation)
                UiBackendNode._arm_redock_expiry_locked(self, int(generation))
            elif int(generation) != current:
                return False, int(generation)
            if not bool(getattr(self, "_latest_platform_is_charging", False)):
                return False, int(generation)
            UiBackendNode._queue_redock_after_disconnect_locked(
                self, source, int(generation)
            )
            return True, int(generation)

    def _take_pending_redock_after_disconnect(
        self,
    ) -> Optional[tuple[str, int]]:
        lock = UiBackendNode._redock_lock(self)
        timer = None
        with lock:
            if not getattr(self, "_redock_after_disconnect_pending", False):
                return None
            generation = int(
                getattr(self, "_redock_after_disconnect_generation", 0)
            )
            source = str(
                getattr(self, "_redock_after_disconnect_source", "")
            ).strip() or "manual_return"
            requested_at_s = float(
                getattr(self, "_redock_after_disconnect_requested_at_s", 0.0)
            )
            timeout_s = float(getattr(self, "redock_request_timeout_s", 10.0))
            expired = (
                requested_at_s <= 0.0
                or time.monotonic() - requested_at_s > timeout_s
                or generation <= 0
                or generation != int(getattr(self, "_redock_generation", 0))
            )
            self._redock_after_disconnect_pending = False
            self._redock_after_disconnect_source = ""
            schedule_broadcast = getattr(self, "_schedule_broadcast", None)
            if expired:
                timer = getattr(self, "_redock_after_disconnect_timer", None)
                self._redock_generation = int(
                    getattr(self, "_redock_generation", 0)
                ) + 1
                self._redock_after_disconnect_requested_at_s = 0.0
                self._redock_after_disconnect_generation = 0
                self._redock_after_disconnect_timer = None
                status = "expired"
                status_message = (
                    "Re-dock request expired before charger release"
                )
            else:
                status = "alignment_preparing"
                status_message = "Re-dock alignment handoff is being prepared"
            UiBackendNode._set_redock_status_locked(
                self, status, status_message
            )
            if callable(schedule_broadcast):
                schedule_broadcast(
                    {
                        "redock_pending": False,
                        "redock_status": status,
                        "redock_message": status_message,
                        "message": status_message,
                    }
                )
        if timer is not None:
            timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(timer)
        if expired:
            get_logger = getattr(self, "get_logger", None)
            if callable(get_logger):
                get_logger().warn("queued re-dock expired before charger release")
            return None
        return source, generation

    def _expire_pending_redock_after_disconnect(self, generation: int) -> None:
        lock = UiBackendNode._redock_lock(self)
        timer = None
        rearm_timer = None
        with lock:
            if (
                int(generation) != int(
                    getattr(self, "_redock_after_disconnect_generation", 0)
                )
                or int(generation) != int(getattr(self, "_redock_generation", 0))
            ):
                return
            timer = getattr(self, "_redock_after_disconnect_timer", None)
            rearm_timer = getattr(self, "_parking_rearm_transition_timer", None)
            self._redock_generation = int(
                getattr(self, "_redock_generation", 0)
            ) + 1
            self._redock_after_disconnect_pending = False
            self._redock_after_disconnect_source = ""
            self._redock_after_disconnect_requested_at_s = 0.0
            self._redock_after_disconnect_generation = 0
            self._redock_after_disconnect_timer = None
            self._parking_rearm_transition_pending = False
            self._parking_rearm_transition_timer = None
            self._parking_rearm_transition_source = ""
            self._parking_rearm_transition_generation = 0
            self._parking_rearm_waiting_for_can = False
            self._parking_rearm_waiting_source = ""
            self._parking_rearm_waiting_generation = 0
            status_message = (
                "Re-dock request expired before motion authorization"
            )
            UiBackendNode._set_redock_status_locked(
                self, "expired", status_message
            )
            schedule_broadcast = getattr(self, "_schedule_broadcast", None)
            if callable(schedule_broadcast):
                schedule_broadcast(
                    {
                        "redock_pending": False,
                        "redock_waiting_for_can": False,
                        "redock_status": "expired",
                        "redock_message": status_message,
                        "message": status_message,
                    }
                )
        if timer is not None:
            timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(timer)
        if rearm_timer is not None and rearm_timer is not timer:
            rearm_timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(rearm_timer)
        get_logger = getattr(self, "get_logger", None)
        if callable(get_logger):
            get_logger().warn("re-dock request expired before motion authorization")

    def _cancel_pending_redock_after_disconnect(self, reason: str) -> None:
        lock = UiBackendNode._redock_lock(self)
        with lock:
            # Invalidate even when the pending fields were already taken by a
            # callback.  Its carried token can no longer start motion afterward.
            self._redock_generation = int(
                getattr(self, "_redock_generation", 0)
            ) + 1
            timer = getattr(self, "_redock_after_disconnect_timer", None)
            was_pending = bool(
                getattr(self, "_redock_after_disconnect_pending", False)
            )
            self._redock_after_disconnect_pending = False
            self._redock_after_disconnect_source = ""
            self._redock_after_disconnect_requested_at_s = 0.0
            self._redock_after_disconnect_generation = 0
            self._redock_after_disconnect_timer = None
            if was_pending:
                status_message = f"Re-dock request cancelled: {reason}"
                UiBackendNode._set_redock_status_locked(
                    self, "cancelled", status_message
                )
                schedule_broadcast = getattr(self, "_schedule_broadcast", None)
                if callable(schedule_broadcast):
                    schedule_broadcast(
                        {
                            "redock_pending": False,
                            "redock_status": "cancelled",
                            "redock_message": status_message,
                            "message": status_message,
                        }
                    )
        if timer is not None:
            timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(timer)
        if was_pending:
            get_logger = getattr(self, "get_logger", None)
            if callable(get_logger):
                get_logger().info(f"pending re-dock cancelled: reason={reason}")

    def _publish_planning_return_request(self, source: str) -> None:
        """Publish one fresh drop-zone route after old Nav2 ownership has ended."""
        recall = PlanningRecallRequest()
        recall.header.stamp = self.get_clock().now().to_msg()
        recall.site_name = self._active_mission_site
        recall.source = source
        self._publish_service_state(
            AvgServiceState.RETURNING_TO_DROP_ZONE,
            source=f"{source}:preempt_complete",
        )
        # HH_260819 - Re-open the mission gate explicitly before the recall. The
        # service-state subscription repeats this idempotently, but its callback
        # can otherwise race planning on a multi-threaded ARM64 executor.
        if self.publish_mission_engage_from_destination:
            self._publish_mission_engage(
                True, source=f"{source}:return_resume"
            )
        else:
            self._publish_platform_drive_enable(
                True, source=f"{source}:return_resume"
            )
        self.pub_planning_return_to_drop_zone.publish(recall)
        self.get_logger().info(
            f"planning return ({source}) -> {self.planning_return_to_drop_zone_topic}"
        )

    @staticmethod
    def _is_guest_recall_source(source: str) -> bool:
        """Return whether a destination command represents a roadside recall."""
        normalized = str(source).strip().lower()
        return normalized.startswith("guest") or normalized.startswith(
            "robot_ui:recall"
        )

    def _claim_active_mission(self, site: str, source: str) -> int:
        """Claim or retry one mission identity and return its generation."""
        # Destination admission owns the command boundary immediately, even
        # while charger departure is still dwelling before mission_engage.
        # Revoke an armed browser lease inside the same serialized dispatch
        # transaction so its later deadman/disconnect cannot close the newly
        # accepted mission gate.
        revoke_manual_drive = getattr(self, "_revoke_manual_drive", None)
        if callable(revoke_manual_drive):
            revoke_manual_drive("destination_admission")
        with self._lock:
            UiBackendNode._clear_generation_zero_authority(self)
            current_site = str(
                getattr(self, "_active_mission_site", "")
            ).strip()
            current_source = str(
                getattr(self, "_active_mission_source", "")
            ).strip()
            current_generation = int(
                getattr(self, "_active_mission_generation", 0)
            )
            same_identity = (
                current_site == str(site).strip()
                and UiBackendNode._destination_request_owner(current_source)
                == UiBackendNode._destination_request_owner(source)
                and UiBackendNode._is_guest_recall_source(current_source)
                == UiBackendNode._is_guest_recall_source(source)
                and current_generation > 0
                and not bool(getattr(self, "_active_mission_retryable", False))
                and not bool(
                    getattr(self, "_drop_zone_exit_failure_latched", False)
                )
            )
            if same_identity:
                generation = current_generation
            else:
                UiBackendNode._advance_command_epoch(self)
                generation = int(getattr(self, "_mission_generation", 0)) + 1
                self._mission_generation = generation
                self._return_requested_generation = 0
                self._return_progress_generation = 0
                self._return_operation_token = ""
                self._terminal_clear_armed_generation = 0
            self._active_mission_site = str(site).strip()
            self._active_mission_source = str(source).strip()
            self._active_mission_generation = generation
            self._active_mission_retryable = False
            self._recall_terminal_clear_armed = False
        return generation

    @staticmethod
    def _destination_request_owner(source: str) -> str:
        """Classify the UI authority behind one destination command."""
        normalized = str(source).strip().lower()
        if normalized.startswith("guest"):
            return "guest"
        if normalized.startswith("robot_ui:"):
            return "robot"
        return "operator" if normalized else ""

    def _clear_recall_owned_transient_site_state(self) -> bool:
        """Clear only the UI site mirror owned by an accepted recall mission.

        Operator delivery intentionally uses the same ``ws_site_states`` map
        for its campsite usage semantics.  The accepted mission source is the
        authority that distinguishes that persistent state from a Guest/Robot
        roadside recall; a service-state number alone is deliberately not
        sufficient.
        """
        cleared_states = {site: False for site in self.site_names}
        with self._lock:
            if not UiBackendNode._is_guest_recall_source(
                getattr(self, "_active_mission_source", "")
            ):
                return False
            changed = (
                any(self._state.ws_site_states.values())
                or bool(self._state.destination.get("run", False))
                or bool(getattr(self, "_active_mission_site", ""))
            )
            self._state.ws_site_states = dict(cleared_states)
            self._state.destination = {"site": "", "run": False}
            self._active_mission_site = ""
            self._active_mission_source = ""
            self._active_mission_generation = 0
            self._return_requested_generation = 0
            self._return_progress_generation = 0
            self._return_operation_token = ""
            self._terminal_clear_armed_generation = 0
            self._active_mission_retryable = False
            self._recall_terminal_clear_armed = False
        if changed:
            self._schedule_broadcast({
                "states": cleared_states,
                "robot_recall_site": "",
            })
        return changed

    def _publish_planning_camping_site_recall(
        self, mission_key: str, source: str
    ) -> bool:
        """Release one guest call through planning without a campsite-entry goal."""
        canonical_key = str(mission_key).strip()
        if not canonical_key:
            self.get_logger().warn(
                f"camping-site recall rejected: empty mission key source={source}"
            )
            return False

        recall = PlanningRecallRequest()
        recall.header.stamp = self.get_clock().now().to_msg()
        # Planning keeps this semantic site key through goal snapping so the
        # campsite controller can construct its signed 0.30 m roadside wait pose.
        recall.site_name = canonical_key
        # Preserve the original marker exactly; downstream logs can distinguish
        # guest, guest:kiosk, and future guest transport variants.
        recall.source = str(source)

        # Match normal destination authorization, but publish only the typed
        # recall request.  No UI-owned mission-key/site-pose pair may race the
        # planning state machine into DELIVERY_TO_SITE.
        if getattr(self, "publish_engage_from_destination", False):
            self._publish_engage(True, source=f"{source}:recall_start")
        self._publish_service_state(
            AvgServiceState.RECALL_TO_SITE_ROAD,
            source=f"{source}:recall_start",
        )
        if getattr(self, "publish_mission_engage_from_destination", False):
            self._publish_mission_engage(
                True, source=f"{source}:recall_resume"
            )
        else:
            self._publish_platform_drive_enable(
                True, source=f"{source}:recall_resume"
            )
        self.pub_planning_camping_site_recall.publish(recall)
        self.get_logger().info(
            "planning camping-site recall "
            f"({source}) site={canonical_key} -> "
            f"{self.planning_camping_site_recall_topic}"
        )
        return True

    def _schedule_manual_return_transition(self, source: str) -> None:
        complete_immediately = False
        with self._manual_return_transition_lock:
            if self._manual_return_transition_pending:
                return
            self._manual_return_transition_pending = True
            self._manual_return_transition_source = source
            self._manual_return_transition_generation = int(
                getattr(self, "_active_mission_generation", 0)
            )
            self._manual_return_transition_token = str(
                getattr(self, "_return_operation_token", "")
            )
            if self.manual_return_preempt_hold_s <= 0.0:
                complete_immediately = True
            else:
                # HH_260819 - Install the timer under the same lock used by
                # Stop, preventing an uncancelled orphan timer at this boundary.
                self._manual_return_transition_timer = self.create_timer(
                    self.manual_return_preempt_hold_s,
                    self._complete_manual_return_transition,
                )

        if complete_immediately:
            self._complete_manual_return_transition()

    def _complete_manual_return_transition(self) -> None:
        # Keep lock ordering identical to Stop/destination admission:
        # destination -> manual-transition. This prevents an old timer from
        # publishing a route between a new mission claim and its cancellation.
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._complete_manual_return_transition_serialized(self)
        with dispatch_lock:
            return UiBackendNode._complete_manual_return_transition_serialized(self)

    def _complete_manual_return_transition_serialized(self) -> None:
        with self._manual_return_transition_lock:
            if not self._manual_return_transition_pending:
                return
            timer = self._manual_return_transition_timer
            self._manual_return_transition_timer = None
            source = self._manual_return_transition_source or "manual_return"
            expected_generation = int(
                getattr(self, "_manual_return_transition_generation", 0)
            )
            expected_token = str(
                getattr(self, "_manual_return_transition_token", "")
            )
            try:
                if timer is not None:
                    timer.cancel()
                    self.destroy_timer(timer)
                active_site, _, active_generation, return_generation = (
                    UiBackendNode._active_mission_identity(self)
                )
                token_matches = bool(
                    expected_token
                    and str(getattr(self, "_return_operation_token", ""))
                    == expected_token
                )
                mission_return_matches = bool(
                    active_site
                    and expected_generation > 0
                    and active_generation == expected_generation
                    and return_generation == expected_generation
                )
                standalone_return_matches = bool(
                    not active_site
                    and expected_generation == 0
                    and active_generation == 0
                )
                if not (
                    token_matches
                    and (mission_return_matches or standalone_return_matches)
                ):
                    self.get_logger().warn(
                        "stale manual-return timer ignored: "
                        f"expected_generation={expected_generation} "
                        f"active_generation={active_generation}"
                    )
                    return
                if expected_generation > 0:
                    self._return_progress_generation = expected_generation
                else:
                    self._standalone_return_progress = True
                self._publish_planning_return_request(source)
            finally:
                self._manual_return_transition_pending = False
                self._manual_return_transition_timer = None
                self._manual_return_transition_source = ""
                self._manual_return_transition_generation = 0
                self._manual_return_transition_token = ""

    def _cancel_pending_manual_return_transition(self, reason: str) -> None:
        with self._manual_return_transition_lock:
            timer = self._manual_return_transition_timer
            was_pending = self._manual_return_transition_pending
            self._manual_return_transition_pending = False
            self._manual_return_transition_timer = None
            self._manual_return_transition_source = ""
            self._manual_return_transition_generation = 0
            self._manual_return_transition_token = ""
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        if was_pending:
            self.get_logger().info(
                f"pending manual return cancelled: reason={reason}"
            )

    def request_manual_return(self) -> Dict[str, Any]:
        """Issue the same supervised return contract from any operator state."""
        startup_block = UiBackendNode._startup_recovery_block(self)
        if startup_block is not None:
            return {"success": False, **startup_block}
        action = self._request_return_to_drop_zone(source="http:manual_return")
        if action == "parking_alignment":
            service_state = AvgServiceState.DROP_ZONE_PARKING
        elif action == "parking_alignment_waiting_for_can":
            service_state = AvgServiceState.DROP_ZONE_PARKING
        elif action == "waiting_for_disconnect":
            service_state = AvgServiceState.CHARGING
        elif action in {"parking_in_progress", "return_in_progress"}:
            service_state = self._latest_service_state
        else:
            service_state = AvgServiceState.RETURNING_TO_DROP_ZONE
        return {
            "success": True,
            "message": action,
            "action": action,
            "service_state": int(service_state),
        }

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

    def _request_nav2_cancel(self, source: str) -> List[str]:
        """Cancel Nav2 goals without cancelling a serialized maneuver owner."""
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
        if sent_topics:
            self.get_logger().info(
                f"Nav2 cancel requested ({source}): {', '.join(sent_topics)}"
            )
        else:
            self.get_logger().warn(
                f"Nav2 cancel skipped ({source}): cancel services not ready"
            )
        return sent_topics

    def _cancel_active_motion(self, source: str) -> None:
        # HH_260724 - Operator cancel/stop should leave no stale Nav2 or maneuver owner active.
        self._request_nav2_cancel(source)
        self._cancel_service_motion_writers(source)

    def _cancel_service_motion_writers(self, source: str) -> None:
        """Cancel campsite/parking writers without racing a new Nav2 goal."""
        self._publish_camping_site_operation(
            MotionOperation.CANCEL, source=f"{source}:operator_stop"
        )
        self._publish_drop_zone_operation(
            MotionOperation.CANCEL, source=f"{source}:operator_stop"
        )
        self._publish_parking_operation(
            MotionOperation.CANCEL, source=f"{source}:operator_stop"
        )

    def _reassert_startup_fail_closed(self) -> None:
        """Repeat startup cancellation after DDS discovery, then retire it."""
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._reassert_startup_fail_closed_serialized(self)
        with dispatch_lock:
            return UiBackendNode._reassert_startup_fail_closed_serialized(self)

    def _reassert_startup_fail_closed_serialized(self) -> None:
        timer = getattr(self, "_startup_fail_closed_timer", None)
        attempts = int(getattr(self, "_startup_fail_closed_attempts", 0))
        if not bool(getattr(self, "_startup_recovery_pending", False)):
            complete = True
        else:
            source = f"backend_startup_discovery_reassert:{attempts + 1}"
            pending_futures = getattr(
                self, "_startup_nav2_cancel_futures", {}
            )
            completed_topics = getattr(
                self, "_startup_nav2_cancel_completed", set()
            )

            # A service future completes only after the action server has
            # processed cancel-all. Preserve it across timer ticks; merely
            # sending call_async and dropping the future is not an ordering
            # barrier against a subsequent /goal_pose.
            for topic, future in list(pending_futures.items()):
                if not future.done():
                    continue
                try:
                    future.result()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        "startup Nav2 cancel failed; will retry: "
                        f"topic={topic} error={exc}"
                    )
                else:
                    completed_topics.add(topic)
                pending_futures.pop(topic, None)

            request = CancelGoal.Request()
            request.goal_info.goal_id.uuid = [0] * 16
            request.goal_info.stamp.sec = 0
            request.goal_info.stamp.nanosec = 0
            for topic, client in zip(
                self.nav2_cancel_action_topics, self.nav2_cancel_clients
            ):
                if topic in completed_topics or topic in pending_futures:
                    continue
                if client.service_is_ready():
                    pending_futures[topic] = client.call_async(request)
                    self.get_logger().info(
                        f"startup Nav2 cancel barrier sent: {topic}"
                    )

            self._startup_nav2_cancel_futures = pending_futures
            self._startup_nav2_cancel_completed = completed_topics
            self._cancel_service_motion_writers(source)
            if self.publish_mission_engage_from_destination:
                self._publish_mission_engage(False, source=source)
            self._publish_engage(False, source=source)
            self._publish_service_state(
                AvgServiceState.OPERATOR_STOPPED,
                source=f"{source}:operator_stop",
            )
            UiBackendNode._publish_destination_dispatch_status(
                self,
                "",
                False,
                source,
                {
                    "message": "backend restart recovery in progress",
                    "blocked": True,
                    "error": "backend_startup_recovery",
                },
            )
            attempts += 1
            self._startup_fail_closed_attempts = attempts
            all_nav2_completed = len(completed_topics) == len(
                self.nav2_cancel_clients
            )
            complete = attempts >= 2 and all_nav2_completed
            if complete:
                self._startup_recovery_pending = False
                UiBackendNode._publish_destination_dispatch_status(
                    self,
                    "",
                    False,
                    "backend_startup_recovery_complete",
                    {"message": "backend restart recovery complete"},
                )
                self._schedule_broadcast({
                    "backend_startup_recovery": False,
                    "message": "Backend restart recovery complete",
                })
        if not complete:
            return
        self._startup_fail_closed_timer = None
        if timer is not None:
            timer.cancel()
            destroy_timer = getattr(self, "destroy_timer", None)
            if callable(destroy_timer):
                destroy_timer(timer)

    def _stop_active_service(self, source: str) -> None:
        """Serialize every stop source against a competing destination."""
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._stop_active_service_serialized(self, source)
        with dispatch_lock:
            return UiBackendNode._stop_active_service_serialized(self, source)

    def _stop_active_service_serialized(
        self, source: str, *, publish_service_state: bool = True
    ) -> None:
        # HH_260724 - Stop/cancel is a state transition, not only a command-gate update.
        UiBackendNode._advance_command_epoch(self)
        UiBackendNode._clear_generation_zero_authority(self)
        # Manual zero is the first boundary so HTTP STOP cannot wait behind
        # Nav2/action cancellation or service-state bookkeeping.
        self._revoke_manual_drive(
            f"operator_stop:{source}",
            notify=source != "ws_manual_drive_arm",
        )
        self._cancel_pending_manual_return_transition(source)
        UiBackendNode._cancel_pending_charging_departure_transition(self, source)
        UiBackendNode._cancel_pending_redock_after_disconnect(self, source)
        UiBackendNode._cancel_pending_parking_rearm_transition(self, source)
        departure_states = {
            int(AvgServiceState.DEPARTING_CHARGER),
            int(AvgServiceState.DEPARTING_DROP_ZONE),
        }
        departure_was_active = (
            getattr(self, "_drop_zone_exit_active", False)
            or getattr(self, "_pending_site_after_drop_zone_exit", None) is not None
            or getattr(self, "_latest_service_state", None) in departure_states
        )
        self._drop_zone_exit_active = False
        self._pending_site_after_drop_zone_exit = None
        self._drop_zone_exit_waiting_for_fresh_status = False
        self._drop_zone_exit_failure_latched = False
        self._drop_zone_exit_cancel_suppressed = departure_was_active
        self._charging_departure_from_charger = False
        if departure_was_active:
            self._drop_zone_exit_handoff_ready = False
        self._cancel_active_motion(source=source)
        if self.publish_mission_engage_from_destination:
            self._publish_mission_engage(False, source=source)
        self._publish_engage(False, source=source)
        with self._lock:
            self._active_mission_site = ""
            self._active_mission_source = ""
            self._active_mission_generation = 0
            self._return_requested_generation = 0
            self._return_progress_generation = 0
            self._return_operation_token = ""
            self._terminal_clear_armed_generation = 0
            self._active_mission_retryable = False
            self._recall_terminal_clear_armed = False
            self._state.ws_site_states = {s: False for s in self.site_names}
            self._state.destination = {"site": "", "run": False}
        if publish_service_state:
            self._publish_service_state(
                AvgServiceState.OPERATOR_STOPPED,
                source=f"{source}:operator_stop",
            )
        self._schedule_broadcast({
            "states": {s: False for s in self.site_names},
            "engage": False,
            "returning": False,
        })
        UiBackendNode._publish_destination_dispatch_status(
            self,
            "",
            False,
            source,
            {"message": "active mission stopped"},
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
        if not self.enable_campsite_occupancy_guard:
            return False
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
        if enabled:
            # Mission takeover revokes the browser command before opening the
            # mission gate. The C++ arbiter independently enforces the same
            # ownership boundary at /planning/mission_engage.
            self._revoke_manual_drive("mission_takeover")
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
            self._remember_pending_site_route_goal(pose, mission_key)
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

    def _apply_destination_command(
        self, site: str, run: bool, source: str
    ) -> Dict[str, Any]:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            # Lightweight unit fixtures created before the production lock was
            # introduced still exercise the exact serialized implementation.
            result = UiBackendNode._apply_destination_command_serialized(
                self, site, run, source
            )
            UiBackendNode._publish_destination_dispatch_status(
                self, site, run, source, result
            )
        else:
            with dispatch_lock:
                result = UiBackendNode._apply_destination_command_serialized(
                    self, site, run, source
                )
                # Publish before releasing the same lock that committed the
                # state. This keeps DataWriter order identical to admission
                # order even under a MultiThreadedExecutor.
                UiBackendNode._publish_destination_dispatch_status(
                    self, site, run, source, result
                )
        return result

    def _publish_destination_dispatch_status(
        self,
        site: str,
        run: bool,
        source: str,
        result: Dict[str, Any],
    ) -> None:
        """Publish the backend's authoritative admission result for peer UIs."""
        publisher = getattr(self, "pub_destination_dispatch_status", None)
        if publisher is None:
            return
        active_site, active_source, active_generation, _ = (
            UiBackendNode._active_mission_identity(self)
        )
        accepted = not bool(result.get("blocked", False))
        active_intent = (
            "recall"
            if active_site and UiBackendNode._is_guest_recall_source(active_source)
            else ("delivery" if active_site else "")
        )
        latest_service_state = int(
            getattr(self, "_latest_service_state", -1)
        )
        state_model = getattr(self, "_state", None)
        with self._lock:
            visible_service_state = int(
                getattr(state_model, "service_state", latest_service_state)
            )
            visible_service_name = str(
                getattr(state_model, "service_state_name", "")
            ).strip()
            visible_service_description = str(
                getattr(state_model, "service_state_description", "")
            ).strip()
        service_state_name = (
            visible_service_name
            if visible_service_state == latest_service_state
            and visible_service_name
            else SERVICE_STATE_NAMES.get(
                latest_service_state, f"UNKNOWN_{latest_service_state}"
            )
        )
        service_state_description = (
            visible_service_description
            if visible_service_state == latest_service_state
            else ""
        )
        message = String()
        message.data = json.dumps(
            {
                "accepted": accepted,
                "request_site": str(site).strip(),
                "request_run": bool(run),
                "request_source": str(source).strip(),
                "error": str(result.get("error", "")),
                "message": str(result.get("message", "")),
                "active_site": active_site,
                "active_source": active_source,
                "active_intent": active_intent,
                "active_generation": int(active_generation),
                # This transient-local backend DataWriter is the only service
                # lifecycle authority consumed by Guest UI. Raw controller
                # heartbeats are first correlated/pose-validated above.
                "service_state": latest_service_state,
                "service_state_name": service_state_name,
                "service_state_description": service_state_description,
                "retryable": bool(
                    active_site
                    and (
                        getattr(self, "_active_mission_retryable", False)
                        or result.get("retryable", False)
                    )
                ),
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        publisher.publish(message)

    def _apply_destination_command_serialized(
        self, site: str, run: bool, source: str
    ) -> Dict[str, Any]:
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

        startup_block = UiBackendNode._startup_recovery_block(self)
        if startup_block is not None:
            return {
                "site": site,
                "run": False,
                "mission_key": "",
                "goal_pose_published": False,
                **startup_block,
            }

        active_site, active_source, _, _ = (
            UiBackendNode._active_mission_identity(self)
        )
        if active_site:
            active_owner = UiBackendNode._destination_request_owner(active_source)
            request_owner = UiBackendNode._destination_request_owner(source)
            active_recall = UiBackendNode._is_guest_recall_source(active_source)
            request_recall = UiBackendNode._is_guest_recall_source(source)
            same_request = (
                active_site == site
                and active_owner == request_owner
                and active_recall == request_recall
            )
            if same_request and not getattr(
                self, "_drop_zone_exit_failure_latched", False
            ):
                self.get_logger().info(
                    "duplicate active destination ignored: "
                    f"site={site} owner={request_owner}"
                )
                return {
                    "site": site,
                    "run": True,
                    "mission_key": self._resolve_mission_key_for_site(site) or "",
                    "goal_pose_published": False,
                    "recall_request_published": False,
                    "message": "destination already owns the active mission",
                }

            if not same_request:
                self.get_logger().warn(
                    "competing destination rejected while mission is active: "
                    f"accepted_site={active_site} accepted_owner={active_owner} "
                    f"rejected_site={site} rejected_owner={request_owner}"
                )
                return {
                    "site": site,
                    "run": False,
                    "mission_key": self._resolve_mission_key_for_site(site) or "",
                    "goal_pose_published": False,
                    "recall_request_published": False,
                    "blocked": True,
                    "error": "mission_already_active",
                    "message": (
                        "another destination already owns the active mission: "
                        f"site={active_site} owner={active_owner}"
                    ),
                }
            self.get_logger().info(
                "retrying failed drop-zone departure for active destination: "
                f"site={site} owner={request_owner}"
            )

        # A campsite selection supersedes a queued no-site re-dock before the
        # charger release edge can start a competing alignment owner.
        UiBackendNode._cancel_pending_redock_after_disconnect(
            self, "destination_selected"
        )
        UiBackendNode._cancel_pending_parking_rearm_transition(
            self, "destination_selected"
        )

        guest_recall = UiBackendNode._is_guest_recall_source(source)
        pending_departure = getattr(self, "_pending_site_after_drop_zone_exit", None)
        if (
            (
                getattr(self, "_drop_zone_exit_active", False)
                or getattr(self, "_charging_departure_delay_pending", False)
            )
            and pending_departure is not None
        ):
            same_request = (
                pending_departure[0] == site
                and UiBackendNode._is_guest_recall_source(pending_departure[2])
                == guest_recall
            )
            if same_request:
                # A reliable ROS topic or websocket retry may deliver the same
                # selection more than once. Keep one owner and transition.
                self.get_logger().info(
                    "duplicate destination ignored during drop-zone departure: "
                    f"site={site}"
                )
                return {
                    "site": site,
                    "run": True,
                    "mission_key": pending_departure[1],
                    "goal_pose_published": False,
                    "recall_request_published": False,
                    "message": "destination already pending drop-zone departure",
                }

            self.get_logger().warn(
                "competing destination rejected during drop-zone departure: "
                f"accepted_site={pending_departure[0]} rejected_site={site}"
            )
            return {
                "site": site,
                "run": False,
                "mission_key": self._resolve_mission_key_for_site(site) or "",
                "goal_pose_published": False,
                "recall_request_published": False,
                "blocked": True,
                "error": "mission_already_active",
                "message": (
                    "another destination already owns drop-zone departure: "
                    f"{pending_departure[0]}"
                ),
            }

        mission_key = self._resolve_mission_key_for_site(site) or ""
        # A guest call intentionally targets a campsite with a tent and stops on
        # the road-side wait pose. Occupancy and in-site adoption apply only to
        # operator delivery, which physically enters the authored campsite.
        if not guest_recall and self._is_site_occupied(site):
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

        already_arrived = False
        distance_m = float("inf")
        match_reason = "guest_recall"
        if not guest_recall:
            already_arrived, mission_key, distance_m, match_reason = (
                self._site_arrival_match(site)
            )
        if already_arrived:
            UiBackendNode._claim_active_mission(self, site, source)
            service_metrics = getattr(self, "_service_metrics", None)
            if service_metrics is not None:
                service_metrics.start_service(
                    site,
                    mission_key=mission_key,
                    source=source,
                    now_s=time.time(),
                )
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
                "recall_request_published": False,
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
        UiBackendNode._claim_active_mission(self, site, source)
        service_metrics = getattr(self, "_service_metrics", None)
        if service_metrics is not None:
            service_metrics.start_service(
                site,
                mission_key=mission_key,
                source=source,
                now_s=time.time(),
            )

        # HH_260730 - Record accepted UI intent before engage so regulated and
        # manual goals expose the same goal-received -> path-preparing order.
        self._update_runtime_state(
            lambda: self._runtime_policy.update_goal_received("regulated")
        )
        # HH_260721 - A parked/charging robot must leave the station before Nav2 gets a site goal.
        physically_at_drop_zone = False
        drop_zone_matcher = getattr(self, "_drop_zone_arrival_match", None)
        if (
            not getattr(self, "_drop_zone_exit_handoff_ready", False)
            and callable(drop_zone_matcher)
        ):
            physically_at_drop_zone, _, _ = drop_zone_matcher()
        departure_required = (
            getattr(self, "_drop_zone_exit_cancel_suppressed", False)
            or self._latest_platform_is_charging
            # A backend restart intentionally replaces the volatile lifecycle
            # with OPERATOR_STOPPED and cancels the parking heartbeat. Fresh
            # authored station geometry then remains the only proof that the
            # first mission must execute bounded EXIT before Nav2.
            or physically_at_drop_zone
            # A roadside recall normally starts from the parked drop-zone
            # position.  On a fresh UI-backend process there may be no retained
            # service-state sample yet; the explicit road-handoff latch is the
            # only safe proof that Nav2 may start without the bounded station
            # exit maneuver.  Starting Nav2 directly from the parking pose
            # makes the lanelet body guard stop it immediately.
            or (
                guest_recall
                and not getattr(self, "_drop_zone_exit_handoff_ready", False)
                and getattr(self, "_latest_service_state", None) is None
            )
            or (
                not getattr(self, "_drop_zone_exit_handoff_ready", False)
                and self._latest_service_state
                in {
                    int(AvgServiceState.DROP_ZONE_PARKING),
                    int(AvgServiceState.DROP_ZONE_WAIT),
                    int(AvgServiceState.CHARGING),
                    int(AvgServiceState.WAITING_FOR_CHARGING),
                    int(AvgServiceState.DEPARTING_CHARGER),
                    int(AvgServiceState.DEPARTING_DROP_ZONE),
                }
            )
        )
        mission_key = self._resolve_mission_key_for_site(site) or ""
        if departure_required and mission_key:
            # HH_260819 - Keep the planning mission identity on drop_zone until
            # physical departure reaches its captured road handoff. Publishing
            # the campsite key here can pair it with the previous reached goal.
            self._pending_site_after_drop_zone_exit = (site, mission_key, source)
            self._drop_zone_exit_handoff_ready = False
            self._drop_zone_exit_cancel_suppressed = False
            charging_delay_required = (
                UiBackendNode._charging_departure_delay_required(self)
            )
            self._charging_departure_from_charger = charging_delay_required
            if charging_delay_required:
                UiBackendNode._schedule_charging_departure_transition(
                    self, source
                )
            else:
                UiBackendNode._begin_drop_zone_departure(self, source)
            return {
                "site": site,
                "run": True,
                "mission_key": mission_key,
                "goal_pose_published": False,
                "recall_request_published": False,
                "message": (
                    f"{'recall request' if guest_recall else 'site goal'} pending "
                    f"{self.charging_departure_delay_s:.1f} s charging safety dwell, "
                    "then drop-zone exit"
                    if charging_delay_required
                    else (
                        f"{'recall request' if guest_recall else 'site goal'} pending "
                        "drop-zone straight exit and yaw alignment"
                    )
                ),
            }

        if guest_recall:
            recall_published = (
                UiBackendNode._publish_planning_camping_site_recall(
                    self, mission_key, source
                )
            )
            return {
                "site": site,
                "run": True,
                "mission_key": mission_key,
                "goal_pose_published": False,
                "recall_request_published": bool(recall_published),
                "message": (
                    "guest roadside recall requested"
                    if recall_published
                    else "guest roadside recall rejected"
                ),
            }

        if self.publish_engage_from_destination:
            self._publish_engage(True, source=f"{source}:destination")
        if self.publish_mission_engage_from_destination:
            self._publish_mission_engage(True, source=f"{source}:destination")
        self._publish_service_state(AvgServiceState.MOVING_TO_SITE, source=f"{source}:start")
        goal_result = self._publish_goal_for_site(site=site, source=source)
        return {
            "site": site,
            "run": True,
            "mission_key": goal_result.get("mission_key", ""),
            "goal_pose_published": bool(goal_result.get("goal_pose_published", False)),
            "recall_request_published": False,
            "message": str(goal_result.get("message", "ok")),
        }

    def _is_recent_direct_destination_echo(self, site: str, run: bool, source: str) -> bool:
        prefix = str(getattr(self, "_direct_destination_echo_prefix", "")).strip()
        own_marker = f"|ui_backend_echo={prefix}-" if prefix else ""
        structurally_owned = bool(own_marker and own_marker in str(source))
        entries = getattr(self, "_direct_destination_echoes", None)
        if entries is None:
            entries = deque()
            legacy = getattr(self, "_last_direct_destination_echo", None)
            if legacy is not None:
                entries.append(legacy)
        matched = False
        remaining = deque()
        for entry in entries:
            entry_site, entry_run, entry_source, entry_time_s = entry
            if (
                not matched
                and site == entry_site
                and bool(run) == bool(entry_run)
                and source == entry_source
            ):
                matched = True
                continue
            remaining.append(entry)
        self._direct_destination_echoes = remaining
        self._last_direct_destination_echo = remaining[-1] if remaining else None
        # The unguessable per-process marker remains authoritative if more than
        # 64 local publications wrapped the observability FIFO before callbacks
        # were scheduled. Queue matching retains compatibility with old tests.
        return matched or structurally_owned

    def _next_direct_destination_echo_source(self, source: str) -> str:
        sequence = int(getattr(self, "_direct_destination_echo_sequence", 0)) + 1
        self._direct_destination_echo_sequence = sequence
        prefix = str(
            getattr(
                self,
                "_direct_destination_echo_prefix",
                f"{os.getpid()}-{time.monotonic_ns()}",
            )
        )
        self._direct_destination_echo_prefix = prefix
        return f"{str(source).strip()}|ui_backend_echo={prefix}-{sequence}"

    def _remember_direct_destination_echo(
        self, site: str, run: bool, source: str
    ) -> None:
        entry = (
            str(site).strip(),
            bool(run),
            str(source).strip(),
            self.get_clock().now().nanoseconds * 1e-9,
        )
        entries = getattr(self, "_direct_destination_echoes", None)
        if entries is None:
            entries = deque(maxlen=64)
            self._direct_destination_echoes = entries
        entries.append(entry)
        self._last_direct_destination_echo = entry

    def _advance_command_epoch(self) -> int:
        """Advance the authority epoch, including generation-0 commands."""
        lock = getattr(self, "_command_epoch_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._command_epoch_lock = lock
        with lock:
            epoch = int(getattr(self, "_command_epoch", 0)) + 1
            self._command_epoch = epoch
            return epoch

    def _set_generation_zero_authority(self, authority: str) -> None:
        self._generation_zero_authority = str(authority).strip()
        self._generation_zero_authority_epoch = (
            UiBackendNode._current_command_epoch(self)
            if self._generation_zero_authority
            else 0
        )

    def _clear_generation_zero_authority(
        self, expected_authority: str = ""
    ) -> bool:
        current = str(
            getattr(self, "_generation_zero_authority", "")
        ).strip()
        expected = str(expected_authority).strip()
        if expected and current != expected:
            return False
        if current == "standalone_return":
            self._standalone_return_progress = False
            self._standalone_return_parking_seen = False
        self._generation_zero_authority = ""
        self._generation_zero_authority_epoch = 0
        return bool(current)

    def _current_command_epoch(self) -> int:
        lock = getattr(self, "_command_epoch_lock", None)
        if lock is None:
            return int(getattr(self, "_command_epoch", 0))
        with lock:
            return int(getattr(self, "_command_epoch", 0))

    def _startup_recovery_block(self) -> Optional[Dict[str, Any]]:
        """Describe the fail-closed restart barrier, if it is still active."""
        if not bool(getattr(self, "_startup_recovery_pending", False)):
            return None
        return {
            "blocked": True,
            "error": "backend_startup_recovery",
            "message": (
                "backend restart recovery is cancelling previous motion; retry "
                "after the Nav2 cancellation barrier completes"
            ),
            "retryable": True,
        }

    def _encode_service_state_echo(
        self, description: str, generation: int
    ) -> str:
        """Attach an unambiguous local-writer correlation to one publication."""
        prefix = str(getattr(self, "_service_state_echo_prefix", "")).strip()
        if not prefix:
            prefix = f"{os.getpid()}-{time.monotonic_ns()}"
            self._service_state_echo_prefix = prefix
        sequence = int(getattr(self, "_service_state_echo_sequence", 0)) + 1
        self._service_state_echo_sequence = sequence
        epoch = UiBackendNode._current_command_epoch(self)
        return (
            f"{str(description).strip()}"
            f" |camrod_ui_echo={prefix}:{sequence}:e={epoch}:g={int(generation)}"
        )

    def _decode_service_state_echo(
        self, description: str
    ) -> tuple[str, Optional[int], Optional[int]]:
        """Return public text plus local generation/epoch correlation."""
        raw = str(description).strip()
        marker = " |camrod_ui_echo="
        if marker not in raw:
            return raw, None, None
        public_description, token = raw.rsplit(marker, 1)
        prefix = str(getattr(self, "_service_state_echo_prefix", "")).strip()
        expected = f"{prefix}:" if prefix else ""
        if not expected or not token.startswith(expected):
            # This marker belongs to a previous backend incarnation. It is not
            # an external controller command and must not acquire authority.
            return public_description.strip(), None, -1
        parts = token[len(expected):].split(":e=", 1)
        if len(parts) != 2 or not parts[0].isdigit():
            return public_description.strip(), None, -1
        epoch_and_generation = parts[1].split(":g=", 1)
        if len(epoch_and_generation) != 2:
            return public_description.strip(), None, -1
        try:
            epoch = int(epoch_and_generation[0])
            generation = int(epoch_and_generation[1])
        except ValueError:
            return public_description.strip(), None, -1
        return public_description.strip(), generation, epoch

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
            snapshot = {
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
                # HH_260825 - Initial clients receive the same departure dwell
                # state as websocket edge updates.
                "departure_delay_active": bool(
                    getattr(self, "_charging_departure_delay_pending", False)
                ),
                "departure_delay_seconds": (
                    self.charging_departure_delay_s
                    if getattr(self, "_charging_departure_delay_pending", False)
                    else 0.0
                ),
                "minimum_battery_percentage": self.low_battery_return_threshold_percent,
                "occupied_sites": list(self._state.occupied_sites),
                "mission_retryable": bool(
                    getattr(self, "_active_mission_retryable", False)
                ),
                "mission_retry_site": (
                    str(getattr(self, "_active_mission_site", ""))
                    if getattr(self, "_active_mission_retryable", False)
                    else ""
                ),
                "mission_retry_owner": (
                    UiBackendNode._destination_request_owner(
                        getattr(self, "_active_mission_source", "")
                    )
                    if getattr(self, "_active_mission_retryable", False)
                    else ""
                ),
            }
        # Keep the lock order flat: re-dock callbacks may publish while holding
        # their own generation lock, so never acquire that lock under _lock.
        snapshot.update(UiBackendNode._redock_status_snapshot(self))
        return snapshot

    # ── Public API methods (called by HTTP handlers) ──────────────────────────

    def set_manual_goal(self, x: Any, y: Any, yaw_deg: Any) -> Dict[str, Any]:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._set_manual_goal_serialized(self, x, y, yaw_deg)
        with dispatch_lock:
            return UiBackendNode._set_manual_goal_serialized(self, x, y, yaw_deg)

    def _set_manual_goal_serialized(
        self, x: Any, y: Any, yaw_deg: Any
    ) -> Dict[str, Any]:
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

        startup_block = UiBackendNode._startup_recovery_block(self)
        if startup_block is not None:
            return {"success": False, **startup_block}

        blocked = self._manual_goal_dispatch_block()
        if blocked is not None:
            return {"success": False, **blocked}

        # A raw map goal is also a command-owner takeover. Zero and revoke the
        # existing manual-drive lease before changing authority so a stale
        # disarm/deadman callback cannot disengage this new goal afterward.
        self._revoke_manual_drive("manual_goal_takeover")
        UiBackendNode._advance_command_epoch(self)
        UiBackendNode._set_generation_zero_authority(self, "manual_goal")
        # Reset service-motion writers before handing Nav2 a raw map goal. Do
        # not issue an asynchronous Nav2 cancel-all here: that service response
        # can arrive after /goal_pose and cancel the newly accepted goal. Nav2
        # performs normal goal preemption, while the generation-0 authority
        # latch makes any queued controller heartbeat harmless.
        self._cancel_service_motion_writers(source="http_manual_goal")

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
            self._active_mission_source = ""
            self._active_mission_generation = 0
            self._return_requested_generation = 0
            self._return_progress_generation = 0
            self._return_operation_token = ""
            self._terminal_clear_armed_generation = 0
            self._active_mission_retryable = False
            self._state.ws_site_states = {site: False for site in self.site_names}
            self._state.destination = {"site": "", "run": False}
        UiBackendNode._publish_destination_dispatch_status(
            self,
            "",
            False,
            "http_manual_goal",
            {"message": "campsite mission replaced by manual map goal"},
        )
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

    def set_engage(
        self, value: bool, source: str = "http"
    ) -> Dict[str, Any]:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._set_engage_serialized(self, value, source)
        with dispatch_lock:
            return UiBackendNode._set_engage_serialized(self, value, source)

    def _set_engage_serialized(
        self, value: bool, source: str
    ) -> Dict[str, Any]:
        startup_block = UiBackendNode._startup_recovery_block(self)
        if bool(value) and startup_block is not None:
            return {"success": False, **startup_block, "value": False}
        UiBackendNode._advance_command_epoch(self)
        active_site, _, _, _ = UiBackendNode._active_mission_identity(self)
        if bool(value) and not active_site:
            UiBackendNode._set_generation_zero_authority(
                self, "manual_engage"
            )
        else:
            UiBackendNode._clear_generation_zero_authority(self)
        self._publish_engage(bool(value), source=source)
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
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            result = UiBackendNode._set_engage_serialized(
                self, value, "http_operation_mode"
            )
        else:
            with dispatch_lock:
                result = UiBackendNode._set_engage_serialized(
                    self, value, "http_operation_mode"
                )
        return {
            "success": bool(result.get("success", False)),
            "message": "operation_mode command forwarded as engage",
            "auto": bool(value),
        }

    def set_auto(self) -> Dict[str, Any]:
        result = self.set_engage(True, source="http_auto")
        return {
            "success": bool(result.get("success", False)),
            "message": "auto command published",
        }

    def set_stop(self) -> Dict[str, Any]:
        self._stop_active_service(source="http_stop")
        return {"success": True, "message": "stop command published"}

    def set_destination(
        self,
        site: str,
        run: bool,
        source: str = "http_ui_destination",
        mission_generation: Optional[int] = None,
    ) -> Dict[str, Any]:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._set_destination_serialized(
                self, site, run, source, mission_generation
            )
        with dispatch_lock:
            return UiBackendNode._set_destination_serialized(
                self, site, run, source, mission_generation
            )

    def request_owned_return_to_drop_zone(
        self,
        site: str,
        mission_generation: int,
        *,
        source: str,
        allowed_owners: set[str],
    ) -> Dict[str, Any]:
        """Accept Return only from the UI that owns the exact live mission.

        Browser frames can arrive after a reconnect or after another UI has
        claimed a new mission.  Site text alone is therefore insufficient: an
        accepted Return is bound to site, monotonic generation, and owner while
        holding the same lock used by destination admission.
        """
        normalized_site = str(site).strip()
        try:
            requested_generation = int(mission_generation)
        except (TypeError, ValueError):
            requested_generation = 0
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)

        def serialized() -> Dict[str, Any]:
            startup_block = UiBackendNode._startup_recovery_block(self)
            if startup_block is not None:
                return {
                    "success": False,
                    "site": normalized_site,
                    **startup_block,
                    **UiBackendNode._mission_dispatch_snapshot(self),
                }
            active_site, active_source, active_generation, _ = (
                UiBackendNode._active_mission_identity(self)
            )
            active_owner = UiBackendNode._destination_request_owner(
                active_source
            )
            current_state = int(getattr(self, "_latest_service_state", -1))
            recall_mission = UiBackendNode._is_guest_recall_source(
                active_source
            )
            valid_return_states = (
                {int(AvgServiceState.GUEST_LOADING_WAIT)}
                if recall_mission
                else {
                    int(AvgServiceState.SITE_ARRIVED),
                    int(AvgServiceState.UNLOAD_WAIT),
                    int(AvgServiceState.WAITING_FOR_RETURN_REQUEST),
                }
            )
            return_wait_context = {
                "service_state": current_state,
                "service_state_name": SERVICE_STATE_NAMES.get(
                    current_state, f"UNKNOWN_{current_state}"
                ),
                "return_wait_active": bool(
                    active_site
                    and active_generation > 0
                    and active_owner in {"operator", "robot"}
                    and current_state in valid_return_states
                ),
                "return_wait_site": active_site,
                "return_wait_owner": active_owner,
            }
            identity_matches = bool(
                normalized_site
                and normalized_site == active_site
                and requested_generation > 0
                and requested_generation == active_generation
                and active_owner in allowed_owners
            )
            if not identity_matches:
                return {
                    "success": False,
                    "site": normalized_site,
                    "error": "stale_or_unowned_return",
                    "message": (
                        "return requires the current mission site, generation, "
                        "and UI owner"
                    ),
                    **return_wait_context,
                    **UiBackendNode._mission_dispatch_snapshot(self),
                }
            if current_state not in valid_return_states:
                return {
                    "success": False,
                    "site": normalized_site,
                    "error": "return_not_at_service_wait",
                    "message": (
                        "usage complete is allowed only after the current "
                        "mission reaches its authored service wait"
                    ),
                    **return_wait_context,
                    **UiBackendNode._mission_dispatch_snapshot(self),
                }
            transition = UiBackendNode._request_return_to_drop_zone_serialized(
                self, source=source
            )
            # Commit the Robot UI mirror in the same destination transaction
            # that accepted this exact site+generation.  Clearing it later in
            # the async websocket coroutine allowed Stop + a new destination
            # to win first, after which the old Return response erased the new
            # site's visible state.
            cleared_states = {
                known_site: False for known_site in self.site_names
            }
            with self._lock:
                self._state.ws_site_states = dict(cleared_states)
            self._schedule_broadcast(
                {"states": cleared_states, "engage": False}
            )
            return {
                "success": True,
                "site": active_site,
                "mission_generation": active_generation,
                "transition": transition,
            }

        if dispatch_lock is None:
            return serialized()
        with dispatch_lock:
            return serialized()

    def _set_destination_serialized(
        self,
        site: str,
        run: bool,
        source: str = "http_ui_destination",
        mission_generation: Optional[int] = None,
    ) -> Dict[str, Any]:
        normalized_site = str(site).strip()
        if not normalized_site:
            return {"success": False, "message": "site is required"}
        if normalized_site not in self.site_names:
            return {
                "success": False,
                "message": f"unknown site: {normalized_site}",
                "valid_sites": list(self.site_names),
            }
        if not run:
            active_site, active_source, active_generation, _ = (
                UiBackendNode._active_mission_identity(self)
            )
            active_owner = UiBackendNode._destination_request_owner(
                active_source
            )
            request_owner = UiBackendNode._destination_request_owner(source)
            try:
                requested_generation = int(mission_generation)
            except (TypeError, ValueError):
                requested_generation = 0
            if not (
                active_site == normalized_site
                and active_generation > 0
                and requested_generation == active_generation
                and request_owner == active_owner
            ):
                return {
                    "success": False,
                    "site": normalized_site,
                    "error": "stale_or_unowned_destination_stop",
                    "message": (
                        "destination OFF requires the current mission generation; "
                        "use /ui/stop for a global operator stop"
                    ),
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

        dispatch = self._apply_destination_command(
            site=normalized_site,
            run=bool(run),
            source=source,
        )
        if dispatch.get("blocked", False):
            return {
                "success": False,
                "site": normalized_site,
                "error": str(dispatch.get("error", "mission_rejected")),
                "message": str(dispatch.get("message", "destination rejected")),
                "dispatch": dispatch,
            }

        states = {
            site_name: site_name == normalized_site and bool(run)
            for site_name in self.site_names
        }
        with self._lock:
            self._state.ws_site_states = dict(states)
        self._schedule_broadcast({"states": states})
        echo_source = UiBackendNode._next_direct_destination_echo_source(
            self, source
        )
        UiBackendNode._remember_direct_destination_echo(
            self, normalized_site, bool(run), echo_source
        )
        payload = self._publish_destination_command(
            site=normalized_site,
            run=bool(run),
            source=echo_source,
        )
        return {"success": True, "destination": payload, "dispatch": dispatch}

    def set_campsite_recall(self, site: str) -> Dict[str, Any]:
        dispatch_lock = getattr(self, "_destination_dispatch_lock", None)
        if dispatch_lock is None:
            return UiBackendNode._set_campsite_recall_serialized(self, site)
        with dispatch_lock:
            return UiBackendNode._set_campsite_recall_serialized(self, site)

    def _set_campsite_recall_serialized(self, site: str) -> Dict[str, Any]:
        """Dispatch a Robot UI campsite selection as a roadside recall."""
        normalized_site = str(site).strip()
        if not normalized_site:
            return {
                "success": False,
                "intent": "recall",
                "site": "",
                "message": "site is required",
            }
        if normalized_site not in self.site_names:
            return {
                "success": False,
                "intent": "recall",
                "site": normalized_site,
                "message": f"unknown site: {normalized_site}",
                "valid_sites": list(self.site_names),
            }

        # Occupancy is intentionally not consulted: a tent at the selected
        # campsite is the expected target for roadside recall.  Battery policy
        # and every normal motion-safety gate remain authoritative.
        battery_block = self._mission_dispatch_battery_block(normalized_site)
        if battery_block is not None:
            return {
                "success": False,
                "intent": "recall",
                "site": normalized_site,
                **battery_block,
            }

        source = "robot_ui:recall"
        dispatch = self._apply_destination_command(
            site=normalized_site,
            run=True,
            source=source,
        )
        if dispatch.get("blocked", False):
            return {
                "success": False,
                "intent": "recall",
                "site": normalized_site,
                "error": str(dispatch.get("error", "mission_rejected")),
                "dispatch": dispatch,
                "message": str(dispatch.get("message", "recall was rejected")),
            }

        states = {
            known_site: known_site == normalized_site
            for known_site in self.site_names
        }
        with self._lock:
            self._state.ws_site_states = dict(states)
        self._schedule_broadcast(
            {"states": states, "robot_recall_site": normalized_site}
        )
        echo_source = UiBackendNode._next_direct_destination_echo_source(
            self, source
        )
        UiBackendNode._remember_direct_destination_echo(
            self, normalized_site, True, echo_source
        )
        payload = self._publish_destination_command(
            site=normalized_site,
            run=True,
            source=echo_source,
        )
        mission_dispatch = UiBackendNode._mission_dispatch_snapshot(self)
        return {
            "success": True,
            "intent": "recall",
            "site": normalized_site,
            "destination": payload,
            "dispatch": dispatch,
            **mission_dispatch,
        }

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
            # Register before reading any snapshot field. Diagnostics, service,
            # and re-dock callbacks do not all take the destination lock; if
            # registration happened afterward their transition could fall in
            # an accept->snapshot gap with no live recipient and be lost.
            with node._ws_clients_lock:
                node._ws_initializing_clients[ws] = []
                node._ws_client_send_locks[ws] = asyncio.Lock()
            # Destination ownership and every UI mirror are snapshotted under
            # the same admission lock. Register as "initializing" before that
            # lock is released; broadcasts after the snapshot are queued until
            # this one authoritative frame has been sent.
            with node._destination_dispatch_lock:
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
                    active_mission_site = str(node._active_mission_site)
                    active_mission_source = str(node._active_mission_source)
                    recall_site = (
                        active_mission_site
                        if (
                            active_mission_site
                            and UiBackendNode._is_guest_recall_source(
                                active_mission_source
                            )
                        )
                        else ""
                    )
                redock_status = UiBackendNode._redock_status_snapshot(node)
                mission_dispatch = UiBackendNode._mission_dispatch_snapshot(
                    node, service_state
                )
                initial_payload = {
                    "states": states,
                    "robot_recall_site": recall_site,
                    "occupied_sites": occupied_sites,
                    "engage": engage,
                    "ready": ready,
                    "ready_message": ready_message,
                    "mission_phase": mission_phase,
                    "mission_source": mission_source,
                    "system_health": system_health,
                    "service_state": service_state,
                    "service_state_name": service_state_name,
                    "service_state_description": service_state_description,
                    "site": active_mission_site,
                    **mission_dispatch,
                    **redock_status,
                }
                if battery >= 0:
                    initial_payload["battery"] = battery
            try:
                await UiBackendNode._send_ws_json(node, ws, initial_payload)
                while True:
                    with node._ws_clients_lock:
                        queued = node._ws_initializing_clients.get(ws)
                        if queued is None:
                            raise RuntimeError("websocket initialization was cancelled")
                        if not queued:
                            node._ws_initializing_clients.pop(ws, None)
                            node._ws_clients.add(ws)
                            break
                        pending_payloads = list(queued)
                        queued.clear()
                    for queued_payload in pending_payloads:
                        await UiBackendNode._send_ws_json(
                            node, ws, queued_payload
                        )
            except Exception:
                with node._ws_clients_lock:
                    node._ws_initializing_clients.pop(ws, None)
                    node._ws_clients.discard(ws)
                    node._ws_client_send_locks.pop(ws, None)
                raise

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
                            await UiBackendNode._send_ws_json(node, ws, {
                                "error": "unknown_site",
                                "site": site,
                                "valid_sites": list(node.site_names),
                            })
                            continue
                        if new_state and node._is_site_occupied(site):
                            await UiBackendNode._send_ws_json(node, ws, {
                                "error": "campsite_occupied",
                                "site": site,
                                "occupied": True,
                            })
                            continue
                        if new_state:
                            battery_block = node._mission_dispatch_battery_block(site)
                            if battery_block is not None:
                                await UiBackendNode._send_ws_json(
                                    node, ws, battery_block
                                )
                                continue
                        if new_state:
                            # Admission must precede every UI mirror. Otherwise
                            # a competing click rejected by the serialized
                            # backend could still replace the visible B-site.
                            result = await asyncio.to_thread(
                                node.set_destination, site, True, "ws"
                            )
                            if not result.get("success", False):
                                with node._lock:
                                    authoritative_states = dict(
                                        node._state.ws_site_states
                                    )
                                rejection = {
                                    "error": result.get("error", "mission_rejected"),
                                    "site": site,
                                    "message": result.get(
                                        "message", "destination rejected"
                                    ),
                                    "states": authoritative_states,
                                }
                                rejection.update(
                                    UiBackendNode._mission_dispatch_snapshot(node)
                                )
                                await UiBackendNode._send_ws_json(
                                    node, ws, rejection
                                )
                                continue
                            # Deliver the committed generation on this same
                            # command socket before a fast subsequent OFF or
                            # Usage-complete click can be emitted.  The ROS
                            # dispatch-status topic remains the cross-UI copy.
                            await UiBackendNode._send_ws_json(
                                node,
                                ws,
                                UiBackendNode._mission_dispatch_snapshot(node)
                            )
                        else:
                            result = await asyncio.to_thread(
                                node.set_destination,
                                site,
                                False,
                                "ws_toggle_off",
                                payload.get("mission_generation"),
                            )
                            if not result.get("success", False):
                                rejection = {
                                    "error": result.get("error", "stop_rejected"),
                                    "site": site,
                                    "message": result.get(
                                        "message", "destination stop rejected"
                                    ),
                                }
                                with node._lock:
                                    rejection["states"] = dict(
                                        node._state.ws_site_states
                                    )
                                rejection.update(
                                    UiBackendNode._mission_dispatch_snapshot(node)
                                )
                                await UiBackendNode._send_ws_json(
                                    node, ws, rejection
                                )
                                continue

                    # {"engage": true/false}
                    if "engage" in payload:
                        new_engage = bool(payload["engage"])
                        await asyncio.to_thread(
                            node.set_engage,
                            new_engage,
                            "ws_engage",
                        )

                    # HH_260617: usage_complete is return-to-drop-zone state=3.
                    # Guest recall request is state=4 and is published by ui_guest_node.
                    if payload.get("usage_complete"):
                        result = await asyncio.to_thread(
                            node.request_owned_return_to_drop_zone,
                            str(payload.get("site", "")),
                            payload.get("mission_generation", 0),
                            source="ws:usage_complete",
                            allowed_owners={"operator", "robot"},
                        )
                        if not result.get("success", False):
                            with node._lock:
                                result["states"] = dict(
                                    node._state.ws_site_states
                                )
                            await UiBackendNode._send_ws_json(node, ws, result)
                            continue

            except WebSocketDisconnect:
                pass
            except KeyError as exc:
                # HH_260616: Some non-browser test clients disconnect without a close
                # code; Starlette can surface that as KeyError('code').
                node.get_logger().debug(f"websocket disconnected without close code: {exc}")
            finally:
                with node._ws_clients_lock:
                    node._ws_initializing_clients.pop(ws, None)
                    node._ws_clients.discard(ws)
                    node._ws_client_send_locks.pop(ws, None)

        @app.websocket("/ws/manual-drive")
        async def manual_drive_websocket(ws: WebSocket) -> None:
            """Single-owner, deadman-protected operator manual control."""
            await ws.accept()
            if not node.manual_drive_available:
                await ws.send_json(node._manual_drive_state_payload())
                await ws.close(code=4403, reason="manual drive disabled")
                return

            owner = object()
            transport: Optional[ManualDriveTransport] = None
            with node._manual_drive_transition_lock:
                try:
                    lease = node._manual_drive_policy.connect(owner)
                except ManualDriveProtocolError as error:
                    busy_payload = {
                        "type": "error",
                        "error": error.error,
                        "message": str(error),
                        "manual_drive": node._manual_drive_policy.snapshot(),
                    }
                else:
                    transport = ManualDriveTransport(
                        websocket=ws,
                        lease=lease,
                        loop=asyncio.get_running_loop(),
                        send_lock=asyncio.Lock(),
                    )
                    node._manual_drive_transport = transport
                    busy_payload = None

            if busy_payload is not None:
                # A rejected observer must never zero or revoke the active owner.
                await ws.send_json(busy_payload)
                await ws.close(code=4409, reason="manual drive busy")
                return

            assert transport is not None
            await node._send_manual_drive_payload(
                transport, node._manual_drive_state_payload()
            )
            try:
                while True:
                    raw = await ws.receive_text()
                    try:
                        payload = json.loads(raw)
                        if not isinstance(payload, dict):
                            raise ManualDriveProtocolError(
                                "invalid_message",
                                "manual-drive frame must be a JSON object",
                            )
                        frame_type = payload.get("type")
                        if frame_type == "arm":
                            response = node._arm_manual_drive(
                                transport.lease, payload
                            )
                        elif frame_type == "drive":
                            response = node._apply_manual_drive(
                                transport.lease, payload
                            )
                        elif frame_type == "disarm":
                            response = node._disarm_manual_drive(
                                transport.lease, payload
                            )
                        else:
                            raise ManualDriveProtocolError(
                                "invalid_type",
                                "manual-drive type must be arm, drive, or disarm",
                            )
                    except json.JSONDecodeError as exc:
                        error = ManualDriveProtocolError(
                            "invalid_json", f"invalid manual-drive JSON: {exc}"
                        )
                        response = node._fail_closed_manual_drive(
                            transport.lease, error
                        )
                    except ManualDriveProtocolError as error:
                        response = node._fail_closed_manual_drive(
                            transport.lease, error
                        )
                    await node._send_manual_drive_payload(transport, response)
            except WebSocketDisconnect:
                pass
            except (KeyError, RuntimeError) as exc:
                node.get_logger().debug(
                    f"manual-drive websocket disconnected: {exc}"
                )
            finally:
                node._disconnect_manual_drive(transport.lease)

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

        # HH_260819 - A compact endpoint keeps the always-visible KPI strip
        # inexpensive; history is fetched only while its evidence modal is open.
        @app.get("/api/service-metrics/summary")
        def get_service_metrics_summary() -> JSONResponse:
            return JSONResponse(node._service_metrics.summary())

        @app.get("/api/service-metrics")
        def get_service_metrics(
            days: int = 30, recent_limit: int = 50
        ) -> JSONResponse:
            return JSONResponse(
                node._service_metrics.snapshot(
                    days=days,
                    recent_limit=recent_limit,
                )
            )

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

        @app.get("/api/camera/docking")
        def get_docking_camera() -> Response:
            # HH_260818 - The detector only renders while this endpoint's lazy
            # ROS subscription exists, avoiding idle image work on ARM64.
            return node._telemetry_camera_response("docking")

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

        @app.post("/ui/manual_return")
        def post_manual_return() -> JSONResponse:
            # HH_260818 - Operator docking tests may request return without a
            # preceding campsite WAIT_RETURN state.
            return JSONResponse(node.request_manual_return())

        @app.post("/ui/camping_site_recall")
        def post_camping_site_recall(
            site: str = "", intent: str = "recall"
        ) -> JSONResponse:
            if str(intent).strip().lower() != "recall":
                return JSONResponse(
                    {
                        "success": False,
                        "intent": str(intent),
                        "site": str(site).strip(),
                        "message": "intent must be recall",
                    },
                    status_code=400,
                )
            result = node.set_campsite_recall(site=site)
            return JSONResponse(
                result,
                status_code=200 if result.get("success") else 400,
            )

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
        def post_destination(
            site: str = "",
            run: str = "false",
            mission_generation: int = 0,
        ) -> JSONResponse:
            run_bool = run.lower() in {"1", "true", "yes", "on"}
            result = node.set_destination(
                site=site,
                run=run_bool,
                mission_generation=mission_generation,
            )
            return JSONResponse(result, status_code=200 if result.get("success") else 400)

        # ── Static frontend serving ───────────────────────────────────────────
        # Starlette 0.18 StaticFiles rejects symlinked files (commonprefix check
        # fails when realpath(file) points outside realpath(directory) due to
        # --symlink-install chains).  Serve all static assets manually so symlinks
        # are followed transparently.

        frontend_dir = node._resolve_frontend_dir()
        if frontend_dir and frontend_dir.exists():
            index_html = frontend_dir / "index.html"
            index_real = Path(os.path.realpath(str(index_html)))
            no_store_headers = {
                "Cache-Control": "no-store, no-cache, must-revalidate",
                "Pragma": "no-cache",
            }

            @app.get("/{full_path:path}")
            async def serve_spa(full_path: str) -> FileResponse:
                candidate = frontend_dir / full_path
                real = Path(os.path.realpath(str(candidate)))
                if real.is_file():
                    if real == index_real:
                        headers = no_store_headers
                    elif full_path.startswith("static/"):
                        # Create React App fingerprints files below static/.
                        headers = {"Cache-Control": "public, max-age=31536000, immutable"}
                    else:
                        # Site photos and other root assets keep stable names.
                        headers = {"Cache-Control": "no-cache"}
                    return FileResponse(str(real), headers=headers)
                # SPA routes all return index.html. It must be fetched on each
                # launch because it selects the current hashed JavaScript file.
                return FileResponse(str(index_real), headers=no_store_headers)
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
    # HH_260805 - launch-driven shutdown and terminal Ctrl+C are normal
    # paths and must leave the process with exit code 0.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    # HH_260807 - Humble can race context shutdown with take_message
    # conversion and raise RuntimeError. Suppress only after shutdown.
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
