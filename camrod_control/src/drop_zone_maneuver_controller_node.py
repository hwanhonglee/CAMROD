#!/usr/bin/env python3
"""Drop-zone departure and parking-yaw alignment controller."""

# HH_260720 - Separate non-parking drop-zone motion from the reverse parking controller.

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import rclpy
import yaml
from avg_msgs.msg import (
    AvgAmrServiceState,
    AvgBool,
    AvgPoseStamped,
    AvgTwist,
    ModuleState,
    MotionOperation,
    PlanningScenario,
    PlanningState,
)
from avg_msgs.srv import RequestMotionOperation
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# HH_260720 - Import flattened shared helpers from the control source directory.
from control_support import clamp, make_diagnostics, make_module_state, normalize_angle, yaw_from_pose
from parking_geometry import (
    body_yaw_for_reverse_axis,
    reverse_axis_yaw_for_body,
    signed_distance_along_axis_m,
)


class DropZoneManeuverPhase(str, Enum):
    """Control-owned phases around, but excluding, reverse parking."""

    IDLE = "IDLE"
    EXIT_STRAIGHT = "EXIT_STRAIGHT"
    ALIGN_EXIT_YAW = "ALIGN_EXIT_YAW"
    ALIGN_PARKING_YAW = "ALIGN_PARKING_YAW"
    ERROR = "ERROR"


@dataclass
class StationPose:
    """Configured drop-zone station pose in the map frame."""

    x_m: float
    y_m: float
    yaw_rad: float


def quaternion_from_yaw(yaw_rad: float):
    # HH_260720 - The exit path is an explicit RViz boundary and uses ROS poses.
    orientation = RosPoseStamped().pose.orientation
    orientation.z = math.sin(yaw_rad * 0.5)
    orientation.w = math.cos(yaw_rad * 0.5)
    return orientation


class DropZoneManeuverControllerNode(Node):
    """Executes drop-zone exit motion and aligns the body before reverse parking."""

    def __init__(self) -> None:
        # HH_260720 - Use a node name that states this controller owns drop-zone maneuvers.
        super().__init__("drop_zone_maneuver_controller")

        # HH_260720 - Use descriptive control-owned command and state interfaces.
        self.command_topic = str(
            self.declare_parameter("command_topic", "/control/cmd_vel_raw").value
        )
        self.vehicle_pose_topic = str(
            self.declare_parameter("vehicle_pose_topic", "/localization/pose").value
        )
        self.planning_state_topic = str(
            self.declare_parameter(
                "planning_state_topic", "/planning/state_machine/state"
            ).value
        )
        self.lanelet_pose_topic = str(
            self.declare_parameter(
                "lanelet_pose_topic", "/planning/lanelet_pose"
            ).value
        )
        self.drop_zone_goal_topic = str(
            self.declare_parameter(
                "drop_zone_goal_topic", "/planning/drop_zone_goal_raw"
            ).value
        )
        self.operation_topic = str(
            self.declare_parameter(
                "operation_topic", "/control/drop_zone_maneuver_controller/operation"
            ).value
        )
        self.exit_complete_topic = str(
            self.declare_parameter(
                "exit_complete_topic", "/control/drop_zone/exit_complete"
            ).value
        )
        self.status_topic = str(
            self.declare_parameter(
                "status_topic", "/control/drop_zone_maneuver_controller/status"
            ).value
        )
        self.exit_path_topic = str(
            self.declare_parameter(
                "exit_path_topic", "/control/drop_zone_maneuver_controller/exit_path_ros"
            ).value
        )
        self.parking_operation_topic = str(
            self.declare_parameter(
                "parking_operation_topic", "/parking/operation"
            ).value
        )
        self.diagnostics_topic = str(
            self.declare_parameter("diagnostics_topic", "/system/diagnostics").value
        )
        self.amr_service_state_topic = str(
            self.declare_parameter(
                "amr_service_state_topic", "/AMR_service_state"
            ).value
        )

        self.drop_zones_yaml = str(
            self.declare_parameter("drop_zones_yaml", "").value
        )
        self.drop_zone_id = str(
            self.declare_parameter("drop_zone_id", "drop_zone").value
        )
        self.use_drop_zone_pose_as_station = bool(
            self.declare_parameter("use_drop_zone_pose_as_station", True).value
        )
        configured_station = StationPose(
            x_m=float(self.declare_parameter("station_x_m", 0.0).value),
            y_m=float(self.declare_parameter("station_y_m", 0.0).value),
            yaw_rad=math.radians(
                float(self.declare_parameter("station_yaw_deg", 0.0).value)
            ),
        )
        self.station_pose = self._load_station_pose(configured_station)

        self.enable_auto_alignment = bool(
            self.declare_parameter("enable_auto_alignment", True).value
        )
        self.return_scenario_id = int(
            self.declare_parameter(
                "return_scenario_id", int(PlanningScenario.RETURN_TO_DROP_ZONE)
            ).value
        )
        self.return_mission_key = str(
            self.declare_parameter("return_mission_key", "drop_zone").value
        )
        # HH_260720 - The semantic drop-zone yaw points from the lanelet approach
        # toward the station, so it is the robot reverse axis rather than body front yaw.
        self.station_yaw_represents_reverse_axis = bool(
            self.declare_parameter("station_yaw_represents_reverse_axis", True).value
        )
        self.automatically_select_reverse_approach_yaw = bool(
            self.declare_parameter(
                "automatically_select_reverse_approach_yaw", False
            ).value
        )
        self.yaw_tolerance_deg = abs(
            float(self.declare_parameter("yaw_tolerance_deg", 5.0).value)
        )
        self.yaw_proportional_gain = float(
            self.declare_parameter("yaw_proportional_gain", 1.2).value
        )
        self.maximum_angular_speed_radps = abs(
            float(
                self.declare_parameter("maximum_angular_speed_radps", 0.35).value
            )
        )
        self.exit_distance_m = abs(
            float(self.declare_parameter("exit_distance_m", 1.2).value)
        )
        self.exit_speed_mps = abs(
            float(self.declare_parameter("exit_speed_mps", 0.16).value)
        )
        self.exit_timeout_s = abs(
            float(self.declare_parameter("exit_timeout_s", 30.0).value)
        )
        self.pose_timeout_s = abs(
            float(self.declare_parameter("pose_timeout_s", 2.0).value)
        )
        self.control_rate_hz = max(
            0.1, float(self.declare_parameter("control_rate_hz", 10.0).value)
        )
        self.idle_rate_hz = max(
            0.1, float(self.declare_parameter("idle_rate_hz", 1.0).value)
        )
        self.status_rate_hz = max(
            0.1, float(self.declare_parameter("status_rate_hz", 1.0).value)
        )

        # HH_260720 - Publish generated control and operation messages internally.
        self.command_publisher = self.create_publisher(AvgTwist, self.command_topic, 10)
        self.status_publisher = self.create_publisher(ModuleState, self.status_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            type(make_diagnostics(self, "", "", 0, "")),
            self.diagnostics_topic,
            10,
        )
        self.service_state_publisher = self.create_publisher(
            AvgAmrServiceState, self.amr_service_state_topic, 10
        )
        self.exit_path_publisher = self.create_publisher(Path, self.exit_path_topic, 10)
        self.exit_complete_publisher = self.create_publisher(
            AvgBool, self.exit_complete_topic, 10
        )
        self.parking_operation_publisher = self.create_publisher(
            MotionOperation, self.parking_operation_topic, 10
        )

        self.create_subscription(
            AvgPoseStamped, self.vehicle_pose_topic, self._on_vehicle_pose, 10
        )
        # HH_260720 - Lanelet poses are generated CAMROD messages, not ROS aliases.
        self.create_subscription(
            AvgPoseStamped, self.lanelet_pose_topic, self._on_lanelet_pose, 10
        )
        self.create_subscription(
            AvgPoseStamped, self.drop_zone_goal_topic, self._on_drop_zone_goal, 10
        )
        self.create_subscription(
            PlanningState, self.planning_state_topic, self._on_planning_state, 10
        )
        self.create_subscription(
            MotionOperation, self.operation_topic, self._on_operation, 10
        )

        # HH_260720 - One typed service replaces alignment/exit/cancel Trigger services.
        self.create_service(
            RequestMotionOperation,
            "/control/drop_zone_maneuver_controller/request_operation",
            self._on_operation_service,
        )

        self.phase = DropZoneManeuverPhase.IDLE
        self.last_vehicle_pose: AvgPoseStamped | None = None
        self.last_vehicle_pose_time_s = 0.0
        self.last_lanelet_pose: AvgPoseStamped | None = None
        self.last_lanelet_pose_time_s = 0.0
        self.phase_start_time_s = self._now_s()
        self.exit_start_x_m = 0.0
        self.exit_start_y_m = 0.0
        self.exit_heading_yaw_rad = 0.0
        self.target_body_yaw_rad = self._configured_parking_body_yaw()
        self.last_auto_alignment_key = ""
        self.last_status_time_s = 0.0

        self.control_timer = self.create_timer(self._timer_period_s(), self._on_timer)
        self.get_logger().info(
            "drop_zone_maneuver_controller ready: "
            f"station=({self.station_pose.x_m:.2f},{self.station_pose.y_m:.2f},"
            f"{math.degrees(self.station_pose.yaw_rad):.1f}deg) "
            f"command={self.command_topic} parking_operation={self.parking_operation_topic}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_active(self) -> bool:
        return self.phase not in {
            DropZoneManeuverPhase.IDLE,
            DropZoneManeuverPhase.ERROR,
        }

    def _timer_period_s(self) -> float:
        return 1.0 / (self.control_rate_hz if self._is_active() else self.idle_rate_hz)

    def _refresh_timer_period(self) -> None:
        if hasattr(self, "control_timer"):
            self.control_timer.timer_period_ns = int(self._timer_period_s() * 1e9)

    def _load_station_pose(self, fallback: StationPose) -> StationPose:
        if not self.use_drop_zone_pose_as_station or not self.drop_zones_yaml:
            return fallback
        try:
            with open(self.drop_zones_yaml, "r", encoding="utf-8") as stream:
                document = yaml.safe_load(stream) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"failed to read drop_zones_yaml={self.drop_zones_yaml}: {exc}"
            )
            return fallback

        zones = document.get("drop_zones", [])
        if not isinstance(zones, list) or not zones:
            return fallback
        selected = next(
            (
                zone
                for zone in zones
                if isinstance(zone, dict)
                and (
                    str(zone.get("id", "")) == self.drop_zone_id
                    or str(zone.get("type", "")) == self.drop_zone_id
                )
            ),
            zones[0],
        )
        if not isinstance(selected, dict):
            return fallback
        return StationPose(
            x_m=float(selected.get("x", fallback.x_m)),
            y_m=float(selected.get("y", fallback.y_m)),
            yaw_rad=math.radians(
                float(selected.get("yaw_deg", math.degrees(fallback.yaw_rad)))
            ),
        )

    def _configured_parking_body_yaw(self) -> float:
        # HH_260720 - A reverse-axis station yaw requires the body front to face 180 degrees away.
        if self.station_yaw_represents_reverse_axis:
            return body_yaw_for_reverse_axis(self.station_pose.yaw_rad)
        return normalize_angle(self.station_pose.yaw_rad)

    def _on_vehicle_pose(self, message: AvgPoseStamped) -> None:
        self.last_vehicle_pose = message
        self.last_vehicle_pose_time_s = self._now_s()

    def _on_lanelet_pose(self, message: AvgPoseStamped) -> None:
        self.last_lanelet_pose = message
        self.last_lanelet_pose_time_s = self._now_s()

    def _on_drop_zone_goal(self, message: AvgPoseStamped) -> None:
        if self._is_active():
            self.get_logger().warn("ignored drop-zone goal update while maneuver is active")
            return
        # HH_260720 - Both control alignment and parking consume the exact selected station.
        self.station_pose = StationPose(
            x_m=message.pose.position.x,
            y_m=message.pose.position.y,
            yaw_rad=yaw_from_pose(message),
        )
        self.target_body_yaw_rad = self._configured_parking_body_yaw()

    def _on_planning_state(self, message: PlanningState) -> None:
        if not self.enable_auto_alignment:
            return
        is_drop_zone_arrival = (
            message.state == PlanningState.GOAL_REACHED
            and message.scenario_id == self.return_scenario_id
            and message.active_mission_key.strip() == self.return_mission_key
        )
        if not is_drop_zone_arrival:
            if not self._is_active():
                self.last_auto_alignment_key = ""
            return
        alignment_key = f"{message.scenario_id}:{message.active_mission_key.strip()}"
        if alignment_key == self.last_auto_alignment_key:
            return
        self.last_auto_alignment_key = alignment_key
        self._start_parking_alignment("planning_state:return_to_drop_zone")

    def _apply_operation(self, operation: int, source: str) -> tuple[bool, str]:
        # HH_260720 - Keep every drop-zone command explicit in one operation dispatcher.
        if operation == MotionOperation.ALIGN_FOR_PARKING:
            return self._start_parking_alignment(source)
        if operation == MotionOperation.EXIT:
            return self._start_exit(source)
        if operation == MotionOperation.CANCEL:
            self._cancel(source)
            return True, "drop-zone maneuver cancelled"
        return False, f"unsupported drop-zone operation={operation}"

    def _on_operation(self, message: MotionOperation) -> None:
        self._apply_operation(int(message.operation), message.source or "topic")

    def _on_operation_service(self, request, response):
        accepted, detail = self._apply_operation(
            int(request.operation), request.source or "service"
        )
        response.accepted = accepted
        response.message = detail
        return response

    def _vehicle_pose_is_fresh(self) -> bool:
        return (
            self.last_vehicle_pose is not None
            and self._now_s() - self.last_vehicle_pose_time_s <= self.pose_timeout_s
        )

    def _lanelet_pose_is_fresh(self) -> bool:
        return (
            self.last_lanelet_pose is not None
            and self._now_s() - self.last_lanelet_pose_time_s <= self.pose_timeout_s
        )

    def _start_parking_alignment(self, source: str) -> tuple[bool, str]:
        if self._is_active():
            return False, f"drop-zone maneuver already active: {self.phase.value}"
        if not self._vehicle_pose_is_fresh():
            self._set_error("fresh pose unavailable for parking alignment")
            return False, "fresh pose unavailable for parking alignment"
        self.target_body_yaw_rad = self._select_parking_body_yaw()
        self._set_phase(
            DropZoneManeuverPhase.ALIGN_PARKING_YAW, f"start={source}"
        )
        return True, "drop-zone parking alignment started"

    def _start_exit(self, source: str) -> tuple[bool, str]:
        if self._is_active():
            self._publish_exit_complete(False)
            return False, f"drop-zone maneuver already active: {self.phase.value}"
        if not self._vehicle_pose_is_fresh():
            self._publish_exit_complete(False)
            self._set_error("fresh pose unavailable for drop-zone exit")
            return False, "fresh pose unavailable for drop-zone exit"
        pose = self.last_vehicle_pose  # type: ignore[assignment]
        self.exit_start_x_m = pose.pose.position.x
        self.exit_start_y_m = pose.pose.position.y
        self.exit_heading_yaw_rad = yaw_from_pose(pose)
        self.target_body_yaw_rad = self._select_exit_yaw()
        self._set_phase(DropZoneManeuverPhase.EXIT_STRAIGHT, f"start={source}")
        return True, "drop-zone exit started"

    def _cancel(self, source: str) -> None:
        self._publish_zero()
        self._set_phase(DropZoneManeuverPhase.IDLE, f"cancel={source}")

    def _set_phase(self, phase: DropZoneManeuverPhase, detail: str) -> None:
        self.phase = phase
        self.phase_start_time_s = self._now_s()
        self._refresh_timer_period()
        self.get_logger().info(f"drop_zone_maneuver_controller {phase.value}: {detail}")
        if phase in {
            DropZoneManeuverPhase.EXIT_STRAIGHT,
            DropZoneManeuverPhase.ALIGN_EXIT_YAW,
        }:
            self._publish_exit_path()
        self._publish_service_state(phase, detail)
        self._publish_status(force=True)

    def _set_error(self, detail: str) -> None:
        self._publish_zero()
        self._set_phase(DropZoneManeuverPhase.ERROR, detail)

    def _publish_service_state(
        self, phase: DropZoneManeuverPhase, detail: str
    ) -> None:
        if phase in {
            DropZoneManeuverPhase.EXIT_STRAIGHT,
            DropZoneManeuverPhase.ALIGN_EXIT_YAW,
        }:
            state = AvgAmrServiceState.MOVING_TO_SITE
        elif phase == DropZoneManeuverPhase.ALIGN_PARKING_YAW:
            state = AvgAmrServiceState.DROP_ZONE_PARKING
        else:
            return
        message = AvgAmrServiceState()
        message.state = int(state)
        message.description = f"drop_zone_maneuver_controller:{phase.value}:{detail}"
        self.service_state_publisher.publish(message)

    def _publish_zero(self) -> None:
        self.command_publisher.publish(AvgTwist())

    def _publish_exit_complete(self, success: bool) -> None:
        message = AvgBool()
        message.data = bool(success)
        self.exit_complete_publisher.publish(message)

    def _request_parking_start(self) -> None:
        # HH_260720 - The selected parking controller starts only after yaw convergence.
        message = MotionOperation()
        message.header.stamp = self.get_clock().now().to_msg()
        message.operation = MotionOperation.START
        message.source = "drop_zone_maneuver_controller"
        self.parking_operation_publisher.publish(message)

    def _publish_exit_path(self) -> None:
        if self.last_vehicle_pose is None:
            return
        frame_id = self.last_vehicle_pose.header.frame_id or "map"
        stamp = self.get_clock().now().to_msg()
        end_x_m = self.exit_start_x_m + math.cos(self.exit_heading_yaw_rad) * self.exit_distance_m
        end_y_m = self.exit_start_y_m + math.sin(self.exit_heading_yaw_rad) * self.exit_distance_m
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp
        step_count = max(1, int(self.exit_distance_m / 0.25))
        for step_index in range(step_count + 1):
            ratio = step_index / step_count
            pose = RosPoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = self.exit_start_x_m + (end_x_m - self.exit_start_x_m) * ratio
            pose.pose.position.y = self.exit_start_y_m + (end_y_m - self.exit_start_y_m) * ratio
            pose.pose.orientation = quaternion_from_yaw(self.exit_heading_yaw_rad)
            path.poses.append(pose)
        self.exit_path_publisher.publish(path)

    def _select_parking_body_yaw(self) -> float:
        configured_yaw = self._configured_parking_body_yaw()
        if (
            not self.automatically_select_reverse_approach_yaw
            or self.last_vehicle_pose is None
        ):
            return configured_yaw
        flipped_yaw = normalize_angle(configured_yaw + math.pi)
        configured_distance = self._station_distance_along_reverse_axis(configured_yaw)
        flipped_distance = self._station_distance_along_reverse_axis(flipped_yaw)
        return flipped_yaw if flipped_distance > configured_distance + 1e-3 else configured_yaw

    def _station_distance_along_reverse_axis(self, body_yaw_rad: float) -> float:
        if self.last_vehicle_pose is None:
            return 0.0
        reverse_axis_yaw = reverse_axis_yaw_for_body(body_yaw_rad)
        return signed_distance_along_axis_m(
            origin_x_m=self.last_vehicle_pose.pose.position.x,
            origin_y_m=self.last_vehicle_pose.pose.position.y,
            target_x_m=self.station_pose.x_m,
            target_y_m=self.station_pose.y_m,
            axis_yaw_rad=reverse_axis_yaw,
        )

    def _select_exit_yaw(self) -> float:
        current_yaw = yaw_from_pose(self.last_vehicle_pose)  # type: ignore[arg-type]
        desired_yaw = (
            yaw_from_pose(self.last_lanelet_pose)  # type: ignore[arg-type]
            if self._lanelet_pose_is_fresh()
            else self._configured_parking_body_yaw()
        )
        return normalize_angle(current_yaw + normalize_angle(desired_yaw - current_yaw))

    def _publish_alignment_command(self) -> bool:
        if not self._vehicle_pose_is_fresh():
            self._set_error("pose timeout during drop-zone yaw alignment")
            return False
        current_yaw = yaw_from_pose(self.last_vehicle_pose)  # type: ignore[arg-type]
        yaw_error = normalize_angle(self.target_body_yaw_rad - current_yaw)
        if abs(math.degrees(yaw_error)) <= self.yaw_tolerance_deg:
            self._publish_zero()
            return True
        command = AvgTwist()
        command.angular.z = clamp(
            self.yaw_proportional_gain * yaw_error,
            -self.maximum_angular_speed_radps,
            self.maximum_angular_speed_radps,
        )
        self.command_publisher.publish(command)
        return False

    def _distance_from_exit_start(self) -> float:
        if self.last_vehicle_pose is None:
            return 0.0
        dx_m = self.last_vehicle_pose.pose.position.x - self.exit_start_x_m
        dy_m = self.last_vehicle_pose.pose.position.y - self.exit_start_y_m
        return math.hypot(dx_m, dy_m)

    def _publish_exit_command(self) -> None:
        if not self._vehicle_pose_is_fresh():
            self._publish_exit_complete(False)
            self._set_error("pose timeout during drop-zone exit")
            return
        if self._distance_from_exit_start() >= self.exit_distance_m:
            self._publish_zero()
            self.target_body_yaw_rad = self._select_exit_yaw()
            self._set_phase(
                DropZoneManeuverPhase.ALIGN_EXIT_YAW, "straight exit complete"
            )
            return
        if self._now_s() - self.phase_start_time_s >= self.exit_timeout_s:
            self._publish_exit_complete(False)
            self._set_error("drop-zone exit timeout")
            return
        current_yaw = yaw_from_pose(self.last_vehicle_pose)  # type: ignore[arg-type]
        heading_error = normalize_angle(self.exit_heading_yaw_rad - current_yaw)
        command = AvgTwist()
        command.linear.x = self.exit_speed_mps
        command.angular.z = clamp(
            self.yaw_proportional_gain * heading_error,
            -self.maximum_angular_speed_radps,
            self.maximum_angular_speed_radps,
        )
        self.command_publisher.publish(command)

    def _on_timer(self) -> None:
        if self.phase == DropZoneManeuverPhase.EXIT_STRAIGHT:
            self._publish_exit_command()
        elif self.phase == DropZoneManeuverPhase.ALIGN_EXIT_YAW:
            if self._publish_alignment_command():
                self._publish_exit_complete(True)
                self._set_phase(DropZoneManeuverPhase.IDLE, "drop-zone exit aligned")
        elif self.phase == DropZoneManeuverPhase.ALIGN_PARKING_YAW:
            if self._publish_alignment_command():
                self._request_parking_start()
                self._set_phase(
                    DropZoneManeuverPhase.IDLE,
                    "parking yaw aligned; reverse parking requested",
                )
        self._publish_status()

    def _publish_status(self, force: bool = False) -> None:
        now_s = self._now_s()
        if (
            not force
            and self.last_status_time_s > 0.0
            and now_s - self.last_status_time_s < 1.0 / self.status_rate_hz
        ):
            return
        if self.phase == DropZoneManeuverPhase.ERROR:
            level = ModuleState.ERROR
        elif self.phase == DropZoneManeuverPhase.IDLE:
            level = ModuleState.OK
        else:
            level = ModuleState.WARN
        message = f"phase={self.phase.value} target_yaw_deg={math.degrees(self.target_body_yaw_rad):.2f}"
        self.status_publisher.publish(make_module_state(self, "control", level, message))
        diagnostic_level = (
            DiagnosticStatus.ERROR
            if level == ModuleState.ERROR
            else DiagnosticStatus.WARN
            if level == ModuleState.WARN
            else DiagnosticStatus.OK
        )
        self.diagnostics_publisher.publish(
            make_diagnostics(
                self,
                "control/drop_zone_maneuver_controller",
                "control",
                diagnostic_level,
                message,
                (
                    ("phase", self.phase.value),
                    ("command_topic", self.command_topic),
                    ("exit_distance_m", f"{self._distance_from_exit_start():.3f}"),
                    ("station_x_m", f"{self.station_pose.x_m:.3f}"),
                    ("station_y_m", f"{self.station_pose.y_m:.3f}"),
                    ("station_yaw_deg", f"{math.degrees(self.station_pose.yaw_rad):.3f}"),
                ),
            )
        )
        self.last_status_time_s = now_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DropZoneManeuverControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
