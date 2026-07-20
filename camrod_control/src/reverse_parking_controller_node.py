#!/usr/bin/env python3
"""Drop-zone reverse parking controller."""

# HH_260720 - Move final reverse parking into the control package's parking module.

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import rclpy
import yaml
from avg_msgs.msg import (
    AvgAmrServiceState,
    AvgPlatformStatus,
    AvgPoseStamped,
    AvgTwist,
    ModuleState,
    MotionOperation,
)
from avg_msgs.srv import RequestMotionOperation
# HH_260720 - Import flattened shared helpers from the control source directory.
from control_support import (
    clamp,
    make_diagnostics,
    make_module_state,
    normalize_angle,
    yaw_from_pose,
)
from parking_geometry import (
    body_yaw_for_reverse_axis,
    reverse_axis_yaw_for_body,
    signed_distance_along_axis_m,
)
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ReverseParkingPhase(str, Enum):
    """Phases owned by the reverse parking controller."""

    IDLE = "IDLE"
    REVERSE_APPROACH = "REVERSE_APPROACH"
    PARKED = "PARKED"
    ERROR = "ERROR"


@dataclass
class StationPose:
    """Drop-zone station pose in the map frame."""

    x_m: float
    y_m: float
    yaw_rad: float


def quaternion_from_yaw(yaw_rad: float):
    # HH_260720 - The path is an explicit RViz boundary and therefore uses a ROS pose.
    orientation = RosPoseStamped().pose.orientation
    orientation.z = math.sin(yaw_rad * 0.5)
    orientation.w = math.cos(yaw_rad * 0.5)
    return orientation


class ReverseParkingControllerNode(Node):
    """Reverses toward the station after control reports yaw alignment complete."""

    def __init__(self) -> None:
        # HH_260720 - Name the node after its concrete reverse-parking control responsibility.
        super().__init__("reverse_parking_controller")

        # HH_260720 - Names describe the exact reverse-parking inputs and outputs.
        self.command_topic = str(
            self.declare_parameter("command_topic", "/control/cmd_vel_raw").value
        )
        self.vehicle_pose_topic = str(
            self.declare_parameter("vehicle_pose_topic", "/localization/pose").value
        )
        self.platform_status_topic = str(
            self.declare_parameter(
                "platform_status_topic", "/platform/status"
            ).value
        )
        self.operation_topic = str(
            self.declare_parameter(
                "operation_topic", "/parking/operation"
            ).value
        )
        self.status_topic = str(
            self.declare_parameter(
                "status_topic", "/parking/reverse_parking_controller/status"
            ).value
        )
        self.path_topic = str(
            self.declare_parameter(
                "path_topic", "/parking/reverse_parking_controller/path_ros"
            ).value
        )
        self.drop_zone_goal_topic = str(
            self.declare_parameter(
                "drop_zone_goal_topic", "/planning/drop_zone_goal_raw"
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

        # HH_260720 - Interpret the semantic drop-zone yaw as the commanded reverse
        # travel axis; standard ROS body yaw points 180 degrees in the other direction.
        self.station_yaw_represents_reverse_axis = bool(
            self.declare_parameter("station_yaw_represents_reverse_axis", True).value
        )
        self.automatically_select_reverse_approach_yaw = bool(
            self.declare_parameter(
                "automatically_select_reverse_approach_yaw", False
            ).value
        )
        self.start_yaw_tolerance_deg = abs(
            float(self.declare_parameter("start_yaw_tolerance_deg", 7.5).value)
        )
        self.reverse_speed_mps = abs(
            float(self.declare_parameter("reverse_speed_mps", 0.16).value)
        )
        self.heading_proportional_gain = float(
            self.declare_parameter("heading_proportional_gain", 0.8).value
        )
        self.lateral_proportional_gain = float(
            self.declare_parameter("lateral_proportional_gain", -0.25).value
        )
        self.maximum_angular_speed_radps = abs(
            float(
                self.declare_parameter("maximum_angular_speed_radps", 0.22).value
            )
        )
        self.maximum_reverse_distance_m = abs(
            float(
                self.declare_parameter("maximum_reverse_distance_m", 1.5).value
            )
        )
        self.reverse_timeout_s = abs(
            float(self.declare_parameter("reverse_timeout_s", 30.0).value)
        )
        self.require_station_on_reverse_axis = bool(
            self.declare_parameter("require_station_on_reverse_axis", True).value
        )
        self.station_axis_tolerance_m = abs(
            float(self.declare_parameter("station_axis_tolerance_m", 0.25).value)
        )
        self.stop_when_charging = bool(
            self.declare_parameter("stop_when_charging", True).value
        )
        self.complete_without_charging = bool(
            self.declare_parameter("complete_without_charging", True).value
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

        # HH_260720 - Parking publishes only the generated command contract internally.
        self.command_publisher = self.create_publisher(AvgTwist, self.command_topic, 10)
        self.status_publisher = self.create_publisher(ModuleState, self.status_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            type(
                make_diagnostics(
                    self, "", "", 0, "", hardware_id="camrod_control"
                )
            ),
            self.diagnostics_topic,
            10,
        )
        self.service_state_publisher = self.create_publisher(
            AvgAmrServiceState, self.amr_service_state_topic, 10
        )
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)

        self.create_subscription(
            AvgPoseStamped, self.vehicle_pose_topic, self._on_vehicle_pose, 10
        )
        self.create_subscription(
            AvgPlatformStatus,
            self.platform_status_topic,
            self._on_platform_status,
            10,
        )
        self.create_subscription(
            MotionOperation, self.operation_topic, self._on_operation, 10
        )
        self.create_subscription(
            AvgPoseStamped, self.drop_zone_goal_topic, self._on_drop_zone_goal, 10
        )
        # HH_260720 - One typed service replaces duplicate Trigger start/cancel services.
        self.create_service(
            RequestMotionOperation,
            "/parking/reverse_parking_controller/request_operation",
            self._on_operation_service,
        )

        self.phase = ReverseParkingPhase.IDLE
        self.last_vehicle_pose: AvgPoseStamped | None = None
        self.last_vehicle_pose_time_s = 0.0
        self.is_charging = False
        self.target_body_yaw_rad = self._configured_parking_body_yaw()
        self.reverse_start_x_m = 0.0
        self.reverse_start_y_m = 0.0
        self.phase_start_time_s = self._now_s()
        self.last_status_time_s = 0.0

        self.control_timer = self.create_timer(self._timer_period_s(), self._on_timer)
        self.get_logger().info(
            "reverse_parking_controller ready: "
            f"station=({self.station_pose.x_m:.2f},{self.station_pose.y_m:.2f},"
            f"{math.degrees(self.station_pose.yaw_rad):.1f}deg) "
            f"operation={self.operation_topic} command={self.command_topic}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_active(self) -> bool:
        return self.phase == ReverseParkingPhase.REVERSE_APPROACH

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
        # HH_260720 - Keep the parking body-yaw contract identical to drop_zone_maneuver_controller.
        if self.station_yaw_represents_reverse_axis:
            return body_yaw_for_reverse_axis(self.station_pose.yaw_rad)
        return normalize_angle(self.station_pose.yaw_rad)

    def _on_vehicle_pose(self, message: AvgPoseStamped) -> None:
        self.last_vehicle_pose = message
        self.last_vehicle_pose_time_s = self._now_s()

    def _on_platform_status(self, message: AvgPlatformStatus) -> None:
        # HH_260720 - Read charging from the canonical CAN-derived platform status.
        self.is_charging = bool(message.is_charging)

    def _on_drop_zone_goal(self, message: AvgPoseStamped) -> None:
        if self._is_active():
            self.get_logger().warn("ignored drop-zone goal update while reverse parking is active")
            return
        # HH_260720 - Use the same selected station pose as drop_zone_maneuver_controller.
        self.station_pose = StationPose(
            x_m=message.pose.position.x,
            y_m=message.pose.position.y,
            yaw_rad=yaw_from_pose(message),
        )
        self.target_body_yaw_rad = self._configured_parking_body_yaw()

    def _apply_operation(self, operation: int, source: str) -> tuple[bool, str]:
        # HH_260720 - Route all parking commands through one explicit operation contract.
        if operation == MotionOperation.START:
            return self._start_reverse_parking_controller(source)
        if operation == MotionOperation.CANCEL:
            self._cancel(source)
            return True, "reverse parking cancelled"
        return False, f"unsupported reverse parking operation={operation}"

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

    def _start_reverse_parking_controller(self, source: str) -> tuple[bool, str]:
        if self.phase == ReverseParkingPhase.REVERSE_APPROACH:
            return False, "reverse parking already active"
        if not self._vehicle_pose_is_fresh():
            self._set_error("fresh pose unavailable")
            return False, "fresh pose unavailable"
        self.target_body_yaw_rad = self._select_parking_body_yaw()
        current_yaw = yaw_from_pose(self.last_vehicle_pose)  # type: ignore[arg-type]
        yaw_error_deg = abs(
            math.degrees(normalize_angle(self.target_body_yaw_rad - current_yaw))
        )
        if yaw_error_deg > self.start_yaw_tolerance_deg:
            # HH_260720 - Reject direct reverse commands until control alignment is complete.
            self._set_error(
                f"parking yaw is not aligned: error={yaw_error_deg:.1f}deg "
                f"tolerance={self.start_yaw_tolerance_deg:.1f}deg"
            )
            return False, "parking yaw alignment required"
        self.reverse_start_x_m = self.last_vehicle_pose.pose.position.x  # type: ignore[union-attr]
        self.reverse_start_y_m = self.last_vehicle_pose.pose.position.y  # type: ignore[union-attr]
        self._set_phase(ReverseParkingPhase.REVERSE_APPROACH, f"start={source}")
        self._publish_path()
        return True, "reverse parking started"

    def _cancel(self, source: str) -> None:
        self._publish_zero()
        self._set_phase(ReverseParkingPhase.IDLE, f"cancel={source}")

    def _set_phase(self, phase: ReverseParkingPhase, detail: str) -> None:
        self.phase = phase
        self.phase_start_time_s = self._now_s()
        self._refresh_timer_period()
        self.get_logger().info(f"reverse_parking_controller {phase.value}: {detail}")
        self._publish_service_state(phase, detail)
        self._publish_status(force=True)

    def _set_error(self, detail: str) -> None:
        self._publish_zero()
        self._set_phase(ReverseParkingPhase.ERROR, detail)

    def _publish_service_state(self, phase: ReverseParkingPhase, detail: str) -> None:
        if phase == ReverseParkingPhase.REVERSE_APPROACH:
            state = AvgAmrServiceState.DROP_ZONE_PARKING
        elif phase == ReverseParkingPhase.PARKED:
            state = AvgAmrServiceState.DROP_ZONE_WAIT
        else:
            return
        message = AvgAmrServiceState()
        message.state = int(state)
        message.description = f"reverse_parking_controller:{phase.value}:{detail}"
        self.service_state_publisher.publish(message)

    def _publish_zero(self) -> None:
        self.command_publisher.publish(AvgTwist())

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

    def _station_distance_along_reverse_axis(self, body_yaw_rad: float | None = None) -> float:
        if self.last_vehicle_pose is None:
            return 0.0
        target_yaw = self.target_body_yaw_rad if body_yaw_rad is None else body_yaw_rad
        reverse_axis_yaw = reverse_axis_yaw_for_body(target_yaw)
        return signed_distance_along_axis_m(
            origin_x_m=self.last_vehicle_pose.pose.position.x,
            origin_y_m=self.last_vehicle_pose.pose.position.y,
            target_x_m=self.station_pose.x_m,
            target_y_m=self.station_pose.y_m,
            axis_yaw_rad=reverse_axis_yaw,
        )

    def _station_lateral_error_m(self) -> float:
        if self.last_vehicle_pose is None:
            return 0.0
        dx_m = self.last_vehicle_pose.pose.position.x - self.station_pose.x_m
        dy_m = self.last_vehicle_pose.pose.position.y - self.station_pose.y_m
        return -math.sin(self.station_pose.yaw_rad) * dx_m + math.cos(
            self.station_pose.yaw_rad
        ) * dy_m

    def _distance_reversed_m(self) -> float:
        if self.last_vehicle_pose is None:
            return 0.0
        dx_m = self.last_vehicle_pose.pose.position.x - self.reverse_start_x_m
        dy_m = self.last_vehicle_pose.pose.position.y - self.reverse_start_y_m
        return math.hypot(dx_m, dy_m)

    def _publish_path(self) -> None:
        if self.last_vehicle_pose is None:
            return
        frame_id = self.last_vehicle_pose.header.frame_id or "map"
        stamp = self.get_clock().now().to_msg()
        start_x_m = self.last_vehicle_pose.pose.position.x
        start_y_m = self.last_vehicle_pose.pose.position.y
        reverse_axis_yaw = reverse_axis_yaw_for_body(self.target_body_yaw_rad)
        station_distance_m = max(0.0, self._station_distance_along_reverse_axis())
        distance_m = min(
            station_distance_m if station_distance_m > 0.05 else self.maximum_reverse_distance_m,
            self.maximum_reverse_distance_m,
        )
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp
        step_count = max(1, int(max(distance_m, 0.05) / 0.25))
        for step_index in range(step_count + 1):
            ratio = step_index / step_count
            pose = RosPoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = start_x_m + math.cos(reverse_axis_yaw) * distance_m * ratio
            pose.pose.position.y = start_y_m + math.sin(reverse_axis_yaw) * distance_m * ratio
            pose.pose.orientation = quaternion_from_yaw(self.target_body_yaw_rad)
            path.poses.append(pose)
        self.path_publisher.publish(path)

    def _finish_or_fail_without_charging(self, detail: str) -> None:
        self._publish_zero()
        if self.complete_without_charging:
            self._set_phase(ReverseParkingPhase.PARKED, detail)
        else:
            self._set_error(f"{detail} without charging")

    def _publish_reverse_command(self) -> None:
        if self.stop_when_charging and self.is_charging:
            self._publish_zero()
            self._set_phase(ReverseParkingPhase.PARKED, "charging detected")
            return
        if not self._vehicle_pose_is_fresh():
            self._set_error("pose timeout during reverse parking")
            return

        distance_reversed_m = self._distance_reversed_m()
        if distance_reversed_m >= self.maximum_reverse_distance_m:
            self._finish_or_fail_without_charging("reverse distance limit reached")
            return
        if self._now_s() - self.phase_start_time_s >= self.reverse_timeout_s:
            self._set_error("reverse parking timeout")
            return

        station_axis_distance_m = self._station_distance_along_reverse_axis()
        if (
            self.require_station_on_reverse_axis
            and distance_reversed_m <= 0.05
            and station_axis_distance_m < -self.station_axis_tolerance_m
        ):
            self._set_error("station is behind the reverse axis; check drop-zone yaw")
            return
        if distance_reversed_m > 0.05 and station_axis_distance_m <= self.station_axis_tolerance_m:
            self._finish_or_fail_without_charging("station reverse axis reached")
            return

        current_yaw = yaw_from_pose(self.last_vehicle_pose)  # type: ignore[arg-type]
        heading_error = normalize_angle(self.target_body_yaw_rad - current_yaw)
        lateral_error_m = self._station_lateral_error_m()
        command = AvgTwist()
        command.linear.x = -self.reverse_speed_mps
        command.angular.z = clamp(
            self.heading_proportional_gain * heading_error
            + self.lateral_proportional_gain * lateral_error_m,
            -self.maximum_angular_speed_radps,
            self.maximum_angular_speed_radps,
        )
        self.command_publisher.publish(command)

    def _on_timer(self) -> None:
        if self.phase == ReverseParkingPhase.REVERSE_APPROACH:
            self._publish_reverse_command()
        # HH_260720 - Inactive parking never publishes zero into the shared raw command topic.
        self._publish_status()

    def _publish_status(self, force: bool = False) -> None:
        now_s = self._now_s()
        if (
            not force
            and self.last_status_time_s > 0.0
            and now_s - self.last_status_time_s < 1.0 / self.status_rate_hz
        ):
            return
        if self.phase == ReverseParkingPhase.ERROR:
            level = ModuleState.ERROR
        elif self.phase in {ReverseParkingPhase.IDLE, ReverseParkingPhase.PARKED}:
            level = ModuleState.OK
        else:
            level = ModuleState.WARN
        message = f"phase={self.phase.value} charging={self.is_charging}"
        self.status_publisher.publish(make_module_state(self, "parking", level, message))
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
                "parking/reverse_parking_controller",
                "parking",
                diagnostic_level,
                message,
                (
                    ("phase", self.phase.value),
                    ("command_topic", self.command_topic),
                    ("reverse_distance_m", f"{self._distance_reversed_m():.3f}"),
                    ("station_axis_distance_m", f"{self._station_distance_along_reverse_axis():.3f}"),
                    ("target_body_yaw_deg", f"{math.degrees(self.target_body_yaw_rad):.3f}"),
                ),
                hardware_id="camrod_control",
            )
        )
        self.last_status_time_s = now_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReverseParkingControllerNode()
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
