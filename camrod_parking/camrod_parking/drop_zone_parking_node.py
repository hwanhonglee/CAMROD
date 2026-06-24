#!/usr/bin/env python3
# HH_260617: Rule-based rear-yaw alignment and reverse parking controller.

from __future__ import annotations

import math
from enum import Enum

import rclpy
import yaml
from avg_msgs.msg import AvgAmrServiceState, ModuleState, PlanningScenario, PlanningState
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .utils import clamp, make_diagnostics, make_module_state, normalize_angle, yaw_from_pose


def quat_from_yaw(yaw: float):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Phase(str, Enum):
    IDLE = "IDLE"
    ALIGN_REAR_YAW = "ALIGN_REAR_YAW"
    REVERSE_APPROACH = "REVERSE_APPROACH"
    PARKED = "PARKED"
    ERROR = "ERROR"


class DropZoneParkingNode(Node):
    def __init__(self) -> None:
        super().__init__("drop_zone_parking")

        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/planning/cmd_vel_raw").value)
        self.pose_topic = str(self.declare_parameter("pose_topic", "/localization/pose").value)
        self.charging_topic = str(
            self.declare_parameter("charging_topic", "/platform/status/is_charging").value
        )
        self.planning_state_topic = str(
            self.declare_parameter("planning_state_topic", "/planning/state_machine/state").value
        )
        self.start_topic = str(self.declare_parameter("start_topic", "/parking/drop_zone/start").value)
        self.cancel_topic = str(self.declare_parameter("cancel_topic", "/parking/drop_zone/cancel").value)
        self.status_topic = str(self.declare_parameter("status_topic", "/parking/drop_zone/status").value)
        self.diagnostics_topic = str(self.declare_parameter("diagnostics_topic", "/system/diagnostics").value)
        # HHL_260622 - Drop-zone parking phase updates the UI/system service state.
        self.amr_service_state_topic = str(
            self.declare_parameter("amr_service_state_topic", "/AMR_service_state").value
        )
        # HH_260619 - Publish the planned drop-zone reverse approach path for RViz verification.
        self.reverse_path_topic = str(
            self.declare_parameter("reverse_path_topic", "/parking/drop_zone/reverse_path").value
        )
        # HHL_260624 - Planning publishes the selected raw drop-zone area here.
        # This keeps rule-based reverse parking aligned with the exact station
        # chosen from drop_zones.yaml instead of a generic first-entry fallback.
        self.drop_zone_goal_raw_topic = str(
            self.declare_parameter("drop_zone_goal_raw_topic", "/planning/drop_zone_goal_raw").value
        )

        self.drop_zones_yaml = str(self.declare_parameter("drop_zones_yaml", "").value)
        self.drop_zone_id = str(self.declare_parameter("drop_zone_id", "drop_zone").value)
        self.use_drop_zone_pose_as_station = bool(
            self.declare_parameter("use_drop_zone_pose_as_station", True).value
        )
        self.station_x_m = float(self.declare_parameter("station_x_m", 0.0).value)
        self.station_y_m = float(self.declare_parameter("station_y_m", 0.0).value)
        self.station_yaw_deg = float(self.declare_parameter("station_yaw_deg", 0.0).value)

        self.enable_auto_start_from_planning_state = bool(
            self.declare_parameter("enable_auto_start_from_planning_state", True).value
        )
        self.return_scenario_id = int(
            self.declare_parameter("return_scenario_id", int(PlanningScenario.RETURN_TO_DROP_ZONE)).value
        )
        self.return_mission_key = str(self.declare_parameter("return_mission_key", "drop_zone").value)

        # HH_260618: map/drop_zones.yaml yaw is treated as the desired parked
        # robot front yaw. Set rear_matches_station_yaw=true only for maps where
        # the station yaw explicitly describes the robot rear/charger-facing yaw.
        self.rear_matches_station_yaw = bool(
            self.declare_parameter("rear_matches_station_yaw", False).value
        )
        # HH_260619 - Drop-zone parking must not auto-select a 180-degree
        # equivalent body yaw. The 180-degree body rotation phase belongs only
        # to campsite handling; drop-zone parking aligns to the configured yaw
        # and reverses straight into the station.
        self.auto_select_reverse_yaw_to_station = bool(
            self.declare_parameter("auto_select_reverse_yaw_to_station", False).value
        )
        self.align_yaw_tolerance_deg = float(
            self.declare_parameter("align_yaw_tolerance_deg", 5.0).value
        )
        self.align_yaw_kp = float(self.declare_parameter("align_yaw_kp", 1.2).value)
        self.align_max_angular_speed_radps = abs(
            float(self.declare_parameter("align_max_angular_speed_radps", 0.35).value)
        )
        # HHL_260622 - Match config default for slightly faster reverse parking.
        self.reverse_speed_mps = abs(float(self.declare_parameter("reverse_speed_mps", 0.16).value))
        self.reverse_yaw_kp = float(self.declare_parameter("reverse_yaw_kp", 0.8).value)
        self.reverse_lateral_kp = float(self.declare_parameter("reverse_lateral_kp", -0.25).value)
        self.reverse_max_angular_speed_radps = abs(
            float(self.declare_parameter("reverse_max_angular_speed_radps", 0.22).value)
        )
        self.max_reverse_distance_m = float(self.declare_parameter("max_reverse_distance_m", 1.5).value)
        self.max_reverse_duration_s = float(self.declare_parameter("max_reverse_duration_s", 30.0).value)
        # HH_260618: Abort before backing out through the opposite side when the
        # configured yaw makes the station lie behind the commanded reverse axis.
        self.require_station_ahead_for_reverse = bool(
            self.declare_parameter("require_station_ahead_for_reverse", True).value
        )
        self.station_ahead_tolerance_m = float(
            self.declare_parameter("station_ahead_tolerance_m", 0.25).value
        )
        self.stop_on_charging = bool(self.declare_parameter("stop_on_charging", True).value)
        self.complete_on_reverse_limit_without_charging = bool(
            self.declare_parameter("complete_on_reverse_limit_without_charging", True).value
        )
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 2.0).value)
        # HH_260618: Reverse parking speed is low, so 10 Hz is enough for yaw
        # correction and reduces baseline Python timer load.
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 10.0).value)
        # HH_260618: Use a slow idle timer and switch to control_rate_hz only
        # while a parking maneuver is active.
        self.idle_tick_rate_hz = float(self.declare_parameter("idle_tick_rate_hz", 1.0).value)
        # HH_260618: Status/diagnostics do not need control-rate publishing.
        self.status_publish_rate_hz = float(
            self.declare_parameter("status_publish_rate_hz", 1.0).value
        )

        station_x_m, station_y_m, station_yaw_deg = self._resolve_station_pose()
        self._apply_station_pose(station_x_m, station_y_m, station_yaw_deg, "config", log=False)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(ModuleState, self.status_topic, 10)
        self.diag_pub = self.create_publisher(type(make_diagnostics(self, "", "", 0, "")), self.diagnostics_topic, 10)
        self.amr_service_pub = self.create_publisher(
            AvgAmrServiceState, self.amr_service_state_topic, 10
        )
        self.reverse_path_pub = self.create_publisher(Path, self.reverse_path_topic, 10)

        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 10)
        if self.drop_zone_goal_raw_topic:
            self.create_subscription(PoseStamped, self.drop_zone_goal_raw_topic, self._on_drop_zone_goal_raw, 10)
        self.create_subscription(Bool, self.charging_topic, self._on_charging, 10)
        self.create_subscription(PlanningState, self.planning_state_topic, self._on_planning_state, 10)
        self.create_subscription(Bool, self.start_topic, self._on_start_bool, 10)
        self.create_subscription(Bool, self.cancel_topic, self._on_cancel_bool, 10)
        self.create_service(Trigger, "/parking/drop_zone/start_service", self._on_start_service)
        self.create_service(Trigger, "/parking/drop_zone/cancel_service", self._on_cancel_service)

        self.phase = Phase.IDLE
        self.last_pose: PoseStamped | None = None
        self.last_pose_time_s = 0.0
        self.is_charging = False
        self.phase_start_s = self._now_s()
        self.reverse_start_x = 0.0
        self.reverse_start_y = 0.0
        self.reverse_start_station_ahead_m = 0.0
        self.last_auto_goal = ""
        self.last_status_publish_s = 0.0

        self.timer = self.create_timer(self._timer_period_s(), self._tick)
        self.get_logger().info(
            "drop_zone_parking ready: "
            f"station=({self.station_x_m:.2f}, {self.station_y_m:.2f}, "
            f"{self.station_yaw_deg:.1f}deg) raw_goal={self.drop_zone_goal_raw_topic or '(disabled)'} "
            f"cmd={self.cmd_vel_topic}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_active_phase(self) -> bool:
        return self.phase not in {Phase.IDLE, Phase.PARKED, Phase.ERROR}

    def _timer_period_s(self) -> float:
        rate_hz = self.control_rate_hz if self._is_active_phase() else self.idle_tick_rate_hz
        return 1.0 / max(rate_hz, 0.1)

    def _update_timer_period(self) -> None:
        if hasattr(self, "timer"):
            self.timer.timer_period_ns = int(self._timer_period_s() * 1e9)

    def _resolve_station_pose(self) -> tuple[float, float, float]:
        if not self.use_drop_zone_pose_as_station or not self.drop_zones_yaml:
            return self.station_x_m, self.station_y_m, self.station_yaw_deg
        try:
            with open(self.drop_zones_yaml, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"failed to read drop_zones_yaml={self.drop_zones_yaml}: {exc}")
            return self.station_x_m, self.station_y_m, self.station_yaw_deg

        zones = data.get("drop_zones", [])
        if not isinstance(zones, list) or not zones:
            return self.station_x_m, self.station_y_m, self.station_yaw_deg

        selected = None
        for zone in zones:
            if not isinstance(zone, dict):
                continue
            zone_id = str(zone.get("id", ""))
            zone_type = str(zone.get("type", ""))
            if zone_id == self.drop_zone_id or zone_type == self.drop_zone_id:
                selected = zone
                break
        if selected is None:
            selected = zones[0]

        return (
            float(selected.get("x", self.station_x_m)),
            float(selected.get("y", self.station_y_m)),
            float(selected.get("yaw_deg", self.station_yaw_deg)),
        )

    def _apply_station_pose(self, x_m: float, y_m: float, yaw_deg: float, source: str, log: bool = True) -> None:
        self.station_x_m = float(x_m)
        self.station_y_m = float(y_m)
        self.station_yaw_deg = float(yaw_deg)
        self.station_yaw = math.radians(self.station_yaw_deg)
        self.base_target_robot_yaw = normalize_angle(
            self.station_yaw + (math.pi if self.rear_matches_station_yaw else 0.0)
        )
        if not hasattr(self, "phase") or not self._is_active_phase():
            self.target_robot_yaw = self.base_target_robot_yaw
        if log:
            self.get_logger().info(
                "drop_zone_parking station updated: "
                f"source={source} xy=({self.station_x_m:.2f},{self.station_y_m:.2f}) "
                f"yaw={self.station_yaw_deg:.1f}deg"
            )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.last_pose = msg
        self.last_pose_time_s = self._now_s()

    def _on_drop_zone_goal_raw(self, msg: PoseStamped) -> None:
        if self._is_active_phase():
            self.get_logger().warn("ignored drop_zone_goal_raw while parking is active")
            return
        yaw_deg = math.degrees(yaw_from_pose(msg))
        self._apply_station_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            yaw_deg,
            "drop_zone_goal_raw",
        )

    def _on_charging(self, msg: Bool) -> None:
        self.is_charging = bool(msg.data)

    def _on_planning_state(self, msg: PlanningState) -> None:
        if not self.enable_auto_start_from_planning_state:
            return
        if (
            msg.state != PlanningState.GOAL_REACHED
            or msg.scenario_id != self.return_scenario_id
            or msg.active_mission_key.strip() != self.return_mission_key
        ):
            if self.phase in {Phase.IDLE, Phase.PARKED, Phase.ERROR}:
                self.last_auto_goal = ""
            return
        goal_key = f"{msg.scenario_id}:{msg.active_mission_key.strip()}"
        if goal_key == self.last_auto_goal:
            return
        self.last_auto_goal = goal_key
        self._start_sequence("planning_state:return_to_drop_zone")

    def _on_start_bool(self, msg: Bool) -> None:
        if msg.data:
            self._start_sequence("topic")

    def _on_cancel_bool(self, msg: Bool) -> None:
        if msg.data:
            self._cancel("topic")

    def _on_start_service(self, _request, response):
        ok, message = self._start_sequence("service")
        response.success = ok
        response.message = message
        return response

    def _on_cancel_service(self, _request, response):
        self._cancel("service")
        response.success = True
        response.message = "drop-zone parking cancelled"
        return response

    def _start_sequence(self, source: str) -> tuple[bool, str]:
        if self.phase not in {Phase.IDLE, Phase.PARKED, Phase.ERROR}:
            return False, f"parking already active: {self.phase.value}"
        if not self._pose_is_fresh():
            self._set_error("fresh pose unavailable")
            return False, "fresh pose unavailable"
        self._select_target_yaw_for_reverse_axis()
        self._set_phase(Phase.ALIGN_REAR_YAW, f"start={source}")
        return True, "drop-zone parking started"

    def _cancel(self, source: str) -> None:
        self._publish_zero()
        self._set_phase(Phase.IDLE, f"cancel={source}")

    def _pose_is_fresh(self) -> bool:
        return self.last_pose is not None and (self._now_s() - self.last_pose_time_s) <= self.pose_timeout_s

    def _set_phase(self, phase: Phase, message: str) -> None:
        self.phase = phase
        self.phase_start_s = self._now_s()
        self._update_timer_period()
        self.get_logger().info(f"drop_zone_parking {phase.value}: {message}")
        if phase == Phase.REVERSE_APPROACH and self.last_pose is not None:
            self.reverse_start_x = self.last_pose.pose.position.x
            self.reverse_start_y = self.last_pose.pose.position.y
            self.reverse_start_station_ahead_m = self._station_ahead_distance()
        if phase in {Phase.ALIGN_REAR_YAW, Phase.REVERSE_APPROACH}:
            self._publish_reverse_path()
        self._publish_amr_service_state_for_phase(phase, message)
        self._publish_status(force=True)

    def _set_error(self, message: str) -> None:
        self._publish_zero()
        self._set_phase(Phase.ERROR, message)

    def _publish_amr_service_state_for_phase(self, phase: Phase, source_message: str) -> None:
        # HHL_260622 - UI sees drop-zone parking as a dedicated post-navigation state.
        service_state = None
        if phase in {Phase.ALIGN_REAR_YAW, Phase.REVERSE_APPROACH}:
            service_state = AvgAmrServiceState.DROP_ZONE_PARKING
        elif phase == Phase.PARKED:
            service_state = AvgAmrServiceState.DROP_ZONE_WAIT

        if service_state is None:
            return

        msg = AvgAmrServiceState()
        msg.state = int(service_state)
        msg.description = f"drop_zone_parking:{phase.value}:{source_message}"
        self.amr_service_pub.publish(msg)

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_reverse_path(self) -> None:
        # HH_260619 - Visualize the low-speed rule-based drop-zone reverse path separately from Nav2 paths.
        if self.last_pose is None:
            return

        frame_id = self.last_pose.header.frame_id or "map"
        stamp = self.get_clock().now().to_msg()
        start_x = self.last_pose.pose.position.x
        start_y = self.last_pose.pose.position.y
        reverse_axis_yaw = self._reverse_axis_yaw()
        station_ahead_m = max(0.0, self._station_ahead_distance())
        max_distance_m = max(0.05, abs(self.max_reverse_distance_m))
        distance_m = station_ahead_m if station_ahead_m > 0.05 else max_distance_m
        distance_m = min(distance_m, max_distance_m)
        end_x = start_x + math.cos(reverse_axis_yaw) * distance_m
        end_y = start_y + math.sin(reverse_axis_yaw) * distance_m

        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp
        steps = max(1, int(distance_m / 0.25))
        for index in range(steps + 1):
            ratio = index / steps
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = start_x + (end_x - start_x) * ratio
            pose.pose.position.y = start_y + (end_y - start_y) * ratio
            pose.pose.orientation = quat_from_yaw(self.target_robot_yaw)
            path.poses.append(pose)
        self.reverse_path_pub.publish(path)

    def _publish_align(self) -> bool:
        if not self._pose_is_fresh():
            self._set_error("pose timeout during rear-yaw alignment")
            return False
        yaw = yaw_from_pose(self.last_pose)  # type: ignore[arg-type]
        err = normalize_angle(self.target_robot_yaw - yaw)
        if abs(math.degrees(err)) <= self.align_yaw_tolerance_deg:
            self._publish_zero()
            return True
        cmd = Twist()
        cmd.angular.z = clamp(
            self.align_yaw_kp * err,
            -self.align_max_angular_speed_radps,
            self.align_max_angular_speed_radps,
        )
        self.cmd_pub.publish(cmd)
        return False

    def _reverse_distance(self) -> float:
        if self.last_pose is None:
            return 0.0
        dx = self.last_pose.pose.position.x - self.reverse_start_x
        dy = self.last_pose.pose.position.y - self.reverse_start_y
        return math.hypot(dx, dy)

    def _station_lateral_error(self) -> float:
        if self.last_pose is None:
            return 0.0
        dx = self.last_pose.pose.position.x - self.station_x_m
        dy = self.last_pose.pose.position.y - self.station_y_m
        return -math.sin(self.station_yaw) * dx + math.cos(self.station_yaw) * dy

    def _reverse_axis_yaw(self) -> float:
        return normalize_angle(self.target_robot_yaw + math.pi)

    def _station_ahead_distance_for_target_yaw(self, target_yaw: float) -> float:
        if self.last_pose is None:
            return 0.0
        reverse_axis_yaw = normalize_angle(target_yaw + math.pi)
        dx = self.station_x_m - self.last_pose.pose.position.x
        dy = self.station_y_m - self.last_pose.pose.position.y
        return math.cos(reverse_axis_yaw) * dx + math.sin(reverse_axis_yaw) * dy

    def _station_ahead_distance(self) -> float:
        return self._station_ahead_distance_for_target_yaw(self.target_robot_yaw)

    def _select_target_yaw_for_reverse_axis(self) -> None:
        self.target_robot_yaw = self.base_target_robot_yaw
        if not self.auto_select_reverse_yaw_to_station or self.last_pose is None:
            return
        base_ahead = self._station_ahead_distance_for_target_yaw(self.base_target_robot_yaw)
        flipped_yaw = normalize_angle(self.base_target_robot_yaw + math.pi)
        flipped_ahead = self._station_ahead_distance_for_target_yaw(flipped_yaw)
        if flipped_ahead > base_ahead + 1e-3:
            self.target_robot_yaw = flipped_yaw
            self.get_logger().info(
                "drop_zone_parking selected flipped reverse yaw: "
                f"base_ahead={base_ahead:.2f}m flipped_ahead={flipped_ahead:.2f}m"
            )

    def _publish_reverse(self) -> bool:
        if self.stop_on_charging and self.is_charging:
            self._publish_zero()
            self._set_phase(Phase.PARKED, "charging detected")
            return True
        if not self._pose_is_fresh():
            self._set_error("pose timeout during reverse parking")
            return False

        elapsed = self._now_s() - self.phase_start_s
        distance = self._reverse_distance()
        if distance >= self.max_reverse_distance_m:
            self._publish_zero()
            if self.complete_on_reverse_limit_without_charging:
                self._set_phase(Phase.PARKED, "reverse distance limit reached")
            else:
                self._set_error("reverse distance limit reached without charging")
            return True
        if elapsed >= self.max_reverse_duration_s:
            self._publish_zero()
            self._set_error("reverse parking timeout")
            return False
        station_ahead_m = self._station_ahead_distance()
        if (
            self.require_station_ahead_for_reverse
            and distance <= 0.05
            and station_ahead_m < -abs(self.station_ahead_tolerance_m)
        ):
            self._set_error(
                "station is behind reverse axis; check drop_zone yaw or rear_matches_station_yaw"
            )
            return False
        if distance > 0.05 and station_ahead_m <= abs(self.station_ahead_tolerance_m):
            # HH_260618: Once reverse motion has started, the station moving from
            # ahead to near/behind the reverse axis means the vehicle has reached
            # the charger line, not that the original yaw choice was invalid.
            self._publish_zero()
            if self.complete_on_reverse_limit_without_charging:
                self._set_phase(Phase.PARKED, "station reverse axis reached")
            else:
                self._set_error("station reverse axis reached without charging")
            return True

        yaw = yaw_from_pose(self.last_pose)  # type: ignore[arg-type]
        heading_error = normalize_angle(self.target_robot_yaw - yaw)
        lateral_error = self._station_lateral_error()
        cmd = Twist()
        cmd.linear.x = -self.reverse_speed_mps
        cmd.angular.z = clamp(
            self.reverse_yaw_kp * heading_error + self.reverse_lateral_kp * lateral_error,
            -self.reverse_max_angular_speed_radps,
            self.reverse_max_angular_speed_radps,
        )
        self.cmd_pub.publish(cmd)
        return False

    def _tick(self) -> None:
        if self.phase == Phase.ALIGN_REAR_YAW:
            if self._publish_align():
                self._set_phase(Phase.REVERSE_APPROACH, "rear yaw aligned")
        elif self.phase == Phase.REVERSE_APPROACH:
            self._publish_reverse()
        elif self.phase in {Phase.IDLE, Phase.PARKED, Phase.ERROR}:
            # HH_260618: Do not publish zero Twist while inactive. Nav2 also
            # publishes /planning/cmd_vel_raw, and idle parking zeros can race
            # the controller output and stop normal driving.
            pass

        self._publish_status()

    def _publish_status(self, force: bool = False) -> None:
        now_s = self._now_s()
        period_s = 1.0 / max(self.status_publish_rate_hz, 0.1)
        if not force and self.last_status_publish_s > 0.0 and (now_s - self.last_status_publish_s) < period_s:
            return
        if self.phase == Phase.ERROR:
            level = ModuleState.ERROR
        elif self.phase in {Phase.IDLE, Phase.PARKED}:
            level = ModuleState.OK
        else:
            level = ModuleState.WARN
        message = f"phase={self.phase.value} charging={self.is_charging}"
        self.status_pub.publish(make_module_state(self, "parking", level, message))
        diag_level = DiagnosticStatus.ERROR if level == ModuleState.ERROR else (
            DiagnosticStatus.WARN if level == ModuleState.WARN else DiagnosticStatus.OK
        )
        self.diag_pub.publish(
            make_diagnostics(
                self,
                "parking/drop_zone_parking",
                "parking",
                diag_level,
                message,
                (
                    ("phase", self.phase.value),
                    ("station_x_m", f"{self.station_x_m:.3f}"),
                    ("station_y_m", f"{self.station_y_m:.3f}"),
                    ("station_yaw_deg", f"{self.station_yaw_deg:.3f}"),
                    ("target_robot_yaw_deg", f"{math.degrees(self.target_robot_yaw):.3f}"),
                    ("reverse_axis_yaw_deg", f"{math.degrees(self._reverse_axis_yaw()):.3f}"),
                    ("station_ahead_m", f"{self._station_ahead_distance():.3f}"),
                    ("reverse_distance_m", f"{self._reverse_distance():.3f}"),
                ),
            )
        )
        self.last_status_publish_s = now_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DropZoneParkingNode()
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
