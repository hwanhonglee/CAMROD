"""Fail-closed ROS 2 wrapper for the CAMROD command mapping."""

import math
import time

import rclpy
from avg_msgs.msg import ModuleState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from carla_extended_ackermann_msgs.msg import ExtendedAckermannDrive

from camrod_carla_adapter.command_mapping import (
    MappingConfig,
    command_age_timed_out,
    map_planar_twist,
    recovery_breakaway_is_authorized,
    rotation_recovery_breakaway_is_authorized,
    stop_command,
    validate_adapter_timing,
    validate_planar_axes,
    validate_ranger_contract,
    validate_recovery_breakaway_contract,
    validate_rotation_recovery_breakaway_contract,
)


class TwistToFourWSNode(Node):
    """Convert CAMROD's final Twist boundary without bypassing CARLA safety."""

    def __init__(self, **node_kwargs):
        super().__init__("camrod_twist_to_4ws", **node_kwargs)

        self.input_topic = self.declare_parameter(
            "input_topic", "/control/cmd_vel_ros").value
        self.output_topic = self.declare_parameter(
            "output_topic",
            "/carla/ego_vehicle/extended_ackermann_cmd",
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/camrod_carla/command_adapter/status").value
        self.recovery_breakaway_status_topic = self.declare_parameter(
            "recovery_breakaway_status_topic",
            "/control/route_safety_recovery_controller/status",
        ).value
        self.recovery_breakaway_enable = bool(self.declare_parameter(
            "recovery_breakaway_enable", False).value)
        self.rotation_recovery_breakaway_status_topic = self.declare_parameter(
            "rotation_recovery_breakaway_status_topic",
            "/control/camping_site_maneuver_controller/status",
        ).value
        self.rotation_recovery_breakaway_enable = bool(
            self.declare_parameter(
                "rotation_recovery_breakaway_enable", False
            ).value
        )
        self.base_frame_id = self.declare_parameter(
            "base_frame_id", "robot_center_link").value

        self.input_timeout_sec = float(self.declare_parameter(
            "input_timeout_sec", 0.35).value)
        self.watchdog_rate_hz = float(self.declare_parameter(
            "watchdog_rate_hz", 50.0).value)
        self.zero_publish_rate_hz = float(self.declare_parameter(
            "zero_publish_rate_hz", 10.0).value)
        self.unsupported_axis_tolerance = float(self.declare_parameter(
            "unsupported_axis_tolerance", 1.0e-6).value)
        self.recovery_breakaway_status_timeout_sec = float(
            self.declare_parameter(
                "recovery_breakaway_status_timeout_sec", 0.30
            ).value
        )
        self.rotation_recovery_breakaway_status_timeout_sec = float(
            self.declare_parameter(
                "rotation_recovery_breakaway_status_timeout_sec", 0.30
            ).value
        )
        self.recovery_breakaway_target_minimum_mps = float(
            self.declare_parameter(
                "recovery_breakaway_target_minimum_mps", 0.04
            ).value
        )
        self.recovery_breakaway_target_maximum_mps = float(
            self.declare_parameter(
                "recovery_breakaway_target_maximum_mps", 0.06
            ).value
        )
        self.config = MappingConfig(
            lateral_deadband_mps=float(self.declare_parameter(
                "lateral_deadband_mps", 0.02).value),
            angular_epsilon_radps=float(self.declare_parameter(
                "angular_epsilon_radps", 1.0e-6).value),
            minimum_turn_radius_m=float(self.declare_parameter(
                "minimum_turn_radius_m", 0.810330349).value),
            wheelbase_m=float(self.declare_parameter(
                "camrod_wheelbase_m", 0.90).value),
            max_ackermann_steer_rad=float(self.declare_parameter(
                "max_ackermann_steer_rad", 0.6981).value),
            max_crab_angle_rad=float(self.declare_parameter(
                "max_crab_angle_rad", math.radians(88.0)).value),
            max_ackermann_speed_mps=float(self.declare_parameter(
                "max_ackermann_speed_mps", 1.4).value),
            max_crab_speed_mps=float(self.declare_parameter(
                "max_crab_speed_mps", 1.0).value),
            max_yaw_rate_radps=float(self.declare_parameter(
                "max_yaw_rate_radps", 0.7853).value),
        )
        validate_ranger_contract(self.config)
        self._validate_timing()

        self.command_publisher = self.create_publisher(
            ExtendedAckermannDrive, self.output_topic, 10)
        self.status_publisher = self.create_publisher(
            DiagnosticArray, self.status_topic, 10)
        self.command_subscription = self.create_subscription(
            Twist, self.input_topic, self._on_twist, 10)

        recovery_status_qos = QoSProfile(depth=1)
        recovery_status_qos.reliability = ReliabilityPolicy.RELIABLE
        recovery_status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.recovery_status_subscription = None
        if self.recovery_breakaway_enable:
            self.recovery_status_subscription = self.create_subscription(
                ModuleState,
                self.recovery_breakaway_status_topic,
                self._on_recovery_status,
                recovery_status_qos,
            )
        self.rotation_recovery_status_subscription = None
        if self.rotation_recovery_breakaway_enable:
            # The existing campsite controller publishes a reliable VOLATILE
            # ModuleState heartbeat.  A transient-local request is DDS-
            # incompatible with that publisher and would silently receive no
            # authorization samples, so keep this QoS separate from the
            # route-safety recovery controller above.
            rotation_recovery_status_qos = QoSProfile(depth=1)
            rotation_recovery_status_qos.reliability = (
                ReliabilityPolicy.RELIABLE
            )
            rotation_recovery_status_qos.durability = (
                DurabilityPolicy.VOLATILE
            )
            self.rotation_recovery_status_subscription = (
                self.create_subscription(
                    ModuleState,
                    self.rotation_recovery_breakaway_status_topic,
                    self._on_rotation_recovery_status,
                    rotation_recovery_status_qos,
                )
            )

        self._last_valid_receive_monotonic = None
        self._last_zero_publish_monotonic = None
        self._last_status_publish_monotonic = None
        self._state = "WAITING_FOR_INPUT"
        self._detail = "no valid CAMROD Twist received"
        self._last_command = stop_command(self._detail)
        self._pure_crab_warning_emitted = False
        self._last_recovery_operating_state = ""
        self._last_recovery_status_receive_monotonic = None
        self._last_rotation_recovery_module_name = ""
        self._last_rotation_recovery_level = ModuleState.ERROR
        self._last_rotation_recovery_operating_state = ""
        self._last_rotation_recovery_message = ""
        self._last_rotation_recovery_status_receive_monotonic = None

        self.watchdog_timer = self.create_timer(
            1.0 / self.watchdog_rate_hz, self._watchdog_tick)
        self.get_logger().info(
            "CAMROD adapter ready: %s -> %s; timeout=%.3fs; "
            "recovery authority=%s%s/%.2fs; rotation recovery=%s%s/%.2fs"
            % (
                self.input_topic,
                self.output_topic,
                self.input_timeout_sec,
                "enabled:" if self.recovery_breakaway_enable else "disabled:",
                self.recovery_breakaway_status_topic,
                self.recovery_breakaway_status_timeout_sec,
                ("enabled:" if self.rotation_recovery_breakaway_enable
                 else "disabled:"),
                self.rotation_recovery_breakaway_status_topic,
                self.rotation_recovery_breakaway_status_timeout_sec,
            )
        )

    def _validate_timing(self):
        validate_adapter_timing(
            self.input_timeout_sec,
            self.watchdog_rate_hz,
            self.zero_publish_rate_hz,
            self.unsupported_axis_tolerance,
        )
        validate_recovery_breakaway_contract(
            self.recovery_breakaway_status_timeout_sec,
            self.recovery_breakaway_target_minimum_mps,
            self.recovery_breakaway_target_maximum_mps,
        )
        validate_rotation_recovery_breakaway_contract(
            self.rotation_recovery_breakaway_status_timeout_sec
        )

    def _on_recovery_status(self, message):
        """Cache only an authenticated, healthy recovery-controller state."""

        self._last_recovery_status_receive_monotonic = time.monotonic()
        if (
            message.module_name == "route_safety_recovery_controller"
            and int(message.level) == int(ModuleState.OK)
        ):
            self._last_recovery_operating_state = message.operating_state
        else:
            self._last_recovery_operating_state = ""

    def _on_rotation_recovery_status(self, message):
        """Cache the complete campsite identity used by the exact predicate."""

        self._last_rotation_recovery_status_receive_monotonic = time.monotonic()
        self._last_rotation_recovery_module_name = message.module_name
        self._last_rotation_recovery_level = int(message.level)
        self._last_rotation_recovery_operating_state = message.operating_state
        self._last_rotation_recovery_message = message.message

    def _recovery_breakaway_is_authorized(self, command, now=None):
        sample_now = time.monotonic() if now is None else now
        route_authorized = False
        if (self.recovery_breakaway_enable
                and self._last_recovery_status_receive_monotonic is not None):
            route_authorized = recovery_breakaway_is_authorized(
                command,
                self._last_recovery_operating_state,
                self._last_recovery_status_receive_monotonic,
                sample_now,
                self.recovery_breakaway_status_timeout_sec,
                self.recovery_breakaway_target_minimum_mps,
                self.recovery_breakaway_target_maximum_mps,
            )
        rotation_authorized = False
        if (self.rotation_recovery_breakaway_enable
                and self._last_rotation_recovery_status_receive_monotonic
                is not None):
            rotation_authorized = rotation_recovery_breakaway_is_authorized(
                command,
                self._last_rotation_recovery_module_name,
                self._last_rotation_recovery_level,
                self._last_rotation_recovery_operating_state,
                self._last_rotation_recovery_message,
                self._last_rotation_recovery_status_receive_monotonic,
                sample_now,
                self.rotation_recovery_breakaway_status_timeout_sec,
            )
        return bool(route_authorized or rotation_authorized)

    def _on_twist(self, message):
        now = time.monotonic()
        try:
            validate_planar_axes(
                message.linear.z,
                message.angular.x,
                message.angular.y,
                self.unsupported_axis_tolerance,
            )
            command = map_planar_twist(
                message.linear.x,
                message.linear.y,
                message.angular.z,
                self.config,
            )
        except (TypeError, ValueError) as exc:
            self._last_valid_receive_monotonic = None
            self._state = "INVALID_INPUT"
            self._detail = str(exc)
            self._last_command = stop_command(self._detail)
            self._publish_command(self._last_command)
            self._last_zero_publish_monotonic = now
            self.get_logger().error(
                "Rejected CAMROD Twist and published stop: %s" % exc)
            self._publish_status(force=True)
            return

        self._last_valid_receive_monotonic = now
        self._state = "ACTIVE"
        self._detail = command.reason
        self._last_command = command
        self._publish_command(command)
        if command.crab_angle_limited and not self._pure_crab_warning_emitted:
            self.get_logger().warning(
                "CAMROD requested near-pure lateral motion; CARLA's accepted "
                "physical limit clips it to %.2f deg"
                % math.degrees(self.config.max_crab_angle_rad)
            )
            self._pure_crab_warning_emitted = True
        self._publish_status(force=command.saturated)

    def _watchdog_tick(self):
        now = time.monotonic()
        if self._last_valid_receive_monotonic is None:
            self._publish_periodic_stop(now)
            self._publish_status()
            return

        if command_age_timed_out(
            self._last_valid_receive_monotonic,
            now,
            self.input_timeout_sec,
        ):
            self._state = "INPUT_TIMEOUT"
            self._detail = "CAMROD Twist age exceeded %.3fs" % (
                self.input_timeout_sec)
            self._last_command = stop_command(self._detail)
            self._last_valid_receive_monotonic = None
            self._publish_periodic_stop(now, force=True)
            self._publish_status(force=True)
            return
        self._publish_status()

    def _publish_periodic_stop(self, now, force=False):
        interval = 1.0 / self.zero_publish_rate_hz
        due = (
            self._last_zero_publish_monotonic is None
            or now - self._last_zero_publish_monotonic >= interval
        )
        if force or due:
            self._publish_command(stop_command(self._detail))
            self._last_zero_publish_monotonic = now

    def _publish_command(self, command):
        message = ExtendedAckermannDrive()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame_id
        message.drive_mode.mode = int(command.mode)
        message.speed = float(command.speed)
        message.acceleration = 0.0
        message.jerk = 0.0
        message.steering_angle = float(command.steering_angle)
        message.steering_angle_velocity = 0.0
        message.crab_angle = float(command.crab_angle)
        message.rear_steering_angle = float(command.rear_steering_angle)
        message.yaw_rate_cmd = float(command.yaw_rate_cmd)
        message.recovery_breakaway_authorized = bool(
            self._recovery_breakaway_is_authorized(command)
        )
        self.command_publisher.publish(message)

    def _publish_status(self, force=False):
        now = time.monotonic()
        if (
            not force
            and self._last_status_publish_monotonic is not None
            and now - self._last_status_publish_monotonic < 0.5
        ):
            return
        self._last_status_publish_monotonic = now
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "camrod_carla/command_adapter"
        status.hardware_id = "vehicle.ranger.default"
        if self._state == "ACTIVE":
            status.level = DiagnosticStatus.OK
            status.message = self._detail
        elif self._state == "WAITING_FOR_INPUT":
            status.level = DiagnosticStatus.WARN
            status.message = self._detail
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = self._detail
        status.values = [
            KeyValue(key="state", value=self._state),
            KeyValue(key="input_topic", value=str(self.input_topic)),
            KeyValue(key="output_topic", value=str(self.output_topic)),
            KeyValue(key="mode", value=str(self._last_command.mode)),
            KeyValue(key="speed_mps", value=str(self._last_command.speed)),
            KeyValue(
                key="steering_angle_rad",
                value=str(self._last_command.steering_angle),
            ),
            KeyValue(
                key="crab_angle_rad", value=str(self._last_command.crab_angle)
            ),
            KeyValue(
                key="yaw_rate_radps", value=str(self._last_command.yaw_rate_cmd)
            ),
            KeyValue(
                key="saturated", value=str(self._last_command.saturated).lower()
            ),
            KeyValue(
                key="recovery_breakaway_authorized",
                value=str(
                    self._recovery_breakaway_is_authorized(
                        self._last_command, now
                    )
                ).lower(),
            ),
        ]
        array.status = [status]
        self.status_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TwistToFourWSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
