"""Simulated raw Ranger platform heartbeat for the CAMROD CARLA adapter.

The dataclasses and mapping helpers in this module deliberately do not depend
on ROS.  Only :class:`CarlaPlatformHeartbeatNode` converts their values into
ROS messages, which keeps the safety mapping testable on a plain Python host.
"""

import math
import threading
from dataclasses import dataclass, replace
from enum import IntEnum


UINT16_MAX = 0xFFFF


class VehicleState(IntEnum):
    """Values defined by ``ranger_msgs/SystemState``."""

    NORMAL = 0
    ESTOP = 1
    EXCEPTION = 2


class ControlMode(IntEnum):
    """Values defined by ``ranger_msgs/SystemState``."""

    RC = 0
    CAN = 1


class MotionMode(IntEnum):
    """Values defined by ``ranger_msgs/SystemState``."""

    DUAL_ACKERMANN = 0
    PARALLEL = 1
    SPINNING = 2
    SIDE_SLIP = 3


class PowerSupplyStatus(IntEnum):
    """Values defined by ``sensor_msgs/BatteryState``."""

    UNKNOWN = 0
    CHARGING = 1
    DISCHARGING = 2
    NOT_CHARGING = 3
    FULL = 4


class PowerSupplyHealth(IntEnum):
    """Subset of ``sensor_msgs/BatteryState`` health values used here."""

    UNKNOWN = 0
    GOOD = 1


class PowerSupplyTechnology(IntEnum):
    """Subset of ``sensor_msgs/BatteryState`` technology values used here."""

    UNKNOWN = 0
    LION = 2


@dataclass(frozen=True)
class HeartbeatConfig:
    """Static properties of the simulated platform heartbeat."""

    publish_rate_hz: float = 5.0
    frame_id: str = "robot_center_link"
    battery_voltage_v: float = 48.0
    charging_current_a: float = 1.0
    discharging_current_a: float = -1.0
    battery_temperature_c: float = 25.0
    battery_location: str = "carla:simulated-platform"


@dataclass(frozen=True)
class HeartbeatInputs:
    """Runtime overrides accepted from the four test-control topics."""

    charging: object = False
    soc: object = 0.8
    estop: object = False
    error_code: object = 0


@dataclass(frozen=True)
class SystemStateValues:
    """ROS-independent values for one ``ranger_msgs/SystemState``."""

    vehicle_state: int
    control_mode: int
    error_code: int
    battery_voltage: float
    motion_mode: int


@dataclass(frozen=True)
class BatteryStateValues:
    """ROS-independent values for one ``sensor_msgs/BatteryState``."""

    voltage: float
    temperature: float
    current: float
    percentage: float
    power_supply_status: int
    power_supply_health: int
    power_supply_technology: int
    present: bool
    location: str


@dataclass(frozen=True)
class PlatformHeartbeatValues:
    """One coherent pair of raw Ranger heartbeat values."""

    system: SystemStateValues
    battery: BatteryStateValues


def validate_heartbeat_config(config):
    """Return a normalized configuration or reject an unhealthy one."""
    rate = _finite_float(config.publish_rate_hz, None)
    if rate is None or rate < 1.0 or rate > 20.0:
        raise ValueError("publish_rate_hz must be finite and in [1, 20]")

    frame_id = str(config.frame_id).strip()
    location = str(config.battery_location).strip()
    if not frame_id:
        raise ValueError("frame_id must not be empty")
    if not location:
        raise ValueError("battery_location must not be empty")

    voltage = _finite_float(config.battery_voltage_v, None)
    if voltage is None or voltage <= 0.0:
        raise ValueError("battery_voltage_v must be finite and positive")

    charging_current = _finite_float(config.charging_current_a, None)
    if charging_current is None or charging_current <= 0.0:
        raise ValueError("charging_current_a must be finite and positive")

    discharging_current = _finite_float(
        config.discharging_current_a, None)
    if discharging_current is None or discharging_current >= 0.0:
        raise ValueError(
            "discharging_current_a must be finite and negative")

    temperature = _finite_float(config.battery_temperature_c, None)
    if temperature is None:
        raise ValueError("battery_temperature_c must be finite")

    return HeartbeatConfig(
        publish_rate_hz=rate,
        frame_id=frame_id,
        battery_voltage_v=voltage,
        charging_current_a=charging_current,
        discharging_current_a=discharging_current,
        battery_temperature_c=temperature,
        battery_location=location,
    )


def sanitize_soc(value):
    """Normalize SOC to [0, 1], failing non-finite input to empty."""
    numeric = _finite_float(value, None)
    if numeric is None:
        return 0.0
    return min(1.0, max(0.0, numeric))


def sanitize_error_code(value):
    """Normalize an error bitmap, failing malformed input to all faults."""
    numeric = _finite_float(value, None)
    if numeric is None:
        return UINT16_MAX
    return min(UINT16_MAX, max(0, int(numeric)))


def sanitize_flag(value, fail_safe):
    """Accept Bool-like 0/1 values and fail malformed values explicitly."""
    if isinstance(value, bool):
        return value
    numeric = _finite_float(value, None)
    if numeric in (0.0, 1.0):
        return bool(numeric)
    return bool(fail_safe)


def sanitize_heartbeat_inputs(inputs):
    """Apply safety policy to externally supplied heartbeat overrides."""
    return HeartbeatInputs(
        charging=sanitize_flag(inputs.charging, fail_safe=False),
        soc=sanitize_soc(inputs.soc),
        estop=sanitize_flag(inputs.estop, fail_safe=True),
        error_code=sanitize_error_code(inputs.error_code),
    )


def build_platform_heartbeat(inputs, config=HeartbeatConfig()):
    """Map override state to a coherent raw Ranger/BMS heartbeat pair."""
    checked_config = validate_heartbeat_config(config)
    checked_inputs = sanitize_heartbeat_inputs(inputs)

    vehicle_state = (
        VehicleState.ESTOP if checked_inputs.estop else VehicleState.NORMAL
    )
    battery_status = (
        PowerSupplyStatus.CHARGING
        if checked_inputs.charging
        else PowerSupplyStatus.DISCHARGING
    )
    battery_current = (
        checked_config.charging_current_a
        if checked_inputs.charging
        else checked_config.discharging_current_a
    )

    return PlatformHeartbeatValues(
        system=SystemStateValues(
            vehicle_state=int(vehicle_state),
            control_mode=int(ControlMode.CAN),
            error_code=int(checked_inputs.error_code),
            battery_voltage=checked_config.battery_voltage_v,
            motion_mode=int(MotionMode.DUAL_ACKERMANN),
        ),
        battery=BatteryStateValues(
            voltage=checked_config.battery_voltage_v,
            temperature=checked_config.battery_temperature_c,
            current=battery_current,
            percentage=float(checked_inputs.soc),
            power_supply_status=int(battery_status),
            power_supply_health=int(PowerSupplyHealth.GOOD),
            power_supply_technology=int(PowerSupplyTechnology.LION),
            present=True,
            location=checked_config.battery_location,
        ),
    )


def _finite_float(value, fallback):
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return fallback
    return numeric if math.isfinite(numeric) else fallback


_ROS_IMPORT_ERROR = None
try:
    import rclpy
    from ranger_msgs.msg import SystemState
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, Float32, UInt16
except ImportError as error:  # Pure mapping remains importable without ROS.
    _ROS_IMPORT_ERROR = error
    rclpy = None
    SystemState = None
    ExternalShutdownException = Exception
    Node = object
    QoSProfile = None
    QoSReliabilityPolicy = None
    BatteryState = None
    Bool = None
    Float32 = None
    UInt16 = None


class CarlaPlatformHeartbeatNode(Node):
    """Publish healthy raw platform state with explicit test overrides."""

    def __init__(self, **node_kwargs):
        if rclpy is None:
            raise RuntimeError(
                "ROS 2 message packages are required to run the heartbeat node"
            ) from _ROS_IMPORT_ERROR
        super().__init__("carla_platform_heartbeat", **node_kwargs)

        self.system_state_topic = self.declare_parameter(
            "system_state_topic", "/system_state").value
        self.battery_state_topic = self.declare_parameter(
            "battery_state_topic", "/battery_state").value
        self.charging_topic = self.declare_parameter(
            "charging_topic",
            "/camrod_carla/platform_heartbeat/charging",
        ).value
        self.soc_topic = self.declare_parameter(
            "soc_topic", "/camrod_carla/platform_heartbeat/soc").value
        self.estop_topic = self.declare_parameter(
            "estop_topic", "/camrod_carla/platform_heartbeat/estop").value
        self.error_code_topic = self.declare_parameter(
            "error_code_topic",
            "/camrod_carla/platform_heartbeat/error_code",
        ).value

        self.config = validate_heartbeat_config(HeartbeatConfig(
            publish_rate_hz=self.declare_parameter(
                "publish_rate_hz", 5.0).value,
            frame_id=self.declare_parameter(
                "frame_id", "robot_center_link").value,
            battery_voltage_v=self.declare_parameter(
                "battery_voltage_v", 48.0).value,
            charging_current_a=self.declare_parameter(
                "charging_current_a", 1.0).value,
            discharging_current_a=self.declare_parameter(
                "discharging_current_a", -1.0).value,
            battery_temperature_c=self.declare_parameter(
                "battery_temperature_c", 25.0).value,
            battery_location=self.declare_parameter(
                "battery_location", "carla:simulated-platform").value,
        ))
        self._inputs = sanitize_heartbeat_inputs(HeartbeatInputs(
            charging=self.declare_parameter(
                "initial_charging", False).value,
            soc=self.declare_parameter("initial_soc", 0.8).value,
            estop=self.declare_parameter("initial_estop", False).value,
            error_code=self.declare_parameter(
                "initial_error_code", 0).value,
        ))
        self._state_lock = threading.Lock()

        raw_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        override_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.system_state_publisher = self.create_publisher(
            SystemState, self.system_state_topic, raw_qos)
        self.battery_state_publisher = self.create_publisher(
            BatteryState, self.battery_state_topic, raw_qos)
        self.charging_subscription = self.create_subscription(
            Bool, self.charging_topic, self._on_charging, override_qos)
        self.soc_subscription = self.create_subscription(
            Float32, self.soc_topic, self._on_soc, override_qos)
        self.estop_subscription = self.create_subscription(
            Bool, self.estop_topic, self._on_estop, override_qos)
        self.error_code_subscription = self.create_subscription(
            UInt16, self.error_code_topic, self._on_error_code, override_qos)
        self.timer = self.create_timer(
            1.0 / self.config.publish_rate_hz, self._publish_heartbeat)

        self.get_logger().info(
            "CARLA platform heartbeat ready at %.1f Hz: %s, %s"
            % (
                self.config.publish_rate_hz,
                self.system_state_topic,
                self.battery_state_topic,
            )
        )

    def _replace_inputs(self, **changes):
        with self._state_lock:
            self._inputs = replace(self._inputs, **changes)

    def _on_charging(self, message):
        self._replace_inputs(charging=bool(message.data))

    def _on_soc(self, message):
        raw_soc = message.data
        soc = sanitize_soc(raw_soc)
        if not math.isfinite(float(raw_soc)):
            self.get_logger().error(
                "Non-finite SOC override rejected; publishing SOC 0.0")
        elif soc != float(raw_soc):
            self.get_logger().warning(
                "SOC override clipped to %.3f" % soc)
        self._replace_inputs(soc=soc)

    def _on_estop(self, message):
        self._replace_inputs(estop=bool(message.data))

    def _on_error_code(self, message):
        self._replace_inputs(error_code=sanitize_error_code(message.data))

    def _snapshot_inputs(self):
        with self._state_lock:
            return self._inputs

    def _publish_heartbeat(self):
        values = build_platform_heartbeat(
            self._snapshot_inputs(), self.config)
        stamp = self.get_clock().now().to_msg()

        system = SystemState()
        system.header.stamp = stamp
        system.header.frame_id = self.config.frame_id
        system.vehicle_state = values.system.vehicle_state
        system.control_mode = values.system.control_mode
        system.error_code = values.system.error_code
        system.battery_voltage = values.system.battery_voltage
        system.motion_mode = values.system.motion_mode

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.header.frame_id = self.config.frame_id
        battery.voltage = values.battery.voltage
        battery.temperature = values.battery.temperature
        battery.current = values.battery.current
        battery.charge = math.nan
        battery.capacity = math.nan
        battery.design_capacity = math.nan
        battery.percentage = values.battery.percentage
        battery.power_supply_status = values.battery.power_supply_status
        battery.power_supply_health = values.battery.power_supply_health
        battery.power_supply_technology = (
            values.battery.power_supply_technology)
        battery.present = values.battery.present
        battery.location = values.battery.location

        self.system_state_publisher.publish(system)
        self.battery_state_publisher.publish(battery)


def main(args=None):
    """Run the CARLA platform heartbeat node."""
    if rclpy is None:
        raise RuntimeError(
            "ROS 2 message packages are required to run the heartbeat node"
        ) from _ROS_IMPORT_ERROR
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaPlatformHeartbeatNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
