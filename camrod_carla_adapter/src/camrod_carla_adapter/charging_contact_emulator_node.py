"""CARLA charger-contact emulator backed by measured pose and velocity.

The node does not synthesize any perception stream.  It closes only the
existing simulated platform charging input after the real CARLA vehicle has
reached the configured Drop Zone station, stopped, and remained there for a
bounded dwell.  Ordinary CAMROD and physical CAN charging remain unchanged.
"""

from dataclasses import dataclass
import math
from pathlib import Path
import threading

import yaml


@dataclass(frozen=True)
class ChargingStation:
    """Canonical planar contact point loaded from the CAMROD map."""

    x_m: float
    y_m: float


@dataclass(frozen=True)
class ContactConfig:
    """Fail-closed spatial, motion, freshness, and dwell thresholds."""

    position_tolerance_m: float = 0.35
    speed_tolerance_mps: float = 0.05
    pose_timeout_s: float = 0.5
    odometry_timeout_s: float = 0.5
    state_timeout_s: float = 2.0
    dwell_s: float = 1.0


@dataclass(frozen=True)
class ContactSample:
    """One ROS-independent contact decision sample."""

    parking_state: str
    x_m: float
    y_m: float
    speed_mps: float
    pose_age_s: float
    odometry_age_s: float
    # HH_260831 - These fields are used only by the CARLA-only restart
    # recovery path.  Terminal parking states retain their existing contract.
    planning_state: str = ""
    parking_status_age_s: float = math.inf
    planning_state_age_s: float = math.inf


def _finite_positive(value, name):
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} must be finite and positive") from error
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
    return numeric


def validate_contact_config(config):
    """Normalize a contact profile and reject permissive/malformed values."""
    return ContactConfig(
        position_tolerance_m=_finite_positive(
            config.position_tolerance_m, "position_tolerance_m"),
        speed_tolerance_mps=_finite_positive(
            config.speed_tolerance_mps, "speed_tolerance_mps"),
        pose_timeout_s=_finite_positive(
            config.pose_timeout_s, "pose_timeout_s"),
        odometry_timeout_s=_finite_positive(
            config.odometry_timeout_s, "odometry_timeout_s"),
        state_timeout_s=_finite_positive(
            config.state_timeout_s, "state_timeout_s"),
        dwell_s=_finite_positive(config.dwell_s, "dwell_s"),
    )


def load_charging_station(path, drop_zone_id="drop_zone"):
    """Load exactly one matching Drop Zone point from a CAMROD YAML file."""
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ValueError(f"drop_zones_yaml is not a regular file: {config_path}")
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    entries = payload.get("drop_zones", [])
    key = str(drop_zone_id).strip()
    matches = [
        entry for entry in entries
        if str(entry.get("id", "")).strip() == key
        or str(entry.get("type", "")).strip() == key
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one Drop Zone matching {key!r}, found {len(matches)}"
        )
    try:
        x_m = float(matches[0]["x"])
        y_m = float(matches[0]["y"])
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("Drop Zone x/y must be finite numbers") from error
    if not math.isfinite(x_m) or not math.isfinite(y_m):
        raise ValueError("Drop Zone x/y must be finite numbers")
    return ChargingStation(x_m=x_m, y_m=y_m)


def contact_candidate(sample, station, config=ContactConfig()):
    """Return true only for a fresh, stopped vehicle at a charging wait."""
    checked = validate_contact_config(config)
    parking_state = str(sample.parking_state).strip().upper()
    planning_state = str(sample.planning_state).strip().upper()
    terminal_parking_evidence = parking_state in {
        "WAIT_FOR_CHARGING", "PARKED"
    }

    # HH_260831 - A CAMROD-only restart cannot restore the in-memory PARKED
    # phase of reverse_parking_controller, even though the CARLA actor remains
    # physically docked.  Recover only from two fresh, independent idle-state
    # heartbeats.  Any mission-active planning state, non-IDLE parking phase,
    # missing heartbeat, stale heartbeat, or malformed age keeps contact
    # deasserted.
    restart_idle_evidence = False
    if parking_state == "IDLE" and planning_state == "WAIT_DZ":
        try:
            parking_age_s = float(sample.parking_status_age_s)
            planning_age_s = float(sample.planning_state_age_s)
        except (TypeError, ValueError, OverflowError):
            parking_age_s = math.inf
            planning_age_s = math.inf
        restart_idle_evidence = (
            math.isfinite(parking_age_s)
            and 0.0 <= parking_age_s <= checked.state_timeout_s
            and math.isfinite(planning_age_s)
            and 0.0 <= planning_age_s <= checked.state_timeout_s
        )

    if not terminal_parking_evidence and not restart_idle_evidence:
        return False
    numeric = (
        sample.x_m,
        sample.y_m,
        sample.speed_mps,
        sample.pose_age_s,
        sample.odometry_age_s,
        station.x_m,
        station.y_m,
    )
    try:
        values = tuple(float(value) for value in numeric)
    except (TypeError, ValueError, OverflowError):
        return False
    if not all(math.isfinite(value) for value in values):
        return False
    x_m, y_m, speed_mps, pose_age_s, odometry_age_s, sx_m, sy_m = values
    if pose_age_s < 0.0 or pose_age_s > checked.pose_timeout_s:
        return False
    if odometry_age_s < 0.0 or odometry_age_s > checked.odometry_timeout_s:
        return False
    if abs(speed_mps) > checked.speed_tolerance_mps:
        return False
    return math.hypot(x_m - sx_m, y_m - sy_m) <= checked.position_tolerance_m


_ROS_IMPORT_ERROR = None
try:
    import rclpy
    from avg_msgs.msg import AvgPoseStamped, ModuleState, PlanningState
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import Bool
except ImportError as error:  # Pure geometry helpers remain unit-testable.
    _ROS_IMPORT_ERROR = error
    rclpy = None
    AvgPoseStamped = None
    ModuleState = None
    PlanningState = None
    Odometry = None
    ExternalShutdownException = Exception
    Node = object
    QoSProfile = None
    QoSReliabilityPolicy = None
    Bool = None


class CarlaChargingContactEmulatorNode(Node):
    """Publish CARLA charging contact only after physical contact evidence."""

    def __init__(self, **node_kwargs):
        if rclpy is None:
            raise RuntimeError("ROS 2 is required to run charging contact") from (
                _ROS_IMPORT_ERROR)
        super().__init__("carla_charging_contact_emulator", **node_kwargs)

        self.charging_topic = self.declare_parameter(
            "charging_topic",
            "/camrod_carla/platform_heartbeat/charging",
        ).value
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/localization/pose").value
        self.odometry_topic = self.declare_parameter(
            "odometry_topic", "/odom").value
        self.parking_status_topic = self.declare_parameter(
            "parking_status_topic",
            "/parking/reverse_parking_controller/status",
        ).value
        self.planning_state_topic = self.declare_parameter(
            "planning_state_topic",
            "/planning/state_machine/state",
        ).value
        drop_zones_yaml = self.declare_parameter(
            "drop_zones_yaml", "").value
        drop_zone_id = self.declare_parameter(
            "drop_zone_id", "drop_zone").value
        self.station = load_charging_station(drop_zones_yaml, drop_zone_id)
        self.config = validate_contact_config(ContactConfig(
            position_tolerance_m=self.declare_parameter(
                "position_tolerance_m", 0.35).value,
            speed_tolerance_mps=self.declare_parameter(
                "speed_tolerance_mps", 0.05).value,
            pose_timeout_s=self.declare_parameter(
                "pose_timeout_s", 0.5).value,
            odometry_timeout_s=self.declare_parameter(
                "odometry_timeout_s", 0.5).value,
            state_timeout_s=self.declare_parameter(
                "state_timeout_s", 2.0).value,
            dwell_s=self.declare_parameter("dwell_s", 1.0).value,
        ))
        publish_rate_hz = _finite_positive(
            self.declare_parameter("publish_rate_hz", 10.0).value,
            "publish_rate_hz",
        )

        self._lock = threading.Lock()
        self._parking_state = "IDLE"
        self._parking_status_received_s = None
        self._planning_state = ""
        self._planning_state_received_s = None
        self._pose = None
        self._pose_received_s = None
        self._speed_mps = None
        self._odometry_received_s = None
        self._candidate_since_s = None
        self._contact = False

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Bool, self.charging_topic, qos)
        self.pose_subscription = self.create_subscription(
            AvgPoseStamped, self.pose_topic, self._on_pose, qos)
        self.odometry_subscription = self.create_subscription(
            Odometry, self.odometry_topic, self._on_odometry, qos)
        self.parking_status_subscription = self.create_subscription(
            ModuleState, self.parking_status_topic, self._on_parking_status, qos)
        self.planning_state_subscription = self.create_subscription(
            PlanningState, self.planning_state_topic, self._on_planning_state, qos)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "CARLA charging contact ready: station=(%.3f,%.3f)m "
            "radius=%.2fm speed<=%.2fm/s dwell=%.1fs state_timeout=%.1fs"
            % (
                self.station.x_m,
                self.station.y_m,
                self.config.position_tolerance_m,
                self.config.speed_tolerance_mps,
                self.config.dwell_s,
                self.config.state_timeout_s,
            )
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_pose(self, message):
        with self._lock:
            self._pose = (
                float(message.pose.position.x),
                float(message.pose.position.y),
            )
            self._pose_received_s = self._now_s()

    def _on_odometry(self, message):
        linear = message.twist.twist.linear
        with self._lock:
            self._speed_mps = math.sqrt(
                float(linear.x) ** 2
                + float(linear.y) ** 2
                + float(linear.z) ** 2
            )
            self._odometry_received_s = self._now_s()

    def _on_parking_status(self, message):
        with self._lock:
            self._parking_state = str(message.operating_state).strip().upper()
            self._parking_status_received_s = self._now_s()

    def _on_planning_state(self, message):
        label = str(message.label).strip().upper()
        # Both the typed enum and its human-readable label must agree.  A
        # malformed/mixed publisher therefore cannot arm restart recovery.
        is_wait_dz = (
            int(message.state) == int(PlanningState.WAIT_DZ)
            and label == "WAIT_DZ"
        )
        with self._lock:
            self._planning_state = (
                "WAIT_DZ" if is_wait_dz else "ACTIVE_OR_UNKNOWN"
            )
            self._planning_state_received_s = self._now_s()

    def _snapshot(self, now_s):
        with self._lock:
            if (
                self._pose is None
                or self._speed_mps is None
                or self._pose_received_s is None
                or self._odometry_received_s is None
            ):
                return None
            return ContactSample(
                parking_state=self._parking_state,
                x_m=self._pose[0],
                y_m=self._pose[1],
                speed_mps=self._speed_mps,
                pose_age_s=now_s - self._pose_received_s,
                odometry_age_s=now_s - self._odometry_received_s,
                planning_state=self._planning_state,
                parking_status_age_s=(
                    math.inf
                    if self._parking_status_received_s is None
                    else now_s - self._parking_status_received_s
                ),
                planning_state_age_s=(
                    math.inf
                    if self._planning_state_received_s is None
                    else now_s - self._planning_state_received_s
                ),
            )

    def _on_timer(self):
        now_s = self._now_s()
        sample = self._snapshot(now_s)
        candidate = sample is not None and contact_candidate(
            sample, self.station, self.config)

        with self._lock:
            previous = self._contact
            if candidate:
                if self._candidate_since_s is None:
                    self._candidate_since_s = now_s
                self._contact = (
                    now_s - self._candidate_since_s >= self.config.dwell_s)
            else:
                self._candidate_since_s = None
                self._contact = False
            current = self._contact

        message = Bool()
        message.data = current
        self.publisher.publish(message)
        if current != previous:
            if current:
                distance = math.hypot(
                    sample.x_m - self.station.x_m,
                    sample.y_m - self.station.y_m,
                )
                self.get_logger().info(
                    "CARLA charging contact asserted: distance=%.3fm "
                    "speed=%.3fm/s parking=%s planning=%s evidence=%s"
                    % (
                        distance,
                        sample.speed_mps,
                        sample.parking_state,
                        sample.planning_state or "unobserved",
                        "restart_idle"
                        if sample.parking_state == "IDLE"
                        else "parking_terminal",
                    )
                )
            else:
                self.get_logger().info(
                    "CARLA charging contact released: parking/pose/motion "
                    "evidence no longer valid"
                )


def main(args=None):
    """Run the CARLA charging-contact emulator."""
    if rclpy is None:
        raise RuntimeError("ROS 2 is required to run charging contact") from (
            _ROS_IMPORT_ERROR)
    rclpy.init(args=args)
    node = None
    try:
        node = CarlaChargingContactEmulatorNode()
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
