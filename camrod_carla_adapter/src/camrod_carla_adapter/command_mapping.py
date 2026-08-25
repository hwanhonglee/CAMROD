"""Pure CAMROD ``Twist`` to CARLA four-wheel-command mapping.

The classifier intentionally mirrors the production Ranger driver's order:
lateral motion has priority, then tight-radius spinning, then Dual-Ackermann.
No ROS or CARLA Python runtime is imported here so the contract is easy to
test offline.
"""

from dataclasses import dataclass, replace
import math


# Audited CAMROD Ranger / accepted CARLA physical-backend contract.  YAML or
# launch overrides may tighten limits but must not silently widen this envelope.
RANGER_LATERAL_DEADBAND_MPS = 0.02
RANGER_ANGULAR_EPSILON_RADPS = 1.0e-6
RANGER_MINIMUM_TURN_RADIUS_M = 0.810330349
RANGER_COMMAND_WHEELBASE_M = 0.90
RANGER_MAX_ACKERMANN_STEER_RAD = 0.6981
RANGER_MAX_CRAB_ANGLE_RAD = math.radians(88.0)
RANGER_MAX_ACKERMANN_SPEED_MPS = 1.4
RANGER_MAX_CRAB_SPEED_MPS = 1.0
RANGER_MAX_YAW_RATE_RADPS = 0.7853
RANGER_MAX_INPUT_TIMEOUT_SEC = 0.35
RANGER_MIN_WATCHDOG_RATE_HZ = 50.0
RANGER_MIN_ZERO_PUBLISH_RATE_HZ = 10.0
RANGER_MAX_UNSUPPORTED_AXIS_TOLERANCE = 1.0e-6
_CONTRACT_ABS_TOLERANCE = 1.0e-12


class DriveMode:
    """Values from ``carla_extended_ackermann_msgs/msg/DriveMode``."""

    ACKERMANN = 0
    CRAB = 1
    ZERO_TURN = 2
    PIVOT = 3


@dataclass(frozen=True)
class MappingConfig:
    """CAMROD Ranger classification and CARLA safety limits."""

    lateral_deadband_mps: float = RANGER_LATERAL_DEADBAND_MPS
    angular_epsilon_radps: float = RANGER_ANGULAR_EPSILON_RADPS
    minimum_turn_radius_m: float = RANGER_MINIMUM_TURN_RADIUS_M
    wheelbase_m: float = RANGER_COMMAND_WHEELBASE_M
    max_ackermann_steer_rad: float = RANGER_MAX_ACKERMANN_STEER_RAD
    max_crab_angle_rad: float = RANGER_MAX_CRAB_ANGLE_RAD
    max_ackermann_speed_mps: float = RANGER_MAX_ACKERMANN_SPEED_MPS
    max_crab_speed_mps: float = RANGER_MAX_CRAB_SPEED_MPS
    max_yaw_rate_radps: float = RANGER_MAX_YAW_RATE_RADPS

    def __post_init__(self):
        values = (
            self.lateral_deadband_mps,
            self.angular_epsilon_radps,
            self.minimum_turn_radius_m,
            self.wheelbase_m,
            self.max_ackermann_steer_rad,
            self.max_crab_angle_rad,
            self.max_ackermann_speed_mps,
            self.max_crab_speed_mps,
            self.max_yaw_rate_radps,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("mapping configuration must be finite")
        if self.lateral_deadband_mps < 0.0:
            raise ValueError("lateral deadband must be >= 0")
        if self.angular_epsilon_radps <= 0.0:
            raise ValueError("angular epsilon must be > 0")
        if self.minimum_turn_radius_m <= 0.0:
            raise ValueError("minimum turn radius must be > 0")
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase must be > 0")
        if not 0.0 < self.max_ackermann_steer_rad < math.pi / 2.0:
            raise ValueError("Ackermann steering limit must be in (0, pi/2)")
        if not 0.0 < self.max_crab_angle_rad < math.pi / 2.0:
            raise ValueError("crab angle limit must be in (0, pi/2)")
        if min(
            self.max_ackermann_speed_mps,
            self.max_crab_speed_mps,
            self.max_yaw_rate_radps,
        ) <= 0.0:
            raise ValueError("speed and yaw-rate limits must be > 0")


def validate_ranger_contract(config):
    """Reject overrides that change CAMROD semantics or widen accepted limits."""

    if not isinstance(config, MappingConfig):
        raise TypeError("config must be a MappingConfig")

    fixed_values = (
        ("lateral_deadband_mps", RANGER_LATERAL_DEADBAND_MPS),
        ("angular_epsilon_radps", RANGER_ANGULAR_EPSILON_RADPS),
        ("minimum_turn_radius_m", RANGER_MINIMUM_TURN_RADIUS_M),
        ("wheelbase_m", RANGER_COMMAND_WHEELBASE_M),
    )
    for name, accepted in fixed_values:
        configured = float(getattr(config, name))
        if not math.isclose(
            configured,
            accepted,
            rel_tol=0.0,
            abs_tol=_CONTRACT_ABS_TOLERANCE,
        ):
            raise ValueError(
                "%s must remain at the audited Ranger value %.12g"
                % (name, accepted)
            )

    upper_limits = (
        ("max_ackermann_steer_rad", RANGER_MAX_ACKERMANN_STEER_RAD),
        ("max_crab_angle_rad", RANGER_MAX_CRAB_ANGLE_RAD),
        ("max_ackermann_speed_mps", RANGER_MAX_ACKERMANN_SPEED_MPS),
        ("max_crab_speed_mps", RANGER_MAX_CRAB_SPEED_MPS),
        ("max_yaw_rate_radps", RANGER_MAX_YAW_RATE_RADPS),
    )
    for name, accepted_maximum in upper_limits:
        configured = float(getattr(config, name))
        if configured > accepted_maximum:
            raise ValueError(
                "%s exceeds accepted Ranger maximum %.12g"
                % (name, accepted_maximum)
            )


def validate_adapter_timing(
    input_timeout_sec,
    watchdog_rate_hz,
    zero_publish_rate_hz,
    unsupported_axis_tolerance,
):
    """Validate watchdog settings against the audited fail-closed envelope."""

    values = tuple(float(value) for value in (
        input_timeout_sec,
        watchdog_rate_hz,
        zero_publish_rate_hz,
        unsupported_axis_tolerance,
    ))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("adapter timing parameters must be finite")

    timeout, watchdog_rate, zero_rate, axis_tolerance = values
    if not 0.0 < timeout <= RANGER_MAX_INPUT_TIMEOUT_SEC:
        raise ValueError(
            "input_timeout_sec must be in (0, %.3f]"
            % RANGER_MAX_INPUT_TIMEOUT_SEC
        )
    if watchdog_rate < RANGER_MIN_WATCHDOG_RATE_HZ:
        raise ValueError(
            "watchdog_rate_hz must be >= %.1f"
            % RANGER_MIN_WATCHDOG_RATE_HZ
        )
    if zero_rate < RANGER_MIN_ZERO_PUBLISH_RATE_HZ:
        raise ValueError(
            "zero_publish_rate_hz must be >= %.1f"
            % RANGER_MIN_ZERO_PUBLISH_RATE_HZ
        )
    if not 0.0 <= axis_tolerance <= RANGER_MAX_UNSUPPORTED_AXIS_TOLERANCE:
        raise ValueError(
            "unsupported_axis_tolerance must be in [0, %.12g]"
            % RANGER_MAX_UNSUPPORTED_AXIS_TOLERANCE
        )


@dataclass(frozen=True)
class MappedCommand:
    """ROS-independent representation of ``ExtendedAckermannDrive``."""

    mode: int
    speed: float = 0.0
    steering_angle: float = 0.0
    crab_angle: float = 0.0
    rear_steering_angle: float = 0.0
    yaw_rate_cmd: float = 0.0
    source_linear_x: float = 0.0
    source_linear_y: float = 0.0
    source_angular_z: float = 0.0
    saturated: bool = False
    crab_angle_limited: bool = False
    reason: str = ""


def stop_command(reason="stop"):
    """Return the canonical fail-closed command."""

    return MappedCommand(mode=DriveMode.ACKERMANN, reason=str(reason))


def _clip(value, maximum):
    return max(-float(maximum), min(float(maximum), float(value)))


def _changed(before, after):
    return abs(float(before) - float(after)) > 1.0e-12


def _parallel_command(linear_x, linear_y, config):
    speed = math.hypot(linear_x, linear_y)
    steering = math.atan2(linear_y, linear_x)

    # Ranger parallel steering is limited to +/-90 degrees. Rear-quadrant
    # vectors reverse wheel speed and rotate the wheel axis into that interval.
    if steering > math.pi / 2.0:
        steering -= math.pi
        speed = -speed
    elif steering < -math.pi / 2.0:
        steering += math.pi
        speed = -speed

    limited_steering = _clip(steering, config.max_crab_angle_rad)
    limited_speed = _clip(speed, config.max_crab_speed_mps)
    angle_limited = _changed(steering, limited_steering)
    return MappedCommand(
        mode=DriveMode.CRAB,
        speed=limited_speed,
        crab_angle=limited_steering,
        source_linear_x=linear_x,
        source_linear_y=linear_y,
        saturated=(angle_limited or _changed(speed, limited_speed)),
        crab_angle_limited=angle_limited,
        reason="camrod_parallel",
    )


def map_planar_twist(linear_x, linear_y, angular_z, config=None):
    """Map one body-frame planar Twist using CAMROD Ranger semantics.

    ``linear_x``/``linear_y`` are metres per second in ROS body coordinates;
    ``angular_z`` is radians per second with counter-clockwise positive. The
    result keeps ROS signs. The existing CARLA controller owns the single
    ROS-left-positive to CARLA-right-positive conversion.
    """

    cfg = config if config is not None else MappingConfig()
    linear_x = float(linear_x)
    linear_y = float(linear_y)
    angular_z = float(angular_z)
    if not all(math.isfinite(value) for value in (
            linear_x, linear_y, angular_z)):
        raise ValueError("planar Twist values must be finite")

    # This strict comparison and its priority match ShouldUseParallelMotion.
    if abs(linear_y) > cfg.lateral_deadband_mps:
        command = _parallel_command(linear_x, linear_y, cfg)
        return replace(command, source_angular_z=angular_z)

    if abs(angular_z) < cfg.angular_epsilon_radps:
        radius = math.inf
    else:
        radius = abs(linear_x) / abs(angular_z)

    # Equality deliberately remains Dual-Ackermann, matching the C++ driver.
    if radius < cfg.minimum_turn_radius_m:
        limited_yaw_rate = _clip(angular_z, cfg.max_yaw_rate_radps)
        return MappedCommand(
            mode=DriveMode.ZERO_TURN,
            yaw_rate_cmd=limited_yaw_rate,
            source_linear_x=linear_x,
            source_linear_y=linear_y,
            source_angular_z=angular_z,
            saturated=_changed(angular_z, limited_yaw_rate),
            reason="camrod_spinning",
        )

    steering = 0.0
    if math.isfinite(radius):
        steering = math.atan((cfg.wheelbase_m / 2.0) / radius)
        if linear_x * angular_z < 0.0:
            steering = -steering
    limited_steering = _clip(steering, cfg.max_ackermann_steer_rad)
    limited_speed = _clip(linear_x, cfg.max_ackermann_speed_mps)
    return MappedCommand(
        mode=DriveMode.ACKERMANN,
        speed=limited_speed,
        steering_angle=limited_steering,
        source_linear_x=linear_x,
        source_linear_y=linear_y,
        source_angular_z=angular_z,
        saturated=(
            _changed(steering, limited_steering)
            or _changed(linear_x, limited_speed)
        ),
        reason="camrod_dual_ackermann",
    )


def validate_planar_axes(linear_z, angular_x, angular_y, tolerance=1.0e-6):
    """Reject non-finite or unsupported non-planar Twist components."""

    values = tuple(float(value) for value in (linear_z, angular_x, angular_y))
    tolerance = float(tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("unsupported-axis tolerance must be finite and >= 0")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-planar Twist values must be finite")
    if any(abs(value) > tolerance for value in values):
        raise ValueError("non-planar Twist components are unsupported")


def command_age_timed_out(last_receive_monotonic, now_monotonic, timeout_sec):
    """Return whether a received command is invalid-aged or strictly stale."""

    last_receive_monotonic = float(last_receive_monotonic)
    now_monotonic = float(now_monotonic)
    timeout_sec = float(timeout_sec)
    if not all(math.isfinite(value) for value in (
            last_receive_monotonic, now_monotonic, timeout_sec)):
        return True
    if timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be > 0")
    age = now_monotonic - last_receive_monotonic
    return age < 0.0 or age > timeout_sec
