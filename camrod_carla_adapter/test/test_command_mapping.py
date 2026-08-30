import math
import random
from dataclasses import replace

import pytest

from camrod_carla_adapter.command_mapping import (
    DriveMode,
    MappingConfig,
    command_age_timed_out,
    map_planar_twist,
    recovery_breakaway_is_authorized,
    validate_adapter_timing,
    validate_planar_axes,
    validate_ranger_contract,
    validate_recovery_breakaway_contract,
)


CFG = MappingConfig()


@pytest.mark.parametrize("linear_x", [0.0, 0.8, -0.8])
def test_zero_yaw_is_ackermann(linear_x):
    command = map_planar_twist(linear_x, 0.0, 0.0, CFG)
    assert command.mode == DriveMode.ACKERMANN
    assert command.speed == pytest.approx(linear_x)
    assert command.steering_angle == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("linear_x", "angular_z", "expected_sign"),
    [
        (1.0, 0.5, 1.0),
        (1.0, -0.5, -1.0),
        (-1.0, 0.5, -1.0),
        (-1.0, -0.5, 1.0),
    ],
)
def test_ackermann_sign_matches_vx_times_wz(
        linear_x, angular_z, expected_sign):
    command = map_planar_twist(linear_x, 0.0, angular_z, CFG)
    assert command.mode == DriveMode.ACKERMANN
    assert math.copysign(1.0, command.steering_angle) == expected_sign


def test_angular_epsilon_is_strict():
    below = math.nextafter(CFG.angular_epsilon_radps, 0.0)
    command_below = map_planar_twist(0.0, 0.0, below, CFG)
    command_equal = map_planar_twist(
        CFG.minimum_turn_radius_m * CFG.angular_epsilon_radps,
        0.0,
        CFG.angular_epsilon_radps,
        CFG,
    )
    assert command_below.mode == DriveMode.ACKERMANN
    assert command_equal.mode == DriveMode.ACKERMANN


def test_turn_radius_equality_is_ackermann_and_below_is_zero_turn():
    equality = map_planar_twist(
        CFG.minimum_turn_radius_m, 0.0, 1.0, CFG)
    below_radius = math.nextafter(CFG.minimum_turn_radius_m, 0.0)
    below = map_planar_twist(below_radius, 0.0, 1.0, CFG)
    assert equality.mode == DriveMode.ACKERMANN
    assert below.mode == DriveMode.ZERO_TURN


@pytest.mark.parametrize("lateral", [0.02, -0.02])
def test_lateral_deadband_equality_is_not_crab(lateral):
    command = map_planar_twist(0.5, lateral, 0.0, CFG)
    assert command.mode == DriveMode.ACKERMANN


@pytest.mark.parametrize("lateral", [0.0200001, -0.0200001])
def test_lateral_outside_deadband_is_crab(lateral):
    command = map_planar_twist(0.5, lateral, 0.0, CFG)
    assert command.mode == DriveMode.CRAB


def test_crab_has_priority_over_zero_turn():
    command = map_planar_twist(0.0, 0.5, 0.7, CFG)
    assert command.mode == DriveMode.CRAB
    assert command.source_angular_z == pytest.approx(0.7)
    assert command.yaw_rate_cmd == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("linear_x", "linear_y", "speed", "angle_deg"),
    [
        (1.0, 1.0, math.sqrt(2.0), 45.0),
        (-1.0, 1.0, -math.sqrt(2.0), -45.0),
        (-1.0, -1.0, -math.sqrt(2.0), 45.0),
        (1.0, -1.0, math.sqrt(2.0), -45.0),
    ],
)
def test_crab_quadrants_preserve_requested_velocity(
        linear_x, linear_y, speed, angle_deg):
    unlimited = MappingConfig(max_crab_speed_mps=3.0)
    command = map_planar_twist(linear_x, linear_y, 0.0, unlimited)
    assert command.mode == DriveMode.CRAB
    assert command.speed == pytest.approx(speed)
    assert command.crab_angle == pytest.approx(math.radians(angle_deg))
    assert command.speed * math.cos(command.crab_angle) == pytest.approx(
        linear_x)
    assert command.speed * math.sin(command.crab_angle) == pytest.approx(
        linear_y)


@pytest.mark.parametrize("linear_y", [1.0, -1.0])
def test_pure_crab_is_explicitly_clipped_to_accepted_limit(linear_y):
    command = map_planar_twist(0.0, linear_y, 0.0, CFG)
    assert command.mode == DriveMode.CRAB
    assert abs(command.crab_angle) == pytest.approx(math.radians(88.0))
    assert command.crab_angle_limited is True
    assert command.saturated is True


@pytest.mark.parametrize("angular_z", [2.0, -2.0])
def test_zero_turn_yaw_rate_is_limited(angular_z):
    command = map_planar_twist(0.0, 0.0, angular_z, CFG)
    assert command.mode == DriveMode.ZERO_TURN
    assert command.speed == 0.0
    assert command.yaw_rate_cmd == pytest.approx(
        math.copysign(CFG.max_yaw_rate_radps, angular_z))
    assert command.saturated is True


@pytest.mark.parametrize(
    ("linear_x", "linear_y", "angular_z"),
    [
        (math.nan, 0.0, 0.0),
        (0.0, math.inf, 0.0),
        (0.0, 0.0, -math.inf),
    ],
)
def test_nonfinite_planar_input_is_rejected(linear_x, linear_y, angular_z):
    with pytest.raises(ValueError):
        map_planar_twist(linear_x, linear_y, angular_z, CFG)


def test_nonplanar_axes_fail_closed_contract():
    validate_planar_axes(1.0e-6, -1.0e-6, 0.0, tolerance=1.0e-6)
    with pytest.raises(ValueError):
        validate_planar_axes(1.0001e-6, 0.0, 0.0, tolerance=1.0e-6)
    with pytest.raises(ValueError):
        validate_planar_axes(0.0, math.nan, 0.0)


def test_mapping_never_generates_pivot():
    random_source = random.Random(260823)
    for _ in range(4000):
        command = map_planar_twist(
            random_source.uniform(-2.0, 2.0),
            random_source.uniform(-2.0, 2.0),
            random_source.uniform(-2.0, 2.0),
            CFG,
        )
        assert command.mode in (
            DriveMode.ACKERMANN,
            DriveMode.CRAB,
            DriveMode.ZERO_TURN,
        )
        assert command.mode != DriveMode.PIVOT
        assert all(math.isfinite(value) for value in (
            command.speed,
            command.steering_angle,
            command.crab_angle,
            command.rear_steering_angle,
            command.yaw_rate_cmd,
        ))


def test_watchdog_timeout_boundary_is_strict_and_bad_time_fails_closed():
    assert command_age_timed_out(10.0, 10.35, 0.35) is False
    assert command_age_timed_out(
        10.0, math.nextafter(10.35, math.inf), 0.35) is True
    assert command_age_timed_out(10.0, 9.9, 0.35) is True
    assert command_age_timed_out(10.0, math.nan, 0.35) is True


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("lateral_deadband_mps", CFG.lateral_deadband_mps + 1.0e-9),
        ("angular_epsilon_radps", CFG.angular_epsilon_radps + 1.0e-9),
        ("minimum_turn_radius_m", CFG.minimum_turn_radius_m + 1.0e-9),
        ("wheelbase_m", CFG.wheelbase_m + 1.0e-9),
        (
            "max_ackermann_steer_rad",
            math.nextafter(CFG.max_ackermann_steer_rad, math.inf),
        ),
        (
            "max_crab_angle_rad",
            math.nextafter(CFG.max_crab_angle_rad, math.inf),
        ),
        (
            "max_ackermann_speed_mps",
            math.nextafter(CFG.max_ackermann_speed_mps, math.inf),
        ),
        (
            "max_crab_speed_mps",
            math.nextafter(CFG.max_crab_speed_mps, math.inf),
        ),
        (
            "max_yaw_rate_radps",
            math.nextafter(CFG.max_yaw_rate_radps, math.inf),
        ),
    ],
)
def test_node_contract_rejects_semantic_changes_and_wider_limits(field, value):
    with pytest.raises(ValueError):
        validate_ranger_contract(replace(CFG, **{field: value}))


def test_node_contract_allows_tighter_actuation_limits():
    validate_ranger_contract(replace(
        CFG,
        max_ackermann_steer_rad=0.5,
        max_crab_angle_rad=1.2,
        max_ackermann_speed_mps=0.8,
        max_crab_speed_mps=0.6,
        max_yaw_rate_radps=0.5,
    ))


@pytest.mark.parametrize(
    "settings",
    [
        (math.nextafter(0.35, math.inf), 50.0, 10.0, 1.0e-6),
        (0.35, math.nextafter(50.0, 0.0), 10.0, 1.0e-6),
        (0.35, 50.0, math.nextafter(10.0, 0.0), 1.0e-6),
        (0.35, 50.0, 10.0, math.nextafter(1.0e-6, math.inf)),
    ],
)
def test_adapter_timing_rejects_weaker_fail_closed_overrides(settings):
    with pytest.raises(ValueError):
        validate_adapter_timing(*settings)


def test_adapter_timing_allows_stricter_settings():
    validate_adapter_timing(0.1, 100.0, 20.0, 0.0)


@pytest.mark.parametrize("state", ["CRAB_LEFT", "CRAB_RIGHT"])
def test_fresh_recovery_crab_authorizes_only_narrow_breakaway_capability(
        state):
    lateral = 0.05 if state == "CRAB_LEFT" else -0.05
    command = map_planar_twist(0.0, lateral, 0.0, CFG)
    assert recovery_breakaway_is_authorized(
        command, state, 10.0, 10.29) is True


def test_recovery_breakaway_authority_fails_closed_for_wrong_source_or_age():
    crab = map_planar_twist(0.0, 0.05, 0.0, CFG)
    ackermann = map_planar_twist(0.05, 0.0, 0.0, CFG)
    too_slow = map_planar_twist(0.0, 0.039, 0.0, CFG)
    too_fast = map_planar_twist(0.0, 0.061, 0.0, CFG)
    assert recovery_breakaway_is_authorized(
        ackermann, "CRAB_LEFT", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        too_slow, "CRAB_LEFT", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        too_fast, "CRAB_LEFT", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        crab, "REVERSE", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        crab, "CRAB_LEFT", 10.0, math.nextafter(10.30, math.inf)) is False
    assert recovery_breakaway_is_authorized(
        crab, "CRAB_LEFT", None, 10.1) is False
    assert recovery_breakaway_is_authorized(
        crab, "CRAB_LEFT", 10.0, math.nan) is False


def test_recovery_breakaway_status_must_match_pure_crab_direction():
    left = map_planar_twist(0.0, 0.05, 0.0, CFG)
    right = map_planar_twist(0.0, -0.05, 0.0, CFG)
    assert recovery_breakaway_is_authorized(
        left, "CRAB_RIGHT", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        right, "CRAB_LEFT", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        replace(left, source_linear_y=0.0),
        "CRAB_LEFT", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        replace(left, crab_angle=0.0),
        "CRAB_LEFT", 10.0, 10.1) is False
    assert recovery_breakaway_is_authorized(
        replace(left, source_linear_x=1.1e-6),
        "CRAB_LEFT", 10.0, 10.1) is False


@pytest.mark.parametrize(
    "settings",
    [
        (math.nextafter(0.30, math.inf), 0.04, 0.06),
        (0.30, math.nextafter(0.04, 0.0), 0.06),
        (0.30, 0.04, math.nextafter(0.06, math.inf)),
        (0.30, 0.055, 0.05),
    ],
)
def test_recovery_breakaway_contract_rejects_wider_or_empty_envelope(
        settings):
    with pytest.raises(ValueError):
        validate_recovery_breakaway_contract(*settings)


def test_recovery_breakaway_contract_allows_stricter_lease_and_window():
    validate_recovery_breakaway_contract(0.10, 0.045, 0.055)
