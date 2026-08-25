"""Pure-Python tests for the simulated Ranger heartbeat mapping."""

import math
from dataclasses import replace

import pytest

from camrod_carla_adapter.carla_platform_heartbeat_node import (
    ControlMode,
    HeartbeatConfig,
    HeartbeatInputs,
    MotionMode,
    PowerSupplyHealth,
    PowerSupplyStatus,
    PowerSupplyTechnology,
    UINT16_MAX,
    VehicleState,
    build_platform_heartbeat,
    sanitize_error_code,
    sanitize_flag,
    sanitize_soc,
    validate_heartbeat_config,
)


def test_default_heartbeat_is_healthy_can_dual_ackermann():
    values = build_platform_heartbeat(HeartbeatInputs())

    assert values.system.vehicle_state == VehicleState.NORMAL
    assert values.system.control_mode == ControlMode.CAN
    assert values.system.error_code == 0
    assert values.system.motion_mode == MotionMode.DUAL_ACKERMANN
    assert values.system.battery_voltage == pytest.approx(48.0)

    assert values.battery.voltage == pytest.approx(48.0)
    assert values.battery.percentage == pytest.approx(0.8)
    assert values.battery.current < 0.0
    assert values.battery.power_supply_status == (
        PowerSupplyStatus.DISCHARGING)
    assert values.battery.power_supply_health == PowerSupplyHealth.GOOD
    assert values.battery.power_supply_technology == (
        PowerSupplyTechnology.LION)
    assert values.battery.present is True


def test_charging_override_sets_explicit_status_and_positive_current():
    values = build_platform_heartbeat(HeartbeatInputs(charging=True))

    assert values.battery.power_supply_status == PowerSupplyStatus.CHARGING
    assert values.battery.current > 0.0


def test_estop_and_error_overrides_are_independent():
    estop = build_platform_heartbeat(HeartbeatInputs(estop=True))
    fault = build_platform_heartbeat(HeartbeatInputs(error_code=0x1234))

    assert estop.system.vehicle_state == VehicleState.ESTOP
    assert estop.system.error_code == 0
    assert fault.system.vehicle_state == VehicleState.NORMAL
    assert fault.system.error_code == 0x1234


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (-0.1, 0.0),
        (0.0, 0.0),
        (0.42, 0.42),
        (1.0, 1.0),
        (1.1, 1.0),
        (math.nan, 0.0),
        (math.inf, 0.0),
        ("invalid", 0.0),
    ],
)
def test_soc_is_clipped_and_nonfinite_input_fails_empty(value, expected):
    assert sanitize_soc(value) == pytest.approx(expected)
    values = build_platform_heartbeat(HeartbeatInputs(soc=value))
    assert values.battery.percentage == pytest.approx(expected)


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (-1, 0),
        (0, 0),
        (0x2345, 0x2345),
        (UINT16_MAX + 1, UINT16_MAX),
        (math.nan, UINT16_MAX),
        (-math.inf, UINT16_MAX),
        ("invalid", UINT16_MAX),
    ],
)
def test_error_code_is_clipped_and_malformed_input_fails_faulted(
        value, expected):
    assert sanitize_error_code(value) == expected
    values = build_platform_heartbeat(HeartbeatInputs(error_code=value))
    assert values.system.error_code == expected


def test_malformed_flags_apply_safe_direction_per_signal():
    assert sanitize_flag(math.nan, fail_safe=False) is False
    assert sanitize_flag(math.nan, fail_safe=True) is True

    values = build_platform_heartbeat(HeartbeatInputs(
        charging=math.nan,
        estop=math.nan,
    ))
    assert values.battery.power_supply_status == (
        PowerSupplyStatus.DISCHARGING)
    assert values.system.vehicle_state == VehicleState.ESTOP


@pytest.mark.parametrize(
    "config",
    [
        replace(HeartbeatConfig(), publish_rate_hz=0.9),
        replace(HeartbeatConfig(), publish_rate_hz=20.1),
        replace(HeartbeatConfig(), publish_rate_hz=math.nan),
        replace(HeartbeatConfig(), frame_id="  "),
        replace(HeartbeatConfig(), battery_voltage_v=0.0),
        replace(HeartbeatConfig(), charging_current_a=-1.0),
        replace(HeartbeatConfig(), discharging_current_a=1.0),
        replace(HeartbeatConfig(), battery_temperature_c=math.inf),
    ],
)
def test_unhealthy_static_configuration_is_rejected(config):
    with pytest.raises(ValueError):
        validate_heartbeat_config(config)


def test_configuration_is_normalized_and_applied():
    config = validate_heartbeat_config(HeartbeatConfig(
        publish_rate_hz="10",
        frame_id=" base_link ",
        battery_voltage_v="51.2",
        charging_current_a="2.5",
        discharging_current_a="-3.0",
        battery_temperature_c="31.5",
        battery_location=" carla:test ",
    ))
    values = build_platform_heartbeat(
        HeartbeatInputs(charging=True, soc=0.6), config)

    assert config.publish_rate_hz == 10.0
    assert config.frame_id == "base_link"
    assert values.system.battery_voltage == pytest.approx(51.2)
    assert values.battery.current == pytest.approx(2.5)
    assert values.battery.temperature == pytest.approx(31.5)
    assert values.battery.location == "carla:test"
