"""Safety and launch-contract tests for the disabled Ranger dummy."""

import importlib.util
import math
from pathlib import Path

import pytest
from builtin_interfaces.msg import Time
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts/platform_dummy_publisher.py"
LAUNCH_PATH = PACKAGE_ROOT / "launch/ranger.launch.py"


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


PLATFORM_DUMMY = _load_module("platform_dummy_publisher", SCRIPT_PATH)
RANGER_LAUNCH = _load_module("camrod_ranger_launch", LAUNCH_PATH)


def _stamp():
    return Time(sec=123, nanosec=456)


def test_dummy_payload_is_estop_non_can_and_faulted():
    message = PLATFORM_DUMMY.build_safe_system_state(
        _stamp(), "robot_base_link"
    )

    assert message.vehicle_state == message.VEHICLE_STATE_ESTOP
    assert message.control_mode == message.CONTROL_MODE_RC
    assert message.error_code == PLATFORM_DUMMY.DUMMY_ERROR_CODE
    assert message.error_code != 0
    assert message.battery_voltage == 0.0


def test_dummy_odometry_is_zero_with_unknown_covariance():
    message = PLATFORM_DUMMY.build_safe_odometry(
        _stamp(), "odom", "robot_base_link"
    )

    assert message.header.frame_id == "odom"
    assert message.child_frame_id == "robot_base_link"
    assert message.pose.pose.orientation.w == 1.0
    assert message.twist.twist.linear.x == 0.0
    assert message.twist.twist.linear.y == 0.0
    assert message.twist.twist.angular.z == 0.0
    for index in (0, 7, 14, 21, 28, 35):
        assert message.pose.covariance[index] == (
            PLATFORM_DUMMY.UNKNOWN_COVARIANCE
        )
        assert message.twist.covariance[index] == (
            PLATFORM_DUMMY.UNKNOWN_COVARIANCE
        )


def test_dummy_actuators_are_stopped_and_explicitly_faulted():
    message = PLATFORM_DUMMY.build_safe_actuator_state(
        _stamp(), "robot_base_link"
    )

    assert [state.id for state in message.states] == list(range(8))
    assert all(state.motor.rpm == 0 for state in message.states)
    assert all(
        state.motor.motor_speeds == 0.0 for state in message.states
    )
    assert all(
        state.driver.driver_state == PLATFORM_DUMMY.DUMMY_DRIVER_STATE
        for state in message.states
    )


def test_dummy_battery_is_absent_and_unknown():
    message = PLATFORM_DUMMY.build_safe_battery_state(
        _stamp(), "robot_base_link"
    )

    assert message.present is False
    assert math.isnan(message.voltage)
    assert math.isnan(message.current)
    assert math.isnan(message.percentage)
    assert (
        message.power_supply_status
        == message.POWER_SUPPLY_STATUS_UNKNOWN
    )
    assert message.power_supply_health == message.POWER_SUPPLY_HEALTH_UNKNOWN


@pytest.mark.parametrize(
    ("rate", "odom_frame", "base_frame"),
    (
        (0.9, "odom", "robot_base_link"),
        (20.1, "odom", "robot_base_link"),
        (math.nan, "odom", "robot_base_link"),
        (5.0, "", "robot_base_link"),
        (5.0, "odom", " "),
    ),
)
def test_invalid_configuration_is_rejected(rate, odom_frame, base_frame):
    with pytest.raises(ValueError):
        PLATFORM_DUMMY.validate_configuration(
            rate, odom_frame, base_frame
        )


def _launch_nodes():
    context = LaunchContext()
    context.launch_configurations.update({
        "params_file": str(
            PACKAGE_ROOT / "config/ranger_driver.yaml"
        ),
        "platform_type": "ranger",
        "enable_ranger_base_node": "false",
        "enable_ranger_dummy_when_disabled": "true",
        "enable_ranger_bridge_node": "false",
        "auto_setup_can": "false",
        "can_bitrate": "500000",
        "can_restart_ms": "100",
        "dummy_publish_rate_hz": "5.0",
    })
    actions = RANGER_LAUNCH._launch_setup(context)
    return context, actions[0], actions[1]


@pytest.mark.parametrize(
    ("platform_type", "real_enabled", "dummy_enabled", "expected"),
    (
        ("ranger", "true", "true", (True, False)),
        ("ranger", "false", "true", (False, True)),
        ("ranger", "false", "false", (False, False)),
        ("rmp401", "false", "true", (False, False)),
    ),
)
def test_real_driver_and_dummy_are_never_active_together(
    platform_type, real_enabled, dummy_enabled, expected
):
    context, real_node, dummy_node = _launch_nodes()
    context.launch_configurations["platform_type"] = platform_type
    context.launch_configurations[
        "enable_ranger_base_node"
    ] = real_enabled
    context.launch_configurations[
        "enable_ranger_dummy_when_disabled"
    ] = dummy_enabled

    actual = (
        real_node.condition.evaluate(context),
        dummy_node.condition.evaluate(context),
    )
    assert actual == expected
