"""Contracts for feeding the full CAMROD sim graph from an external plant."""

import math
import importlib.util
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = (
    REPO_ROOT / "camrod_bringup" / "scripts" / "fake_sensor_publisher.py"
)
SPEC = importlib.util.spec_from_file_location(
    "fake_sensor_publisher_external_test", SCRIPT_PATH
)
FAKE_SENSOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FAKE_SENSOR)


def _odometry(yaw_rad=0.0):
    message = Odometry()
    message.pose.pose.position.x = 1.5
    message.pose.pose.position.y = -2.0
    message.pose.pose.position.z = 0.25
    message.pose.pose.orientation.z = math.sin(0.5 * yaw_rad)
    message.pose.pose.orientation.w = math.cos(0.5 * yaw_rad)
    message.twist.twist.linear.x = 0.4
    message.twist.twist.linear.y = -0.2
    message.twist.twist.angular.z = 0.3
    return message


def test_external_odometry_preserves_pose_and_body_twist():
    state = FAKE_SENSOR.external_odometry_state_from_message(_odometry(0.75))
    assert state == pytest.approx((1.5, -2.0, 0.25, 0.75, 0.4, -0.2, 0.3))


def test_external_odometry_rejects_nonfinite_and_zero_quaternion():
    nonfinite = _odometry()
    nonfinite.pose.pose.position.x = math.nan
    with pytest.raises(ValueError, match="non-finite"):
        FAKE_SENSOR.external_odometry_state_from_message(nonfinite)

    zero_quaternion = _odometry()
    zero_quaternion.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="zero quaternion"):
        FAKE_SENSOR.external_odometry_state_from_message(zero_quaternion)


def test_bringup_routes_external_plant_without_disabling_full_sim_graph():
    source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    assert "'external_simulator'" in source
    assert "'external_odometry' if str(" in source
    assert "'sim_publish_platform_status'" in source
    assert "('camrod_bringup', 'fake_sensors.launch.py'" in source


def test_bringup_forwards_optional_nav2_behavior_tree_overrides():
    source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    defaults = (
        REPO_ROOT
        / "camrod_bringup"
        / "config"
        / "bringup"
        / "launch_defaults.yaml"
    ).read_text(encoding="utf-8")
    for argument in (
        "planning_nav2_bt_xml_nav_to_pose",
        "planning_nav2_bt_xml_nav_through_poses",
    ):
        assert argument in source
        assert argument.removeprefix("planning_") in defaults
    assert "'nav2_bt_xml_nav_to_pose'" in source
    assert "'nav2_bt_xml_nav_through_poses'" in source
    assert (
        "'nav2_bt_xml_nav_to_pose': "
        "lc['planning_nav2_bt_xml_nav_to_pose']"
    ) in source
    assert (
        "'nav2_bt_xml_nav_through_poses': lc[\n"
        "            'planning_nav2_bt_xml_nav_through_poses'"
    ) in source
