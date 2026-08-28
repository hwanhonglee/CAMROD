"""Contracts for feeding the full CAMROD sim graph from an external plant."""

import math
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry
import yaml

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


class _TimerHarness:
    def __init__(self):
        self.timer_period_ns = 200_000_000
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1


def test_runtime_publish_rate_update_retimes_the_existing_timer():
    timer = _TimerHarness()
    node = SimpleNamespace(publish_rate_hz=5.0, timer=timer)

    result = FAKE_SENSOR.FakeSensorPublisher._on_set_parameters(
        node,
        [SimpleNamespace(name="publish_rate_hz", value=10.0)],
    )

    assert result.successful is True
    assert node.publish_rate_hz == 10.0
    assert timer.timer_period_ns == 100_000_000
    assert timer.reset_count == 1


def test_runtime_publish_rate_update_rejects_invalid_rate_without_mutation():
    timer = _TimerHarness()
    node = SimpleNamespace(publish_rate_hz=5.0, timer=timer)

    result = FAKE_SENSOR.FakeSensorPublisher._on_set_parameters(
        node,
        [SimpleNamespace(name="publish_rate_hz", value=float("nan"))],
    )

    assert result.successful is False
    assert "finite and greater than zero" in result.reason
    assert node.publish_rate_hz == 5.0
    assert timer.timer_period_ns == 200_000_000
    assert timer.reset_count == 0


def test_bringup_routes_external_plant_without_disabling_full_sim_graph():
    source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    assert "'external_simulator'" in source
    assert "'external_odometry' if str(" in source
    assert "'sim_publish_platform_status'" in source
    assert "('camrod_bringup', 'fake_sensors.launch.py'" in source


def test_fake_sensor_publishers_release_external_sensor_topic_ownership():
    source = SCRIPT_PATH.read_text(encoding="utf-8")
    assert 'self.declare_parameter("publish_fake_gnss", True)' in source
    assert 'self.declare_parameter("publish_fake_imu", True)' in source
    assert "if self.publish_fake_gnss:" in source
    assert "if self.publish_fake_imu:" in source
    assert "if self.publish_fake_lidar_obstacle_cloud:" in source
    assert "self.pub_navsat = None" in source
    assert "self.pub_imu_ros = None" in source
    assert "self.pub_lidar_filtered = None" in source


def test_external_plant_is_opt_in_and_preserves_develop_defaults():
    """Ordinary CAMROD must not inherit the CARLA runtime profile."""
    launch_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    fake_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "sim"
            / "fake_sensors.yaml"
        ).read_text(encoding="utf-8")
    )["/bringup/fake_sensor_publisher"]["ros__parameters"]

    assert launch_defaults["runtime"]["external_simulator"] is False
    assert launch_defaults["runtime"]["sim_publish_platform_status"] is True
    for key in (
        "sim_publish_fake_gnss",
        "sim_publish_fake_imu",
        "sim_publish_fake_lidar_obstacle_cloud",
        "sim_publish_fake_radar_ranges",
        "sim_publish_velocity_converter_output",
        "sim_publish_dummy_lidar_cost_grid",
    ):
        assert launch_defaults["runtime"][key] is True
    assert fake_defaults["publish_fake_gnss"] is True
    assert fake_defaults["publish_fake_imu"] is True
    assert fake_defaults["publish_fake_lidar_obstacle_cloud"] is True
    assert fake_defaults["publish_fake_radar_ranges"] is True
    assert fake_defaults["motion_source"] == "cmd_vel"


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


def test_bringup_default_through_poses_tree_matches_develop():
    source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    assert "'nav2_bt_navigator'" in source
    assert "'navigate_through_poses_w_replanning_and_recovery.xml'" in source


def test_dedicated_manual_command_boundary_is_opt_in():
    source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    ui_launch = (
        REPO_ROOT / "camrod_ui" / "camrod_ui_robot" / "launch" / "ui.launch.py"
    ).read_text(encoding="utf-8")
    assert "'control_manual_cmd_vel_ros_topic'" in source
    assert (
        "cfg_get(launch_cfg, 'control/manual_cmd_vel_ros_topic', '')"
        in source
    )
    assert (
        "'manual_cmd_vel_ros_topic': lc['control_manual_cmd_vel_ros_topic']"
        in source
    )
    # The empty develop/default value reaches both the safety gate and UI.  A
    # simulator overlay must opt in before the backend creates a publisher or
    # exposes the operator drive panel.
    assert source.count(
        "'manual_cmd_vel_ros_topic': lc['control_manual_cmd_vel_ros_topic']"
    ) >= 2
    assert (
        "'manual_cmd_vel_ros_topic',\n"
        "        default_value='',"
    ) in ui_launch
    assert (
        "'manual_cmd_vel_ros_topic': LaunchConfiguration("
        "'manual_cmd_vel_ros_topic')"
    ) in ui_launch
    assert "'control_manual_drive_deadman_timeout_s'" in source
    assert (
        "cfg_get(launch_cfg, 'control/manual_drive_deadman_timeout_s', 0.25)"
        in source
    )
    assert (
        "'manual_drive_deadman_timeout_s': lc[\n"
        "            'control_manual_drive_deadman_timeout_s'"
        in source
    )
    assert "'manual_drive_deadman_timeout_s'" in ui_launch
