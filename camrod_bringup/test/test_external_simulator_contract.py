"""Contracts for feeding the full CAMROD sim graph from an external plant."""

import math
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import SetLaunchConfiguration
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

PERCEPTION_LAUNCH_PATH = (
    REPO_ROOT / "camrod_perception" / "launch" / "perception.launch.py"
)
PERCEPTION_SPEC = importlib.util.spec_from_file_location(
    "camrod_perception_external_camera_test", PERCEPTION_LAUNCH_PATH
)
PERCEPTION_LAUNCH = importlib.util.module_from_spec(PERCEPTION_SPEC)
PERCEPTION_SPEC.loader.exec_module(PERCEPTION_LAUNCH)


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


@pytest.mark.parametrize(
    ("external_camera", "expected"),
    (("false", "false"), ("true", "true")),
)
def test_external_camera_keeps_production_yolo_and_fusion_in_sim(
    external_camera, expected
):
    context = LaunchContext()
    context.launch_configurations.update({
        "perception_mode": "auto",
        "enable_camera": "true",
        "enable_front_camera": "true",
        "sim": "true",
        "front_camera_source_external": external_camera,
        "enable_yolo": "true",
        "camera_device_path": "/definitely/not/a/hardware/camera",
    })

    actions = PERCEPTION_LAUNCH._resolve_camera_pipeline(context)
    for action in actions:
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)

    assert context.launch_configurations["enable_camera_effective"] == expected
    assert context.launch_configurations["enable_yolo_effective"] == expected
    assert (
        context.launch_configurations["enable_obstacle_fusion_effective"]
        == expected
    )


def test_non_external_camera_resolution_matches_develop_truth_table(tmp_path):
    """The CARLA extension must be inert when its owner flag is false."""
    existing_device = tmp_path / "video0"
    existing_device.touch()
    missing_device = tmp_path / "missing-video"

    for mode in ("auto", "lidar_only", "camera_lidar", "invalid-mode"):
        for enable_camera in (False, True):
            for enable_front in (False, True):
                for sim in (False, True):
                    for enable_yolo in (False, True):
                        for device_exists in (False, True):
                            camera_path = (
                                existing_device
                                if device_exists
                                else missing_device
                            )
                            context = LaunchContext()
                            context.launch_configurations.update({
                                "perception_mode": mode,
                                "enable_camera": str(enable_camera).lower(),
                                "enable_front_camera": str(enable_front).lower(),
                                "sim": str(sim).lower(),
                                "front_camera_source_external": "false",
                                "enable_yolo": str(enable_yolo).lower(),
                                "camera_device_path": str(camera_path),
                            })

                            actions = PERCEPTION_LAUNCH._resolve_camera_pipeline(
                                context
                            )
                            for action in actions:
                                if isinstance(action, SetLaunchConfiguration):
                                    action.execute(context)

                            # This is the exact origin/develop resolver truth
                            # table before external ownership was introduced.
                            requested = (
                                enable_camera and enable_front and not sim
                            )
                            yolo_requested = enable_yolo
                            normalized_mode = mode
                            if mode == "lidar_only":
                                requested = False
                                yolo_requested = False
                            elif mode == "camera_lidar":
                                requested = True
                            elif mode != "auto":
                                normalized_mode = "auto"
                            available = requested and device_exists

                            assert normalized_mode in {
                                "auto", "lidar_only", "camera_lidar"
                            }
                            assert context.launch_configurations[
                                "enable_camera_effective"
                            ] == str(available).lower()
                            assert context.launch_configurations[
                                "enable_yolo_effective"
                            ] == str(
                                yolo_requested and available
                            ).lower()
                            assert context.launch_configurations[
                                "enable_obstacle_fusion_effective"
                            ] == str(available).lower()


class _TimerHarness:
    """Record whether a runtime update mutates the startup timer."""

    def __init__(self):
        self.timer_period_ns = 200_000_000
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1


@pytest.mark.parametrize(
    ("rate_hz", "expected_period_s"),
    (
        (10.0, 0.1),
        (1.0, 1.0),
        (0.5, 1.0),
        (0.0, 1.0),
        (-5.0, 1.0),
        (float("inf"), 0.0),
    ),
)
def test_startup_publish_rate_period_matches_develop_clamp(
    rate_hz, expected_period_s
):
    assert FAKE_SENSOR.publish_timer_period_seconds(rate_hz) == pytest.approx(
        expected_period_s
    )


def test_startup_publish_rate_nan_matches_develop_edge_behavior():
    assert math.isnan(
        FAKE_SENSOR.publish_timer_period_seconds(float("nan"))
    )


def test_runtime_publish_rate_update_preserves_develop_timer_period():
    timer = _TimerHarness()
    node = SimpleNamespace(publish_rate_hz=5.0, timer=timer)

    result = FAKE_SENSOR.FakeSensorPublisher._on_set_parameters(
        node,
        [SimpleNamespace(name="publish_rate_hz", value=10.0)],
    )

    assert result.successful is True
    assert node.publish_rate_hz == 10.0
    assert timer.timer_period_ns == 200_000_000
    assert timer.reset_count == 0


@pytest.mark.parametrize("rate_hz", (0.0, -5.0, float("inf"), float("nan")))
def test_runtime_publish_rate_edge_values_match_develop(rate_hz):
    timer = _TimerHarness()
    node = SimpleNamespace(publish_rate_hz=5.0, timer=timer)

    result = FAKE_SENSOR.FakeSensorPublisher._on_set_parameters(
        node,
        [SimpleNamespace(name="publish_rate_hz", value=rate_hz)],
    )

    assert result.successful is True
    if math.isnan(rate_hz):
        assert math.isnan(node.publish_rate_hz)
    else:
        assert node.publish_rate_hz == rate_hz
    assert timer.timer_period_ns == 200_000_000
    assert timer.reset_count == 0


def test_bringup_exposes_external_plant_fixture_boundaries():
    source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    assert "'external_simulator'" in source
    assert "'external_odometry' if str(" in source
    assert "'sim_publish_platform_status'" in source
    assert "'fake_sensors.launch.py'" in source
    assert "'enable_fake_sensors'" in source
    assert "'use_sim_planning_profile'" in source
    assert "'use_sim_localization_profile'" in source
    assert "'use_sim_parking_method'" in source
    assert "'external_front_camera_source'" in source
    assert "'external_rear_camera_source'" in source


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
    assert launch_defaults["runtime"]["enable_fake_sensors"] is True
    assert launch_defaults["runtime"]["use_sim_planning_profile"] is True
    assert launch_defaults["runtime"]["use_sim_localization_profile"] is True
    assert launch_defaults["runtime"]["use_sim_parking_method"] is True
    assert launch_defaults["runtime"]["external_front_camera_source"] is False
    assert launch_defaults["runtime"]["external_rear_camera_source"] is False
    assert launch_defaults["system"][
        "operator_telemetry_raw_lidar_bbox_overlay_enabled"
    ] is True
    assert launch_defaults["perception"][
        "runtime_override_param_file"
    ] == "__module_default__"
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


def test_production_perception_runtime_overlay_is_empty_and_forwarded_last():
    disabled = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_perception"
            / "config"
            / "perception_runtime_profiles"
            / "disabled.yaml"
        ).read_text(encoding="utf-8")
    )
    fusion_launch = (
        REPO_ROOT
        / "camrod_perception"
        / "launch"
        / "obstacle_fusion.launch.py"
    ).read_text(encoding="utf-8")
    yolo_launch = (
        REPO_ROOT
        / "camrod_perception"
        / "launch"
        / "yolo.launch.py"
    ).read_text(encoding="utf-8")
    perception_launch = PERCEPTION_LAUNCH_PATH.read_text(encoding="utf-8")
    bringup_launch = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")

    assert disabled["/perception/obstacle_fusion"]["ros__parameters"] == {}
    parameter_chain = fusion_launch.split("parameters=[", 1)[1].split(
        "],", 1
    )[0]
    assert parameter_chain.index(
        "LaunchConfiguration('perception_param_file')"
    ) < parameter_chain.index(
        "LaunchConfiguration('perception_runtime_override_param_file')"
    )
    yolo_parameter_chain = yolo_launch.split("parameters=[", 1)[1].split(
        "],", 1
    )[0]
    assert yolo_parameter_chain.index(
        "LaunchConfiguration('perception_param_file')"
    ) < yolo_parameter_chain.index(
        "LaunchConfiguration('perception_runtime_override_param_file')"
    )
    assert perception_launch.count(
        "'perception_runtime_override_param_file'"
    ) >= 2
    assert "'perception_runtime_override_param_file': lc[" in bringup_launch


def test_carla_recovery_behaviors_are_explicit_default_off_overlays():
    defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")

    assert defaults["planning"][
        "goal_snapper_reissue_active_goal_after_route_recovery_when_nav_active"
    ] is False
    for key in (
        "route_safety_recovery_zero_hold_pauses_limits",
        "route_safety_recovery_allow_corrective_yaw_beyond_limit",
        "cmd_vel_gate_route_safety_path_relative_recovery_enable",
    ):
        assert defaults["control"][key] is False
    assert defaults["control"][
        "cmd_vel_gate_route_safety_path_center_reentry_m"
    ] == 0.08
    assert defaults["control"][
        "cmd_vel_gate_cost_stop_latch_use_trigger_source_for_merged_clear"
    ] is False
    assert defaults["control"][
        "cmd_vel_gate_cost_stop_merged_dynamic_source_labels"
    ] == ""

    for launch_argument in (
        "planning_goal_snapper_reissue_active_goal_after_route_recovery_when_nav_active",
        "control_route_safety_recovery_zero_hold_pauses_limits",
        "control_route_safety_recovery_allow_corrective_yaw_beyond_limit",
        "control_cmd_vel_gate_route_safety_path_relative_recovery_enable",
        "control_cmd_vel_gate_route_safety_path_center_reentry_m",
        "control_cmd_vel_gate_cost_stop_latch_use_trigger_source_for_merged_clear",
        "control_cmd_vel_gate_cost_stop_merged_dynamic_source_labels",
    ):
        assert launch_argument in source


def test_local_path_subscription_expansion_is_path_recovery_opt_in_only():
    """The generic develop recovery switch must not widen the topic graph."""
    source = (
        REPO_ROOT
        / "camrod_control"
        / "src"
        / "cmd_vel_safety_gate_node.cpp"
    ).read_text(encoding="utf-8")
    subscription_index = source.index("route_path_subscription_ =")
    condition_start = source.rfind("if (", 0, subscription_index)
    condition = " ".join(source[condition_start:subscription_index].split())

    assert "path_relative_recovery_config_.enabled" in condition
    assert "route_safety_recovery_config_.enabled" not in condition


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
