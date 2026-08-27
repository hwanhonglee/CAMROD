"""Contracts for the full CAMROD-on-CARLA composition."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parent


def _load_module(path):
    spec = importlib.util.spec_from_file_location(
        path.name.replace(".", "_"), str(path)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _resolve_rate(module, primary_name, primary_value, legacy_value):
    context = LaunchContext()
    context.launch_configurations[primary_name] = primary_value
    context.launch_configurations["publish_rate_hz"] = legacy_value
    substitution = module._prefer_explicit_legacy_rate(primary_name)
    return perform_substitutions(context, [substitution])


def test_full_launch_keeps_carla_lifecycle_external_and_enables_full_bringup():
    source = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    assert 'get_package_share_directory("carla_ros_bridge")' not in source
    assert 'get_package_share_directory("carla_spawn_objects")' not in source
    assert '"sim": "true"' in source
    assert '"external_simulator": "true"' in source
    assert '"sim_platform_status_enable": "true"' in source
    assert '"clean_before_launch": "false"' in source
    assert '"platform_odometry_topic": "/odom"' in source
    assert '"platform_ranger_bridge_enable": "true"' in source
    assert '"planning_nav2_bt_xml_nav_through_poses"' in source
    assert '"navigate_through_poses_w_planner_selector.xml"' in source
    assert '"enable_api_ui": LaunchConfiguration("enable_api_ui")' in source
    assert 'controller_share, "full_stack.launch.py"' in source
    assert 'controller_share, "launch", "full_stack.launch.py"' not in source


def test_fake_sensor_rate_isolated_from_platform_heartbeat_launch_scope():
    """A sibling 5 Hz include must not down-rate 10 Hz readiness topics."""
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    heartbeat_launch = (
        PACKAGE_ROOT / "launch" / "platform_heartbeat.launch.py"
    ).read_text(encoding="utf-8")
    fake_launch = (
        REPO_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    ).read_text(encoding="utf-8")
    bringup_impl = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]

    assert '"platform_heartbeat_publish_rate_hz"' in heartbeat_launch
    assert (
        '"platform_heartbeat_publish_rate_hz", default_value="5.0"'
        in full_launch
    )
    assert (
        '"platform_heartbeat_publish_rate_hz": LaunchConfiguration('
        in full_launch
    )
    assert (
        'DeclareLaunchArgument(\n            "publish_rate_hz",'
        in heartbeat_launch
    )
    assert 'default_value="",' in heartbeat_launch
    assert 'LaunchConfiguration("publish_rate_hz")' in heartbeat_launch
    assert (
        '"platform_heartbeat_publish_rate_hz"'
        in heartbeat_launch
    )
    assert '"sim_fake_sensor_publish_rate_hz": "10.0"' in full_launch
    assert "'fake_sensor_publish_rate_hz'" in fake_launch
    assert "'fake_sensor_publish_rate_hz'\n    )" in fake_launch
    assert "LaunchConfiguration('publish_rate_hz')" in fake_launch
    assert "'publish_rate_hz',\n        default_value=''," in fake_launch
    assert "PythonExpression([" in fake_launch
    assert "'sim_fake_sensor_publish_rate_hz'" in bringup_impl
    assert "'fake_sensor_publish_rate_hz': lc[" in bringup_impl
    assert defaults["sim"]["fake_sensor_publish_rate_hz"] == 10.0


def test_rate_alias_defaults_do_not_cross_includes_and_legacy_is_supported():
    heartbeat_module = _load_module(
        PACKAGE_ROOT / "launch" / "platform_heartbeat.launch.py"
    )
    fake_module = _load_module(
        REPO_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    )

    assert _resolve_rate(
        heartbeat_module,
        "platform_heartbeat_publish_rate_hz",
        "5.0",
        "",
    ) == "5.0"
    assert _resolve_rate(
        fake_module,
        "fake_sensor_publish_rate_hz",
        "10.0",
        "",
    ) == "10.0"
    assert _resolve_rate(
        heartbeat_module,
        "platform_heartbeat_publish_rate_hz",
        "5.0",
        "7.5",
    ) == "7.5"
    assert _resolve_rate(
        fake_module,
        "fake_sensor_publish_rate_hz",
        "10.0",
        "12.0",
    ) == "12.0"


def test_subset_launch_forwards_multi_pose_selector_tree():
    source = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")
    assert '"nav2_bt_xml_nav_through_poses"' in source
    assert '"navigate_through_poses_w_planner_selector.xml"' in source
    assert (
        '"nav2_bt_xml_nav_through_poses": LaunchConfiguration('
        in source
    )


def test_dedicated_manual_twist_is_enabled_only_by_carla_compositions():
    control_launch = (
        REPO_ROOT / "camrod_control" / "launch" / "cmd_vel_safety_gate.launch.py"
    ).read_text(encoding="utf-8")
    bringup = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    ui_launch = (
        REPO_ROOT / "camrod_ui" / "camrod_ui_robot" / "launch" / "ui.launch.py"
    ).read_text(encoding="utf-8")

    assert (
        '"manual_input_topic": LaunchConfiguration("manual_cmd_vel_ros_topic")'
        in control_launch
    )
    assert (
        'DeclareLaunchArgument("manual_cmd_vel_ros_topic", default_value="")'
        in control_launch
    )
    assert (
        "cfg_get(launch_cfg, 'control/manual_cmd_vel_ros_topic', '')"
        in bringup
    )
    assert (
        "'manual_cmd_vel_ros_topic': lc['control_manual_cmd_vel_ros_topic']"
        in bringup
    )
    assert bringup.count(
        "'manual_cmd_vel_ros_topic': lc['control_manual_cmd_vel_ros_topic']"
    ) >= 2
    assert (
        "'manual_cmd_vel_ros_topic': LaunchConfiguration("
        "'manual_cmd_vel_ros_topic')"
    ) in ui_launch

    for filename in (
        "camrod_carla.launch.py",
        "camrod_carla_full.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / filename).read_text(
            encoding="utf-8"
        )
        assert '"manual_cmd_vel_ros_topic"' in source
        assert 'default_value="/control/manual_cmd_vel_ros"' in source

    subset = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")
    full = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    assert '"manual_cmd_vel_ros_topic": LaunchConfiguration(' in subset
    assert '"control_manual_cmd_vel_ros_topic": LaunchConfiguration(' in full


def test_composition_launches_have_no_host_absolute_paths_and_use_env_gates():
    for filename in (
        "camrod_carla.launch.py",
        "camrod_carla_full.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / filename).read_text(
            encoding="utf-8"
        )
        assert "/home/hong" not in source
        assert "/home/nvidia" not in source
        assert "RANGER_CARLA_ROOT" in source
        assert "RANGER_BASELINE_MANIFEST" in source
        assert "RANGER_PHYSICAL_MANIFEST" in source
        assert "RANGER_CARLA_PYTHON_EGG" in source
        assert "RANGER_PYTHON_EGG_CACHE" in source
        assert "ranger_ros_backend_gate_20260814" not in source
        assert "ranger_physical_4ws_acceptance_gate_20260823" not in source


def test_composition_launches_forward_the_explicit_camrod_map_path():
    for filename in (
        "camrod_carla.launch.py",
        "camrod_carla_full.launch.py",
    ):
        source = (PACKAGE_ROOT / "launch" / filename).read_text(
            encoding="utf-8"
        )
        assert '"camrod_map_path", default_value=camrod_map_path' in source
        assert '"map_path": LaunchConfiguration("camrod_map_path")' in source


def test_sensor_relay_uses_camrod_canonical_topics():
    source = (
        PACKAGE_ROOT
        / "src"
        / "camrod_carla_adapter"
        / "sensor_relay_node.py"
    ).read_text(encoding="utf-8")
    for topic in (
        "/sensing/camera/econ_front/image_raw",
        "/sensing/camera/econ_front/image_rect",
        "/sensing/camera/econ_front/camera_info",
        "/sensing/camera/econ_rear/image_raw",
        "/sensing/camera/econ_rear/image_rect",
        "/sensing/camera/econ_rear/camera_info",
        "/sensing/lidar/vanjee/points_raw",
    ):
        assert topic in source
