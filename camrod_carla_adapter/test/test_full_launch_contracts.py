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
    assert '"diagnostics_profile": "carla"' in source
    assert '"diagnostics_profile_fallback": "sim,default"' in source
    assert '"planning_nav2_bt_xml_nav_through_poses"' in source
    assert '"navigate_through_poses_w_planner_selector.xml"' in source
    assert '"enable_api_ui": LaunchConfiguration("enable_api_ui")' in source
    assert (
        '"operator_telemetry_camera_raw_fallback_enabled": "false"'
        in source
    )
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


def test_full_launch_selects_carla_metric_localization_adapter_only():
    """Full CARLA must reuse the accepted subset localization source profile."""
    full = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    subset = (
        PACKAGE_ROOT / "launch" / "camrod_carla.launch.py"
    ).read_text(encoding="utf-8")
    bringup = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    ordinary_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    carla_params = yaml.safe_load(
        (
            PACKAGE_ROOT
            / "config"
            / "camrod_input_adapter_carla.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    for source in (subset, full):
        assert '"camrod_input_adapter_config"' in source
        assert '"camrod_input_adapter_carla.yaml"' in source

    assert (
        '"localization_adapter_param_file": LaunchConfiguration('
        in full
    )
    assert "'localization_adapter_param_file'," in bringup
    assert (
        "localization_args['adapter_param_file'] = lc["
        in bringup
    )
    assert ordinary_defaults["localization"]["adapter_param_file"] == (
        "localization/source/input_adapter.yaml"
    )
    assert carla_params["utm_pose_topic"] == "/camrod_carla/metric_pose"
    assert carla_params["navsat_topic"] == "/camrod_carla/unused_navsat"
    assert carla_params["enable_gnss_heading"] is False


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
    for name, default in (
        ("manual_drive_linear_limit_mps", "1.40"),
        ("manual_drive_lateral_limit_mps", "1.00"),
        ("manual_drive_angular_limit_radps", "0.7853"),
        ("manual_drive_deadman_timeout_s", "0.75"),
    ):
        assert f'"{name}"' in full
        assert f'"{default}"' in full
        assert f'"control_{name}": LaunchConfiguration(' in full


def test_full_carla_uses_hard_lanelet_costs_without_soft_inflation_deadlock():
    """CARLA manual motion keeps lethal cost 100 while ignoring soft 85..99."""
    full = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")

    for argument in (
        "control_cmd_vel_gate_cost_threshold",
        "control_cmd_vel_gate_lanelet_safety_threshold",
        "control_cmd_vel_gate_lanelet_safety_current_threshold",
    ):
        assert f'"{argument}": "100"' in full


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
        "/sensing/lidar/points_filtered",
    ):
        assert topic in source


def test_full_carla_composition_uses_only_actual_carla_ui_sensor_sources():
    full = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    fake_launch = (
        REPO_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    ).read_text(encoding="utf-8")
    relay_launch = (
        PACKAGE_ROOT / "launch" / "sensor_relay.launch.py"
    ).read_text(encoding="utf-8")

    assert '"relay_imu": "true"' in full
    assert '"relay_gnss": "true"' in full
    assert '"compressed_image_max_rate_hz"' in full
    assert '"CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ", "5.0"' in full
    assert '"CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ", "10.0"' in full
    assert (
        '"compressed_image_max_rate_hz": LaunchConfiguration('
        in full
    )
    assert '"raw_image_max_rate_hz": LaunchConfiguration(' in full
    for argument in (
        "sim_publish_fake_gnss",
        "sim_publish_fake_imu",
        "sim_publish_fake_lidar_obstacle_cloud",
        "sim_publish_fake_radar_ranges",
        "sim_publish_velocity_converter_output",
        "sim_publish_dummy_lidar_cost_grid",
    ):
        assert f'"{argument}": "false"' in full

    for argument in (
        "publish_fake_gnss",
        "publish_fake_imu",
        "publish_fake_lidar_obstacle_cloud",
        "publish_fake_radar_ranges",
        "publish_velocity_converter_output",
        "publish_dummy_lidar_cost_grid",
    ):
        assert f"'{argument}'" in fake_launch

    assert '"lidar_output"' in relay_launch
    assert '"lidar_filtered_output"' in relay_launch
    assert '"obstacle_cloud_output"' in relay_launch
    assert 'default_value="/perception/obstacles"' in relay_launch

    feedback_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "feedback_bridge.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    assert feedback_config["publish_gnss_heading_from_imu"] is True
    assert feedback_config["output_gnss_heading_topic"] == (
        "/sensing/gnss/navheading"
    )
    assert feedback_config["gnss_heading_yaw_bias_rad"] == (
        1.5707963267948966
    )
    assert feedback_config["imu_frame_id"] == "imu_link"
    assert feedback_config["gnss_frame_id"] == "gnss_link"
    assert feedback_config["input_gnss_right_topic"] == (
        "/carla/ego_vehicle/gnss_right"
    )
    assert feedback_config["publish_gnss_ui_compat"] is True
    assert feedback_config["output_gnss_navpvt_topic"] == (
        "/sensing/gnss/ublox_gps_node/navpvt"
    )
    assert feedback_config["output_gnss_navcov_topic"] == (
        "/sensing/gnss/navcov"
    )
    assert feedback_config["output_gnss_relpos_topic"] == (
        "/sensing/gnss/navrelposned"
    )


def test_carla_diagnostics_match_rendered_sensor_contract():
    root = (
        REPO_ROOT
        / "camrod_bringup"
        / "config"
        / "system"
        / "diagnostics"
        / "carla"
    )
    camera = yaml.safe_load(
        (root / "sensing" / "camera_checker.yaml").read_text(
            encoding="utf-8"
        )
    )["/system/camera_checker"]["ros__parameters"]
    assert camera["econ_front"]["expected_fps"] == 2.0
    assert camera["econ_rear"]["expected_fps"] == 2.0
    assert camera["econ_front"]["expected_width"] == 800
    assert camera["econ_front"]["expected_height"] == 600
    assert camera["econ_rear"]["image_type"] == "compressed"
    assert camera["econ_rear"]["image_topic"] == (
        "/sensing/camera/econ_rear/image_raw/compressed"
    )
    assert camera["econ_rear"]["expected_encoding"] == ""

    aggregator = yaml.safe_load(
        (root / "aggregator" / "diagnostics_config.yaml").read_text(
            encoding="utf-8"
        )
    )
    ignored = set(aggregator["global"]["ignored_names"])
    for active_name in (
        "camera_checker: /sensor/camera/econ_front",
        "camera_checker: /sensor/camera/econ_rear",
        "lidar_checker: /sensor/lidar/main",
        "radar_checker: /sensor/radar/FRONT1",
        "radar_checker: /sensor/radar/REAR",
        "perception_obstacle_checker: /perception/obstacles/fused_obstacles",
    ):
        assert active_name not in ignored
    assert (
        "perception_obstacle_checker: /perception/obstacles/camera_detections"
        in ignored
    )

    for filename, node_name, rate_key, stream_name in (
        ("gnss_checker.yaml", "gnss_checker", "expected_hz", "main"),
        ("imu_checker.yaml", "imu_checker", "expected_hz", "main"),
        ("lidar_checker.yaml", "lidar_checker", "expected_hz", "main"),
        (
            "wheel_odometry_checker.yaml",
            "wheel_odometry_checker",
            "expected_hz",
            "main",
        ),
    ):
        params = yaml.safe_load(
            (root / "sensing" / filename).read_text(encoding="utf-8")
        )[f"/system/{node_name}"]["ros__parameters"]
        assert params[stream_name][rate_key] == 2.0
