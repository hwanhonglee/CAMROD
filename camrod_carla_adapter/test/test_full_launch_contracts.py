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
    assert (
        '"CAMROD_CARLA_UI_TF_LATEST_FALLBACK_TOLERANCE_S", "0.075"'
        in source
    )
    assert (
        '"operator_telemetry_tf_latest_fallback_tolerance_s": ('
        in source
    )
    assert 'controller_share, "full_stack.launch.py"' in source
    assert 'controller_share, "launch", "full_stack.launch.py"' not in source


def test_full_launch_defaults_carla_route_heading_to_production_profile():
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    production_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]

    assert '"CAMROD_CARLA_ROUTE_HEADING_ERROR_ENTER_DEG", "75.0"' in full_launch
    assert '"CAMROD_CARLA_ROUTE_HEADING_LOOKAHEAD_M", "2.0"' in full_launch
    assert (
        '"control_cmd_vel_gate_route_heading_lookahead_m": ('
        in full_launch
    )
    assert (
        '"control_cmd_vel_gate_route_heading_error_enter_deg": ('
        in full_launch
    )
    assert (
        production_defaults["control"][
            "cmd_vel_gate_route_heading_error_enter_deg"
        ]
        == 75.0
    )
    assert (
        production_defaults["control"][
            "cmd_vel_gate_route_heading_error_exit_deg"
        ]
        == 5.0
    )
    assert (
        production_defaults["control"][
            "cmd_vel_gate_route_heading_lookahead_m"
        ]
        == 2.0
    )


def test_full_carla_opts_into_reverse_roadside_return_without_changing_production():
    """CARLA alone exits and retraces in reverse through typed opt-ins."""
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    production_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]

    assert (
        '"CAMROD_CARLA_ROADSIDE_REVERSE_RETURN_ENABLE", "true"'
        in full_launch
    )
    assert (
        '"control_camping_site_roadside_reverse_return_enable": ('
        in full_launch
    )
    assert (
        'LaunchConfiguration(\n'
        '                        "carla_roadside_reverse_return_enable"\n'
        "                    )"
        in full_launch
    )
    assert (
        '"CAMROD_CARLA_ROADSIDE_REVERSE_HANDOFF_DISTANCE_M", "0.10"'
        in full_launch
    )
    assert (
        '"control_camping_site_roadside_reverse_handoff_distance_m": ('
        in full_launch
    )
    assert (
        'LaunchConfiguration(\n'
        '                        "carla_roadside_reverse_handoff_distance_m"\n'
        "                    )"
        in full_launch
    )
    assert "Exit a CARLA roadside campsite, preserve its outbound yaw" in full_launch
    assert '"planning_nav2_runtime_override_param_file": (' in full_launch
    assert '"control_cmd_vel_gate_lanelet_safety_check_reverse": "true"' in full_launch
    assert (
        production_defaults["control"][
            "camping_site_roadside_reverse_return_enable"
        ]
        is False
    )
    assert (
        production_defaults["control"][
            "camping_site_roadside_reverse_handoff_distance_m"
        ]
        == 0.03
    )


def test_carla_reverse_return_overlay_is_slow_and_reverse_capable():
    """CARLA uses a bounded reverse profile without mutating production Nav2."""
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    overlay_path = PACKAGE_ROOT / "config" / "nav2_carla_reverse_return.yaml"
    overlay = yaml.safe_load(overlay_path.read_text(encoding="utf-8"))
    rpp = overlay["controller_server"]["ros__parameters"]["RPP"]

    assert rpp == {
        "allow_reversing": True,
        "use_rotate_to_heading": False,
        "desired_linear_vel": 0.20,
        "use_velocity_scaled_lookahead_dist": False,
        "lookahead_dist": 0.80,
        "min_lookahead_dist": 0.80,
        "max_lookahead_dist": 0.80,
        "regulated_linear_scaling_min_speed": 0.20,
    }

    production_vehicle = yaml.safe_load(
        (REPO_ROOT / "camrod_planning" / "config" / "nav2_vehicle.yaml").read_text(
            encoding="utf-8"
        )
    )
    production_rpp = production_vehicle["controller_server"]["ros__parameters"]["RPP"]
    assert production_rpp["allow_reversing"] is False
    assert production_rpp["use_rotate_to_heading"] is False
    assert production_rpp["desired_linear_vel"] == 1.111111
    assert production_rpp["use_velocity_scaled_lookahead_dist"] is False
    assert production_rpp["lookahead_dist"] == 1.2
    assert production_rpp["min_lookahead_dist"] == 1.1
    assert production_rpp["max_lookahead_dist"] == 2.0
    assert production_rpp["regulated_linear_scaling_min_speed"] == 0.333333

    production_gate_scale = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]["control"]["cmd_vel_gate_speed_scale"]
    assert production_gate_scale == 0.5
    assert (
        '"CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE", "1.0"'
        in full_launch
    )
    assert (
        '"control_cmd_vel_gate_speed_scale": LaunchConfiguration('
        in full_launch
    )
    assert (
        rpp["regulated_linear_scaling_min_speed"] * 1.0
        == 0.20
    )


def test_carla_nav2_radius_stabilizer_is_opt_in_and_above_adapter_boundary():
    """CARLA removes radius chatter without changing production/manual paths."""
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    adapter = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "command_adapter.yaml").read_text(
            encoding="utf-8"
        )
    )["/**"]["ros__parameters"]
    production_defaults = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]["control"]
    package_gate = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_control"
            / "config"
            / "cmd_vel_safety_gate.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    bringup_gate = yaml.safe_load(
        (
            REPO_ROOT
            / "camrod_bringup"
            / "config"
            / "control"
            / "cmd_vel_safety_gate.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    carla_radius = 0.82
    assert carla_radius > adapter["minimum_turn_radius_m"]
    assert (
        '"CAMROD_CARLA_NAVIGATION_MINIMUM_ACKERMANN_TURN_RADIUS_M",\n'
        '                "0.82",'
    ) in full_launch
    assert (
        '"control_cmd_vel_gate_navigation_minimum_ackermann_turn_radius_m": ('
        in full_launch
    )
    assert (
        production_defaults[
            "cmd_vel_gate_navigation_minimum_ackermann_turn_radius_m"
        ]
        == 0.0
    )
    assert package_gate["navigation_minimum_ackermann_turn_radius_m"] == 0.0
    assert bringup_gate["navigation_minimum_ackermann_turn_radius_m"] == 0.0


def test_carla_parking_reaches_station_before_emulating_charger_contact():
    """CARLA may widen the cap, but contact still needs pose/motion evidence."""
    full_launch = (
        PACKAGE_ROOT / "launch" / "camrod_carla_full.launch.py"
    ).read_text(encoding="utf-8")
    overlay = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "parking_carla.yaml").read_text(
            encoding="utf-8"
        )
    )["/parking/reverse_parking_controller"]["ros__parameters"]
    production = yaml.safe_load(
        (REPO_ROOT / "camrod_control" / "config" / "parking.yaml").read_text(
            encoding="utf-8"
        )
    )["/parking/reverse_parking_controller"]["ros__parameters"]

    assert overlay == {"maximum_reverse_distance_m": 4.2}
    assert production["maximum_reverse_distance_m"] == 1.5
    assert production["complete_without_charging"] is False
    assert '"launch_charging_contact_emulator", default_value="true"' in full_launch
    assert '"parking_runtime_override_param_file": LaunchConfiguration(' in full_launch
    assert '"carla_charging_contact_position_tolerance_m"' in full_launch
    assert '"carla_charging_contact_speed_tolerance_mps"' in full_launch
    assert '"carla_charging_contact_state_timeout_s"' in full_launch
    assert '"state_timeout_s": LaunchConfiguration(' in full_launch


def test_charging_contact_rate_isolated_from_heartbeat_legacy_alias():
    """An empty sibling ``publish_rate_hz`` must not disable contact launch."""
    charging_launch = (
        PACKAGE_ROOT / "launch" / "charging_contact_emulator.launch.py"
    ).read_text(encoding="utf-8")

    assert '"charging_contact_publish_rate_hz"' in charging_launch
    assert 'default_value="/planning/state_machine/state"' in charging_launch
    assert 'DeclareLaunchArgument("state_timeout_s", default_value="2.0")' in charging_launch
    assert 'DeclareLaunchArgument("publish_rate_hz"' not in charging_launch
    assert (
        'LaunchConfiguration(\n'
        '                        "charging_contact_publish_rate_hz"\n'
        '                    )'
        in charging_launch
    )


def test_reverse_runtime_overlay_is_forwarded_last_without_production_mutation():
    """Bringup forwards the sparse overlay through planning into Nav2's final chain."""
    bringup_source = (
        REPO_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    planning_source = (
        REPO_ROOT / "camrod_planning" / "launch" / "planning.launch.py"
    ).read_text(encoding="utf-8")
    nav2_source = (
        REPO_ROOT / "camrod_planning" / "launch" / "nav2_lanelet.launch.py"
    ).read_text(encoding="utf-8")

    assert "'planning_nav2_runtime_override_param_file'," in bringup_source
    assert "'nav2_runtime_override_param_file': lc[" in bringup_source
    assert "'nav2_runtime_override_param_file'," in planning_source
    assert "nav2_runtime_override_params = RewrittenYaml(" in nav2_source
    chain_start = nav2_source.index("nav2_param_chain = [")
    runtime_index = nav2_source.index("nav2_runtime_override_params,", chain_start)
    immutable_index = nav2_source.index("force_base_link_overrides,", chain_start)
    assert runtime_index < immutable_index


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
        assert '"config", "woraksan_carla_lanelet2.osm"' in source
        assert (
            '"CAMROD_LANELET_MAP", default=virtual_lanelet_map' in source
        )


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
    assert "/sensing/lidar/points_filtered" not in source
    assert "/perception/obstacles" not in source


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
    lidar_processing_launch = (
        PACKAGE_ROOT / "launch" / "carla_lidar_processing.launch.py"
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
    assert '"lidar_filtered_output"' not in relay_launch
    assert '"obstacle_cloud_output"' not in relay_launch
    assert 'executable="carla_lidar_filter"' in lidar_processing_launch
    assert 'executable="obstacle_lidar_node"' in lidar_processing_launch
    assert '"publish_cluster_cloud": True' in lidar_processing_launch
    assert '"perception_enable_lidar_obstacle": "false"' in full

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
