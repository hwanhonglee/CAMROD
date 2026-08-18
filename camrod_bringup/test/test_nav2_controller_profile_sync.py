"""Keep manual and UI Nav2 path-tracking behavior synchronized."""

import math
from pathlib import Path
import re

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
PLANNING_CONFIG = SRC_ROOT / "camrod_planning" / "config"
BRINGUP_CONFIG = SRC_ROOT / "camrod_bringup" / "config" / "planning"
CONTROL_CONFIG = SRC_ROOT / "camrod_control" / "config"
BRINGUP_CONTROL_CONFIG = SRC_ROOT / "camrod_bringup" / "config" / "control"

SHARED_TRACKING_KEYS = (
    "desired_linear_vel",
    "lookahead_dist",
    "min_lookahead_dist",
    "max_lookahead_dist",
    "lookahead_time",
    "use_velocity_scaled_lookahead_dist",
    "use_interpolation",
    "allow_reversing",
    "use_rotate_to_heading",
    "min_approach_linear_velocity",
    "approach_velocity_scaling_dist",
    "use_collision_detection",
    "max_allowed_time_to_collision_up_to_carrot",
    "use_regulated_linear_velocity_scaling",
    "regulated_linear_scaling_min_radius",
    "regulated_linear_scaling_min_speed",
    "use_cost_regulated_linear_velocity_scaling",
    "cost_scaling_dist",
    "cost_scaling_gain",
)


def _parameters(path: Path) -> dict:
    """Load one Nav2 parameter overlay."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))[
        "controller_server"
    ]["ros__parameters"]


def _node_parameters(path: Path, node_name: str) -> dict:
    """Load parameters for one explicitly named control node."""
    return yaml.safe_load(path.read_text(encoding="utf-8"))[node_name][
        "ros__parameters"
    ]


def _final_kph(raw_mps: float, gate_scale: float) -> float:
    return raw_mps * gate_scale * 3.6


def test_package_and_bringup_nav2_profiles_keep_only_the_preview_ab_split() -> None:
    """Keep the field A/B difference explicit and limited to preview geometry."""
    preview_keys = {
        "lookahead_dist",
        "min_lookahead_dist",
        "max_lookahead_dist",
        "use_velocity_scaled_lookahead_dist",
    }
    for filename, plugin in (
        ("nav2_base.yaml", "RotationShim"),
        ("nav2_vehicle.yaml", "RPP"),
    ):
        package = yaml.safe_load(
            (PLANNING_CONFIG / filename).read_text(encoding="utf-8")
        )
        deployed = yaml.safe_load(
            (BRINGUP_CONFIG / filename).read_text(encoding="utf-8")
        )
        package_profile = package["controller_server"]["ros__parameters"][plugin]
        deployed_profile = deployed["controller_server"]["ros__parameters"][plugin]

        # HH_260818 - worak-test intentionally deploys the long scaled preview
        # while retaining a fixed package profile for the pending field A/B.
        assert {key: package_profile[key] for key in preview_keys} == {
            "lookahead_dist": 1.2,
            "min_lookahead_dist": 1.1,
            "max_lookahead_dist": 2.0,
            "use_velocity_scaled_lookahead_dist": False,
        }
        assert {key: deployed_profile[key] for key in preview_keys} == {
            "lookahead_dist": 3.5,
            "min_lookahead_dist": 1.5,
            "max_lookahead_dist": 3.5,
            "use_velocity_scaled_lookahead_dist": True,
        }
        for key in preview_keys:
            deployed_profile[key] = package_profile[key]
        assert deployed == package


def test_real_controller_matches_twenty_hz_ekf_prediction() -> None:
    """Keep localization prediction and path-control periods synchronized."""
    controller = _parameters(PLANNING_CONFIG / "nav2_base.yaml")
    localization = yaml.safe_load(
        (
            SRC_ROOT / "camrod_localization" / "config" / "filter" / "ekf.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    checker_path = (
        SRC_ROOT
        / "camrod_system"
        / "config"
        / "diagnostics"
        / "default"
        / "localization"
        / "localization_pose_checker.yaml"
    )
    checker_mirror = (
        SRC_ROOT
        / "camrod_bringup"
        / "config"
        / "system"
        / "diagnostics"
        / "default"
        / "localization"
        / "localization_pose_checker.yaml"
    )
    checker = yaml.safe_load(checker_path.read_text(encoding="utf-8"))[
        "localization_pose_checker"
    ]["ros__parameters"]

    # HH_260807 - GNSS corrections are configured at 5 Hz; the 20 Hz EKF and
    # controller still use IMU/wheel prediction between absolute updates.
    assert checker_path.read_bytes() == checker_mirror.read_bytes()
    assert controller["controller_frequency"] == 20.0
    assert localization["frequency"] == 20.0
    assert checker["expected_hz"] == 20.0
    assert checker["hz_warn_ratio"] == 0.90
    assert checker["hz_error_ratio"] == 0.70


def test_local_path_diagnostic_ignores_service_owned_motion_handoffs() -> None:
    """An expected Nav2 path clear must not flash SYSTEM ERROR in a maneuver."""
    package_path = (
        SRC_ROOT
        / "camrod_system"
        / "config"
        / "diagnostics"
        / "default"
        / "planning"
        / "planning_path_checker.yaml"
    )
    bringup_path = (
        SRC_ROOT
        / "camrod_bringup"
        / "config"
        / "system"
        / "diagnostics"
        / "default"
        / "planning"
        / "planning_path_checker.yaml"
    )
    parameters = yaml.safe_load(package_path.read_text(encoding="utf-8"))[
        "planning_path_checker"
    ]["ros__parameters"]
    source = (
        SRC_ROOT
        / "camrod_system"
        / "src"
        / "diagnostics"
        / "planning_path_checker_node.cpp"
    ).read_text(encoding="utf-8")
    # HH_260807 - Keep the package and deployed checker contract byte-identical.
    assert package_path.read_bytes() == bringup_path.read_bytes()
    assert parameters["service_state_topic"] == "/service/state"
    assert parameters["local_path_suppressed_service_states"] == [
        0, 2, 5, 6, 8, 10, 11, 12, 13, 14, 15, 16
    ]
    assert parameters["global_path"]["point_count_grace_s"] == 0.0
    assert parameters["local_path"]["point_count_grace_s"] == 3.0
    assert 'src.name == "local_path"' in source
    assert source.count("rclcpp::QoS(1).reliable()") >= 3
    assert "src->point_count < src->min_points_error" in source
    assert "warn_point_count_since" in source
    # HH_260807 - Idle empty paths cannot pre-consume the next route's grace.
    assert "if (active && !nav_active_)" in source
    assert "nav_active_since_ = this->now()" in source
    assert "error_grace_elapsed" in source
    assert "std::min(\n      low_point_count_elapsed, nav_active_elapsed)" in source
    assert "path point count warning grace" in source
    assert "service state does not require a Nav2 local path" in source
    assert "path point count transition grace" in source


def test_local_path_chain_matches_twenty_hz_pose_control_clock() -> None:
    """Keep safety-path and tracking feedback on the 50 ms control period."""
    package_path = PLANNING_CONFIG / "local_path_extractor.yaml"
    bringup_path = BRINGUP_CONFIG / "local_path_extractor.yaml"
    parameters = _node_parameters(
        package_path,
        "/planning/local_path_extractor",
    )
    launch_text = (
        SRC_ROOT / "camrod_planning" / "launch" / "local_path.launch.py"
    ).read_text(encoding="utf-8")
    extractor_source = (
        SRC_ROOT
        / "camrod_planning"
        / "src"
        / "local_path_extractor_node.cpp"
    ).read_text(encoding="utf-8")
    tracking_source = (
        SRC_ROOT / "camrod_planning" / "src" / "path_tracking_error_node.cpp"
    ).read_text(encoding="utf-8")
    checker_source = (
        SRC_ROOT
        / "camrod_system"
        / "src"
        / "diagnostics"
        / "localization_pose_checker_node.cpp"
    ).read_text(encoding="utf-8")

    # HH_260806 - Pose callbacks publish immediately; 20 Hz also governs each
    # fallback/default path when an overlay or callback is temporarily absent.
    assert package_path.read_bytes() == bringup_path.read_bytes()
    assert parameters["publish_rate_hz"] == 20.0
    assert "'publish_rate_hz': 20.0" in launch_text
    assert 'declare_parameter<double>("publish_rate_hz", 20.0)' in extractor_source
    assert 'declare_parameter<double>("publish_rate_hz", 20.0)' in tracking_source
    assert 'declare_parameter("expected_hz",         20.0)' in checker_source


def test_obstacle_fallback_requires_twenty_seconds_on_a_wide_lane() -> None:
    """Delay planner preemption without weakening immediate command stopping."""
    filename = "obstacle_replan_monitor.yaml"
    package_path = PLANNING_CONFIG / filename
    bringup_path = BRINGUP_CONFIG / filename

    assert package_path.read_bytes() == bringup_path.read_bytes()
    parameters = yaml.safe_load(package_path.read_text(encoding="utf-8"))[
        "/planning/obstacle_replan_monitor"
    ]["ros__parameters"]
    assert parameters["block_hold_s"] == 20.0
    assert parameters["minimum_replan_lane_width_m"] == 2.50
    assert parameters["minimum_side_clearance_m"] == 0.60
    assert parameters["lanelet_grid_max_age_s"] == 2.5
    assert parameters["compute_path_action_name"] == "/planning/compute_path_to_pose"

    monitor_source = (
        SRC_ROOT
        / "camrod_planning"
        / "scripts"
        / "obstacle_replan_monitor_node.py"
    ).read_text(encoding="utf-8")
    runner_source = (
        SRC_ROOT / "camrod_bringup" / "scripts" / "sim_validation_runner.py"
    ).read_text(encoding="utf-8")
    assert "ComputePathToPose" in monitor_source
    assert "BLOCKED_REPLAN_FAILED_HOLD" in monitor_source
    assert "obstacle_replan_expect_safe_hold" in runner_source


def test_production_loads_only_policy_reachable_planners() -> None:
    """Keep dormant planner implementations available without constructing them."""
    for profile in ("production.yaml", "all.yaml"):
        package = yaml.safe_load(
            (PLANNING_CONFIG / "nav2_planner_profiles" / profile).read_text(
                encoding="utf-8"
            )
        )
        bringup = yaml.safe_load(
            (BRINGUP_CONFIG / "nav2_planner_profiles" / profile).read_text(
                encoding="utf-8"
            )
        )
        assert package == bringup

    production = yaml.safe_load(
        (
            PLANNING_CONFIG
            / "nav2_planner_profiles"
            / "production.yaml"
        ).read_text(encoding="utf-8")
    )
    plugins = production["planner_server"]["ros__parameters"]["planner_plugins"]
    assert plugins == ["LaneletRoute", "SmacLattice"]

    base = yaml.safe_load(
        (PLANNING_CONFIG / "nav2_base.yaml").read_text(encoding="utf-8")
    )
    definitions = base["planner_server"]["ros__parameters"]
    for planner_id in ("Smac2D", "NavFn", "ThetaStar", "SmacHybrid"):
        assert planner_id in definitions


def test_production_loads_only_policy_reachable_controllers() -> None:
    """Keep optional controller implementations without constructing them."""
    profile_dir = PLANNING_CONFIG / "nav2_controller_profiles"
    mirror_dir = BRINGUP_CONFIG / "nav2_controller_profiles"
    for profile in ("production.yaml", "all.yaml"):
        assert (profile_dir / profile).read_bytes() == (
            mirror_dir / profile
        ).read_bytes()

    production = yaml.safe_load(
        (profile_dir / "production.yaml").read_text(encoding="utf-8")
    )
    plugins = production["controller_server"]["ros__parameters"][
        "controller_plugins"
    ]
    assert plugins == ["RPP", "RotationShim"]

    definitions = {
        **_parameters(PLANNING_CONFIG / "nav2_base.yaml"),
        **_parameters(PLANNING_CONFIG / "nav2_vehicle.yaml"),
    }
    for controller_id in ("RPP", "DWB", "MPPI", "Graceful", "RotationShim"):
        assert controller_id in definitions


def test_launch_defaults_select_production_load_profiles_and_graceful_shutdown() -> None:
    """Bringup must select low-overhead profiles and avoid shutdown pkill races."""
    defaults = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    assert defaults["runtime"]["clean_before_launch"] is True
    assert defaults["runtime"]["clean_on_shutdown"] is False
    assert defaults["planning"]["nav2_planner_plugins_param_file"].endswith(
        "nav2_planner_profiles/production.yaml"
    )
    assert defaults["planning"]["nav2_controller_plugins_param_file"].endswith(
        "nav2_controller_profiles/production.yaml"
    )

    launch_text = (
        SRC_ROOT / "camrod_planning" / "launch" / "nav2_lanelet.launch.py"
    ).read_text(encoding="utf-8")
    assert "nav2_controller_plugins_param_file" in launch_text
    assert "nav2_controller_profiles', 'production.yaml" in launch_text


def test_progress_checker_releases_parent_callbacks_during_lifecycle_cleanup() -> None:
    """Prevent a node/plugin ownership cycle from surviving rcl shutdown."""
    progress_source = (
        SRC_ROOT / "camrod_planning" / "src" / "engage_aware_progress_checker.cpp"
    ).read_text(encoding="utf-8")
    controller_source = (
        SRC_ROOT
        / "camrod_planning"
        / "external"
        / "nav2_controller"
        / "src"
        / "controller_server.cpp"
    ).read_text(encoding="utf-8")

    assert "node_ = parent.lock()" not in progress_source
    assert "dyn_params_handler_.reset();" in progress_source
    cleanup = controller_source.split("ControllerServer::on_cleanup", maxsplit=1)[1]
    cleanup = cleanup.split("ControllerServer::on_shutdown", maxsplit=1)[0]
    assert "progress_checker_.reset();" in cleanup


def test_package_manual_rotation_shim_matches_the_fixed_rpp_profile() -> None:
    """Keep the package's manual and mission controllers on one fixed A/B profile."""
    base = _parameters(PLANNING_CONFIG / "nav2_base.yaml")
    vehicle = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")
    direct_rpp = vehicle["RPP"]
    manual_rpp = base["RotationShim"]

    assert manual_rpp["primary_controller"].endswith(
        "RegulatedPurePursuitController"
    )
    for key in SHARED_TRACKING_KEYS:
        assert manual_rpp[key] == direct_rpp[key], key

    # HH_260818 - This is the retained fixed side of the worak-test field A/B,
    # not the velocity-scaled profile loaded by full bringup.
    assert direct_rpp["lookahead_dist"] == 1.2
    assert manual_rpp["lookahead_dist"] == 1.2
    assert manual_rpp["min_lookahead_dist"] == 1.1
    assert manual_rpp["max_lookahead_dist"] == 2.0
    assert manual_rpp["use_velocity_scaled_lookahead_dist"] is False


def test_rpp_profile_contains_only_effective_limit_names() -> None:
    """Prevent reintroducing RPP keys ignored by the bundled implementation."""
    rpp = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")["RPP"]

    assert "max_allowed_time_to_collision_up_to_carrot" in rpp
    assert "max_allowed_time_to_collision" not in rpp
    for ignored_key in (
        "max_linear_vel",
        "min_linear_vel",
        "max_linear_accel",
        "max_angular_vel",
    ):
        assert ignored_key not in rpp


def test_package_two_kph_rpp_keeps_the_fixed_ab_preview() -> None:
    """The package side of the pending A/B must remain fixed and reproducible."""
    rpp = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")["RPP"]

    # HH_260818 - Full bringup deliberately loads the scaled counterpart; this
    # assertion protects only the fixed package candidate.
    assert rpp["use_velocity_scaled_lookahead_dist"] is False
    assert rpp["lookahead_dist"] == 1.2
    assert rpp["min_lookahead_dist"] == 1.1
    assert rpp["max_lookahead_dist"] == 2.0


def test_two_kph_operational_speed_ratios_and_safety_exception() -> None:
    """Scale service motion from 2 km/h while keeping recovery deliberately slow."""
    for filename in ("control.yaml", "parking.yaml", "yaw_alignment_zones.yaml"):
        assert (CONTROL_CONFIG / filename).read_bytes() == (
            BRINGUP_CONTROL_CONFIG / filename
        ).read_bytes()

    defaults = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    gate_scale = defaults["control"]["cmd_vel_gate_speed_scale"]
    assert gate_scale == 0.5
    gate = yaml.safe_load(
        (CONTROL_CONFIG / "cmd_vel_safety_gate.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert gate["speed_scale"] == gate_scale
    # HH_260807 - Map-v17 B2 recontact testing failed at 1.0 s and completed
    # three consecutive runs at the smallest tested passing value, 1.5 s.
    assert gate["route_safety_recovery_clear_required_s"] == 1.5

    rpp = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")["RPP"]
    campsite = _node_parameters(
        CONTROL_CONFIG / "control.yaml",
        "/control/camping_site_maneuver_controller",
    )
    drop_zone = _node_parameters(
        CONTROL_CONFIG / "control.yaml",
        "/control/drop_zone_maneuver_controller",
    )
    # HH_260810 - A single tolerance crossing must not start translation while
    # the zero-turn or drop-zone alignment is still moving.
    assert campsite["rotate_settle_hold_s"] == 0.8
    assert campsite["rotate_settle_max_rate_degps"] == 3.0
    assert drop_zone["yaw_settle_hold_s"] == 1.0
    assert drop_zone["yaw_settle_max_rate_degps"] == 3.0
    recovery = _node_parameters(
        CONTROL_CONFIG / "control.yaml",
        "/control/route_safety_recovery_controller",
    )
    reverse_parking = _node_parameters(
        CONTROL_CONFIG / "parking.yaml",
        "/parking/reverse_parking_controller",
    )
    tag_parking = _node_parameters(
        CONTROL_CONFIG / "parking.yaml",
        "/parking/apriltag_parking_controller",
    )
    yaw_zones = yaml.safe_load(
        (CONTROL_CONFIG / "yaw_alignment_zones.yaml").read_text(encoding="utf-8")
    )["yaw_alignment_zones"]["zones"]

    expected_kph = {
        "cruise": (rpp["desired_linear_vel"], 2.0),
        # HJ_0812 - Curve floor lowered from 50% to 30% of the cruise
        # reference so the widened regulation trigger has range to act.
        "curve_floor": (rpp["regulated_linear_scaling_min_speed"], 0.6),
        "final_approach": (rpp["min_approach_linear_velocity"], 0.5),
        "campsite_crab": (campsite["crab_speed_mps"], 1.2),
        # HH_260819 - B11-B13 move only 0.30 m beside the narrow road and use
        # a dedicated low-speed crab profile after the common command gate.
        "roadside_crab": (campsite["roadside_crab_speed_mps"], 0.36),
        "campsite_reverse": (campsite["reverse_entry_speed_mps"], 0.8),
        "drop_zone_exit": (drop_zone["exit_speed_mps"], 0.8),
        "reverse_parking": (reverse_parking["reverse_speed_mps"], 0.8),
        "tag_approach": (tag_parking["reverse_approach_speed_mps"], 1.0),
        "tag_insertion": (tag_parking["final_insertion_speed_mps"], 0.25),
        "yaw_zone_1": (yaw_zones[0]["max_approach_linear_x"], 1.25),
        "yaw_zone_2": (yaw_zones[1]["max_approach_linear_x"], 1.0),
    }
    for name, (raw_mps, target_kph) in expected_kph.items():
        assert math.isclose(
            _final_kph(raw_mps, gate_scale),
            target_kph,
            abs_tol=0.00001,
        ), name

    # HH_260806 - Route-boundary escape is a safety primitive, not an ordinary
    # mission state. It intentionally remains 0.18 km/h instead of scaling up.
    assert recovery["maximum_speed_mps"] == 0.10
    assert recovery["maximum_distance_m"] == 0.40
    assert recovery["maximum_duration_s"] == 10.0
    assert recovery["maximum_attempts"] == 50
    assert recovery["retry_pause_s"] == 0.5
    assert recovery["maximum_total_distance_m"] == 1.50
    assert recovery["maximum_total_duration_s"] == 90.0
    assert math.isclose(
        _final_kph(recovery["maximum_speed_mps"], gate_scale),
        0.18,
        abs_tol=0.00001,
    )


def test_fake_constant_speed_fallback_matches_two_kph_cruise() -> None:
    """Standalone and YAML simulation fallbacks must model the final command."""
    fake_config = yaml.safe_load(
        (
            SRC_ROOT / "camrod_bringup" / "config" / "sim" / "fake_sensors.yaml"
        ).read_text(encoding="utf-8")
    )["/bringup/fake_sensor_publisher"]["ros__parameters"]
    fake_launch = (
        SRC_ROOT / "camrod_bringup" / "launch" / "fake_sensors.launch.py"
    ).read_text(encoding="utf-8")
    fake_publisher = (
        SRC_ROOT / "camrod_bringup" / "scripts" / "fake_sensor_publisher.py"
    ).read_text(encoding="utf-8")

    assert math.isclose(fake_config["speed_mps"] * 3.6, 2.0, abs_tol=0.00001)
    assert "default_value='0.555556'" in fake_launch
    assert 'declare_parameter("speed_mps", 0.555556)' in fake_publisher


def test_safety_gate_evaluates_the_final_scaled_command() -> None:
    """Keep collision projection and publication on the same final command."""
    source = (
        SRC_ROOT / "camrod_control" / "src" / "cmd_vel_safety_gate_node.cpp"
    ).read_text(encoding="utf-8")

    # HH_260807 - The upstream RPP command is 1.111 m/s while the deployed
    # speed scale publishes 0.556 m/s. Safety evidence must use the latter.
    assert "const auto evaluated_command = scaleCommand(command);" in source
    assert "motion_cost_stop_.evaluate(evaluated_command, now_sec)" in source
    assert "cost_decision, evaluated_command, now_sec" in source
    assert "publishCommand(evaluated_command);" in source


def test_classified_fusion_is_a_direct_two_meter_stop_source() -> None:
    """Classified fusion stops directly without enabling raw LiDAR cost."""
    gate = yaml.safe_load(
        (CONTROL_CONFIG / "cmd_vel_safety_gate.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    source = (
        SRC_ROOT / "camrod_control" / "src" / "cmd_vel_safety_gate_node.cpp"
    ).read_text(encoding="utf-8")

    assert gate["cost_source_debug_topics"] == [
        "/map/cost_grid/lanelet_safety",
        "/sensing/cost_grid/radar",
        "/sensing/cost_grid/lidar",
        "/planning/cost_grid/global_path",
    ]
    assert gate["cost_source_debug_labels"] == [
        "lanelet",
        "radar",
        "fusion",
        "global_path",
    ]
    assert gate["cost_stop_dynamic_source_labels"] == "radar,fusion"
    assert gate["cost_stop_classified_source_labels"] == "fusion"
    assert gate["cost_stop_classified_front_lookahead_m"] == 2.0
    assert '"cost_stop_dynamic_source_labels", "radar,fusion"' in source


def test_campsite_return_uses_axis_staged_route_goal_anchor() -> None:
    """Crab laterally to the centerline snap before straight drift correction."""
    source = (
        SRC_ROOT
        / "camrod_control"
        / "src"
        / "camping_site_maneuver_controller_node.cpp"
    ).read_text(encoding="utf-8")
    campsite = _node_parameters(
        CONTROL_CONFIG / "control.yaml",
        "/control/camping_site_maneuver_controller",
    )

    # HH_260807 - GOAL_REACHED permits a small position error. Automatic
    # service must remove that error before handing control back to Nav2.
    assert 'return_anchor_source_ = "route_goal_snap";' in source
    assert "captureReturnAnchor(" in source
    assert "bodyAxisPrioritizedTranslationTowardTarget(" in source
    assert re.search(
        r"return_anchor_x_,\s*return_anchor_y_,\s*activeCrabSpeedMps\(\),"
        r"\s*return_translation_gain_,\s*return_position_tolerance_m_",
        source,
    )
    assert campsite["roadside_max_lateral_offset_m"] == 0.30
    assert "bodyTranslationTowardTarget(" not in source
    assert campsite["return_position_tolerance_m"] == 0.04
    assert campsite["return_translation_kp"] == 4.0


def test_campsite_occupancy_guard_is_one_opt_in_policy() -> None:
    """UI admission and maneuver start must use one disabled-by-default flag."""
    defaults = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_bringup"
            / "config"
            / "bringup"
            / "launch_defaults.yaml"
        ).read_text(encoding="utf-8")
    )["bringup"]
    package = _node_parameters(
        CONTROL_CONFIG / "control.yaml",
        "/control/camping_site_maneuver_controller",
    )
    deployed = _node_parameters(
        BRINGUP_CONTROL_CONFIG / "control.yaml",
        "/control/camping_site_maneuver_controller",
    )

    # HH_260818 - A normal destination can contain the guest's own tent, so
    # occupancy remains opt-in while retaining one switch for both consumers.
    assert defaults["control"]["enable_campsite_occupancy_guard"] is False
    assert package["enable_campsite_occupancy_guard"] is False
    assert deployed["enable_campsite_occupancy_guard"] is False
    bringup_source = (
        SRC_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
    ).read_text(encoding="utf-8")
    maneuver_source = (
        SRC_ROOT / "camrod_control" / "launch" / "maneuvers.launch.py"
    ).read_text(encoding="utf-8")
    ui_source = (
        SRC_ROOT / "camrod_ui" / "camrod_ui_robot" / "launch" / "ui.launch.py"
    ).read_text(encoding="utf-8")
    for source in (bringup_source, maneuver_source, ui_source):
        assert "enable_campsite_occupancy_guard" in source


def test_ranger_mode_change_settles_before_campsite_translation() -> None:
    """Keep the deployed 20% Ackermann floor out of DA/parallel transitions."""
    bringup = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_bringup"
            / "config"
            / "platform"
            / "ranger_driver.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    package = yaml.safe_load(
        (
            SRC_ROOT / "camrod_platform" / "config" / "ranger_driver.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    assert bringup["steering_transition_rate_radps"] == 1.5
    assert bringup["steering_transition_min_velocity_scale"] == 0.2
    for profile in (bringup, package):
        assert profile["steering_mode_transition_stationary_enabled"] is True
        assert profile["steering_mode_transition_ready_error_rad"] == 0.05

    driver_source = (
        SRC_ROOT
        / "camrod_platform"
        / "external"
        / "ranger_ros2"
        / "ranger_base"
        / "src"
        / "ranger_messenger.cpp"
    ).read_text(encoding="utf-8")
    assert "steering_command_initialized_ = false;" in driver_source
    assert "translational_mode_initialized_ = false;" in driver_source


def test_gross_start_alignment_is_separate_from_continuous_curve_tracking() -> None:
    """Finish a gross start turn without making normal RPP curves stop-turn."""
    vehicle = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")
    base = _parameters(PLANNING_CONFIG / "nav2_base.yaml")
    gate = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_control"
            / "config"
            / "cmd_vel_safety_gate.yaml"
        ).read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]

    # HH_260806 - RPP's rotate-to-path check evaluates every carrot. A 2-degree
    # threshold produced 403 rotation/translation switches in one B7 route.
    assert vehicle["RPP"]["use_rotate_to_heading"] is False
    assert base["RotationShim"]["use_rotate_to_heading"] is False
    assert base["RotationShim"]["angular_dist_threshold"] == 0.785398
    assert base["RotationShim"]["angular_disengage_threshold"] == 0.0872665
    assert gate["enable_route_heading_alignment"] is True
    assert gate["route_heading_error_enter_deg"] == 75.0
    assert gate["route_heading_error_exit_deg"] == 5.0
    assert gate["route_heading_max_linear_x"] == 0.0


def test_sim_runner_locks_map_fixed_obstacle_and_repeated_service_contract() -> None:
    """Keep the release soak broader than a one-way campsite smoke test."""
    runner = (
        SRC_ROOT / "camrod_bringup" / "scripts" / "sim_validation_runner.py"
    ).read_text(encoding="utf-8")
    fake_source = (
        SRC_ROOT / "camrod_bringup" / "scripts" / "fake_sensor_publisher.py"
    ).read_text(encoding="utf-8")
    fake_config = yaml.safe_load(
        (
            SRC_ROOT
            / "camrod_bringup"
            / "config"
            / "sim"
            / "fake_sensors.yaml"
        ).read_text(encoding="utf-8")
    )["/bringup/fake_sensor_publisher"]["ros__parameters"]

    # HH_260807 - The obstacle must remain in the map while the robot moves;
    # otherwise a supposed avoidance test only follows a robot-relative target.
    assert fake_config["obstacle_reference_frame"] == "robot"
    assert fake_config["obstacle_world_x"] == 0.0
    assert fake_config["obstacle_world_y"] == 0.0
    assert 'if self.obstacle_reference_frame == "map":' in fake_source
    assert 'obstacle_reference_frame="map"' in runner
    assert "Keep the base LiDAR/perception health stream alive" in runner

    # HH_260807 - A release soak must exercise at least two destinations in one
    # process and issue RETURN only after the public unload-wait state appears.
    assert 'self.declare_parameter("run_service_soak", False)' in runner
    assert 'self.declare_parameter("expect_lidar_cost_grid", True)' in runner
    assert '"camping_site_1", "camping_site_2", "camping_site_3"' in runner
    assert "service_soak_mission_keys requires at least two sites" in runner
    assert '"WAITING_FOR_RETURN_REQUEST" in service_events' in runner
    assert "MotionOperation.RETURN" in runner
    assert '"bringup_restart_count": 0' in runner
    assert '"/control/route_safety_recovery_controller/status"' in runner
    assert '"boundary_recovery_motion_seen"' in runner
    assert '"boundary_recovery_released"' in runner
    assert '"boundary_retry_latched"' in runner
    # HH_260807 - The raw BMS simulator and platform bridge must be the only
    # status path. A second normalized publisher oscillated charging state.
    assert 'AvgPlatformStatus, "/platform/status", self._on_platform_status' in runner
    assert "self.create_publisher(\n            AvgPlatformStatus" not in runner
    assert "_publish_fake_platform_status" not in runner
    assert 'p.name == "simulated_battery_percentage"' in fake_source
    assert "ranger_platform_bridge remains the single owner" in runner
