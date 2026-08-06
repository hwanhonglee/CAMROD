"""Keep manual and UI Nav2 path-tracking behavior synchronized."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
PLANNING_CONFIG = SRC_ROOT / "camrod_planning" / "config"
BRINGUP_CONFIG = SRC_ROOT / "camrod_bringup" / "config" / "planning"

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
    "rotate_to_heading_min_angle",
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


def test_package_and_bringup_nav2_profiles_are_byte_synchronized() -> None:
    """Deployment mirrors must not silently retain old controller tuning."""
    for filename in ("nav2_base.yaml", "nav2_vehicle.yaml"):
        assert (PLANNING_CONFIG / filename).read_bytes() == (
            BRINGUP_CONFIG / filename
        ).read_bytes()


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


def test_manual_rotation_shim_uses_the_ui_rpp_tracking_profile() -> None:
    """Only manual yaw handling may differ; path tracking must be identical."""
    base = _parameters(PLANNING_CONFIG / "nav2_base.yaml")
    vehicle = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")
    direct_rpp = vehicle["RPP"]
    manual_rpp = base["RotationShim"]

    assert manual_rpp["primary_controller"].endswith(
        "RegulatedPurePursuitController"
    )
    for key in SHARED_TRACKING_KEYS:
        assert manual_rpp[key] == direct_rpp[key], key


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


def test_low_speed_rpp_keeps_damped_preview_floor() -> None:
    """The 0.20 m/s field limit must not collapse the swept 1.1 m preview."""
    rpp = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")["RPP"]

    assert rpp["use_velocity_scaled_lookahead_dist"] is True
    assert rpp["lookahead_dist"] == 1.1
    assert rpp["min_lookahead_dist"] == 1.1
    assert rpp["max_lookahead_dist"] >= rpp["min_lookahead_dist"]


def test_nav2_rotation_finishes_before_translation() -> None:
    """Initial yaw alignment must not emit forward motion at a large yaw error."""
    vehicle = _parameters(PLANNING_CONFIG / "nav2_vehicle.yaml")
    base = _parameters(PLANNING_CONFIG / "nav2_base.yaml")
    threshold = 0.0349066

    assert vehicle["RPP"]["use_rotate_to_heading"] is True
    assert vehicle["RPP"]["rotate_to_heading_min_angle"] == threshold
    assert base["RotationShim"]["use_rotate_to_heading"] is True
    assert base["RotationShim"]["rotate_to_heading_min_angle"] == threshold
    assert base["RotationShim"]["angular_dist_threshold"] == threshold
