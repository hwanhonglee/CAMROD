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
