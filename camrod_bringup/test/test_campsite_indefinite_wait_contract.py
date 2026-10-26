"""Guest use is operator-completed, not an automatic return countdown."""

from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def _parameters(relative_path: str, node_name: str) -> dict:
    document = yaml.safe_load((ROOT / relative_path).read_text(encoding="utf-8"))
    return document[node_name]["ros__parameters"]


# HH_261024 - Both default and normal simulation deployment profiles must
# remain button-only; a finite legacy dwell value is inactive with this flag.
@pytest.mark.parametrize(
    "relative_path",
    [
        "camrod_planning/config/planning_state_machine.yaml",
        "camrod_planning/config/planning_state_machine_sim.yaml",
        "camrod_bringup/config/planning/planning_state_machine.yaml",
        "camrod_bringup/config/planning/planning_state_machine_sim.yaml",
    ],
)
def test_default_profiles_never_auto_return_after_guest_usage(relative_path: str) -> None:
    params = _parameters(relative_path, "/planning/planning_state_machine")
    assert params["enable_auto_return_on_site_goal"] is False


def test_unloading_minimum_hold_is_not_a_service_usage_deadline() -> None:
    package = _parameters(
        "camrod_control/config/control.yaml", "/control/camping_site_maneuver_controller"
    )
    deployed = _parameters(
        "camrod_bringup/config/control/control.yaml",
        "/control/camping_site_maneuver_controller",
    )
    assert package == deployed
    assert package["auto_return_after_unload_wait"] is False
    assert package["unload_wait_s"] == 5.0
    # HH_261024 - Do not confuse stationary guest waiting with motion failure.
    # Keeping these finite leaves a stuck crab or lost localization fail-closed.
    assert package["crab_return_timeout_s"] > 0.0
    assert package["entry_yaw_alignment_timeout_s"] > 0.0


@pytest.mark.parametrize(
    ("profile", "seconds"), [("auto_return_fast", 15.0), ("auto_return_10min", 600.0)]
)
def test_explicit_auto_return_test_profiles_remain_separate(profile: str, seconds: float) -> None:
    # HH_261024 - Preserve deliberately selected automated test profiles;
    # never silently enable them in the ordinary deployed guest-use profile.
    for prefix in ("camrod_planning/config", "camrod_bringup/config/planning"):
        params = _parameters(
            f"{prefix}/planning_state_machine_{profile}.yaml",
            "/planning/planning_state_machine",
        )
        assert params["enable_auto_return_on_site_goal"] is True
        assert params["goal_reached_dwell_s"] == seconds
