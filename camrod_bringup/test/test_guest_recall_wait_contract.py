"""Cross-package contract for occupied-site guest recall and return."""

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
CONTROL_SOURCE = (
    ROOT / "camrod_control" / "src" / "camping_site_maneuver_controller_node.cpp"
)
CONTROL_CONFIG = ROOT / "camrod_control" / "config" / "control.yaml"
BRINGUP_CONTROL_CONFIG = (
    ROOT / "camrod_bringup" / "config" / "control" / "control.yaml"
)
PLANNING_SOURCE = (
    ROOT / "camrod_planning" / "scripts" / "planning_state_machine_node.py"
)
CAMPSITES = ROOT / "camrod_planning" / "config" / "camping_sites.yaml"


def _controller_params(path: Path) -> dict:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    return document["/control/camping_site_maneuver_controller"]["ros__parameters"]


def test_recall_wait_offset_is_explicit_and_deployed_mirror_matches() -> None:
    package = _controller_params(CONTROL_CONFIG)
    bringup = _controller_params(BRINGUP_CONTROL_CONFIG)
    assert package["recall_wait_lateral_offset_m"] == 0.30
    assert bringup["recall_wait_lateral_offset_m"] == 0.30
    assert package == bringup


def test_all_sites_use_authored_geometry_instead_of_guessed_recall_points() -> None:
    sites = yaml.safe_load(CAMPSITES.read_text(encoding="utf-8"))["camping_sites"]
    assert [site["type"] for site in sites] == [
        f"camping_site_{index}" for index in range(1, 14)
    ]
    assert all(len(site["corners"]) >= 3 for site in sites)
    assert all("recall_x" not in site and "recall_y" not in site for site in sites)

    planning = PLANNING_SOURCE.read_text(encoding="utf-8")
    assert "normalized = site_name.strip()" in planning
    assert "return normalized" in planning
    assert "return road_key" not in planning
    assert "_goal_update_belongs_to_active_recall" in planning
    assert "regulated_recall_goal" in planning


def test_recall_uses_roadside_motion_but_normal_delivery_still_blocks_occupancy() -> None:
    source = CONTROL_SOURCE.read_text(encoding="utf-8")
    normalized_source = " ".join(source.split())
    assert (
        "message.scenario_id == "
        "avg_msgs::msg::PlanningScenario::RECALL_TO_SITE"
    ) in normalized_source
    assert (
        "return !active_recall_wait_mission_ && isSiteOccupied(mission_key);"
    ) in source
    assert (
        "active_recall_wait_mission_ ? CampsiteServiceMode::kRoadsideStop"
    ) in normalized_source
    assert "std::min(bounded_offset, activeRoadsideMaximumOffsetM())" in source
    assert "AvgServiceState::RECALL_TO_SITE_ROAD" in source
    assert "AvgServiceState::GUEST_LOADING_WAIT" in source
    assert 'publishReturnRequest("done_roadside_forward' in source
