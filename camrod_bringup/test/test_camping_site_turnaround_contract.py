"""Lock the active drive-in and constrained-roadside campsite policy."""

import json
from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_CONFIG = SRC_ROOT / "camrod_planning" / "config" / "camping_sites.yaml"
BRINGUP_CONFIG = (
    SRC_ROOT / "camrod_bringup" / "config" / "planning" / "camping_sites.yaml"
)
PACKAGE_CONTROL_CONFIG = SRC_ROOT / "camrod_control" / "config" / "control.yaml"
BRINGUP_CONTROL_CONFIG = (
    SRC_ROOT / "camrod_bringup" / "config" / "control" / "control.yaml"
)
LAUNCH_DEFAULTS = (
    SRC_ROOT / "camrod_bringup" / "config" / "bringup" / "launch_defaults.yaml"
)
MANEUVERS_LAUNCH = SRC_ROOT / "camrod_control" / "launch" / "maneuvers.launch.py"
BRINGUP_LAUNCH = SRC_ROOT / "camrod_bringup" / "launch" / "_bringup_impl.py"
CONTROLLER_SOURCE = (
    SRC_ROOT
    / "camrod_control"
    / "src"
    / "camping_site_maneuver_controller_node.cpp"
)
EVIDENCE_SUMMARY = (
    SRC_ROOT
    / "docs"
    / "assets"
    / "module-guides"
    / "bringup"
    / "test-results"
    / "camping-site-full-return-20260819"
    / "campsite-policy-summary.json"
)


def _normalized_source(source: str) -> str:
    """Make source contracts independent of clang-format line wrapping."""
    return " ".join(source.split())


def _sites(path: Path) -> list[dict]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["camping_sites"]


def _controller_params(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))[
        "/control/camping_site_maneuver_controller"
    ]["ros__parameters"]


def test_active_campsite_config_is_byte_synchronized() -> None:
    """Standalone planning and full bringup must select the same maneuver policy."""
    expected = PACKAGE_CONFIG.read_bytes()
    assert BRINGUP_CONFIG.read_bytes() == expected
    # HH_260810 - User-maintained named copies are historical references, not
    # runtime mirrors. Only the two active paths form the deployment contract.


def test_drive_in_sites_turn_and_constrained_sites_default_to_forward_loop() -> None:
    """Production keeps B1-B10 turnaround and B11-B13 roadside semantics."""
    modes = {
        site["type"]: str(site.get("service_mode", "turnaround")).strip().lower()
        for site in _sites(PACKAGE_CONFIG)
    }

    assert modes
    assert len(modes) == 13
    for index in range(1, 11):
        assert modes[f"camping_site_{index}"] == "turnaround"
    for index in range(11, 14):
        assert modes[f"camping_site_{index}"] == "roadside_stop"


def test_roadside_reverse_override_is_fail_closed_in_production() -> None:
    """Ordinary CAMROD must preserve the accepted roadside forward-loop policy."""
    package_params = _controller_params(PACKAGE_CONTROL_CONFIG)
    bringup_params = _controller_params(BRINGUP_CONTROL_CONFIG)
    launch_defaults = yaml.safe_load(
        LAUNCH_DEFAULTS.read_text(encoding="utf-8")
    )["bringup"]
    maneuver_source = MANEUVERS_LAUNCH.read_text(encoding="utf-8")
    controller_source = CONTROLLER_SOURCE.read_text(encoding="utf-8")

    assert package_params["roadside_reverse_return_enable"] is False
    assert bringup_params["roadside_reverse_return_enable"] is False
    assert package_params["roadside_reverse_handoff_distance_m"] == 0.03
    assert bringup_params["roadside_reverse_handoff_distance_m"] == 0.03
    assert package_params == bringup_params
    assert (
        launch_defaults["control"][
            "camping_site_roadside_reverse_return_enable"
        ]
        is False
    )
    assert (
        launch_defaults["control"][
            "camping_site_roadside_reverse_handoff_distance_m"
        ]
        == 0.03
    )
    assert (
        '"roadside_reverse_return_enable", default_value="false"'
        in maneuver_source
    )
    assert (
        'declare_parameter<bool>("roadside_reverse_return_enable", false)'
        in _normalized_source(controller_source)
    )
    assert (
        "return active_service_mode_ == CampsiteServiceMode::kRoadsideStop &&\n"
        "           !roadside_reverse_return_enable_;"
        in controller_source
    )


def test_roadside_reverse_override_is_forwarded_without_changing_site_mode() -> None:
    """Bringup may opt in, but only through the typed controller parameter."""
    package_params = _controller_params(PACKAGE_CONTROL_CONFIG)
    maneuver_source = MANEUVERS_LAUNCH.read_text(encoding="utf-8")
    bringup_source = BRINGUP_LAUNCH.read_text(encoding="utf-8")
    controller_source = CONTROLLER_SOURCE.read_text(encoding="utf-8")

    assert (
        '"roadside_reverse_return_enable": ParameterValue(\n'
        in maneuver_source
    )
    assert (
        'LaunchConfiguration(\n'
        '                            "roadside_reverse_return_enable"\n'
        "                        ),\n"
        "                        value_type=bool,"
        in maneuver_source
    )
    assert (
        '"roadside_reverse_handoff_distance_m": ParameterValue('
        in maneuver_source
    )
    assert (
        "'control_camping_site_roadside_reverse_handoff_distance_m',"
        in bringup_source
    )
    assert (
        "'roadside_reverse_handoff_distance_m': lc[\n"
        "            'control_camping_site_roadside_reverse_handoff_distance_m'\n"
        "        ]"
        in bringup_source
    )
    assert (
        "'control_camping_site_roadside_reverse_return_enable',"
        in bringup_source
    )
    assert (
        "'roadside_reverse_return_enable': lc[\n"
        "            'control_camping_site_roadside_reverse_return_enable'\n"
        "        ]"
        in bringup_source
    )

    # Roadside geometry, restart/adoption, and arrival all preserve the entry
    # heading. Only the post-CRAB_OUT handoff chooses production forward-loop
    # versus simulator reverse tracking without a turnaround.
    assert "bool roadsideForwardLoopActive() const" in controller_source
    assert "bool roadsideReverseReturnActive() const" in controller_source
    assert (
        "start_yaw_ = active_service_mode_ == "
        "CampsiteServiceMode::kRoadsideStop"
        in controller_source
    )
    assert (
        "entry_target_yaw_ =\n"
        "        active_service_mode_ == CampsiteServiceMode::kRoadsideStop"
        in controller_source
    )
    assert (
        "if (active_service_mode_ == CampsiteServiceMode::kRoadsideStop) {\n"
        "      // HH_260830 - Always leave B11-B13 at the original heading."
        in controller_source
    )
    assert controller_source.count("if (roadsideForwardLoopActive())") >= 2
    assert controller_source.count(
        "else if (roadsideReverseReturnActive())"
    ) >= 2
    assert (
        "setPhase(CampingSiteManeuverPhase::kAlignReturnRouteYaw"
        not in controller_source
    )
    assert controller_source.count(
        "CampingSiteManeuverPhase::kAlignOutboundLaneYaw"
    ) >= 5
    assert "target_yaw_ = start_yaw_;" in controller_source
    assert "outbound_lane_before_reverse" in controller_source
    assert "start_yaw_ + M_PI" not in controller_source[
        controller_source.index("void finishReturnAtLiveLaneletHandoff()") :
        controller_source.index("void setPhase(")
    ]
    assert "reverse route requested " in controller_source
    assert '"without turnaround"' in controller_source
    assert "reverse return " in controller_source
    assert '"deferred until lanelet exit"' in controller_source
    assert (
        "roadsideReverseReturnActive()\n"
        "            ? std::min(return_lanelet_handoff_distance_m_,\n"
        "                       roadside_reverse_handoff_distance_m_)"
        in controller_source
    )
    assert package_params["roadside_reverse_handoff_distance_m"] == 0.03
    assert (
        'publishReturnRequest("done_roadside_forward_retry")'
        not in controller_source
    )
    assert (
        'roadsideReverseReturnActive() ? "done_roadside_reverse_retry" '
        ': "done_retry"'
        in _normalized_source(controller_source)
    )


def test_roadside_sites_use_map_centers_without_legacy_service_pose() -> None:
    """A copied B12 service pose must not redirect B11-B13 into unsafe terrain."""
    sites = {site["type"]: site for site in _sites(PACKAGE_CONFIG)}
    for index in range(11, 14):
        site = sites[f"camping_site_{index}"]
        assert not any(key.startswith("service_") and key != "service_mode" for key in site)


def test_checked_in_runtime_evidence_covers_all_thirteen_sites() -> None:
    """Every active site must have direct ordered runtime evidence."""
    summary = json.loads(EVIDENCE_SUMMARY.read_text(encoding="utf-8"))
    runtime = summary["runtime"]
    assert summary["config_contract_pass"] is True
    assert set(runtime) == {f"B{index}" for index in range(1, 14)}

    turnaround_sequence = [
        "CRAB_IN",
        "ROTATE_180",
        "UNLOAD_WAIT",
        "WAIT_RETURN",
        "ALIGN_RETRACE_YAW",
        "CRAB_OUT",
        "DONE",
    ]
    for index in range(1, 11):
        result = runtime[f"B{index}"]
        assert result["pass"] is True
        assert result["service_mode"] == "turnaround"
        assert result["arrival_only"] is False
        assert result["phase_sequence"] == turnaround_sequence
        assert result["rotate_180"] is True
        assert float(result["site_distance_m"]) > 0.0

    roadside_sequence = [
        "CRAB_IN",
        "UNLOAD_WAIT",
        "WAIT_RETURN",
        "CRAB_OUT",
        "DONE",
    ]
    for index in range(11, 14):
        result = runtime[f"B{index}"]
        assert result["pass"] is True
        assert result["service_mode"] == "roadside_stop"
        assert result["arrival_only"] is False
        assert result["phase_sequence"] == roadside_sequence
        assert result["rotate_180"] is False
        assert float(result["max_final_cmd"]) <= 0.10

    # HH_260819 - The constrained sites exit laterally without an on-lane
    # zero-turn, then select the legal forward loop from the Return source.
    assert summary["policy"]["roadside_max_lateral_offset_m"] == 0.30
    assert (
        summary["policy"]["roadside_return_policy"]
        == "forward_one_way_after_crab_out"
    )
