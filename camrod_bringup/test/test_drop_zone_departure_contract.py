"""Lock UI -> drop-zone -> charging gate -> road handoff sequencing."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
CONTROL = SRC_ROOT / "camrod_control"
BRINGUP = SRC_ROOT / "camrod_bringup"
UI_NODE = SRC_ROOT / "camrod_ui/runtime/python/camrod_ui/ui_backend_node.py"
DROP_ZONE_NODE = CONTROL / "src/drop_zone_maneuver_controller_node.cpp"
GATE_NODE = CONTROL / "src/cmd_vel_safety_gate_node.cpp"


def _parameters(path: Path, node: str = "/**") -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))[node][
        "ros__parameters"
    ]


def test_exit_target_and_gate_config_mirrors_are_exact() -> None:
    """Deployment and package copies carry one production safety policy."""
    control_package = CONTROL / "config/control.yaml"
    control_deployment = BRINGUP / "config/control/control.yaml"
    gate_package = CONTROL / "config/cmd_vel_safety_gate.yaml"
    gate_deployment = BRINGUP / "config/control/cmd_vel_safety_gate.yaml"
    assert control_package.read_bytes() == control_deployment.read_bytes()
    assert gate_package.read_bytes() == gate_deployment.read_bytes()

    drop_zone = _parameters(
        control_package, "/control/drop_zone_maneuver_controller"
    )
    assert drop_zone["require_lanelet_exit_target"] is True
    assert drop_zone["allow_fixed_distance_fallback"] is False
    assert drop_zone["exit_target_min_distance_m"] == 0.3
    assert drop_zone["exit_target_max_distance_m"] == 8.0
    assert drop_zone["exit_target_min_forward_m"] == 0.1
    assert drop_zone["exit_target_max_lateral_m"] == 1.0
    assert drop_zone["exit_position_tolerance_m"] == 0.20
    assert drop_zone["allow_unstamped_pose_fallback"] is False

    gate = _parameters(gate_package)
    assert gate["drop_zone_maneuver_controller_status_topic"] == (
        drop_zone["status_topic"]
    )
    assert gate["allow_drop_zone_departure_while_charging"] is True
    assert gate["drop_zone_departure_status_timeout_s"] == 2.0
    assert gate["drop_zone_departure_require_source_stamp"] is True
    assert gate["drop_zone_departure_expected_module_name"] == "control"


def test_fresh_exact_controller_status_is_the_only_status_authorization() -> None:
    """Text fallbacks may own commands but cannot release charging."""
    gate = GATE_NODE.read_text(encoding="utf-8")
    callback = gate.split("drop_zone_status_subscription_ =", 1)[1].split(
        "campsite_status_subscription_ =", 1
    )[0]
    assert "drop_zone_phase_ = message->operating_state.empty()" in callback
    assert "message->module_name, message->operating_state" in callback
    assert "message->level == avg_msgs::msg::ModuleState::OK" in callback
    assert "source_stamp_sec" in callback
    assert 'onAuthorizationChanged("drop_zone_status")' in callback

    helper = (
        CONTROL
        / "include/camrod_control/drop_zone_charging_departure_authorization.hpp"
    ).read_text(encoding="utf-8")
    assert 'phase == "EXIT_STRAIGHT" || phase == "ALIGN_EXIT_YAW"' in helper
    assert "module_name != config_.expected_module_name" in helper
    assert "config_.require_source_stamp && !source_stamp_sec.has_value()" in helper
    assert "receipt_age > config_.heartbeat_timeout_s" in helper
    assert "source_age > config_.heartbeat_timeout_s" in helper

    active = gate.split(
        "bool dropZoneStatusChargingDepartureActive", 1
    )[1].split("bool chargingMotionOverrideActive", 1)[0]
    assert "charging_mission_override_.charging()" in active
    assert "charging_mission_override_.batteryReadyForDeparture()" in active


def test_ui_never_releases_campsite_before_exit_complete() -> None:
    """Restart heartbeats attach to the active exit without restarting it."""
    ui = UI_NODE.read_text(encoding="utf-8")
    departure = ui.split("departure_required =", 1)[1].split(
        "return {", 1
    )[0]
    for state in (
        "DROP_ZONE_PARKING",
        "WAITING_FOR_CHARGING",
        "DEPARTING_CHARGER",
        "DEPARTING_DROP_ZONE",
    ):
        assert f"AvgServiceState.{state}" in departure
    assert "resumed_active_departure" in departure
    assert "if not resumed_active_departure:" in departure
    assert "_publish_site_mission_key" not in departure
    assert "_publish_site_goal_pose" not in departure

    completion = ui.split("def _on_drop_zone_exit_complete", 1)[1].split(
        "def _on_service_state", 1
    )[0]
    assert completion.index("if not bool(msg.data):") < completion.index(
        "self._publish_site_mission_key"
    )
    assert completion.index("self._publish_site_mission_key") < completion.index(
        "self._publish_site_goal_pose"
    )

    assert "service_heartbeat_qos = QoSProfile" in ui
    assert "durability=DurabilityPolicy.VOLATILE" in ui
    assert "if not state_changed:" in ui


def test_drop_zone_heartbeat_and_pose_provenance_survive_restart() -> None:
    """Active service state repeats and source timestamps cannot be replayed."""
    node = DROP_ZONE_NODE.read_text(encoding="utf-8")
    status = node.split("void publishStatus", 1)[1].split(
        "std::string command_topic_", 1
    )[0]
    assert 'publishServiceState("heartbeat")' in status
    assert "if (!force)" in status
    assert '? "ROAD_HANDOFF_READY"' in status
    assert 'makeModuleState(\n        *this, "control", module_level, message, operating_state)' in status
    assert "_on_drop_zone_maneuver_status" in UI_NODE.read_text(encoding="utf-8")

    freshness = node.split("bool vehiclePoseIsFresh() const", 1)[1].split(
        "startParkingAlignment", 1
    )[0]
    assert "last_vehicle_pose_->header.stamp" in freshness
    assert "dropZoneExitPoseSampleIsFresh" in freshness
    assert "allow_unstamped_pose_fallback_" in freshness

    snapper = (
        SRC_ROOT / "camrod_planning/src/centerline_snapper_node.cpp"
    ).read_text(encoding="utf-8")
    assert "out_pose.header = msg->header;" in snapper
