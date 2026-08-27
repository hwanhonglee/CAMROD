"""Lock the camera-range AprilTag docking stop and charging handoff contract."""

from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
CONTROL = SRC_ROOT / "camrod_control"
BRINGUP = SRC_ROOT / "camrod_bringup"
NODE = CONTROL / "src/apriltag_parking_controller_node.cpp"


def _parameters(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))[
        "/parking/apriltag_parking_controller"
    ]["ros__parameters"]


def test_camera_range_thresholds_are_exact() -> None:
    """The runtime threshold must use the same camera norm shown by the UI."""
    package = CONTROL / "config/parking.yaml"
    parameters = _parameters(package)
    assert parameters["slowdown_start_tag_distance_m"] == 0.80
    assert parameters["translation_stop_tag_distance_m"] == 0.40
    assert parameters["minimum_approach_turn_radius_m"] == 0.85
    assert parameters["final_yaw_angular_speed_radps"] == 0.20
    assert parameters["final_yaw_settle_hold_s"] == 0.8
    assert parameters["final_yaw_settle_max_rate_degps"] == 3.0
    assert parameters["tag_timeout_s"] == 0.5
    assert parameters["tag_wait_timeout_s"] == 60.0
    assert parameters["odometry_timeout_s"] == 0.5


def test_tag_reacquisition_wait_is_one_minute_with_immediate_safe_stop() -> None:
    """A stale tag stops quickly but remains recoverable for one minute."""
    for config in (
        CONTROL / "config/parking.yaml",
        BRINGUP / "config/control/parking.yaml",
    ):
        parameters = _parameters(config)
        assert parameters["tag_timeout_s"] == 0.5
        assert parameters["tag_wait_timeout_s"] == 60.0

    source = NODE.read_text(encoding="utf-8")
    assert 'declare_parameter<double>("tag_timeout_s", 0.5)' in source
    assert 'declare_parameter<double>("tag_wait_timeout_s", 60.0)' in source

    waiting_block = source.split("case State::WAITING_FOR_TAG:", 1)[1].split(
        "case State::TAG_GUIDED_REVERSE:", 1
    )[0]
    assert "stateElapsed() > tag_wait_timeout_s_" in waiting_block
    assert "publishStop();" in waiting_block
    assert "fail();" in waiting_block


def test_parking_parameter_mirrors_are_exact() -> None:
    """Full bringup mirrors the package-owned runtime file byte for byte."""
    package = CONTROL / "config/parking.yaml"
    deployment = BRINGUP / "config/control/parking.yaml"
    assert package.read_bytes() == deployment.read_bytes()


def test_translation_latches_off_before_yaw_and_charging_wait() -> None:
    """No post-0.40 m phase may restore blind insertion or retry motion."""
    source = NODE.read_text(encoding="utf-8")
    assert "tag_camera_distance_m_ <= translation_stop_tag_distance_m_" in source
    assert "transitionTo(State::FINAL_YAW_ALIGNMENT)" in source
    assert 'translation_stop_reason_ = "tag_range"' in source
    assert "translation_stop_trigger_tag_distance_m_ = tag_camera_distance_m_" in source
    assert "case State::FINAL_REVERSE_INSERTION" not in source
    assert "startFinalReverseInsertion" not in source

    yaw_block = source.split("case State::FINAL_YAW_ALIGNMENT:", 1)[1].split(
        "case State::WAITING_FOR_CHARGING:", 1
    )[0]
    assert "command.linear" not in yaw_block
    assert "calculateLateralCorrectionHeading" not in yaw_block
    assert "transitionTo(State::WAITING_FOR_CHARGING)" in yaw_block
    assert "heading_gain_ * heading_error_rad_" in yaw_block
    assert yaw_block.index("odometry_is_fresh") < yaw_block.index("command.angular.z")
    stale_guard = yaw_block.split("if (!odometry_is_fresh)", 1)[1].split(
        "const bool settled", 1
    )[0]
    assert "break;" in stale_guard
    assert "command.angular" not in stale_guard

    waiting_block = source.split("case State::WAITING_FOR_CHARGING:", 1)[1].split(
        "}", 1
    )[0]
    assert "command.linear" not in waiting_block
    assert "command.angular" not in waiting_block


def test_fresh_odometry_gates_tag_capture_and_every_motion_entry() -> None:
    """No tag-guided reverse or yaw command may use a stale vehicle pose."""
    source = NODE.read_text(encoding="utf-8")
    helper = source.split(
        "bool odometryIsFresh(const rclcpp::Time & current_time) const", 1
    )[1].split("bool startParking", 1)[0]
    assert "last_odometry_time_.nanoseconds() <= 0" in helper
    assert "age_s >= 0.0 && age_s <= odometry_timeout_s_" in helper

    tag_callback = source.split("void tagCallback", 1)[1].split(
        "void updateParkingErrorsFromOdometry", 1
    )[0]
    assert "if (!odometryIsFresh(observation_time))" in tag_callback
    assert tag_callback.index("if (!odometryIsFresh(observation_time))") < (
        tag_callback.index("axis_valid_ = true")
    )

    start_block = source.split("bool startParking", 1)[1].split(
        "void cancelParking", 1
    )[0]
    assert "if (!odometryIsFresh(now()))" in start_block

    waiting_block = source.split("case State::WAITING_FOR_TAG:", 1)[1].split(
        "case State::TAG_GUIDED_REVERSE:", 1
    )[0]
    assert waiting_block.index("if (!odometry_is_fresh)") < waiting_block.index(
        "transitionTo(State::TAG_GUIDED_REVERSE)"
    )
    waiting_stale = waiting_block.split("if (!odometry_is_fresh)", 1)[1].split(
        "if (tag_fresh)", 1
    )[0]
    assert "publishStop();" in waiting_stale
    assert "return;" in waiting_stale

    reverse_block = source.split("case State::TAG_GUIDED_REVERSE:", 1)[1].split(
        "case State::FINAL_YAW_ALIGNMENT:", 1
    )[0]
    assert reverse_block.index("if (!odometry_is_fresh)") < reverse_block.index(
        "command.linear.x"
    )
    reverse_stale = reverse_block.split("if (!odometry_is_fresh)", 1)[1].split(
        "if (!tag_fresh)", 1
    )[0]
    assert "publishStop();" in reverse_stale
    assert "return;" in reverse_stale
    assert "command.linear" not in reverse_stale
    assert "command.angular" not in reverse_stale


def test_recoverable_service_state_repeats_on_status_heartbeat() -> None:
    """A restarted UI recovers parked state without relying on a transition edge."""
    source = NODE.read_text(encoding="utf-8")
    service_block = source.split("void publishServiceState(bool force = false)", 1)[
        1
    ].split("void publishStatus", 1)[0]
    assert "last_service_state_time_" in service_block
    assert "1.0 / std::max(status_rate_hz_, 0.1)" in service_block
    assert "last_service_state_time_ = current_time" in service_block

    status_block = source.split("void publishStatus(bool force = false)", 1)[1].split(
        "// HH_260720 - Descriptive parameters", 1
    )[0]
    assert "state_ == State::PARKED || state_ == State::WAITING_FOR_CHARGING" in (
        status_block
    )
    assert "publishServiceState();" in status_block
    assert "publishServiceState(true);" in source.split("void transitionTo", 1)[1]


def test_charging_preempts_every_active_motion_phase() -> None:
    """Normalized charging must publish zero before the state-specific commands."""
    source = NODE.read_text(encoding="utf-8")
    preemption = source.index("if (stop_when_charging_ && charging_detected_")
    state_switch = source.index("switch (state_)", preemption)
    assert preemption < state_switch
    guard = source[preemption:state_switch]
    assert "publishStop();" in guard
    assert "transitionTo(State::PARKED);" in guard
    assert 'translation_stop_reason_ = "charging"' in guard
    assert 'case State::WAITING_FOR_CHARGING: return "WAITING_FOR_CHARGING";' in source
    assert 'message.state_name = "WAITING_FOR_CHARGING";' in source


def test_safety_gate_owns_new_stopped_phases_without_dynamic_bypass() -> None:
    """The phases retain command ownership; rotation obstacle checks stay enabled."""
    gate_source = (CONTROL / "src/cmd_vel_safety_gate_node.cpp").read_text(
        encoding="utf-8"
    )
    arbiter = (
        CONTROL / "include/camrod_control/command_source_arbiter.hpp"
    ).read_text(encoding="utf-8")
    for phase in ("FINAL_YAW_ALIGNMENT", "WAITING_FOR_CHARGING"):
        assert phase in gate_source
    for phase in ("final_yaw_alignment", "waiting_for_charging"):
        assert phase in arbiter
    gate = yaml.safe_load(
        (CONTROL / "config/cmd_vel_safety_gate.yaml").read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    assert gate["rotation_cmd_dynamic_obstacle_stop"] is True
    assert gate["rotation_cmd_dynamic_obstacle_radius_m"] == 1.5
