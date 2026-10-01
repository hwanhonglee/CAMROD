from pathlib import Path
import subprocess


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "field_test_tool.sh"


def _script_text() -> str:
    return SCRIPT.read_text(encoding="utf-8")


# HH_260730 - Keep the field acceptance shell tool parseable before deployment.
def test_field_tool_shell_syntax() -> None:
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)


# HH_260730 - Require the native gate message and explicit drive/engage controls.
def test_gate_commands_use_native_avg_bool_contract() -> None:
    text = _script_text()

    assert "avg_msgs/msg/AvgBool" in text
    assert "std_msgs/msg/Bool" not in text
    for topic in (
        "/planning/engage",
        "/planning/mission_engage",
        "/platform/drive_enable",
    ):
        assert topic in text
    assert 'publish_gate_bool "${topic}" false' in text
    assert "publish_gate_bool /planning/engage true" in text
    assert "publish_gate_bool /platform/drive_enable true" in text


# HH_260730 - Do not report a stop until subscribers and disabled outputs agree.
def test_stop_gate_fails_closed_and_verifies_output() -> None:
    text = _script_text()

    assert "require_gate_subscriber" in text
    assert "has no subscriber; refusing to report a gate change" in text
    assert "verify_command_disabled" in text
    assert "/control/command_enabled did not confirm false" in text
    assert "/control/planning_engaged did not confirm false" in text
    assert "Attempt every close command" in text
    assert "/platform/set_enabled" not in text


# HH_260730 - Record recovery evidence only while each safety owner is live.
def test_recovery_recording_requires_live_safety_owners() -> None:
    text = _script_text()

    assert "ros2 topic list --include-hidden-topics" in text
    assert "required recovery topic is not active" in text
    for topic in (
        "/control/cmd_vel_safety_gate/status",
        "/control/command_enabled",
        "/localization/pose",
        "/map/cost_grid/lanelet",
        "/platform/status",
    ):
        assert topic in text


# HH_260730 - Distinguish intended topics from topics present in the captured bag.
def test_recovery_metadata_separates_requested_and_recorded_topics() -> None:
    text = _script_text()

    assert "requested_topics.txt" in text
    assert "available_at_start_topics.txt" in text
    assert "bag_info.txt" in text
    assert "recorded_topics.txt" in text


# HH_260730 - Collect watch and profiling samples concurrently in the field.
def test_watch_and_profile_collect_concurrent_evidence() -> None:
    text = _script_text()

    assert "collect_watch_sample" in text
    assert "The old sequential" in text
    assert "profile_topics" in text
    assert "tegrastats --interval 1000" in text
    assert "profile) cmd_profile" in text


# HH_260730 - Route the field command through the synchronized pose probe.
def test_pose_latency_command_runs_the_synchronized_probe() -> None:
    text = _script_text()

    assert "pose-latency [seconds] [output_json]" in text
    assert "cmd_pose_latency" in text
    assert "ros2 run camrod_bringup pose_latency_probe.py" in text
    assert "pose-latency) cmd_pose_latency" in text


# HH_260730 - Preserve the full camera/YOLO acceptance window and evidence topics.
def test_camera_yolo_defaults_to_full_acceptance_window() -> None:
    text = _script_text()

    assert "CAMERA_YOLO_ACCEPTANCE_SECONDS=300" in text
    assert "camera_payload_probe.py" in text
    assert "/sensing/camera/econ_front/camera_info" in text
    assert "/sensing/camera/econ_front/dummy_active" in text
    assert "## component native libraries" in text


# HH_260730 - Fail configuration sync when either tree contains unmirrored files.
def test_config_sync_rejects_extra_package_and_install_files() -> None:
    text = _script_text()

    assert "EXTRA package file:" in text
    assert "EXTRA installed file:" in text
