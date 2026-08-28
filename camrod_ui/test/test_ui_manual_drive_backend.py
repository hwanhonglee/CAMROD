"""Backend ordering tests for the manual-drive ROS/WebSocket boundary."""

from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from camrod_ui.manual_drive_policy import (  # noqa: E402
    ManualDriveCommand,
    ManualDrivePolicy,
    ManualDriveProtocolError,
)
from camrod_ui.ui_backend_node import UiBackendNode  # noqa: E402


def control_frame(frame_type: str, seq: int) -> dict:
    return {"type": frame_type, "seq": seq}


def drive_frame(
    seq: int,
    *,
    mode: str = "ackermann",
    forward: int = 0,
    turn: int = 0,
    crab: int = 0,
    scale: float = 1.0,
) -> dict:
    return {
        "type": "drive",
        "seq": seq,
        "mode": mode,
        "forward": forward,
        "turn": turn,
        "crab": crab,
        "scale": scale,
    }


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class UiManualDriveBackendTest(unittest.TestCase):
    def make_backend(self):
        events = []
        policy = ManualDrivePolicy(available=True)
        lease = policy.connect(object())
        backend = SimpleNamespace(
            manual_drive_available=True,
            _manual_drive_transition_lock=threading.RLock(),
            _manual_drive_policy=policy,
            _manual_drive_transport=None,
            pub_manual_cmd_vel_ros=_Publisher(),
        )
        backend._publish_manual_drive_command = (
            lambda command: events.append(("command", command))
        )
        backend._publish_manual_drive_zero = (
            lambda: events.append(("zero",))
        )
        backend._stop_active_service = (
            lambda source: events.append(("full_stop", source))
        )
        backend._publish_mission_engage = (
            lambda enabled, source: events.append(
                ("mission_engage", enabled, source)
            )
        )
        backend._publish_engage = (
            lambda enabled, source: events.append(
                ("manual_engage", enabled, source)
            )
        )
        return backend, policy, lease, events

    def test_arm_orders_full_stop_mission_release_zero_then_engage(self) -> None:
        backend, policy, lease, events = self.make_backend()

        response = UiBackendNode._arm_manual_drive(
            backend, lease, control_frame("arm", 1)
        )

        self.assertEqual(
            [event[0] for event in events],
            ["full_stop", "mission_engage", "zero", "manual_engage"],
        )
        self.assertFalse(events[1][1])
        self.assertTrue(events[3][1])
        self.assertTrue(policy.snapshot()["armed"])
        self.assertEqual(response["type"], "state")
        self.assertTrue(response["manual_drive"]["armed"])

    def test_drive_publishes_only_server_derived_bounded_command(self) -> None:
        backend, policy, lease, events = self.make_backend()
        policy.arm(lease, control_frame("arm", 1), time.monotonic())

        response = UiBackendNode._apply_manual_drive(
            backend,
            lease,
            drive_frame(2, forward=1, turn=-1, scale=0.5),
        )

        self.assertEqual(len(events), 1)
        command = events[0][1]
        self.assertIsInstance(command, ManualDriveCommand)
        self.assertAlmostEqual(command.linear_x, 0.10)
        self.assertAlmostEqual(command.angular_z, -0.10)
        self.assertTrue(response["manual_drive"]["holding"])

    def test_zero_drive_releases_motion_but_keeps_manual_arm(self) -> None:
        backend, policy, lease, events = self.make_backend()
        policy.arm(lease, control_frame("arm", 1), time.monotonic())
        UiBackendNode._apply_manual_drive(
            backend, lease, drive_frame(2, forward=1)
        )
        response = UiBackendNode._apply_manual_drive(
            backend, lease, drive_frame(3, scale=0.5)
        )

        self.assertEqual(events[-1][0], "command")
        self.assertFalse(events[-1][1].moving)
        self.assertTrue(response["manual_drive"]["armed"])
        self.assertFalse(response["manual_drive"]["holding"])

    def test_late_frame_returns_deadman_error_and_keeps_timeout_status(self) -> None:
        backend, policy, lease, events = self.make_backend()
        policy.arm(lease, control_frame("arm", 1), 10.0)
        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=10.0
        ):
            UiBackendNode._apply_manual_drive(
                backend, lease, drive_frame(2, forward=1)
            )
        events.clear()

        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=10.25
        ):
            with self.assertRaises(ManualDriveProtocolError) as context:
                UiBackendNode._apply_manual_drive(
                    backend, lease, drive_frame(3, forward=-1)
                )
        response = UiBackendNode._fail_closed_manual_drive(
            backend, lease, context.exception
        )

        self.assertEqual(response["type"], "error")
        self.assertEqual(response["error"], "manual_drive_deadman_expired")
        self.assertEqual(response["manual_drive"]["reason"], "deadman_timeout")
        self.assertFalse(response["manual_drive"]["armed"])
        self.assertEqual(
            [event[0] for event in events], ["zero", "manual_engage"]
        )
        self.assertFalse(events[-1][1])

    def test_zero_heartbeat_is_disarmed_when_backend_timer_expires(self) -> None:
        backend, policy, lease, events = self.make_backend()
        policy.arm(lease, control_frame("arm", 1), 20.0)
        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=20.0
        ):
            UiBackendNode._apply_manual_drive(
                backend, lease, drive_frame(2)
            )
        events.clear()
        transport = object()
        backend._manual_drive_transport = transport
        backend._schedule_manual_drive_payload = (
            lambda selected, payload: events.append(
                ("notify", selected, payload["manual_drive"]["reason"])
            )
        )
        backend.get_logger = lambda: SimpleNamespace(
            warn=lambda message: events.append(("warn", message))
        )

        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=20.25
        ):
            UiBackendNode._on_manual_drive_deadman(backend)

        self.assertEqual(
            [event[0] for event in events],
            ["zero", "manual_engage", "notify", "warn"],
        )
        self.assertIs(events[2][1], transport)
        self.assertEqual(events[2][2], "deadman_timeout")
        state = policy.snapshot()
        self.assertFalse(state["armed"])
        self.assertFalse(state["holding"])
        self.assertEqual(state["reason"], "deadman_timeout")

    def test_arm_without_first_heartbeat_is_disarmed_by_backend_timer(self) -> None:
        backend, policy, lease, events = self.make_backend()
        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=30.0
        ):
            UiBackendNode._arm_manual_drive(
                backend, lease, control_frame("arm", 1)
            )
        events.clear()
        transport = object()
        backend._manual_drive_transport = transport
        backend._schedule_manual_drive_payload = (
            lambda selected, payload: events.append(
                ("notify", selected, payload["manual_drive"]["reason"])
            )
        )
        backend.get_logger = lambda: SimpleNamespace(
            warn=lambda message: events.append(("warn", message))
        )

        with mock.patch(
            "camrod_ui.ui_backend_node.time.monotonic", return_value=30.25
        ):
            UiBackendNode._on_manual_drive_deadman(backend)

        self.assertEqual(
            [event[0] for event in events],
            ["zero", "manual_engage", "notify", "warn"],
        )
        self.assertIs(events[2][1], transport)
        self.assertEqual(events[2][2], "deadman_timeout")
        self.assertFalse(policy.snapshot()["armed"])

    def test_disarm_publishes_zero_before_manual_engage_false(self) -> None:
        backend, policy, lease, events = self.make_backend()
        policy.arm(lease, control_frame("arm", 1), 10.0)
        policy.drive(lease, drive_frame(2, mode="crab", crab=1), 10.0)

        response = UiBackendNode._disarm_manual_drive(
            backend, lease, control_frame("disarm", 3)
        )

        self.assertEqual(
            [event[0] for event in events], ["zero", "manual_engage"]
        )
        self.assertFalse(events[1][1])
        self.assertFalse(response["manual_drive"]["armed"])
        self.assertEqual(response["manual_drive"]["reason"], "client_disarm")

    def test_malformed_owner_frame_is_fail_closed(self) -> None:
        backend, policy, lease, events = self.make_backend()
        policy.arm(lease, control_frame("arm", 1), 10.0)
        policy.drive(lease, drive_frame(2, forward=1), 10.0)
        error = ManualDriveProtocolError(
            "conflicting_mode", "crab cannot be combined with forward or turn"
        )

        response = UiBackendNode._fail_closed_manual_drive(
            backend, lease, error
        )

        self.assertEqual(
            [event[0] for event in events], ["zero", "manual_engage"]
        )
        self.assertFalse(policy.snapshot()["armed"])
        self.assertEqual(response["type"], "error")
        self.assertEqual(response["error"], "conflicting_mode")
        self.assertEqual(
            response["manual_drive"]["reason"],
            "protocol_error:conflicting_mode",
        )

    def test_stale_socket_error_cannot_zero_new_generation(self) -> None:
        backend, policy, old_lease, events = self.make_backend()
        policy.disconnect(old_lease)
        new_lease = policy.connect(object())
        policy.arm(new_lease, control_frame("arm", 1), 0.0)

        response = UiBackendNode._fail_closed_manual_drive(
            backend,
            old_lease,
            ManualDriveProtocolError("stale_session", "old socket"),
        )

        self.assertEqual(events, [])
        self.assertTrue(policy.snapshot()["armed"])
        self.assertEqual(response["error"], "stale_session")

    def test_ros_publisher_sets_only_planar_twist_axes(self) -> None:
        publisher = _Publisher()
        backend = SimpleNamespace(pub_manual_cmd_vel_ros=publisher)
        UiBackendNode._publish_manual_drive_command(
            backend,
            ManualDriveCommand(
                linear_x=0.12, linear_y=-0.08, angular_z=0.04
            ),
        )

        self.assertEqual(len(publisher.messages), 1)
        message = publisher.messages[0]
        self.assertAlmostEqual(message.linear.x, 0.12)
        self.assertAlmostEqual(message.linear.y, -0.08)
        self.assertAlmostEqual(message.angular.z, 0.04)
        self.assertEqual(message.linear.z, 0.0)
        self.assertEqual(message.angular.x, 0.0)
        self.assertEqual(message.angular.y, 0.0)

    def test_empty_topic_has_no_ros_publish_side_effect(self) -> None:
        backend = SimpleNamespace(pub_manual_cmd_vel_ros=None)
        UiBackendNode._publish_manual_drive_command(
            backend, ManualDriveCommand(linear_x=0.20)
        )

    def test_shutdown_zeroes_and_disengages_before_transport_teardown(self) -> None:
        backend, policy, lease, events = self.make_backend()
        policy.arm(lease, control_frame("arm", 1), 10.0)
        policy.drive(lease, drive_frame(2, forward=1), 10.0)
        transport = object()
        backend._manual_drive_transport = transport
        backend._manual_drive_shutdown_done = False
        backend._schedule_manual_drive_payload = (
            lambda selected, payload: events.append(
                ("notify", selected, payload["manual_drive"]["reason"])
            )
        )

        UiBackendNode._shutdown_manual_drive(backend)

        self.assertEqual(
            [event[0] for event in events],
            ["zero", "manual_engage", "notify"],
        )
        self.assertIs(events[-1][1], transport)
        self.assertEqual(events[-1][2], "shutdown")
        self.assertIsNone(backend._manual_drive_transport)
        self.assertFalse(policy.snapshot()["connected"])

    def test_launch_keeps_manual_drive_opt_in(self) -> None:
        launch_path = (
            Path(__file__).resolve().parents[1]
            / "camrod_ui_robot"
            / "launch"
            / "ui.launch.py"
        )
        source = launch_path.read_text(encoding="utf-8")
        self.assertIn("'manual_cmd_vel_ros_topic'", source)
        self.assertRegex(
            source,
            r"manual_cmd_vel_ros_topic_arg\s*=\s*DeclareLaunchArgument\("
            r"\s*'manual_cmd_vel_ros_topic',\s*default_value='',",
        )
        self.assertIn(
            "'manual_cmd_vel_ros_topic': "
            "LaunchConfiguration('manual_cmd_vel_ros_topic')",
            source,
        )
        for name in (
            "manual_drive_linear_limit_mps",
            "manual_drive_lateral_limit_mps",
            "manual_drive_angular_limit_radps",
            "manual_drive_deadman_timeout_s",
        ):
            self.assertIn(f"'{name}'", source)
            self.assertIn(
                f"LaunchConfiguration('{name}')",
                source,
            )


if __name__ == "__main__":
    unittest.main()
