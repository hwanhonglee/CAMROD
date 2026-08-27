"""Fail-closed unit coverage for the operator manual-drive protocol."""

from pathlib import Path
import math
import sys
import unittest


sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / "runtime" / "python"),
)

from camrod_ui.manual_drive_policy import (  # noqa: E402
    MANUAL_DRIVE_DEADMAN_TIMEOUT_S,
    ManualDriveLimits,
    ManualDrivePolicy,
    ManualDriveProtocolError,
    validate_control_frame,
    validate_drive_frame,
)


def control_frame(frame_type: str, seq: int) -> dict:
    return {"type": frame_type, "seq": seq}


def drive_frame(
    seq: int,
    *,
    forward: int = 0,
    turn: int = 0,
    crab: int = 0,
    scale: float = 1.0,
) -> dict:
    return {
        "type": "drive",
        "seq": seq,
        "forward": forward,
        "turn": turn,
        "crab": crab,
        "scale": scale,
    }


class ManualDriveValidationTest(unittest.TestCase):
    def assert_protocol_error(self, error: str, callback) -> None:
        with self.assertRaises(ManualDriveProtocolError) as context:
            callback()
        self.assertEqual(context.exception.error, error)

    def test_server_limits_are_fixed_at_safe_carla_test_values(self) -> None:
        _, command = validate_drive_frame(
            drive_frame(1, forward=1, turn=1, scale=1.0)
        )
        self.assertAlmostEqual(command.linear_x, 0.20)
        self.assertAlmostEqual(command.angular_z, 0.20)
        # The ordinary W+A command remains Dual-Ackermann: 0.2/0.2 = 1 m,
        # above the audited Ranger minimum radius 0.810330349 m.
        self.assertAlmostEqual(
            abs(command.linear_x) / abs(command.angular_z), 1.0
        )

    def test_server_envelope_cannot_be_configured_above_fixed_limits(self) -> None:
        for field in (
            "linear_x_mps",
            "lateral_y_mps",
            "angular_z_radps",
        ):
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    ManualDriveLimits(**{field: 0.200001})

    def test_scale_can_only_reduce_server_limits(self) -> None:
        _, command = validate_drive_frame(
            drive_frame(2, crab=1, scale=0.10)
        )
        self.assertAlmostEqual(command.linear_x, 0.0)
        self.assertAlmostEqual(command.linear_y, 0.02)
        self.assertEqual(command.angular_z, 0.0)

    def test_zero_direction_frame_is_a_release_command(self) -> None:
        _, command = validate_drive_frame(drive_frame(3, scale=0.5))
        self.assertFalse(command.moving)
        self.assertEqual(
            command.as_dict(),
            {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
        )

    def test_crab_cannot_be_mixed_with_forward_or_turn(self) -> None:
        for frame in (
            drive_frame(4, turn=1, crab=-1, scale=0.5),
            drive_frame(4, forward=1, crab=1, scale=0.5),
        ):
            with self.subTest(frame=frame):
                self.assert_protocol_error(
                    "conflicting_mode",
                    lambda frame=frame: validate_drive_frame(frame),
                )

    def test_directions_are_exact_discrete_integers(self) -> None:
        for invalid in (True, 0.5, 2, -2, "1", None):
            with self.subTest(invalid=invalid):
                self.assert_protocol_error(
                    "invalid_direction",
                    lambda invalid=invalid: validate_drive_frame(
                        drive_frame(5, forward=invalid)
                    ),
                )

    def test_scale_rejects_nonfinite_and_out_of_range_values(self) -> None:
        for invalid in (
            True,
            float("nan"),
            float("inf"),
            0.09,
            1.01,
            "0.5",
            None,
        ):
            with self.subTest(invalid=invalid):
                self.assert_protocol_error(
                    "invalid_scale",
                    lambda invalid=invalid: validate_drive_frame(
                        drive_frame(6, forward=1, scale=invalid)
                    ),
                )

    def test_sequence_rejects_bool_negative_and_fractional_values(self) -> None:
        for invalid in (True, -1, 1.0, "1", None):
            with self.subTest(invalid=invalid):
                self.assert_protocol_error(
                    "invalid_sequence",
                    lambda invalid=invalid: validate_control_frame(
                        {"type": "arm", "seq": invalid}, "arm"
                    ),
                )

    def test_unknown_or_missing_fields_are_rejected(self) -> None:
        extra = drive_frame(7, forward=1)
        extra["linear_x"] = 99.0
        self.assert_protocol_error(
            "invalid_message", lambda: validate_drive_frame(extra)
        )
        missing = drive_frame(8, forward=1)
        del missing["crab"]
        self.assert_protocol_error(
            "invalid_message", lambda: validate_drive_frame(missing)
        )

    def test_non_object_payload_is_rejected(self) -> None:
        for invalid in (None, [], "drive", 1):
            with self.subTest(invalid=invalid):
                self.assert_protocol_error(
                    "invalid_message",
                    lambda invalid=invalid: validate_drive_frame(invalid),
                )


class ManualDriveSessionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.policy = ManualDrivePolicy(available=True)
        self.owner = object()
        self.lease = self.policy.connect(self.owner)

    def assert_protocol_error(self, error: str, callback) -> None:
        with self.assertRaises(ManualDriveProtocolError) as context:
            callback()
        self.assertEqual(context.exception.error, error)

    def test_initial_state_and_limits_match_websocket_contract(self) -> None:
        self.assertEqual(
            self.policy.snapshot(),
            {
                "available": True,
                "connected": True,
                "armed": False,
                "holding": False,
                "reason": "connected",
                "limits": {
                    "linear_x_mps": 0.20,
                    "lateral_y_mps": 0.20,
                    "angular_z_radps": 0.20,
                },
                "command": {
                    "linear_x": 0.0,
                    "linear_y": 0.0,
                    "angular_z": 0.0,
                },
            },
        )

    def test_drive_requires_arm_and_monotonic_sequence(self) -> None:
        self.assert_protocol_error(
            "manual_drive_not_armed",
            lambda: self.policy.drive(
                self.lease, drive_frame(1, forward=1), 10.0
            ),
        )
        self.policy.arm(self.lease, control_frame("arm", 1))
        command = self.policy.drive(
            self.lease, drive_frame(2, forward=1), 10.0
        )
        self.assertTrue(command.moving)
        self.assert_protocol_error(
            "stale_sequence",
            lambda: self.policy.drive(
                self.lease, drive_frame(2, forward=-1), 10.1
            ),
        )

    def test_validate_arm_does_not_consume_sequence_before_full_stop(self) -> None:
        self.assertEqual(
            self.policy.validate_arm(self.lease, control_frame("arm", 10)),
            10,
        )
        self.policy.arm(self.lease, control_frame("arm", 10))
        self.assertTrue(self.policy.snapshot()["armed"])

    def test_zero_drive_releases_hold_without_disarming(self) -> None:
        self.policy.arm(self.lease, control_frame("arm", 1))
        self.policy.drive(
            self.lease, drive_frame(2, forward=1, scale=0.5), 20.0
        )
        self.policy.drive(self.lease, drive_frame(3, scale=0.5), 20.1)
        state = self.policy.snapshot()
        self.assertTrue(state["armed"])
        self.assertFalse(state["holding"])
        self.assertEqual(state["reason"], "released")
        self.assertFalse(self.policy.expire(100.0))

    def test_deadman_expires_at_250_ms_and_revokes_arm(self) -> None:
        self.policy.arm(self.lease, control_frame("arm", 1))
        self.policy.drive(
            self.lease, drive_frame(2, forward=1), 100.0
        )
        self.assertFalse(
            self.policy.expire(100.0 + MANUAL_DRIVE_DEADMAN_TIMEOUT_S - 1e-6)
        )
        self.assertTrue(
            self.policy.expire(100.0 + MANUAL_DRIVE_DEADMAN_TIMEOUT_S)
        )
        state = self.policy.snapshot()
        self.assertFalse(state["armed"])
        self.assertFalse(state["holding"])
        self.assertEqual(state["reason"], "deadman_timeout")
        self.assertEqual(state["command"]["linear_x"], 0.0)
        self.assertFalse(self.policy.expire(101.0))

    def test_nonfinite_deadman_sample_fails_closed(self) -> None:
        self.policy.arm(self.lease, control_frame("arm", 1))
        self.policy.drive(
            self.lease, drive_frame(2, forward=1), 100.0
        )

        self.assertTrue(self.policy.expire(math.nan))
        state = self.policy.snapshot()
        self.assertFalse(state["armed"])
        self.assertFalse(state["holding"])
        self.assertEqual(state["reason"], "deadman_timeout")

    def test_disarm_is_sequenced_and_keeps_connection(self) -> None:
        self.policy.arm(self.lease, control_frame("arm", 1))
        self.policy.drive(
            self.lease, drive_frame(2, turn=-1), 30.0
        )
        self.policy.disarm(self.lease, control_frame("disarm", 3))
        state = self.policy.snapshot()
        self.assertTrue(state["connected"])
        self.assertFalse(state["armed"])
        self.assertFalse(state["holding"])
        self.assertEqual(state["reason"], "client_disarm")

    def test_external_revoke_requires_a_fresh_rearm_sequence(self) -> None:
        self.policy.arm(self.lease, control_frame("arm", 1))
        self.policy.drive(
            self.lease, drive_frame(2, forward=1), 40.0
        )
        self.assertTrue(self.policy.revoke("mission_takeover"))
        self.assert_protocol_error(
            "manual_drive_not_armed",
            lambda: self.policy.drive(
                self.lease, drive_frame(3, forward=1), 40.1
            ),
        )
        self.policy.arm(self.lease, control_frame("arm", 3))
        self.assertTrue(self.policy.snapshot()["armed"])

    def test_second_owner_is_rejected_without_changing_active_state(self) -> None:
        self.policy.arm(self.lease, control_frame("arm", 1))
        self.assert_protocol_error(
            "manual_drive_busy", lambda: self.policy.connect(object())
        )
        self.assertTrue(self.policy.snapshot()["armed"])

    def test_generation_blocks_delayed_old_socket_cleanup(self) -> None:
        old_lease = self.lease
        self.assertTrue(self.policy.disconnect(old_lease))
        new_lease = self.policy.connect(object())
        self.policy.arm(new_lease, control_frame("arm", 1))
        self.assertFalse(self.policy.disconnect(old_lease))
        state = self.policy.snapshot()
        self.assertTrue(state["connected"])
        self.assertTrue(state["armed"])
        self.assertNotEqual(old_lease.generation, new_lease.generation)

    def test_stale_lease_cannot_publish_or_disarm_new_owner(self) -> None:
        old_lease = self.lease
        self.policy.disconnect(old_lease)
        new_lease = self.policy.connect(object())
        self.policy.arm(new_lease, control_frame("arm", 1))
        for callback in (
            lambda: self.policy.drive(
                old_lease, drive_frame(2, forward=1), 50.0
            ),
            lambda: self.policy.disarm(
                old_lease, control_frame("disarm", 2)
            ),
        ):
            self.assert_protocol_error("stale_session", callback)
        self.assertTrue(self.policy.snapshot()["armed"])

    def test_shutdown_clears_owner_command_and_authorization(self) -> None:
        self.policy.arm(self.lease, control_frame("arm", 1))
        self.policy.drive(
            self.lease, drive_frame(2, crab=1), 60.0
        )
        self.policy.shutdown()
        state = self.policy.snapshot()
        self.assertFalse(state["connected"])
        self.assertFalse(state["armed"])
        self.assertFalse(state["holding"])
        self.assertEqual(state["reason"], "shutdown")


class DisabledManualDriveTest(unittest.TestCase):
    def test_empty_topic_policy_remains_unavailable(self) -> None:
        policy = ManualDrivePolicy(available=False)
        self.assertEqual(policy.snapshot()["reason"], "disabled")
        with self.assertRaises(ManualDriveProtocolError) as context:
            policy.connect(object())
        self.assertEqual(context.exception.error, "manual_drive_unavailable")

    def test_deadman_configuration_must_be_finite_and_positive(self) -> None:
        for invalid in (0.0, -1.0, math.inf, math.nan):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    ManualDrivePolicy(
                        available=True, deadman_timeout_s=invalid
                    )


if __name__ == "__main__":
    unittest.main()
