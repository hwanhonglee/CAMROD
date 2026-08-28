"""Pure policy for the bounded operator manual-drive WebSocket.

The browser sends only discrete direction selectors.  This module validates
that protocol, computes a bounded planar command, and owns the single-client
lease/deadman state without importing ROS or FastAPI.  Keeping this policy
pure makes the fail-closed behavior directly testable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional


MANUAL_DRIVE_DEADMAN_TIMEOUT_S = 0.25
# Ordinary CAMROD keeps the original conservative envelope.  The CARLA
# composition may opt into the audited Ranger adapter limits below, without
# changing the production/default launch contract.
MANUAL_DRIVE_DEFAULT_LINEAR_LIMIT_MPS = 0.20
MANUAL_DRIVE_DEFAULT_LATERAL_LIMIT_MPS = 0.20
MANUAL_DRIVE_DEFAULT_ANGULAR_LIMIT_RADPS = 0.20
MANUAL_DRIVE_MAX_LINEAR_LIMIT_MPS = 1.40
MANUAL_DRIVE_MAX_LATERAL_LIMIT_MPS = 1.00
MANUAL_DRIVE_MAX_ANGULAR_LIMIT_RADPS = 0.7853
MANUAL_DRIVE_MINIMUM_SCALE = 0.10
MANUAL_DRIVE_MAXIMUM_SCALE = 1.00
MANUAL_DRIVE_MODES = frozenset({"ackermann", "zero_turn", "crab"})


class ManualDriveProtocolError(ValueError):
    """Protocol violation with a stable UI-facing error code."""

    def __init__(self, error: str, message: str) -> None:
        super().__init__(message)
        self.error = str(error)


@dataclass(frozen=True)
class ManualDriveLimits:
    """Server-owned command envelope; browser values can only scale it down."""

    linear_x_mps: float = MANUAL_DRIVE_DEFAULT_LINEAR_LIMIT_MPS
    lateral_y_mps: float = MANUAL_DRIVE_DEFAULT_LATERAL_LIMIT_MPS
    angular_z_radps: float = MANUAL_DRIVE_DEFAULT_ANGULAR_LIMIT_RADPS

    def __post_init__(self) -> None:
        maxima = {
            "linear_x_mps": MANUAL_DRIVE_MAX_LINEAR_LIMIT_MPS,
            "lateral_y_mps": MANUAL_DRIVE_MAX_LATERAL_LIMIT_MPS,
            "angular_z_radps": MANUAL_DRIVE_MAX_ANGULAR_LIMIT_RADPS,
        }
        for name, maximum in maxima.items():
            value = float(getattr(self, name))
            if not math.isfinite(value) or not 0.0 <= value <= maximum:
                raise ValueError(
                    f"{name} must be finite and within [0.0, {maximum}]"
                )

    def as_dict(self) -> Dict[str, float]:
        return {
            "linear_x_mps": float(self.linear_x_mps),
            "lateral_y_mps": float(self.lateral_y_mps),
            "angular_z_radps": float(self.angular_z_radps),
        }


@dataclass(frozen=True)
class ManualDriveCommand:
    """Validated body-frame planar command."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def moving(self) -> bool:
        return any(
            abs(value) > 1.0e-12
            for value in (self.linear_x, self.linear_y, self.angular_z)
        )

    def as_dict(self) -> Dict[str, float]:
        return {
            "linear_x": float(self.linear_x),
            "linear_y": float(self.linear_y),
            "angular_z": float(self.angular_z),
        }


@dataclass(frozen=True)
class ManualDriveLease:
    """Generation-qualified lease used to reject delayed socket cleanup."""

    owner: object
    generation: int


def _require_object(payload: Any) -> Mapping[str, Any]:
    if not isinstance(payload, Mapping):
        raise ManualDriveProtocolError(
            "invalid_message", "manual-drive frame must be a JSON object"
        )
    return payload


def _require_exact_keys(
    payload: Mapping[str, Any], required: set[str]
) -> None:
    present = set(payload.keys())
    missing = sorted(required - present)
    unknown = sorted(present - required)
    if missing:
        raise ManualDriveProtocolError(
            "invalid_message",
            "manual-drive frame is missing: " + ", ".join(missing),
        )
    if unknown:
        raise ManualDriveProtocolError(
            "invalid_message",
            "manual-drive frame has unknown fields: " + ", ".join(unknown),
        )


def _parse_sequence(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ManualDriveProtocolError(
            "invalid_sequence", "seq must be a non-negative integer"
        )
    return int(value)


def _parse_direction(name: str, value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value not in {-1, 0, 1}:
        raise ManualDriveProtocolError(
            "invalid_direction", f"{name} must be exactly -1, 0, or 1"
        )
    return int(value)


def _parse_scale(value: Any) -> float:
    # JSON protocol numbers arrive as int/float.  Do not coerce strings or
    # other truthy values at the safety boundary.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ManualDriveProtocolError(
            "invalid_scale", "scale must be a finite number in [0.1, 1.0]"
        )
    parsed = float(value)
    if not math.isfinite(parsed) or not (
        MANUAL_DRIVE_MINIMUM_SCALE <= parsed <= MANUAL_DRIVE_MAXIMUM_SCALE
    ):
        raise ManualDriveProtocolError(
            "invalid_scale", "scale must be a finite number in [0.1, 1.0]"
        )
    return parsed


def _parse_mode(value: Any) -> str:
    if not isinstance(value, str) or value not in MANUAL_DRIVE_MODES:
        raise ManualDriveProtocolError(
            "invalid_mode",
            "mode must be exactly 'ackermann', 'zero_turn', or 'crab'",
        )
    return value


def validate_control_frame(
    payload: Any, expected_type: str
) -> int:
    """Validate an ``arm`` or ``disarm`` frame and return its sequence."""
    frame = _require_object(payload)
    _require_exact_keys(frame, {"type", "seq"})
    if frame["type"] != expected_type:
        raise ManualDriveProtocolError(
            "invalid_type", f"expected manual-drive frame type {expected_type!r}"
        )
    return _parse_sequence(frame["seq"])


def validate_drive_frame(
    payload: Any, limits: Optional[ManualDriveLimits] = None
) -> tuple[int, ManualDriveCommand]:
    """Validate one discrete drive frame and derive its bounded command."""
    frame = _require_object(payload)
    _require_exact_keys(
        frame, {"type", "seq", "mode", "forward", "turn", "crab", "scale"}
    )
    if frame["type"] != "drive":
        raise ManualDriveProtocolError(
            "invalid_type", "expected manual-drive frame type 'drive'"
        )
    sequence = _parse_sequence(frame["seq"])
    mode = _parse_mode(frame["mode"])
    forward = _parse_direction("forward", frame["forward"])
    turn = _parse_direction("turn", frame["turn"])
    crab = _parse_direction("crab", frame["crab"])
    scale = _parse_scale(frame["scale"])

    # The mode is explicit so an A/D-only command cannot silently change from
    # Ackermann steering into zero-turn, and Crab cannot steal precedence from
    # another axis in the CARLA Ranger mapper.
    mode_conflict = (
        (mode == "ackermann" and (crab != 0 or (turn != 0 and forward == 0)))
        or (mode == "zero_turn" and (forward != 0 or crab != 0))
        or (mode == "crab" and (forward != 0 or turn != 0))
    )
    if mode_conflict:
        raise ManualDriveProtocolError(
            "conflicting_mode",
            f"drive axes do not match explicit mode {mode!r}",
        )

    envelope = limits if limits is not None else ManualDriveLimits()
    command = ManualDriveCommand(
        linear_x=float(forward) * float(envelope.linear_x_mps) * scale,
        linear_y=float(crab) * float(envelope.lateral_y_mps) * scale,
        angular_z=float(turn) * float(envelope.angular_z_radps) * scale,
    )
    return sequence, command


class ManualDrivePolicy:
    """Single-owner manual-drive protocol state and monotonic deadman."""

    def __init__(
        self,
        *,
        available: bool,
        limits: Optional[ManualDriveLimits] = None,
        deadman_timeout_s: float = MANUAL_DRIVE_DEADMAN_TIMEOUT_S,
    ) -> None:
        timeout = float(deadman_timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("deadman_timeout_s must be finite and > 0")
        self.available = bool(available)
        self.limits = limits if limits is not None else ManualDriveLimits()
        self.deadman_timeout_s = timeout
        self._generation = 0
        self._lease: Optional[ManualDriveLease] = None
        self._armed = False
        self._holding = False
        self._reason = "disabled" if not self.available else "available"
        self._command = ManualDriveCommand()
        self._deadline: Optional[float] = None
        self._last_sequence = -1

    def connect(self, owner: object) -> ManualDriveLease:
        if not self.available:
            raise ManualDriveProtocolError(
                "manual_drive_unavailable", "manual drive is not enabled"
            )
        if self._lease is not None:
            raise ManualDriveProtocolError(
                "manual_drive_busy", "another manual-drive client owns the lease"
            )
        self._generation += 1
        self._lease = ManualDriveLease(owner=owner, generation=self._generation)
        self._armed = False
        self._clear_motion()
        self._last_sequence = -1
        self._reason = "connected"
        return self._lease

    def lease_matches(self, lease: ManualDriveLease) -> bool:
        return (
            self._lease is not None
            and self._lease.owner is lease.owner
            and self._lease.generation == lease.generation
        )

    def _require_lease(self, lease: ManualDriveLease) -> None:
        if not self.lease_matches(lease):
            raise ManualDriveProtocolError(
                "stale_session", "manual-drive session is no longer the owner"
            )

    def _require_new_sequence(self, sequence: int) -> None:
        if sequence <= self._last_sequence:
            raise ManualDriveProtocolError(
                "stale_sequence",
                f"seq must increase monotonically beyond {self._last_sequence}",
            )

    def validate_arm(
        self, lease: ManualDriveLease, payload: Any
    ) -> int:
        """Validate an arm frame without mutating state before full STOP."""
        self._require_lease(lease)
        sequence = validate_control_frame(payload, "arm")
        self._require_new_sequence(sequence)
        return sequence

    def arm(
        self,
        lease: ManualDriveLease,
        payload: Any,
        now_monotonic: float,
    ) -> None:
        self._require_lease(lease)
        sequence = validate_control_frame(payload, "arm")
        self._require_new_sequence(sequence)
        now = float(now_monotonic)
        if not math.isfinite(now):
            raise ManualDriveProtocolError(
                "invalid_time", "manual-drive monotonic timestamp is invalid"
            )
        self._last_sequence = sequence
        self._armed = True
        self._clear_motion()
        # Authorization starts aging immediately.  A browser that disappears
        # after arm but before its first zero/motion heartbeat cannot leave the
        # manual gate armed indefinitely.
        self._deadline = now + self.deadman_timeout_s
        self._reason = "armed"

    def drive(
        self, lease: ManualDriveLease, payload: Any, now_monotonic: float
    ) -> ManualDriveCommand:
        self._require_lease(lease)
        if not self._armed:
            raise ManualDriveProtocolError(
                "manual_drive_not_armed", "arm manual drive before sending motion"
            )
        now = float(now_monotonic)
        if not math.isfinite(now):
            raise ManualDriveProtocolError(
                "invalid_time", "manual-drive monotonic timestamp is invalid"
            )
        # Check the existing lease deadline before validating or accepting this
        # frame.  If the ROS timer was delayed, a frame arriving at/after the
        # deadline must not renew the expired authorization or restart motion.
        if self._expire_deadline(now):
            raise ManualDriveProtocolError(
                "manual_drive_deadman_expired",
                "manual-drive heartbeat expired; arm again before driving",
            )
        sequence, command = validate_drive_frame(payload, self.limits)
        self._require_new_sequence(sequence)
        self._last_sequence = sequence
        self._command = command
        self._holding = command.moving
        # Zero is a release command, not a lease release.  The browser sends it
        # as a 10-Hz heartbeat while armed, so it refreshes the same deadline
        # without claiming that the operator is holding a motion direction.
        self._deadline = now + self.deadman_timeout_s
        self._reason = "driving" if command.moving else "released"
        return command

    def disarm(self, lease: ManualDriveLease, payload: Any) -> None:
        self._require_lease(lease)
        sequence = validate_control_frame(payload, "disarm")
        self._require_new_sequence(sequence)
        self._last_sequence = sequence
        self._armed = False
        self._clear_motion()
        self._reason = "client_disarm"

    def revoke(self, reason: str) -> bool:
        """Revoke authorization while retaining the connected owner's lease."""
        changed = self._armed or self._holding or self._command.moving
        self._armed = False
        self._clear_motion()
        self._reason = str(reason)
        return changed

    def expire(self, now_monotonic: float) -> bool:
        now = float(now_monotonic)
        return self._expire_deadline(now)

    def _expire_deadline(self, now_monotonic: float) -> bool:
        if not self._armed or self._deadline is None:
            return False
        # A corrupted clock sample must not preserve manual authorization.
        # ``time.monotonic()`` is finite in normal operation; treating any
        # impossible value as expired keeps the authorization fail-closed.
        now = float(now_monotonic)
        if math.isfinite(now) and now < self._deadline:
            return False
        self._armed = False
        self._clear_motion()
        self._reason = "deadman_timeout"
        return True

    def disconnect(self, lease: ManualDriveLease) -> bool:
        """Release only a matching generation; delayed cleanup is a no-op."""
        if not self.lease_matches(lease):
            return False
        self._lease = None
        self._armed = False
        self._clear_motion()
        self._reason = "disconnected"
        return True

    def shutdown(self) -> None:
        self._lease = None
        self._armed = False
        self._clear_motion()
        self._reason = "shutdown"

    def snapshot(self) -> Dict[str, Any]:
        return {
            "available": bool(self.available),
            "connected": self._lease is not None,
            "armed": bool(self._armed),
            "holding": bool(self._holding),
            "reason": self._reason,
            "limits": self.limits.as_dict(),
            "command": self._command.as_dict(),
        }

    def _clear_motion(self) -> None:
        self._holding = False
        self._command = ManualDriveCommand()
        self._deadline = None
