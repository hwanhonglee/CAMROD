#!/usr/bin/env python3
"""Bounded, read-only readiness check for the Ranger physical 4WS bridge."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import math
import sys
import time


EXPECTED_BACKEND = "PHYSX_FOUR_WHEEL_STEERING"
LIFECYCLE_HINT = (
    "Start/restart in order server -> bridge -> spawn -> camrod, wait for "
    "the bridge to arm, then retry manual."
)


class BridgeStatusError(RuntimeError):
    """Expected, user-facing physical bridge readiness failure."""


def _field(status, name: str):
    if isinstance(status, Mapping):
        return status.get(name)
    return getattr(status, name, None)


def validate_status(status, role_name: str) -> int:
    """Validate one status sample and return its bound CARLA actor id."""
    required = {
        "ready": True,
        "physical_gate_accepted": True,
        "physx_substep_control_verified": True,
        "independent_wheel_drive_available": True,
        "motion_backend": EXPECTED_BACKEND,
    }
    mismatches = [
        f"{name}={_field(status, name)!r} (expected {expected!r})"
        for name, expected in required.items()
        if _field(status, name) != expected
    ]
    try:
        actor_id = int(_field(status, "actor_id") or 0)
    except (TypeError, ValueError):
        actor_id = 0
    if actor_id <= 0:
        mismatches.append(
            f"actor_id={_field(status, 'actor_id')!r} (expected > 0)"
        )
    if mismatches:
        reason = str(_field(status, "reason") or "").strip()
        raise BridgeStatusError(
            "physical 4WS bridge is not ready for role "
            f"{role_name!r}: {', '.join(mismatches)}; "
            f"reason={reason or 'no reason reported'}. {LIFECYCLE_HINT}"
        )
    return actor_id


def _positive_timeout(value: str) -> float:
    try:
        result = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("timeout must be a number") from None
    if not math.isfinite(result) or result <= 0.0 or result > 30.0:
        raise argparse.ArgumentTypeError(
            "timeout must be finite, positive, and at most 30 seconds"
        )
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role-name", required=True)
    parser.add_argument("--timeout-seconds", type=_positive_timeout, default=5.0)
    parser.add_argument(
        "--actor-id-only",
        action="store_true",
        help="print only the positive bridge-bound CARLA actor id on success",
    )
    return parser


def format_success(actor_id: int, role_name: str, actor_id_only: bool) -> str:
    """Format either the human-readable or machine-readable success result."""
    if actor_id_only:
        return str(actor_id)
    return (
        "[virtual_carla] physical 4WS bridge ready: "
        f"role={role_name} actor_id={actor_id} backend={EXPECTED_BACKEND}"
    )


def check_live_status(role_name: str, timeout_seconds: float) -> int:
    """Poll the ROS graph until a fresh, ready physical status is observed."""
    try:
        import rclpy  # pylint: disable=import-outside-toplevel
        from rclpy.qos import (  # pylint: disable=import-outside-toplevel
            QoSDurabilityPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )
        from carla_extended_ackermann_msgs.msg import (  # pylint: disable=import-outside-toplevel
            PhysicalFourWheelStatus,
        )
    except (ImportError, ModuleNotFoundError) as error:
        raise BridgeStatusError(
            "cannot import physical 4WS ROS interfaces: "
            f"{type(error).__name__}: {error}"
        ) from None

    topic = f"/carla/{role_name}/physical_four_wheel_status"
    latest = []
    last_rejection = "no status sample received"
    node = None
    initialized = False
    try:
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node("camrod_physical_4ws_manual_preflight")
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        node.create_subscription(
            PhysicalFourWheelStatus,
            topic,
            lambda message: latest.append(message),
            qos,
        )
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
            while latest:
                status = latest.pop(0)
                try:
                    return validate_status(status, role_name)
                except BridgeStatusError as error:
                    last_rejection = str(error)
    except (KeyboardInterrupt, RuntimeError, TypeError, ValueError) as error:
        if isinstance(error, KeyboardInterrupt):
            raise
        raise BridgeStatusError(
            "physical 4WS ROS status check failed: "
            f"{type(error).__name__}: {error}"
        ) from None
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()

    raise BridgeStatusError(
        f"no ready physical 4WS status on {topic} within "
        f"{timeout_seconds:g}s; last observation: {last_rejection}. "
        f"{LIFECYCLE_HINT}"
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    role_name = args.role_name.strip()
    if not role_name:
        print(
            "[virtual_carla] ERROR: physical 4WS preflight: "
            "role name must not be empty",
            file=sys.stderr,
        )
        return 1
    try:
        actor_id = check_live_status(role_name, args.timeout_seconds)
    except KeyboardInterrupt:
        return 130
    except BridgeStatusError as error:
        print(
            f"[virtual_carla] ERROR: physical 4WS preflight: {error}",
            file=sys.stderr,
        )
        return 1
    except Exception as error:  # Unexpected ROS data must also fail closed.
        print(
            "[virtual_carla] ERROR: physical 4WS preflight failed closed: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    print(format_success(actor_id, role_name, args.actor_id_only))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
