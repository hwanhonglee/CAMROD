#!/usr/bin/env python3
"""Read-only CARLA preflight for the exact CAMROD Ranger actor."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time
from typing import Sequence


EXACT_RANGER_BLUEPRINT = "vehicle.ranger.default"
ACTOR_POLL_INTERVAL_SECONDS = 0.2
LIFECYCLE_HINT = (
    "Required lifecycle order: server -> bridge -> spawn -> camrod. "
    "Wait for each preceding stage to report success before starting camrod."
)


class ActorPreflightError(RuntimeError):
    """Expected, user-facing CARLA readiness failure."""


def _regular_file(path: Path, label: str) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_file():
        raise ActorPreflightError(f"{label} is not a regular file: {resolved}")
    return resolved


def _directory(path: Path, label: str) -> Path:
    resolved = path.expanduser().resolve()
    if not resolved.is_dir():
        raise ActorPreflightError(f"{label} is not a directory: {resolved}")
    return resolved


def _is_under(path: str, root: Path) -> bool:
    resolved_path = os.path.realpath(path)
    resolved_root = os.path.realpath(str(root))
    return resolved_path.startswith(resolved_root + os.sep)


def _load_bound_carla(accepted_egg: Path, private_cache: Path):
    environment_cache = os.environ.get("PYTHON_EGG_CACHE", "")
    if not environment_cache or not os.path.isabs(environment_cache):
        raise ActorPreflightError(
            "PYTHON_EGG_CACHE must select the private absolute preflight cache"
        )
    if os.path.realpath(environment_cache) != os.path.realpath(str(private_cache)):
        raise ActorPreflightError(
            "PYTHON_EGG_CACHE does not match the private preflight cache"
        )

    try:
        import carla  # pylint: disable=import-outside-toplevel
    except Exception as error:
        raise ActorPreflightError(
            "cannot import the gate-bound CARLA Python API: "
            f"{type(error).__name__}: {error}"
        ) from None

    module_origin = str(getattr(carla, "__file__", "") or "")
    if not module_origin or not _is_under(module_origin, accepted_egg):
        raise ActorPreflightError(
            "CARLA Python API resolved outside the gate-bound egg: "
            f"{module_origin or '<unknown>'} (expected under {accepted_egg})"
        )
    return carla


def _actor_identity(actor) -> tuple[str, str, int]:
    type_id = str(getattr(actor, "type_id", ""))
    attributes = getattr(actor, "attributes", {})
    role_name = ""
    if hasattr(attributes, "get"):
        role_name = str(attributes.get("role_name", ""))
    actor_id = getattr(actor, "id", -1)
    try:
        actor_id = int(actor_id)
    except (TypeError, ValueError):
        actor_id = -1
    return type_id, role_name, actor_id


def check_actor(
    carla,
    host: str,
    port: int,
    role_name: str,
    timeout_seconds: float,
    expected_count: int = 1,
) -> int:
    endpoint = f"{host}:{port}"
    deadline = time.monotonic() + timeout_seconds
    client = None
    attempts = 0
    inventory_polls = 0
    last_endpoint_error = ""

    while time.monotonic() < deadline:
        attempts += 1
        try:
            if client is None:
                client = carla.Client(host, port)
            remaining = max(0.001, deadline - time.monotonic())
            client.set_timeout(min(1.0, remaining))
            world = client.get_world()
            actors = list(world.get_actors())
            inventory_polls += 1
        except Exception as error:
            last_endpoint_error = f"{type(error).__name__}: {error}"
        else:
            role_actors = []
            for actor in actors:
                type_id, actor_role, actor_id = _actor_identity(actor)
                if actor_role == role_name:
                    role_actors.append((actor_id, type_id))

            if expected_count == 0:
                if not role_actors:
                    return 0
                identities = ",".join(
                    f"{actor_id}:{type_id}"
                    for actor_id, type_id in sorted(role_actors)
                )
                raise ActorPreflightError(
                    f"found {len(role_actors)} pre-existing actor(s) using "
                    f"role_name={role_name!r} at {endpoint} "
                    f"(actor_id:type=[{identities}]); "
                    "refusing duplicate spawn. Stop CAMROD and the old spawn "
                    f"stage or restart CARLA before spawning again. {LIFECYCLE_HINT}"
                )
            if len(role_actors) == 1:
                actor_id, type_id = role_actors[0]
                if type_id != EXACT_RANGER_BLUEPRINT:
                    raise ActorPreflightError(
                        f"actor using required role_name={role_name!r} at "
                        f"{endpoint} has type={type_id!r}, expected exact "
                        f"{EXACT_RANGER_BLUEPRINT!r}; remove the conflicting "
                        f"ego actor and repeat the lifecycle. {LIFECYCLE_HINT}"
                    )
                if actor_id <= 0:
                    raise ActorPreflightError(
                        "exact Ranger actor reported an invalid actor id at "
                        f"{endpoint} (actor_id={actor_id!r}); restart CARLA "
                        f"and repeat the lifecycle. {LIFECYCLE_HINT}"
                    )
                return actor_id
            if len(role_actors) > 1:
                identities = ",".join(
                    f"{actor_id}:{type_id}"
                    for actor_id, type_id in sorted(role_actors)
                )
                raise ActorPreflightError(
                    f"found {len(role_actors)} actors using required "
                    f"role_name={role_name!r} at {endpoint} "
                    f"(actor_id:type=[{identities}]); remove every duplicate "
                    f"ego actor and run spawn exactly once. {LIFECYCLE_HINT}"
                )

        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(min(ACTOR_POLL_INTERVAL_SECONDS, remaining))

    if inventory_polls == 0:
        last_error = last_endpoint_error or "no successful actor inventory read"
        raise ActorPreflightError(
            f"cannot read CARLA endpoint {endpoint} within "
            f"{timeout_seconds:g}s after {attempts} attempt(s); "
            f"last error: {last_error}. {LIFECYCLE_HINT}"
        )

    endpoint_detail = ""
    if last_endpoint_error:
        endpoint_detail = f" Last transient endpoint error: {last_endpoint_error}."
    raise ActorPreflightError(
        "found 0 actors using the required Ranger role at "
        f"{endpoint} after {inventory_polls} actor inventory poll(s) within "
        f"{timeout_seconds:g}s (type={EXACT_RANGER_BLUEPRINT!r}, "
        f"role_name={role_name!r}); the spawn stage has not completed."
        f"{endpoint_detail} {LIFECYCLE_HINT}"
    )


def _port(value: str) -> int:
    try:
        result = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("port must be an integer") from None
    if not 1 <= result <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return result


def _positive_timeout(value: str) -> float:
    try:
        result = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("timeout must be a number") from None
    if not math.isfinite(result) or result <= 0.0 or result > 60.0:
        raise argparse.ArgumentTypeError(
            "timeout must be finite, positive, and at most 60 seconds"
        )
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", required=True, type=_port)
    parser.add_argument("--role-name", required=True)
    parser.add_argument("--accepted-python-egg", required=True, type=Path)
    parser.add_argument("--private-egg-cache", required=True, type=Path)
    parser.add_argument("--timeout-seconds", type=_positive_timeout, default=5.0)
    parser.add_argument(
        "--expected-count",
        type=int,
        choices=(0, 1),
        default=1,
        help="require zero actors before spawn or one actor before CAMROD",
    )
    parser.add_argument(
        "--actor-id-only",
        action="store_true",
        help="print only the positive CARLA actor id on success",
    )
    return parser


def format_success(
    actor_id: int,
    role_name: str,
    host: str,
    port: int,
    actor_id_only: bool,
    expected_count: int = 1,
) -> str:
    """Format either the human-readable or machine-readable success result."""
    if expected_count == 0:
        return (
            "[virtual_carla] Ranger actor inventory ready for spawn: "
            f"count=0 type={EXACT_RANGER_BLUEPRINT} "
            f"role_name={role_name} endpoint={host}:{port}"
        )
    if actor_id_only:
        return str(actor_id)
    return (
        "[virtual_carla] Ranger actor preflight ready: "
        f"actor_id={actor_id} type={EXACT_RANGER_BLUEPRINT} "
        f"role_name={role_name} endpoint={host}:{port}"
    )


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if not args.host.strip():
            raise ActorPreflightError("CARLA host must not be empty")
        if not args.role_name.strip():
            raise ActorPreflightError("CARLA role name must not be empty")
        if args.actor_id_only and args.expected_count != 1:
            raise ActorPreflightError(
                "--actor-id-only requires --expected-count 1"
            )
        accepted_egg = _regular_file(
            args.accepted_python_egg, "gate-bound CARLA Python egg"
        )
        private_cache = _directory(
            args.private_egg_cache, "private actor-preflight egg cache"
        )
        carla = _load_bound_carla(accepted_egg, private_cache)
        actor_id = check_actor(
            carla,
            args.host,
            args.port,
            args.role_name,
            args.timeout_seconds,
            args.expected_count,
        )
    except ActorPreflightError as error:
        print(f"[virtual_carla] ERROR: Ranger actor preflight: {error}", file=sys.stderr)
        return 1
    except Exception as error:  # Unexpected CARLA data must also fail closed.
        print(
            "[virtual_carla] ERROR: Ranger actor preflight failed closed: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    print(
        format_success(
            actor_id,
            args.role_name,
            args.host,
            args.port,
            args.actor_id_only,
            args.expected_count,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
