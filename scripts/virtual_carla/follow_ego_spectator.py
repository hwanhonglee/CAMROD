#!/usr/bin/env python3
"""Keep CARLA's visual-only spectator camera centred on the live Ranger.

The helper never ticks the world, controls an actor, publishes ROS data, or
spawns/destroys anything.  It only updates the built-in spectator transform
after bridge-owned snapshots arrive.  An exact actor id, blueprint id and
role name are required so a hot respawn cannot silently change the evidence
subject.
"""

from __future__ import annotations

import argparse
import math
import sys
import time


EXPECTED_TYPE_ID = "vehicle.ranger.default"


def chase_transform_values(
    actor_transform,
    distance_m: float,
    side_m: float,
    height_m: float,
    target_height_m: float,
) -> dict[str, float]:
    """Return a rear three-quarter camera pose looking at the actor centre."""
    yaw_rad = math.radians(float(actor_transform.rotation.yaw))
    forward_x, forward_y = math.cos(yaw_rad), math.sin(yaw_rad)
    right_x, right_y = -forward_y, forward_x
    actor_location = actor_transform.location
    camera_x = float(actor_location.x) - distance_m * forward_x + side_m * right_x
    camera_y = float(actor_location.y) - distance_m * forward_y + side_m * right_y
    camera_z = float(actor_location.z) + height_m
    target_x = float(actor_location.x)
    target_y = float(actor_location.y)
    target_z = float(actor_location.z) + target_height_m
    look_x = target_x - camera_x
    look_y = target_y - camera_y
    look_z = target_z - camera_z
    horizontal = math.hypot(look_x, look_y)
    if horizontal <= 1.0e-9:
        raise ValueError("spectator camera cannot occupy the target vertical axis")
    return {
        "x": camera_x,
        "y": camera_y,
        "z": camera_z,
        "pitch": math.degrees(math.atan2(look_z, horizontal)),
        "yaw": math.degrees(math.atan2(look_y, look_x)),
        "roll": 0.0,
    }


def _actor_identity(actor) -> tuple[int, str, str]:
    attributes = getattr(actor, "attributes", {}) or {}
    return (
        int(getattr(actor, "id", 0)),
        str(getattr(actor, "type_id", "")),
        str(attributes.get("role_name", "")),
    )


def select_exact_actor(world, actor_id: int, type_id: str, role_name: str):
    matches = []
    for actor in world.get_actors():
        identity = _actor_identity(actor)
        if identity == (actor_id, type_id, role_name):
            matches.append(actor)
    if len(matches) != 1:
        raise RuntimeError(
            "expected exactly one actor identity id={} type={} role={}, found {}".format(
                actor_id, type_id, role_name, len(matches)
            )
        )
    return matches[0]


def wait_for_exact_actor(
    world,
    actor_id: int,
    type_id: str,
    role_name: str,
    discovery_timeout_s: float,
    tick_timeout_s: float,
    monotonic=None,
):
    """Wait for an exact actor identity without ever advancing the world.

    A freshly connected CARLA client can transiently observe an empty actor
    inventory even though the spawn-side preflight already resolved the
    Ranger id.  Only bridge/pacer-owned snapshots are observed here via
    ``wait_for_tick``; this helper deliberately never calls ``world.tick``.
    """
    if discovery_timeout_s <= 0.0 or tick_timeout_s <= 0.0:
        raise ValueError("actor discovery and tick timeouts must be positive")
    if monotonic is None:
        monotonic = time.monotonic

    deadline = monotonic() + discovery_timeout_s
    inventory_checks = 0
    snapshots_waited = 0
    while True:
        inventory_checks += 1
        matches = [
            actor
            for actor in world.get_actors()
            if _actor_identity(actor) == (actor_id, type_id, role_name)
        ]
        if len(matches) == 1:
            return matches[0], inventory_checks, snapshots_waited
        if len(matches) > 1:
            raise RuntimeError(
                "actor identity is ambiguous id={} type={} role={}: found {}".format(
                    actor_id, type_id, role_name, len(matches)
                )
            )

        remaining_s = deadline - monotonic()
        if remaining_s <= 0.0:
            raise RuntimeError(
                "timed out after {:.3f}s waiting for exact actor identity "
                "id={} type={} role={} ({} inventory checks, {} snapshots)".format(
                    discovery_timeout_s,
                    actor_id,
                    type_id,
                    role_name,
                    inventory_checks,
                    snapshots_waited,
                )
            )
        snapshot = world.wait_for_tick(min(tick_timeout_s, remaining_s))
        if snapshot is not None:
            snapshots_waited += 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--timeout-s", default=3.0, type=float)
    parser.add_argument("--actor-discovery-timeout-s", default=15.0, type=float)
    parser.add_argument("--actor-id", required=True, type=int)
    parser.add_argument("--type-id", default=EXPECTED_TYPE_ID)
    parser.add_argument("--role-name", default="ego_vehicle")
    # Keep the evidence camera close to the Ranger and almost inside the
    # driven corridor.  The former 5.0 m / 1.8 m rear-quarter pose entered
    # roadside tree crowns around B11 and made the robot too small in the
    # desktop evidence.  These values are derived from the live Ranger
    # 1.553 x 1.120 x 1.324 m bounding box; the camera remains above the roof
    # and looks at the bounding-box centre without changing any actor state.
    parser.add_argument("--distance-m", default=3.0, type=float)
    parser.add_argument("--side-m", default=0.65, type=float)
    parser.add_argument("--height-m", default=1.65, type=float)
    parser.add_argument("--target-height-m", default=0.65, type=float)
    parser.add_argument("--log-every-frames", default=200, type=int)
    return parser


def run(args, carla_module=None) -> int:
    for name in (
        "timeout_s",
        "actor_discovery_timeout_s",
        "distance_m",
        "height_m",
    ):
        if float(getattr(args, name)) <= 0.0:
            raise ValueError("--{} must be positive".format(name.replace("_", "-")))
    if args.actor_id <= 0 or args.log_every_frames <= 0:
        raise ValueError("actor id and log interval must be positive")
    if carla_module is None:
        import carla as carla_module

    client = carla_module.Client(args.host, args.port)
    client.set_timeout(args.timeout_s)
    world = client.get_world()
    actor, inventory_checks, snapshots_waited = wait_for_exact_actor(
        world,
        args.actor_id,
        args.type_id,
        args.role_name,
        args.actor_discovery_timeout_s,
        args.timeout_s,
    )
    spectator = world.get_spectator()
    initial_identity = _actor_identity(actor)
    print(
        "[virtual_carla_spectator] ready actor_id={} type={} role={} endpoint={}:{} "
        "inventory_checks={} snapshots_waited={}; visual-only, no world ticks or "
        "vehicle commands".format(
            initial_identity[0], initial_identity[1], initial_identity[2],
            args.host, args.port, inventory_checks, snapshots_waited,
        ),
        flush=True,
    )
    updates = 0
    while True:
        snapshot = world.wait_for_tick(args.timeout_s)
        if snapshot is None:
            raise RuntimeError("timed out waiting for a bridge-owned CARLA snapshot")
        if not bool(getattr(actor, "is_alive", False)):
            raise RuntimeError("bound Ranger actor was destroyed; refusing hot rebind")
        if _actor_identity(actor) != initial_identity:
            raise RuntimeError("bound Ranger actor identity changed")
        values = chase_transform_values(
            actor.get_transform(),
            args.distance_m,
            args.side_m,
            args.height_m,
            args.target_height_m,
        )
        spectator.set_transform(
            carla_module.Transform(
                carla_module.Location(x=values["x"], y=values["y"], z=values["z"]),
                carla_module.Rotation(
                    pitch=values["pitch"], yaw=values["yaw"], roll=values["roll"]
                ),
            )
        )
        updates += 1
        if updates == 1 or updates % args.log_every_frames == 0:
            print(
                "[virtual_carla_spectator] frame={} updates={} camera=({:.2f},{:.2f},{:.2f})".format(
                    int(snapshot.frame), updates, values["x"], values["y"], values["z"]
                ),
                flush=True,
            )


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except KeyboardInterrupt:
        print("[virtual_carla_spectator] stopped", flush=True)
        return 0
    except Exception as exc:
        print(
            "[virtual_carla_spectator] ERROR: {}: {}".format(type(exc).__name__, exc),
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
