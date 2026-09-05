"""Source-level tests for the visual-only CARLA spectator helper."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "follow_ego_spectator.py"
)
SPEC = importlib.util.spec_from_file_location("follow_ego_spectator", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def transform(x=10.0, y=20.0, z=1.0, yaw=0.0):
    return SimpleNamespace(
        location=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(yaw=yaw),
    )


def test_default_evidence_camera_is_close_and_corridor_centred():
    args = MODULE.build_parser().parse_args(["--actor-id", "17"])
    assert args.distance_m == 3.0
    assert args.side_m == 0.65
    assert args.height_m == 1.65
    assert args.target_height_m == 0.65

    values = MODULE.chase_transform_values(
        transform(yaw=0.0),
        args.distance_m,
        args.side_m,
        args.height_m,
        args.target_height_m,
    )
    assert values["x"] == 7.0
    assert values["y"] == 20.65
    assert values["z"] == 2.65
    assert values["pitch"] < 0.0


def test_chase_pose_tracks_actor_heading_and_looks_at_actor():
    values = MODULE.chase_transform_values(
        transform(yaw=0.0), 5.0, 2.0, 3.0, 0.5
    )
    assert values["x"] == 5.0
    assert values["y"] == 22.0
    assert values["z"] == 4.0
    assert math.isclose(values["yaw"], math.degrees(math.atan2(-2.0, 5.0)))
    assert values["pitch"] < 0.0

    rotated = MODULE.chase_transform_values(
        transform(yaw=90.0), 5.0, 2.0, 3.0, 0.5
    )
    assert math.isclose(rotated["x"], 8.0, abs_tol=1.0e-12)
    assert math.isclose(rotated["y"], 15.0, abs_tol=1.0e-12)


def test_exact_actor_selection_rejects_hot_rebind_or_wrong_identity():
    correct = SimpleNamespace(
        id=7,
        type_id="vehicle.ranger.default",
        attributes={"role_name": "ego_vehicle"},
    )
    wrong_role = SimpleNamespace(
        id=8,
        type_id="vehicle.ranger.default",
        attributes={"role_name": "other"},
    )
    world = SimpleNamespace(get_actors=lambda: [correct, wrong_role])
    assert MODULE.select_exact_actor(
        world, 7, "vehicle.ranger.default", "ego_vehicle"
    ) is correct

    try:
        MODULE.select_exact_actor(
            world, 8, "vehicle.ranger.default", "ego_vehicle"
        )
    except RuntimeError as error:
        assert "found 0" in str(error)
    else:
        raise AssertionError("wrong role was accepted")


def test_actor_discovery_waits_for_bridge_owned_snapshot_after_empty_inventory():
    correct = SimpleNamespace(
        id=17,
        type_id="vehicle.ranger.default",
        attributes={"role_name": "ego_vehicle"},
    )

    class World:
        def __init__(self):
            self.inventory_calls = 0
            self.wait_timeouts = []

        def get_actors(self):
            self.inventory_calls += 1
            return [] if self.inventory_calls == 1 else [correct]

        def wait_for_tick(self, timeout_s):
            self.wait_timeouts.append(timeout_s)
            return SimpleNamespace(frame=41)

    world = World()
    actor, checks, snapshots = MODULE.wait_for_exact_actor(
        world,
        17,
        "vehicle.ranger.default",
        "ego_vehicle",
        discovery_timeout_s=2.0,
        tick_timeout_s=0.5,
    )
    assert actor is correct
    assert checks == 2
    assert snapshots == 1
    assert world.wait_timeouts == [0.5]


def test_actor_discovery_timeout_is_bounded_and_keeps_exact_identity():
    wrong_role = SimpleNamespace(
        id=17,
        type_id="vehicle.ranger.default",
        attributes={"role_name": "other"},
    )
    clock = [100.0]

    class World:
        def __init__(self):
            self.wait_calls = 0

        def get_actors(self):
            return [wrong_role]

        def wait_for_tick(self, timeout_s):
            self.wait_calls += 1
            clock[0] += timeout_s
            return SimpleNamespace(frame=self.wait_calls)

    world = World()
    try:
        MODULE.wait_for_exact_actor(
            world,
            17,
            "vehicle.ranger.default",
            "ego_vehicle",
            discovery_timeout_s=0.5,
            tick_timeout_s=0.2,
            monotonic=lambda: clock[0],
        )
    except RuntimeError as error:
        message = str(error)
        assert "timed out after 0.500s" in message
        assert "id=17 type=vehicle.ranger.default role=ego_vehicle" in message
    else:
        raise AssertionError("wrong-role actor was accepted")
    assert world.wait_calls == 3


def test_actor_discovery_rejects_duplicate_exact_identity_immediately():
    exact = SimpleNamespace(
        id=17,
        type_id="vehicle.ranger.default",
        attributes={"role_name": "ego_vehicle"},
    )
    world = SimpleNamespace(
        get_actors=lambda: [exact, exact],
        wait_for_tick=lambda _timeout: (_ for _ in ()).throw(
            AssertionError("ambiguous identity must fail before waiting")
        ),
    )
    try:
        MODULE.wait_for_exact_actor(
            world,
            17,
            "vehicle.ranger.default",
            "ego_vehicle",
            discovery_timeout_s=1.0,
            tick_timeout_s=0.1,
        )
    except RuntimeError as error:
        assert "ambiguous" in str(error)
        assert "found 2" in str(error)
    else:
        raise AssertionError("duplicate exact identity was accepted")
