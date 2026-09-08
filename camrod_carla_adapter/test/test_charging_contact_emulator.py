"""Contracts for the pose-verified CARLA charging contact."""

from dataclasses import replace
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from camrod_carla_adapter import charging_contact_emulator_node as emulator

from camrod_carla_adapter.charging_contact_emulator_node import (
    ChargingStation,
    contact_candidate,
    ContactConfig,
    ContactSample,
    load_charging_station,
    validate_contact_config,
)
import pytest


STATION = ChargingStation(x_m=-14.2347, y_m=39.7863)
GOOD = ContactSample(
    parking_state="WAITING_FOR_CHARGING",
    x_m=-14.20,
    y_m=39.80,
    speed_mps=0.01,
    pose_age_s=0.1,
    odometry_age_s=0.1,
    parking_method="apriltag",
    parking_status_age_s=0.1,
)

RESTARTED_AT_STATION = replace(
    GOOD,
    parking_state="IDLE",
    planning_state="WAIT_DZ",
    parking_status_age_s=0.2,
    planning_state_age_s=0.1,
)


def test_station_loader_accepts_type_or_explicit_id(tmp_path):
    path = tmp_path / "drop_zones.yaml"
    path.write_text(
        "drop_zones:\n"
        "  - id: dz_area_2320\n"
        "    type: drop_zone\n"
        "    x: -14.2347\n"
        "    y: 39.7863\n",
        encoding="utf-8",
    )
    assert load_charging_station(path) == STATION
    assert load_charging_station(path, "dz_area_2320") == STATION


@pytest.mark.parametrize(
    "sample",
    [
        replace(GOOD, parking_state="REVERSE_APPROACH"),
        replace(GOOD, x_m=-13.0),
        replace(GOOD, speed_mps=0.051),
        replace(GOOD, pose_age_s=0.501),
        replace(GOOD, odometry_age_s=0.501),
        replace(GOOD, x_m=math.nan),
    ],
)
def test_contact_fails_closed_without_complete_fresh_stopped_evidence(sample):
    assert contact_candidate(sample, STATION) is False


def test_waiting_or_parked_vehicle_at_station_is_a_contact_candidate():
    assert contact_candidate(GOOD, STATION) is True
    assert contact_candidate(
        replace(GOOD, parking_state="WAITING_FOR_CHARGING"), STATION
    )
    assert contact_candidate(replace(GOOD, parking_state="PARKED"), STATION)


def test_fresh_apriltag_terminal_does_not_require_a_planning_heartbeat():
    assert contact_candidate(
        replace(
            GOOD,
            planning_state="RUNNING",
            planning_state_age_s=math.inf,
        ),
        STATION,
    )


@pytest.mark.parametrize("method", ["", None, "none", "unknown", "reverse", "apriltag"])
def test_restart_idle_wait_dz_never_manufactures_contact(method):
    assert not contact_candidate(replace(RESTARTED_AT_STATION, parking_method=method), STATION)


def test_observed_failed_dock_stop_pose_cannot_be_relabelled_charging_after_restart():
    station = ChargingStation(x_m=-11.3585, y_m=40.0901)
    sample = replace(
        RESTARTED_AT_STATION, x_m=-11.25674, y_m=39.95635,
        speed_mps=1.2e-7, parking_method="",
    )
    assert math.hypot(sample.x_m - station.x_m, sample.y_m - station.y_m) < 0.35
    assert not contact_candidate(sample, station)


@pytest.mark.parametrize("method", ["", None, "none", "unknown", "reverse"])
@pytest.mark.parametrize("phase", ["WAITING_FOR_CHARGING", "PARKED"])
def test_terminal_phase_without_exact_apriltag_owner_is_not_contact(method, phase):
    assert not contact_candidate(replace(GOOD, parking_method=method, parking_state=phase), STATION)


@pytest.mark.parametrize("age", [None, "bad", -0.001, 2.001, math.inf, math.nan])
def test_apriltag_terminal_heartbeat_must_be_fresh(age):
    assert not contact_candidate(replace(GOOD, parking_status_age_s=age), STATION)


@pytest.mark.parametrize("phase", ["WAIT_FOR_CHARGING", "ERROR", "IDLE", "TAG_GUIDED_REVERSE",
                                     "RETRY_FORWARD_EXIT", "FINAL_YAW_ALIGNMENT"])
def test_nonterminal_or_reverse_alias_never_asserts_contact(phase):
    assert not contact_candidate(replace(GOOD, parking_state=phase), STATION)


@pytest.mark.parametrize("phase", ["IDLE", "PARKED", "WAIT_FOR_CHARGING"])
def test_dispatcher_reverse_owner_never_synthesizes_docking_contact(phase):
    assert not contact_candidate(
        replace(RESTARTED_AT_STATION, parking_state=phase, parking_method="reverse"),
        STATION,
    )


def test_dispatcher_apriltag_owner_requires_measured_contact():
    assert contact_candidate(replace(GOOD, parking_method="apriltag"), STATION)
    assert not contact_candidate(
        replace(GOOD, parking_method="apriltag", x_m=-13.0), STATION,
    )


@pytest.mark.parametrize(
    "sample",
    [
        replace(RESTARTED_AT_STATION, parking_state="REVERSE_APPROACH"),
        replace(RESTARTED_AT_STATION, planning_state="RUNNING"),
        replace(RESTARTED_AT_STATION, parking_status_age_s=2.001),
        replace(RESTARTED_AT_STATION, planning_state_age_s=2.001),
        replace(RESTARTED_AT_STATION, parking_status_age_s=-0.001),
        replace(RESTARTED_AT_STATION, planning_state_age_s=math.nan),
        replace(RESTARTED_AT_STATION, speed_mps=0.051),
        replace(RESTARTED_AT_STATION, x_m=-13.0),
    ],
)
def test_restart_recovery_fails_closed_without_every_idle_contact_input(sample):
    assert contact_candidate(sample, STATION) is False


@pytest.mark.parametrize(
    "field",
    (
        "position_tolerance_m",
        "speed_tolerance_mps",
        "pose_timeout_s",
        "odometry_timeout_s",
        "state_timeout_s",
        "dwell_s",
    ),
)
def test_nonpositive_or_nonfinite_contact_thresholds_are_rejected(field):
    with pytest.raises(ValueError):
        validate_contact_config(replace(ContactConfig(), **{field: 0.0}))
    with pytest.raises(ValueError):
        validate_contact_config(replace(ContactConfig(), **{field: math.nan}))


def test_loader_rejects_ambiguous_or_missing_station(tmp_path):
    ambiguous = tmp_path / "ambiguous.yaml"
    ambiguous.write_text(
        "drop_zones:\n"
        "  - {id: a, type: drop_zone, x: 0, y: 0}\n"
        "  - {id: b, type: drop_zone, x: 1, y: 1}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="exactly one"):
        load_charging_station(ambiguous)
    with pytest.raises(ValueError, match="regular file"):
        load_charging_station(tmp_path / "missing.yaml")


def test_real_timer_preserves_dwell_but_restart_idle_never_asserts(monkeypatch):
    # Exercise the production timer without creating a ROS node or publishing
    # on a live topic. A successful Dock still needs the unchanged full dwell.
    monkeypatch.setattr(emulator, "Bool", SimpleNamespace)
    now = [0.0]
    current = [replace(RESTARTED_AT_STATION, parking_method="")]
    outputs = []
    node = SimpleNamespace(
        _now_s=lambda: now[0], _snapshot=lambda _: current[0],
        station=STATION, config=ContactConfig(), _lock=threading.Lock(),
        _contact=False, _candidate_since_s=None,
        publisher=SimpleNamespace(publish=lambda msg: outputs.append(msg.data)),
        get_logger=lambda: SimpleNamespace(info=Mock()),
    )
    for now[0] in [0., 1., 10., 60.]:
        emulator.CarlaChargingContactEmulatorNode._on_timer(node)
    assert outputs == [False, False, False, False]
    assert node._candidate_since_s is None
    current[0] = GOOD
    for now[0] in [100., 100.999, 101.]:
        emulator.CarlaChargingContactEmulatorNode._on_timer(node)
    assert outputs[-3:] == [False, False, True]
    current[0] = replace(GOOD, parking_method="")
    now[0] = 101.1
    emulator.CarlaChargingContactEmulatorNode._on_timer(node)
    assert outputs[-1] is False
    assert node._candidate_since_s is None
    current[0] = GOOD
    now[0] = 102.
    emulator.CarlaChargingContactEmulatorNode._on_timer(node)
    assert outputs[-1] is False  # Must satisfy a new complete dwell.


def test_dispatcher_callback_clears_previous_apriltag_identity_on_restart():
    node = SimpleNamespace(_lock=threading.Lock(), _now_s=lambda: 123.)
    emulator.CarlaChargingContactEmulatorNode._on_parking_status(node, SimpleNamespace(
        operating_state="PARKED", message="parking_method=apriltag phase=PARKED"))
    assert node._parking_method == "apriltag"
    emulator.CarlaChargingContactEmulatorNode._on_parking_status(node, SimpleNamespace(
        operating_state="IDLE", message="phase=IDLE"))
    assert node._parking_method == ""
    assert node._parking_state == "IDLE"
