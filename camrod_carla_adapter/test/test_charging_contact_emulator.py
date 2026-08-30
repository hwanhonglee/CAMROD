"""Contracts for the pose-verified CARLA charging contact."""

from dataclasses import replace
import math

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
    parking_state="WAIT_FOR_CHARGING",
    x_m=-14.20,
    y_m=39.80,
    speed_mps=0.01,
    pose_age_s=0.1,
    odometry_age_s=0.1,
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
    assert contact_candidate(replace(GOOD, parking_state="PARKED"), STATION)


def test_terminal_parking_evidence_keeps_existing_contract_without_planning():
    assert contact_candidate(
        replace(
            GOOD,
            planning_state="RUNNING",
            parking_status_age_s=math.inf,
            planning_state_age_s=math.inf,
        ),
        STATION,
    )


def test_restart_recovers_only_from_fresh_idle_wait_dz_at_station():
    assert contact_candidate(RESTARTED_AT_STATION, STATION)


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
