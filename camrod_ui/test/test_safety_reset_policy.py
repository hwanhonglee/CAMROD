"""HH_261024 - Fail-closed, side-effect-free reset admission boundaries."""

import base64
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.safety_reset import (  # noqa: E402
    admin_authorized,
    cancellations_complete,
    stationary_reset_blockers,
)


def _basic(value: str, scheme: str = "Basic") -> str:
    return f"{scheme} " + base64.b64encode(value.encode("utf-8")).decode("ascii")


@pytest.mark.parametrize(
    ("user", "password"),
    [("admin", "1234"), ("관리자", "암호-솔나리"), ("operator", "password:with:colons")],
)
def test_admin_credentials_allow_exact_utf8_and_password_colons(user, password):
    assert admin_authorized(_basic(f"{user}:{password}"), user, password) is True
    assert admin_authorized(_basic(f"{user}:{password}", "bAsIc"), user, password) is True
    assert admin_authorized(_basic(f"{user}:{password}!"), user, password) is False
    assert admin_authorized(_basic(f"wrong-{user}:{password}"), user, password) is False


@pytest.mark.parametrize(
    "header",
    [None, b"Basic YTpi", 1, "", "Bearer secret", "Basic", "Basic ",
     "Basic !!!", "Basic /w==", "Basic bm9fY29sb24=", "Basic " + "A" * 4097],
)
def test_admin_credentials_reject_malformed_or_oversize_headers(header):
    assert admin_authorized(header, "admin", "1234") is False


@pytest.mark.parametrize(("user", "password"), [("", "1234"), ("admin", ""), (None, "1234"), ("admin", None)])
def test_admin_credentials_fail_closed_for_empty_configured_secrets(user, password):
    assert admin_authorized(_basic("admin:1234"), user, password) is False


def _observation(**changes):
    observation = {
        "source_stamp_s": 100.0,
        "received_monotonic_s": 500.0,
        "speed_mps": 0.0,
        "yaw_rate_radps": 0.0,
        "estop": False,
        "error_code": 0,
        "vehicle_state": 0,
    }
    observation.update(changes)
    return observation


def _blockers(observation=None, *, now_ros=100.0, now_monotonic=500.0):
    return stationary_reset_blockers(observation, now_ros, now_monotonic)


@pytest.mark.parametrize("speed", [-0.03, 0.0, 0.03])
@pytest.mark.parametrize("yaw_rate", [-0.03, 0.0, 0.03])
def test_stationary_exact_motion_and_half_second_boundaries_are_accepted(speed, yaw_rate):
    # HH_261024 - Match the final control gate exactly, including equality;
    # a stale zero or a merely transmitted zero does not count as standstill.
    observation = _observation(
        source_stamp_s=99.5, received_monotonic_s=499.5,
        speed_mps=speed, yaw_rate_radps=yaw_rate,
    )
    original = dict(observation)
    assert _blockers(observation) == []
    assert observation == original


@pytest.mark.parametrize("field", ["speed_mps", "yaw_rate_radps"])
@pytest.mark.parametrize("value", [-0.0300001, 0.0300001])
def test_stationary_motion_above_either_limit_is_blocked(field, value):
    assert _blockers(_observation(**{field: value})) == ["robot_not_stationary"]


@pytest.mark.parametrize(
    "changes",
    [
        {"source_stamp_s": 99.4999999},
        {"source_stamp_s": 100.0000001},
        {"source_stamp_s": 0.0},
        {"source_stamp_s": -1.0},
        {"received_monotonic_s": 499.4999999},
        {"received_monotonic_s": 500.0000001},
    ],
)
def test_stationary_rejects_stale_zero_uninitialized_and_future_timestamps(changes):
    assert _blockers(_observation(**changes)) == ["motion_feedback_missing_or_stale"]


@pytest.mark.parametrize("field", ["source_stamp_s", "received_monotonic_s", "speed_mps", "yaw_rate_radps"])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, None, "not-a-number"])
def test_stationary_rejects_nonfinite_and_invalid_observations(field, value):
    assert _blockers(_observation(**{field: value})) == ["invalid_motion_feedback"]


@pytest.mark.parametrize("clock", ["now_ros", "now_monotonic"])
@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_stationary_rejects_nonfinite_clocks(clock, value):
    assert _blockers(_observation(), **{clock: value}) == ["invalid_motion_feedback"]


@pytest.mark.parametrize("observation", [None, [], "zero", 0, False])
def test_stationary_requires_a_measured_observation(observation):
    assert _blockers(observation) == ["motion_feedback_missing_or_stale"]


@pytest.mark.parametrize("missing", list(_observation()))
def test_stationary_missing_any_required_source_is_invalid(missing):
    observation = _observation()
    observation.pop(missing)
    assert _blockers(observation) == ["invalid_motion_feedback"]


@pytest.mark.parametrize("estop", [True, 0, 1, None, "false"])
def test_stationary_requires_literal_false_estop(estop):
    assert _blockers(_observation(estop=estop)) == ["estop_active"]


@pytest.mark.parametrize("field", ["error_code", "vehicle_state"])
@pytest.mark.parametrize("value", [1, -1, None, "0"])
def test_stationary_rejects_each_platform_fault_source(field, value):
    assert _blockers(_observation(**{field: value})) == ["platform_fault_active"]


def test_stationary_reports_concurrent_blockers_without_mutation():
    observation = _observation(speed_mps=0.1, estop=True, error_code=2, vehicle_state=1)
    assert _blockers(observation) == ["robot_not_stationary", "estop_active", "platform_fault_active"]
    assert observation["speed_mps"] == 0.1
    assert observation["estop"] is True


class _Future:
    def __init__(self, result=None, *, done=True, raises=False):
        self.response = result
        self.completed = done
        self.raises = raises
        self.result_calls = 0

    def done(self):
        return self.completed

    def result(self):
        self.result_calls += 1
        if self.raises:
            raise RuntimeError("cancel transport unavailable")
        return self.response


def _ack(return_code=0, goals=None):
    return SimpleNamespace(return_code=return_code, goals_canceling=[] if goals is None else goals)


@pytest.mark.parametrize("nav_status", [0, 4, 5, 6])
@pytest.mark.parametrize("return_code", [0, 3])
def test_cancellation_requires_all_acks_and_observed_idle_or_terminal_nav(nav_status, return_code):
    futures = [_Future(_ack(return_code)), _Future(_ack(0))]
    assert cancellations_complete(futures, 2, nav_status) is True
    assert [future.result_calls for future in futures] == [1, 1]


@pytest.mark.parametrize("nav_status", [1, 2, 3])
def test_cancellation_ack_cannot_release_executing_accepted_or_canceling_nav(nav_status):
    future = _Future(_ack())
    assert cancellations_complete([future], 1, nav_status) is False
    assert future.result_calls == 0


@pytest.mark.parametrize(
    ("futures", "expected"),
    [([], 0), ([], 1), ([_Future(_ack())], 2), ([_Future(_ack()), _Future(_ack())], 1),
     (None, 1), ([], None), ([_Future(_ack())], "1"), ([_Future(_ack())], True),
     ({"future": _Future(_ack())}, 1)],
)
def test_cancellation_missing_services_or_incomplete_future_inventory_never_succeeds(futures, expected):
    assert cancellations_complete(futures, expected, 0) is False


@pytest.mark.parametrize("nav_status", [None, -1, 7, 99, "0", [], False, True])
def test_cancellation_unknown_navigation_status_never_grants_release(nav_status):
    assert cancellations_complete([_Future(_ack())], 1, nav_status) is False


def test_cancellation_pending_future_is_not_read_or_treated_as_success():
    future = _Future(_ack(), done=False)
    assert cancellations_complete([future], 1, 0) is False
    assert future.result_calls == 0


@pytest.mark.parametrize("result", [None, object(), _ack(1), _ack(2), _ack(4), _ack(99), _ack(3, [object()]), _ack(0, [object()])])
def test_cancellation_rejects_missing_unknown_error_and_still_canceling_results(result):
    assert cancellations_complete([_Future(result)], 1, 0) is False


@pytest.mark.parametrize("nav_status", [0, 4, 5, 6])
@pytest.mark.parametrize("return_code", [0, 3])
def test_cancellation_nonempty_ack_is_not_terminal_even_with_terminal_nav_status(nav_status, return_code):
    # HH_261024 - A cancel response may only acknowledge starting cancellation.
    # Do not trust an older terminal navigation observation over this live ACK.
    assert cancellations_complete([_Future(_ack(return_code, [object()]))], 1, nav_status) is False


def test_cancellation_requires_explicit_empty_canceling_goal_inventory():
    for result in (
        SimpleNamespace(return_code=0),
        SimpleNamespace(return_code=0, goals_canceling=None),
    ):
        assert cancellations_complete([_Future(result)], 1, 0) is False


def test_cancellation_transport_errors_and_missing_futures_fail_closed():
    assert cancellations_complete([_Future(raises=True)], 1, 0) is False
    assert cancellations_complete([None], 1, 0) is False
