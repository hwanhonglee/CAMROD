"""HJ_261120 - Robot UI site-PIN verify decision (store + per-site attempt limit)."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.site_access_store import AttemptLimiter, SiteAccessStore  # noqa: E402
from camrod_ui.site_pin_api import evaluate_site_pin  # noqa: E402


def _store(tmp_path):
    store = SiteAccessStore(tmp_path / "site_access.json", ["B1", "B2"])
    store.ensure_initialized()
    store.set_pin("B1", "1234", "test")
    return store


def test_correct_pin_is_accepted_case_insensitive_site(tmp_path):
    status, body = evaluate_site_pin(_store(tmp_path), AttemptLimiter(5, 60), {"site": "b1", "pin": "1234"})
    assert (status, body) == (200, {"ok": True})


def test_wrong_pin_reports_remaining_attempts(tmp_path):
    status, body = evaluate_site_pin(_store(tmp_path), AttemptLimiter(5, 60), {"site": "B1", "pin": "0000"})
    assert status == 403
    assert body == {"ok": False, "error": "wrong_pin", "remaining_attempts": 4}


def test_invalid_input_is_400(tmp_path):
    store, limiter = _store(tmp_path), AttemptLimiter(5, 60)
    for payload in (None, [], {"site": "B1"}, {"site": "B1", "pin": "12a4"},
                    {"site": "B1", "pin": "12345"}, {"site": "", "pin": "1234"}, {"site": "B1", "pin": 1234}):
        assert evaluate_site_pin(store, limiter, payload)[0] == 400


def test_unknown_site_is_wrong_pin(tmp_path):
    assert evaluate_site_pin(_store(tmp_path), AttemptLimiter(5, 60), {"site": "C9", "pin": "1234"})[0] == 403


def test_five_failures_lock_the_site_even_for_the_right_pin(tmp_path):
    now = [0.0]
    store, limiter = _store(tmp_path), AttemptLimiter(5, 60, clock=lambda: now[0])
    for _ in range(4):
        assert evaluate_site_pin(store, limiter, {"site": "B1", "pin": "9999"})[0] == 403
    status, body = evaluate_site_pin(store, limiter, {"site": "B1", "pin": "9999"})
    assert status == 429 and body["error"] == "locked" and body["locked_s"] == 60
    assert evaluate_site_pin(store, limiter, {"site": "B1", "pin": "1234"})[0] == 429
    # Another site is unaffected.
    assert evaluate_site_pin(store, limiter, {"site": "B2", "pin": "0000"})[0] == 403
    now[0] = 61.0
    assert evaluate_site_pin(store, limiter, {"site": "B1", "pin": "1234"})[0] == 200


def test_missing_store_is_503():
    assert evaluate_site_pin(None, AttemptLimiter(5, 60), {"site": "B1", "pin": "1234"})[0] == 503
