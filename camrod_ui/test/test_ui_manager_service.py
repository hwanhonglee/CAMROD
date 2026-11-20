"""HJ_261120 - manager_ui rules: login, forced first change, PIN edits, LAN guard, store."""

from pathlib import Path
import ipaddress
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.lan_guard import LanPolicy  # noqa: E402
from camrod_ui.site_access_store import AttemptLimiter, SiteAccessStore, valid_pin  # noqa: E402
from camrod_ui.ui_manager_node import ManagerService  # noqa: E402

SITES = ["B1", "B2", "B3"]


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def store(tmp_path, monkeypatch):
    monkeypatch.delenv("CAMROD_MANAGER_USER", raising=False)
    monkeypatch.delenv("CAMROD_MANAGER_PASSWORD", raising=False)
    s = SiteAccessStore(tmp_path / "site_access.json", SITES)
    s.ensure_initialized()
    return s


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def service(store, clock):
    return ManagerService(store, session_ttl_s=60, clock=clock)


def login(service, user="admin", pw="1234", client="192.168.200.5"):
    return service.login(user, pw, client)


def test_store_seeds_random_pins_and_is_private(store):
    pins = store.pins()
    assert sorted(pins) == SITES and all(valid_pin(p["pin"]) for p in pins.values())
    assert (store.path.stat().st_mode & 0o777) == 0o600
    # A second process sees the same PINs (no re-seeding).
    other = SiteAccessStore(store.path, SITES)
    other.ensure_initialized()
    assert {k: v["pin"] for k, v in other.pins().items()} == {k: v["pin"] for k, v in pins.items()}


def test_verify_pin_is_exact_and_four_digits(store):
    store.set_pin("B2", "0420", by="t")
    assert store.verify_pin("b2", "0420")
    for wrong in ("0421", "420", "04200", "abcd", "", None):
        assert not store.verify_pin("B2", wrong)
    assert not store.verify_pin("B9", "0420")


def test_default_account_must_be_changed_before_pins(service):
    status, body, token = login(service)
    assert status == 200 and token and body["must_change"] is True
    assert service.list_sites(token)[0] == 403
    assert service.set_pin(token, "B1", "1111")[0] == 403
    status, body = service.change_account(token, "1234", "woraksan", "camp2026", "camp2026")
    assert status == 200 and body["must_change"] is False and body["username"] == "woraksan"
    assert service.list_sites(token)[0] == 200


def test_wrong_login_locks_after_five_tries(service, clock):
    for _ in range(5):
        assert login(service, pw="nope")[0] == 401
    status, body, token = login(service)
    assert status == 429 and token is None and body["locked_s"] > 0
    clock.t += 301
    assert login(service)[0] == 200


def test_session_expires_and_logout(service, clock):
    _, _, token = login(service)
    assert service.session_info(token)["authenticated"]
    clock.t += 61
    assert not service.session_info(token)["authenticated"]
    _, _, token = login(service)
    service.logout(token)
    assert service.list_sites(token)[0] == 401


def ready(service):
    _, _, token = login(service)
    service.change_account(token, "1234", "admin2", "camp2026", "camp2026")
    return token


def test_set_and_reveal_pin_records_history(service, store):
    token = ready(service)
    assert service.set_pin(token, "B3", "12a4")[0] == 400
    assert service.set_pin(token, "B9", "1234")[0] == 404
    status, body = service.set_pin(token, "b3", "5190")
    assert status == 200 and body["site"] == "B3"
    assert service.reveal_pin(token, "B3")[1]["pin"] == "5190"
    assert store.verify_pin("B3", "5190")
    history = service.history(token)[1]["history"]
    assert history[0]["site"] == "B3" and history[0]["action"] == "pin_changed"
    # Listing never exposes PINs.
    assert all("pin" not in s for s in service.list_sites(token)[1]["sites"])


def test_account_change_rules(service):
    token = ready(service)
    assert service.change_account(token, "wrong", "admin3", "camp2027", "camp2027")[0] == 403
    assert service.change_account(token, "camp2026", "a", "camp2027", "camp2027")[1]["error"] == "invalid_username"
    assert service.change_account(token, "camp2026", "admin3", "123", "123")[1]["error"] == "weak_password"
    assert service.change_account(token, "camp2026", "admin3", "camp2027", "camp2028")[1]["error"] == "password_mismatch"
    other = login(service, user="admin2", pw="camp2026", client="10.0.0.9")[2]
    assert service.change_account(token, "camp2026", "admin3", "camp2027", "camp2027")[0] == 200
    # Other devices must log in again with the new account.
    assert service.list_sites(other)[0] == 401
    assert login(service, user="admin3", pw="camp2027")[0] == 200


def test_attempt_limiter_unlocks_after_timeout(clock):
    limiter = AttemptLimiter(2, 10, clock=clock)
    limiter.record("B1", False)
    assert limiter.remaining_lock_s("B1") == 0
    limiter.record("B1", False)
    assert limiter.remaining_lock_s("B1") > 0
    clock.t += 11
    assert limiter.remaining_lock_s("B1") == 0


def test_lan_policy_allows_only_own_subnets(monkeypatch):
    monkeypatch.delenv("CAMROD_UI_LAN_GUARD", raising=False)
    monkeypatch.delenv("CAMROD_UI_ALLOWED_CIDRS", raising=False)
    policy = LanPolicy(interface_networks=lambda: [ipaddress.ip_network("192.168.50.0/24")])
    assert policy.allows("127.0.0.1") and policy.allows("::1")
    assert policy.allows("192.168.50.23") and policy.allows("::ffff:192.168.50.23")
    assert not policy.allows("192.168.51.23") and not policy.allows("8.8.8.8")
    assert not policy.allows("") and not policy.allows(None)
    monkeypatch.setenv("CAMROD_UI_ALLOWED_CIDRS", "10.1.0.0/16")
    assert LanPolicy(interface_networks=lambda: []).allows("10.1.2.3")
    monkeypatch.setenv("CAMROD_UI_LAN_GUARD", "0")
    assert LanPolicy(interface_networks=lambda: []).allows("8.8.8.8")


def test_new_manager_password_needs_four_characters(service):
    _, _, token = login(service)
    assert service.change_account(token, "1234", "admin2", "abc", "abc")[1]["error"] == "weak_password"
    assert service.change_account(token, "1234", "admin2", "camp", "camp")[0] == 200


# ── HJ_261120 - sites closed for delivery from the manager's 미션현황 ──────────

def _ready_token(service):
    status, body, token = login(service)
    service.change_account(token, "1234", "admin", "camp2026", "camp2026")
    return token


def test_disabled_sites_persist_and_survive_pin_changes(store):
    assert store.disabled_sites() == {}
    assert sorted(store.set_site_disabled("b2", True, by="admin")) == ["B2"]
    # A PIN edit rewrites the whole file; the disabled section must survive it.
    store.set_pin("B1", "0420", by="admin")
    other = SiteAccessStore(store.path, SITES)
    assert sorted(other.disabled_sites()) == ["B2"]
    assert other.disabled_sites()["B2"]["by"] == "admin"
    assert store.set_site_disabled("B2", False, by="admin") == {}
    actions = [item["action"] for item in store.history(10)]
    assert actions[:3] == ["site_enabled", "pin_changed", "site_disabled"]


def test_repeating_the_same_state_adds_no_history(store):
    store.set_site_disabled("B3", True, by="admin")
    store.set_site_disabled("B3", True, by="admin")
    assert [item["action"] for item in store.history(10)].count("site_disabled") == 1


def test_unknown_site_cannot_be_disabled(store):
    with pytest.raises(ValueError):
        store.set_site_disabled("B99", True, by="admin")


def test_disabling_needs_a_changed_manager_account(service, store):
    status, body, token = login(service)
    # Default account: everything except the account change is refused.
    assert service.set_site_disabled(token, "B1", True)[1]["error"] == "must_change"
    assert service.set_site_disabled("bogus", "B1", True)[0] == 401
    assert store.disabled_sites() == {}


def test_manager_disables_and_enables_a_site(service, store):
    token = _ready_token(service)
    status, body = service.set_site_disabled(token, "B2", True)
    assert status == 200 and body["disabled_sites"] == ["B2"]
    assert {s["site"]: s["disabled"] for s in service.list_sites(token)[1]["sites"]} == {
        "B1": False, "B2": True, "B3": False}
    assert service.set_site_disabled(token, "B2", "yes")[1]["error"] == "invalid_request"
    assert service.set_site_disabled(token, "B9", True)[0] == 404
    status, body = service.set_site_disabled(token, "B2", False)
    assert status == 200 and body["disabled_sites"] == []
    assert store.history(1)[0]["action"] == "site_enabled"
