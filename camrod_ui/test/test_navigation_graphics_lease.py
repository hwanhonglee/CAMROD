"""HH_261006 - Navigation opt-in must not replace the legacy kiosk defaults."""
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock


spec = importlib.util.spec_from_file_location(
    "kiosk_under_test", Path(__file__).resolve().parents[1]
    / "runtime/python/camrod_ui/operator_ui_window.py")
kiosk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(kiosk)
URL = "http://127.0.0.1:8010/"
A = "nav-0123456789abcdef-1"
B = "nav-0123456789abcdef-2"


def request(token, active=True):
    return json.dumps({"id": token, "active": active})


def test_defaults_unchanged_and_home_does_not_enable_graphics():
    assert kiosk._build_parser().parse_args([]).engine == "webkit"
    settings = Mock()
    kiosk.NavigationGraphicsLease(settings, URL)
    settings.set_enable_webgl.assert_not_called()


def test_navigation_only_and_last_owner_restores_disabled_default():
    settings = Mock()
    lease = kiosk.NavigationGraphicsLease(settings, URL)
    for token in (A, B):
        assert lease.receive(request(token), URL)["enabled"]
    lease.receive(request(A, False), URL)
    settings.set_enable_webgl.assert_called_with(True)
    lease.receive(request(B, False), URL)
    settings.set_enable_webgl.assert_called_with(False)
    assert not lease.tokens


def test_untrusted_or_malformed_requests_do_not_change_settings():
    settings = Mock()
    lease = kiosk.NavigationGraphicsLease(settings, URL)
    for message, uri in [
        (request(A), "https://outside.invalid/"),
        (request(A), "http://127.0.0.1:9999/"),
        (request(A, "true"), URL),
        ('{"id":"arbitrary js","active":true}', URL),
        ('[]', URL), ('null', URL), ('bad', URL), ('x' * 513, URL),
    ]:
        assert lease.receive(message, uri) is None
    settings.set_enable_webgl.assert_not_called()


def test_reload_restores_default_and_drops_stale_owner():
    settings = Mock()
    lease = kiosk.NavigationGraphicsLease(settings, URL)
    lease.receive(request(A), URL)
    lease.reset()
    settings.set_enable_webgl.assert_called_with(False)
    assert not lease.tokens


def test_previous_document_release_cannot_disable_new_document():
    settings = Mock()
    lease = kiosk.NavigationGraphicsLease(settings, URL)
    lease.receive(request(A), URL)
    lease.reset()
    new_document = "nav-fedcba9876543210-1"
    lease.receive(request(new_document), URL)
    lease.receive(request(A, False), URL)
    assert lease.tokens == {new_document}
    settings.set_enable_webgl.assert_called_with(True)


def test_acquire_is_idempotent_and_owner_count_is_bounded():
    lease = kiosk.NavigationGraphicsLease(Mock(), URL)
    for index in range(1, 5):
        token = f"nav-0123456789abcdef-{index}"
        assert lease.receive(request(token), URL)["enabled"]
        assert lease.receive(request(token), URL)["enabled"]
    assert len(lease.tokens) == 4
    assert not lease.receive(request("nav-0123456789abcdef-5"), URL)["enabled"]
