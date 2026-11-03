"""HH_261102 - Verify the charging adapter's identity boundary without connecting CDP."""

import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location("live_carla_browser", Path(__file__).with_name("browser.py"))
browser = importlib.util.module_from_spec(spec)
spec.loader.exec_module(browser)


def test_target_lookup_requires_one_exact_local_page_and_local_debugger():
    target = {"type": "page", "url": "http://127.0.0.1:8010/",
              "webSocketDebuggerUrl": "ws://127.0.0.1:9224/devtools/page/1"}
    assert browser.select_target([target], "http://127.0.0.1:8010") == target
    for targets in ([], [target, target], [{**target, "webSocketDebuggerUrl": "ws://example.com/page/1"}]):
        with pytest.raises(browser.matrix.MatrixError):
            browser.select_target(targets, "http://127.0.0.1:8010")
    for url in ("https://127.0.0.1:8010", "http://example.com:8010", "http://user@localhost:8010", "http://localhost:8010/page"):
        with pytest.raises(browser.matrix.MatrixError):
            browser.local_url(url, origin=True)


def test_charging_readiness_still_requires_visible_production_identity():
    ready = {"ready": "complete", "href": "http://127.0.0.1:8010/", "title": "Robot UI",
             "visibility": "visible", "rootReady": True, "screenReady": True}
    browser.validate_page(ready, "http://127.0.0.1:8010")
    for key, value in (("screenReady", False), ("rootReady", False), ("visibility", "hidden"), ("title", "Other UI")):
        with pytest.raises(browser.matrix.MatrixError):
            browser.validate_page({**ready, key: value}, "http://127.0.0.1:8010")


def test_inherited_dispatch_and_input_path_are_not_reimplemented():
    assert browser.LiveOperatorBrowser.dispatch is browser.matrix.OperatorBrowserClient.dispatch
    assert browser.LiveOperatorBrowser._click is browser.matrix.OperatorBrowserClient._click
    assert browser.LiveOperatorBrowser._insert_text is browser.matrix.OperatorBrowserClient._insert_text
