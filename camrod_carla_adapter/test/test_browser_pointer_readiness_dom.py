"""Real DOM regression in a disposable Chrome, never the production browser.

The HTML and PNG here are offline test fixtures, not mission evidence. Chrome
selects its own ephemeral CDP port; no fixed/live UI, ROS or CARLA endpoint is used.
"""
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
from urllib.request import urlopen

import pytest


SCRIPT = Path(__file__).resolve().parents[2] / "scripts/virtual_carla/camping_site_matrix.py"
SPEC = importlib.util.spec_from_file_location("camping_site_matrix_pointer_dom", SCRIPT)
matrix = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = matrix
SPEC.loader.exec_module(matrix)


class OfflineChrome:
    timeout_s = 2.0

    def __init__(self, connection):
        self.connection = connection
        self.serial = 0

    def call(self, method, params=None):
        self.serial += 1
        self.connection.send(json.dumps({"id": self.serial, "method": method, "params": params or {}}))
        while True:
            response = json.loads(self.connection.recv())
            if response.get("id") == self.serial:
                assert not response.get("error"), response
                return response["result"]

    def _evaluate(self, expression):
        response = self.call("Runtime.evaluate", {"expression": expression, "returnByValue": True, "awaitPromise": True})
        assert not response.get("exceptionDetails"), response
        return response["result"].get("value")

    def install(self, style="", extra=""):
        # Activate only this fixture's own headless target; never focus a live UI.
        self.call("Page.bringToFront")
        frame = self.call("Page.getFrameTree")["frameTree"]["frame"]["id"]
        html = """<!doctype html><html><head><meta charset="utf-8"><title>OFFLINE POINTER FIXTURE</title>
        <style>body{margin:0}#panel{position:absolute;left:100px;top:100px}
        #target{width:220px;height:70px}#cover{position:fixed;inset:0;z-index:10;background:#ccc}
        """ + style + """</style></head><body><div id="panel"><button id="target"
        onclick="window.fixtureClicks++">짐 싣기 완료 · 복귀</button></div>""" + extra + """
        <script>window.fixtureClicks=0</script></body></html>"""
        self.call("Page.setDocumentContent", {"frameId": frame, "html": html})
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            if self._evaluate("document.readyState") == "complete":
                return
            time.sleep(0.01)
        pytest.fail("Offline fixture document did not complete loading")


@pytest.fixture(scope="module")
def chrome(tmp_path_factory):
    executable = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")
    if not executable:
        pytest.skip("Real offline DOM test requires Chrome/Chromium")
    websocket = pytest.importorskip("websocket")
    profile = tmp_path_factory.mktemp("offline-pointer-chrome")
    process = subprocess.Popen([
        executable, "--headless=new", "--no-sandbox", "--disable-gpu", "--disable-dev-shm-usage",
        "--no-first-run", "--no-default-browser-check", "--disable-background-networking",
        "--remote-debugging-address=127.0.0.1", "--remote-debugging-port=0", "--remote-allow-origins=*",
        f"--user-data-dir={profile}", "--window-size=800,600", "about:blank",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    connection = None
    try:
        port_file = profile / "DevToolsActivePort"
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not port_file.is_file():
            assert process.poll() is None, "Disposable Chrome exited before exposing its own CDP port"
            time.sleep(0.02)
        assert port_file.is_file(), "Disposable Chrome startup timed out"
        port = int(port_file.read_text().splitlines()[0])
        assert port not in {9222, 9223, 9224}, "Never attach this test to a production debugging port"
        with urlopen(f"http://127.0.0.1:{port}/json/list", timeout=2) as response:
            pages = json.load(response)
        page = next(page for page in pages if page.get("type") == "page" and page.get("url") == "about:blank")
        connection = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=3)
        yield OfflineChrome(connection)
    finally:
        if connection is not None:
            connection.close()
        # Only the process created by this fixture is ever signalled.
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


def snap(chrome):
    return chrome._evaluate(matrix.pointer_target_snapshot_script("#target"))


def test_real_overlay_blocks_target_without_clicking(chrome):
    chrome.install(extra='<div id="cover"></div>')
    assert snap(chrome)["hit"] is False
    with pytest.raises(matrix.MatrixError):
        matrix.wait_pointer_target_ready(chrome, "#target", "offline covered target", timeout_s=0.4)
    assert chrome._evaluate("window.fixtureClicks") == 0


def test_natural_overlay_removal_allows_stable_target(chrome):
    chrome.install(extra='<div id="cover"></div>')
    chrome._evaluate("setTimeout(() => document.getElementById('cover').remove(), 250); true")
    started = time.monotonic()
    result = matrix.wait_pointer_target_ready(chrome, "#target", "offline fading overlay", timeout_s=2)
    assert time.monotonic() - started >= 0.45
    assert result["hit"] is True and result["settled"] is True
    assert chrome._evaluate("window.fixtureClicks") == 0


def test_ancestor_fade_must_finish_before_pointer_readiness(chrome):
    chrome.install(style="@keyframes fade{from{opacity:.1}to{opacity:1}}#panel{animation:fade .7s linear}")
    assert snap(chrome)["settled"] is False
    with pytest.raises(matrix.MatrixError):
        matrix.wait_pointer_target_ready(chrome, "#target", "offline ancestor fade", timeout_s=0.15)
    result = matrix.wait_pointer_target_ready(chrome, "#target", "offline finished fade", timeout_s=2)
    assert result["settled"] is True
    assert chrome._evaluate("window.fixtureClicks") == 0


def test_geometry_motion_resets_stability_interval(chrome):
    chrome.install()
    chrome._evaluate("window.fixtureStep=0;window.fixtureMove=setInterval(()=>{const p=document.getElementById('panel');p.style.left=(100+80*(++window.fixtureStep%2))+'px'},30);setTimeout(()=>clearInterval(window.fixtureMove),400);true")
    started = time.monotonic()
    result = matrix.wait_pointer_target_ready(chrome, "#target", "offline moving target", timeout_s=2)
    assert time.monotonic() - started >= 0.60
    assert result["hit"] is True
    assert chrome._evaluate("window.fixtureClicks") == 0


def test_ready_then_png_capture_preserves_original_fixture_png(chrome, tmp_path):
    chrome.install()
    ready = matrix.wait_pointer_target_ready(chrome, "#target", "offline PNG target", timeout_s=2)
    assert ready["hit"] and ready["settled"]
    assert chrome._evaluate("document.title") == "OFFLINE POINTER FIXTURE"
    png = base64.b64decode(chrome.call("Page.captureScreenshot", {"format": "png"})["data"])
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    path = tmp_path / "offline_ready_fixture.png"
    path.write_bytes(png)
    original = hashlib.sha256(path.read_bytes()).hexdigest()
    chrome._evaluate("document.getElementById('target').textContent='changed offline fixture';true")
    assert hashlib.sha256(path.read_bytes()).hexdigest() == original
    assert chrome._evaluate("window.fixtureClicks") == 0
