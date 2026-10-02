#!/usr/bin/env python3
"""HH_261001 - Capture browser-rendered synthetic UI fixtures, not road tests.

Requires the local show_driving_preview.py window and python websocket-client.
No image generation or robot-control endpoints are used.
"""
import argparse
import base64
import json
from pathlib import Path
import time
from urllib.request import urlopen
import websocket


class Devtools:
    def __init__(self, url):
        self.socket = websocket.create_connection(url, suppress_origin=True, timeout=15)
        self.sequence = 0
        self.events = []

    def call(self, method, params=None):
        self.sequence += 1
        self.socket.send(json.dumps({"id": self.sequence, "method": method, "params": params or {}}))
        while True:
            result = json.loads(self.socket.recv())
            if result.get("method") and len(self.events) < 2000:
                self.events.append(result)
            if result.get("id") == self.sequence:
                if "error" in result:
                    raise RuntimeError(result["error"])
                return result.get("result", {})

    def evaluate(self, expression):
        result = self.call("Runtime.evaluate", {"expression": expression, "returnByValue": True})
        if result.get("exceptionDetails"):
            raise RuntimeError(result["exceptionDetails"])
        return result.get("result", {}).get("value")

    def capture(self, path):
        result = self.call("Page.captureScreenshot", {"format": "png", "captureBeyondViewport": False})
        path.write_bytes(base64.b64decode(result["data"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug-port", type=int, default=9231)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with urlopen(f"http://127.0.0.1:{args.debug_port}/json", timeout=5) as response:
        pages = json.load(response)
    page = next(page for page in pages if page.get("type") == "page" and "driving_preview=1" in page.get("url", ""))
    cdp = Devtools(page["webSocketDebuggerUrl"])
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    frames = output / "frames"
    frames.mkdir(exist_ok=True)
    cdp.call("Emulation.setDeviceMetricsOverride", {"width": 1920, "height": 1080, "deviceScaleFactor": 1, "mobile": False})
    cdp.call("Network.enable")
    cdp.call("Runtime.enable")
    cdp.call("Page.reload", {"ignoreCache": True})
    until = time.monotonic() + 10
    while time.monotonic() < until:
        time.sleep(0.2)
        if cdp.evaluate("Boolean(document.querySelector('[data-preview=delivery]'))"):
            break
    else:
        raise RuntimeError("Preview did not render after reload")
    def click(name):
        cdp.evaluate(f"document.querySelector('[data-preview=\"{name}\"]').click()")
        time.sleep(0.4)
    def reopen():
        cdp.evaluate("document.querySelector('[data-ui=open-driving-display]')?.click()")
        time.sleep(0.4)
    click("delivery")
    # HH_261001 - Keep initial screenshots at the same deterministic demo pose.
    if cdp.evaluate("document.querySelector('[data-preview=animate]').textContent.includes('정지')"):
        click("animate")
    report = {"evidence_type": "browser_render_of_synthetic_ui_fixture", "real_driving": False,
              "viewport": {"width": 1920, "height": 1080}, "screenshots": [], "checks": {}}
    report["javascript_assets"] = cdp.evaluate("Array.from(document.scripts).map(item => item.src).filter(Boolean)")
    cdp.evaluate("window.originalCamrodHeader = document.querySelector('.control-header'); window.originalSiteGrid = document.querySelector('.toggle-grid'); true")
    def shot(name):
        cdp.capture(output / name)
        report["screenshots"].append(name)
    def layout():
        return cdp.evaluate("""(() => {
          const bounds = selector => {
            const node = document.querySelector(selector);
            if (!node) return null;
            const r = node.getBoundingClientRect();
            return {top:r.top, bottom:r.bottom, left:r.left, right:r.right};
          };
          return Object.fromEntries(['.driving-display','.dd-sensors','.dd-route-card','.dd-footer'].map(s => [s,bounds(s)]));
        })()""")
    if cdp.evaluate("document.querySelector('.driving-preview-shell').dataset.theme") != "light":
        click("theme")
    shot("01_delivery_light.png")
    report["checks"]["desktop_layout"] = layout()
    click("theme")
    shot("02_delivery_dark.png")
    click("recall")
    shot("03_recall.png")
    click("return")
    shot("04_return.png")
    click("stop")
    shot("05_safety_stop.png")
    click("offline")
    shot("06_disconnected.png")
    click("delivery")
    reopen()
    # HH_261001 - Use a full pointer sequence without invoking a mission handler.
    cdp.call("Input.dispatchMouseEvent", {"type": "mousePressed", "x": 800, "y": 500, "button": "left", "clickCount": 1})
    cdp.call("Input.dispatchMouseEvent", {"type": "mouseReleased", "x": 800, "y": 500, "button": "left", "clickCount": 1})
    time.sleep(0.5)
    report["checks"]["touch_return_panel_visible"] = cdp.evaluate("Boolean(document.querySelector('[data-preview=returned]'))")
    report["checks"]["original_header_preserved"] = cdp.evaluate("window.originalCamrodHeader === document.querySelector('.control-header')")
    report["checks"]["original_site_grid_preserved"] = cdp.evaluate("window.originalSiteGrid === document.querySelector('.toggle-grid') && Boolean(window.originalSiteGrid) && !document.querySelector('.driving-preview-returned')")
    shot("07_touch_return.png")
    reopen()
    click("theme")
    click("delivery")
    click("animate")
    started = time.monotonic()
    for index in range(36):
        if index == 9:
            click("recall")
        if index == 18:
            click("return")
        if index == 27:
            cdp.call("Input.dispatchMouseEvent", {"type": "mousePressed", "x": 800, "y": 500, "button": "left", "clickCount": 1})
            cdp.call("Input.dispatchMouseEvent", {"type": "mouseReleased", "x": 800, "y": 500, "button": "left", "clickCount": 1})
        if index == 33:
            reopen()
        cdp.capture(frames / f"{index:04d}.png")
        time.sleep(max(0.0, started + (index + 1) / 3.0 - time.monotonic()))
    report["checks"]["preview_label_visible"] = cdp.evaluate("document.body.innerText.includes('실제 주행')")
    # HH_261001 - Capture the narrow-screen layout as separate evidence.
    cdp.call("Emulation.setDeviceMetricsOverride", {"width": 1280, "height": 800, "deviceScaleFactor": 1, "mobile": False})
    time.sleep(0.5)
    shot("08_tablet_1280x800.png")
    report["checks"]["tablet_layout"] = layout()
    cdp.call("Emulation.clearDeviceMetricsOverride")
    click("idle")
    shot("09_existing_home.png")
    click("delivery")
    report["note"] = "Actual existing App is rendered with guarded synthetic snapshots. Header and site grid DOM are retained when opening/dismissing the embedded display. No robot or CARLA driving is claimed."
    requests = [event["params"]["request"] for event in cdp.events if event.get("method") == "Network.requestWillBeSent"]
    report["checks"]["control_requests"] = [request["url"] for request in requests if "/ui/" in request["url"] or request["method"] not in ("GET", "HEAD")]
    report["checks"]["api_requests"] = [request["url"] for request in requests if "/api/" in request["url"]]
    report["checks"]["websocket_connections"] = [event["params"]["url"] for event in cdp.events if event.get("method") == "Network.webSocketCreated"]
    report["checks"]["runtime_exceptions"] = [event["params"] for event in cdp.events if event.get("method") == "Runtime.exceptionThrown"]
    (output / "capture_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    cdp.socket.close()


if __name__ == "__main__":
    main()
