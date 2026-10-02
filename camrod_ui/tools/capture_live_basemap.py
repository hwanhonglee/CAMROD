#!/usr/bin/env python3
"""HH_261002 - Capture the connected idle map, never a driving/demo success claim.

Only reloads the visible page and clicks read-only map/camera controls. Refuses
an active mission; never dispatches robot commands or modifies sensor data.
"""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import time
from urllib.request import urlopen

from capture_driving_preview import Devtools


def fetch(url):
    with urlopen(url, timeout=5) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug-port", type=int, default=9224)
    parser.add_argument("--url", default="http://127.0.0.1:8010/")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    snapshot = fetch(args.url.rstrip("/") + "/api/driving")
    if snapshot["mission"]["active"]:
        raise RuntimeError("Refusing to reload UI during an active mission")
    page = next(item for item in fetch(f"http://127.0.0.1:{args.debug_port}/json")
                if item.get("type") == "page" and item.get("url") == args.url)
    client = Devtools(page["webSocketDebuggerUrl"])
    args.output.mkdir(parents=True, exist_ok=True)

    def click(selector):
        location = client.evaluate("""(() => {
          const el = document.querySelector(%s); if (!el) return null;
          const r = el.getBoundingClientRect();
          return {x:r.x+r.width/2,y:r.y+r.height/2};
        })()""" % json.dumps(selector))
        if location is None:
            raise RuntimeError(f"Missing read-only UI control: {selector}")
        for event in ("mousePressed", "mouseReleased"):
            client.call("Input.dispatchMouseEvent", {"type": event, **location,
                        "button": "left", "clickCount": 1})

    def state():
        return client.evaluate("JSON.parse(document.querySelector('canvas[data-navigation-state]')"
                               "?.dataset.navigationState || 'null')")

    try:
        client.call("Network.enable")
        client.call("Page.reload", {"ignoreCache": True})
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if client.evaluate("Boolean(document.querySelector('[data-ui=open-idle-navigation-map]'))"):
                click("[data-ui=open-idle-navigation-map]")
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("Idle navigation entry did not appear")
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            current = state()
            if (current and current.get("illustratedRoadCount", 0) > 0
                    and len(current.get("wrapFacesLoaded", [])) == 4):
                break
            time.sleep(0.2)
        else:
            raise RuntimeError("Filled map/robot appearance not ready")
        assert current["routeVertices"] == 0, "Idle evidence must not contain a mission path"
        assert current["baseMapVisible"] and current["illustrativeEnvironmentVisible"]
        assert abs(current["robotProjectedXNdc"]) < 0.01
        time.sleep(0.5)
        client.capture(args.output / "idle_filled_roads_surroundings.png")
        report = {"evidence_type": "connected_live_idle_ui_camera_transition",
                  "autonomous_driving_test": False, "mission_before": snapshot["mission"],
                  "normal_view": state(), "url": args.url,
                  "scripts": client.evaluate("Array.from(document.scripts).map(s=>s.src).filter(Boolean)")}
        with tempfile.TemporaryDirectory(prefix="camrod-basemap-capture-") as temp:
            for index in range(24):
                if index in (4, 14):
                    click('[data-testid="ranger-model-detail-toggle"]')
                client.capture(Path(temp) / f"{index:03d}.png")
                if index == 12:
                    client.capture(args.output / "idle_filled_roads_exterior.png")
                    report["exterior_view"] = state()
                time.sleep(0.2)
            subprocess.run([
                "ffmpeg", "-v", "error", "-y", "-framerate", "5", "-i", f"{temp}/%03d.png",
                "-filter_complex", "[0:v]scale=1000:-1:flags=lanczos,split[a][b];"
                "[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer",
                "-loop", "0", str(args.output / "idle_filled_roads_camera_views.gif"),
            ], check=True)
        report["normal_view_final"] = state()
        report["mission_after"] = fetch(args.url.rstrip("/") + "/api/driving")["mission"]
        report["mutating_http_requests"] = [event["params"]["request"]["url"]
            for event in client.events if event.get("method") == "Network.requestWillBeSent"
            and event["params"]["request"]["method"] in {"POST", "PUT", "PATCH", "DELETE"}]
        (args.output / "idle_filled_roads_capture.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        client.socket.close()


if __name__ == "__main__":
    main()
