#!/usr/bin/env python3
"""HH_261006 - Exercise the real WebKit kiosk with labelled synthetic UI data.

No robot commands or parameter changes. Verify the graphics lease is released
on home and reacquired on navigation. Save PNG/GIF and machine-readable results.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import time
from urllib.request import urlopen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8030/?driving_preview=1")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--live-idle", action="store_true",
                        help="Read-only connected idle map; refuse active missions")
    args = parser.parse_args()
    if not args.live_idle and "driving_preview=1" not in args.url:
        parser.error("Only the guarded synthetic preview is allowed")
    if args.live_idle:
        if "?" in args.url:
            parser.error("Live idle URL must not select preview data")
        with urlopen(args.url.rstrip('/') + '/api/driving', timeout=5) as response:
            mission = json.load(response)["mission"]
        with urlopen(args.url.rstrip('/') + '/ui/state', timeout=5) as response:
            control = json.load(response)
        if mission["active"] or control.get("engaged") or control.get("manual_drive_armed"):
            parser.error("Refusing UI reload during an active mission/control session")
    args.output.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parents[1] / "runtime/python/camrod_ui/operator_ui_window.py"
    spec = importlib.util.spec_from_file_location("kiosk_test_subject", source)
    kiosk = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(kiosk)
    Gdk, GLib, Gtk, WebKit2 = kiosk._load_gtk()
    report = {"real_driving": False, "evidence_type": "production_WebKit_synthetic_UI_lifecycle",
              "url": args.url, "webgl_setting_events": [], "checks": {},
              "webkit_version": [WebKit2.get_major_version(), WebKit2.get_minor_version(), WebKit2.get_micro_version()]}
    if args.live_idle:
        report.update(evidence_type="production_WebKit_connected_idle_UI_lifecycle", mission_before=mission)
    state = {"view": None, "busy": False, "done": False, "phase": "initial_home" if args.live_idle else "navigation",
             "frame": 0, "loaded_count": 0, "started": time.monotonic()}
    temp = tempfile.TemporaryDirectory(prefix="camrod-webkit-frames-")

    def make_view():
        view = WebKit2.WebView()
        state["view"] = view
        def count_document_load(unused_view, event):
            if event == WebKit2.LoadEvent.FINISHED:
                state["loaded_count"] += 1
        view.connect("load-changed", count_document_load)
        settings = view.get_settings()
        settings.connect("notify::enable-webgl", lambda value, unused:
                         report["webgl_setting_events"].append(value.get_enable_webgl()))
        manager = view.get_user_content_manager()
        manager.add_script(WebKit2.UserScript.new("""
          window.__auditErrors=[];
          window.addEventListener('error', e=>window.__auditErrors.push(String(e.message)));
          const oldError=console.error;
          console.error=(...xs)=>{window.__auditErrors.push(xs.map(String).join(' '));oldError(...xs);};
        """, WebKit2.UserContentInjectedFrames.TOP_FRAME, WebKit2.UserScriptInjectionTime.START, None, None))
        return view

    class WebKitProxy:
        WebView = staticmethod(make_view)
        def __getattr__(self, name):
            return getattr(WebKit2, name)

    kiosk._load_gtk = lambda: (Gdk, GLib, Gtk, WebKitProxy())
    expression = """JSON.stringify({
      status: document.querySelector('.ranger-navigation-scene')?.dataset.modelStatus || null,
      navigation: JSON.parse(document.querySelector('canvas[data-navigation-state]')?.dataset.navigationState || '{}'),
      errors: window.__auditErrors || [],
      idleEntry: Boolean(document.querySelector('[data-ui="open-idle-navigation-map"]')),
      text: document.body.innerText
    })"""

    def capture(path):
        window = state["view"].get_toplevel()
        width, height = window.get_size()
        pixbuf = Gdk.pixbuf_get_from_window(window.get_window(), 0, 0, width, height)
        if pixbuf:
            pixbuf.savev(str(path), "png", [], [])

    def click(selector):
        state["view"].run_javascript(
            f"document.querySelector({json.dumps(selector)})?.click()", None, None, None)

    def finish(error=None):
        if state["done"]:
            return
        state["done"] = True
        if error:
            report["error"] = error
            capture(args.output / "failure.png")
        report["passed"] = not error
        if state["view"]:
            state["view"].get_toplevel().destroy()
        (args.output / "webkit_lifecycle.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))

    def on_result(view, result, unused):
        state["busy"] = False
        try:
            snapshot = json.loads(view.run_javascript_finish(result).get_js_value().to_string())
            enabled = view.get_settings().get_enable_webgl()
            report["last_snapshot"] = snapshot
            # HH_261006 - Never count the outgoing document as a reload success
            # while WebKit is asynchronously beginning the replacement load.
            if (state["phase"] in {"reload", "reload_wait"}
                    and state["loaded_count"] <= state["reload_generation"]):
                return
            if snapshot["status"] in {"webgl-unavailable", "error"}:
                finish("Navigation renderer failed: " + str(snapshot["status"]))
                return
            ready = (snapshot["status"] == "ready"
                     and len(snapshot["navigation"].get("wrapFacesLoaded", [])) == 4
                     # HH_261006 - A loaded GLB is not yet a presented GTK frame.
                     # Allow compositor presentation before capturing evidence.
                     and snapshot["navigation"].get("renderedFrames", 0) >= 60)
            if args.live_idle:
                ready = (ready and snapshot["navigation"].get("baseMapVisible")
                         and snapshot["navigation"].get("serviceAreaCount") == 14
                         and snapshot["navigation"].get("routeVertices") == 0)
            if state["phase"] == "initial_home" and snapshot["idleEntry"] and not enabled:
                report["checks"]["initial_home"] = {"enabled": enabled}
                click('[data-ui="open-idle-navigation-map"]')
                state["phase"] = "navigation"
            elif state["phase"] == "navigation" and ready:
                report["checks"]["navigation"] = {"enabled": enabled, "snapshot": snapshot}
                if not enabled:
                    finish("Navigation did not acquire scoped graphics")
                    return
                capture(args.output / "navigation_webkit.png")
                state["phase"] = "frames"
            elif state["phase"] == "frames":
                index = state["frame"]
                capture(Path(temp.name) / f"{index:03d}.png")
                if index in (6, 16):
                    click('[data-testid="ranger-model-detail-toggle"]')
                state["frame"] += 1
                if state["frame"] == 24:
                    state["phase"] = "home"
                    click('.dd-back-action' if args.live_idle else '[data-preview="idle"]')
            elif state["phase"] == "home" and snapshot["status"] is None and not enabled:
                report["checks"]["home_restored"] = {"enabled": enabled, "snapshot": snapshot}
                capture(args.output / "home_webkit.png")
                state["phase"] = "reopen"
                click('[data-ui="open-idle-navigation-map"]' if args.live_idle else '[data-preview="delivery"]')
            elif state["phase"] == "reopen" and ready and enabled:
                report["checks"]["navigation_reopened"] = {"enabled": enabled, "snapshot": snapshot}
                capture(args.output / "navigation_reopened_webkit.png")
                state["phase"] = "reload"
                state["reload_generation"] = state["loaded_count"]
                view.reload_bypass_cache()
            elif state["phase"] == "reload" and args.live_idle and snapshot["idleEntry"] and not enabled:
                click('[data-ui="open-idle-navigation-map"]')
                state["phase"] = "reload_wait"
            elif state["phase"] in {"reload", "reload_wait"} and ready and enabled:
                report["checks"]["navigation_reloaded"] = {"enabled": enabled, "snapshot": snapshot}
                report["reload_document_finished"] = state["loaded_count"]
                capture(args.output / "navigation_reloaded_webkit.png")
                finish()
        except Exception as error:
            report["probe_error"] = str(error)

    def poll():
        if state["done"]:
            return GLib.SOURCE_REMOVE
        if time.monotonic() - state["started"] > 75:
            finish("Timed out in phase " + state["phase"])
            return GLib.SOURCE_REMOVE
        if state["view"] and not state["busy"]:
            state["busy"] = True
            state["view"].run_javascript(expression, None, on_result, None)
        return GLib.SOURCE_CONTINUE

    GLib.timeout_add(400, poll)
    parsed = kiosk._build_parser().parse_args(["--url", args.url, "--no-fullscreen", "--width", "1600", "--height", "1000"])
    kiosk._run_webkit(parsed)
    if report.get("passed"):
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", "2.5", "-i", f"{temp.name}/%03d.png",
                        "-filter_complex", "[0:v]scale=1000:-1,split[a][b];[a]palettegen[p];[b][p]paletteuse",
                        "-loop", "0", str(args.output / "navigation_webkit.gif")], check=True)
    temp.cleanup()
    print(json.dumps({"passed": report.get("passed"), "error": report.get("error"),
                      "settings": report["webgl_setting_events"], "output": str(args.output)}))
    return 0 if report.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
