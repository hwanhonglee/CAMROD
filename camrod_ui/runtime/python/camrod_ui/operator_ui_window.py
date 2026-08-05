#!/usr/bin/env python3
# HH_260805 - Use Chromium as the production kiosk while retaining WebKit fallback.
"""Open the CAMROD operator web UI in a managed local kiosk window."""

from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
from typing import Any, Sequence
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_URL = "http://127.0.0.1:8010"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 800
READINESS_POLL_INTERVAL_S = 0.10
READINESS_REQUEST_TIMEOUT_S = 0.25
CHROMIUM_CANDIDATES = (
    "chromium-browser",
    "chromium",
    "google-chrome-stable",
    "google-chrome",
    "brave-browser",
    "brave-browser-stable",
    "brave",
)


def _positive_dimension(value: str) -> int:
    dimension = int(value)
    if dimension <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return dimension


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="camrod_ui_window",
        description="Open the CAMROD operator UI in a managed kiosk window.",
    )
    parser.add_argument(
        "--url",
        default=DEFAULT_URL,
        help=f"operator UI URL (default: {DEFAULT_URL})",
    )
    parser.add_argument(
        "--width",
        type=_positive_dimension,
        default=DEFAULT_WIDTH,
        help=f"initial window width in pixels (default: {DEFAULT_WIDTH})",
    )
    parser.add_argument(
        "--height",
        type=_positive_dimension,
        default=DEFAULT_HEIGHT,
        help=f"initial window height in pixels (default: {DEFAULT_HEIGHT})",
    )
    parser.add_argument(
        "--fullscreen",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="open the window fullscreen (default: enabled; use --no-fullscreen to opt out)",
    )
    parser.add_argument(
        "--engine",
        choices=("auto", "chromium", "webkit"),
        default="chromium",
        help="rendering engine (default: chromium; auto falls back to WebKit)",
    )
    return parser


def _find_chromium_browser() -> str | None:
    """Return a portable Chromium-family executable without host-specific paths."""
    configured = os.environ.get("CAMROD_UI_BROWSER", "").strip()
    candidates = (configured,) if configured else CHROMIUM_CANDIDATES
    for candidate in candidates:
        executable = shutil.which(candidate)
        if executable:
            return executable
    return None


def _build_chromium_command(
    executable: str,
    args: argparse.Namespace,
    profile_dir: str,
) -> list[str]:
    # HH_260804 - A private profile prevents a running personal browser from
    # absorbing the kiosk command and leaving the ROS launch process orphaned.
    command = [
        executable,
        f"--user-data-dir={profile_dir}",
        "--no-first-run",
        "--no-default-browser-check",
        "--noerrdialogs",
        "--disable-session-crashed-bubble",
        "--disable-background-networking",
        "--disable-component-update",
        "--disable-default-apps",
        "--disable-sync",
        "--disable-translate",
        "--disable-pinch",
        "--overscroll-history-navigation=0",
        "--password-store=basic",
        # HH_260805 - Keep Chromium on the Jetson GPU path without bypassing
        # its driver blocklist or disabling the software safety fallback.
        "--enable-gpu-rasterization",
        "--enable-zero-copy",
        "--disable-backgrounding-occluded-windows",
        "--disable-renderer-backgrounding",
    ]
    if args.fullscreen:
        command.extend(("--kiosk", args.url))
    else:
        command.extend(
            (
                f"--app={args.url}",
                f"--window-size={args.width},{args.height}",
            )
        )
    return command


def _ui_is_ready(url: str, timeout_s: float = READINESS_REQUEST_TIMEOUT_S) -> bool:
    request = Request(url, headers={"Cache-Control": "no-cache"})
    try:
        with urlopen(request, timeout=max(0.05, timeout_s)) as response:
            return response.status < 400
    except (HTTPError, URLError, TimeoutError, ValueError):
        return False


def _wait_for_ui_ready(
    url: str,
    stop_requested: threading.Event,
    poll_interval_s: float = READINESS_POLL_INTERVAL_S,
) -> bool:
    """Wait cheaply for uvicorn so a renderer never opens an error page first."""
    attempts = 0
    while not stop_requested.is_set():
        if _ui_is_ready(url):
            return True
        attempts += 1
        if attempts == 1 or attempts % 50 == 0:
            print(
                f"camrod_ui_window: UI not ready at {url}; waiting for backend",
                file=sys.stderr,
                flush=True,
            )
        stop_requested.wait(max(0.01, poll_interval_s))
    return False


def _terminate_process_group(process: subprocess.Popen[Any]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=2.0)


def _run_chromium(args: argparse.Namespace, executable: str) -> int:
    stop_requested = threading.Event()
    process: subprocess.Popen[Any] | None = None

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    if not _wait_for_ui_ready(args.url, stop_requested):
        return 0

    try:
        with tempfile.TemporaryDirectory(prefix="camrod-operator-ui-") as profile_dir:
            command = _build_chromium_command(executable, args, profile_dir)
            process = subprocess.Popen(
                command,
                start_new_session=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print(
                f"camrod_ui_window: opened {args.url} with {os.path.basename(executable)}",
                flush=True,
            )
            while process.poll() is None and not stop_requested.wait(0.2):
                pass
            if stop_requested.is_set():
                _terminate_process_group(process)
                return 0
            return int(process.returncode or 0)
    except OSError as exc:
        print(
            f"camrod_ui_window: failed to start Chromium browser: {exc}",
            file=sys.stderr,
        )
        return 3
    finally:
        if process is not None and process.poll() is None:
            _terminate_process_group(process)


def _load_gtk() -> tuple[Any, Any, Any, Any]:
    try:
        import gi

        gi.require_version("Gdk", "3.0")
        gi.require_version("Gtk", "3.0")
        gi.require_version("WebKit2", "4.0")
        from gi.repository import Gdk, GLib, Gtk, WebKit2
    except (ImportError, ValueError) as exc:
        raise RuntimeError(
            "GTK/WebKit is unavailable. Install the runtime packages "
            "'python3-gi' and 'gir1.2-webkit2-4.0'."
        ) from exc
    return Gdk, GLib, Gtk, WebKit2


def _run_webkit(args: argparse.Namespace) -> int:
    try:
        Gdk, GLib, Gtk, WebKit2 = _load_gtk()
    except RuntimeError as exc:
        print(f"camrod_ui_window: {exc}", file=sys.stderr)
        return 2

    try:
        context = WebKit2.WebContext.get_default()
        # HH_260805 - Retain cached React/static assets across reloads without
        # forcing WebKit's deprecated process-model override.
        if hasattr(context, "set_cache_model"):
            context.set_cache_model(WebKit2.CacheModel.DOCUMENT_BROWSER)
        window = Gtk.Window(title="CAMROD Operator UI")
        window.set_deletable(False)
        window.set_type_hint(Gdk.WindowTypeHint.UTILITY)
        web_view = WebKit2.WebView()
        settings = web_view.get_settings()
        # HH_260805 - Keep GPU compositing active for the kiosk and enable
        # smooth touch scrolling. WebGL stays off because this UI does not use it.
        setting_values = {
            "set_enable_developer_extras": False,
            "set_enable_write_console_messages_to_stdout": False,
            "set_enable_page_cache": True,
            "set_enable_smooth_scrolling": True,
            "set_enable_webgl": False,
        }
        for setter_name, value in setting_values.items():
            setter = getattr(settings, setter_name, None)
            if setter is not None:
                setter(value)
        if hasattr(settings, "set_hardware_acceleration_policy") and hasattr(
            WebKit2, "HardwareAccelerationPolicy"
        ):
            settings.set_hardware_acceleration_policy(
                WebKit2.HardwareAccelerationPolicy.ALWAYS
            )
    except RuntimeError as exc:
        print(
            "camrod_ui_window: GTK could not open a display. "
            f"Run it from a graphical session. Details: {exc}",
            file=sys.stderr,
        )
        return 3

    window.set_default_size(args.width, args.height)
    window.add(web_view)
    if args.fullscreen:
        window.fullscreen()

    # HH_260805 - Probe uvicorn on a worker thread before the first WebKit load.
    # This avoids rendering a network error page and then reloading it every
    # second, which made startup slower and needlessly exercised the renderer.
    window_closed = False
    load_succeeded = False
    load_in_progress = False
    attempt_failed = False
    load_attempt_count = 0
    load_failure_count = 0
    readiness_stop = threading.Event()
    readiness_probe_running = False

    def request_load() -> None:
        nonlocal attempt_failed, load_attempt_count, load_in_progress
        if window_closed or load_succeeded or load_in_progress:
            return
        attempt_failed = False
        load_in_progress = True
        load_attempt_count += 1
        web_view.load_uri(args.url)

    def mark_load_failed(detail: str) -> None:
        nonlocal attempt_failed, load_failure_count, load_in_progress
        if window_closed or load_succeeded:
            return
        attempt_failed = True
        load_in_progress = False
        load_failure_count += 1
        if load_failure_count == 1 or load_failure_count % 10 == 0:
            print(
                f"camrod_ui_window: UI not ready ({detail}); "
                "waiting for backend",
                file=sys.stderr,
                flush=True,
            )
        start_readiness_probe()

    def on_load_changed(_view: Any, load_event: Any) -> None:
        nonlocal load_in_progress, load_succeeded
        if window_closed or load_succeeded:
            return
        if load_event == WebKit2.LoadEvent.STARTED:
            load_in_progress = True
            return
        if load_event != WebKit2.LoadEvent.FINISHED:
            return

        load_in_progress = False
        if attempt_failed:
            return

        main_resource = web_view.get_main_resource()
        response = main_resource.get_response() if main_resource is not None else None
        status_code = response.get_status_code() if response is not None else 0
        if status_code >= 400:
            mark_load_failed(f"HTTP {status_code}")
            return

        load_succeeded = True
        window.set_title("CAMROD Operator UI")
        print(
            f"camrod_ui_window: loaded {args.url} "
            f"(attempt {load_attempt_count}, WebKit)",
            flush=True,
        )

    def on_load_failed(
        _view: Any,
        _load_event: Any,
        _failing_uri: str,
        error: Any,
    ) -> bool:
        if window_closed or load_succeeded:
            return True
        detail = getattr(error, "message", str(error))
        mark_load_failed(detail)
        return True

    def load_after_ready() -> bool:
        nonlocal readiness_probe_running
        readiness_probe_running = False
        if not window_closed and not load_succeeded:
            request_load()
        return GLib.SOURCE_REMOVE

    def start_readiness_probe() -> None:
        nonlocal readiness_probe_running
        if readiness_probe_running or window_closed or load_succeeded:
            return
        readiness_probe_running = True

        def probe() -> None:
            if _wait_for_ui_ready(args.url, readiness_stop):
                GLib.idle_add(load_after_ready)

        threading.Thread(
            target=probe,
            name="camrod-ui-readiness",
            daemon=True,
        ).start()

    def quit_main_loop(*_unused: object) -> None:
        nonlocal window_closed
        window_closed = True
        readiness_stop.set()
        Gtk.main_quit()

    def close_window_from_signal(*_unused: object) -> bool:
        if not window_closed:
            window.destroy()
        return GLib.SOURCE_REMOVE

    web_view.connect("load-changed", on_load_changed)
    web_view.connect("load-failed", on_load_failed)
    window.connect("destroy", quit_main_loop)
    GLib.unix_signal_add(
        GLib.PRIORITY_DEFAULT, signal.SIGINT, close_window_from_signal
    )
    GLib.unix_signal_add(
        GLib.PRIORITY_DEFAULT, signal.SIGTERM, close_window_from_signal
    )

    window.set_title("CAMROD Operator UI - Connecting...")
    window.show_all()
    start_readiness_probe()
    Gtk.main()
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    if args.engine in ("auto", "chromium"):
        executable = _find_chromium_browser()
        if executable is not None:
            return _run_chromium(args, executable)
        if args.engine == "chromium":
            print(
                "camrod_ui_window: no Chromium-family browser found. "
                "Install Chromium/Chrome/Brave or set CAMROD_UI_BROWSER.",
                file=sys.stderr,
            )
            return 2
        print(
            "camrod_ui_window: no Chromium-family browser found; using WebKit fallback",
            file=sys.stderr,
            flush=True,
        )

    return _run_webkit(args)


if __name__ == "__main__":
    raise SystemExit(main())
