#!/usr/bin/env python3
# HH_260807 - Use the field-verified WebKit kiosk while retaining Chromium override.
"""Open the CAMROD operator web UI in a managed local kiosk window."""

from __future__ import annotations

import argparse
import hashlib
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
REVISION_POLL_INTERVAL_S = 1.0
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
        default="webkit",
        help="rendering engine (default: webkit; auto tries Chromium then WebKit)",
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


def _read_ui_revision(
    url: str,
    timeout_s: float = READINESS_REQUEST_TIMEOUT_S,
) -> str | None:
    """Return a content fingerprint for the current frontend entry document."""
    try:
        request = Request(
            url,
            headers={
                "Cache-Control": "no-cache, no-store",
                "Pragma": "no-cache",
            },
        )
        with urlopen(request, timeout=max(0.05, timeout_s)) as response:
            if response.status >= 400:
                return None
            return hashlib.sha256(response.read()).hexdigest()
    except (HTTPError, URLError, TimeoutError, ValueError):
        return None


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
        # The React entry point changes its hashed bundle name on every build.
        # Clear the process-local HTTP cache once at kiosk startup so a robot
        # restart cannot restore an older index.html and hide a new UI.
        if hasattr(context, "clear_cache"):
            context.clear_cache()
        # Retain the newly fetched hashed assets during this kiosk session
        # without forcing WebKit's deprecated process-model override.
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
    revision_monitor_started = False
    revision_lock = threading.Lock()
    active_revision: str | None = None
    pending_revision: str | None = None

    def request_load() -> None:
        nonlocal active_revision, attempt_failed, load_attempt_count, load_in_progress
        if window_closed or load_succeeded or load_in_progress:
            return
        attempt_failed = False
        load_in_progress = True
        load_attempt_count += 1
        revision = _read_ui_revision(args.url)
        if revision is not None:
            with revision_lock:
                active_revision = revision
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
        nonlocal active_revision, load_in_progress, load_succeeded, pending_revision
        if window_closed:
            return
        if load_event == WebKit2.LoadEvent.STARTED:
            load_in_progress = True
            return
        if load_event != WebKit2.LoadEvent.FINISHED:
            return

        load_in_progress = False
        if attempt_failed and not load_succeeded:
            return

        main_resource = web_view.get_main_resource()
        response = main_resource.get_response() if main_resource is not None else None
        status_code = response.get_status_code() if response is not None else 0
        if status_code >= 400:
            if load_succeeded:
                with revision_lock:
                    pending_revision = None
                return
            mark_load_failed(f"HTTP {status_code}")
            return

        if load_succeeded:
            with revision_lock:
                if pending_revision is not None:
                    active_revision = pending_revision
                    pending_revision = None
            print(
                "camrod_ui_window: frontend build changed; reloaded current UI",
                flush=True,
            )
            return

        load_succeeded = True
        window.set_title("CAMROD Operator UI")
        print(
            f"camrod_ui_window: loaded {args.url} "
            f"(attempt {load_attempt_count}, WebKit)",
            flush=True,
        )
        start_revision_monitor()

    def on_load_failed(
        _view: Any,
        _load_event: Any,
        _failing_uri: str,
        error: Any,
    ) -> bool:
        nonlocal pending_revision
        if window_closed:
            return True
        detail = getattr(error, "message", str(error))
        if load_succeeded:
            with revision_lock:
                pending_revision = None
            print(
                f"camrod_ui_window: frontend refresh failed ({detail}); will retry",
                file=sys.stderr,
                flush=True,
            )
            return True
        mark_load_failed(detail)
        return True

    def reload_changed_frontend() -> bool:
        if window_closed:
            return GLib.SOURCE_REMOVE
        reload_without_cache = getattr(web_view, "reload_bypass_cache", None)
        if reload_without_cache is not None:
            reload_without_cache()
        else:
            web_view.reload()
        return GLib.SOURCE_REMOVE

    def start_revision_monitor() -> None:
        nonlocal revision_monitor_started
        if revision_monitor_started or window_closed:
            return
        revision_monitor_started = True

        def monitor() -> None:
            nonlocal active_revision, pending_revision
            candidate_revision: str | None = None
            candidate_observations = 0
            while not readiness_stop.is_set():
                revision = _read_ui_revision(args.url)
                reload_required = False
                if revision is not None:
                    with revision_lock:
                        if active_revision is None:
                            active_revision = revision
                        elif pending_revision is not None:
                            pass
                        elif revision == active_revision:
                            candidate_revision = None
                            candidate_observations = 0
                        elif revision == candidate_revision:
                            candidate_observations += 1
                            if candidate_observations >= 2:
                                pending_revision = revision
                                reload_required = True
                        else:
                            candidate_revision = revision
                            candidate_observations = 1
                if reload_required:
                    GLib.idle_add(reload_changed_frontend)
                if readiness_stop.wait(REVISION_POLL_INTERVAL_S):
                    break

        threading.Thread(
            target=monitor,
            name="camrod-ui-revision-monitor",
            daemon=True,
        ).start()

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
