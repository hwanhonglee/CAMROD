#!/usr/bin/env python3
# HH_260727 - Lightweight GTK/WebKit operator window for hosts that do not need a full browser.
"""Open the CAMROD operator web UI in a small native GTK window."""

from __future__ import annotations

import argparse
import signal
import sys
from typing import Any, Sequence


DEFAULT_URL = "http://127.0.0.1:8010"
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 800


def _positive_dimension(value: str) -> int:
    dimension = int(value)
    if dimension <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return dimension


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="camrod_ui_window",
        description="Open the CAMROD operator UI in a lightweight GTK/WebKit window.",
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
    return parser


def _load_gtk() -> tuple[Any, Any, Any]:
    try:
        import gi

        gi.require_version("Gtk", "3.0")
        gi.require_version("WebKit2", "4.0")
        from gi.repository import GLib, Gtk, WebKit2
    except (ImportError, ValueError) as exc:
        raise RuntimeError(
            "GTK/WebKit is unavailable. Install the runtime packages "
            "'python3-gi' and 'gir1.2-webkit2-4.0'."
        ) from exc
    return GLib, Gtk, WebKit2


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)

    try:
        GLib, Gtk, WebKit2 = _load_gtk()
    except RuntimeError as exc:
        print(f"camrod_ui_window: {exc}", file=sys.stderr)
        return 2

    try:
        window = Gtk.Window(title="CAMROD Operator UI")
        web_view = WebKit2.WebView()
    except RuntimeError as exc:
        print(
            "camrod_ui_window: GTK could not open a display. "
            f"Run it from a graphical session. Details: {exc}",
            file=sys.stderr,
        )
        return 3

    window.set_default_size(args.width, args.height)
    window.add(web_view)

    # HH_260727 - Full bringup can start the WebKit process before uvicorn has
    # bound its socket. Retry only until the first complete page load, then
    # remove the timer so an operational UI is never refreshed unexpectedly.
    window_closed = False
    load_succeeded = False
    load_in_progress = False
    attempt_failed = False
    load_attempt_count = 0
    load_failure_count = 0
    retry_source_id: int | None = None

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
                "retrying every 1 second",
                file=sys.stderr,
                flush=True,
            )

    def on_load_changed(_view: Any, load_event: Any) -> None:
        nonlocal load_in_progress, load_succeeded, retry_source_id
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
        if retry_source_id is not None:
            GLib.source_remove(retry_source_id)
            retry_source_id = None
        print(
            f"camrod_ui_window: loaded {args.url} "
            f"(attempt {load_attempt_count})",
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
        # Suppress WebKit's static error page; the retry timer owns recovery.
        return True

    def retry_until_loaded() -> bool:
        nonlocal retry_source_id
        if window_closed or load_succeeded:
            retry_source_id = None
            return GLib.SOURCE_REMOVE
        request_load()
        return GLib.SOURCE_CONTINUE

    def quit_main_loop(*_unused: object) -> None:
        nonlocal retry_source_id, window_closed
        window_closed = True
        if retry_source_id is not None:
            GLib.source_remove(retry_source_id)
            retry_source_id = None
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

    window.set_title("CAMROD Operator UI — Connecting…")
    retry_source_id = GLib.timeout_add_seconds(1, retry_until_loaded)
    request_load()
    window.show_all()
    Gtk.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
