#!/usr/bin/env python3
"""Read-only CAMROD UI readiness/idle preflight for keyboard control."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import json
import math
import sys
import urllib.error
import urllib.request


MAX_RESPONSE_BYTES = 1024 * 1024


class UiManualReadyError(RuntimeError):
    """Expected, user-facing UI readiness or idle-state failure."""


def validate_health(document) -> None:
    if not isinstance(document, Mapping) or document.get("ok") is not True:
        raise UiManualReadyError(
            f"CAMROD UI health is not ok: {document!r}"
        )


def validate_state(document) -> None:
    if not isinstance(document, Mapping):
        raise UiManualReadyError("CAMROD UI state is not a JSON object")
    mismatches = []
    if document.get("ready") is not True:
        mismatches.append(f"ready={document.get('ready')!r} (expected True)")
    if document.get("engaged") is not False:
        mismatches.append(
            f"engaged={document.get('engaged')!r} (expected False)"
        )
    mission_phase = str(document.get("mission_phase", "")).strip().upper()
    if mission_phase != "READY":
        mismatches.append(
            f"mission_phase={document.get('mission_phase')!r} "
            "(expected 'READY')"
        )
    mission_source = str(document.get("mission_source", "")).strip().lower()
    if mission_source != "none":
        mismatches.append(
            f"mission_source={document.get('mission_source')!r} "
            "(expected 'none')"
        )
    if mismatches:
        ready_message = str(document.get("ready_message", "")).strip()
        detail = f"; ready_message={ready_message}" if ready_message else ""
        raise UiManualReadyError(
            "CAMROD UI is not healthy and idle for manual control: "
            f"{', '.join(mismatches)}{detail}. Press UI STOP, wait for "
            "ready=true, engaged=false, mission_phase=READY and "
            "mission_source=none, then retry manual."
        )


def _positive_timeout(value: str) -> float:
    try:
        result = float(value)
    except ValueError:
        raise argparse.ArgumentTypeError("timeout must be a number") from None
    if not math.isfinite(result) or result <= 0.0 or result > 10.0:
        raise argparse.ArgumentTypeError(
            "timeout must be finite, positive, and at most 10 seconds"
        )
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", required=True)
    parser.add_argument("--timeout-seconds", type=_positive_timeout, default=2.0)
    return parser


def _read_json(url: str, timeout_seconds: float):
    request = urllib.request.Request(
        url,
        headers={"Accept": "application/json"},
        method="GET",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout_seconds) as response:
            status = int(getattr(response, "status", 0))
            if status != 200:
                raise UiManualReadyError(f"GET {url} returned HTTP {status}")
            payload = response.read(MAX_RESPONSE_BYTES + 1)
    except (OSError, TimeoutError, urllib.error.URLError) as error:
        raise UiManualReadyError(
            f"GET {url} failed: {type(error).__name__}: {error}"
        ) from None
    if len(payload) > MAX_RESPONSE_BYTES:
        raise UiManualReadyError(f"GET {url} response exceeds 1 MiB")
    try:
        return json.loads(payload.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise UiManualReadyError(
            f"GET {url} returned invalid JSON: {error}"
        ) from None


def check_ui(base_url: str, timeout_seconds: float) -> None:
    root = base_url.strip().rstrip("/")
    if not root.startswith(("http://", "https://")):
        raise UiManualReadyError("CAMROD UI base URL must use http or https")
    validate_health(_read_json(f"{root}/ui/health", timeout_seconds))
    validate_state(_read_json(f"{root}/ui/state", timeout_seconds))


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        check_ui(args.base_url, args.timeout_seconds)
    except KeyboardInterrupt:
        return 130
    except UiManualReadyError as error:
        print(
            f"[virtual_carla] ERROR: CAMROD UI manual preflight: {error}",
            file=sys.stderr,
        )
        return 1
    except Exception as error:  # Unexpected HTTP data must fail closed.
        print(
            "[virtual_carla] ERROR: CAMROD UI manual preflight failed closed: "
            f"{type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 1

    print(
        "[virtual_carla] CAMROD UI ready and idle for manual control: "
        f"{args.base_url.rstrip('/')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
