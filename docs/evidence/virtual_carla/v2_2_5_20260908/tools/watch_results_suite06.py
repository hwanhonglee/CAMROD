#!/usr/bin/env python3
"""Explicit suite and source-bound evidence updater. No robot/browser access."""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
SUITE = "full_suite_v224_06"
SUITE_NAME = re.compile(r"full_suite_v224_[0-9]{2}")
HEAD = "9f7cd2bacda17a4581bd44ed66aaba857e89f1d4"
POLL_SECONDS = 30.0
REFRESH_SECONDS = 60.0
MAX_SECONDS = 12 * 60 * 60
TERMINAL = {"PASS", "FAIL", "COMPLETE", "COMPLETED", "ERROR", "INTERRUPTED", "CANCELLED", "CANCELED", "ABORTED"}
STAGE = re.compile(r"B(?:[1-9]|1[0-3])_(?:robot_delivery|operator_recall|guest_recall_robot_handoff|optional_docking)")


def validate_suite_name(suite_name):
    if not isinstance(suite_name, str) or not SUITE_NAME.fullmatch(suite_name):
        raise ValueError("An explicit full_suite_v224_NN directory name is required")
    return suite_name


def suite_directory(suite_name, root=HERE):
    validate_suite_name(suite_name)
    target = (root / suite_name).resolve()
    target.relative_to(root.resolve())
    return target


def status_paths(suite_name, root=HERE):
    suite_directory(suite_name, root)
    suffix = suite_name.rsplit("_", 1)[1]
    return (root / f"results_live_suite{suffix}_status.json",
            root / f".results_live_suite{suffix}_status.{os.getpid()}.tmp")


def source_sha(value):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{40}", value):
        raise ValueError("An explicit full lowercase 40-character source SHA is required")
    return value


def read_object(path, root):
    """Read a named small manifest, never discover/traverse arbitrary paths."""
    try:
        path.resolve().relative_to(root.resolve())
        if path.stat().st_size > 20 * 1024 * 1024:
            return {"read_error": "manifest exceeds 20 MiB"}
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {"read_error": "not an object"}
    except (OSError, ValueError, TypeError) as error:
        return {"read_error": type(error).__name__}


def snapshot(root=HERE, suite_name=SUITE):
    suite_root = suite_directory(suite_name, root)
    suite = read_object(suite_root / "suite_status.json", suite_root)
    # Producer heartbeats alone do not count as meaningful state changes.
    semantic = {key: value for key, value in suite.items() if key != "updated_utc"}
    names = {suite.get("active_stage"), suite.get("failed_stage")}
    names.update(item.get("stage") for item in suite.get("completed", []) if isinstance(item, dict))
    manifests = {}
    for name in sorted(name for name in names if isinstance(name, str) and STAGE.fullmatch(name)):
        site = name.split("_", 1)[0]
        for relative in (f"{name}/run_manifest.json", f"{name}/{site}/site_manifest.json",
                         f"{name}.strict_validation/site_evidence_collection.json",
                         f"{name}/functional/result.json" if name.endswith("optional_docking") else None):
            if relative is None:
                continue
            value = read_object(suite_root / relative, suite_root)
            manifests[relative] = {key: value.get(key) for key in
                                  ("read_error", "status", "failure_site", "failure_reason", "failure_reasons",
                                   "completed_site_manifests", "motion_metrics", "metrics_summary", "aggregate",
                                   "charging_confirmed", "source", "sites") if key in value}
    digest = hashlib.sha256(json.dumps({"suite": semantic, "manifests": manifests},
                                      sort_keys=True, ensure_ascii=False).encode()).hexdigest()
    return {"status": suite.get("status", "UNAVAILABLE"), "active_stage": suite.get("active_stage"),
            "fingerprint": digest, "manifest_count": len(manifests)}


def index_command(suite_name=SUITE, expected_head=HEAD, runtime_head=HEAD):
    suite_directory(suite_name)
    return [sys.executable, "-B", str(HERE / "update_results_index.py"), "--suite", suite_name,
            "--expected-head", source_sha(expected_head), "--runtime-head", source_sha(runtime_head)]


def invoke_index(remaining_seconds, suite_name=SUITE, expected_head=HEAD, runtime_head=HEAD):
    try:
        result = subprocess.run(index_command(suite_name, expected_head, runtime_head), cwd=HERE, check=False, capture_output=True,
                                text=True, timeout=max(0.1, min(25.0, remaining_seconds)))
        return result.returncode, (result.stdout + result.stderr)[-8000:]
    except subprocess.TimeoutExpired:
        return 124, "Index update exceeded its bounded timeout"


def write_observer_status(value, suite_name=SUITE, expected_head=HEAD, runtime_head=HEAD):
    target, temporary = status_paths(suite_name)
    if target.is_symlink() or temporary.is_symlink():
        raise RuntimeError("Refuse observer status symlinks")
    value = dict(value, updated_utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                 suite=suite_name, expected_head=source_sha(expected_head), runtime_head=source_sha(runtime_head), pid=os.getpid())
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(target)


def run_loop(read_snapshot=snapshot, update=invoke_index, write_status=write_observer_status,
             monotonic=time.monotonic, sleep=time.sleep, max_seconds=MAX_SECONDS):
    started = monotonic()
    last_fingerprint = None
    last_refresh = None
    updates = 0
    last_returncode = None
    while True:
        elapsed = monotonic() - started
        if elapsed >= min(max_seconds, MAX_SECONDS):
            write_status({"observer_status": "HARD_LIMIT", "updates": updates, "elapsed_s": elapsed})
            return 2
        current = read_snapshot()
        terminal = current["status"] in TERMINAL
        due = last_refresh is None or elapsed - last_refresh >= REFRESH_SECONDS
        changed = current["fingerprint"] != last_fingerprint
        if changed or due or terminal:
            reason = "terminal_final" if terminal else "meaningful_change" if changed else "60s_refresh"
            last_returncode, output = update(min(max_seconds, MAX_SECONDS) - elapsed)
            updates += 1
            print(f"{dt.datetime.now(dt.timezone.utc).isoformat()} {reason} "
                  f"status={current['status']} stage={current.get('active_stage')} rc={last_returncode}\n{output}", flush=True)
            last_refresh = monotonic() - started
            if last_returncode == 0:
                last_fingerprint = current["fingerprint"]
        observer_status = "FINAL_UPDATED" if terminal and last_returncode == 0 else "FINAL_UPDATE_FAILED" if terminal else "RUNNING"
        write_status(dict(current, observer_status=observer_status, updates=updates,
                          last_index_returncode=last_returncode, elapsed_s=monotonic() - started,
                          next_poll_seconds=POLL_SECONDS))
        if terminal:
            return 0 if last_returncode == 0 else 1
        remaining = min(max_seconds, MAX_SECONDS) - (monotonic() - started)
        if remaining > 0:
            sleep(min(POLL_SECONDS, remaining))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", required=True, type=validate_suite_name)
    parser.add_argument("--expected-head", required=True, type=source_sha)
    parser.add_argument("--runtime-head", required=True, type=source_sha)
    args = parser.parse_args()
    suite_root = suite_directory(args.suite)
    if not all((suite_root / name).is_file() for name in ("suite_status.json", "suite_plan.json")):
        parser.error("Actual suite status and plan must already exist; no automatic suite discovery/start")
    return run_loop(read_snapshot=lambda: snapshot(suite_name=args.suite),
                    update=lambda remaining: invoke_index(remaining, args.suite, args.expected_head, args.runtime_head),
                    write_status=lambda value: write_observer_status(value, args.suite, args.expected_head, args.runtime_head))


if __name__ == "__main__":
    raise SystemExit(main())
