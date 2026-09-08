#!/usr/bin/env python3
"""Sequential real-UI evidence suite. Default plan is entirely offline.

Execute only in a dedicated systemd user service, after the runtime is healthy
and the robot is reverse PARKED. No automatic restart, retry, reset, or push.
"""
import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("CAMROD_SRC_ROOT", "/home/hong/camrod_ws/src"))
SCRIPTS = SRC / "scripts/virtual_carla"


def utc():
    return dt.datetime.now(dt.timezone.utc).isoformat()


def stages(skip_dock=False):
    result = []
    for number in range(1, 14):
        site = f"B{number}"
        for name, authority, intent in (("robot_delivery", "operator-browser", "delivery"),
                                        ("operator_recall", "operator-browser", "recall"),
                                        ("guest_recall_robot_handoff", "guest", "recall")):
            result.append(dict(name=f"{site}_{name}", site=site, authority=authority, intent=intent))
        if number == 1 and not skip_dock:
            result.append(dict(name="B1_optional_docking", kind="dock"))
    return result


def artifact(path):
    path = Path(path)
    if not path.is_file():
        raise ValueError(f"Required evidence is not a regular file: {path}")
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            size += len(block)
            digest.update(block)
    return dict(path=str(path), bytes=size, sha256=digest.hexdigest())


def driver_identity():
    # Git HEAD here identifies test-driver checkout, NOT the already running
    # robot binary. Its actual runtime binding remains in strict stage reports.
    head = subprocess.check_output(["git", "-C", str(SRC), "rev-parse", "HEAD"], text=True).strip()
    files = [SCRIPTS / name for name in ("camping_site_matrix.py", "run_site_evidence_matrix.sh",
                                       "validate_site_evidence_collection.py", "capture_ui_evidence.sh")]
    return dict(checkout_head=head, meaning="test_driver_checkout_not_live_runtime_binary",
                files=[artifact(path) for path in files], suite_helper=artifact(Path(__file__)))


def accepted_matrix_reference(prior, stage):
    """Read/hash prior evidence only; never rerun a newer validator on history."""
    stage_root = prior / stage["name"]
    run_path = stage_root / "run_manifest.json"
    site_path = stage_root / stage["site"] / "site_manifest.json"
    strict_path = prior / (stage["name"] + ".strict_validation") / "site_evidence_collection.json"
    run, site, strict = [json.loads(path.read_text()) for path in (run_path, site_path, strict_path)]
    if any(value.get("status") != "PASS" for value in (run, site, strict)):
        raise ValueError(f"Prior stage requires PASS run/site/strict manifests: {stage['name']}")
    if run.get("selected_sites") != [stage["site"]] or run.get("sites") != [site]:
        raise ValueError("Prior run and site manifests disagree")
    identity = dict(frontend=stage["authority"], mission_intent=stage["intent"])
    if any(site.get("authority", {}).get(key) != value for key, value in identity.items()):
        raise ValueError("Prior stage authority/mission does not match requested skipped stage")
    if strict.get("expectation", {}).get("sites") != [stage["site"]] or len(strict.get("sites", [])) != 1:
        raise ValueError("Prior strict report does not cover exactly the skipped site")
    row = strict["sites"][0]
    if (row.get("status") != "PASS" or row.get("site") != stage["site"] or
            row.get("authority") != stage["authority"] or row.get("mission_intent") != stage["intent"] or
            strict.get("aggregate", {}).get("fail_count") != 0 or
            strict.get("aggregate", {}).get("pass_count") != 1):
        raise ValueError("Prior strict stage acceptance is not exact")
    if Path(strict["source"]["input_root"]).resolve() != stage_root.resolve():
        raise ValueError("Prior strict report refers to another stage directory")
    if artifact(site_path)["sha256"] != row.get("site_manifest_sha256"):
        raise ValueError("Prior site manifest has changed since strict validation")
    checksums = strict_path.parent / "SHA256SUMS"
    entries = [line.split(None, 1) for line in checksums.read_text().splitlines() if line.strip()]
    claimed = [digest for digest, name in entries if name.lstrip("*") == strict_path.name]
    if claimed != [artifact(strict_path)["sha256"]]:
        raise ValueError("Prior strict validation JSON checksum is missing or changed")
    # Recheck the native matrix, PNG/GIF, wheel evidence and logs whose hashes
    # were accepted then. Never compare historical source-file hashes with the
    # newer test driver, and never touch production source through manifests.
    evidence_root = Path(os.environ["RANGER_EVIDENCE_ROOT"]).resolve()
    checked = []

    def verify_claims(value):
        if isinstance(value, dict):
            if all(key in value for key in ("path", "bytes", "sha256")):
                path = Path(value["path"])
                if not path.resolve().is_relative_to(evidence_root):
                    raise ValueError(f"Prior evidence points outside the evidence root: {path}")
                actual = artifact(path)
                if any(actual[key] != value[key] for key in ("bytes", "sha256")):
                    raise ValueError(f"Prior accepted artifact changed: {path}")
                checked.append(actual)
            else:
                for child in value.values():
                    verify_claims(child)
        elif isinstance(value, list):
            for child in value:
                verify_claims(child)

    verify_claims(site)
    verify_claims(run.get("metrics_summary", {}))
    if row.get("matrix_sha256") != site.get("matrix_report", {}).get("sha256"):
        raise ValueError("Prior native matrix identity differs between manifests")
    return dict(acceptance="PRIOR_ACCEPTED_EVIDENCE", prior_stage_root=str(stage_root),
                run_manifest=artifact(run_path), site_manifest=artifact(site_path),
                strict_report=artifact(strict_path), strict_checksums=artifact(checksums),
                prior_test_driver_checkout_head=run.get("source", {}).get("head"),
                prior_runner_identity=run.get("source", {}).get("runner"),
                prior_runtime_identity={key: row.get(key) for key in ("runtime_source_head", "runtime_source_branch",
                    "runtime_install_sha256", "runtime_audit_sha256", "software_binding_sha256", "launch_cmdline_sha256")},
                accepted_metrics={key: row.get(key) for key in ("elapsed_s", "total_odom_distance_m", "drop_zone_error_m")},
                checked_artifacts=checked,
                note="Historical strict report is retained; its runner prepass hash predates final PASS rewrite. No historical driver/runtime identity is relabeled.")


def continuation_stage_order(all_stages, prior):
    """Recover an exact full permutation, not arbitrary commands or acceptance."""
    if prior is None or not prior.is_absolute() or prior.is_symlink():
        raise ValueError("--use-continuation-order requires an absolute non-symlink continuation suite")
    plan = json.loads((prior / "suite_plan.json").read_text())
    status = json.loads((prior / "suite_status.json").read_text())
    if status.get("status") not in {"PASS", "FAIL"}:
        raise ValueError("Cannot resume the order of a still-running or unknown suite")
    order = plan.get("stage_order", plan.get("stages"))
    canonical = {stage["name"]: stage for stage in all_stages}
    if (not isinstance(order, list) or len(order) != len(all_stages)
            or any(not isinstance(stage, dict) for stage in order)
            or len({stage.get("name") for stage in order}) != len(all_stages)
            or any(canonical.get(stage.get("name")) != stage for stage in order)):
        raise ValueError("Prior stage order must contain every exact canonical case once")
    return order


def select_stages(all_stages, start_at, continuation_of, *, defer_earlier_stages=False):
    names = [stage["name"] for stage in all_stages]
    if start_at not in names:
        raise ValueError(f"Unknown/disabled --start-at stage: {start_at}")
    index = names.index(start_at)
    if defer_earlier_stages:
        if continuation_of is not None:
            raise ValueError("--defer-earlier-stages cannot import prior acceptance with --continuation-of")
        if index == 0:
            raise ValueError("--defer-earlier-stages requires a later --start-at")
        # Reorder a fresh run, never omit or accept a previous stage. Every
        # original case executes exactly once before this suite may pass.
        return all_stages[index:] + all_stages[:index], []
    if continuation_of is not None and (not continuation_of.is_absolute() or continuation_of.is_symlink() or not index):
        raise ValueError("--continuation-of requires an existing absolute suite and a later --start-at stage")
    previous = None
    if continuation_of is not None:
        previous = json.loads((continuation_of / "suite_status.json").read_text())
    skipped = []
    for stage in all_stages[:index]:
        record = dict(stage=stage["name"], status="NOT_EXECUTED_IN_THIS_SUITE", prior_evidence=None)
        if previous is not None:
            matches = [value for value in previous.get("completed", []) if value.get("stage") == stage["name"]]
            prior_skips = [value for value in previous.get("skipped_prior_stages", [])
                           if value.get("stage") == stage["name"]]
            empty_skip = dict(stage=stage["name"], status="NOT_EXECUTED_IN_THIS_SUITE", prior_evidence=None)
            if (not matches and prior_skips == [empty_skip]
                    and previous.get("failed_stage") != stage["name"]):
                # Carry only an explicit absence of evidence, never transitively
                # import PASS or reinterpret a failed stage as unexecuted.
                skipped.append(record)
                continue
            if prior_skips:
                raise ValueError(f"Prior suite did not itself execute and accept {stage['name']}: ambiguous or accepted skip")
            if len(matches) != 1 or matches[0].get("status") != "PASS":
                raise ValueError(f"Prior suite did not itself execute and accept {stage['name']}")
            if stage.get("kind") == "dock":
                raise ValueError("Continuation across prior optional docking needs its own acceptance contract; not supported")
            record["prior_evidence"] = accepted_matrix_reference(continuation_of, stage)
        skipped.append(record)
    return all_stages[index:], skipped


def matrix_command(stage, root, action, args):
    return [str(SCRIPTS / "run_site_evidence_matrix.sh"), action,
            "--authority", stage["authority"], "--mission-intent", stage["intent"],
            "--guest-final-return-authority", "robot", "--sites", stage["site"],
            "--output-root", str(root / stage["name"]),
            "--phase-timeout-s", str(args.phase_timeout_s),
            "--capture-fps", str(args.capture_fps), "--gif-fps", "8",
            "--derived-width", "1920", "--wheel-rate-hz", "10",
            "--retain-source-video", "false", "--display", ":0",
            "--xauthority", os.environ["XAUTHORITY"]]


def target(kind):
    port, url = (9223, "http://127.0.0.1:8012") if kind == "guest" else (9224, "http://127.0.0.1:8010")
    with urllib.request.urlopen(f"http://127.0.0.1:{port}/json", timeout=2) as response:
        values = json.load(response)
    pages = [item for item in values if item.get("type") == "page" and item.get("url", "").rstrip("/") == url]
    if len(pages) != 1 or not pages[0].get("webSocketDebuggerUrl", "").startswith(f"ws://127.0.0.1:{port}/"):
        raise RuntimeError(f"Expected exactly one local production {kind} page on {port}")
    return pages[0]


def foreground(page):
    import websocket
    # Focus the existing, actual page only. Never mutate DOM/UI/application state.
    connection = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=5, suppress_origin=True)
    try:
        connection.send(json.dumps({"id": 1, "method": "Page.bringToFront"}))
        while True:
            reply = json.loads(connection.recv())
            if reply.get("id") == 1:
                if "error" in reply:
                    raise RuntimeError(str(reply["error"]))
                return
    finally:
        connection.close()


def window(title, *, exact=False):
    deadline = time.monotonic() + 5.0
    while True:
        # C locale can transliterate Korean _NET_WM_NAME before Python sees it.
        lines = subprocess.check_output(["wmctrl", "-lpG"], text=True, encoding="utf-8",
            env={**os.environ, "LC_ALL": "C.UTF-8"},
            timeout=max(0.001, deadline - time.monotonic())).splitlines()
        rows = [line.split(None, 8) for line in lines if len(line.split(None, 8)) == 9]
        matches = [row for row in rows if (row[8] == title if exact else title in row[8])]
        if len(matches) > 1:
            raise RuntimeError(f"Expected exactly one X11 window containing {title!r}: {matches}")
        if matches:
            values = matches[0]
            return dict(id=values[0], pid=int(values[2]), x=int(values[3]), y=int(values[4]),
                        width=int(values[5]), height=int(values[6]), title=values[8])
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(f"Expected exactly one X11 window containing {title!r}: [] (5 s startup timeout)")
        time.sleep(min(0.1, remaining))


def arrange(kind):
    page = target(kind)
    foreground(page)
    title = "국립공원 로봇 서비스" if kind == "guest" else "Robot UI"
    carla, selected = window("CarlaUE4"), window(title, exact=kind == "guest")
    robot = window("Robot UI")
    # Both genuine browser windows occupy the same right pane. Handoffs use
    # Page.bringToFront in the production matrix; the CARLA pane never moves.
    # Mutter clamps this UE window's client left edge to x=140 even when the
    # requested frame x is 80. Keep its measured right edge at 1920, not 1980,
    # so it cannot overlap the browser. The capture's independent X11 geometry
    # check remains authoritative; never widen that acceptance tolerance.
    positions = {carla["id"]: (80, 60, 1780, 2010), robot["id"]: (1920, 60, 1840, 2010),
                 selected["id"]: (1920, 60, 1840, 2010)}
    for identifier, geometry in positions.items():
        subprocess.run(["wmctrl", "-i", "-r", identifier, "-b", "remove,maximized_vert,maximized_horz"], check=True)
        subprocess.run(["wmctrl", "-i", "-r", identifier, "-e", "0," + ",".join(map(str, geometry))], check=True)
    subprocess.run(["wmctrl", "-i", "-a", carla["id"]], check=True)
    foreground(page)
    subprocess.run(["wmctrl", "-i", "-a", selected["id"]], check=True)
    time.sleep(.5)
    return dict(utc=utc(), kind=kind, cdp_page_id=page["id"], url=page["url"],
                carla=window("CarlaUE4"), selected_ui=window(title, exact=kind == "guest"), robot_ui=window("Robot UI"))


def guest_preparation_server_state():
    """Read-only admission snapshot; this is not a manual-drive lease query."""
    with urllib.request.urlopen("http://127.0.0.1:8010/ui/state", timeout=3) as response:
        state = json.load(response)
    keys = ("service_state", "service_state_name", "ready", "engaged", "mission_dispatch_active",
            "mission_dispatch_generation", "mission_dispatch_owner", "mission_dispatch_intent",
            "mission_phase", "mission_source")
    return {key: state.get(key) for key in keys}


def require_guest_preparation_idle(state):
    # Do not label these observations as proof of the private manual-drive
    # policy's `armed` flag. Opening that WebSocket would acquire a lease.
    if (type(state.get("service_state")) is not int or state["service_state"] not in (0, 12, 13)
            or state.get("ready") is not True or state.get("engaged") is not False
            or state.get("mission_dispatch_active") is not False):
        raise RuntimeError("Guest page preparation requires ready, inactive, disengaged idle/station server state")


def guest_page_call(page, method, params=None):
    import websocket
    connection = websocket.create_connection(page["webSocketDebuggerUrl"], timeout=3, suppress_origin=True)
    try:
        connection.send(json.dumps(dict(id=1, method=method, params=params or {})))
        while True:
            reply = json.loads(connection.recv())
            if reply.get("id") == 1:
                if "error" in reply:
                    raise RuntimeError("Guest preparation CDP request failed: " + str(reply["error"])[:400])
                return reply.get("result", {})
    finally:
        connection.close()


def guest_page_snapshot(page):
    # Observe actual production globals/DOM only; never assign lifecycle state,
    # clear storage, call mission handlers, or manufacture a visible picker.
    expression = """(() => {
      const visible = el => {
        if (!el || !el.getBoundingClientRect().width || !el.getBoundingClientRect().height) return false;
        for (let p=el; p; p=p.parentElement) {
          const s=getComputedStyle(p);
          if (s.display==='none' || s.visibility!=='visible' || Number(s.opacity)<=0) return false;
        }
        return true;
      };
      return {url:location.href, title:document.title, ready_state:document.readyState,
        time_origin:performance.timeOrigin, ws_ready:typeof ws!=='undefined' && ws ? ws.readyState : -1,
        identity_revision:typeof lastIdentityRevision!=='undefined' ? lastIdentityRevision : null,
        service_state:typeof currentState!=='undefined' ? currentState : null,
        phase:typeof currentPhase!=='undefined' ? currentPhase : null,
        active_intent:typeof activeRequestIntent!=='undefined' ? activeRequestIntent : null,
        mission_generation:typeof activeMissionGeneration!=='undefined' ? activeMissionGeneration : null,
        site_card_visible:visible(document.querySelector('#siteCard')),
        visible_site_count:[...document.querySelectorAll('#siteGrid .site-btn')].filter(visible).length};
    })()"""
    result = guest_page_call(page, "Runtime.evaluate", dict(expression=expression, returnByValue=True))
    if result.get("exceptionDetails") or not isinstance(result.get("result", {}).get("value"), dict):
        raise RuntimeError("Guest preparation observation is not available yet")
    return result["result"]["value"]


def prepare_guest_page(page, *, reload_existing, log, evidence_path=None, timeout_s=20.0):
    record = dict(started_utc=utc(), page_id=page["id"], same_url=page["url"],
                  action="Page.reload" if reload_existing else "observe_new_page", reload_count=0,
                  manual_drive_armed="not_observed_no_manual_websocket_opened", status="PREPARING")
    try:
        record["server_before"] = guest_preparation_server_state()
        require_guest_preparation_idle(record["server_before"])
        if page.get("url", "").rstrip("/") != "http://127.0.0.1:8012":
            raise RuntimeError("Guest preparation refuses a different URL")
        record["before"] = guest_page_snapshot(page)
        if reload_existing:
            guest_page_call(page, "Page.reload", dict(ignoreCache=False))
            record["reload_count"] = 1
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                after = guest_page_snapshot(page)
                record["after"] = after
                fresh_document = not reload_existing or after.get("time_origin") != record["before"].get("time_origin")
                if (fresh_document and after.get("url", "").rstrip("/") == "http://127.0.0.1:8012"
                        and after.get("ready_state") == "complete" and after.get("ws_ready") == 1
                        and type(after.get("identity_revision")) is int and after["identity_revision"] >= 0
                        and after.get("service_state") == record["server_before"]["service_state"]
                        and not after.get("active_intent") and after.get("site_card_visible") is True
                        and after.get("visible_site_count", 0) > 0):
                    record["server_after"] = guest_preparation_server_state()
                    require_guest_preparation_idle(record["server_after"])
                    if record["server_after"]["service_state"] != after["service_state"]:
                        raise RuntimeError("Server station state changed during Guest page preparation")
                    record["status"] = "READY_AFTER_RELOAD" if reload_existing else "READY_NEW_PAGE"
                    return record
            except (OSError, RuntimeError) as error:
                record["last_observation_error"] = str(error)[:400]
            if time.monotonic() >= deadline:
                raise RuntimeError("Guest page did not receive fresh idle identity and visible site picker after preparation")
            time.sleep(.1)
    except BaseException as error:
        record["status"] = "FAIL_BEFORE_DISPATCH"
        record["error"] = str(error)[:600]
        raise
    finally:
        record["finished_utc"] = utc()
        encoded = json.dumps(record, ensure_ascii=False, indent=2)
        log.write("Guest preparation observation: " + json.dumps(record, ensure_ascii=False) + "\n")
        log.flush()
        if evidence_path is not None:
            with Path(evidence_path).open("x") as output:
                output.write(encoded + "\n")


def ensure_guest(unit, log, evidence_path=None):
    try:
        page = target("guest")
    except (OSError, urllib.error.URLError):
        page = None
    if page is not None:
        return dict(reused_existing_valid_production_page=True, page_id=page["id"],
                    preparation=prepare_guest_page(page, reload_existing=True, log=log, evidence_path=evidence_path))
    command = ["systemd-run", "--user", "--unit=" + unit, "--property=Restart=no",
               "--property=KillSignal=SIGINT", "--property=TimeoutStopSec=30",
               "/bin/bash", str(HERE / "run_durable_suite.sh"), "guest"]
    subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        try:
            page = target("guest")
        except (OSError, urllib.error.URLError):
            time.sleep(.25)
            continue
        return dict(unit=unit, page_id=page["id"], started_utc=utc(),
                    preparation=prepare_guest_page(page, reload_existing=False, log=log, evidence_path=evidence_path))
    raise RuntimeError(f"Guest browser did not become ready; inspect journalctl --user -u {unit}")


def dock(stage_root, args, log):
    stage_root.mkdir()
    capture = stage_root / "desktop"
    command = [str(SCRIPTS / "capture_ui_evidence.sh"), "capture", "--output-dir", str(capture),
               "--duration-seconds", "86400", "--capture-fps", str(args.capture_fps),
               "--gif-fps", "8", "--derived-width", "1920", "--retain-source-video", "false",
               "--allow-short-capture", "true", "--display", ":0", "--xauthority", os.environ["XAUTHORITY"],
               "--ui-window-title", "Robot UI", "--ui-kind", "operator"]
    recorder = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
    start = time.monotonic()
    error = None
    try:
        while not (capture / "carla_camrod_desktop.mp4").exists():
            if recorder.poll() is not None or time.monotonic() - start > 30:
                raise RuntimeError("Dock desktop capture failed to start; no docking command issued")
            time.sleep(.2)
        subprocess.run([sys.executable, str(HERE / "run_optional_docking_ui.py"), "--run",
                        "--camrod-root", str(SRC), "--output", str(stage_root / "functional"),
                        "--timeout", str(args.docking_timeout_s)], check=True, stdout=log, stderr=subprocess.STDOUT)
    except BaseException as caught:
        error = caught
    finally:
        while recorder.poll() is None and time.monotonic() - start < 14:
            time.sleep(.2)
        if recorder.poll() is None:
            try:
                recorder.stdin.write(b"q")
                recorder.stdin.flush()
            except BrokenPipeError:
                pass
        capture_result = recorder.wait(timeout=180)
    if error is not None:
        raise error
    if capture_result:
        raise RuntimeError(f"Dock desktop PNG/GIF finalization failed: {capture_result}")
    result = json.loads((stage_root / "functional/result.json").read_text())
    if result.get("status") != "PASS" or not result.get("charging_confirmed"):
        raise RuntimeError("Dock functional evidence did not prove charging completion")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", choices=("plan", "run"), default="plan")
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--start-at", default="B1_robot_delivery", help="first stage executed in this new suite")
    parser.add_argument("--continuation-of", type=Path, help="prior suite whose actually accepted earlier stages are referenced")
    parser.add_argument("--defer-earlier-stages", action="store_true",
                        help="without continuation, execute earlier stages last instead of skipping them")
    parser.add_argument("--use-continuation-order", action="store_true",
                        help="resume the prior complete canonical order, retaining original PASS evidence")
    parser.add_argument("--skip-optional-dock", action="store_true")
    parser.add_argument("--capture-fps", type=int, default=5)
    parser.add_argument("--phase-timeout-s", type=float, default=900)
    parser.add_argument("--docking-timeout-s", type=float, default=300)
    args = parser.parse_args()
    root = args.output_root
    if not root.is_absolute() or root == Path("/") or root.is_symlink() or root.exists():
        parser.error("--output-root must be a new, non-symlink absolute directory")
    try:
        all_stages = stages(args.skip_optional_dock)
        if args.use_continuation_order:
            if args.defer_earlier_stages:
                raise ValueError("Cannot defer and restore the prior order together")
            all_stages = continuation_stage_order(all_stages, args.continuation_of)
        selected, skipped = select_stages(all_stages, args.start_at, args.continuation_of,
                                         defer_earlier_stages=args.defer_earlier_stages)
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    plan = dict(action=args.action, created_utc=utc(), output_root=str(root), stages=selected,
                start_at=args.start_at, continuation_of=str(args.continuation_of) if args.continuation_of else None,
                defer_earlier_stages=args.defer_earlier_stages,
                use_continuation_order=args.use_continuation_order,
                stage_order=(selected if args.defer_earlier_stages else all_stages),
                skipped_prior_stages=skipped, total_planned_stages=len(all_stages), execution_stage_count=len(selected),
                test_driver_identity=driver_identity(),
                env={key: os.environ.get(key) for key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "DISPLAY", "XAUTHORITY",
                     "CAMROD_CARLA_MAP_PROFILE", "CARLA_UE_MAP", "CARLA_TOWN")},
                stop_on_first_failure=True, automatically_push=False)
    if args.action == "plan":
        # Exercise all three canonical launch selections without network, X11,
        # directories, captures, browser services, or vehicle commands.
        checked_commands = set()
        for stage in selected:
            if stage.get("kind") == "dock":
                continue
            key = (stage["authority"], stage["intent"])
            if key in checked_commands:
                continue
            checked_commands.add(key)
            subprocess.run(matrix_command(stage, root, "plan", args), check=True)
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        return
    if not os.environ.get("INVOCATION_ID"):
        parser.error("run must be launched by a dedicated systemd --user service; see DURABLE_SUITE.md")
    root.mkdir(parents=True)
    (root / "logs").mkdir()
    (root / "suite_plan.json").write_text(json.dumps(plan, ensure_ascii=False, indent=2))
    state = dict(status="RUNNING", started_utc=utc(), active_stage=None, completed=[], failed_stage=None,
                 guest_service=None, total_stages=len(selected), status_scope="STAGES_EXECUTED_IN_THIS_SUITE_ONLY",
                 total_planned_stages=len(all_stages), skipped_prior_stages=skipped,
                 continuation_of=plan["continuation_of"], test_driver_identity=plan["test_driver_identity"])
    state["defer_earlier_stages"] = args.defer_earlier_stages

    def save():
        state["updated_utc"] = utc()
        temporary = root / "suite_status.json.tmp"
        temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2))
        temporary.replace(root / "suite_status.json")

    def event(value):
        with (root / "suite_events.jsonl").open("a") as output:
            output.write(json.dumps(dict(utc=utc(), **value), ensure_ascii=False) + "\n")
        print(json.dumps(value, ensure_ascii=False), flush=True)

    def interrupted(number, _frame):
        raise RuntimeError(f"Suite received signal {number}; do not auto-retry a possibly active mission")

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        save()
        for index, stage in enumerate(selected, 1):
            name = stage["name"]
            state["active_stage"] = name
            save()
            started = time.monotonic()
            stage_driver = driver_identity()
            (root / "logs" / (name + "_test_driver_identity.json")).write_text(json.dumps(stage_driver, indent=2))
            event(dict(event="STAGE_START", index=index, total=len(selected), stage=name))
            with (root / "logs" / (name + ".log")).open("x", buffering=1) as log:
                if stage.get("authority") == "guest" and state["guest_service"] is None:
                    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dt%H%M%S")
                    state["guest_service"] = ensure_guest("camrod-v224-suite-guest-" + stamp, log,
                        root / "logs" / (name + "_guest_preparation.json"))
                    save()
                kind = "guest" if stage.get("authority") == "guest" else "operator"
                layout = arrange(kind)
                (root / "logs" / (name + "_actual_window_layout.json")).write_text(json.dumps(layout, ensure_ascii=False, indent=2))
                if stage.get("kind") == "dock":
                    dock(root / name, args, log)
                else:
                    subprocess.run(matrix_command(stage, root, "run", args), check=True, stdout=log, stderr=subprocess.STDOUT)
            completed = dict(stage=name, status="PASS", elapsed_wall_s=time.monotonic()-started, ended_utc=utc(),
                             test_driver_checkout_head=stage_driver["checkout_head"],
                             test_driver_identity_file=str(root / "logs" / (name + "_test_driver_identity.json")))
            state["completed"].append(completed)
            save()
            event(dict(event="STAGE_PASS", **completed))
        state.update(status="PASS", active_stage=None, ended_utc=utc())
    except BaseException as error:
        state.update(status="FAIL", failed_stage=state["active_stage"], error=str(error), ended_utc=utc())
        event(dict(event="SUITE_FAIL", stage=state["active_stage"], error=str(error)))
        raise
    finally:
        save()
        # Guest and audio services are independent; do not kill any external
        # service, reset a robot, discard evidence, or start a recovery here.
    event(dict(event="SUITE_PASS", completed=len(state["completed"]), output_root=str(root)))


if __name__ == "__main__":
    main()
