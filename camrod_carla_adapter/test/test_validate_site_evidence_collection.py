"""Offline contracts for the strict site-evidence collection validator."""
# flake8: noqa

from __future__ import annotations

import csv
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Callable

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "validate_site_evidence_collection.py"
)
SPEC = importlib.util.spec_from_file_location(
    "validate_site_evidence_collection", SCRIPT
)
assert SPEC and SPEC.loader
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
SPEC.loader.exec_module(validator)


def _write_json(path: Path, value: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return path


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {
        "path": str(path.resolve()),
        "bytes": path.stat().st_size,
        "sha256": _sha(path),
    }


def _write_sums(root: Path, names: list[str]) -> Path:
    path = root / "SHA256SUMS"
    path.write_text(
        "".join(f"{_sha(root / name)}  {name}\n" for name in sorted(names)),
        encoding="utf-8",
    )
    return path


def _authority(authority: str, mission: str) -> dict[str, str]:
    if authority == "guest":
        return {
            "matrix_return_authority": "guest_browser",
            "expected_return_source": "guest:usage_complete",
            "captured_ui_kind": "guest",
            "matrix_subcommand": "camping-sites-guest",
        }
    if authority == "operator-browser":
        return {
            "matrix_return_authority": "operator_browser",
            "expected_return_source": "ws:usage_complete:site_exit_first",
            "captured_ui_kind": "operator",
            "matrix_subcommand": (
                "camping-sites-browser-recall"
                if mission == "recall"
                else "camping-sites-browser"
            ),
        }
    return {
        "matrix_return_authority": "operator_rest",
        "expected_return_source": "",
        "captured_ui_kind": "operator",
        "matrix_subcommand": (
            "camping-sites-recall" if mission == "recall" else "camping-sites"
        ),
    }


def _git(root: Path, *arguments: str) -> str:
    return subprocess.run(
        ["git", *arguments],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _build_runtime_contract_fixture(tmp_path: Path) -> dict[str, Any]:
    source_root = tmp_path / "software" / "src"
    frontend = (
        source_root / "camrod_ui" / "camrod_ui_robot" / "assets" / "frontend"
    )
    for relative, content in (
        ("package.json", '{"name":"fixture"}\n'),
        ("package-lock.json", '{"lockfileVersion":3}\n'),
        ("src/App.js", "export default 1;\n"),
        ("public/index.html", "<html></html>\n"),
    ):
        path = frontend / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    _write_json(frontend / "camrod-build-env.json", validator.CANONICAL_UI_BUILD_ENV)
    inputs_sha = validator._frontend_source_input_fingerprint(frontend)
    source_build = frontend / "build"
    (source_build / "static" / "js").mkdir(parents=True)
    (source_build / "index.html").write_text(
        '<script src="/static/js/main.fixture.js"></script>\n', encoding="utf-8"
    )
    (source_build / "static" / "js" / "main.fixture.js").write_text(
        "fixture production bundle\n", encoding="utf-8"
    )
    _write_json(
        source_build / ".camrod-build-env.json", validator.CANONICAL_UI_BUILD_ENV
    )
    (source_build / ".camrod-inputs.sha256").write_text(
        inputs_sha + "\n", encoding="ascii"
    )
    script_root = source_root / "scripts" / "virtual_carla"
    script_root.mkdir(parents=True)
    for name in ("run_site_evidence_matrix.sh", "site_access.sh", "run.sh"):
        script = script_root / name
        script.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
        script.chmod(0o755)
    lanelet = source_root / "lanelet2_maps.osm"
    lanelet.write_text("<osm version=\"0.6\"/>\n", encoding="utf-8")

    _git(source_root, "init", "-b", validator.EXPECTED_SOURCE_BRANCH)
    _git(source_root, "config", "user.email", "fixture@example.invalid")
    _git(source_root, "config", "user.name", "Fixture")
    _git(source_root, "add", ".")
    _git(source_root, "commit", "-m", "fixture")
    _git(source_root, "update-ref", "refs/remotes/origin/develop", "HEAD")

    install_root = tmp_path / "software" / "install"
    installed_build = (
        install_root
        / "camrod_ui"
        / "share"
        / "camrod_ui"
        / "camrod_ui_robot"
        / "assets"
        / "frontend"
        / "build"
    )
    (installed_build / "static" / "js").mkdir(parents=True)
    for relative in (
        "index.html",
        ".camrod-build-env.json",
        ".camrod-inputs.sha256",
        "static/js/main.fixture.js",
    ):
        source = source_build / relative
        target = installed_build / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    for package in validator.RUNTIME_INSTALL_PREFIXES:
        if package == "camrod_ui":
            continue
        path = install_root / package / "share" / package / "fixture.txt"
        path.parent.mkdir(parents=True)
        path.write_text(package + "\n", encoding="utf-8")

    source_identity = validator._source_identity(source_root)
    install_identity = validator._install_identity(install_root)
    ui_identity = validator._ui_build_identity(source_root, install_root)
    binding = validator._canonical_tree_digest(
        [
            {
                "head_commit": source_identity["head_commit"],
                "install_sha256": install_identity["sha256"],
                "runtime_source_sha256": source_identity["runtime_tree"]["sha256"],
                "ui_build_environment_sha256": ui_identity[
                    "installed_build_environment"
                ]["sha256"],
                "ui_inputs_sha256": ui_identity["source_inputs_sha256"],
                "ui_main_bundle_sha256": ui_identity["installed_main_bundle"][
                    "sha256"
                ],
            }
        ]
    )
    software_identity = {
        "schema": validator.SOFTWARE_IDENTITY_SCHEMA,
        "binding_sha256": binding,
        "source": source_identity,
        "install": install_identity,
        "ui_build": ui_identity,
    }

    umap = (
        tmp_path
        / "CARLA"
        / "Unreal"
        / "CarlaUE4"
        / "Content"
        / f"{validator.CURRENT_CARLA_MAP}.umap"
    )
    umap.parent.mkdir(parents=True)
    umap.write_bytes(b"fixture-v15-umap\n")
    launch_argv = [
        "ros2",
        "launch",
        "camrod_carla_adapter",
        validator.CURRENT_RUNTIME_LAUNCH,
        f"camrod_map_path:={lanelet.resolve()}",
    ]
    cmdline = b"\0".join(token.encode() for token in launch_argv) + b"\0"
    return {
        "software_identity": software_identity,
        "source_manifest": {
            "git_root": str(source_root.resolve()),
            "branch": source_identity["branch"],
            "head": source_identity["head_commit"],
            "runner": _artifact(script_root / "run_site_evidence_matrix.sh"),
            "entrypoint": _artifact(script_root / "site_access.sh"),
        },
        "lanelet": lanelet.resolve(),
        "umap": umap.resolve(),
        "launch_argv": launch_argv,
        "launch_cmdline_sha256": hashlib.sha256(cmdline).hexdigest(),
    }


def _build_collection(
    tmp_path: Path,
    *,
    sites: tuple[str, ...] = ("B1", "B2"),
    authority: str = "operator",
    mission: str = "recall",
) -> Path:
    root = tmp_path / "runner-output"
    root.mkdir(parents=True)
    native_root = tmp_path / "native"
    contract = _authority(authority, mission)
    runtime_contract = _build_runtime_contract_fixture(tmp_path)
    site_manifests: list[dict[str, Any]] = []
    metric_rows: list[dict[str, Any]] = []

    for offset, site in enumerate(sites, 1):
        actor_id = 27
        elapsed = 30.0 + offset
        outbound_duration = 10.0
        return_duration = elapsed - outbound_duration
        outbound_distance = 5.0 + offset
        return_distance = 7.0 + offset
        total_distance = outbound_distance + return_distance
        site_dir = root / site
        site_dir.mkdir()
        native_dir = native_root / site
        native_dir.mkdir(parents=True)

        selected_live_parameters = copy.deepcopy(
            validator.CURRENT_RUNTIME_PROFILE_PARAMETERS
        )
        lanelet_path = str(runtime_contract["lanelet"])
        for node in validator.LANELET_PARAMETER_NODES:
            selected_live_parameters.setdefault(node, {})["map_path"] = lanelet_path
        selected_live_parameters["/carla_ros_bridge"] = {
            "town": validator.CURRENT_CARLA_MAP,
        }
        selected_live_parameters["/physical_four_wheel_bridge"] = {
            "host": "127.0.0.1",
            "port": 2000,
            "role_name": validator.EXPECTED_ROLE_NAME,
            "expected_blueprint_id": validator.EXPECTED_TYPE_ID,
            "extended_mode_backend": validator.EXPECTED_BACKEND,
            "use_sim_time": True,
        }
        runtime_audit = {
            "schema": validator.RUNTIME_AUDIT_SCHEMA,
            "status": "PASS",
            "accepted": True,
            "errors": [],
            "profile": validator.CURRENT_RUNTIME_PROFILE,
            "software_identity": copy.deepcopy(runtime_contract["software_identity"]),
            "launch": {
                "pid": 4242,
                "launch_file": validator.CURRENT_RUNTIME_LAUNCH,
                "profile": validator.CURRENT_RUNTIME_PROFILE,
                "lanelet_map_argument": lanelet_path,
                "cmdline_sha256": runtime_contract["launch_cmdline_sha256"],
                "argv": list(runtime_contract["launch_argv"]),
            },
            "carla": {
                "map_name": f"/Game/{validator.CURRENT_CARLA_MAP}",
                "normalized_map_name": validator.CURRENT_CARLA_MAP,
                "expected_ue_map": f"/Game/{validator.CURRENT_CARLA_MAP}",
                "expected_town": validator.CURRENT_CARLA_MAP,
                "ue_map_asset": {
                    "path": str(runtime_contract["umap"]),
                    "sha256": _sha(runtime_contract["umap"]),
                },
                "ego_actor": {
                    "actor_id": actor_id,
                    "type_id": validator.EXPECTED_TYPE_ID,
                    "role_name": validator.EXPECTED_ROLE_NAME,
                }
            },
            "lanelet_map": {
                "path": lanelet_path,
                "sha256": _sha(runtime_contract["lanelet"]),
                "launch_argument_match": True,
                "parameter_nodes": list(validator.LANELET_PARAMETER_NODES),
                "all_parameter_paths_match": True,
            },
            "selected_live_parameters": selected_live_parameters,
            "profile_contract": {
                "requested_profile": validator.CURRENT_RUNTIME_PROFILE,
                "site_geometry_allowed_carla_maps": [validator.CURRENT_CARLA_MAP],
            },
        }
        runtime_path = _write_json(native_dir / "runtime_profile_audit.json", runtime_audit)
        sensor_audit = {
            "schema_version": "camrod.virtual_carla.sensor_source_audit.v1",
            "status": "PASS",
            "summary": {
                "streams_checked": 36,
                "stream_failures": 0,
                "actors_checked": 13,
                "actor_failures": 0,
            },
        }
        sensor_path = _write_json(native_dir / "sensor_source_audit.json", sensor_audit)
        service_sequence = [
            "CHARGING",
            "DEPARTING_CHARGER",
            "RECALL_TO_SITE_ROAD",
            "GUEST_LOADING_WAIT",
            "RETURN_WITH_CARGO",
            "DROP_ZONE_PARKING",
            "WAITING_FOR_CHARGING",
            "CHARGING",
        ]
        native_site = {
            "site": site,
            "status": "PASS",
            "failure_reason": None,
            "mission_intent": mission,
            "service_mode": "roadside_stop" if mission == "recall" else "turnaround",
            "elapsed_s": elapsed,
            "outbound_duration_s": outbound_duration,
            "return_duration_s": return_duration,
            "outbound_distance_m": outbound_distance,
            "return_distance_m": return_distance,
            "total_odom_distance_m": total_distance,
            "drop_zone_error_m": 0.1,
            "parking_confirmed": True,
            "charging_confirmed": True,
            "actor_id": actor_id,
            "collision_evidence": {
                "topic": "/carla/ego_vehicle/collision",
                "subscriber_created": True,
                "publisher_count": 1,
                "publisher_discovered": True,
                "subscriber_and_publisher_discovered": True,
                "event_count": 0,
                "sampled_event_count": 0,
                "omitted_event_count": 0,
                "stale_queued_event_count": 0,
                "events": [],
            },
            "final_service_state": {"state": 13, "state_name": "CHARGING"},
            "final_gate_status": {"operating_state": "CHARGING"},
            "final_parking_status": {
                "apriltag": {"operating_state": "PARKED"}
            },
            "final_carla_odometry": {"speed_mps": 0.001},
            "final_physical_four_wheel_status": {
                "actor_id": actor_id,
                "ready": True,
                "api_version": validator.EXPECTED_API_VERSION,
                "motion_backend": validator.EXPECTED_BACKEND,
                "physical_gate_accepted": True,
                "independent_wheel_drive_available": True,
                "physical_manifest_sha256": "a" * 64,
            },
            "motion_metrics": {
                "motion_command_observed": True,
                "carla_odometry_samples": 100,
                "carla_odometry_distance_m": total_distance,
            },
            "service_state_sequence": service_sequence,
            "dispatch_response": {"success": True, "intent": mission},
            "return_response": {"success": True, "action": "site_exit_then_return"},
            "ui_operation_request_sequence": [],
        }
        if authority == "guest":
            native_site["return_response"] = {
                "success": True,
                "action": "usage_complete",
                "transport": "visible_guest_page_websocket_via_cdp",
            }
            native_site["ui_operation_request_sequence"] = [
                {"operation": 3, "source": "guest:usage_complete"}
            ]
        elif authority == "operator-browser":
            pointer = {
                "transport": "CDP.Input.dispatchMouseEvent",
                "selector": '[data-ui="operator-site-B1"]',
            }
            native_site["dispatch_response"] = {
                "accepted": True,
                "intent": mission,
                "source": (
                    "robot_ui:recall" if mission == "recall" else "ws"
                ),
                "transport": (
                    "visible_operator_page_http_via_cdp_input"
                    if mission == "recall"
                    else "visible_operator_page_websocket_via_cdp_input"
                ),
                "interactions": [dict(pointer) for _ in range(7)] + [{
                    "transport": "CDP.Input.insertText",
                    "selector": '[data-ui="operator-site-code-input"]',
                }],
            }
            native_site["dispatch_source_verified"] = True
            native_site["return_response"] = {
                "accepted": True,
                "action": "usage_complete",
                "source": "ws:usage_complete",
                "transport": "visible_operator_page_websocket_via_cdp_input",
            }
            native_site["controller_operation_request_sequence"] = [{
                "operation": 3,
                "source": "ws:usage_complete:site_exit_first",
            }]
        matrix = {
            "schema": validator.MATRIX_SCHEMA,
            "status": "PASS",
            "scope": {
                "selected_sites": [site],
                "mission_intent": mission,
                "motion_commands_sent": True,
                "pose_teleport_used": False,
                "fake_sensor_data_used": False,
                "return_authority": contract["matrix_return_authority"],
                "expected_return_source": contract["expected_return_source"],
            },
            "runtime_profile_audit": {
                "path": str(runtime_path.resolve()),
                "sha256": _sha(runtime_path),
                "status": "PASS",
            },
            "sensor_source_audit": {
                "path": str(sensor_path.resolve()),
                "sha256": _sha(sensor_path),
                "status": "PASS",
            },
            "sites": [native_site],
        }
        matrix_path = _write_json(native_dir / "camping_site_matrix.json", matrix)
        evidence_link = site_dir / "camping_site_matrix.json"
        evidence_link.symlink_to(matrix_path)

        raw_wheels = site_dir / "physical_wheels.jsonl"
        raw_wheels.write_text('{"fixture":true}\n', encoding="utf-8")
        sample_count = 100
        wheel_manifest = {
            "schema": validator.WHEEL_MANIFEST_SCHEMA,
            "status": "STOPPED",
            "stop_reason": "signal",
            "read_only": True,
            "sample_count": sample_count,
            "actor": {
                "id": actor_id,
                "type_id": validator.EXPECTED_TYPE_ID,
                "role_name": validator.EXPECTED_ROLE_NAME,
            },
            "output": _artifact(raw_wheels),
        }
        wheel_manifest_path = _write_json(
            site_dir / "physical_wheels.manifest.json", wheel_manifest
        )
        wheel_summary_root = site_dir / "wheel_summary"
        wheel_summary_root.mkdir()
        wheels = {
            name: {
                "sample_count": sample_count,
                "in_air_count": 0,
                "in_air_ratio": 0,
            }
            for name in validator.CANONICAL_WHEELS
        }
        wheel_summary = {
            "schema": validator.WHEEL_SUMMARY_SCHEMA,
            "status": "PASS",
            "source": {
                **_artifact(raw_wheels),
                "schema": "camrod.virtual_carla.physical_wheel_telemetry.v1",
                "line_count": sample_count + 2,
            },
            "samples": {
                "count": sample_count,
                "recording_duration_seconds": elapsed + 1.0,
            },
            "actor": wheel_manifest["actor"],
            "motion": {"planar_distance_m": total_distance - 0.01},
            "wheels": wheels,
            "validation": {
                "actor_identity_consistent": True,
                "canonical_wheels_exact": list(validator.CANONICAL_WHEELS),
                "footer_sample_count_matches": True,
                "footer_status": "STOPPED",
                "footer_valid": True,
                "header_valid": True,
                "streaming_reader": True,
            },
        }
        wheel_summary_path = _write_json(
            wheel_summary_root / "wheel_summary.json", wheel_summary
        )
        wheel_csv_path = wheel_summary_root / "wheel_measurements.csv"
        with wheel_csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=(
                    "wheel",
                    "sample_count",
                    "in_air_count",
                    "in_air_ratio",
                    "source_sha256",
                ),
                lineterminator="\n",
            )
            writer.writeheader()
            for wheel in validator.CANONICAL_WHEELS:
                writer.writerow(
                    {
                        "wheel": wheel,
                        "sample_count": sample_count,
                        "in_air_count": 0,
                        "in_air_ratio": 0.0,
                        "source_sha256": _sha(raw_wheels),
                    }
                )
        wheel_png = wheel_summary_root / "wheel_summary.png"
        wheel_png.write_bytes(b"\x89PNG\r\n\x1a\nfixture-wheel")
        wheel_sums = _write_sums(
            wheel_summary_root,
            ["wheel_summary.json", "wheel_measurements.csv", "wheel_summary.png"],
        )
        wheel_artifacts = {
            path.name: _artifact(path)
            for path in (wheel_summary_path, wheel_csv_path, wheel_png, wheel_sums)
        }

        visual_root = site_dir / "visual"
        visual_root.mkdir()
        png_path = visual_root / "representative_contact_sheet.png"
        png_path.write_bytes(b"\x89PNG\r\n\x1a\nfixture-contact-sheet")
        gif_path = visual_root / "representative_motion.gif"
        gif_path.write_bytes(b"GIF89afixture-motion")
        ui_key = "camrod_guest_ui" if authority == "guest" else "camrod_operator_ui"
        capture_manifest = {
            "schema": validator.CAPTURE_SCHEMA,
            "status": "PASS",
            "scope": {
                "vehicle_motion_or_ui_input_sent_by_capture": False,
                "ai_generated_or_enhanced": False,
            },
            "recording": {
                "source_video": {
                    "path": "carla_camrod_desktop.mp4",
                    "retained": False,
                    "removed_after_derivation": True,
                }
            },
            "x11": {
                "geometry_validated_side_by_side": True,
                "windows": {
                    "carla": {"title": "CarlaUE4"},
                    ui_key: {
                        "kind": contract["captured_ui_kind"],
                        "title": "UI",
                    },
                },
            },
            "artifacts": {
                "contact_sheet_png": {
                    "path": png_path.name,
                    "bytes": png_path.stat().st_size,
                    "sha256": _sha(png_path),
                },
                "representative_gif": {
                    "path": gif_path.name,
                    "bytes": gif_path.stat().st_size,
                    "sha256": _sha(gif_path),
                },
            },
        }
        capture_manifest_path = _write_json(
            visual_root / "capture_manifest.json", capture_manifest
        )

        matrix_record = {
            **_artifact(matrix_path),
            "status": "PASS",
            "site_status": "PASS",
            "evidence_link": str(evidence_link),
        }
        motion_metrics = {
            "elapsed_s": elapsed,
            "outbound_duration_s": outbound_duration,
            "return_duration_s": return_duration,
            "outbound_distance_m": outbound_distance,
            "return_distance_m": return_distance,
            "total_odom_distance_m": total_distance,
        }
        site_manifest = {
            "schema": validator.SITE_SCHEMA,
            "status": "PASS",
            "site": site,
            "stop_on_failure": True,
            "authority": {
                "frontend": authority,
                "mission_intent": mission,
                "matrix_return_authority": contract["matrix_return_authority"],
                "expected_return_source": contract["expected_return_source"],
                "captured_ui_kind": contract["captured_ui_kind"],
            },
            "retain_source_video": False,
            "matrix_command": {
                "argv": ["site_access.sh", contract["matrix_subcommand"]],
                "environment": {
                    "CAMROD_CARLA_CAMPING_SITES": site,
                    "CAMROD_CARLA_MATRIX_MISSION_INTENT": mission,
                },
                "exit_code": 0,
                "tee_exit_code": 0,
                "only_motion_authority": True,
            },
            "matrix_report": matrix_record,
            "motion_metrics": motion_metrics,
            "physical_wheels": {
                "jsonl": _artifact(raw_wheels),
                "manifest": _artifact(wheel_manifest_path),
                "status": "STOPPED",
                "sample_count": sample_count,
                "summary": {
                    "exit_code": 0,
                    "tee_exit_code": 0,
                    "status": "PASS",
                    "artifacts": wheel_artifacts,
                },
            },
            "visual": {
                "manifest": _artifact(capture_manifest_path),
                "source_mp4_retained": False,
                "capture_finalized_with_q": True,
                "png": _artifact(png_path),
                "gif": _artifact(gif_path),
            },
            "failure_reasons": [],
        }
        _write_json(site_dir / "site_manifest.json", site_manifest)
        site_manifests.append(site_manifest)
        metric_rows.append(
            {
                "site": site,
                "status": "PASS",
                "service_mode": native_site["service_mode"],
                "total_elapsed_s": elapsed,
                "outbound_duration_s": outbound_duration,
                "return_duration_s": return_duration,
                "total_odom_distance_m": total_distance,
                "outbound_distance_m": outbound_distance,
                "return_distance_m": return_distance,
                "drop_zone_error_m": 0.1,
                "final_service_state": "CHARGING",
                "parking_confirmed": True,
                "charging_confirmed": True,
                "actor_id": actor_id,
                "source_report": str(matrix_path),
                "source_report_sha256": _sha(matrix_path),
            }
        )

    summary_root = root / "summary"
    summary_root.mkdir()
    expected_set = set(sites)
    metrics_document = {
        "schema": validator.METRICS_SCHEMA,
        "coverage": (
            "B1_B13_COMPLETE"
            if expected_set == set(validator.ALL_SITES)
            else "PARTIAL"
        ),
        "aggregate": {
            "site_count": len(sites),
            "pass_count": len(sites),
            "fail_count": 0,
            "missing_sites": [
                site for site in validator.ALL_SITES if site not in expected_set
            ],
            "total_elapsed_s": round(
                sum(row["total_elapsed_s"] for row in metric_rows), 6
            ),
            "outbound_duration_s": round(
                sum(row["outbound_duration_s"] for row in metric_rows), 6
            ),
            "return_duration_s": round(
                sum(row["return_duration_s"] for row in metric_rows), 6
            ),
            "total_odom_distance_m": round(
                sum(row["total_odom_distance_m"] for row in metric_rows), 6
            ),
            "outbound_distance_m": round(
                sum(row["outbound_distance_m"] for row in metric_rows), 6
            ),
            "return_distance_m": round(
                sum(row["return_distance_m"] for row in metric_rows), 6
            ),
        },
        "sites": metric_rows,
    }
    metrics_json = _write_json(
        summary_root / "camping_site_metrics.json", metrics_document
    )
    metrics_csv = summary_root / "camping_site_metrics.csv"
    with metrics_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("site", "status"), lineterminator="\n")
        writer.writeheader()
        for site in sites:
            writer.writerow({"site": site, "status": "PASS"})
    metrics_md = summary_root / "camping_site_metrics.md"
    metrics_md.write_text(
        "# Metrics\n\n" + "".join(f"| {site} | PASS |\n" for site in sites),
        encoding="utf-8",
    )
    metrics_sums = _write_sums(
        summary_root,
        ["camping_site_metrics.json", "camping_site_metrics.csv", "camping_site_metrics.md"],
    )
    summary_artifacts = {
        path.name: _artifact(path)
        for path in (metrics_json, metrics_csv, metrics_md, metrics_sums)
    }
    run_manifest = {
        "schema": validator.RUN_SCHEMA,
        "status": "PASS",
        "selected_sites": list(sites),
        "completed_site_manifests": len(sites),
        "stop_on_first_failure": True,
        "failure_site": None,
        "failure_reason": None,
        "exit_code": 0,
        "source": copy.deepcopy(runtime_contract["source_manifest"]),
        "motion_authority": {
            "frontend": authority,
            "mission_intent": mission,
            "matrix_return_authority": contract["matrix_return_authority"],
            "expected_return_source": contract["expected_return_source"],
            "captured_ui_kind": contract["captured_ui_kind"],
            "only_command": f"site_access.sh {contract['matrix_subcommand']}",
            "sites_run_independently": True,
            "observers_publish_or_control_vehicle": False,
        },
        "metrics_summary": {
            "schema": validator.METRICS_SCHEMA,
            "coverage": metrics_document["coverage"],
            "artifacts": summary_artifacts,
        },
        "physical_wheel_summaries": {
            item["site"]: item["physical_wheels"]["summary"]
            for item in site_manifests
        },
        "sites": site_manifests,
    }
    _write_json(root / "run_manifest.json", run_manifest)
    return root


def _refresh_site_in_run(root: Path, site: str) -> None:
    manifest = json.loads((root / site / "site_manifest.json").read_text())
    run_path = root / "run_manifest.json"
    run = json.loads(run_path.read_text())
    run["sites"] = [manifest if item["site"] == site else item for item in run["sites"]]
    run["physical_wheel_summaries"][site] = manifest["physical_wheels"]["summary"]
    _write_json(run_path, run)


def _mutate_native_matrix(
    root: Path, site: str, mutation: Callable[[dict[str, Any]], None]
) -> None:
    site_path = root / site / "site_manifest.json"
    manifest = json.loads(site_path.read_text())
    matrix_path = Path(manifest["matrix_report"]["path"])
    matrix = json.loads(matrix_path.read_text())
    mutation(matrix)
    _write_json(matrix_path, matrix)
    manifest["matrix_report"].update(_artifact(matrix_path))
    _write_json(site_path, manifest)
    _refresh_site_in_run(root, site)


def _mutate_runtime_audit(
    root: Path, site: str, mutation: Callable[[dict[str, Any]], None]
) -> None:
    site_path = root / site / "site_manifest.json"
    manifest = json.loads(site_path.read_text())
    matrix_path = Path(manifest["matrix_report"]["path"])
    matrix = json.loads(matrix_path.read_text())
    runtime_path = Path(matrix["runtime_profile_audit"]["path"])
    runtime = json.loads(runtime_path.read_text())
    mutation(runtime)
    _write_json(runtime_path, runtime)
    matrix["runtime_profile_audit"]["sha256"] = _sha(runtime_path)
    _write_json(matrix_path, matrix)
    manifest["matrix_report"].update(_artifact(matrix_path))
    _write_json(site_path, manifest)
    _refresh_site_in_run(root, site)


def _mutate_wheel_summary(
    root: Path, site: str, mutation: Callable[[dict[str, Any]], None]
) -> None:
    site_path = root / site / "site_manifest.json"
    manifest = json.loads(site_path.read_text())
    summary_root = root / site / "wheel_summary"
    summary_path = summary_root / "wheel_summary.json"
    summary = json.loads(summary_path.read_text())
    mutation(summary)
    _write_json(summary_path, summary)
    _write_sums(
        summary_root,
        ["wheel_summary.json", "wheel_measurements.csv", "wheel_summary.png"],
    )
    manifest["physical_wheels"]["summary"]["artifacts"] = {
        name: _artifact(summary_root / name)
        for name in (
            "wheel_summary.json",
            "wheel_measurements.csv",
            "wheel_summary.png",
            "SHA256SUMS",
        )
    }
    _write_json(site_path, manifest)
    _refresh_site_in_run(root, site)


def _snapshot(root: Path) -> dict[str, tuple[str, int, str]]:
    result: dict[str, tuple[str, int, str]] = {}
    for path in sorted(root.rglob("*")):
        relative = str(path.relative_to(root))
        if path.is_symlink():
            result[relative] = ("link", path.lstat().st_mode, os.readlink(path))
        elif path.is_file():
            result[relative] = ("file", path.stat().st_mode, _sha(path))
        else:
            result[relative] = ("dir", path.stat().st_mode, "")
    return result


def _validate(root: Path, sites: tuple[str, ...] = ("B1", "B2"), **kwargs):
    return validator.validate_collection(
        root,
        expected_sites=sites,
        authority=kwargs.get("authority", "operator"),
        mission_intent=kwargs.get("mission", "recall"),
    )


def test_valid_collection_is_read_only_and_writes_atomic_bound_aggregates(tmp_path):
    root = _build_collection(tmp_path)
    before = _snapshot(root)

    document = _validate(root)

    assert document["status"] == "PASS"
    assert [row["site"] for row in document["sites"]] == ["B1", "B2"]
    assert document["aggregate"]["total_collision_events"] == 0
    assert document["aggregate"]["all_four_wheels_grounded_for_every_sample"]
    assert before == _snapshot(root)

    output = tmp_path / "validated"
    hashes = validator.write_outputs(output.resolve(), document)
    assert set(path.name for path in output.iterdir()) == {
        *validator.OUTPUT_FILES,
        "SHA256SUMS",
    }
    decoded = json.loads((output / "site_evidence_collection.json").read_text())
    assert decoded["aggregate"]["site_count"] == 2
    with (output / "site_evidence_collection.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["site"] for row in rows] == ["B1", "B2"]
    assert "| B2 | PASS |" in (output / "site_evidence_collection.md").read_text()
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ", 1)
        assert _sha(output / name) == digest == hashes[name]
    assert before == _snapshot(root)


def test_existing_empty_absolute_output_is_supported_but_nonempty_is_rejected(tmp_path):
    root = _build_collection(tmp_path)
    document = _validate(root)
    output = tmp_path / "empty"
    output.mkdir()
    validator.write_outputs(output.resolve(), document)
    assert (output / "site_evidence_collection.json").is_file()
    with pytest.raises(validator.CollectionValidationError, match="new or empty"):
        validator.write_outputs(output.resolve(), document)
    with pytest.raises(validator.CollectionValidationError, match="outside"):
        validator.write_outputs((root / "must-not-write").resolve(), document)


def test_site_expectations_are_exact_unique_ordered_and_authority_bound(tmp_path):
    root = _build_collection(tmp_path)
    with pytest.raises(validator.CollectionValidationError, match="duplicate expected"):
        validator.parse_sites("B1,B1")
    with pytest.raises(validator.CollectionValidationError, match="uppercase B1..B13"):
        validator.parse_sites("b1")
    with pytest.raises(validator.CollectionValidationError, match="selected_sites"):
        _validate(root, sites=("B1",))
    with pytest.raises(validator.CollectionValidationError, match="frontend"):
        _validate(root, authority="guest")


def test_current_collection_rejects_noncanonical_recorded_entrypoint(tmp_path):
    root = _build_collection(tmp_path, sites=("B1",))
    run_path = root / "run_manifest.json"
    run = json.loads(run_path.read_text())
    source_root = Path(run["source"]["git_root"])
    run["source"]["entrypoint"] = _artifact(
        source_root / "scripts" / "virtual_carla" / "run.sh"
    )
    _write_json(run_path, run)
    with pytest.raises(
        validator.CollectionValidationError,
        match="run_manifest.source.entrypoint.path resolves",
    ):
        _validate(root, sites=("B1",))


@pytest.mark.parametrize(
    "authority,mission",
    (
        ("operator", "delivery"),
        ("operator", "recall"),
        ("operator-browser", "delivery"),
        ("operator-browser", "recall"),
        ("guest", "recall"),
    ),
)
def test_every_supported_frontend_authority_and_mission_contract_passes(
    tmp_path, authority, mission
):
    root = _build_collection(
        tmp_path / f"{authority}-{mission}",
        sites=("B1",),
        authority=authority,
        mission=mission,
    )
    document = _validate(
        root,
        sites=("B1",),
        authority=authority,
        mission=mission,
    )
    assert document["expectation"]["authority"] == authority
    assert document["expectation"]["mission_intent"] == mission


@pytest.mark.parametrize(
    "mutation,pattern",
    [
        (
            lambda matrix: matrix["sites"][0]["collision_evidence"].update(
                event_count=1
            ),
            "event_count must be 0",
        ),
        (
            lambda matrix: matrix["sites"][0]["collision_evidence"].update(
                publisher_count=0
            ),
            "publisher_count must be >= 1",
        ),
        (
            lambda matrix: matrix["sites"][0].update(parking_confirmed=False),
            "parking_confirmed",
        ),
        (
            lambda matrix: matrix["sites"][0].update(charging_confirmed=False),
            "charging_confirmed",
        ),
        (
            lambda matrix: matrix["sites"][0]["final_carla_odometry"].update(
                speed_mps=0.051
            ),
            "exceeds 0.05",
        ),
        (
            lambda matrix: matrix["sites"][0][
                "final_physical_four_wheel_status"
            ].update(motion_backend="KINEMATIC"),
            "physical.backend",
        ),
        (
            lambda matrix: matrix["sites"][0].update(return_duration_s=1.0),
            "duration split is inconsistent",
        ),
    ],
)
def test_native_pass_is_rejected_when_runtime_acceptance_fact_is_false(
    tmp_path, mutation, pattern
):
    root = _build_collection(tmp_path, sites=("B1",))
    _mutate_native_matrix(root, "B1", mutation)
    with pytest.raises(validator.CollectionValidationError, match=pattern):
        _validate(root, sites=("B1",))


def test_runtime_actor_type_is_bound_not_inferred_from_pass_status(tmp_path):
    root = _build_collection(tmp_path, sites=("B1",))
    manifest = json.loads((root / "B1" / "site_manifest.json").read_text())
    matrix = json.loads(Path(manifest["matrix_report"]["path"]).read_text())
    runtime_path = Path(matrix["runtime_profile_audit"]["path"])
    runtime = json.loads(runtime_path.read_text())
    runtime["carla"]["ego_actor"]["type_id"] = "vehicle.tesla.model3"
    _write_json(runtime_path, runtime)
    matrix["runtime_profile_audit"]["sha256"] = _sha(runtime_path)
    matrix_path = Path(manifest["matrix_report"]["path"])
    _write_json(matrix_path, matrix)
    manifest["matrix_report"].update(_artifact(matrix_path))
    _write_json(root / "B1" / "site_manifest.json", manifest)
    _refresh_site_in_run(root, "B1")
    with pytest.raises(validator.CollectionValidationError, match="runtime.type_id"):
        _validate(root, sites=("B1",))


@pytest.mark.parametrize(
    "mutation,pattern",
    [
        (
            lambda audit: audit.update(
                profile="develop-plus-carla-site-geometry-v26"
            ),
            "runtime current profile",
        ),
        (
            lambda audit: audit["selected_live_parameters"]["/ui_backend"].update(
                return_site_exit_rearm_enabled=False
            ),
            "runtime authoritative signature./ui_backend.return_site_exit_rearm_enabled",
        ),
        (
            lambda audit: audit["selected_live_parameters"].pop(
                "/perception/yolov9mit"
            ),
            "runtime authoritative signature keys mismatch",
        ),
    ],
)
def test_current_collection_rejects_legacy_or_mutated_runtime_profile(
    tmp_path, mutation, pattern
):
    root = _build_collection(tmp_path, sites=("B1",))
    _mutate_runtime_audit(root, "B1", mutation)
    with pytest.raises(validator.CollectionValidationError, match=pattern):
        _validate(root, sites=("B1",))


@pytest.mark.parametrize(
    "mutation,pattern",
    [
        (
            lambda audit: audit["software_identity"].update(
                binding_sha256="0" * 64
            ),
            "software_identity.binding_sha256 mismatch",
        ),
        (
            lambda audit: audit["software_identity"]["source"].update(
                runtime_worktree_clean=False
            ),
            "software_identity.source.runtime_worktree_clean mismatch",
        ),
        (
            lambda audit: audit["software_identity"]["install"].update(
                sha256="0" * 64
            ),
            "software_identity.install.sha256 mismatch",
        ),
        (
            lambda audit: audit["software_identity"]["ui_build"][
                "installed_main_bundle"
            ].update(sha256="0" * 64),
            "software_identity.ui_build.installed_main_bundle.sha256 mismatch",
        ),
    ],
)
def test_current_collection_rejects_tampered_software_identity(
    tmp_path, mutation, pattern
):
    root = _build_collection(tmp_path, sites=("B1",))
    _mutate_runtime_audit(root, "B1", mutation)
    with pytest.raises(validator.CollectionValidationError, match=pattern):
        _validate(root, sites=("B1",))


def test_current_collection_rejects_installed_bundle_changed_after_audit(tmp_path):
    root = _build_collection(tmp_path, sites=("B1",))
    site = json.loads((root / "B1" / "site_manifest.json").read_text())
    matrix = json.loads(Path(site["matrix_report"]["path"]).read_text())
    runtime = json.loads(Path(matrix["runtime_profile_audit"]["path"]).read_text())
    bundle = Path(
        runtime["software_identity"]["ui_build"]["installed_main_bundle"]["path"]
    )
    bundle.write_bytes(bundle.read_bytes() + b"tamper")
    with pytest.raises(
        validator.CollectionValidationError,
        match="software_identity.install.sha256 mismatch",
    ):
        _validate(root, sites=("B1",))


@pytest.mark.parametrize(
    "mutation,pattern",
    [
        (
            lambda audit: audit["carla"].update(
                expected_town="map_package/Maps/Woraksan_old/Woraksan_old"
            ),
            "carla.expected_town mismatch",
        ),
        (
            lambda audit: audit["carla"]["ue_map_asset"].update(
                sha256="0" * 64
            ),
            "ue_map_asset.sha256 mismatch",
        ),
        (
            lambda audit: audit["selected_live_parameters"][
                "/carla_ros_bridge"
            ].update(town="map_package/Maps/Woraksan_old/Woraksan_old"),
            "parameters./carla_ros_bridge.town mismatch",
        ),
        (
            lambda audit: audit["lanelet_map"].update(sha256="0" * 64),
            "lanelet_map.sha256 mismatch",
        ),
        (
            lambda audit: audit["launch"].update(
                launch_file="camrod_carla_full.launch.py"
            ),
            "launch.file mismatch",
        ),
        (
            lambda audit: audit["launch"].update(cmdline_sha256="0" * 64),
            "launch.cmdline_sha256 mismatch",
        ),
        (
            lambda audit: audit["selected_live_parameters"][
                "/map/lanelet_map_provider"
            ].update(map_path="/tmp/not-the-audited-map.osm"),
            "parameters./map/lanelet_map_provider.map_path",
        ),
    ],
)
def test_current_collection_rejects_tampered_v15_map_lanelet_or_launch_identity(
    tmp_path, mutation, pattern
):
    root = _build_collection(tmp_path, sites=("B1",))
    _mutate_runtime_audit(root, "B1", mutation)
    with pytest.raises(validator.CollectionValidationError, match=pattern):
        _validate(root, sites=("B1",))


def test_current_collection_rejects_umap_file_changed_after_runtime_audit(tmp_path):
    root = _build_collection(tmp_path, sites=("B1",))
    site = json.loads((root / "B1" / "site_manifest.json").read_text())
    matrix = json.loads(Path(site["matrix_report"]["path"]).read_text())
    runtime = json.loads(Path(matrix["runtime_profile_audit"]["path"]).read_text())
    umap_path = Path(runtime["carla"]["ue_map_asset"]["path"])
    umap_path.write_bytes(umap_path.read_bytes() + b"tamper")
    with pytest.raises(validator.CollectionValidationError, match="ue_map_asset.sha256 mismatch"):
        _validate(root, sites=("B1",))


@pytest.mark.parametrize(
    "mutation,pattern",
    [
        (
            lambda summary: summary["wheels"]["RR"].update(in_air_count=1),
            "RR.in_air_count must be 0",
        ),
        (
            lambda summary: summary["wheels"].pop("RL"),
            "exactly FL, FR, RL, RR",
        ),
        (
            lambda summary: summary.update(status="FAIL"),
            "summary PASS",
        ),
    ],
)
def test_wheel_summary_requires_pass_four_canonical_never_airborne(
    tmp_path, mutation, pattern
):
    root = _build_collection(tmp_path, sites=("B1",))
    _mutate_wheel_summary(root, "B1", mutation)
    with pytest.raises(validator.CollectionValidationError, match=pattern):
        _validate(root, sites=("B1",))


def test_visual_hash_and_file_signature_are_both_required(tmp_path):
    root = _build_collection(tmp_path, sites=("B1",))
    png = root / "B1" / "visual" / "representative_contact_sheet.png"
    png.write_bytes(b"not-a-png-but-rehashed")
    capture_path = root / "B1" / "visual" / "capture_manifest.json"
    capture = json.loads(capture_path.read_text())
    capture["artifacts"]["contact_sheet_png"].update(
        bytes=png.stat().st_size, sha256=_sha(png)
    )
    _write_json(capture_path, capture)
    site_path = root / "B1" / "site_manifest.json"
    site = json.loads(site_path.read_text())
    site["visual"]["png"] = _artifact(png)
    site["visual"]["manifest"] = _artifact(capture_path)
    _write_json(site_path, site)
    _refresh_site_in_run(root, "B1")
    with pytest.raises(validator.CollectionValidationError, match="wrong file signature"):
        _validate(root, sites=("B1",))

    root = _build_collection(tmp_path / "hash-case", sites=("B1",))
    gif = root / "B1" / "visual" / "representative_motion.gif"
    gif.write_bytes(gif.read_bytes() + b"tamper")
    with pytest.raises(validator.CollectionValidationError, match="byte count mismatch"):
        _validate(root, sites=("B1",))


def test_duplicate_json_keys_and_checksum_entries_fail_closed(tmp_path):
    root = _build_collection(tmp_path, sites=("B1",))
    run_path = root / "run_manifest.json"
    original = run_path.read_text()
    run_path.write_text(
        original.replace(
            '"schema": "camrod.virtual_carla.site_evidence_matrix.v1",',
            '"schema": "camrod.virtual_carla.site_evidence_matrix.v1",\n'
            '  "schema": "camrod.virtual_carla.site_evidence_matrix.v1",',
            1,
        ),
        encoding="utf-8",
    )
    with pytest.raises(validator.CollectionValidationError, match="duplicate JSON key"):
        _validate(root, sites=("B1",))

    root = _build_collection(tmp_path / "checksum-case", sites=("B1",))
    sums = root / "B1" / "wheel_summary" / "SHA256SUMS"
    sums.write_text(sums.read_text() + sums.read_text().splitlines()[0] + "\n")
    site_path = root / "B1" / "site_manifest.json"
    site = json.loads(site_path.read_text())
    site["physical_wheels"]["summary"]["artifacts"]["SHA256SUMS"] = _artifact(sums)
    _write_json(site_path, site)
    _refresh_site_in_run(root, "B1")
    with pytest.raises(validator.CollectionValidationError, match="duplicates"):
        _validate(root, sites=("B1",))


def test_validation_failure_creates_no_output_and_cli_success_is_explicit(
    tmp_path, capsys
):
    root = _build_collection(tmp_path, sites=("B1",))
    _mutate_native_matrix(
        root,
        "B1",
        lambda matrix: matrix["sites"][0]["collision_evidence"].update(
            event_count=2
        ),
    )
    output = tmp_path / "must-not-exist"
    result = validator.main(
        [
            "--input-root",
            str(root.resolve()),
            "--output-dir",
            str(output.resolve()),
            "--sites",
            "B1",
            "--authority",
            "operator",
            "--mission-intent",
            "recall",
        ]
    )
    assert result == 2
    assert not output.exists()
    assert "event_count must be 0" in capsys.readouterr().err

    root = _build_collection(tmp_path / "success", sites=("B1",))
    output = tmp_path / "success-output"
    result = validator.main(
        [
            "--input-root",
            str(root.resolve()),
            "--output-dir",
            str(output.resolve()),
            "--sites",
            "B1",
            "--authority",
            "operator",
            "--mission-intent",
            "recall",
        ]
    )
    assert result == 0
    assert "PASS sites=1" in capsys.readouterr().out
    assert (output / "SHA256SUMS").is_file()
