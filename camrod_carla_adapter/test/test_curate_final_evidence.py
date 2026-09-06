"""Offline contracts for deterministic final-evidence curation."""
# flake8: noqa

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / "scripts"
    / "virtual_carla"
    / "curate_final_evidence.py"
)
SPEC = importlib.util.spec_from_file_location("curate_final_evidence", SCRIPT)
assert SPEC and SPEC.loader
curator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = curator
SPEC.loader.exec_module(curator)


CONFIG_HASHES = {"camping_sites": "a" * 64, "drop_zones": "b" * 64}
GATE_HASH = "c" * 64
PHYSICAL_HASH = "d" * 64


def _write(path: Path, value: bytes | str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(value, str):
        path.write_text(value, encoding="utf-8")
    else:
        path.write_bytes(value)
    return path


def _write_json(path: Path, value) -> Path:
    return _write(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _artifact(path: Path) -> dict[str, object]:
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": _sha(path)}


def _authority(bundle: str) -> tuple[str, str, str]:
    contract = curator.BUNDLE_CONTRACTS[bundle]
    return contract["frontend"], contract["mission"], contract["return_authority"]


def _make_site(root: Path, bundle: str, site: str, *, symlink_matrix=False) -> Path:
    site_root = root / site
    frontend, mission, return_authority = _authority(bundle)
    number = int(site[1:])
    site_row = {
        "site": site,
        "status": "PASS",
        "service_mode": "roadside_stop" if number >= 11 else "turnaround",
        "elapsed_s": float(100 + number),
        "outbound_duration_s": 40.0,
        "return_duration_s": float(60 + number),
        "outbound_distance_m": 10.0,
        "return_distance_m": float(20 + number),
        "motion_metrics": {"carla_odometry_distance_m": float(30 + number)},
        "actor_id": 50,
        "drop_zone_error_m": 0.1,
        "final_service_state": {"state": 13, "state_name": "CHARGING"},
        "parking_confirmed": True,
        "charging_confirmed": True,
        "physical_manifest_sha256": PHYSICAL_HASH,
        "final_physical_four_wheel_status": {
            "actor_id": 50,
            "physical_manifest_sha256": PHYSICAL_HASH,
        },
    }
    matrix = {
        "schema": curator.MATRIX_SCHEMA,
        "status": "PASS",
        "active_configuration": {
            "sha256": CONFIG_HASHES,
            "baseline_gate_manifest_sha256": GATE_HASH,
        },
        "scope": {
            "selected_sites": [site],
            "mission_intent": mission,
            "return_authority": return_authority,
        },
        "sites": [site_row],
    }
    if symlink_matrix:
        native = root / "native" / f"{site}.json"
        _write_json(native, matrix)
        site_root.mkdir(parents=True, exist_ok=True)
        (site_root / "camping_site_matrix.json").symlink_to(native)
    else:
        _write_json(site_root / "camping_site_matrix.json", matrix)

    capture = _write_json(
        site_root / "visual/capture_manifest.json",
        {"schema": "capture.fixture", "status": "PASS"},
    )
    png = _write(site_root / "visual/representative_contact_sheet.png", b"PNG" + site.encode())
    gif = _write(site_root / "visual/representative_motion.gif", b"GIF89a" + site.encode())
    physical = _write_json(
        site_root / "physical_wheels.manifest.json",
        {"schema": "physical.fixture", "status": "STOPPED"},
    )
    wheel_json = _write_json(
        site_root / "wheel_summary/wheel_summary.json",
        {"schema": "wheel.fixture", "status": "PASS"},
    )
    wheel_csv = _write(site_root / "wheel_summary/wheel_measurements.csv", "wheel,value\nFL,1\n")
    wheel_png = _write(site_root / "wheel_summary/wheel_summary.png", b"PNG-WHEEL" + site.encode())
    matrix_path = site_root / "camping_site_matrix.json"
    manifest = {
        "schema": curator.SITE_MANIFEST_SCHEMA,
        "status": "PASS",
        "site": site,
        "authority": {
            "frontend": frontend,
            "mission_intent": mission,
            "matrix_return_authority": return_authority,
        },
        "matrix_report": _artifact(matrix_path),
        "visual": {
            "manifest": _artifact(capture),
            "png": _artifact(png),
            "gif": _artifact(gif),
        },
        "physical_wheels": {
            "manifest": _artifact(physical),
            "summary": {
                "status": "PASS",
                "artifacts": {
                    "wheel_summary.json": _artifact(wheel_json),
                    "wheel_measurements.csv": _artifact(wheel_csv),
                    "wheel_summary.png": _artifact(wheel_png),
                },
            },
        },
    }
    return _write_json(site_root / "site_manifest.json", manifest)


def _make_site_source(
    parent: Path, bundle: str, name: str, sites: list[str], *, symlink_first=False
) -> Path:
    root = parent / name
    _write_json(
        root / "run_manifest.json",
        {"schema": "runner.fixture", "status": "PASS", "selected_sites": sites},
    )
    for index, site in enumerate(sites):
        _make_site(root, bundle, site, symlink_matrix=symlink_first and index == 0)
    return root


def _write_sums(root: Path, names: list[str]) -> None:
    _write(
        root / "SHA256SUMS",
        "".join(f"{_sha(root / name)}  {name}\n" for name in sorted(names)),
    )


def _make_strict(parent: Path, bundle: str, name: str, sites: list[str], source_roots) -> Path:
    root = parent / name
    evidence = {}
    for source in source_roots:
        for site in sites:
            if (source / site).is_dir():
                evidence[site] = source / site
    rows = []
    for site in sites:
        site_root = evidence[site]
        rows.append(
            {
                "site": site,
                "status": "PASS",
                "matrix_sha256": _sha(site_root / "camping_site_matrix.json"),
                "site_manifest_sha256": _sha(site_root / "site_manifest.json"),
                "wheel_summary_sha256": _sha(site_root / "wheel_summary/wheel_summary.json"),
                "png_sha256": _sha(site_root / "visual/representative_contact_sheet.png"),
                "gif_sha256": _sha(site_root / "visual/representative_motion.gif"),
            }
        )
    contract = curator.BUNDLE_CONTRACTS[bundle]
    _write_json(
        root / "site_evidence_collection.json",
        {
            "schema": curator.STRICT_SCHEMA,
            "status": "PASS",
            "expectation": {
                "authority": contract["strict_authority"],
                "mission_intent": contract["mission"],
                "sites": sites,
            },
            "validation": {
                "runner_status_pass": True,
                "runner_prepass_validating": False,
                "fixture_check": True,
            },
            "sites": rows,
        },
    )
    _write(root / "site_evidence_collection.csv", "site,status\n")
    _write(root / "site_evidence_collection.md", "# strict fixture\n")
    _write_sums(
        root,
        [
            "site_evidence_collection.csv",
            "site_evidence_collection.json",
            "site_evidence_collection.md",
        ],
    )
    return root


def _make_manual(parent: Path) -> Path:
    root = parent / "manual"
    _write(root / "manual_4ws_summary.csv", "scenario,status\n")
    _write(root / "manual_4ws_report.md", "# manual fixture\n")
    setup = _write_json(root / "session/session_setup.json", {"status": "PASS"})
    teardown = _write_json(root / "session_teardown/session_teardown.json", {"status": "PASS"})
    scenario_manifests = []
    for scenario in curator.MANUAL_SCENARIOS:
        scenario_root = root / scenario
        run = _write_json(
            scenario_root / "scenario_run.json", {"scenario": scenario, "status": "PASS"}
        )
        interactions = _write_json(
            scenario_root / "ui_interactions.json", {"scenario": scenario}
        )
        physical = _write_json(
            scenario_root / "physical_wheels.manifest.json", {"status": "STOPPED"}
        )
        capture = _write_json(
            scenario_root / "visual/capture_manifest.json", {"status": "PASS"}
        )
        png = _write(
            scenario_root / "visual/representative_contact_sheet.png",
            b"PNG" + scenario.encode(),
        )
        gif = _write(
            scenario_root / "visual/representative_motion.gif",
            b"GIF89a" + scenario.encode(),
        )
        wheel_json = _write_json(
            scenario_root / "wheel_summary/wheel_summary.json", {"status": "PASS"}
        )
        wheel_csv = _write(
            scenario_root / "wheel_summary/wheel_measurements.csv", "wheel,value\n"
        )
        wheel_png = _write(
            scenario_root / "wheel_summary/wheel_summary.png",
            b"PNG-WHEEL" + scenario.encode(),
        )
        _write(scenario_root / "ros_trace.jsonl", "large raw data excluded\n")
        _write(scenario_root / "physical_wheels.jsonl", "large raw data excluded\n")
        manifest = _write_json(
            scenario_root / "scenario_manifest.json",
            {
                "scenario": scenario,
                "status": "PASS",
                "artifacts": {
                    "scenario_run": _artifact(run),
                    "ui_interactions": _artifact(interactions),
                    "physical_wheels_manifest": _artifact(physical),
                    "wheel_summary": {
                        "wheel_summary.json": _artifact(wheel_json),
                        "wheel_measurements.csv": _artifact(wheel_csv),
                        "wheel_summary.png": _artifact(wheel_png),
                    },
                    "visual": {
                        "manifest": _artifact(capture),
                        "png": _artifact(png),
                        "gif": _artifact(gif),
                    },
                },
            },
        )
        binding = _artifact(manifest)
        binding["path"] = f"{scenario}/scenario_manifest.json"
        scenario_manifests.append(binding)
    setup_binding = _artifact(setup)
    setup_binding["path"] = "session/session_setup.json"
    teardown_binding = _artifact(teardown)
    teardown_binding["path"] = "session_teardown/session_teardown.json"
    _write_json(
        root / "manual_4ws_summary.json",
        {
            "schema": curator.MANUAL_SCHEMA,
            "status": "PASS",
            "scenario_order": list(curator.MANUAL_SCENARIOS),
            "passed_scenarios": list(curator.MANUAL_SCENARIOS),
            "session": {"setup": setup_binding, "teardown": teardown_binding},
            "scenario_manifests": scenario_manifests,
        },
    )
    return root


def _verify_sums(root: Path) -> None:
    for line in (root / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        digest, relative = line.split("  ", 1)
        assert _sha(root / relative) == digest


def _full_fixture(tmp_path: Path):
    sites = list(curator.ALL_SITES)
    delivery = _make_site_source(tmp_path, "operator_delivery", "delivery", sites, symlink_first=True)
    delivery_strict = _make_strict(tmp_path, "operator_delivery", "delivery_strict", sites, [delivery])

    recall_a = _make_site_source(tmp_path, "operator_recall", "recall_a", sites[:3])
    recall_b = _make_site_source(tmp_path, "operator_recall", "recall_b", sites[3:])
    recall_strict_a = _make_strict(
        tmp_path, "operator_recall", "recall_strict_a", sites[:3], [recall_a]
    )
    recall_strict_b = _make_strict(
        tmp_path, "operator_recall", "recall_strict_b", sites[3:], [recall_b]
    )
    manual = _make_manual(tmp_path)
    return {
        "site_sources": {
            "operator_delivery": [delivery],
            "operator_recall": [recall_a, recall_b],
        },
        "strict_sources": {
            "operator_delivery": [delivery_strict],
            "operator_recall": [recall_strict_a, recall_strict_b],
        },
        "manual": manual,
    }


def test_curates_full_and_split_roots_without_raw_data_or_symlinks(tmp_path):
    fixture = _full_fixture(tmp_path / "inputs")
    output = tmp_path / "current"
    manifest = curator.curate(
        output,
        site_sources=fixture["site_sources"],
        strict_sources=fixture["strict_sources"],
        manual_source=fixture["manual"],
    )

    assert manifest["status"] == "PASS"
    assert set(manifest["site_bundles"]) == {"operator_delivery", "operator_recall"}
    assert manifest["manual_4ws"]["scenarios"] == list(curator.MANUAL_SCENARIOS)
    assert not (output / "operator_delivery/sites/B1/camping_site_matrix.json").is_symlink()
    assert (output / "operator_delivery/sites/B1/visual/representative_motion.gif").is_file()
    assert (output / "operator_recall/provenance/run_manifest_part_02.json").is_file()
    assert (output / "operator_recall/provenance/strict_validation/part_02/site_evidence_collection.json").is_file()
    summary = json.loads(
        (output / "operator_recall/summary/camping_site_metrics.json").read_text()
    )
    assert [row["site"] for row in summary["sites"]] == list(curator.ALL_SITES)
    assert summary["sites"][0]["source_report"] == "sites/B1/camping_site_matrix.json"

    all_files = [path for path in output.rglob("*") if path.is_file()]
    assert not any(path.is_symlink() for path in output.rglob("*"))
    assert not any(path.suffix in {".jsonl", ".mp4", ".log"} for path in all_files)
    for sums in output.rglob("SHA256SUMS"):
        _verify_sums(sums.parent)

    second = tmp_path / "current_second"
    curator.curate(
        second,
        site_sources=fixture["site_sources"],
        strict_sources=fixture["strict_sources"],
        manual_source=fixture["manual"],
    )
    first_hashes = {
        path.relative_to(output).as_posix(): _sha(path)
        for path in output.rglob("*")
        if path.is_file()
    }
    second_hashes = {
        path.relative_to(second).as_posix(): _sha(path)
        for path in second.rglob("*")
        if path.is_file()
    }
    assert first_hashes == second_hashes


def test_nonempty_output_requires_explicit_replace(tmp_path):
    fixture = _full_fixture(tmp_path / "inputs")
    output = tmp_path / "current"
    output.mkdir()
    (output / "user-file.txt").write_text("preserve unless explicit\n")
    with pytest.raises(curator.CurationError, match="--replace-nonempty"):
        curator.curate(
            output,
            site_sources=fixture["site_sources"],
            strict_sources=fixture["strict_sources"],
            manual_source=fixture["manual"],
        )
    assert (output / "user-file.txt").read_text() == "preserve unless explicit\n"

    curator.curate(
        output,
        site_sources=fixture["site_sources"],
        strict_sources=fixture["strict_sources"],
        manual_source=fixture["manual"],
        replace_nonempty=True,
    )
    assert not (output / "user-file.txt").exists()
    _verify_sums(output)


def test_incomplete_strict_coverage_fails_without_publishing_output(tmp_path):
    sites = list(curator.ALL_SITES)
    source = _make_site_source(tmp_path / "inputs", "guest_recall", "guest", sites)
    strict = _make_strict(
        tmp_path / "inputs", "guest_recall", "guest_strict", sites[:-1], [source]
    )
    output = tmp_path / "current"
    with pytest.raises(curator.CurationError, match="coverage is incomplete.*B13"):
        curator.curate(
            output,
            site_sources={"guest_recall": [source]},
            strict_sources={"guest_recall": [strict]},
        )
    assert not output.exists()
    assert not list(tmp_path.glob(".current.staging-*"))


def test_strict_hash_mismatch_fails_closed(tmp_path):
    sites = list(curator.ALL_SITES)
    source = _make_site_source(tmp_path / "inputs", "operator_delivery", "delivery", sites)
    strict = _make_strict(
        tmp_path / "inputs", "operator_delivery", "strict", sites, [source]
    )
    document_path = strict / "site_evidence_collection.json"
    document = json.loads(document_path.read_text())
    document["sites"][0]["gif_sha256"] = "f" * 64
    _write_json(document_path, document)
    _write_sums(
        strict,
        [
            "site_evidence_collection.csv",
            "site_evidence_collection.json",
            "site_evidence_collection.md",
        ],
    )
    with pytest.raises(curator.CurationError, match="strict gif_sha256"):
        curator.curate(
            tmp_path / "current",
            site_sources={"operator_delivery": [source]},
            strict_sources={"operator_delivery": [strict]},
        )
