#!/usr/bin/env python3
"""Curate completed CAMROD x CARLA evidence into a small, reviewable tree.

The runtime evidence roots intentionally retain large JSONL streams, logs and
intermediate videos.  This tool copies only the accepted manifests, derived
PNG/GIF files and wheel summaries needed for a repository evidence bundle.
It also supports a scenario split across multiple runner roots (for example
operator recall B1..B3 plus B4..B13).

Inputs are read-only.  The destination is built in a sibling staging directory
and installed only after every site, strict-validation binding and hash passes.
A non-empty destination is never replaced unless ``--replace-nonempty`` is
given explicitly.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile
from types import ModuleType
from typing import Any, Iterable, Mapping, Sequence


SCHEMA = "camrod.virtual_carla.curated_evidence.v1"
SITE_MANIFEST_SCHEMA = "camrod.virtual_carla.site_evidence.v1"
MATRIX_SCHEMA = "camrod.virtual_carla.camping_site_matrix.v1"
STRICT_SCHEMA = "camrod.virtual_carla.site_evidence_collection_validation.v1"
MANUAL_SCHEMA = "camrod.virtual_carla.manual_4ws_collection.v1"
ALL_SITES = tuple(f"B{number}" for number in range(1, 14))
SITE_RE = re.compile(r"^B(?:[1-9]|1[0-3])$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
MANUAL_SCENARIOS = ("straight", "turn", "crab", "zero_turn")

BUNDLE_CONTRACTS: dict[str, dict[str, str]] = {
    "operator_delivery": {
        "title": "Operator delivery + Return",
        "frontend": "operator-browser",
        "strict_authority": "operator-browser",
        "mission": "delivery",
        "return_authority": "operator_browser",
    },
    "operator_recall": {
        "title": "Operator recall + Return",
        "frontend": "operator-browser",
        "strict_authority": "operator-browser",
        "mission": "recall",
        "return_authority": "operator_browser",
    },
    "guest_recall": {
        "title": "Guest UI recall + usage complete",
        "frontend": "guest",
        "strict_authority": "guest",
        "mission": "recall",
        "return_authority": "guest_browser",
    },
}

SITE_COPY_LAYOUT: tuple[tuple[str, str, bool], ...] = (
    ("site_manifest.json", "site_manifest.json", False),
    ("camping_site_matrix.json", "camping_site_matrix.json", True),
    ("physical_wheels.manifest.json", "physical_wheels.manifest.json", False),
    ("visual/capture_manifest.json", "visual/capture_manifest.json", False),
    (
        "visual/representative_contact_sheet.png",
        "visual/representative_contact_sheet.png",
        False,
    ),
    (
        "visual/representative_motion.gif",
        "visual/representative_motion.gif",
        False,
    ),
    (
        "wheel_summary/wheel_summary.json",
        "wheel_summary/wheel_summary.json",
        False,
    ),
    (
        "wheel_summary/wheel_measurements.csv",
        "wheel_summary/wheel_measurements.csv",
        False,
    ),
    (
        "wheel_summary/wheel_summary.png",
        "wheel_summary/wheel_summary.png",
        False,
    ),
)

STRICT_FILES = (
    "site_evidence_collection.json",
    "site_evidence_collection.csv",
    "site_evidence_collection.md",
    "SHA256SUMS",
)

MANUAL_ROOT_FILES = (
    "manual_4ws_summary.json",
    "manual_4ws_summary.csv",
    "manual_4ws_report.md",
)
MANUAL_SCENARIO_FILES: tuple[tuple[str, str], ...] = (
    ("scenario_manifest.json", "scenario_manifest.json"),
    ("scenario_run.json", "scenario_run.json"),
    ("ui_interactions.json", "ui_interactions.json"),
    ("physical_wheels.manifest.json", "physical_wheels.manifest.json"),
    ("visual/capture_manifest.json", "visual/capture_manifest.json"),
    (
        "visual/representative_contact_sheet.png",
        "visual/representative_contact_sheet.png",
    ),
    ("visual/representative_motion.gif", "visual/representative_motion.gif"),
    (
        "wheel_summary/wheel_summary.json",
        "wheel_summary/wheel_summary.json",
    ),
    (
        "wheel_summary/wheel_measurements.csv",
        "wheel_summary/wheel_measurements.csv",
    ),
    ("wheel_summary/wheel_summary.png", "wheel_summary/wheel_summary.png"),
)

EXCLUDED_CLASSES = (
    "raw *.jsonl telemetry/ROS traces",
    "source *.mp4 recordings",
    "runtime *.log files",
    "ffprobe and exact-command intermediates",
)


class CurationError(RuntimeError):
    """Raised when evidence is incomplete, inconsistent or unsafe to install."""


@dataclass(frozen=True)
class SiteEvidence:
    bundle: str
    site: str
    source_root: Path
    site_root: Path
    matrix: Mapping[str, Any]
    site_manifest: Mapping[str, Any]
    artifact_sha256: Mapping[str, str]


@dataclass(frozen=True)
class SourcePart:
    root: Path
    sites: tuple[str, ...]
    run_manifest: Path
    runner_status: str


@dataclass(frozen=True)
class StrictPart:
    root: Path
    sites: tuple[str, ...]
    document: Mapping[str, Any]


def _duplicate_key_rejector(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise CurationError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _read_json(path: Path, label: str) -> Mapping[str, Any]:
    _require_regular_file(path, label, allow_symlink=False)
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_duplicate_key_rejector,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CurationError(f"cannot read {label} JSON {path}: {error}") from error
    if not isinstance(value, Mapping):
        raise CurationError(f"{label} must be a JSON object: {path}")
    return value


def _require_regular_file(path: Path, label: str, *, allow_symlink: bool) -> Path:
    if path.is_symlink() and not allow_symlink:
        raise CurationError(f"{label} must not be a symlink: {path}")
    if not path.is_file():
        raise CurationError(f"{label} is not a regular file: {path}")
    resolved = path.resolve(strict=True)
    if not resolved.is_file():
        raise CurationError(f"{label} does not resolve to a regular file: {path}")
    return resolved


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact_hashes(site_root: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for source_name, _destination_name, allow_symlink in SITE_COPY_LAYOUT:
        path = site_root / source_name
        resolved = _require_regular_file(
            path, f"site artifact {source_name}", allow_symlink=allow_symlink
        )
        result[source_name] = _sha256(resolved)
    return result


def _expected_sha(record: Any, label: str) -> str:
    if not isinstance(record, Mapping):
        raise CurationError(f"{label} artifact binding is missing")
    value = str(record.get("sha256", "")).strip().lower()
    if not SHA256_RE.fullmatch(value):
        raise CurationError(f"{label}.sha256 is invalid: {value!r}")
    return value


def _assert_bound(record: Any, actual: str, label: str) -> None:
    expected = _expected_sha(record, label)
    if expected != actual:
        raise CurationError(
            f"{label} SHA-256 mismatch: manifest={expected}, actual={actual}"
        )


def _assert_file_bound(record: Any, path: Path, label: str) -> None:
    resolved = _require_regular_file(path, label, allow_symlink=False)
    _assert_bound(record, _sha256(resolved), label)
    if not isinstance(record, Mapping):
        raise CurationError(f"{label} artifact binding is missing")
    try:
        expected_bytes = int(record.get("bytes", -1))
    except (TypeError, ValueError):
        expected_bytes = -1
    if expected_bytes != resolved.stat().st_size:
        raise CurationError(
            f"{label} byte-size mismatch: manifest={expected_bytes}, "
            f"actual={resolved.stat().st_size}"
        )


def _site_number(site: str) -> int:
    return int(site[1:])


def _discover_site_sources(
    bundle: str, roots: Sequence[Path]
) -> tuple[dict[str, SiteEvidence], list[SourcePart]]:
    contract = BUNDLE_CONTRACTS[bundle]
    by_site: dict[str, SiteEvidence] = {}
    parts: list[SourcePart] = []
    seen_roots: set[Path] = set()

    for raw_root in roots:
        expanded_root = raw_root.expanduser()
        if expanded_root.is_symlink():
            raise CurationError(f"{bundle} site source must not be a symlink: {expanded_root}")
        root = expanded_root.resolve(strict=True)
        if root in seen_roots:
            raise CurationError(f"duplicate {bundle} site source root: {root}")
        seen_roots.add(root)
        if not root.is_dir():
            raise CurationError(f"{bundle} site source must be a directory: {root}")
        run_manifest = root / "run_manifest.json"
        run_document = _read_json(run_manifest, f"{bundle} run manifest")
        part_sites: list[str] = []

        for site in ALL_SITES:
            site_root = root / site
            if not site_root.is_dir():
                continue
            if site_root.is_symlink():
                raise CurationError(f"site directory must not be a symlink: {site_root}")
            if site in by_site:
                raise CurationError(
                    f"duplicate finalized {bundle} site {site}: "
                    f"{by_site[site].source_root} and {root}"
                )
            hashes = _artifact_hashes(site_root)
            site_manifest = _read_json(
                site_root / "site_manifest.json", f"{bundle} {site} site manifest"
            )
            matrix_path = _require_regular_file(
                site_root / "camping_site_matrix.json",
                f"{bundle} {site} matrix",
                allow_symlink=True,
            )
            try:
                matrix = json.loads(
                    matrix_path.read_text(encoding="utf-8"),
                    object_pairs_hook=_duplicate_key_rejector,
                )
            except (OSError, UnicodeError, json.JSONDecodeError) as error:
                raise CurationError(
                    f"cannot read {bundle} {site} matrix {matrix_path}: {error}"
                ) from error
            if not isinstance(matrix, Mapping):
                raise CurationError(f"{bundle} {site} matrix must be a JSON object")

            if site_manifest.get("schema") != SITE_MANIFEST_SCHEMA:
                raise CurationError(f"{bundle} {site} site manifest schema is invalid")
            if str(site_manifest.get("status", "")).upper() != "PASS":
                raise CurationError(f"{bundle} {site} site manifest is not PASS")
            if str(site_manifest.get("site", "")).upper() != site:
                raise CurationError(f"{bundle} {site} site manifest identity differs")
            authority = site_manifest.get("authority")
            if not isinstance(authority, Mapping):
                raise CurationError(f"{bundle} {site} authority is missing")
            if authority.get("frontend") != contract["frontend"]:
                raise CurationError(f"{bundle} {site} frontend authority differs")
            if authority.get("mission_intent") != contract["mission"]:
                raise CurationError(f"{bundle} {site} mission intent differs")
            if authority.get("matrix_return_authority") != contract["return_authority"]:
                raise CurationError(f"{bundle} {site} return authority differs")

            if matrix.get("schema") != MATRIX_SCHEMA:
                raise CurationError(f"{bundle} {site} matrix schema is invalid")
            if str(matrix.get("status", "")).upper() != "PASS":
                raise CurationError(f"{bundle} {site} matrix is not PASS")
            scope = matrix.get("scope")
            if not isinstance(scope, Mapping):
                raise CurationError(f"{bundle} {site} matrix scope is missing")
            selected = scope.get("selected_sites")
            if selected != [site]:
                raise CurationError(
                    f"{bundle} {site} matrix must select exactly [{site!r}], got {selected!r}"
                )
            if scope.get("mission_intent") != contract["mission"]:
                raise CurationError(f"{bundle} {site} matrix mission intent differs")
            if scope.get("return_authority") != contract["return_authority"]:
                raise CurationError(f"{bundle} {site} matrix return authority differs")

            capture = _read_json(
                site_root / "visual/capture_manifest.json",
                f"{bundle} {site} capture manifest",
            )
            physical = _read_json(
                site_root / "physical_wheels.manifest.json",
                f"{bundle} {site} physical-wheel manifest",
            )
            wheel = _read_json(
                site_root / "wheel_summary/wheel_summary.json",
                f"{bundle} {site} wheel summary",
            )
            if str(capture.get("status", "")).upper() != "PASS":
                raise CurationError(f"{bundle} {site} capture is not PASS")
            if str(physical.get("status", "")).upper() not in {"STOPPED", "COMPLETED"}:
                raise CurationError(f"{bundle} {site} physical-wheel stream is not finalized")
            if str(wheel.get("status", "")).upper() != "PASS":
                raise CurationError(f"{bundle} {site} wheel summary is not PASS")

            _assert_bound(
                site_manifest.get("matrix_report"),
                hashes["camping_site_matrix.json"],
                f"{bundle} {site} matrix",
            )
            visual = site_manifest.get("visual")
            if not isinstance(visual, Mapping):
                raise CurationError(f"{bundle} {site} visual bindings are missing")
            _assert_bound(
                visual.get("manifest"),
                hashes["visual/capture_manifest.json"],
                f"{bundle} {site} capture manifest",
            )
            _assert_bound(
                visual.get("png"),
                hashes["visual/representative_contact_sheet.png"],
                f"{bundle} {site} PNG",
            )
            _assert_bound(
                visual.get("gif"),
                hashes["visual/representative_motion.gif"],
                f"{bundle} {site} GIF",
            )
            physical_binding = site_manifest.get("physical_wheels")
            if not isinstance(physical_binding, Mapping):
                raise CurationError(f"{bundle} {site} physical bindings are missing")
            _assert_bound(
                physical_binding.get("manifest"),
                hashes["physical_wheels.manifest.json"],
                f"{bundle} {site} physical-wheel manifest",
            )
            summary_binding = physical_binding.get("summary")
            if not isinstance(summary_binding, Mapping):
                raise CurationError(f"{bundle} {site} wheel-summary binding is missing")
            summary_artifacts = summary_binding.get("artifacts")
            if not isinstance(summary_artifacts, Mapping):
                raise CurationError(f"{bundle} {site} wheel-summary artifacts are missing")
            for name in (
                "wheel_summary.json",
                "wheel_measurements.csv",
                "wheel_summary.png",
            ):
                _assert_bound(
                    summary_artifacts.get(name),
                    hashes[f"wheel_summary/{name}"],
                    f"{bundle} {site} {name}",
                )

            by_site[site] = SiteEvidence(
                bundle=bundle,
                site=site,
                source_root=root,
                site_root=site_root,
                matrix=matrix,
                site_manifest=site_manifest,
                artifact_sha256=hashes,
            )
            part_sites.append(site)

        if not part_sites:
            raise CurationError(f"{bundle} source contains no finalized site directories: {root}")
        parts.append(
            SourcePart(
                root=root,
                sites=tuple(sorted(part_sites, key=_site_number)),
                run_manifest=run_manifest,
                runner_status=str(run_document.get("status", "UNKNOWN")).upper(),
            )
        )

    missing = [site for site in ALL_SITES if site not in by_site]
    if missing:
        raise CurationError(
            f"{bundle} B1..B13 coverage is incomplete; missing: {', '.join(missing)}"
        )
    parts.sort(key=lambda part: tuple(_site_number(site) for site in part.sites))
    return by_site, parts


def _verify_sha256s_file(root: Path) -> None:
    sums = _require_regular_file(
        root / "SHA256SUMS", "strict-validation SHA256SUMS", allow_symlink=False
    )
    listed: set[str] = set()
    try:
        lines = sums.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise CurationError(f"cannot read {sums}: {error}") from error
    for number, line in enumerate(lines, start=1):
        if not line:
            continue
        try:
            digest, raw_name = line.split("  ", 1)
        except ValueError:
            raise CurationError(f"invalid SHA256SUMS line {number}: {line!r}") from None
        if not SHA256_RE.fullmatch(digest):
            raise CurationError(f"invalid SHA-256 on line {number}: {digest!r}")
        relative = Path(raw_name)
        if relative.is_absolute() or ".." in relative.parts:
            raise CurationError(f"unsafe SHA256SUMS path on line {number}: {raw_name!r}")
        normalized = relative.as_posix().removeprefix("./")
        if normalized in listed:
            raise CurationError(f"duplicate SHA256SUMS entry: {normalized}")
        listed.add(normalized)
        candidate = _require_regular_file(
            root / normalized,
            f"strict-validation artifact {normalized}",
            allow_symlink=False,
        )
        if _sha256(candidate) != digest:
            raise CurationError(f"strict-validation hash mismatch: {candidate}")
    expected = set(STRICT_FILES) - {"SHA256SUMS"}
    if listed != expected:
        raise CurationError(
            f"strict-validation SHA256SUMS must list {sorted(expected)}, got {sorted(listed)}"
        )


def _discover_strict_sources(
    bundle: str,
    roots: Sequence[Path],
    sites: Mapping[str, SiteEvidence],
) -> list[StrictPart]:
    contract = BUNDLE_CONTRACTS[bundle]
    covered: set[str] = set()
    parts: list[StrictPart] = []
    seen_roots: set[Path] = set()
    for raw_root in roots:
        expanded_root = raw_root.expanduser()
        if expanded_root.is_symlink():
            raise CurationError(
                f"{bundle} strict-validation source must not be a symlink: {expanded_root}"
            )
        root = expanded_root.resolve(strict=True)
        if root in seen_roots:
            raise CurationError(f"duplicate {bundle} strict-validation root: {root}")
        seen_roots.add(root)
        if not root.is_dir():
            raise CurationError(f"strict-validation source must be a directory: {root}")
        for name in STRICT_FILES:
            _require_regular_file(
                root / name,
                f"strict-validation artifact {name}",
                allow_symlink=False,
            )
        _verify_sha256s_file(root)
        document = _read_json(
            root / "site_evidence_collection.json", f"{bundle} strict validation"
        )
        if document.get("schema") != STRICT_SCHEMA:
            raise CurationError(f"{bundle} strict-validation schema is invalid")
        if str(document.get("status", "")).upper() != "PASS":
            raise CurationError(f"{bundle} strict validation is not PASS")
        expectation = document.get("expectation")
        if not isinstance(expectation, Mapping):
            raise CurationError(f"{bundle} strict-validation expectation is missing")
        if expectation.get("authority") != contract["strict_authority"]:
            raise CurationError(f"{bundle} strict-validation authority differs")
        if expectation.get("mission_intent") != contract["mission"]:
            raise CurationError(f"{bundle} strict-validation mission differs")
        validations = document.get("validation")
        if not isinstance(validations, Mapping) or not validations:
            raise CurationError(f"{bundle} strict-validation checks are missing")
        runner_modes = (
            validations.get("runner_status_pass") is True,
            validations.get("runner_prepass_validating") is True,
        )
        if sum(runner_modes) != 1:
            raise CurationError(
                f"{bundle} strict validation must bind exactly one finalized runner mode"
            )
        failed_checks = sorted(
            key
            for key, value in validations.items()
            if key not in {"runner_status_pass", "runner_prepass_validating"}
            and value is not True
        )
        if failed_checks:
            raise CurationError(
                f"{bundle} strict-validation checks are not true: {failed_checks}"
            )
        raw_rows = document.get("sites")
        if not isinstance(raw_rows, list) or not raw_rows:
            raise CurationError(f"{bundle} strict-validation sites are missing")
        part_sites: list[str] = []
        for row in raw_rows:
            if not isinstance(row, Mapping):
                raise CurationError(f"{bundle} strict-validation site row is invalid")
            site = str(row.get("site", "")).upper()
            if site not in sites:
                raise CurationError(f"{bundle} strict validation names unknown site {site!r}")
            if site in covered or site in part_sites:
                raise CurationError(f"duplicate strict-validation coverage for {bundle} {site}")
            if str(row.get("status", "")).upper() != "PASS":
                raise CurationError(f"{bundle} {site} strict-validation row is not PASS")
            evidence = sites[site]
            bindings = {
                "matrix_sha256": "camping_site_matrix.json",
                "site_manifest_sha256": "site_manifest.json",
                "wheel_summary_sha256": "wheel_summary/wheel_summary.json",
                "png_sha256": "visual/representative_contact_sheet.png",
                "gif_sha256": "visual/representative_motion.gif",
            }
            for field, artifact_name in bindings.items():
                actual = evidence.artifact_sha256[artifact_name]
                if row.get(field) != actual:
                    raise CurationError(
                        f"{bundle} {site} strict {field} does not bind the curated artifact"
                    )
            part_sites.append(site)
        expected_part = expectation.get("sites")
        if not isinstance(expected_part, list) or set(expected_part) != set(part_sites):
            raise CurationError(
                f"{bundle} strict-validation expectation site set differs from its rows"
            )
        covered.update(part_sites)
        parts.append(
            StrictPart(
                root=root,
                sites=tuple(sorted(part_sites, key=_site_number)),
                document=document,
            )
        )
    missing = [site for site in ALL_SITES if site not in covered]
    if missing:
        raise CurationError(
            f"{bundle} strict-validation coverage is incomplete; missing: {', '.join(missing)}"
        )
    parts.sort(key=lambda part: tuple(_site_number(site) for site in part.sites))
    return parts


def _copy_file(source: Path, destination: Path, *, allow_symlink: bool = False) -> str:
    resolved = _require_regular_file(source, "curation source", allow_symlink=allow_symlink)
    before = _sha256(resolved)
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        with resolved.open("rb") as reader, destination.open("xb") as writer:
            while chunk := reader.read(1024 * 1024):
                writer.write(chunk)
            writer.flush()
            os.fsync(writer.fileno())
    except OSError as error:
        raise CurationError(f"cannot copy {source} to {destination}: {error}") from error
    after = _sha256(resolved)
    copied = _sha256(destination)
    if before != after or copied != before:
        raise CurationError(f"source changed while copying: {source}")
    return copied


def _write_bytes(path: Path, value: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("xb") as stream:
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
    except OSError as error:
        raise CurationError(f"cannot create {path}: {error}") from error


def _write_text(path: Path, value: str) -> None:
    _write_bytes(path, value.encode("utf-8"))


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    _write_text(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )


def _regular_files(root: Path) -> Iterable[Path]:
    for path in sorted(root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()):
        if path.is_symlink():
            raise CurationError(f"curated output unexpectedly contains a symlink: {path}")
        if path.is_file():
            yield path


def _write_sha256s(root: Path, *, recursive: bool) -> Path:
    output = root / "SHA256SUMS"
    if output.exists() or output.is_symlink():
        raise CurationError(f"refusing to overwrite SHA256SUMS: {output}")
    if recursive:
        candidates = [path for path in _regular_files(root) if path != output]
    else:
        candidates = sorted(
            (path for path in root.iterdir() if path.is_file() and path != output),
            key=lambda path: path.name,
        )
    lines = [
        f"{_sha256(path)}  {path.relative_to(root).as_posix()}\n"
        for path in candidates
    ]
    _write_text(output, "".join(lines))
    return output


def _load_metrics_module() -> ModuleType:
    path = Path(__file__).with_name("summarize_camping_site_metrics.py")
    spec = importlib.util.spec_from_file_location("_camrod_curate_metrics", path)
    if spec is None or spec.loader is None:
        raise CurationError(f"cannot load metrics helper: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _build_summary(
    destination: Path, sites: Mapping[str, SiteEvidence]
) -> Mapping[str, Any]:
    metrics = _load_metrics_module()
    matrices = [sites[site].site_root / "camping_site_matrix.json" for site in ALL_SITES]
    try:
        summary = metrics.build_summary(matrices, require_all_sites=True)
    except Exception as error:
        raise CurationError(f"cannot construct B1..B13 metrics summary: {error}") from error
    if summary.get("aggregate", {}).get("pass_count") != len(ALL_SITES):
        raise CurationError("metrics summary does not contain 13 PASS sites")

    for row in summary["sites"]:
        site = str(row["site"])
        row["source_report"] = f"sites/{site}/camping_site_matrix.json"
    summary["source_reports"] = [
        {
            "path": f"sites/{site}/camping_site_matrix.json",
            "sha256": sites[site].artifact_sha256["camping_site_matrix.json"],
            "report_status": "PASS",
            "finalized_sites": [site],
        }
        for site in ALL_SITES
    ]
    try:
        metrics.write_summary(destination, summary)
    except Exception as error:
        raise CurationError(f"cannot write B1..B13 metrics summary: {error}") from error
    return summary


def _copy_strict_parts(destination: Path, parts: Sequence[StrictPart]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for index, part in enumerate(parts, start=1):
        if len(parts) == 1:
            part_destination = destination
            relative_root = "provenance/strict_validation"
        else:
            part_destination = destination / f"part_{index:02d}"
            relative_root = f"provenance/strict_validation/part_{index:02d}"
        for name in STRICT_FILES:
            _copy_file(part.root / name, part_destination / name)
        records.append(
            {
                "sites": list(part.sites),
                "path": relative_root,
                "validation_sha256": _sha256(part.root / "site_evidence_collection.json"),
            }
        )
    return records


def _bundle_readme(bundle: str, aggregate: Mapping[str, Any], strict_count: int) -> str:
    contract = BUNDLE_CONTRACTS[bundle]
    return f"""# {contract['title']} — final accepted evidence

이 bundle은 Woraksan CARLA에서 B1~B13을 실제 주행한 최종 PASS 자료를 선별한
저장소다. 원본 runtime evidence는 변경하지 않았고 matrix symlink는 일반 JSON 파일로
역참조 복사했다.

## 결과

- B1~B13: `13/13 PASS`
- 전체 경과 시간: `{aggregate['total_elapsed_s']} s`
- 전체 odometry 거리: `{aggregate['total_odom_distance_m']} m`
- outbound: `{aggregate['outbound_duration_s']} s`, `{aggregate['outbound_distance_m']} m`
- return: `{aggregate['return_duration_s']} s`, `{aggregate['return_distance_m']} m`
- strict-validation 묶음: `{strict_count}`개, 전체 사이트와 PNG/GIF/matrix/wheel hash 대조 완료

사이트별 시간·거리·Drop Zone 오차는
[`summary/camping_site_metrics.md`](summary/camping_site_metrics.md)에 있다.

## 보존 범위

각 `sites/B1`~`sites/B13`에는 site/matrix/capture/physical-wheel manifest,
실제 CARLA와 UI를 함께 담은 `representative_contact_sheet.png`,
`representative_motion.gif`, wheel JSON/CSV/PNG가 있다. `provenance/`에는 원본 runner
manifest와 독립 strict-validator 결과가 있다.

## 제외 범위

용량이 큰 raw wheel/ROS JSONL, 원본 MP4, runtime log, ffprobe/command 중간 파일은
의도적으로 복사하지 않았다. 이 파일들의 원본 hash와 byte 수는 보존된 manifest가
계속 기록한다.

## 무결성 확인

```bash
sha256sum -c SHA256SUMS
```
"""


def _curate_site_bundle(
    bundle: str,
    source_roots: Sequence[Path],
    strict_roots: Sequence[Path],
    destination: Path,
) -> dict[str, Any]:
    sites, source_parts = _discover_site_sources(bundle, source_roots)
    strict_parts = _discover_strict_sources(bundle, strict_roots, sites)
    destination.mkdir(parents=True, exist_ok=False)

    part_records: list[dict[str, Any]] = []
    provenance = destination / "provenance"
    provenance.mkdir()
    for index, part in enumerate(source_parts, start=1):
        name = "run_manifest.json" if len(source_parts) == 1 else f"run_manifest_part_{index:02d}.json"
        digest = _copy_file(part.run_manifest, provenance / name)
        part_records.append(
            {
                "sites": list(part.sites),
                "path": f"provenance/{name}",
                "sha256": digest,
                "runner_status": part.runner_status,
            }
        )

    strict_records = _copy_strict_parts(provenance / "strict_validation", strict_parts)
    for site in ALL_SITES:
        evidence = sites[site]
        site_destination = destination / "sites" / site
        for source_name, destination_name, allow_symlink in SITE_COPY_LAYOUT:
            digest = _copy_file(
                evidence.site_root / source_name,
                site_destination / destination_name,
                allow_symlink=allow_symlink,
            )
            if digest != evidence.artifact_sha256[source_name]:
                raise CurationError(f"{bundle} {site} changed after input validation")
        _write_sha256s(site_destination / "visual", recursive=False)
        _write_sha256s(site_destination / "wheel_summary", recursive=False)

    summary = _build_summary(destination / "summary", sites)
    aggregate = summary["aggregate"]
    bundle_manifest = {
        "schema": SCHEMA,
        "status": "PASS",
        "kind": "site_bundle",
        "bundle": bundle,
        "title": BUNDLE_CONTRACTS[bundle]["title"],
        "authority": BUNDLE_CONTRACTS[bundle]["strict_authority"],
        "mission_intent": BUNDLE_CONTRACTS[bundle]["mission"],
        "sites": list(ALL_SITES),
        "source_parts": part_records,
        "strict_validation_parts": strict_records,
        "aggregate": aggregate,
        "excluded_classes": list(EXCLUDED_CLASSES),
    }
    _write_json(destination / "curation_manifest.json", bundle_manifest)
    _write_text(
        destination / "README.md",
        _bundle_readme(bundle, aggregate, len(strict_parts)),
    )
    bundle_sums = _write_sha256s(destination, recursive=True)
    return {
        "status": "PASS",
        "sites": list(ALL_SITES),
        "aggregate": aggregate,
        "source_parts": part_records,
        "strict_validation_parts": strict_records,
        "path": bundle,
        "sha256sums_sha256": _sha256(bundle_sums),
    }


def _manual_readme() -> str:
    return """# Visible Robot UI manual 4WS — final accepted evidence

이 bundle은 표시 중인 production Robot UI의 실제 포인터·키보드 입력만으로 straight,
turn, crab, zero-turn을 실행한 결과다. 각 시나리오에는 실제 CARLA/UI PNG·GIF와
FL/FR/RL/RR physical-wheel 측정 요약이 들어 있다.

raw ROS/wheel JSONL, 원본 MP4와 runtime log는 저장소 용량을 위해 제외했다. 원본
파일의 hash와 byte 수는 scenario manifest에 남아 있다.

```bash
sha256sum -c SHA256SUMS
```
"""


def _curate_manual(source_root: Path, destination: Path) -> dict[str, Any]:
    expanded_root = source_root.expanduser()
    if expanded_root.is_symlink():
        raise CurationError(f"manual source must not be a symlink: {expanded_root}")
    root = expanded_root.resolve(strict=True)
    if not root.is_dir():
        raise CurationError(f"manual source must be a directory: {root}")
    summary = _read_json(root / "manual_4ws_summary.json", "manual 4WS summary")
    if summary.get("schema") != MANUAL_SCHEMA:
        raise CurationError(f"manual 4WS collection schema is invalid: {summary.get('schema')!r}")
    if str(summary.get("status", "")).upper() != "PASS":
        raise CurationError("manual 4WS collection is not PASS")
    if tuple(summary.get("scenario_order", ())) != MANUAL_SCENARIOS:
        raise CurationError("manual 4WS scenario order is incomplete or changed")
    if set(summary.get("passed_scenarios", ())) != set(MANUAL_SCENARIOS):
        raise CurationError("manual 4WS collection does not pass every scenario")

    session_bindings = summary.get("session")
    if not isinstance(session_bindings, Mapping):
        raise CurationError("manual 4WS session artifact bindings are missing")
    _assert_file_bound(
        session_bindings.get("setup"),
        root / "session/session_setup.json",
        "manual session setup",
    )
    _assert_file_bound(
        session_bindings.get("teardown"),
        root / "session_teardown/session_teardown.json",
        "manual session teardown",
    )
    raw_manifest_bindings = summary.get("scenario_manifests")
    if not isinstance(raw_manifest_bindings, list):
        raise CurationError("manual scenario-manifest bindings are missing")
    manifest_bindings: dict[str, Mapping[str, Any]] = {}
    for record in raw_manifest_bindings:
        if not isinstance(record, Mapping):
            raise CurationError("manual scenario-manifest artifact binding is invalid")
        path = str(record.get("path", ""))
        matches = [
            scenario
            for scenario in MANUAL_SCENARIOS
            if path == f"{scenario}/scenario_manifest.json"
        ]
        if len(matches) != 1 or matches[0] in manifest_bindings:
            raise CurationError(f"manual scenario-manifest path is invalid: {path!r}")
        manifest_bindings[matches[0]] = record
    if set(manifest_bindings) != set(MANUAL_SCENARIOS):
        raise CurationError("manual summary does not bind all scenario manifests")

    destination.mkdir(parents=True, exist_ok=False)
    for name in MANUAL_ROOT_FILES:
        _copy_file(root / name, destination / name)
    for source_name, destination_name in (
        ("session/session_setup.json", "session/session_setup.json"),
        ("session_teardown/session_teardown.json", "session_teardown/session_teardown.json"),
    ):
        document = _read_json(root / source_name, f"manual {source_name}")
        if str(document.get("status", "")).upper() != "PASS":
            raise CurationError(f"manual session boundary is not PASS: {source_name}")
        _copy_file(root / source_name, destination / destination_name)

    scenario_records: list[dict[str, str]] = []
    for scenario in MANUAL_SCENARIOS:
        scenario_root = root / scenario
        manifest = _read_json(
            scenario_root / "scenario_manifest.json", f"manual {scenario} manifest"
        )
        if str(manifest.get("scenario", "")) != scenario:
            raise CurationError(f"manual scenario identity differs: {scenario}")
        if str(manifest.get("status", "")).upper() != "PASS":
            raise CurationError(f"manual scenario is not PASS: {scenario}")
        _assert_file_bound(
            manifest_bindings[scenario],
            scenario_root / "scenario_manifest.json",
            f"manual {scenario} scenario manifest",
        )
        scenario_run = _read_json(
            scenario_root / "scenario_run.json", f"manual {scenario} scenario run"
        )
        capture = _read_json(
            scenario_root / "visual/capture_manifest.json",
            f"manual {scenario} capture manifest",
        )
        physical = _read_json(
            scenario_root / "physical_wheels.manifest.json",
            f"manual {scenario} physical-wheel manifest",
        )
        wheel = _read_json(
            scenario_root / "wheel_summary/wheel_summary.json",
            f"manual {scenario} wheel summary",
        )
        if str(scenario_run.get("status", "")).upper() != "PASS":
            raise CurationError(f"manual {scenario} scenario run is not PASS")
        if str(capture.get("status", "")).upper() != "PASS":
            raise CurationError(f"manual {scenario} capture is not PASS")
        if str(physical.get("status", "")).upper() not in {"STOPPED", "COMPLETED"}:
            raise CurationError(f"manual {scenario} physical-wheel stream is not finalized")
        if str(wheel.get("status", "")).upper() != "PASS":
            raise CurationError(f"manual {scenario} wheel summary is not PASS")
        artifacts = manifest.get("artifacts")
        if not isinstance(artifacts, Mapping):
            raise CurationError(f"manual {scenario} artifact bindings are missing")
        direct_bindings = {
            "scenario_run": "scenario_run.json",
            "ui_interactions": "ui_interactions.json",
            "physical_wheels_manifest": "physical_wheels.manifest.json",
        }
        for field, relative in direct_bindings.items():
            _assert_file_bound(
                artifacts.get(field),
                scenario_root / relative,
                f"manual {scenario} {field}",
            )
        wheel_bindings = artifacts.get("wheel_summary")
        if not isinstance(wheel_bindings, Mapping):
            raise CurationError(f"manual {scenario} wheel-summary bindings are missing")
        for name in ("wheel_summary.json", "wheel_measurements.csv", "wheel_summary.png"):
            _assert_file_bound(
                wheel_bindings.get(name),
                scenario_root / "wheel_summary" / name,
                f"manual {scenario} {name}",
            )
        visual_bindings = artifacts.get("visual")
        if not isinstance(visual_bindings, Mapping):
            raise CurationError(f"manual {scenario} visual bindings are missing")
        for field, relative in (
            ("manifest", "capture_manifest.json"),
            ("png", "representative_contact_sheet.png"),
            ("gif", "representative_motion.gif"),
        ):
            _assert_file_bound(
                visual_bindings.get(field),
                scenario_root / "visual" / relative,
                f"manual {scenario} visual {field}",
            )
        scenario_destination = destination / "scenarios" / scenario
        for source_name, destination_name in MANUAL_SCENARIO_FILES:
            _copy_file(
                scenario_root / source_name,
                scenario_destination / destination_name,
            )
        _write_sha256s(scenario_destination / "visual", recursive=False)
        _write_sha256s(scenario_destination / "wheel_summary", recursive=False)
        scenario_records.append(
            {
                "scenario": scenario,
                "manifest_sha256": _sha256(scenario_root / "scenario_manifest.json"),
            }
        )
    manual_manifest = {
        "schema": SCHEMA,
        "status": "PASS",
        "kind": "manual_4ws",
        "scenarios": scenario_records,
        "excluded_classes": list(EXCLUDED_CLASSES),
    }
    _write_json(destination / "curation_manifest.json", manual_manifest)
    _write_text(destination / "README.md", _manual_readme())
    sums = _write_sha256s(destination, recursive=True)
    return {
        "status": "PASS",
        "scenarios": list(MANUAL_SCENARIOS),
        "path": "manual_4ws",
        "sha256sums_sha256": _sha256(sums),
    }


def _root_readme(records: Mapping[str, Any], manual: Mapping[str, Any] | None) -> str:
    rows = []
    for name in BUNDLE_CONTRACTS:
        if name not in records:
            continue
        aggregate = records[name]["aggregate"]
        rows.append(
            f"| {BUNDLE_CONTRACTS[name]['title']} | 13/13 PASS | "
            f"{aggregate['total_elapsed_s']} s | {aggregate['total_odom_distance_m']} m | "
            f"[{name}/]({name}/) |"
        )
    if manual is not None:
        rows.append(
            "| Visible Robot UI manual 4WS | 4/4 PASS | - | - | "
            "[manual_4ws/](manual_4ws/) |"
        )
    return """# CAMROD x CARLA final evidence index

이 디렉터리는 최종 acceptance를 통과한 실제 CARLA 주행 자료만 허용 목록으로 선별한
인덱스다. 모든 mission bundle은 B1~B13 strict validation과 원본 artifact hash를
대조했고, matrix symlink는 저장소 안의 일반 파일로 복사했다.

| 시나리오 | 결과 | 전체 시간 | 전체 거리 | 자료 |
|---|---:|---:|---:|---|
""" + "\n".join(rows) + """

각 사이트/수동 시나리오 폴더에는 실제 화면 `PNG`, 실제 motion `GIF`, 4개 wheel 측정
요약이 있다. raw JSONL, MP4, 로그는 제외했으며 `curation_manifest.json`이 선별 규칙과
provenance를 기록한다.

```bash
sha256sum -c SHA256SUMS
```
"""


def _safe_output_root(output_root: Path, inputs: Sequence[Path], replace: bool) -> Path:
    expanded_output = output_root.expanduser()
    if expanded_output.is_symlink():
        raise CurationError(f"output root must not be a symlink: {expanded_output}")
    output = expanded_output.resolve(strict=False)
    if output == Path("/"):
        raise CurationError("output root must not be /")
    if output.exists() and not output.is_dir():
        raise CurationError(f"output root exists but is not a directory: {output}")
    if output.exists() and any(output.iterdir()) and not replace:
        raise CurationError(
            f"output root is non-empty; pass --replace-nonempty explicitly: {output}"
        )
    for raw_input in inputs:
        source = raw_input.expanduser().resolve(strict=True)
        if output == source or output in source.parents or source in output.parents:
            raise CurationError(f"output and input trees must not overlap: {output} / {source}")
    return output


def _install_staging(staging: Path, output: Path, replace: bool) -> None:
    if not output.exists():
        os.replace(staging, output)
        return
    if not any(output.iterdir()):
        output.rmdir()
        os.replace(staging, output)
        return
    if not replace:
        raise CurationError(f"refusing to replace non-empty output: {output}")
    backup = Path(tempfile.mkdtemp(prefix=f".{output.name}.backup-", dir=output.parent))
    backup.rmdir()
    os.replace(output, backup)
    try:
        os.replace(staging, output)
    except BaseException:
        os.replace(backup, output)
        raise
    shutil.rmtree(backup)


def curate(
    output_root: Path,
    *,
    site_sources: Mapping[str, Sequence[Path]],
    strict_sources: Mapping[str, Sequence[Path]],
    manual_source: Path | None = None,
    replace_nonempty: bool = False,
) -> Mapping[str, Any]:
    unknown = (set(site_sources) | set(strict_sources)) - set(BUNDLE_CONTRACTS)
    if unknown:
        raise CurationError(f"unknown site bundle names: {sorted(unknown)}")
    if set(site_sources) != set(strict_sources):
        raise CurationError("every site bundle needs matching --site-source and --strict-source")
    if not site_sources and manual_source is None:
        raise CurationError("at least one site bundle or --manual-source is required")
    empty = [name for name, roots in site_sources.items() if not roots]
    empty += [name for name, roots in strict_sources.items() if not roots]
    if empty:
        raise CurationError(f"bundle inputs must not be empty: {sorted(set(empty))}")

    inputs = [path for roots in site_sources.values() for path in roots]
    inputs += [path for roots in strict_sources.values() for path in roots]
    if manual_source is not None:
        inputs.append(manual_source)
    output = _safe_output_root(output_root, inputs, replace_nonempty)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{output.name}.staging-", dir=output.parent))
    try:
        bundle_records: dict[str, Any] = {}
        for bundle in BUNDLE_CONTRACTS:
            if bundle not in site_sources:
                continue
            bundle_records[bundle] = _curate_site_bundle(
                bundle,
                site_sources[bundle],
                strict_sources[bundle],
                staging / bundle,
            )
        manual_record = None
        if manual_source is not None:
            manual_record = _curate_manual(manual_source, staging / "manual_4ws")
        root_manifest = {
            "schema": SCHEMA,
            "status": "PASS",
            "site_bundles": bundle_records,
            "manual_4ws": manual_record,
            "excluded_classes": list(EXCLUDED_CLASSES),
        }
        _write_json(staging / "curation_manifest.json", root_manifest)
        _write_text(staging / "README.md", _root_readme(bundle_records, manual_record))
        _write_sha256s(staging, recursive=True)
        _install_staging(staging, output, replace_nonempty)
        return root_manifest
    except BaseException:
        if staging.exists():
            shutil.rmtree(staging)
        raise


def _named_path(raw: str, option: str) -> tuple[str, Path]:
    try:
        name, path = raw.split("=", 1)
    except ValueError:
        raise CurationError(f"{option} must be BUNDLE=/absolute/path: {raw!r}") from None
    if name not in BUNDLE_CONTRACTS:
        raise CurationError(
            f"{option} bundle must be one of {sorted(BUNDLE_CONTRACTS)}: {name!r}"
        )
    candidate = Path(path).expanduser()
    if not candidate.is_absolute():
        raise CurationError(f"{option} path must be absolute: {path!r}")
    return name, candidate


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", required=True, type=Path)
    parser.add_argument(
        "--site-source",
        action="append",
        default=[],
        metavar="BUNDLE=/ABS/PATH",
        help="completed site runner root; repeat to combine split roots",
    )
    parser.add_argument(
        "--strict-source",
        action="append",
        default=[],
        metavar="BUNDLE=/ABS/PATH",
        help="strict validator output; repeat to combine split validation",
    )
    parser.add_argument("--manual-source", type=Path)
    parser.add_argument(
        "--replace-nonempty",
        action="store_true",
        help="explicitly replace an existing non-empty output after staging succeeds",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    site_sources: dict[str, list[Path]] = {}
    strict_sources: dict[str, list[Path]] = {}
    try:
        for raw in args.site_source:
            name, path = _named_path(raw, "--site-source")
            site_sources.setdefault(name, []).append(path)
        for raw in args.strict_source:
            name, path = _named_path(raw, "--strict-source")
            strict_sources.setdefault(name, []).append(path)
        manifest = curate(
            args.output_root,
            site_sources=site_sources,
            strict_sources=strict_sources,
            manual_source=args.manual_source,
            replace_nonempty=args.replace_nonempty,
        )
    except (CurationError, FileNotFoundError, OSError) as error:
        print(f"[curate-evidence] ERROR: {error}", file=sys.stderr)
        return 2
    bundle_names = ", ".join(manifest["site_bundles"]) or "none"
    manual = "yes" if manifest.get("manual_4ws") else "no"
    print(
        f"[curate-evidence] PASS: {args.output_root} "
        f"site_bundles={bundle_names} manual_4ws={manual}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
