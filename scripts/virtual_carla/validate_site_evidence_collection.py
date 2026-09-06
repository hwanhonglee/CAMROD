#!/usr/bin/env python3
# flake8: noqa
"""
Fail-closed validation for a completed site-evidence runner collection.

This command is deliberately offline.  It reads a completed output directory
from ``run_site_evidence_matrix.sh``, validates the native matrix, wheel, and
visual evidence behind every selected site, and atomically installs a compact
JSON/CSV/Markdown validation report in a separate new or empty directory.
Nothing below imports ROS or CARLA, and nothing is written below the input
root.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence


COLLECTION_SCHEMA = "camrod.virtual_carla.site_evidence_collection_validation.v1"
RUN_SCHEMA = "camrod.virtual_carla.site_evidence_matrix.v1"
SITE_SCHEMA = "camrod.virtual_carla.site_evidence.v1"
MATRIX_SCHEMA = "camrod.virtual_carla.camping_site_matrix.v1"
METRICS_SCHEMA = "camrod.virtual_carla.camping_site_metrics_summary.v1"
WHEEL_MANIFEST_SCHEMA = (
    "camrod.virtual_carla.physical_wheel_telemetry_manifest.v1"
)
WHEEL_SUMMARY_SCHEMA = "camrod.virtual_carla.physical_wheel_summary.v1"
CAPTURE_SCHEMA = "camrod.virtual_carla.desktop_ui_capture.v4"
RUNTIME_AUDIT_SCHEMA = "camrod.virtual_carla.runtime_profile_audit.v1"
CURRENT_RUNTIME_PROFILE = "develop-plus-carla-site-geometry-v27"
CURRENT_RUNTIME_LAUNCH = "camrod_carla_develop_site_geometry.launch.py"
EXPECTED_TYPE_ID = "vehicle.ranger.default"
EXPECTED_ROLE_NAME = "ego_vehicle"
EXPECTED_BACKEND = "PHYSX_FOUR_WHEEL_STEERING"
EXPECTED_API_VERSION = "carla.physx_four_wheel_steer_torque.v2"
CANONICAL_WHEELS = ("FL", "FR", "RL", "RR")
ALL_SITES = tuple(f"B{number}" for number in range(1, 14))
SITE_RE = re.compile(r"^B([1-9]|1[0-3])$")
SITE_DIR_RE = re.compile(r"^B[0-9]+$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
GIT_OBJECT_RE = re.compile(r"^(?:[0-9a-f]{40}|[0-9a-f]{64})$")
UI_RETURN_TOKEN_RE = re.compile(r"^g[1-9][0-9]*-s[1-9][0-9]*-[0-9a-f]+$")
GUEST_RETURN_SOURCE_RE = re.compile(
    r"^guest:usage_complete:site=(B(?:[1-9]|1[0-3])):g=([1-9][0-9]*)$"
)
OUTPUT_FILES = (
    "site_evidence_collection.json",
    "site_evidence_collection.csv",
    "site_evidence_collection.md",
)
CSV_COLUMNS = (
    "site",
    "status",
    "authority",
    "mission_intent",
    "actor_id",
    "actor_type_id",
    "motion_backend",
    "elapsed_s",
    "outbound_duration_s",
    "return_duration_s",
    "total_odom_distance_m",
    "outbound_distance_m",
    "return_distance_m",
    "wheel_planar_distance_m",
    "drop_zone_error_m",
    "final_speed_mps",
    "collision_publisher_count",
    "collision_event_count",
    "parking_confirmed",
    "charging_confirmed",
    "wheel_sample_count",
    "matrix_sha256",
    "wheel_jsonl_sha256",
    "png_sha256",
    "gif_sha256",
    "site_manifest_sha256",
)


class CollectionValidationError(RuntimeError):
    """The collection, an artifact, or an output destination is invalid."""


def _load_runtime_contract_module() -> Any:
    """Load the producer contract without importing ROS or CARLA."""
    contract_path = Path(__file__).with_name("audit_runtime_profile.py")
    spec = importlib.util.spec_from_file_location(
        "_camrod_site_collection_runtime_contract", contract_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load runtime-profile contract: {contract_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_RUNTIME_CONTRACT = _load_runtime_contract_module()


def _load_current_runtime_profile_parameters() -> dict[str, dict[str, Any]]:
    """Load the exact current signature from the audit that produced it."""
    profiles = getattr(_RUNTIME_CONTRACT, "AUDITED_PROFILE_PARAMETERS", None)
    if not isinstance(profiles, Mapping):
        raise RuntimeError("runtime-profile producer has no profile mapping")
    selected = profiles.get(CURRENT_RUNTIME_PROFILE)
    if not isinstance(selected, Mapping):
        raise RuntimeError(
            f"runtime-profile producer has no {CURRENT_RUNTIME_PROFILE!r} contract"
        )
    return {
        str(node): dict(parameters)
        for node, parameters in selected.items()
    }


CURRENT_RUNTIME_PROFILE_PARAMETERS = _load_current_runtime_profile_parameters()
CURRENT_CARLA_MAP = str(_RUNTIME_CONTRACT.SITE_GEOMETRY_CURRENT_CARLA_MAP)
EXPECTED_SOURCE_BRANCH = str(_RUNTIME_CONTRACT.EXPECTED_SOURCE_BRANCH)
PROTECTED_SOURCE_RELATIVE_PATH = str(
    _RUNTIME_CONTRACT.PROTECTED_SOURCE_RELATIVE_PATH
)
RUNTIME_SOURCE_PATHS = tuple(_RUNTIME_CONTRACT.RUNTIME_SOURCE_PATHS)
RUNTIME_INSTALL_PREFIXES = tuple(_RUNTIME_CONTRACT.RUNTIME_INSTALL_PREFIXES)
CANONICAL_UI_BUILD_ENV = dict(_RUNTIME_CONTRACT.CANONICAL_UI_BUILD_ENV)
SOFTWARE_IDENTITY_SCHEMA = "camrod.virtual_carla.software_identity.v1"
LANELET_PARAMETER_NODES = (
    "/map/lanelet_map_provider",
    "/map/lanelet_boundary_cost_grid",
    "/map/lanelet_safety_cost_grid",
    "/planning/goal_snapper",
)


def _reject_duplicate_keys(pairs: Iterable[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise CollectionValidationError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise CollectionValidationError(f"{label} must be a JSON object")
    return value


def _list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise CollectionValidationError(f"{label} must be a JSON array")
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise CollectionValidationError(f"{label} must be a string")
    return value


def _exact_integer(value: Any, label: str, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CollectionValidationError(f"{label} must be an exact integer")
    if minimum is not None and value < minimum:
        raise CollectionValidationError(f"{label} must be >= {minimum}")
    return value


def _boolean(value: Any, label: str) -> bool:
    if type(value) is not bool:
        raise CollectionValidationError(f"{label} must be a JSON boolean")
    return value


def _finite(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CollectionValidationError(f"{label} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise CollectionValidationError(f"{label} must be a finite number")
    if minimum is not None and number < minimum:
        raise CollectionValidationError(f"{label} must be >= {minimum}")
    return number


def _positive(value: Any, label: str) -> float:
    number = _finite(value, label, minimum=0.0)
    if number <= 0.0:
        raise CollectionValidationError(f"{label} must be > 0")
    return number


def _sha256(value: Any, label: str) -> str:
    digest = _string(value, label)
    if not SHA256_RE.fullmatch(digest):
        raise CollectionValidationError(
            f"{label} must be a lowercase 64-character SHA-256"
        )
    return digest


def _git_object_id(value: Any, label: str) -> str:
    object_id = _string(value, label)
    if not GIT_OBJECT_RE.fullmatch(object_id):
        raise CollectionValidationError(
            f"{label} must be a lowercase full Git object id"
        )
    return object_id


def _same_number(left: float, right: float, label: str) -> None:
    tolerance = max(1e-6, max(abs(left), abs(right)) * 1e-8)
    if abs(left - right) > tolerance:
        raise CollectionValidationError(
            f"{label} mismatch: {left!r} != {right!r}"
        )


def _split_sum(total: float, first: float, second: float, label: str) -> None:
    tolerance = max(0.02, total * 1e-6)
    if abs(total - first - second) > tolerance:
        raise CollectionValidationError(
            f"{label} is inconsistent: total={total}, parts={first}+{second}"
        )


def _assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise CollectionValidationError(
            f"{label} mismatch: expected {expected!r}, got {actual!r}"
        )


def _assert_exact_tree(actual: Any, expected: Any, label: str) -> None:
    """Compare an audit signature recursively, including scalar JSON types."""
    if type(actual) is not type(expected):
        raise CollectionValidationError(
            f"{label} type mismatch: expected {type(expected).__name__}, "
            f"got {type(actual).__name__}"
        )
    if isinstance(expected, dict):
        if set(actual) != set(expected):
            missing = sorted(set(expected) - set(actual))
            extra = sorted(set(actual) - set(expected))
            raise CollectionValidationError(
                f"{label} keys mismatch: missing={missing}, extra={extra}"
            )
        for key, expected_value in expected.items():
            _assert_exact_tree(actual[key], expected_value, f"{label}.{key}")
        return
    if isinstance(expected, list):
        if len(actual) != len(expected):
            raise CollectionValidationError(
                f"{label} length mismatch: expected {len(expected)}, got {len(actual)}"
            )
        for index, (actual_value, expected_value) in enumerate(zip(actual, expected)):
            _assert_exact_tree(
                actual_value, expected_value, f"{label}[{index}]"
            )
        return
    if actual != expected:
        raise CollectionValidationError(
            f"{label} mismatch: expected {expected!r}, got {actual!r}"
        )


def _assert_authoritative_profile_signature(
    actual: Mapping[str, Any], expected: Mapping[str, Any], label: str
) -> None:
    """Require every producer-owned profile field, allowing audited live bindings.

    The producer adds lanelet paths, CARLA endpoint values, and the physical
    bridge binding at audit time.  Those are validated separately against the
    collection identity; this comparison covers every static v27 field without
    maintaining a weaker copied subset.
    """
    missing_nodes = sorted(set(expected) - set(actual))
    if missing_nodes:
        raise CollectionValidationError(
            f"{label} keys mismatch: missing={missing_nodes}"
        )
    for node, expected_parameters in expected.items():
        actual_parameters = _mapping(actual.get(node), f"{label}.{node}")
        missing_parameters = sorted(
            set(expected_parameters) - set(actual_parameters)
        )
        if missing_parameters:
            raise CollectionValidationError(
                f"{label}.{node} keys mismatch: missing={missing_parameters}"
            )
        for name, expected_value in expected_parameters.items():
            _assert_exact_tree(
                actual_parameters[name],
                expected_value,
                f"{label}.{node}.{name}",
            )


def _absolute_path(value: Any, label: str) -> Path:
    raw = _string(value, label)
    path = Path(raw)
    if not path.is_absolute():
        raise CollectionValidationError(f"{label} must be an absolute path: {raw}")
    return path


def _stable_file_fact(
    path: Path,
    label: str,
    *,
    allow_symlink: bool = False,
) -> dict[str, Any]:
    try:
        link_stat = path.lstat()
    except OSError as error:
        raise CollectionValidationError(f"missing {label}: {path}: {error}") from None
    if stat.S_ISLNK(link_stat.st_mode) and not allow_symlink:
        raise CollectionValidationError(f"{label} must not be a symlink: {path}")
    try:
        before = path.stat()
    except OSError as error:
        raise CollectionValidationError(f"cannot stat {label}: {path}: {error}") from None
    if not stat.S_ISREG(before.st_mode):
        raise CollectionValidationError(f"{label} is not a regular file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        after = path.stat()
    except OSError as error:
        raise CollectionValidationError(f"cannot read {label}: {path}: {error}") from None
    identity_before = (
        before.st_dev,
        before.st_ino,
        before.st_size,
        before.st_mtime_ns,
    )
    identity_after = (
        after.st_dev,
        after.st_ino,
        after.st_size,
        after.st_mtime_ns,
    )
    if identity_before != identity_after:
        raise CollectionValidationError(f"{label} changed while it was read: {path}")
    return {
        "path": str(path.resolve(strict=True)),
        "bytes": before.st_size,
        "sha256": digest.hexdigest(),
    }


def _canonical_tree_digest(entries: Sequence[Mapping[str, Any]]) -> str:
    digest = hashlib.sha256()
    for entry in entries:
        digest.update(
            json.dumps(
                entry,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8", "surrogateescape")
        )
        digest.update(b"\n")
    return digest.hexdigest()


def _git_output(source_root: Path, *arguments: str) -> str:
    try:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=source_root,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="surrogateescape",
            timeout=20.0,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise CollectionValidationError(
            f"cannot independently inspect runtime source identity: {error}"
        ) from None
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise CollectionValidationError(
            "runtime source identity command failed "
            f"({' '.join(arguments)}): {detail}"
        )
    return completed.stdout


def _runtime_git_pathspecs() -> list[str]:
    pathspecs = [f":(top,literal){path}" for path in RUNTIME_SOURCE_PATHS]
    pathspecs.append(
        f":(top,exclude,literal){PROTECTED_SOURCE_RELATIVE_PATH}"
    )
    return pathspecs


def _porcelain_dirty_paths(raw: str) -> list[str]:
    tokens = [token for token in raw.split("\0") if token]
    paths: list[str] = []
    index = 0
    while index < len(tokens):
        token = tokens[index]
        status_code = token[:2] if len(token) >= 3 and token[2] == " " else ""
        path = token[3:] if status_code else token
        if path != PROTECTED_SOURCE_RELATIVE_PATH:
            paths.append(path)
        if status_code and any(code in status_code for code in ("R", "C")):
            index += 1
            if index < len(tokens):
                original = tokens[index]
                if original != PROTECTED_SOURCE_RELATIVE_PATH:
                    paths.append(original)
        index += 1
    return sorted(set(paths))


def _source_tree_fingerprint(source_root: Path) -> dict[str, Any]:
    tracked_raw = _git_output(
        source_root, "ls-files", "-z", "--", *_runtime_git_pathspecs()
    )
    relative_paths = sorted(path for path in tracked_raw.split("\0") if path)
    if not relative_paths:
        raise CollectionValidationError(
            "runtime source fingerprint contains no tracked files"
        )
    if PROTECTED_SOURCE_RELATIVE_PATH in relative_paths:
        raise CollectionValidationError(
            "protected Vanjee config entered the runtime source fingerprint"
        )
    entries: list[dict[str, Any]] = []
    for relative in relative_paths:
        path = source_root / relative
        try:
            metadata = path.lstat()
        except OSError as error:
            raise CollectionValidationError(
                f"tracked runtime source is unavailable: {relative}: {error}"
            ) from None
        if stat.S_ISLNK(metadata.st_mode):
            try:
                link_target = os.readlink(path)
            except OSError as error:
                raise CollectionValidationError(
                    f"cannot read runtime source symlink {relative}: {error}"
                ) from None
            entries.append(
                {
                    "kind": "symlink",
                    "link_target": link_target,
                    "mode": "120000",
                    "path": relative,
                }
            )
        elif stat.S_ISREG(metadata.st_mode):
            fact = _stable_file_fact(path, f"runtime source {relative}")
            entries.append(
                {
                    "content_sha256": fact["sha256"],
                    "kind": "file",
                    "mode": "100755" if metadata.st_mode & stat.S_IXUSR else "100644",
                    "path": relative,
                }
            )
        else:
            raise CollectionValidationError(
                f"unsupported tracked runtime source type: {relative}"
            )
    return {
        "algorithm": "sha256-canonical-path-kind-mode-content-v1",
        "sha256": _canonical_tree_digest(entries),
        "tracked_entry_count": len(entries),
        "roots": list(RUNTIME_SOURCE_PATHS),
    }


def _source_identity(source_root: Path) -> dict[str, Any]:
    try:
        source_root = source_root.resolve(strict=True)
    except OSError as error:
        raise CollectionValidationError(
            f"cannot resolve runtime source root {source_root}: {error}"
        ) from None
    if not source_root.is_dir():
        raise CollectionValidationError(
            f"runtime source root is not a directory: {source_root}"
        )
    git_root = Path(
        _git_output(source_root, "rev-parse", "--show-toplevel").strip()
    ).resolve()
    if git_root != source_root:
        raise CollectionValidationError(
            f"runtime source root is not the Git top level: {source_root} != {git_root}"
        )
    branch = _git_output(
        source_root, "symbolic-ref", "--quiet", "--short", "HEAD"
    ).strip()
    if branch != EXPECTED_SOURCE_BRANCH:
        raise CollectionValidationError(
            f"current v27 source branch must be {EXPECTED_SOURCE_BRANCH!r}, got {branch!r}"
        )
    head_commit = _git_output(source_root, "rev-parse", "HEAD").strip()
    origin_develop_commit = _git_output(
        source_root, "rev-parse", "refs/remotes/origin/develop"
    ).strip()
    merge_base = _git_output(
        source_root, "merge-base", "HEAD", "refs/remotes/origin/develop"
    ).strip()
    counts = _git_output(
        source_root,
        "rev-list",
        "--left-right",
        "--count",
        "HEAD...refs/remotes/origin/develop",
    ).split()
    if len(counts) != 2:
        raise CollectionValidationError(
            f"malformed runtime source ahead/behind result: {counts!r}"
        )
    try:
        ahead, behind = (int(value) for value in counts)
    except ValueError:
        raise CollectionValidationError(
            f"malformed runtime source ahead/behind result: {counts!r}"
        ) from None
    if behind != 0 or merge_base != origin_develop_commit:
        raise CollectionValidationError(
            "runtime source no longer contains the exact origin/develop tip"
        )
    dirty_raw = _git_output(
        source_root,
        "status",
        "--porcelain=v1",
        "-z",
        "--untracked-files=all",
        "--",
        *_runtime_git_pathspecs(),
    )
    dirty_paths = _porcelain_dirty_paths(dirty_raw)
    if dirty_paths:
        raise CollectionValidationError(
            "runtime source is dirty outside the explicit Vanjee exclusion: "
            + ", ".join(dirty_paths[:20])
        )
    return {
        "path": str(source_root),
        "branch": branch,
        "head_commit": head_commit,
        "origin_develop_commit": origin_develop_commit,
        "merge_base": merge_base,
        "ahead_of_origin_develop": ahead,
        "behind_origin_develop": behind,
        "runtime_worktree_clean": True,
        "runtime_tree": _source_tree_fingerprint(source_root),
        "explicit_exclusions": [
            {
                "path": PROTECTED_SOURCE_RELATIVE_PATH,
                "content_hashed": False,
                "dirty_state_checked": False,
                "reason": "protected user-owned generated Vanjee CMake config",
            }
        ],
    }


def _install_cache_path(relative: Path) -> bool:
    return "__pycache__" in relative.parts or relative.suffix in (".pyc", ".pyo")


def _installed_entry(path: Path, relative: Path) -> dict[str, Any]:
    try:
        metadata = path.lstat()
    except OSError as error:
        raise CollectionValidationError(
            f"cannot inspect installed entry {path}: {error}"
        ) from None
    if stat.S_ISREG(metadata.st_mode):
        fact = _stable_file_fact(path, f"installed entry {relative}")
        return {
            "content_sha256": fact["sha256"],
            "executable": bool(metadata.st_mode & stat.S_IXUSR),
            "kind": "file",
            "path": relative.as_posix(),
        }
    if not stat.S_ISLNK(metadata.st_mode):
        raise CollectionValidationError(f"unsupported installed entry type: {path}")
    try:
        link_target = os.readlink(path)
        resolved_target = path.resolve(strict=True)
        target_metadata = resolved_target.stat()
    except (OSError, RuntimeError) as error:
        raise CollectionValidationError(
            f"broken installed symlink {path}: {error}"
        ) from None
    entry: dict[str, Any] = {
        "kind": "symlink",
        "link_target": link_target,
        "path": relative.as_posix(),
        "resolved_target": str(resolved_target),
    }
    if stat.S_ISREG(target_metadata.st_mode):
        fact = _stable_file_fact(resolved_target, f"installed target {relative}")
        entry.update(
            {
                "target_content_sha256": fact["sha256"],
                "target_executable": bool(target_metadata.st_mode & stat.S_IXUSR),
                "target_kind": "file",
            }
        )
    elif stat.S_ISDIR(target_metadata.st_mode):
        entry["target_kind"] = "directory"
    else:
        raise CollectionValidationError(
            f"unsupported installed symlink target: {path}"
        )
    return entry


def _install_prefix_fingerprint(prefix: Path) -> dict[str, Any]:
    prefix = prefix.expanduser().absolute()
    if not prefix.is_dir():
        raise CollectionValidationError(f"required install prefix is missing: {prefix}")
    entries: list[dict[str, Any]] = []

    def walk_error(error: OSError) -> None:
        raise CollectionValidationError(
            f"cannot enumerate install prefix {prefix}: {error}"
        )

    for directory, dirnames, filenames in os.walk(
        prefix, topdown=True, followlinks=False, onerror=walk_error
    ):
        directory_path = Path(directory)
        kept_directories: list[str] = []
        for name in sorted(dirnames):
            path = directory_path / name
            relative = path.relative_to(prefix)
            if _install_cache_path(relative):
                continue
            if path.is_symlink():
                entries.append(_installed_entry(path, relative))
            else:
                kept_directories.append(name)
        dirnames[:] = kept_directories
        for name in sorted(filenames):
            path = directory_path / name
            relative = path.relative_to(prefix)
            if _install_cache_path(relative):
                continue
            entries.append(_installed_entry(path, relative))
    entries.sort(key=lambda entry: entry["path"])
    if not entries:
        raise CollectionValidationError(
            f"install prefix contains no runtime entries: {prefix}"
        )
    return {
        "path": str(prefix),
        "algorithm": "sha256-canonical-install-entry-v1",
        "sha256": _canonical_tree_digest(entries),
        "entry_count": len(entries),
        "regular_file_count": sum(entry["kind"] == "file" for entry in entries),
        "symlink_count": sum(entry["kind"] == "symlink" for entry in entries),
        "cache_policy": "exclude __pycache__ directories and .pyc/.pyo files",
    }


def _install_identity(install_root: Path) -> dict[str, Any]:
    install_root = install_root.expanduser().absolute()
    if not install_root.is_dir():
        raise CollectionValidationError(
            f"runtime install root is missing: {install_root}"
        )
    prefixes: dict[str, dict[str, Any]] = {}
    combined: list[dict[str, str]] = []
    for package in RUNTIME_INSTALL_PREFIXES:
        fingerprint = _install_prefix_fingerprint(install_root / package)
        prefixes[package] = fingerprint
        combined.append({"package": package, "sha256": fingerprint["sha256"]})
    return {
        "path": str(install_root),
        "algorithm": "sha256-canonical-package-fingerprint-v1",
        "sha256": _canonical_tree_digest(combined),
        "prefixes": prefixes,
    }


def _frontend_source_input_fingerprint(frontend_root: Path) -> str:
    dotenv_files = sorted(path.name for path in frontend_root.glob(".env*"))
    if dotenv_files:
        raise CollectionValidationError(
            "CRA .env files are forbidden in the canonical UI build: "
            + ", ".join(dotenv_files)
        )
    build_environment, _ = _load_json(
        frontend_root / "camrod-build-env.json", "UI source build environment"
    )
    _assert_exact_tree(
        build_environment, CANONICAL_UI_BUILD_ENV, "UI source build environment"
    )
    relative_files = [
        Path("package.json"),
        Path("package-lock.json"),
        Path("camrod-build-env.json"),
    ]
    for directory_name in ("src", "public"):
        directory = frontend_root / directory_name
        if not directory.is_dir():
            raise CollectionValidationError(
                f"UI source input directory is missing: {directory}"
            )
        relative_files.extend(
            path.relative_to(frontend_root)
            for path in directory.rglob("*")
            if path.is_file() and not path.is_symlink()
        )
    relative_files = sorted(set(relative_files), key=lambda path: path.as_posix())
    digest = hashlib.sha256()
    for relative in relative_files:
        path = frontend_root / relative
        if not path.is_file() or path.is_symlink():
            raise CollectionValidationError(f"UI source input is missing: {path}")
        fact = _stable_file_fact(path, f"UI source input {relative}")
        digest.update(os.fsencode(relative.as_posix()))
        digest.update(b"\0")
        digest.update(fact["sha256"].encode("ascii"))
        digest.update(b"\\0")
    return digest.hexdigest()


def _read_sha_stamp(path: Path, label: str) -> tuple[str, dict[str, Any]]:
    fact = _stable_file_fact(path, label)
    try:
        value = path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as error:
        raise CollectionValidationError(f"cannot read {label}: {error}") from None
    return _sha256(value, f"{label}.value"), fact


def _ui_main_bundle(build_root: Path, label: str) -> tuple[Path, Path]:
    index = build_root / "index.html"
    fact = _stable_file_fact(index, f"{label}.index")
    try:
        index_text = index.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise CollectionValidationError(f"cannot read {label}.index: {error}") from None
    pattern = _RUNTIME_CONTRACT.UI_MAIN_BUNDLE_RE
    matches = sorted(set(pattern.findall(index_text)))
    if len(matches) != 1:
        raise CollectionValidationError(
            f"{label}.index must reference exactly one hashed main bundle"
        )
    bundle = build_root / matches[0]
    _stable_file_fact(bundle, f"{label}.main bundle")
    return index, bundle


def _ui_build_identity(source_root: Path, install_root: Path) -> dict[str, Any]:
    frontend_root = (
        source_root / "camrod_ui" / "camrod_ui_robot" / "assets" / "frontend"
    )
    source_build = frontend_root / "build"
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
    current_inputs = _frontend_source_input_fingerprint(frontend_root)
    source_env, _ = _load_json(
        source_build / ".camrod-build-env.json", "UI source build environment evidence"
    )
    installed_env, installed_env_fact = _load_json(
        installed_build / ".camrod-build-env.json",
        "UI installed build environment evidence",
    )
    _assert_exact_tree(source_env, CANONICAL_UI_BUILD_ENV, "UI source build environment evidence")
    _assert_exact_tree(
        installed_env,
        CANONICAL_UI_BUILD_ENV,
        "UI installed build environment evidence",
    )
    source_stamp, _ = _read_sha_stamp(
        source_build / ".camrod-inputs.sha256", "UI source input stamp"
    )
    installed_stamp, installed_stamp_fact = _read_sha_stamp(
        installed_build / ".camrod-inputs.sha256", "UI installed input stamp"
    )
    if source_stamp != current_inputs or installed_stamp != current_inputs:
        raise CollectionValidationError(
            "UI source/install input stamp does not match current source inputs"
        )
    source_index, source_bundle = _ui_main_bundle(source_build, "UI source build")
    installed_index, installed_bundle = _ui_main_bundle(
        installed_build, "UI installed build"
    )
    if source_bundle.relative_to(source_build) != installed_bundle.relative_to(
        installed_build
    ):
        raise CollectionValidationError("UI source/install main bundle names differ")
    source_index_fact = _stable_file_fact(source_index, "UI source index")
    installed_index_fact = _stable_file_fact(installed_index, "UI installed index")
    source_bundle_fact = _stable_file_fact(source_bundle, "UI source main bundle")
    installed_bundle_fact = _stable_file_fact(
        installed_bundle, "UI installed main bundle"
    )
    if source_index_fact["sha256"] != installed_index_fact["sha256"]:
        raise CollectionValidationError("UI source/install index content differs")
    if source_bundle_fact["sha256"] != installed_bundle_fact["sha256"]:
        raise CollectionValidationError("UI source/install main bundle content differs")
    return {
        "source_inputs_sha256": current_inputs,
        "canonical_build_environment": installed_env,
        "installed_build_environment": {
            "path": str((installed_build / ".camrod-build-env.json").absolute()),
            "sha256": installed_env_fact["sha256"],
        },
        "installed_inputs_stamp": {
            "path": str((installed_build / ".camrod-inputs.sha256").absolute()),
            "file_sha256": installed_stamp_fact["sha256"],
            "value": installed_stamp,
        },
        "installed_index": {
            "path": str(installed_index.absolute()),
            "sha256": installed_index_fact["sha256"],
        },
        "installed_main_bundle": {
            "path": str(installed_bundle.absolute()),
            "sha256": installed_bundle_fact["sha256"],
        },
        "source_install_match": True,
    }


def _validate_software_identity(
    value: Any,
    label: str,
    cache: dict[str, dict[str, str]],
) -> dict[str, str]:
    identity = _mapping(value, label)
    _assert_equal(
        set(identity),
        {"schema", "binding_sha256", "source", "install", "ui_build"},
        f"{label}.keys",
    )
    cache_key = hashlib.sha256(
        json.dumps(
            identity, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        ).encode("utf-8", "surrogateescape")
    ).hexdigest()
    if cache_key in cache:
        return dict(cache[cache_key])
    _assert_equal(identity.get("schema"), SOFTWARE_IDENTITY_SCHEMA, f"{label}.schema")
    recorded_source = _mapping(identity.get("source"), f"{label}.source")
    source_root = _absolute_path(recorded_source.get("path"), f"{label}.source.path")
    actual_source = _source_identity(source_root)
    _assert_exact_tree(recorded_source, actual_source, f"{label}.source")
    recorded_install = _mapping(identity.get("install"), f"{label}.install")
    install_root = _absolute_path(recorded_install.get("path"), f"{label}.install.path")
    actual_install = _install_identity(install_root)
    _assert_exact_tree(recorded_install, actual_install, f"{label}.install")
    actual_ui_build = _ui_build_identity(source_root, install_root)
    recorded_ui_build = _mapping(identity.get("ui_build"), f"{label}.ui_build")
    _assert_exact_tree(recorded_ui_build, actual_ui_build, f"{label}.ui_build")
    binding = [
        {
            "head_commit": actual_source["head_commit"],
            "install_sha256": actual_install["sha256"],
            "runtime_source_sha256": actual_source["runtime_tree"]["sha256"],
            "ui_build_environment_sha256": actual_ui_build[
                "installed_build_environment"
            ]["sha256"],
            "ui_inputs_sha256": actual_ui_build["source_inputs_sha256"],
            "ui_main_bundle_sha256": actual_ui_build["installed_main_bundle"][
                "sha256"
            ],
        }
    ]
    expected_binding = _canonical_tree_digest(binding)
    _assert_equal(
        _sha256(identity.get("binding_sha256"), f"{label}.binding_sha256"),
        expected_binding,
        f"{label}.binding_sha256",
    )
    result = {
        "binding_sha256": expected_binding,
        "source_path": str(source_root.resolve()),
        "source_branch": actual_source["branch"],
        "source_head": actual_source["head_commit"],
        "install_sha256": actual_install["sha256"],
    }
    cache[cache_key] = result
    return dict(result)


def _normalize_carla_map(value: Any, label: str) -> str:
    result = _string(value, label).strip()
    if result.startswith("/Game/"):
        result = result[len("/Game/") :]
    return result.strip("/")


def _validate_runtime_world_identity(
    runtime_audit: Mapping[str, Any],
    parameters: Mapping[str, Any],
    label: str,
) -> dict[str, str]:
    carla = _mapping(runtime_audit.get("carla"), f"{label}.carla")
    _assert_equal(
        carla.get("expected_ue_map"),
        f"/Game/{CURRENT_CARLA_MAP}",
        f"{label}.carla.expected_ue_map",
    )
    _assert_equal(
        carla.get("expected_town"),
        CURRENT_CARLA_MAP,
        f"{label}.carla.expected_town",
    )
    for key in ("map_name", "normalized_map_name"):
        normalized = _normalize_carla_map(carla.get(key), f"{label}.carla.{key}")
        _assert_equal(normalized, CURRENT_CARLA_MAP, f"{label}.carla.{key}")
    carla_bridge = _mapping(
        parameters.get("/carla_ros_bridge"), f"{label}.parameters./carla_ros_bridge"
    )
    _assert_equal(
        carla_bridge.get("town"),
        CURRENT_CARLA_MAP,
        f"{label}.parameters./carla_ros_bridge.town",
    )

    umap = _mapping(carla.get("ue_map_asset"), f"{label}.carla.ue_map_asset")
    umap_path = _absolute_path(umap.get("path"), f"{label}.carla.ue_map_asset.path")
    expected_suffix = (
        Path("Unreal") / "CarlaUE4" / "Content" / f"{CURRENT_CARLA_MAP}.umap"
    ).as_posix()
    if not umap_path.as_posix().endswith("/" + expected_suffix):
        raise CollectionValidationError(
            f"{label}.carla.ue_map_asset.path is not the exact v15 UMAP identity: "
            f"{umap_path}"
        )
    umap_fact = _stable_file_fact(umap_path, f"{label}.carla.ue_map_asset")
    _assert_equal(
        _sha256(umap.get("sha256"), f"{label}.carla.ue_map_asset.sha256"),
        umap_fact["sha256"],
        f"{label}.carla.ue_map_asset.sha256",
    )

    lanelet = _mapping(runtime_audit.get("lanelet_map"), f"{label}.lanelet_map")
    lanelet_path = _absolute_path(lanelet.get("path"), f"{label}.lanelet_map.path")
    lanelet_fact = _stable_file_fact(lanelet_path, f"{label}.lanelet_map")
    _assert_equal(
        _sha256(lanelet.get("sha256"), f"{label}.lanelet_map.sha256"),
        lanelet_fact["sha256"],
        f"{label}.lanelet_map.sha256",
    )
    _assert_equal(
        lanelet.get("launch_argument_match"), True, f"{label}.lanelet_map.launch_match"
    )
    _assert_equal(
        lanelet.get("all_parameter_paths_match"),
        True,
        f"{label}.lanelet_map.parameter_match",
    )
    _assert_equal(
        lanelet.get("parameter_nodes"),
        list(LANELET_PARAMETER_NODES),
        f"{label}.lanelet_map.parameter_nodes",
    )
    for node in LANELET_PARAMETER_NODES:
        node_parameters = _mapping(parameters.get(node), f"{label}.parameters.{node}")
        selected_path = _absolute_path(
            node_parameters.get("map_path"), f"{label}.parameters.{node}.map_path"
        )
        _assert_equal(
            selected_path,
            lanelet_path,
            f"{label}.parameters.{node}.map_path",
        )

    launch = _mapping(runtime_audit.get("launch"), f"{label}.launch")
    _assert_equal(
        launch.get("launch_file"), CURRENT_RUNTIME_LAUNCH, f"{label}.launch.file"
    )
    _assert_equal(
        launch.get("profile"), CURRENT_RUNTIME_PROFILE, f"{label}.launch.profile"
    )
    launch_lanelet = _absolute_path(
        launch.get("lanelet_map_argument"), f"{label}.launch.lanelet_map_argument"
    )
    _assert_equal(
        launch_lanelet,
        lanelet_path,
        f"{label}.launch.lanelet_map_argument",
    )
    _exact_integer(launch.get("pid"), f"{label}.launch.pid", 1)
    raw_argv = _list(launch.get("argv"), f"{label}.launch.argv")
    if not raw_argv or any(not isinstance(token, str) for token in raw_argv):
        raise CollectionValidationError(f"{label}.launch.argv must contain strings")
    argv = [str(token) for token in raw_argv]
    launch_tokens = [
        token for token in argv if Path(token).name == CURRENT_RUNTIME_LAUNCH
    ]
    if len(launch_tokens) != 1 or "launch" not in argv or "camrod_carla_adapter" not in argv:
        raise CollectionValidationError(
            f"{label}.launch.argv is not the exact CAMROD v27 launch identity"
        )
    lanelet_tokens = [
        token.split(":=", 1)[1]
        for token in argv
        if token.startswith("camrod_map_path:=")
    ]
    if len(lanelet_tokens) != 1:
        raise CollectionValidationError(
            f"{label}.launch.argv must contain exactly one camrod_map_path"
        )
    argv_lanelet = Path(lanelet_tokens[0]).expanduser()
    _assert_equal(
        argv_lanelet,
        lanelet_path,
        f"{label}.launch.argv.camrod_map_path",
    )
    command_line = b"\0".join(
        token.encode("utf-8", "surrogateescape") for token in argv
    ) + b"\0"
    _assert_equal(
        _sha256(launch.get("cmdline_sha256"), f"{label}.launch.cmdline_sha256"),
        hashlib.sha256(command_line).hexdigest(),
        f"{label}.launch.cmdline_sha256",
    )

    profile_contract = _mapping(
        runtime_audit.get("profile_contract"), f"{label}.profile_contract"
    )
    _assert_equal(
        profile_contract.get("requested_profile"),
        CURRENT_RUNTIME_PROFILE,
        f"{label}.profile_contract.requested_profile",
    )
    _assert_equal(
        profile_contract.get("site_geometry_allowed_carla_maps"),
        [CURRENT_CARLA_MAP],
        f"{label}.profile_contract.allowed_maps",
    )
    return {
        "umap_sha256": umap_fact["sha256"],
        "lanelet_sha256": lanelet_fact["sha256"],
        "launch_cmdline_sha256": hashlib.sha256(command_line).hexdigest(),
    }


def _load_json(
    path: Path,
    label: str,
    *,
    allow_symlink: bool = False,
) -> tuple[Mapping[str, Any], dict[str, Any]]:
    fact = _stable_file_fact(path, label, allow_symlink=allow_symlink)
    try:
        payload = path.read_bytes()
        decoded = json.loads(
            payload.decode("utf-8"), object_pairs_hook=_reject_duplicate_keys
        )
    except CollectionValidationError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise CollectionValidationError(f"invalid {label} {path}: {error}") from None
    # _stable_file_fact already bounded the read by a stable stat tuple.  The
    # second small read is checked against that digest so an ambiguous race is
    # rejected rather than silently accepted.
    if hashlib.sha256(payload).hexdigest() != fact["sha256"]:
        raise CollectionValidationError(f"{label} changed while JSON was decoded: {path}")
    return _mapping(decoded, label), fact


def _verify_artifact_record(
    record_value: Any,
    expected_path: Path,
    label: str,
    *,
    allow_symlink: bool = False,
) -> dict[str, Any]:
    record = _mapping(record_value, label)
    recorded_path = _absolute_path(record.get("path"), f"{label}.path")
    try:
        recorded_resolved = recorded_path.resolve(strict=True)
        expected_resolved = expected_path.resolve(strict=True)
    except OSError as error:
        raise CollectionValidationError(f"cannot resolve {label}: {error}") from None
    if recorded_resolved != expected_resolved:
        raise CollectionValidationError(
            f"{label}.path resolves to {recorded_resolved}, expected {expected_resolved}"
        )
    fact = _stable_file_fact(expected_path, label, allow_symlink=allow_symlink)
    expected_bytes = _exact_integer(record.get("bytes"), f"{label}.bytes", 1)
    if expected_bytes != fact["bytes"]:
        raise CollectionValidationError(
            f"{label} byte count mismatch: {expected_bytes} != {fact['bytes']}"
        )
    expected_digest = _sha256(record.get("sha256"), f"{label}.sha256")
    if expected_digest != fact["sha256"]:
        raise CollectionValidationError(f"{label} SHA-256 mismatch")
    return fact


def _verify_sha_reference(
    record_value: Any,
    label: str,
) -> tuple[Mapping[str, Any], dict[str, Any]]:
    record = _mapping(record_value, label)
    path = _absolute_path(record.get("path"), f"{label}.path")
    document, fact = _load_json(path, label)
    if _sha256(record.get("sha256"), f"{label}.sha256") != fact["sha256"]:
        raise CollectionValidationError(f"{label} SHA-256 mismatch")
    return document, fact


def _parse_checksum_file(path: Path, expected_names: set[str], label: str) -> None:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise CollectionValidationError(f"cannot read {label}: {error}") from None
    entries: dict[str, str] = {}
    for index, line in enumerate(lines, 1):
        parts = line.split("  ", 1)
        if len(parts) != 2:
            raise CollectionValidationError(f"{label} line {index} is malformed")
        digest, name = parts
        _sha256(digest, f"{label} line {index} digest")
        if name in entries:
            raise CollectionValidationError(f"{label} duplicates {name!r}")
        if "/" in name or name in ("", ".", ".."):
            raise CollectionValidationError(f"{label} has unsafe artifact name {name!r}")
        entries[name] = digest
    if set(entries) != expected_names:
        raise CollectionValidationError(
            f"{label} artifact set mismatch: expected {sorted(expected_names)}, "
            f"got {sorted(entries)}"
        )
    for name, digest in entries.items():
        actual = _stable_file_fact(path.parent / name, f"{label}:{name}")
        if digest != actual["sha256"]:
            raise CollectionValidationError(f"{label} SHA-256 mismatch for {name}")


def _has_nested_state(value: Any, key: str, expected: str) -> bool:
    if isinstance(value, dict):
        if value.get(key) == expected:
            return True
        return any(_has_nested_state(child, key, expected) for child in value.values())
    if isinstance(value, list):
        return any(_has_nested_state(child, key, expected) for child in value)
    return False


def parse_sites(raw: str) -> tuple[str, ...]:
    """Parse an ordered, non-empty and duplicate-free B1..B13 CSV."""
    if not isinstance(raw, str) or not raw:
        raise CollectionValidationError("--sites cannot be empty")
    sites = tuple(raw.split(","))
    seen: set[str] = set()
    for site in sites:
        if not SITE_RE.fullmatch(site):
            raise CollectionValidationError(
                f"invalid site {site!r}; expected uppercase B1..B13"
            )
        if site in seen:
            raise CollectionValidationError(f"duplicate expected site {site}")
        seen.add(site)
    return sites


def _return_source_matches(
    observed: Any,
    expected: str,
    expected_site: str = "",
) -> bool:
    """Accept the legacy exact source or the current mission-bound nonce."""
    if observed == expected:
        return True
    if not isinstance(observed, str) or not expected:
        return False
    if expected == "guest:usage_complete":
        match = GUEST_RETURN_SOURCE_RE.fullmatch(observed)
        return match is not None and (
            not expected_site or match.group(1) == expected_site
        )
    site_exit_suffix = ":site_exit_first"
    if not expected.endswith(site_exit_suffix):
        return False
    authority = expected[: -len(site_exit_suffix)]
    token_prefix = f"{authority}:ui_return_token="
    if not observed.startswith(token_prefix) or not observed.endswith(
        site_exit_suffix
    ):
        return False
    token = observed[len(token_prefix) : -len(site_exit_suffix)]
    return UI_RETURN_TOKEN_RE.fullmatch(token) is not None


def _authority_contract(authority: str, mission_intent: str) -> dict[str, Any]:
    key = (authority, mission_intent)
    contracts = {
        ("operator", "delivery"): {
            "matrix_return_authority": "operator_rest",
            "expected_return_source": "",
            "captured_ui_kind": "operator",
            "matrix_subcommand": "camping-sites",
        },
        ("operator", "recall"): {
            "matrix_return_authority": "operator_rest",
            "expected_return_source": "",
            "captured_ui_kind": "operator",
            "matrix_subcommand": "camping-sites-recall",
        },
        ("operator-browser", "delivery"): {
            "matrix_return_authority": "operator_browser",
            "expected_return_source": "ws:usage_complete:site_exit_first",
            "captured_ui_kind": "operator",
            "matrix_subcommand": "camping-sites-browser",
        },
        ("operator-browser", "recall"): {
            "matrix_return_authority": "operator_browser",
            "expected_return_source": "ws:usage_complete:site_exit_first",
            "captured_ui_kind": "operator",
            "matrix_subcommand": "camping-sites-browser-recall",
        },
        ("guest", "recall"): {
            "matrix_return_authority": "guest_browser",
            "expected_return_source": "guest:usage_complete",
            "captured_ui_kind": "guest",
            "matrix_subcommand": "camping-sites-guest",
        },
    }
    if key not in contracts:
        if authority not in ("operator", "operator-browser", "guest"):
            raise CollectionValidationError(
                "--authority must be operator, operator-browser, or guest"
            )
        if mission_intent not in ("delivery", "recall"):
            raise CollectionValidationError(
                "--mission-intent must be delivery or recall"
            )
        raise CollectionValidationError(
            "guest authority is valid only for recall evidence"
        )
    return contracts[key]


def _validate_native_matrix(
    matrix: Mapping[str, Any],
    *,
    site: str,
    authority: str,
    mission_intent: str,
    contract: Mapping[str, Any],
    software_identity_cache: dict[str, dict[str, str]],
) -> dict[str, Any]:
    label = f"{site}.native_matrix"
    _assert_equal(matrix.get("schema"), MATRIX_SCHEMA, f"{label}.schema")
    _assert_equal(matrix.get("status"), "PASS", f"{label}.status")
    scope = _mapping(matrix.get("scope"), f"{label}.scope")
    _assert_equal(scope.get("selected_sites"), [site], f"{label}.selected_sites")
    _assert_equal(scope.get("mission_intent"), mission_intent, f"{label}.intent")
    _assert_equal(
        scope.get("return_authority"),
        contract["matrix_return_authority"],
        f"{label}.return_authority",
    )
    _assert_equal(
        scope.get("expected_return_source", ""),
        contract["expected_return_source"],
        f"{label}.expected_return_source",
    )
    _assert_equal(scope.get("motion_commands_sent"), True, f"{label}.motion")
    _assert_equal(scope.get("pose_teleport_used"), False, f"{label}.teleport")
    _assert_equal(scope.get("fake_sensor_data_used"), False, f"{label}.fake_sensor")

    native_sites = _list(matrix.get("sites"), f"{label}.sites")
    if len(native_sites) != 1:
        raise CollectionValidationError(f"{label} must contain exactly one site")
    item = _mapping(native_sites[0], f"{label}.sites[0]")
    _assert_equal(item.get("site"), site, f"{label}.site")
    _assert_equal(item.get("status"), "PASS", f"{label}.site_status")
    _assert_equal(item.get("mission_intent"), mission_intent, f"{label}.site_intent")
    if item.get("failure_reason") not in (None, ""):
        raise CollectionValidationError(f"{label} contains a failure reason")

    if mission_intent == "recall":
        _assert_equal(item.get("service_mode"), "roadside_stop", f"{label}.service_mode")
        sequence = _list(
            item.get("service_state_sequence"), f"{label}.service_state_sequence"
        )
        for required in (
            "RECALL_TO_SITE_ROAD",
            "GUEST_LOADING_WAIT",
            "RETURN_WITH_CARGO",
        ):
            if required not in sequence:
                raise CollectionValidationError(
                    f"{label} recall sequence is missing {required}"
                )
        if authority == "operator":
            response = _mapping(item.get("dispatch_response"), f"{label}.dispatch")
            _assert_equal(response.get("intent"), "recall", f"{label}.dispatch.intent")
        elif authority == "guest":
            response = _mapping(item.get("return_response"), f"{label}.return")
            _assert_equal(response.get("action"), "usage_complete", f"{label}.action")
            _assert_equal(
                response.get("transport"),
                "visible_guest_page_websocket_via_cdp",
                f"{label}.transport",
            )
            requests = _list(
                item.get("ui_operation_request_sequence"),
                f"{label}.ui_operation_request_sequence",
            )
            if not any(
                isinstance(request, dict)
                and request.get("operation") == 3
                and _return_source_matches(
                    request.get("source"),
                    contract["expected_return_source"],
                    site,
                )
                for request in requests
            ):
                raise CollectionValidationError(
                    f"{label} did not observe the Guest RETURN source"
                )

    if authority == "operator-browser":
        dispatch = _mapping(item.get("dispatch_response"), f"{label}.dispatch")
        expected_dispatch_transport = (
            "visible_operator_page_http_via_cdp_input"
            if mission_intent == "recall"
            else "visible_operator_page_websocket_via_cdp_input"
        )
        expected_dispatch_source = (
            "robot_ui:recall" if mission_intent == "recall" else "ws"
        )
        _assert_equal(
            dispatch.get("transport"),
            expected_dispatch_transport,
            f"{label}.dispatch.transport",
        )
        _assert_equal(
            dispatch.get("source"),
            expected_dispatch_source,
            f"{label}.dispatch.source",
        )
        _assert_equal(
            item.get("dispatch_source_verified"),
            True,
            f"{label}.dispatch_source_verified",
        )
        interactions = _list(
            dispatch.get("interactions"), f"{label}.dispatch.interactions"
        )
        pointer_count = sum(
            1
            for interaction in interactions
            if isinstance(interaction, dict)
            and interaction.get("transport")
            == "CDP.Input.dispatchMouseEvent"
        )
        text_count = sum(
            1
            for interaction in interactions
            if isinstance(interaction, dict)
            and interaction.get("transport") == "CDP.Input.insertText"
        )
        if pointer_count < 7 or text_count != 1:
            raise CollectionValidationError(
                f"{label} does not retain the complete Robot UI input sequence"
            )
        response = _mapping(item.get("return_response"), f"{label}.return")
        _assert_equal(response.get("action"), "usage_complete", f"{label}.return.action")
        _assert_equal(response.get("source"), "ws:usage_complete", f"{label}.return.source")
        _assert_equal(
            response.get("transport"),
            "visible_operator_page_websocket_via_cdp_input",
            f"{label}.return.transport",
        )
        controller_requests = _list(
            item.get("controller_operation_request_sequence"),
            f"{label}.controller_operation_request_sequence",
        )
        if not any(
            isinstance(request, dict)
            and request.get("operation") == 3
            and _return_source_matches(
                request.get("source"), contract["expected_return_source"]
            )
            for request in controller_requests
        ):
            raise CollectionValidationError(
                f"{label} did not observe the Robot UI controller RETURN source"
            )

    collision = _mapping(item.get("collision_evidence"), f"{label}.collision")
    _assert_equal(
        collision.get("subscriber_created"), True, f"{label}.collision subscriber"
    )
    publisher_count = _exact_integer(
        collision.get("publisher_count"), f"{label}.collision.publisher_count", 1
    )
    _assert_equal(
        collision.get("publisher_discovered"),
        True,
        f"{label}.collision publisher",
    )
    _assert_equal(
        collision.get("subscriber_and_publisher_discovered"),
        True,
        f"{label}.collision graph",
    )
    event_count = _exact_integer(
        collision.get("event_count"), f"{label}.collision.event_count", 0
    )
    if event_count != 0:
        raise CollectionValidationError(
            f"{label}.collision.event_count must be 0, got {event_count}"
        )
    for count_key in ("sampled_event_count", "omitted_event_count"):
        if _exact_integer(
            collision.get(count_key), f"{label}.collision.{count_key}", 0
        ) != 0:
            raise CollectionValidationError(f"{label}.collision.{count_key} must be 0")
    _assert_equal(collision.get("events"), [], f"{label}.collision.events")

    _assert_equal(item.get("parking_confirmed"), True, f"{label}.parking_confirmed")
    _assert_equal(item.get("charging_confirmed"), True, f"{label}.charging_confirmed")
    final_state = _mapping(
        item.get("final_service_state"), f"{label}.final_service_state"
    )
    _assert_equal(final_state.get("state_name"), "CHARGING", f"{label}.final_state")
    final_gate = _mapping(item.get("final_gate_status"), f"{label}.final_gate_status")
    _assert_equal(final_gate.get("operating_state"), "CHARGING", f"{label}.gate")
    final_parking = _mapping(
        item.get("final_parking_status"), f"{label}.final_parking_status"
    )
    if not _has_nested_state(final_parking, "operating_state", "PARKED"):
        raise CollectionValidationError(f"{label} has no final PARKED controller state")

    actor_id = _exact_integer(item.get("actor_id"), f"{label}.actor_id", 1)
    physical = _mapping(
        item.get("final_physical_four_wheel_status"), f"{label}.physical"
    )
    _assert_equal(physical.get("actor_id"), actor_id, f"{label}.physical.actor_id")
    _assert_equal(physical.get("ready"), True, f"{label}.physical.ready")
    _assert_equal(
        physical.get("motion_backend"), EXPECTED_BACKEND, f"{label}.physical.backend"
    )
    _assert_equal(
        physical.get("api_version"), EXPECTED_API_VERSION, f"{label}.physical.api"
    )
    _assert_equal(
        physical.get("physical_gate_accepted"), True, f"{label}.physical.gate"
    )
    _assert_equal(
        physical.get("independent_wheel_drive_available"),
        True,
        f"{label}.physical.independent_drive",
    )

    final_odom = _mapping(
        item.get("final_carla_odometry"), f"{label}.final_carla_odometry"
    )
    final_speed = _finite(
        final_odom.get("speed_mps"), f"{label}.final_speed_mps", minimum=0.0
    )
    if final_speed > 0.05:
        raise CollectionValidationError(
            f"{label}.final_speed_mps {final_speed} exceeds 0.05"
        )

    elapsed = _positive(item.get("elapsed_s"), f"{label}.elapsed_s")
    outbound_duration = _positive(
        item.get("outbound_duration_s"), f"{label}.outbound_duration_s"
    )
    return_duration = _positive(
        item.get("return_duration_s"), f"{label}.return_duration_s"
    )
    total_distance = _positive(
        item.get("total_odom_distance_m"), f"{label}.total_odom_distance_m"
    )
    outbound_distance = _positive(
        item.get("outbound_distance_m"), f"{label}.outbound_distance_m"
    )
    return_distance = _positive(
        item.get("return_distance_m"), f"{label}.return_distance_m"
    )
    _split_sum(elapsed, outbound_duration, return_duration, f"{label}.duration split")
    _split_sum(
        total_distance, outbound_distance, return_distance, f"{label}.distance split"
    )
    live_metrics = _mapping(item.get("motion_metrics"), f"{label}.motion_metrics")
    _assert_equal(
        live_metrics.get("motion_command_observed"),
        True,
        f"{label}.motion_command_observed",
    )
    _exact_integer(
        live_metrics.get("carla_odometry_samples"),
        f"{label}.carla_odometry_samples",
        1,
    )
    measured_distance = _positive(
        live_metrics.get("carla_odometry_distance_m"),
        f"{label}.carla_odometry_distance_m",
    )
    _same_number(total_distance, measured_distance, f"{label}.odometry distance")
    drop_zone_error = _finite(
        item.get("drop_zone_error_m"), f"{label}.drop_zone_error_m", minimum=0.0
    )

    runtime_reference = _mapping(
        matrix.get("runtime_profile_audit"), f"{label}.runtime_profile_audit"
    )
    _assert_equal(runtime_reference.get("status"), "PASS", f"{label}.runtime audit")
    runtime_audit, runtime_fact = _verify_sha_reference(
        runtime_reference, f"{label}.runtime_profile_audit"
    )
    _assert_equal(
        runtime_audit.get("schema"), RUNTIME_AUDIT_SCHEMA, f"{label}.runtime schema"
    )
    _assert_equal(runtime_audit.get("status"), "PASS", f"{label}.runtime status")
    _assert_equal(runtime_audit.get("accepted"), True, f"{label}.runtime accepted")
    _assert_equal(runtime_audit.get("errors"), [], f"{label}.runtime errors")
    _assert_equal(
        runtime_audit.get("profile"),
        CURRENT_RUNTIME_PROFILE,
        f"{label}.runtime current profile",
    )
    ego = _mapping(
        _mapping(runtime_audit.get("carla"), f"{label}.runtime.carla").get(
            "ego_actor"
        ),
        f"{label}.runtime.ego_actor",
    )
    _assert_equal(ego.get("actor_id"), actor_id, f"{label}.runtime.actor_id")
    _assert_equal(ego.get("type_id"), EXPECTED_TYPE_ID, f"{label}.runtime.type_id")
    _assert_equal(ego.get("role_name"), EXPECTED_ROLE_NAME, f"{label}.runtime.role")
    parameters = _mapping(
        runtime_audit.get("selected_live_parameters"), f"{label}.runtime.parameters"
    )
    _assert_authoritative_profile_signature(
        parameters,
        CURRENT_RUNTIME_PROFILE_PARAMETERS,
        f"{label}.runtime authoritative signature",
    )
    bridge = _mapping(
        parameters.get("/physical_four_wheel_bridge"),
        f"{label}.runtime.physical_bridge",
    )
    _assert_equal(
        bridge.get("expected_blueprint_id"), EXPECTED_TYPE_ID, f"{label}.blueprint"
    )
    _assert_equal(
        bridge.get("extended_mode_backend"), EXPECTED_BACKEND, f"{label}.backend"
    )
    _assert_equal(bridge.get("role_name"), EXPECTED_ROLE_NAME, f"{label}.role")
    software_identity = _validate_software_identity(
        runtime_audit.get("software_identity"),
        f"{label}.runtime.software_identity",
        software_identity_cache,
    )
    world_identity = _validate_runtime_world_identity(
        runtime_audit,
        parameters,
        f"{label}.runtime",
    )

    source_reference = _mapping(
        matrix.get("sensor_source_audit"), f"{label}.sensor_source_audit"
    )
    _assert_equal(source_reference.get("status"), "PASS", f"{label}.sensor audit")
    source_audit, source_fact = _verify_sha_reference(
        source_reference, f"{label}.sensor_source_audit"
    )
    _assert_equal(source_audit.get("status"), "PASS", f"{label}.sensor status")
    source_summary = _mapping(
        source_audit.get("summary"), f"{label}.sensor.summary"
    )
    for count_key in ("streams_checked", "actors_checked"):
        _exact_integer(source_summary.get(count_key), f"{label}.{count_key}", 1)
    for count_key in ("stream_failures", "actor_failures"):
        if _exact_integer(source_summary.get(count_key), f"{label}.{count_key}", 0):
            raise CollectionValidationError(f"{label}.{count_key} must be 0")

    return {
        "item": item,
        "actor_id": actor_id,
        "publisher_count": publisher_count,
        "final_speed_mps": final_speed,
        "elapsed_s": elapsed,
        "outbound_duration_s": outbound_duration,
        "return_duration_s": return_duration,
        "total_odom_distance_m": total_distance,
        "outbound_distance_m": outbound_distance,
        "return_distance_m": return_distance,
        "drop_zone_error_m": drop_zone_error,
        "physical_manifest_sha256": _sha256(
            physical.get("physical_manifest_sha256"),
            f"{label}.physical_manifest_sha256",
        ),
        "runtime_audit_sha256": runtime_fact["sha256"],
        "sensor_audit_sha256": source_fact["sha256"],
        **software_identity,
        **world_identity,
    }


def _validate_wheels(
    site_dir: Path,
    site_manifest: Mapping[str, Any],
    *,
    site: str,
    actor_id: int,
    odom_distance: float,
) -> dict[str, Any]:
    label = f"{site}.physical_wheels"
    record = _mapping(site_manifest.get("physical_wheels"), label)
    raw_path = site_dir / "physical_wheels.jsonl"
    raw_fact = _verify_artifact_record(record.get("jsonl"), raw_path, f"{label}.jsonl")
    manifest_path = site_dir / "physical_wheels.manifest.json"
    manifest, manifest_fact = _load_json(manifest_path, f"{label}.manifest")
    _verify_artifact_record(record.get("manifest"), manifest_path, f"{label}.manifest")
    _assert_equal(manifest.get("schema"), WHEEL_MANIFEST_SCHEMA, f"{label}.schema")
    _assert_equal(manifest.get("status"), "STOPPED", f"{label}.status")
    _assert_equal(manifest.get("stop_reason"), "signal", f"{label}.stop_reason")
    _assert_equal(manifest.get("read_only"), True, f"{label}.read_only")
    _assert_equal(record.get("status"), "STOPPED", f"{label}.record.status")
    sample_count = _exact_integer(
        manifest.get("sample_count"), f"{label}.sample_count", 1
    )
    _assert_equal(record.get("sample_count"), sample_count, f"{label}.record.samples")
    actor = _mapping(manifest.get("actor"), f"{label}.actor")
    _assert_equal(actor.get("id"), actor_id, f"{label}.actor.id")
    _assert_equal(actor.get("type_id"), EXPECTED_TYPE_ID, f"{label}.actor.type_id")
    _assert_equal(actor.get("role_name"), EXPECTED_ROLE_NAME, f"{label}.actor.role")
    raw_output = _mapping(manifest.get("output"), f"{label}.output")
    _assert_equal(
        _absolute_path(raw_output.get("path"), f"{label}.output.path").resolve(),
        raw_path.resolve(),
        f"{label}.output.path",
    )
    _assert_equal(raw_output.get("bytes"), raw_fact["bytes"], f"{label}.output.bytes")
    _assert_equal(raw_output.get("sha256"), raw_fact["sha256"], f"{label}.output.sha256")

    summary_record = _mapping(record.get("summary"), f"{label}.summary_record")
    _assert_equal(summary_record.get("status"), "PASS", f"{label}.summary status")
    _assert_equal(summary_record.get("exit_code"), 0, f"{label}.summary exit")
    _assert_equal(summary_record.get("tee_exit_code"), 0, f"{label}.summary tee")
    artifact_records = _mapping(
        summary_record.get("artifacts"), f"{label}.summary artifacts"
    )
    expected_artifacts = {
        "wheel_summary.json",
        "wheel_measurements.csv",
        "wheel_summary.png",
        "SHA256SUMS",
    }
    if set(artifact_records) != expected_artifacts:
        raise CollectionValidationError(f"{label}.summary artifact set mismatch")
    summary_root = site_dir / "wheel_summary"
    try:
        root_stat = summary_root.lstat()
    except OSError as error:
        raise CollectionValidationError(f"missing {label}.summary directory: {error}") from None
    if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
        raise CollectionValidationError(f"{label}.summary must be a real directory")
    artifact_facts: dict[str, dict[str, Any]] = {}
    for name in sorted(expected_artifacts):
        artifact_facts[name] = _verify_artifact_record(
            artifact_records.get(name), summary_root / name, f"{label}.summary:{name}"
        )
    _parse_checksum_file(
        summary_root / "SHA256SUMS",
        expected_artifacts - {"SHA256SUMS"},
        f"{label}.summary.SHA256SUMS",
    )

    summary_path = summary_root / "wheel_summary.json"
    summary, summary_fact = _load_json(summary_path, f"{label}.summary JSON")
    _assert_equal(
        summary_fact["sha256"],
        artifact_facts["wheel_summary.json"]["sha256"],
        f"{label}.summary JSON hash",
    )
    _assert_equal(summary.get("schema"), WHEEL_SUMMARY_SCHEMA, f"{label}.summary schema")
    _assert_equal(summary.get("status"), "PASS", f"{label}.summary PASS")
    source = _mapping(summary.get("source"), f"{label}.summary.source")
    _assert_equal(
        _absolute_path(source.get("path"), f"{label}.summary.source.path").resolve(),
        raw_path.resolve(),
        f"{label}.summary.source.path",
    )
    _assert_equal(source.get("bytes"), raw_fact["bytes"], f"{label}.summary.source.bytes")
    _assert_equal(source.get("sha256"), raw_fact["sha256"], f"{label}.summary.source.sha")
    samples = _mapping(summary.get("samples"), f"{label}.summary.samples")
    _assert_equal(samples.get("count"), sample_count, f"{label}.summary.samples.count")
    _positive(
        samples.get("recording_duration_seconds"),
        f"{label}.summary.recording_duration_seconds",
    )
    summary_actor = _mapping(summary.get("actor"), f"{label}.summary.actor")
    _assert_equal(summary_actor.get("id"), actor_id, f"{label}.summary.actor.id")
    _assert_equal(
        summary_actor.get("type_id"), EXPECTED_TYPE_ID, f"{label}.summary.actor.type"
    )
    _assert_equal(
        summary_actor.get("role_name"), EXPECTED_ROLE_NAME, f"{label}.summary.actor.role"
    )
    validation = _mapping(summary.get("validation"), f"{label}.summary.validation")
    _assert_equal(
        validation.get("canonical_wheels_exact"),
        list(CANONICAL_WHEELS),
        f"{label}.canonical wheels validation",
    )
    for key in (
        "actor_identity_consistent",
        "footer_sample_count_matches",
        "footer_valid",
        "header_valid",
        "streaming_reader",
    ):
        _assert_equal(validation.get(key), True, f"{label}.validation.{key}")
    _assert_equal(validation.get("footer_status"), "STOPPED", f"{label}.footer status")

    wheels = _mapping(summary.get("wheels"), f"{label}.summary.wheels")
    if set(wheels) != set(CANONICAL_WHEELS) or len(wheels) != 4:
        raise CollectionValidationError(
            f"{label} must contain exactly FL, FR, RL, RR"
        )
    for wheel_name in CANONICAL_WHEELS:
        wheel = _mapping(wheels.get(wheel_name), f"{label}.{wheel_name}")
        _assert_equal(
            wheel.get("sample_count"), sample_count, f"{label}.{wheel_name}.samples"
        )
        if _exact_integer(
            wheel.get("in_air_count"), f"{label}.{wheel_name}.in_air_count", 0
        ) != 0:
            raise CollectionValidationError(
                f"{label}.{wheel_name}.in_air_count must be 0"
            )
        _assert_equal(wheel.get("in_air_ratio"), 0, f"{label}.{wheel_name}.in_air_ratio")

    motion = _mapping(summary.get("motion"), f"{label}.summary.motion")
    wheel_distance = _positive(
        motion.get("planar_distance_m"), f"{label}.summary.planar_distance_m"
    )
    distance_tolerance = max(1.0, odom_distance * 0.02)
    if abs(wheel_distance - odom_distance) > distance_tolerance:
        raise CollectionValidationError(
            f"{label} wheel/odometry distance mismatch: "
            f"{wheel_distance} vs {odom_distance} m"
        )

    csv_path = summary_root / "wheel_measurements.csv"
    try:
        with csv_path.open("r", encoding="utf-8", newline="") as stream:
            csv_rows = list(csv.DictReader(stream))
    except (OSError, UnicodeError, csv.Error) as error:
        raise CollectionValidationError(f"invalid {label} wheel CSV: {error}") from None
    if [row.get("wheel") for row in csv_rows] != list(CANONICAL_WHEELS):
        raise CollectionValidationError(f"{label} wheel CSV is not canonical FL/FR/RL/RR")
    for row in csv_rows:
        if row.get("sample_count") != str(sample_count):
            raise CollectionValidationError(f"{label} wheel CSV sample count mismatch")
        try:
            csv_in_air_ratio = float(row.get("in_air_ratio", "nan"))
        except (TypeError, ValueError):
            raise CollectionValidationError(
                f"{label} wheel CSV has an invalid in_air_ratio"
            ) from None
        if row.get("in_air_count") != "0" or csv_in_air_ratio != 0.0:
            raise CollectionValidationError(f"{label} wheel CSV records an airborne wheel")
        if row.get("source_sha256") != raw_fact["sha256"]:
            raise CollectionValidationError(f"{label} wheel CSV source SHA mismatch")

    png_path = summary_root / "wheel_summary.png"
    if not png_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n"):
        raise CollectionValidationError(f"{label}.wheel_summary.png is not PNG data")

    return {
        "sample_count": sample_count,
        "planar_distance_m": wheel_distance,
        "raw_sha256": raw_fact["sha256"],
        "manifest_sha256": manifest_fact["sha256"],
        "summary_sha256": summary_fact["sha256"],
    }


def _validate_visual(
    site_dir: Path,
    site_manifest: Mapping[str, Any],
    *,
    site: str,
    expected_ui_kind: str,
) -> dict[str, Any]:
    label = f"{site}.visual"
    record = _mapping(site_manifest.get("visual"), label)
    _assert_equal(record.get("capture_finalized_with_q"), True, f"{label}.q")
    capture_root = site_dir / "visual"
    manifest_path = capture_root / "capture_manifest.json"
    manifest, _ = _load_json(manifest_path, f"{label}.capture_manifest")
    _verify_artifact_record(
        record.get("manifest"), manifest_path, f"{label}.capture_manifest"
    )
    _assert_equal(manifest.get("schema"), CAPTURE_SCHEMA, f"{label}.schema")
    _assert_equal(manifest.get("status"), "PASS", f"{label}.status")
    scope = _mapping(manifest.get("scope"), f"{label}.scope")
    _assert_equal(
        scope.get("vehicle_motion_or_ui_input_sent_by_capture"),
        False,
        f"{label}.passive",
    )
    _assert_equal(
        scope.get("ai_generated_or_enhanced"), False, f"{label}.not_generated"
    )
    x11 = _mapping(manifest.get("x11"), f"{label}.x11")
    _assert_equal(
        x11.get("geometry_validated_side_by_side"), True, f"{label}.side_by_side"
    )
    windows = _mapping(x11.get("windows"), f"{label}.windows")
    ui_key = "camrod_guest_ui" if expected_ui_kind == "guest" else "camrod_operator_ui"
    ui_window = _mapping(windows.get(ui_key), f"{label}.{ui_key}")
    _assert_equal(ui_window.get("kind"), expected_ui_kind, f"{label}.ui_kind")
    _mapping(windows.get("carla"), f"{label}.carla_window")

    artifacts = _mapping(manifest.get("artifacts"), f"{label}.artifacts")
    results: dict[str, Any] = {}
    for manifest_key, filename, record_key, signature in (
        (
            "contact_sheet_png",
            "representative_contact_sheet.png",
            "png",
            b"\x89PNG\r\n\x1a\n",
        ),
        ("representative_gif", "representative_motion.gif", "gif", b"GIF"),
    ):
        path = capture_root / filename
        fact = _verify_artifact_record(record.get(record_key), path, f"{label}.{record_key}")
        internal = _mapping(artifacts.get(manifest_key), f"{label}.{manifest_key}")
        _assert_equal(internal.get("path"), filename, f"{label}.{manifest_key}.path")
        _assert_equal(internal.get("bytes"), fact["bytes"], f"{label}.{manifest_key}.bytes")
        _assert_equal(
            internal.get("sha256"), fact["sha256"], f"{label}.{manifest_key}.sha"
        )
        try:
            prefix = path.read_bytes()[:8]
        except OSError as error:
            raise CollectionValidationError(f"cannot inspect {label}.{record_key}: {error}") from None
        if not prefix.startswith(signature):
            raise CollectionValidationError(f"{label}.{filename} has the wrong file signature")
        if record_key == "gif" and prefix[:6] not in (b"GIF87a", b"GIF89a"):
            raise CollectionValidationError(f"{label}.{filename} is not GIF87a/GIF89a")
        results[f"{record_key}_sha256"] = fact["sha256"]

    retained = _boolean(record.get("source_mp4_retained"), f"{label}.source_mp4_retained")
    _assert_equal(
        site_manifest.get("retain_source_video"), retained, f"{label}.retention request"
    )
    recording = _mapping(manifest.get("recording"), f"{label}.recording")
    source_video = _mapping(recording.get("source_video"), f"{label}.source_video")
    mp4_path = capture_root / "carla_camrod_desktop.mp4"
    if retained:
        _assert_equal(source_video.get("retained"), True, f"{label}.mp4 retained")
        _assert_equal(
            source_video.get("removed_after_derivation"), False, f"{label}.mp4 removed"
        )
        mp4_fact = _verify_artifact_record(record.get("mp4"), mp4_path, f"{label}.mp4")
        internal_mp4 = _mapping(artifacts.get("source_mp4"), f"{label}.source_mp4")
        _assert_equal(internal_mp4.get("sha256"), mp4_fact["sha256"], f"{label}.mp4 sha")
    else:
        _assert_equal(source_video.get("retained"), False, f"{label}.mp4 retained")
        _assert_equal(
            source_video.get("removed_after_derivation"), True, f"{label}.mp4 removed"
        )
        if mp4_path.exists() or mp4_path.is_symlink():
            raise CollectionValidationError(f"{label} source MP4 should have been removed")
    return results


def _validate_site(
    input_root: Path,
    site: str,
    run_site: Mapping[str, Any],
    *,
    authority: str,
    mission_intent: str,
    contract: Mapping[str, Any],
    software_identity_cache: dict[str, dict[str, str]],
) -> dict[str, Any]:
    site_dir = input_root / site
    try:
        site_stat = site_dir.lstat()
    except OSError as error:
        raise CollectionValidationError(f"missing site directory {site_dir}: {error}") from None
    if stat.S_ISLNK(site_stat.st_mode) or not stat.S_ISDIR(site_stat.st_mode):
        raise CollectionValidationError(f"{site} evidence must be a real directory")
    site_manifest_path = site_dir / "site_manifest.json"
    manifest, manifest_fact = _load_json(site_manifest_path, f"{site}.site_manifest")
    if manifest != run_site:
        raise CollectionValidationError(
            f"{site}.site_manifest does not exactly match run_manifest.sites entry"
        )
    _assert_equal(manifest.get("schema"), SITE_SCHEMA, f"{site}.schema")
    _assert_equal(manifest.get("status"), "PASS", f"{site}.status")
    _assert_equal(manifest.get("site"), site, f"{site}.site")
    _assert_equal(manifest.get("failure_reasons"), [], f"{site}.failure_reasons")
    _assert_equal(manifest.get("stop_on_failure"), True, f"{site}.stop_on_failure")
    authority_record = _mapping(manifest.get("authority"), f"{site}.authority")
    expected_authority = {
        "frontend": authority,
        "mission_intent": mission_intent,
        "matrix_return_authority": contract["matrix_return_authority"],
        "expected_return_source": contract["expected_return_source"],
        "captured_ui_kind": contract["captured_ui_kind"],
    }
    _assert_equal(authority_record, expected_authority, f"{site}.authority")
    command = _mapping(manifest.get("matrix_command"), f"{site}.matrix_command")
    _assert_equal(
        command.get("argv"),
        ["site_access.sh", contract["matrix_subcommand"]],
        f"{site}.matrix argv",
    )
    _assert_equal(command.get("exit_code"), 0, f"{site}.matrix exit")
    _assert_equal(command.get("tee_exit_code"), 0, f"{site}.matrix tee")
    _assert_equal(command.get("only_motion_authority"), True, f"{site}.motion authority")
    command_env = _mapping(command.get("environment"), f"{site}.matrix environment")
    _assert_equal(
        command_env.get("CAMROD_CARLA_CAMPING_SITES"), site, f"{site}.command site"
    )
    _assert_equal(
        command_env.get("CAMROD_CARLA_MATRIX_MISSION_INTENT"),
        mission_intent,
        f"{site}.command intent",
    )

    matrix_record = _mapping(manifest.get("matrix_report"), f"{site}.matrix_report")
    _assert_equal(matrix_record.get("status"), "PASS", f"{site}.matrix status")
    _assert_equal(matrix_record.get("site_status"), "PASS", f"{site}.matrix site status")
    matrix_path = _absolute_path(matrix_record.get("path"), f"{site}.matrix path")
    matrix, matrix_fact = _load_json(matrix_path, f"{site}.native matrix")
    _verify_artifact_record(matrix_record, matrix_path, f"{site}.native matrix")
    evidence_link = _absolute_path(
        matrix_record.get("evidence_link"), f"{site}.matrix evidence_link"
    )
    expected_link = site_dir / "camping_site_matrix.json"
    if evidence_link != expected_link:
        raise CollectionValidationError(
            f"{site}.matrix evidence_link must be {expected_link}"
        )
    if not expected_link.is_symlink():
        raise CollectionValidationError(f"{site}.matrix evidence_link must be a symlink")
    try:
        if expected_link.resolve(strict=True) != matrix_path.resolve(strict=True):
            raise CollectionValidationError(
                f"{site}.matrix evidence_link does not target the native report"
            )
    except OSError as error:
        raise CollectionValidationError(f"cannot resolve {site}.matrix link: {error}") from None

    native = _validate_native_matrix(
        matrix,
        site=site,
        authority=authority,
        mission_intent=mission_intent,
        contract=contract,
        software_identity_cache=software_identity_cache,
    )
    manifest_metrics = _mapping(manifest.get("motion_metrics"), f"{site}.motion_metrics")
    for key in (
        "elapsed_s",
        "outbound_duration_s",
        "return_duration_s",
        "outbound_distance_m",
        "return_distance_m",
        "total_odom_distance_m",
    ):
        actual = _finite(manifest_metrics.get(key), f"{site}.manifest metric {key}")
        _same_number(actual, native[key], f"{site}.manifest metric {key}")

    wheels = _validate_wheels(
        site_dir,
        manifest,
        site=site,
        actor_id=native["actor_id"],
        odom_distance=native["total_odom_distance_m"],
    )
    visual = _validate_visual(
        site_dir,
        manifest,
        site=site,
        expected_ui_kind=contract["captured_ui_kind"],
    )
    return {
        "site": site,
        "status": "PASS",
        "authority": authority,
        "mission_intent": mission_intent,
        "actor_id": native["actor_id"],
        "actor_type_id": EXPECTED_TYPE_ID,
        "actor_role_name": EXPECTED_ROLE_NAME,
        "motion_backend": EXPECTED_BACKEND,
        "elapsed_s": native["elapsed_s"],
        "outbound_duration_s": native["outbound_duration_s"],
        "return_duration_s": native["return_duration_s"],
        "total_odom_distance_m": native["total_odom_distance_m"],
        "outbound_distance_m": native["outbound_distance_m"],
        "return_distance_m": native["return_distance_m"],
        "wheel_planar_distance_m": wheels["planar_distance_m"],
        "drop_zone_error_m": native["drop_zone_error_m"],
        "final_speed_mps": native["final_speed_mps"],
        "collision_publisher_count": native["publisher_count"],
        "collision_event_count": 0,
        "parking_confirmed": True,
        "charging_confirmed": True,
        "wheel_sample_count": wheels["sample_count"],
        "all_wheel_in_air_count": 0,
        "matrix_sha256": matrix_fact["sha256"],
        "wheel_jsonl_sha256": wheels["raw_sha256"],
        "wheel_summary_sha256": wheels["summary_sha256"],
        "runtime_audit_sha256": native["runtime_audit_sha256"],
        "software_binding_sha256": native["binding_sha256"],
        "runtime_source_path": native["source_path"],
        "runtime_source_branch": native["source_branch"],
        "runtime_source_head": native["source_head"],
        "runtime_install_sha256": native["install_sha256"],
        "umap_sha256": native["umap_sha256"],
        "lanelet_sha256": native["lanelet_sha256"],
        "launch_cmdline_sha256": native["launch_cmdline_sha256"],
        "sensor_audit_sha256": native["sensor_audit_sha256"],
        "physical_manifest_sha256": native["physical_manifest_sha256"],
        "png_sha256": visual["png_sha256"],
        "gif_sha256": visual["gif_sha256"],
        "site_manifest_sha256": manifest_fact["sha256"],
    }


def _validate_metrics_summary(
    input_root: Path,
    run_manifest: Mapping[str, Any],
    rows: Sequence[Mapping[str, Any]],
    expected_sites: tuple[str, ...],
) -> dict[str, Any]:
    label = "metrics_summary"
    run_record = _mapping(run_manifest.get("metrics_summary"), label)
    artifact_records = _mapping(run_record.get("artifacts"), f"{label}.artifacts")
    expected_names = {
        "camping_site_metrics.json",
        "camping_site_metrics.csv",
        "camping_site_metrics.md",
        "SHA256SUMS",
    }
    if set(artifact_records) != expected_names:
        raise CollectionValidationError(f"{label} artifact set mismatch")
    summary_root = input_root / "summary"
    try:
        summary_stat = summary_root.lstat()
    except OSError as error:
        raise CollectionValidationError(f"missing summary directory: {error}") from None
    if stat.S_ISLNK(summary_stat.st_mode) or not stat.S_ISDIR(summary_stat.st_mode):
        raise CollectionValidationError("summary must be a real directory")
    facts = {
        name: _verify_artifact_record(
            artifact_records.get(name), summary_root / name, f"{label}:{name}"
        )
        for name in expected_names
    }
    _parse_checksum_file(
        summary_root / "SHA256SUMS",
        expected_names - {"SHA256SUMS"},
        "metrics_summary.SHA256SUMS",
    )
    document, document_fact = _load_json(
        summary_root / "camping_site_metrics.json", "metrics summary JSON"
    )
    _assert_equal(document.get("schema"), METRICS_SCHEMA, f"{label}.schema")
    _assert_equal(run_record.get("schema"), METRICS_SCHEMA, f"{label}.run schema")
    expected_coverage = (
        "B1_B13_COMPLETE" if set(expected_sites) == set(ALL_SITES) else "PARTIAL"
    )
    _assert_equal(document.get("coverage"), expected_coverage, f"{label}.coverage")
    _assert_equal(run_record.get("coverage"), expected_coverage, f"{label}.run coverage")
    metric_rows = _list(document.get("sites"), f"{label}.sites")
    metrics_site_order = tuple(sorted(expected_sites, key=lambda value: int(value[1:])))
    if [row.get("site") if isinstance(row, dict) else None for row in metric_rows] != list(
        metrics_site_order
    ):
        raise CollectionValidationError(f"{label}.sites has missing, duplicate, or reordered sites")
    row_by_site = {row["site"]: row for row in rows}
    for index, metric_value in enumerate(metric_rows):
        metric = _mapping(metric_value, f"{label}.sites[{index}]")
        site = metrics_site_order[index]
        expected = row_by_site[site]
        _assert_equal(metric.get("status"), "PASS", f"{label}.{site}.status")
        for metric_key, row_key in (
            ("total_elapsed_s", "elapsed_s"),
            ("outbound_duration_s", "outbound_duration_s"),
            ("return_duration_s", "return_duration_s"),
            ("total_odom_distance_m", "total_odom_distance_m"),
            ("outbound_distance_m", "outbound_distance_m"),
            ("return_distance_m", "return_distance_m"),
            ("drop_zone_error_m", "drop_zone_error_m"),
        ):
            actual = _finite(metric.get(metric_key), f"{label}.{site}.{metric_key}")
            _same_number(actual, float(expected[row_key]), f"{label}.{site}.{metric_key}")
        _assert_equal(metric.get("actor_id"), expected["actor_id"], f"{label}.{site}.actor")
        _assert_equal(metric.get("parking_confirmed"), True, f"{label}.{site}.parking")
        _assert_equal(metric.get("charging_confirmed"), True, f"{label}.{site}.charging")
        _assert_equal(metric.get("final_service_state"), "CHARGING", f"{label}.{site}.state")
        _assert_equal(
            _sha256(metric.get("source_report_sha256"), f"{label}.{site}.report sha"),
            expected["matrix_sha256"],
            f"{label}.{site}.report sha",
        )

    aggregate = _mapping(document.get("aggregate"), f"{label}.aggregate")
    count = len(rows)
    _assert_equal(aggregate.get("site_count"), count, f"{label}.site_count")
    _assert_equal(aggregate.get("pass_count"), count, f"{label}.pass_count")
    _assert_equal(aggregate.get("fail_count"), 0, f"{label}.fail_count")
    missing_sites = [site for site in ALL_SITES if site not in set(expected_sites)]
    _assert_equal(aggregate.get("missing_sites"), missing_sites, f"{label}.missing_sites")
    for aggregate_key, row_key in (
        ("total_elapsed_s", "elapsed_s"),
        ("outbound_duration_s", "outbound_duration_s"),
        ("return_duration_s", "return_duration_s"),
        ("total_odom_distance_m", "total_odom_distance_m"),
        ("outbound_distance_m", "outbound_distance_m"),
        ("return_distance_m", "return_distance_m"),
    ):
        expected_total = round(sum(float(row[row_key]) for row in rows), 6)
        actual_total = _finite(aggregate.get(aggregate_key), f"{label}.{aggregate_key}")
        _same_number(actual_total, expected_total, f"{label}.{aggregate_key}")

    try:
        with (summary_root / "camping_site_metrics.csv").open(
            "r", encoding="utf-8", newline=""
        ) as stream:
            csv_rows = list(csv.DictReader(stream))
    except (OSError, UnicodeError, csv.Error) as error:
        raise CollectionValidationError(f"invalid metrics CSV: {error}") from None
    if [row.get("site") for row in csv_rows] != list(metrics_site_order):
        raise CollectionValidationError("metrics CSV site set/order mismatch")
    try:
        markdown = (summary_root / "camping_site_metrics.md").read_text(
            encoding="utf-8"
        )
    except (OSError, UnicodeError) as error:
        raise CollectionValidationError(f"invalid metrics Markdown: {error}") from None
    for site in expected_sites:
        if f"| {site} | PASS |" not in markdown:
            raise CollectionValidationError(f"metrics Markdown is missing PASS row {site}")
    _assert_equal(
        facts["camping_site_metrics.json"]["sha256"],
        document_fact["sha256"],
        "metrics JSON digest",
    )
    return {
        "json_sha256": document_fact["sha256"],
        "csv_sha256": facts["camping_site_metrics.csv"]["sha256"],
        "markdown_sha256": facts["camping_site_metrics.md"]["sha256"],
        "checksums_sha256": facts["SHA256SUMS"]["sha256"],
    }


def validate_collection(
    input_root: Path | str,
    *,
    expected_sites: Sequence[str],
    authority: str,
    mission_intent: str,
    allow_validating_manifest: bool = False,
) -> dict[str, Any]:
    """Read and fully validate one completed runner output collection."""
    raw_root = Path(input_root)
    if not raw_root.is_absolute():
        raise CollectionValidationError(f"input root must be absolute: {raw_root}")
    try:
        root_stat = raw_root.lstat()
        root = raw_root.resolve(strict=True)
    except OSError as error:
        raise CollectionValidationError(f"cannot resolve input root {raw_root}: {error}") from None
    if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
        raise CollectionValidationError("input root must be a real directory, not a symlink")
    normalized_sites = tuple(expected_sites)
    if not normalized_sites:
        raise CollectionValidationError("expected sites cannot be empty")
    # Reuse the CLI parser contract even when called as a Python API.
    parsed_sites = parse_sites(",".join(normalized_sites))
    contract = _authority_contract(authority, mission_intent)

    run_path = root / "run_manifest.json"
    run, run_fact = _load_json(run_path, "run_manifest")
    _assert_equal(run.get("schema"), RUN_SCHEMA, "run_manifest.schema")
    run_status = run.get("status")
    allowed_statuses = (
        {"PASS", "VALIDATING"} if allow_validating_manifest else {"PASS"}
    )
    if run_status not in allowed_statuses:
        raise CollectionValidationError(
            "run_manifest.status mismatch: expected "
            f"{sorted(allowed_statuses)!r}, got {run_status!r}"
        )
    _assert_equal(run.get("exit_code"), 0, "run_manifest.exit_code")
    if run.get("failure_site") not in (None, "") or run.get("failure_reason") not in (
        None,
        "",
    ):
        raise CollectionValidationError("run_manifest retains failure metadata")
    _assert_equal(run.get("selected_sites"), list(parsed_sites), "run_manifest.selected_sites")
    _assert_equal(
        run.get("completed_site_manifests"),
        len(parsed_sites),
        "run_manifest.completed_site_manifests",
    )
    _assert_equal(run.get("stop_on_first_failure"), True, "run_manifest.stop_on_failure")
    motion_authority = _mapping(run.get("motion_authority"), "run_manifest.motion_authority")
    for key, expected in (
        ("frontend", authority),
        ("mission_intent", mission_intent),
        ("matrix_return_authority", contract["matrix_return_authority"]),
        ("expected_return_source", contract["expected_return_source"]),
        ("captured_ui_kind", contract["captured_ui_kind"]),
        ("sites_run_independently", True),
        ("observers_publish_or_control_vehicle", False),
    ):
        _assert_equal(motion_authority.get(key), expected, f"run_manifest.motion_authority.{key}")
    _assert_equal(
        motion_authority.get("only_command"),
        f"site_access.sh {contract['matrix_subcommand']}",
        "run_manifest.motion_authority.only_command",
    )
    source_record = _mapping(run.get("source"), "run_manifest.source")
    source_root = _absolute_path(
        source_record.get("git_root"), "run_manifest.source.git_root"
    )
    expected_runner = source_root / "scripts" / "virtual_carla" / "run_site_evidence_matrix.sh"
    expected_entrypoint = source_root / "scripts" / "virtual_carla" / "site_access.sh"
    _verify_artifact_record(
        source_record.get("runner"), expected_runner, "run_manifest.source.runner"
    )
    _verify_artifact_record(
        source_record.get("entrypoint"),
        expected_entrypoint,
        "run_manifest.source.entrypoint",
    )

    site_directories = {
        child.name
        for child in root.iterdir()
        if SITE_DIR_RE.fullmatch(child.name)
    }
    if site_directories != set(parsed_sites):
        raise CollectionValidationError(
            "site directories have missing or unexpected entries: "
            f"expected {sorted(parsed_sites)}, got {sorted(site_directories)}"
        )
    run_sites_raw = _list(run.get("sites"), "run_manifest.sites")
    if len(run_sites_raw) != len(parsed_sites):
        raise CollectionValidationError("run_manifest.sites count mismatch")
    run_sites: dict[str, Mapping[str, Any]] = {}
    for index, raw in enumerate(run_sites_raw):
        item = _mapping(raw, f"run_manifest.sites[{index}]")
        site = _string(item.get("site"), f"run_manifest.sites[{index}].site")
        if site in run_sites:
            raise CollectionValidationError(f"run_manifest.sites duplicates {site}")
        run_sites[site] = item
    if tuple(run_sites) != parsed_sites:
        raise CollectionValidationError(
            "run_manifest.sites has missing, unexpected, duplicate, or reordered sites"
        )

    software_identity_cache: dict[str, dict[str, str]] = {}
    rows = [
        _validate_site(
            root,
            site,
            run_sites[site],
            authority=authority,
            mission_intent=mission_intent,
            contract=contract,
            software_identity_cache=software_identity_cache,
        )
        for site in parsed_sites
    ]
    runtime_source_paths = {row["runtime_source_path"] for row in rows}
    runtime_source_branches = {row["runtime_source_branch"] for row in rows}
    runtime_source_heads = {row["runtime_source_head"] for row in rows}
    _assert_equal(
        runtime_source_paths,
        {str(source_root.resolve(strict=True))},
        "run_manifest.source.git_root",
    )
    _assert_equal(
        runtime_source_branches,
        {_string(source_record.get("branch"), "run_manifest.source.branch")},
        "run_manifest.source.branch",
    )
    _assert_equal(
        runtime_source_heads,
        {_git_object_id(source_record.get("head"), "run_manifest.source.head")},
        "run_manifest.source.head",
    )
    for key, description in (
        ("software_binding_sha256", "software identity binding"),
        ("runtime_install_sha256", "runtime install identity"),
        ("umap_sha256", "v15 UMAP identity"),
        ("lanelet_sha256", "lanelet identity"),
        ("launch_cmdline_sha256", "v27 launch identity"),
    ):
        if len({row[key] for row in rows}) != 1:
            raise CollectionValidationError(f"{description} changed across sites")
    actor_ids = {row["actor_id"] for row in rows}
    if len(actor_ids) != 1:
        raise CollectionValidationError(
            f"physical Ranger actor identity changed across sites: {sorted(actor_ids)}"
        )
    physical_hashes = {row["physical_manifest_sha256"] for row in rows}
    if len(physical_hashes) != 1:
        raise CollectionValidationError(
            "physical 4WS acceptance manifest changed across sites"
        )
    run_wheel_summaries = _mapping(
        run.get("physical_wheel_summaries"), "run_manifest.physical_wheel_summaries"
    )
    if set(run_wheel_summaries) != set(parsed_sites):
        raise CollectionValidationError(
            "run_manifest.physical_wheel_summaries site set mismatch"
        )
    for site in parsed_sites:
        expected = _mapping(run_sites[site].get("physical_wheels"), f"{site}.wheels").get(
            "summary"
        )
        _assert_equal(
            run_wheel_summaries.get(site), expected, f"run_manifest wheel summary {site}"
        )

    metrics = _validate_metrics_summary(root, run, rows, parsed_sites)
    totals = {
        "site_count": len(rows),
        "pass_count": len(rows),
        "fail_count": 0,
        "actor_id": next(iter(actor_ids)),
        "total_elapsed_s": round(sum(row["elapsed_s"] for row in rows), 6),
        "total_odom_distance_m": round(
            sum(row["total_odom_distance_m"] for row in rows), 6
        ),
        "total_collision_events": 0,
        "maximum_final_speed_mps": max(row["final_speed_mps"] for row in rows),
        "all_sites_parking_confirmed": True,
        "all_sites_charging_confirmed": True,
        "all_four_wheels_grounded_for_every_sample": True,
    }
    return {
        "schema": COLLECTION_SCHEMA,
        "status": "PASS",
        "validated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "source": {
            "input_root": str(root),
            "run_manifest": {
                "path": str(run_path),
                "bytes": run_fact["bytes"],
                "sha256": run_fact["sha256"],
            },
            "metrics_summary": metrics,
        },
        "expectation": {
            "sites": list(parsed_sites),
            "authority": authority,
            "mission_intent": mission_intent,
            "actor_type_id": EXPECTED_TYPE_ID,
            "actor_role_name": EXPECTED_ROLE_NAME,
            "motion_backend": EXPECTED_BACKEND,
            "maximum_final_speed_mps": 0.05,
            "collision_event_count": 0,
            "canonical_wheels": list(CANONICAL_WHEELS),
            "wheel_in_air_count": 0,
        },
        "aggregate": totals,
        "sites": rows,
        "validation": {
            "runner_status_pass": run_status == "PASS",
            "runner_prepass_validating": run_status == "VALIDATING",
            "native_matrix_status_pass": True,
            "site_manifest_status_pass": True,
            "site_set_exact_and_unique": True,
            "authority_and_mission_exact": True,
            "collision_graph_discovered_and_zero_events": True,
            "physical_actor_type_role_backend_consistent": True,
            "software_identity_independently_recomputed": True,
            "v15_map_umap_lanelet_identity_verified": True,
            "v27_launch_identity_verified": True,
            "parking_and_charging_confirmed": True,
            "final_speed_at_or_below_0_05_mps": True,
            "canonical_wheels_grounded": True,
            "visual_png_gif_and_hashes_verified": True,
            "metrics_cross_checked": True,
            "input_was_read_only": True,
        },
    }


def _json_bytes(document: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _display(value: Any) -> str:
    if type(value) is bool:
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.6f}".rstrip("0").rstrip(".")
    return str(value)


def _csv_bytes(rows: Sequence[Mapping[str, Any]]) -> bytes:
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS, lineterminator="\n")
    writer.writeheader()
    for row in rows:
        writer.writerow({key: _display(row[key]) for key in CSV_COLUMNS})
    return stream.getvalue().encode("utf-8")


def _markdown_bytes(document: Mapping[str, Any]) -> bytes:
    aggregate = _mapping(document.get("aggregate"), "aggregate")
    expectation = _mapping(document.get("expectation"), "expectation")
    lines = [
        "# CAMROD CARLA site-evidence collection validation",
        "",
        "- Result: `PASS`",
        f"- Sites: {aggregate['site_count']} ({', '.join(expectation['sites'])})",
        f"- Authority / mission: `{expectation['authority']}` / `{expectation['mission_intent']}`",
        f"- Ranger identity: `{expectation['actor_type_id']}` / `{expectation['actor_role_name']}`",
        f"- Physical backend: `{expectation['motion_backend']}`",
        f"- Total elapsed: {aggregate['total_elapsed_s']} s",
        f"- Total odometry distance: {aggregate['total_odom_distance_m']} m",
        "- Collision events: 0; every final speed <= 0.05 m/s; every wheel grounded",
        "",
        "| Site | Result | Elapsed s | Out s | Return s | Total m | Out m | Return m | Final m/s | Actor | Collision pub/events | Wheels/samples | Parked | Charging |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for row in document["sites"]:
        lines.append(
            "| {site} | PASS | {elapsed_s:.3f} | {outbound_duration_s:.3f} | "
            "{return_duration_s:.3f} | {total_odom_distance_m:.3f} | "
            "{outbound_distance_m:.3f} | {return_distance_m:.3f} | "
            "{final_speed_mps:.6f} | {actor_id} | {collision_publisher_count}/0 | "
            "4/{wheel_sample_count} | true | true |".format(**row)
        )
    lines.extend(
        [
            "",
            "Every row is cross-checked against its site manifest, native matrix report, "
            "physical-wheel stream summary, visual capture manifest, and SHA-256 bindings.",
        ]
    )
    return ("\n".join(lines) + "\n").encode("utf-8")


def _validate_output_destination(output_dir: Path, input_root: Path) -> Path:
    if not output_dir.is_absolute():
        raise CollectionValidationError(f"output directory must be absolute: {output_dir}")
    if output_dir == Path("/"):
        raise CollectionValidationError("output directory cannot be /")
    if output_dir.exists() or output_dir.is_symlink():
        try:
            output_stat = output_dir.lstat()
        except OSError as error:
            raise CollectionValidationError(f"cannot inspect output directory: {error}") from None
        if stat.S_ISLNK(output_stat.st_mode) or not stat.S_ISDIR(output_stat.st_mode):
            raise CollectionValidationError("output path must be a real directory")
        try:
            next(output_dir.iterdir())
        except StopIteration:
            pass
        else:
            raise CollectionValidationError(
                f"output directory must be new or empty: {output_dir}"
            )
    normalized = output_dir.resolve(strict=False)
    try:
        normalized.relative_to(input_root.resolve(strict=True))
    except ValueError:
        pass
    else:
        raise CollectionValidationError(
            "output directory must be outside the read-only input root"
        )
    return normalized


def write_outputs(output_dir: Path | str, document: Mapping[str, Any]) -> dict[str, str]:
    """Atomically install four aggregate artifacts outside the input root."""
    input_root = _absolute_path(
        _mapping(document.get("source"), "source").get("input_root"),
        "source.input_root",
    )
    target = _validate_output_destination(Path(output_dir), input_root)
    payloads = {
        "site_evidence_collection.json": _json_bytes(document),
        "site_evidence_collection.csv": _csv_bytes(document["sites"]),
        "site_evidence_collection.md": _markdown_bytes(document),
    }
    hashes = {name: hashlib.sha256(data).hexdigest() for name, data in payloads.items()}
    checksum_payload = "".join(
        f"{hashes[name]}  {name}\n" for name in sorted(hashes)
    ).encode("utf-8")
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(
            tempfile.mkdtemp(prefix=f".{target.name}.tmp-", dir=target.parent)
        )
    except OSError as error:
        raise CollectionValidationError(f"cannot prepare output {target}: {error}") from None
    try:
        for name, payload in payloads.items():
            path = temporary / name
            path.write_bytes(payload)
            with path.open("rb") as stream:
                os.fsync(stream.fileno())
        sums = temporary / "SHA256SUMS"
        sums.write_bytes(checksum_payload)
        with sums.open("rb") as stream:
            os.fsync(stream.fileno())
        directory_fd = os.open(temporary, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        os.replace(temporary, target)
        parent_fd = os.open(target.parent, os.O_RDONLY)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    except OSError as error:
        if temporary.exists():
            shutil.rmtree(temporary)
        raise CollectionValidationError(
            f"cannot atomically install output directory {target}: {error}"
        ) from None
    return hashes


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-root",
        required=True,
        type=Path,
        help="absolute completed run_site_evidence_matrix.sh output root",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="absolute new or empty directory outside the input root",
    )
    parser.add_argument(
        "--sites",
        required=True,
        help="ordered, unique expected site CSV (for example B1,B2,...,B13)",
    )
    parser.add_argument(
        "--authority",
        required=True,
        choices=("operator", "operator-browser", "guest"),
    )
    parser.add_argument(
        "--mission-intent", required=True, choices=("delivery", "recall")
    )
    parser.add_argument(
        "--allow-validating-manifest",
        action="store_true",
        help=(
            "runner-only pre-PASS mode: accept a VALIDATING manifest after "
            "all artifacts exist; independent validation still requires PASS"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        expected_sites = parse_sites(args.sites)
        # Reject an invalid destination before spending time hashing a large
        # collection.  This check creates and removes nothing.
        raw_input = args.input_root
        if not raw_input.is_absolute():
            raise CollectionValidationError(f"input root must be absolute: {raw_input}")
        resolved_input = raw_input.resolve(strict=True)
        _validate_output_destination(args.output_dir, resolved_input)
        document = validate_collection(
            args.input_root,
            expected_sites=expected_sites,
            authority=args.authority,
            mission_intent=args.mission_intent,
            allow_validating_manifest=args.allow_validating_manifest,
        )
        hashes = write_outputs(args.output_dir, document)
    except (CollectionValidationError, OSError) as error:
        print(f"[site-evidence-validator] ERROR: {error}", file=sys.stderr)
        return 2
    print(
        "[site-evidence-validator] PASS "
        f"sites={len(document['sites'])} authority={args.authority} "
        f"mission_intent={args.mission_intent} output={args.output_dir} "
        f"json_sha256={hashes['site_evidence_collection.json']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
