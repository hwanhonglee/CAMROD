#!/usr/bin/env python3
"""Create a fail-closed B1..B13 time/distance evidence summary.

The input is one or more completed ``camping_site_matrix.json`` reports.  An
input report may contain one site (the usual per-site run) or several sites.
The output directory must not already exist.  Four deterministic artifacts
are produced there:

* ``camping_site_metrics.json`` -- lossless machine-readable summary;
* ``camping_site_metrics.csv`` -- one flat row per attempted site;
* ``camping_site_metrics.md`` -- a human-readable table and totals; and
* ``SHA256SUMS`` -- hashes of the three summaries above.

The tool never estimates a missing split.  When explicit outbound/return
fields are absent, it uses the first WAITING_FOR_RETURN_REQUEST milestone as
the measured boundary.  A missing boundary value is emitted as JSON ``null``
and ``UNAVAILABLE`` in CSV/Markdown.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence


SCHEMA = "camrod.virtual_carla.camping_site_metrics_summary.v1"
MATRIX_SCHEMA = "camrod.virtual_carla.camping_site_matrix.v1"
ALL_SITES = tuple(f"B{number}" for number in range(1, 14))
SITE_RE = re.compile(r"^B([1-9]|1[0-3])$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
FINAL_STATUSES = {"PASS", "FAIL"}
IGNORED_SITE_STATUSES = {"NOT_ATTEMPTED"}
SERVICE_MODES = {"turnaround", "roadside_stop"}
WAITING_STATE_ID = 11
WAITING_STATE_NAME = "WAITING_FOR_RETURN_REQUEST"
SPLIT_KEYS = (
    "outbound_duration_s",
    "return_duration_s",
    "outbound_distance_m",
    "return_distance_m",
)
CSV_COLUMNS = (
    "site",
    "status",
    "service_mode",
    "total_elapsed_s",
    "outbound_duration_s",
    "return_duration_s",
    "total_odom_distance_m",
    "outbound_distance_m",
    "return_distance_m",
    "drop_zone_error_m",
    "final_service_state",
    "parking_confirmed",
    "charging_confirmed",
    "actor_id",
    "gate_manifest_sha256",
    "physical_manifest_sha256",
    "split_duration_source",
    "split_distance_source",
    "source_report",
    "source_report_sha256",
)
GATE_HASH_KEYS = {
    "baseline_gate_manifest_sha256",
    "baseline_gate_sha256",
    "gate_manifest_sha256",
    "verified_baseline_manifest_sha256",
    "baseline_manifest_sha256",
}


class SummaryError(RuntimeError):
    """A fail-closed input, consistency, or output error."""


def _duplicate_key_rejector(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise SummaryError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SummaryError(f"{name} must be a JSON object")
    return value


def _finite_nonnegative(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise SummaryError(f"{name} must be a non-negative finite number")
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise SummaryError(
            f"{name} must be a non-negative finite number, got {value!r}"
        ) from None
    if not math.isfinite(number) or number < 0.0:
        raise SummaryError(
            f"{name} must be a non-negative finite number, got {value!r}"
        )
    return round(number, 6)


def _optional_finite_nonnegative(value: Any, name: str) -> float | None:
    if value is None:
        return None
    return _finite_nonnegative(value, name)


def _required_bool(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise SummaryError(f"{name} must be a JSON boolean")
    return value


def _required_sha256(value: Any, name: str) -> str:
    digest = str(value).strip().lower()
    if not SHA256_RE.fullmatch(digest):
        raise SummaryError(f"{name} must be a lowercase 64-character SHA256")
    return digest


def _read_report(path: Path) -> tuple[Mapping[str, Any], str]:
    if not path.is_file():
        raise SummaryError(f"source report is not a regular file: {path}")
    try:
        payload = path.read_bytes()
        text = payload.decode("utf-8")
        decoded = json.loads(text, object_pairs_hook=_duplicate_key_rejector)
    except SummaryError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SummaryError(f"cannot decode source report {path}: {error}") from None
    return _mapping(decoded, str(path)), hashlib.sha256(payload).hexdigest()


def _walk_hashes(value: Any, accepted_keys: set[str]) -> Iterable[tuple[str, Any]]:
    if isinstance(value, Mapping):
        for key, child in value.items():
            normalized = str(key).lower()
            if normalized in accepted_keys:
                yield normalized, child
            yield from _walk_hashes(child, accepted_keys)
    elif isinstance(value, list):
        for child in value:
            yield from _walk_hashes(child, accepted_keys)


def _consistent_hash(
    value: Any,
    accepted_keys: set[str],
    name: str,
) -> str | None:
    found = {
        _required_sha256(candidate, f"{name}.{key}")
        for key, candidate in _walk_hashes(value, accepted_keys)
    }
    if len(found) > 1:
        raise SummaryError(f"inconsistent {name} hashes: {sorted(found)}")
    return next(iter(found), None)


def _configuration_hashes(report: Mapping[str, Any], source: str) -> dict[str, str]:
    active = _mapping(report.get("active_configuration"), f"{source}.active_configuration")
    hashes = _mapping(active.get("sha256"), f"{source}.active_configuration.sha256")
    return {
        "camping_sites_sha256": _required_sha256(
            hashes.get("camping_sites"),
            f"{source}.active_configuration.sha256.camping_sites",
        ),
        "drop_zones_sha256": _required_sha256(
            hashes.get("drop_zones"),
            f"{source}.active_configuration.sha256.drop_zones",
        ),
    }


def _selected_sites(report: Mapping[str, Any], source: str) -> tuple[str, ...]:
    scope = _mapping(report.get("scope"), f"{source}.scope")
    selected = scope.get("selected_sites")
    if not isinstance(selected, list) or not selected:
        raise SummaryError(f"{source}.scope.selected_sites must be a non-empty list")
    normalized: list[str] = []
    for index, raw in enumerate(selected):
        site = str(raw).strip().upper()
        if not SITE_RE.fullmatch(site):
            raise SummaryError(
                f"{source}.scope.selected_sites[{index}] is not B1..B13: {raw!r}"
            )
        if site in normalized:
            raise SummaryError(f"{source}.scope.selected_sites contains duplicate {site}")
        normalized.append(site)
    return tuple(normalized)


def _waiting_milestone(site: Mapping[str, Any], name: str) -> Mapping[str, Any] | None:
    milestones = site.get("milestones")
    if not isinstance(milestones, list):
        return None
    mappings: list[Mapping[str, Any]] = []
    for index, raw in enumerate(milestones):
        if not isinstance(raw, Mapping):
            raise SummaryError(f"{name}.milestones[{index}] must be a JSON object")
        mappings.append(raw)

    named = [
        item
        for item in mappings
        if WAITING_STATE_NAME in str(item.get("event", "")).upper()
    ]
    candidates = named
    if not candidates:
        candidates = []
        for item in mappings:
            observation = item.get("observation")
            if not isinstance(observation, Mapping):
                continue
            service_state = observation.get("service_state")
            if not isinstance(service_state, Mapping):
                continue
            if (
                service_state.get("state") == WAITING_STATE_ID
                or str(service_state.get("state_name", "")).upper()
                == WAITING_STATE_NAME
            ):
                candidates.append(item)
    if not candidates:
        return None

    measured: list[tuple[float, Mapping[str, Any]]] = []
    for item in candidates:
        if item.get("elapsed_s") is None:
            continue
        measured.append(
            (
                _finite_nonnegative(
                    item.get("elapsed_s"), f"{name}.waiting.elapsed_s"
                ),
                item,
            )
        )
    if measured:
        return min(measured, key=lambda pair: pair[0])[1]
    # The boundary itself is still useful provenance.  Its absent measurement
    # is handled as UNAVAILABLE by _derive_splits rather than fabricated.
    return candidates[0]


def _validate_split_sum(total: float, first: float, second: float, name: str) -> None:
    tolerance = max(0.02, total * 1e-6)
    if abs(total - (first + second)) > tolerance:
        raise SummaryError(
            f"inconsistent {name}: total={total}, outbound={first}, return={second}"
        )


def _derive_splits(
    site: Mapping[str, Any],
    name: str,
    total_elapsed_s: float,
    total_distance_m: float,
) -> dict[str, Any]:
    explicit = [key in site for key in SPLIT_KEYS]
    if any(explicit):
        if not all(explicit):
            missing = [key for key in SPLIT_KEYS if key not in site]
            raise SummaryError(f"{name} has a partial explicit split; missing {missing}")
        outbound_duration = _finite_nonnegative(
            site["outbound_duration_s"], f"{name}.outbound_duration_s"
        )
        return_duration = _finite_nonnegative(
            site["return_duration_s"], f"{name}.return_duration_s"
        )
        outbound_distance = _finite_nonnegative(
            site["outbound_distance_m"], f"{name}.outbound_distance_m"
        )
        return_distance = _finite_nonnegative(
            site["return_distance_m"], f"{name}.return_distance_m"
        )
        _validate_split_sum(
            total_elapsed_s, outbound_duration, return_duration, f"{name} duration split"
        )
        _validate_split_sum(
            total_distance_m, outbound_distance, return_distance, f"{name} distance split"
        )
        return {
            "outbound_duration_s": outbound_duration,
            "return_duration_s": return_duration,
            "outbound_distance_m": outbound_distance,
            "return_distance_m": return_distance,
            "split_duration_source": "explicit",
            "split_distance_source": "explicit",
        }

    boundary = _waiting_milestone(site, name)
    if boundary is None:
        return {
            "outbound_duration_s": None,
            "return_duration_s": None,
            "outbound_distance_m": None,
            "return_distance_m": None,
            "split_duration_source": "UNAVAILABLE: WAITING_FOR_RETURN_REQUEST milestone missing",
            "split_distance_source": "UNAVAILABLE: WAITING_FOR_RETURN_REQUEST milestone missing",
        }

    outbound_duration: float | None = None
    duration_source = "UNAVAILABLE: waiting milestone elapsed_s missing"
    if boundary.get("elapsed_s") is not None:
        outbound_duration = _finite_nonnegative(
            boundary.get("elapsed_s"), f"{name}.waiting.elapsed_s"
        )
        if outbound_duration > total_elapsed_s + 1e-6:
            raise SummaryError(
                f"inconsistent {name} duration boundary: {outbound_duration} > {total_elapsed_s}"
            )
        duration_source = "WAITING_FOR_RETURN_REQUEST milestone"

    outbound_distance: float | None = None
    distance_source = "UNAVAILABLE: waiting milestone odometry distance missing"
    observation = boundary.get("observation")
    if isinstance(observation, Mapping):
        metrics = observation.get("motion_metrics")
        if isinstance(metrics, Mapping) and metrics.get("carla_odometry_distance_m") is not None:
            outbound_distance = _finite_nonnegative(
                metrics.get("carla_odometry_distance_m"),
                f"{name}.waiting.observation.motion_metrics.carla_odometry_distance_m",
            )
            if outbound_distance > total_distance_m + 1e-6:
                raise SummaryError(
                    f"inconsistent {name} distance boundary: "
                    f"{outbound_distance} > {total_distance_m}"
                )
            distance_source = "WAITING_FOR_RETURN_REQUEST milestone"

    return {
        "outbound_duration_s": outbound_duration,
        "return_duration_s": (
            round(total_elapsed_s - outbound_duration, 6)
            if outbound_duration is not None
            else None
        ),
        "outbound_distance_m": outbound_distance,
        "return_distance_m": (
            round(total_distance_m - outbound_distance, 6)
            if outbound_distance is not None
            else None
        ),
        "split_duration_source": duration_source,
        "split_distance_source": distance_source,
    }


def _site_row(
    site: Mapping[str, Any],
    source_path: Path,
    source_sha256: str,
    gate_hash: str | None,
) -> dict[str, Any]:
    raw_site = site.get("site")
    site_name = str(raw_site).strip().upper()
    name = f"{source_path}:{site_name or '<missing-site>'}"
    if not SITE_RE.fullmatch(site_name):
        raise SummaryError(f"{name}.site must be B1..B13, got {raw_site!r}")
    status = str(site.get("status", "")).strip().upper()
    if status not in FINAL_STATUSES:
        raise SummaryError(f"{name}.status must be PASS or FAIL, got {status!r}")
    service_mode = str(site.get("service_mode", "")).strip()
    if service_mode not in SERVICE_MODES:
        raise SummaryError(
            f"{name}.service_mode must be one of {sorted(SERVICE_MODES)}, "
            f"got {service_mode!r}"
        )
    total_elapsed = _finite_nonnegative(site.get("elapsed_s"), f"{name}.elapsed_s")
    motion_metrics = _mapping(site.get("motion_metrics"), f"{name}.motion_metrics")
    total_distance = _finite_nonnegative(
        motion_metrics.get("carla_odometry_distance_m"),
        f"{name}.motion_metrics.carla_odometry_distance_m",
    )
    actor_id = site.get("actor_id")
    if isinstance(actor_id, bool) or not isinstance(actor_id, int) or actor_id <= 0:
        raise SummaryError(f"{name}.actor_id must be a positive integer")
    parking = _required_bool(site.get("parking_confirmed"), f"{name}.parking_confirmed")
    charging = _required_bool(site.get("charging_confirmed"), f"{name}.charging_confirmed")
    final_state = _mapping(site.get("final_service_state"), f"{name}.final_service_state")
    final_state_name = str(final_state.get("state_name", "")).strip().upper()
    if not final_state_name:
        raise SummaryError(f"{name}.final_service_state.state_name is required")
    drop_zone_error = _optional_finite_nonnegative(
        site.get("drop_zone_error_m"), f"{name}.drop_zone_error_m"
    )

    final_physical = site.get("final_physical_four_wheel_status")
    if isinstance(final_physical, Mapping) and final_physical.get("actor_id") is not None:
        if final_physical.get("actor_id") != actor_id:
            raise SummaryError(
                f"inconsistent {name} actor_id: site={actor_id}, "
                f"physical={final_physical.get('actor_id')!r}"
            )
    physical_hash = _consistent_hash(
        site, {"physical_manifest_sha256"}, f"{name}.physical_manifest"
    )

    if status == "PASS":
        if final_state_name != "CHARGING":
            raise SummaryError(
                f"inconsistent {name} PASS final_service_state={final_state_name}"
            )
        if not parking or not charging:
            raise SummaryError(
                f"inconsistent {name} PASS parking/charging="
                f"{parking}/{charging}"
            )
        if drop_zone_error is None:
            raise SummaryError(f"{name}.drop_zone_error_m is required for PASS")

    split = _derive_splits(site, name, total_elapsed, total_distance)
    return {
        "site": site_name,
        "status": status,
        "service_mode": service_mode,
        "total_elapsed_s": total_elapsed,
        **split,
        "total_odom_distance_m": total_distance,
        "drop_zone_error_m": drop_zone_error,
        "final_service_state": final_state_name,
        "parking_confirmed": parking,
        "charging_confirmed": charging,
        "actor_id": actor_id,
        "gate_manifest_sha256": gate_hash,
        "physical_manifest_sha256": physical_hash,
        "source_report": str(source_path),
        "source_report_sha256": source_sha256,
    }


def build_summary(
    report_paths: Sequence[Path],
    *,
    require_all_sites: bool = False,
) -> dict[str, Any]:
    """Validate reports and return the deterministic summary document."""
    if not report_paths:
        raise SummaryError("at least one camping_site_matrix.json is required")
    resolved = [path.expanduser().resolve() for path in report_paths]
    if len(set(resolved)) != len(resolved):
        raise SummaryError("duplicate source report path")

    rows_by_site: dict[str, dict[str, Any]] = {}
    sources: list[dict[str, Any]] = []
    common_configuration: dict[str, str] | None = None
    observed_gate_hashes: set[str] = set()
    observed_physical_hashes: set[str] = set()

    for path in resolved:
        report, source_hash = _read_report(path)
        if report.get("schema") != MATRIX_SCHEMA:
            raise SummaryError(
                f"{path}.schema must be {MATRIX_SCHEMA!r}, got {report.get('schema')!r}"
            )
        report_status = str(report.get("status", "")).strip().upper()
        if report_status not in FINAL_STATUSES:
            raise SummaryError(
                f"{path}.status must be a completed PASS or FAIL, got {report_status!r}"
            )
        selected = _selected_sites(report, str(path))
        configuration = _configuration_hashes(report, str(path))
        if common_configuration is None:
            common_configuration = configuration
        elif configuration != common_configuration:
            raise SummaryError(
                f"inconsistent active configuration hashes in {path}: "
                f"{configuration} != {common_configuration}"
            )
        gate_hash = _consistent_hash(report, GATE_HASH_KEYS, f"{path}.gate_manifest")
        if gate_hash:
            observed_gate_hashes.add(gate_hash)

        raw_sites = report.get("sites")
        if not isinstance(raw_sites, list) or not raw_sites:
            raise SummaryError(f"{path}.sites must be a non-empty list")
        seen_in_report: set[str] = set()
        finalized_in_report: list[str] = []
        failed_in_report = False
        for index, raw in enumerate(raw_sites):
            site = _mapping(raw, f"{path}.sites[{index}]")
            site_name = str(site.get("site", "")).strip().upper()
            if not SITE_RE.fullmatch(site_name):
                raise SummaryError(
                    f"{path}.sites[{index}].site must be B1..B13, got {site.get('site')!r}"
                )
            if site_name in seen_in_report:
                raise SummaryError(f"duplicate site {site_name} inside {path}")
            seen_in_report.add(site_name)
            if site_name not in selected:
                raise SummaryError(f"inconsistent {path}: site {site_name} was not selected")
            site_status = str(site.get("status", "")).strip().upper()
            if site_status in IGNORED_SITE_STATUSES:
                continue
            if site_status not in FINAL_STATUSES:
                raise SummaryError(
                    f"{path}:{site_name}.status must be PASS/FAIL/NOT_ATTEMPTED, "
                    f"got {site_status!r}"
                )
            row = _site_row(site, path, source_hash, gate_hash)
            if site_name in rows_by_site:
                previous = rows_by_site[site_name]["source_report"]
                raise SummaryError(
                    f"duplicate finalized site {site_name}: {previous} and {path}"
                )
            rows_by_site[site_name] = row
            finalized_in_report.append(site_name)
            failed_in_report = failed_in_report or site_status == "FAIL"
            if row["physical_manifest_sha256"]:
                observed_physical_hashes.add(row["physical_manifest_sha256"])

        if not finalized_in_report:
            raise SummaryError(f"{path} contains no finalized PASS/FAIL site")
        if report_status == "PASS" and (
            failed_in_report or set(finalized_in_report) != set(selected)
        ):
            raise SummaryError(
                f"inconsistent {path}: PASS report does not finalize every selected site"
            )
        if report_status == "FAIL" and not failed_in_report:
            raise SummaryError(f"inconsistent {path}: FAIL report contains no failed site")
        sources.append(
            {
                "path": str(path),
                "sha256": source_hash,
                "report_status": report_status,
                "finalized_sites": sorted(
                    finalized_in_report, key=lambda item: int(item[1:])
                ),
            }
        )

    if len(observed_gate_hashes) > 1:
        raise SummaryError(
            f"inconsistent gate manifest hashes across reports: {sorted(observed_gate_hashes)}"
        )
    if len(observed_physical_hashes) > 1:
        raise SummaryError(
            "inconsistent physical manifest hashes across reports: "
            f"{sorted(observed_physical_hashes)}"
        )

    rows = sorted(rows_by_site.values(), key=lambda row: int(row["site"][1:]))
    covered_sites = tuple(row["site"] for row in rows)
    missing_sites = [site for site in ALL_SITES if site not in covered_sites]
    if require_all_sites and missing_sites:
        raise SummaryError(
            "B1..B13 coverage is incomplete; missing finalized sites: "
            + ", ".join(missing_sites)
        )

    duration_complete = all(row["outbound_duration_s"] is not None for row in rows)
    distance_complete = all(row["outbound_distance_m"] is not None for row in rows)
    aggregate = {
        "site_count": len(rows),
        "pass_count": sum(row["status"] == "PASS" for row in rows),
        "fail_count": sum(row["status"] == "FAIL" for row in rows),
        "missing_sites": missing_sites,
        "total_elapsed_s": round(sum(row["total_elapsed_s"] for row in rows), 6),
        "total_odom_distance_m": round(
            sum(row["total_odom_distance_m"] for row in rows), 6
        ),
        "outbound_duration_s": (
            round(sum(row["outbound_duration_s"] for row in rows), 6)
            if duration_complete
            else None
        ),
        "return_duration_s": (
            round(sum(row["return_duration_s"] for row in rows), 6)
            if duration_complete
            else None
        ),
        "outbound_distance_m": (
            round(sum(row["outbound_distance_m"] for row in rows), 6)
            if distance_complete
            else None
        ),
        "return_distance_m": (
            round(sum(row["return_distance_m"] for row in rows), 6)
            if distance_complete
            else None
        ),
        "split_duration_complete": duration_complete,
        "split_distance_complete": distance_complete,
    }
    assert common_configuration is not None
    return {
        "schema": SCHEMA,
        "coverage": "B1_B13_COMPLETE" if not missing_sites else "PARTIAL",
        "configuration": common_configuration,
        "shared_gate_manifest_sha256": next(iter(observed_gate_hashes), None),
        "shared_physical_manifest_sha256": next(
            iter(observed_physical_hashes), None
        ),
        "aggregate": aggregate,
        "sites": rows,
        "source_reports": sorted(sources, key=lambda source: source["path"]),
    }


def _json_bytes(summary: Mapping[str, Any]) -> bytes:
    return (
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")


def _display(value: Any) -> str:
    if value is None:
        return "UNAVAILABLE"
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
        writer.writerow({column: _display(row.get(column)) for column in CSV_COLUMNS})
    return stream.getvalue().encode("utf-8")


def _md(value: Any) -> str:
    return _display(value).replace("\\", "\\\\").replace("|", "\\|").replace("\n", " ")


def _markdown_bytes(summary: Mapping[str, Any]) -> bytes:
    aggregate = _mapping(summary["aggregate"], "aggregate")
    rows = summary["sites"]
    lines = [
        "# CAMROD CARLA camping-site time and distance summary",
        "",
        f"- Coverage: `{summary['coverage']}`",
        f"- Sites: {aggregate['site_count']} (PASS {aggregate['pass_count']}, FAIL {aggregate['fail_count']})",
        f"- Total elapsed: {_display(aggregate['total_elapsed_s'])} s",
        f"- Total odometry distance: {_display(aggregate['total_odom_distance_m'])} m",
        "",
        "`UNAVAILABLE` means the source evidence did not contain a measured split; no value was estimated.",
        "",
        "| Site | Result | Mode | Total s | Outbound s | Return s | Total m | Outbound m | Return m | Drop-zone error m | Final state | Parked | Charging | Actor | Gate manifest | Physical manifest | Source report SHA256 |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|---:|---|---|---|",
    ]
    for row in rows:
        values = (
            row["site"],
            row["status"],
            row["service_mode"],
            row["total_elapsed_s"],
            row["outbound_duration_s"],
            row["return_duration_s"],
            row["total_odom_distance_m"],
            row["outbound_distance_m"],
            row["return_distance_m"],
            row["drop_zone_error_m"],
            row["final_service_state"],
            row["parking_confirmed"],
            row["charging_confirmed"],
            row["actor_id"],
            row["gate_manifest_sha256"],
            row["physical_manifest_sha256"],
            row["source_report_sha256"],
        )
        lines.append("| " + " | ".join(_md(value) for value in values) + " |")
    lines.extend(
        [
            "",
            "## Split provenance",
            "",
            "| Site | Duration split | Distance split | Source report |",
            "|---|---|---|---|",
        ]
    )
    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                _md(value)
                for value in (
                    row["site"],
                    row["split_duration_source"],
                    row["split_distance_source"],
                    row["source_report"],
                )
            )
            + " |"
        )
    return ("\n".join(lines) + "\n").encode("utf-8")


def write_summary(output_dir: Path, summary: Mapping[str, Any]) -> dict[str, str]:
    """Atomically install a new evidence directory and return artifact hashes."""
    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise SummaryError(f"output directory already exists: {output_dir}")
    parent = output_dir.parent
    try:
        parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}.tmp-", dir=parent))
    except OSError as error:
        raise SummaryError(f"cannot prepare output directory {output_dir}: {error}") from None

    artifacts = {
        "camping_site_metrics.json": _json_bytes(summary),
        "camping_site_metrics.csv": _csv_bytes(summary["sites"]),
        "camping_site_metrics.md": _markdown_bytes(summary),
    }
    hashes = {
        name: hashlib.sha256(payload).hexdigest()
        for name, payload in artifacts.items()
    }
    sums = "".join(f"{digest}  {name}\n" for name, digest in sorted(hashes.items()))
    try:
        for name, payload in artifacts.items():
            (temporary / name).write_bytes(payload)
        (temporary / "SHA256SUMS").write_text(sums, encoding="utf-8", newline="\n")
        os.replace(temporary, output_dir)
    except OSError as error:
        for child in temporary.iterdir():
            child.unlink()
        temporary.rmdir()
        raise SummaryError(f"cannot write output directory {output_dir}: {error}") from None
    return hashes


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "reports",
        type=Path,
        nargs="+",
        help="one or more completed camping_site_matrix.json reports",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help="new (non-existing) directory for JSON, CSV, Markdown, and SHA256SUMS",
    )
    parser.add_argument(
        "--require-all-sites",
        action="store_true",
        help="reject unless finalized B1 through B13 evidence is present",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        summary = build_summary(
            args.reports,
            require_all_sites=args.require_all_sites,
        )
        hashes = write_summary(args.output_dir, summary)
    except SummaryError as error:
        print(f"[camping-site-metrics] ERROR: {error}", file=sys.stderr)
        return 2
    print(
        f"[camping-site-metrics] wrote {len(summary['sites'])} site rows to "
        f"{args.output_dir} (JSON sha256={hashes['camping_site_metrics.json']})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
