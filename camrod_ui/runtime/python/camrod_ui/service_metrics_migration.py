"""Copy and verify a service-metrics schema migration; never activate a database.

Run with ``python -m camrod_ui.service_metrics_migration --source FILE
--output-dir NEW_DIRECTORY``. The source is opened read-only. ``backup.sqlite3``
is an online SQLite snapshot, not a byte copy of a potentially changing WAL
database; only ``migrated.sqlite3`` is opened by ServiceMetricsTracker.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sqlite3
import stat
from contextlib import closing
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .service_metrics import SERVICE_METRICS_SCHEMA_VERSION, ServiceMetricsTracker


REQUIRED_COLUMNS = {
    "id", "service_date", "site", "mission_key", "source", "started_at",
    "ended_at", "result", "distance_m", "last_state", "last_state_name",
    "updated_at",
}


class MigrationRejected(ValueError):
    """The input/path contract is invalid; no migration should be attempted."""


def _path_without_symlinks(value: str | Path, *, must_exist: bool) -> Path:
    path = Path(value).expanduser().absolute()
    if ".." in path.parts:
        raise MigrationRejected("parent traversal is not accepted")
    for component in (path, *path.parents):
        try:
            info = component.lstat()
        except FileNotFoundError:
            if component == path and not must_exist:
                continue
            raise MigrationRejected(f"path does not exist: {component}") from None
        if stat.S_ISLNK(info.st_mode):
            raise MigrationRejected(f"symlink paths are not accepted: {component}")
    return path


def _readonly(path: Path) -> sqlite3.Connection:
    connection = sqlite3.connect(path.as_uri() + "?mode=ro", uri=True, timeout=5.0)
    connection.execute("PRAGMA query_only=ON")
    return connection


def _quote(identifier: str) -> str:
    return '"' + identifier.replace('"', '""') + '"'


def _integrity(connection: sqlite3.Connection) -> None:
    messages = [row[0] for row in connection.execute("PRAGMA integrity_check")]
    if messages != ["ok"]:
        raise MigrationRejected("SQLite integrity_check failed: " + repr(messages))


def _validate_store(connection: sqlite3.Connection) -> None:
    version = connection.execute("PRAGMA user_version").fetchone()[0]
    if version > SERVICE_METRICS_SCHEMA_VERSION:
        raise MigrationRejected(f"newer database schema is not supported: {version}")
    table = connection.execute(
        "SELECT type FROM sqlite_master WHERE name='service_runs'"
    ).fetchone()
    columns = {row[1] for row in connection.execute("PRAGMA table_info(service_runs)")}
    if table != ("table",) or not REQUIRED_COLUMNS.issubset(columns):
        raise MigrationRejected("not a supported service_metrics database")
    _integrity(connection)


def _typed(value: Any) -> list[Any]:
    # Preserve SQL value types and the exact float representation, not rounded
    # JSON numbers. The row digest includes every existing column and value.
    if value is None:
        return ["null", None]
    if isinstance(value, bytes):
        return ["blob", value.hex()]
    if isinstance(value, float):
        return ["real", value.hex()]
    if isinstance(value, int):
        return ["integer", str(value)]
    return ["text", value]


def _state(connection: sqlite3.Connection, original_columns=None) -> dict[str, Any]:
    tables = {}
    names = [row[0] for row in connection.execute(
        "SELECT name FROM sqlite_master WHERE type='table' "
        "AND name NOT LIKE 'sqlite_%' ORDER BY name"
    )]
    if original_columns is not None:
        if not set(original_columns).issubset(names):
            raise MigrationRejected("an original table disappeared")
        names = sorted(original_columns)
    for name in names:
        available = [row[1] for row in connection.execute(f"PRAGMA table_info({_quote(name)})")]
        columns = available if original_columns is None else original_columns[name]
        if not set(columns).issubset(available):
            raise MigrationRejected(f"original columns disappeared in {name}")
        rows = connection.execute(
            f"SELECT {','.join(map(_quote, columns))} FROM {_quote(name)}"
        ).fetchall()
        canonical = sorted(json.dumps([_typed(value) for value in row],
                                      ensure_ascii=True, separators=(",", ":"))
                           for row in rows)
        tables[name] = {
            "columns": columns,
            "row_count": len(rows),
            "typed_rows_sha256": hashlib.sha256(
                json.dumps(canonical, separators=(",", ":")).encode("utf-8")
            ).hexdigest(),
        }
    distances = [row[0] for row in connection.execute("SELECT distance_m FROM service_runs")]
    if any(not isinstance(value, (float, int)) or not math.isfinite(value)
           or value < 0 for value in distances):
        raise MigrationRejected("distance_m must contain finite nonnegative numbers")
    return {
        "schema_version": connection.execute("PRAGMA user_version").fetchone()[0],
        "tables": tables,
        "raw_total_distance_m": math.fsum(distances),
    }


def _same_original_values(before: dict, after: dict) -> bool:
    return (before["tables"] == after["tables"]
            and before["raw_total_distance_m"] == after["raw_total_distance_m"])


def _backup_database(connection: sqlite3.Connection, destination: Path) -> None:
    # The enclosing private output directory is newly and exclusively created.
    # x mode prevents accidentally reusing either named output artifact.
    with destination.open("xb"):
        pass
    with closing(sqlite3.connect(str(destination))) as output:
        connection.backup(output)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _legacy_interpretation(connection: sqlite3.Connection) -> dict:
    connection.row_factory = sqlite3.Row
    records = [ServiceMetricsTracker._row_to_record(row) for row in
               connection.execute("SELECT * FROM service_runs")]
    legacy = [record for record in records if any(
        segment["phase"] == "LEGACY_UNCLASSIFIED" for segment in record["segments"]
    )]
    return {
        "legacy_unclassified_row_count": len(legacy),
        "legacy_unknown_distance_m": math.fsum(
            segment["distance_m"] for record in legacy for segment in record["segments"]
            if segment["phase"] == "LEGACY_UNCLASSIFIED"
        ),
        "unknown_segment_distance_m": math.fsum(
            segment["distance_m"] for record in records for segment in record["segments"]
            if segment["kind"] == "unknown"
        ),
        "basis": "tracker read interpretation; no source/site/intent inference or reclassification",
    }


def migrate_copy(source: str | Path, output_dir: str | Path) -> dict[str, Any]:
    """Create two private copies and a report; FAILED copies are never activated."""
    source = _path_without_symlinks(source, must_exist=True)
    if not source.is_file():
        raise MigrationRejected("source must be a regular existing file")
    output_dir = _path_without_symlinks(output_dir, must_exist=False)
    if output_dir.exists():
        raise MigrationRejected("output directory must not exist")
    report = {
        "schema": "camrod.service_metrics.copy_migration_report.v1",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_path": str(source),
        "output_directory": str(output_dir),
        "source_open_mode": "SQLite URI mode=ro; query_only; read transaction",
        "source_bytes_unchanged_claimed": False,
        "source_provenance": "not inferred; no physical/simulator classification",
        "snapshot_basis": "one SQLite read transaction and online backup; concurrent writers may change the live source",
        "activated": False,
        "status": "FAILED",
        "artifacts": {},
    }
    # Validate before creating output. BEGIN pins the same source snapshot for
    # the canonical read and online backup, including committed WAL contents.
    try:
        connection = _readonly(source)
        try:
            connection.execute("BEGIN")
            _validate_store(connection)
            before = _state(connection)
            output_dir.mkdir(mode=0o700, exist_ok=False)
            backup = output_dir / "backup.sqlite3"
            migrated = output_dir / "migrated.sqlite3"
            report["source_snapshot"] = before
            _backup_database(connection, backup)
        finally:
            connection.close()
    except (sqlite3.Error, OSError, ValueError) as exc:
        if not output_dir.is_dir():
            raise MigrationRejected(str(exc)) from exc
        # Only own an output directory created by this invocation. If mkdir
        # lost a race, never write into the winning caller's directory.
        if "source_snapshot" not in report:
            raise MigrationRejected(str(exc)) from exc
        report["error"] = str(exc)

    if "error" not in report:
        try:
            with closing(_readonly(backup)) as snapshot:
                _integrity(snapshot)
                copied = _state(snapshot)
                if before != copied:
                    raise MigrationRejected("backup does not match the pinned source snapshot")
                report["backup_snapshot"] = copied
                report["backup_matches_source_snapshot"] = True
                _backup_database(snapshot, migrated)
            tracker = ServiceMetricsTracker(migrated)
            try:
                if not tracker.persistence_enabled or tracker.persistence_error:
                    raise MigrationRejected(tracker.persistence_error or "tracker migration failed")
                snapshot_data = tracker.snapshot(days=30, recent_limit=500)
            finally:
                tracker.close()
            if tracker.persistence_error:
                raise MigrationRejected(tracker.persistence_error)
            with closing(_readonly(migrated)) as converted:
                _integrity(converted)
                original_columns = {name: table["columns"] for name, table in before["tables"].items()}
                after_original = _state(converted, original_columns)
                after = _state(converted)
                report["migrated_snapshot"] = after
                report["original_values_preserved"] = _same_original_values(before, after_original)
                report["columns_added"] = {
                    name: [column for column in table["columns"]
                           if column not in original_columns.get(name, [])]
                    for name, table in after["tables"].items()
                }
                if not report["original_values_preserved"]:
                    raise MigrationRejected("existing rows/columns/values changed, possibly active-record recovery; migrated copy is NOT approved")
                if after["schema_version"] != SERVICE_METRICS_SCHEMA_VERSION:
                    raise MigrationRejected("migration did not produce the supported schema")
                report["legacy_unknown"] = _legacy_interpretation(converted)
            # Export only after the database's old values have passed the exact
            # comparison. This is local historical data, not proof of a real
            # robot run or of separation from simulator history.
            with (output_dir / "snapshot.json").open("x", encoding="utf-8") as stream:
                json.dump(snapshot_data, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
            report["snapshot_export"] = {
                "days": 30, "recent_limit": 500,
                "scope": "local historical snapshot; physical/simulator provenance not established",
                "contains_original_record_identifiers": True,
            }
            report["integrity_check"] = "ok (source snapshot, backup, migrated)"
            report["status"] = "PASS"
        except (sqlite3.Error, OSError, ValueError, TypeError, KeyError) as exc:
            report["error"] = str(exc)

    for name in ("backup.sqlite3", "migrated.sqlite3", "snapshot.json"):
        artifact = output_dir / name
        if artifact.is_file() and not artifact.is_symlink():
            report["artifacts"][name] = {"bytes": artifact.stat().st_size, "sha256": _sha256(artifact)}
    with (output_dir / "report.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        report = migrate_copy(args.source, args.output_dir)
    except (MigrationRejected, OSError, sqlite3.Error) as exc:
        print(json.dumps({"status": "REJECTED", "error": str(exc)}, ensure_ascii=False))
        return 2
    print(json.dumps({"status": report["status"],
                      "report": str(args.output_dir.absolute() / "report.json"),
                      "original_values_preserved": report.get("original_values_preserved", False),
                      "schema_version": report.get("migrated_snapshot", {}).get("schema_version"),
                      "record_count": report.get("source_snapshot", {}).get("tables", {}).get("service_runs", {}).get("row_count"),
                      "raw_total_distance_m": report.get("source_snapshot", {}).get("raw_total_distance_m"),
                      "activated": False}, ensure_ascii=False))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
