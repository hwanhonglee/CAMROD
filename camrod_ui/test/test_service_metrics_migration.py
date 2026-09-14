"""Fixture-only tests: no runtime, default database, or production paths."""

import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))
from camrod_ui import service_metrics_migration as migration  # noqa: E402
from camrod_ui.service_metrics import ServiceMetricsTracker  # noqa: E402


DDL = """CREATE TABLE service_runs (
    id TEXT PRIMARY KEY, service_date TEXT NOT NULL, site TEXT NOT NULL,
    mission_key TEXT NOT NULL, source TEXT NOT NULL, started_at REAL NOT NULL,
    ended_at REAL, result TEXT NOT NULL, distance_m REAL NOT NULL,
    last_state INTEGER, last_state_name TEXT NOT NULL, updated_at REAL NOT NULL
)"""


def fixture_database(path, *, version=1, rows=2, active=False):
    with sqlite3.connect(path) as db:
        db.execute(DDL)
        db.execute(f"PRAGMA user_version={version}")
        for index in range(rows):
            db.execute("INSERT INTO service_runs VALUES (?,?,?,?,?,?,?,?,?,?,?,?)", (
                f"svc-{index}", "2026-09-14", f"B{index + 1}", "mission", "unclassified source",
                1000.0 + index, None if active else 1020.0 + index,
                "active" if active else "completed", 3.810441 + index,
                5 if active else 0, "SITE_ENTRY" if active else "DROP_ZONE_WAIT", 1021.0 + index,
            ))
    return path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_legacy_copy_preserves_every_original_value_and_reports_unknown(tmp_path):
    source = fixture_database(tmp_path / "source #?.sqlite3")
    source_hash = digest(source)
    output = tmp_path / "converted"
    result = migration.migrate_copy(source, output)
    assert result["status"] == "PASS"
    assert result["activated"] is False
    assert result["source_bytes_unchanged_claimed"] is False
    assert result["source_provenance"].startswith("not inferred")
    assert digest(source) == source_hash  # quiescent fixture only
    assert result["backup_matches_source_snapshot"] is True
    assert result["original_values_preserved"] is True
    assert result["source_snapshot"]["schema_version"] == 1
    assert result["backup_snapshot"]["schema_version"] == 1
    assert result["migrated_snapshot"]["schema_version"] == 2
    assert result["source_snapshot"]["tables"]["service_runs"]["row_count"] == 2
    assert result["migrated_snapshot"]["raw_total_distance_m"] == 8.620882
    assert result["legacy_unknown"]["legacy_unclassified_row_count"] == 2
    assert result["legacy_unknown"]["legacy_unknown_distance_m"] == 8.620882
    assert set(result["columns_added"]["service_runs"]) == {
        "intent", "request_id", "phase", "segments_json", "interruption_reason"
    }
    assert json.loads((output / "report.json").read_text()) == result
    for name, artifact in result["artifacts"].items():
        assert artifact["sha256"] == digest(output / name)
        assert artifact["bytes"] == (output / name).stat().st_size
        if name.endswith(".sqlite3"):
            with sqlite3.connect(output / name) as db:
                assert db.execute("PRAGMA integrity_check").fetchone() == ("ok",)
    snapshot = json.loads((output / "snapshot.json").read_text())
    assert snapshot["schema_version"] == 2
    assert snapshot["lifetime"]["distance_m"] == 8.62
    assert {row["id"] for row in snapshot["recent_services"]} == {"svc-0", "svc-1"}
    assert {row["source"] for row in snapshot["recent_services"]} == {"unclassified source"}
    assert result["snapshot_export"]["recent_limit"] == 500
    assert "provenance not established" in result["snapshot_export"]["scope"]


def test_recent_export_limit_does_not_truncate_database_or_lifetime_totals(tmp_path):
    source = fixture_database(tmp_path / "source.sqlite3", rows=501)
    source_hash = digest(source)
    output = tmp_path / "converted"
    result = migration.migrate_copy(source, output)
    expected_total = math.fsum(3.810441 + index for index in range(501))

    assert result["status"] == "PASS"
    assert result["original_values_preserved"] is True
    assert digest(source) == source_hash
    assert result["snapshot_export"]["recent_limit"] == 500
    for field in ("source_snapshot", "backup_snapshot", "migrated_snapshot"):
        assert result[field]["tables"]["service_runs"]["row_count"] == 501
        assert result[field]["raw_total_distance_m"] == expected_total
    with sqlite3.connect(output / "migrated.sqlite3") as db:
        distances = db.execute("SELECT distance_m FROM service_runs").fetchall()
    assert len(distances) == 501
    assert math.fsum(row[0] for row in distances) == expected_total

    snapshot = json.loads((output / "snapshot.json").read_text())
    assert len(snapshot["recent_services"]) == 500
    assert {row["id"] for row in snapshot["recent_services"]} == {
        f"svc-{index}" for index in range(1, 501)
    }
    assert snapshot["lifetime"]["service_attempt_count"] == 501
    assert snapshot["lifetime"]["completed_service_count"] == 501
    assert snapshot["lifetime"]["distance_m"] == round(expected_total, 2)
    assert snapshot["historical_unclassified"]["record_count"] == 501
    assert snapshot["historical_unclassified"]["distance_m"] == round(expected_total, 2)


@pytest.mark.parametrize("version", [0, 1, 2])
def test_supported_versions_and_empty_history(tmp_path, version):
    source = fixture_database(tmp_path / "source.sqlite3", version=version, rows=0)
    result = migration.migrate_copy(source, tmp_path / "converted")
    assert result["status"] == "PASS"
    assert result["source_snapshot"]["raw_total_distance_m"] == 0


def test_extra_columns_blobs_and_other_tables_are_preserved(tmp_path):
    source = fixture_database(tmp_path / "source.sqlite3")
    with sqlite3.connect(source) as db:
        db.execute("ALTER TABLE service_runs ADD COLUMN custom_blob BLOB")
        db.execute("UPDATE service_runs SET custom_blob=? WHERE id='svc-1'", (b"\x00\xff",))
        db.execute('CREATE TABLE "extra metadata" (key TEXT, value BLOB)')
        db.execute('INSERT INTO "extra metadata" VALUES (?,?)', ("utf8 한글", b"\xff\x00"))
    result = migration.migrate_copy(source, tmp_path / "converted")
    assert result["status"] == "PASS"
    assert result["columns_added"]["extra metadata"] == []
    assert result["migrated_snapshot"]["tables"]["extra metadata"] == result["source_snapshot"]["tables"]["extra metadata"]


def test_multiple_active_recovery_is_failed_not_silently_approved(tmp_path):
    source = fixture_database(tmp_path / "source.sqlite3", active=True)
    original_hash = digest(source)
    result = migration.migrate_copy(source, tmp_path / "converted")
    assert result["status"] == "FAILED"
    assert result["original_values_preserved"] is False
    assert "active-record recovery" in result["error"]
    assert not (tmp_path / "converted" / "snapshot.json").exists()
    assert digest(source) == original_hash
    with sqlite3.connect(tmp_path / "converted" / "backup.sqlite3") as db:
        assert db.execute("SELECT result FROM service_runs ORDER BY id").fetchall() == [("active",), ("active",)]


def test_existing_schema2_active_value_rewrite_is_rejected(tmp_path):
    source = fixture_database(tmp_path / "source.sqlite3", rows=1, active=True)
    # Existing (not newly added) phase/intent must stay byte-for-byte unchanged,
    # even if the runtime tracker would normally normalize this legacy input.
    with sqlite3.connect(source) as db:
        db.execute("ALTER TABLE service_runs ADD COLUMN intent TEXT NOT NULL DEFAULT ''")
    result = migration.migrate_copy(source, tmp_path / "converted")
    assert result["status"] == "FAILED"
    assert result["original_values_preserved"] is False


def test_fresh_schema2_closed_record_is_not_reclassified(tmp_path):
    source = tmp_path / "source.sqlite3"
    tracker = ServiceMetricsTracker(source, now_fn=lambda: 1000.0)
    tracker.start_service("B1", source="fixture", intent="recall")
    tracker.observe_service_state(0, "DROP_ZONE_WAIT", now_s=1020.0)
    tracker.close()
    result = migration.migrate_copy(source, tmp_path / "converted")
    assert result["status"] == "PASS"
    assert result["columns_added"]["service_runs"] == []


def test_consistent_online_snapshot_includes_wal_not_later_writer(tmp_path, monkeypatch):
    source = fixture_database(tmp_path / "source.sqlite3", rows=1)
    writer = sqlite3.connect(source)
    writer.execute("PRAGMA journal_mode=WAL")
    writer.execute("UPDATE service_runs SET distance_m=7.125")
    writer.commit()
    original_backup = migration._backup_database
    calls = []

    def concurrent_backup(connection, destination):
        if not calls:
            writer.execute("UPDATE service_runs SET distance_m=9.75")
            writer.commit()
        calls.append(destination)
        return original_backup(connection, destination)

    monkeypatch.setattr(migration, "_backup_database", concurrent_backup)
    try:
        result = migration.migrate_copy(source, tmp_path / "converted")
        assert result["status"] == "PASS"
        assert result["source_snapshot"]["raw_total_distance_m"] == 7.125
        assert result["migrated_snapshot"]["raw_total_distance_m"] == 7.125
        assert writer.execute("SELECT distance_m FROM service_runs").fetchone() == (9.75,)
        assert result["source_bytes_unchanged_claimed"] is False
    finally:
        writer.close()


@pytest.mark.parametrize("kind", ["newer", "wrong_table", "not_sqlite", "empty", "missing", "directory"])
def test_invalid_sources_are_rejected_without_output(tmp_path, kind):
    source = tmp_path / "source.sqlite3"
    if kind == "newer":
        fixture_database(source, version=3)
    elif kind == "wrong_table":
        with sqlite3.connect(source) as db:
            db.execute("CREATE TABLE wrong_table (id TEXT)")
    elif kind == "not_sqlite":
        source.write_text("not SQLite")
    elif kind == "empty":
        source.touch()
    elif kind == "directory":
        source.mkdir()
    with pytest.raises(migration.MigrationRejected):
        migration.migrate_copy(source, tmp_path / "converted")
    assert not (tmp_path / "converted").exists()


@pytest.mark.parametrize("kind", ["source_link", "source_parent_link", "output_link", "output_parent_link", "existing_dir", "existing_file", "source_as_output", "missing_parent"])
def test_no_symlinks_existing_destinations_or_path_overwrites(tmp_path, kind):
    source = fixture_database(tmp_path / "source.sqlite3")
    output = tmp_path / "converted"
    before = digest(source)
    if kind == "source_link":
        link = tmp_path / "link.sqlite3"
        link.symlink_to(source)
        source = link
    elif kind in {"source_parent_link", "output_parent_link"}:
        link = tmp_path / "linked-parent"
        link.symlink_to(tmp_path, target_is_directory=True)
        if kind == "source_parent_link":
            source = link / source.name
        else:
            output = link / "converted"
    elif kind == "output_link":
        output.symlink_to(tmp_path / "missing")
    elif kind == "existing_dir":
        output.mkdir()
    elif kind == "existing_file":
        output.write_text("keep")
    elif kind == "source_as_output":
        output = source
    else:
        output = tmp_path / "missing" / "converted"
    with pytest.raises(migration.MigrationRejected):
        migration.migrate_copy(source, output)
    assert digest(source) == before


def test_tracker_persistence_failure_produces_failed_report_keeps_backup(tmp_path, monkeypatch):
    source = fixture_database(tmp_path / "source.sqlite3")

    class FailedTracker:
        persistence_enabled = False
        persistence_error = "fixture failure"

        def __init__(self, path):
            assert path == tmp_path / "converted" / "migrated.sqlite3"

        def close(self):
            pass

    monkeypatch.setattr(migration, "ServiceMetricsTracker", FailedTracker)
    result = migration.migrate_copy(source, tmp_path / "converted")
    assert result["status"] == "FAILED"
    assert "fixture failure" in result["error"]
    assert set(result["artifacts"]) == {"backup.sqlite3", "migrated.sqlite3"}


def test_cli_required_flags_success_and_existing_output_rejection(tmp_path, capsys):
    source = fixture_database(tmp_path / "source.sqlite3")
    output = tmp_path / "converted"
    with pytest.raises(SystemExit):
        migration.main([])
    args = ["--source", str(source), "--output-dir", str(output)]
    assert migration.main(args) == 0
    assert json.loads(capsys.readouterr().out)["status"] == "PASS"
    assert migration.main(args) == 2
    assert json.loads(capsys.readouterr().out)["status"] == "REJECTED"


def test_cli_failed_active_recovery_has_nonzero_exit(tmp_path, capsys):
    source = fixture_database(tmp_path / "source.sqlite3", active=True)
    assert migration.main(["--source", str(source), "--output-dir", str(tmp_path / "converted")]) == 1
    assert json.loads(capsys.readouterr().out)["status"] == "FAILED"


def test_output_creation_race_does_not_write_into_competing_directory(tmp_path, monkeypatch):
    source = fixture_database(tmp_path / "source.sqlite3")
    output = tmp_path / "converted"
    original_state = migration._state

    def competing_creator(connection, original_columns=None):
        output.mkdir()
        (output / "owned-by-other-caller.txt").write_text("keep")
        return original_state(connection, original_columns)

    monkeypatch.setattr(migration, "_state", competing_creator)
    with pytest.raises(migration.MigrationRejected):
        migration.migrate_copy(source, output)
    assert [path.name for path in output.iterdir()] == ["owned-by-other-caller.txt"]


@pytest.mark.parametrize("distance", [-1.0, float("inf"), "not-a-distance"])
def test_invalid_distances_rejected_without_normalizing_source(tmp_path, distance):
    source = fixture_database(tmp_path / "source.sqlite3")
    with sqlite3.connect(source) as db:
        db.execute("UPDATE service_runs SET distance_m=?", (distance,))
    source_hash = digest(source)
    with pytest.raises(migration.MigrationRejected):
        migration.migrate_copy(source, tmp_path / "converted")
    assert digest(source) == source_hash
    assert not (tmp_path / "converted").exists()


def test_mixed_legacy_and_new_segments_do_not_inflate_legacy_distance(tmp_path):
    source = fixture_database(tmp_path / "source.sqlite3", rows=1)
    tracker = ServiceMetricsTracker(source)
    tracker.close()
    segments = [
        {"kind": "unknown", "phase": "LEGACY_UNCLASSIFIED", "distance_m": 1.25,
         "moving_s": None, "waiting_s": None, "started_at": 1000.0,
         "ended_at": 1010.0, "timing_complete": False},
        {"kind": "delivery", "phase": "MOVING_TO_SITE", "distance_m": 2.560441,
         "moving_s": 5.0, "waiting_s": 0.0, "started_at": 1010.0,
         "ended_at": 1020.0, "timing_complete": True},
    ]
    with sqlite3.connect(source) as db:
        db.execute("UPDATE service_runs SET segments_json=?", (json.dumps(segments),))
    result = migration.migrate_copy(source, tmp_path / "converted")
    assert result["status"] == "PASS"
    assert result["source_snapshot"]["raw_total_distance_m"] == 3.810441
    assert result["legacy_unknown"]["legacy_unclassified_row_count"] == 1
    assert result["legacy_unknown"]["legacy_unknown_distance_m"] == 1.25
    assert result["legacy_unknown"]["unknown_segment_distance_m"] == 1.25


def test_module_cli_subprocess_uses_only_explicit_fixture_database(tmp_path):
    source = fixture_database(tmp_path / "source.sqlite3")
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1",
                       PYTHONPATH=str(Path(__file__).resolve().parents[1] / "runtime" / "python"))
    completed = subprocess.run(
        [sys.executable, "-m", "camrod_ui.service_metrics_migration", "--source", str(source),
         "--output-dir", str(tmp_path / "converted")],
        check=False, capture_output=True, text=True, env=environment, timeout=10,
    )
    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result["status"] == "PASS"
    assert result["record_count"] == 2
    assert result["raw_total_distance_m"] == 8.620882
    assert result["schema_version"] == 2
