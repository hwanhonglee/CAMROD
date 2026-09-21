"""HH_260915 - Pure event bridge and read-only API regression; no live ROS/DB."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'runtime/python'))
from camrod_ui.mission_recording_bridge import (  # noqa: E402
    MissionRecordingEmitter, default_mission_records_path,
    load_mission_recording_snapshot,
)


def recorder():
    rows = []
    emitter = MissionRecordingEmitter(lambda value: rows.append(json.loads(value)),
                                     now_fn=lambda: 1000, session='test-session')
    return emitter, rows


def test_same_mission_keeps_attempts_stop_and_return_without_granting_motion():
    emitter, rows = recorder()
    emitter.start('B7', 'delivery', 42, 'attempt1', 'robot_ui:private-token')
    emitter.start('B7', 'delivery', 42, 'attempt2')
    emitter.phase(15, 'DEPARTING_DROP_ZONE')
    emitter.phase(0, 'ROAD_HANDOFF_READY')
    emitter.phase(1, 'MOVING_TO_SITE')
    emitter.phase(1, 'MOVING_TO_SITE')
    emitter.stop('operator_stop')
    emitter.phase(16, 'OPERATOR_STOPPED')
    emitter.request_return('ui_return')
    emitter.phase(3, 'RETURNING_TO_DROP_ZONE')
    emitter.phase(10, 'DROP_ZONE_PARKING')
    emitter.phase(0, 'DROP_ZONE_WAIT')
    assert {row['mission_id'] for row in rows} == {'test-session:mission:42'}
    assert [row['attempt_id'] for row in rows if row['event'] == 'mission_started'] == ['attempt1', 'attempt2']
    assert [row['seq'] for row in rows] == list(range(1, len(rows) + 1))
    assert len([row for row in rows if row.get('state') == 1]) == 1
    assert not any(row.get('state_name') == 'ROAD_HANDOFF_READY' for row in rows)
    assert rows[0]['source'] == 'robot_ui'
    assert emitter.mission_id == ''


def test_recall_first_return_is_not_final_return_and_standalone_can_attach():
    emitter, rows = recorder()
    emitter.start('B8', 'recall', 3, 'attempt')
    emitter.request_return('recall_prepare', final_return=False)
    emitter.phase(9, 'RETURN_WITH_CARGO', 'camping_site_maneuver_controller:ROTATE_180:status', 'recall')
    emitter.phase(0, 'DROP_ZONE_WAIT')
    assert emitter.mission_id
    assert rows[1]['final_return'] is False
    assert rows[2]['phase'] == 'ROTATE_180'
    emitter.request_return('recall_final')
    emitter.phase(10, 'DROP_ZONE_PARKING')
    emitter.phase(12, 'WAITING_FOR_CHARGING')
    assert not emitter.mission_id
    emitter.request_return('standalone')
    assert rows[-1]['mission_id'] == ''
    assert emitter.phase(3, 'RETURNING_TO_DROP_ZONE')
    assert emitter.phase(10, 'DROP_ZONE_PARKING')
    assert emitter.phase(0, 'DROP_ZONE_WAIT')
    assert all(row['mission_id'] == '' for row in rows[-4:])
    assert not emitter.phase(1, 'UNRELATED_AFTER_RETURN')


def test_publish_failure_is_observational_and_phase_retries():
    emitter, rows = recorder()
    emitter.start('B1', 'delivery', 1)
    def fail(value):
        raise RuntimeError('recorder transport unavailable')
    emitter.publish = fail
    assert emitter.phase(1, 'MOVING_TO_SITE') is False
    assert 'unavailable' in emitter.error
    emitter.publish = lambda value: rows.append(json.loads(value))
    assert emitter.phase(1, 'MOVING_TO_SITE') is True
    assert emitter.error == ''


def test_backend_restart_global_stop_then_return_keeps_persisted_envelope(tmp_path):
    from camrod_ui.mission_journal import MissionJournal
    journal = MissionJournal(tmp_path, environment='test', now_fn=lambda: 1000)
    accepted = []

    def publish(value):
        accepted.append(journal.observe_event(json.loads(value), received_unix=1000))

    original = MissionRecordingEmitter(publish, now_fn=lambda: 1000, session='original')
    original.start('B7', 'delivery', 1, 'attempt')
    restarted = MissionRecordingEmitter(publish, now_fn=lambda: 1000, session='restart')
    assert restarted.stop('backend_startup_recovery')
    assert accepted == [True, True]
    assert journal.snapshot()['current_mission']['result'] == 'paused'
    assert restarted.request_return('operator_return')
    assert restarted.phase(3, 'RETURNING_TO_DROP_ZONE')
    assert restarted.phase(10, 'DROP_ZONE_PARKING')
    assert restarted.phase(0, 'DROP_ZONE_WAIT')
    result = journal.snapshot()
    assert result['lifetime']['mission_count'] == 1
    assert result['lifetime']['completed_count'] == 1
    assert result['missions'][0]['id'] == 'original:mission:1'
    journal.close()


def test_last_publish_failure_is_visible_even_without_followup_event(tmp_path):
    emitter, _ = recorder()
    emitter.start('B1', 'delivery', 1)

    def fail(value):
        raise RuntimeError('terminal event write failed')

    emitter.publish = fail
    assert emitter.stop('operator_stop') is False
    original = {'schema_version': 1, 'generated_at': 1000, 'missions': [],
                'lifetime': {'total_m': 12.5}, 'recorder': {'status': 'READY'}}
    path = tmp_path / 'snapshot.json'
    path.write_text(json.dumps(original))
    before = path.read_bytes()
    data, status = load_mission_recording_snapshot(tmp_path, now_s=1000,
                                                  emitter_error=emitter.error)
    assert status == 503
    assert data['recorder']['status'] == 'DEGRADED'
    assert 'terminal event write failed' in data['recorder']['error']
    assert data['lifetime']['total_m'] == 12.5
    assert path.read_bytes() == before
    missing, status = load_mission_recording_snapshot(tmp_path / 'missing', now_s=1000,
                                                     emitter_error=emitter.error)
    assert status == 503 and 'terminal event write failed' in missing['error']


def test_default_path_uses_device_state_root_without_legacy_db(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    assert default_mission_records_path() == tmp_path / 'camrod/mission_records'
    assert not list(tmp_path.iterdir())


def test_readonly_snapshot_missing_stale_invalid_and_bounded(tmp_path):
    data, status = load_mission_recording_snapshot(tmp_path, now_s=1000)
    assert status == 503 and 'lifetime' not in data
    path = tmp_path / 'snapshot.json'
    value = {'schema_version': 1, 'generated_at': 998,
             'missions': [{'id': str(i)} for i in range(101)],
             'recorder': {'status': 'recording'}}
    path.write_text(json.dumps(value))
    before = path.read_bytes()
    data, status = load_mission_recording_snapshot(tmp_path, now_s=1000, limit=3)
    assert status == 200 and len(data['missions']) == 3
    assert path.read_bytes() == before
    assert load_mission_recording_snapshot(tmp_path, now_s=1010)[1] == 503
    value['generated_at'] = '1970-01-01T00:16:39+00:00'
    path.write_text(json.dumps(value))
    assert load_mission_recording_snapshot(tmp_path, now_s=1000)[1] == 200
    path.write_text('not json')
    assert load_mission_recording_snapshot(tmp_path, now_s=1000)[1] == 503


def test_backend_existing_callbacks_emit_only_observation_and_keep_legacy_distance():
    from types import SimpleNamespace
    from camrod_ui.service_metrics import ServiceMetricsTracker
    from camrod_ui.ui_backend_node import UiBackendNode
    emitter, rows = recorder()
    tracker = ServiceMetricsTracker(None)
    backend = SimpleNamespace(_service_metrics=tracker, _mission_recording=emitter,
                              _active_mission_generation=11, _active_mission_source='robot_ui',
                              _active_mission_site='B9')
    UiBackendNode._start_service_metrics(backend, 'B9', 'camping_site_9', 'robot_ui', 11)
    UiBackendNode._observe_service_metrics(backend, 1, 'MOVING_TO_SITE')
    tracker.observe_velocity(1, 0, 1)
    tracker.observe_velocity(1, 0, 2)
    UiBackendNode._ensure_return_service_metrics(backend, 'ui_return')
    UiBackendNode._observe_service_metrics(backend, 3, 'RETURNING_TO_DROP_ZONE')
    UiBackendNode._observe_service_metrics(backend, 10, 'DROP_ZONE_PARKING')
    UiBackendNode._observe_service_metrics(backend, 0, 'DROP_ZONE_WAIT')
    assert rows[0]['event'] == 'mission_started'
    assert any(row['event'] == 'return_requested' for row in rows)
    assert tracker.summary()['lifetime']['distance_m'] == 1
    assert tracker.summary()['lifetime']['completed_service_count'] == 1
    tracker.close()
