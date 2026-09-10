"""HH_260911 - Station retry and delayed-command regression contracts."""
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import sys
import threading
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'runtime' / 'python'))
from avg_msgs.msg import AvgServiceState, VoiceState
from camrod_ui.ui_backend_node import UiBackendNode
from camrod_ui.voice_departure_gate import VoiceDepartureGate


def node_at_station(state=AvgServiceState.OPERATOR_STOPPED):
    n = SimpleNamespace(_lock=threading.RLock(), _destination_dispatch_lock=threading.RLock(),
        _latest_service_state=int(state), parking_method='auto',
        _latest_platform_is_charging=False, _drop_zone_exit_active=False,
        _active_mission_site='', _active_mission_source='', _active_mission_generation=0,
        _return_requested_generation=0, _parking_controller_operating_states={'auto':'IDLE'},
        _drop_zone_polygons=[[(-1,-1),(1,-1),(1,1),(-1,1)]], _now_s=lambda:100.0,
        _latest_arrival_pose_time_s=100.0, site_arrival_pose_timeout_s=2.0,
        _latest_arrival_pose=SimpleNamespace(header=SimpleNamespace(frame_id='map',
            stamp=SimpleNamespace(sec=100,nanosec=0)), pose=SimpleNamespace(position=SimpleNamespace(x=0.0,y=0.0))))
    return n

@pytest.mark.parametrize('state',[AvgServiceState.OPERATOR_STOPPED, AvgServiceState.DROP_ZONE_WAIT, AvgServiceState.WAITING_FOR_CHARGING])
def test_station_dock_uses_alignment_and_preserves_force_intent(state):
    n=node_at_station(state)
    with mock.patch.object(UiBackendNode,'_request_return_to_drop_zone_serialized',return_value='parking_alignment') as dispatch:
        result=UiBackendNode.request_manual_dock(n)
    assert result['success'] and result['parking_requested_final_method']=='apriltag'
    dispatch.assert_called_once_with(n, source='http:manual_dock:force_docking')

@pytest.mark.parametrize('invalid',['outside','stale','unknown'])
def test_stopped_state_alone_never_authorizes_docking(invalid):
    n=node_at_station()
    if invalid=='outside': n._latest_arrival_pose.pose.position.x=5.0
    if invalid=='stale': n._latest_arrival_pose_time_s=90.0
    if invalid=='unknown': n._latest_arrival_pose=None
    with mock.patch.object(UiBackendNode,'_request_return_to_drop_zone_serialized') as dispatch:
        result=UiBackendNode.request_manual_dock(n)
    assert not result['success']; dispatch.assert_not_called()

@pytest.mark.parametrize('phase',['ERROR','PARKED','IDLE'])
def test_terminal_parking_attempt_can_be_retried(phase):
    n=node_at_station(AvgServiceState.DROP_ZONE_PARKING)
    n._parking_controller_operating_states={'auto':phase}
    with mock.patch.object(UiBackendNode,'_request_return_to_drop_zone_serialized',return_value='parking_alignment') as dispatch:
        assert UiBackendNode.request_manual_dock(n)['success']
    assert dispatch.call_count==1

@pytest.mark.parametrize('phase',['WAITING_FOR_PARKING_OWNER','REVERSE_APPROACH','WAITING_FOR_TAG','FINAL_YAW_ALIGN'])
def test_active_parking_is_not_restarted(phase):
    n=node_at_station(AvgServiceState.DROP_ZONE_PARKING)
    n._parking_controller_operating_states={'auto':phase}
    with mock.patch.object(UiBackendNode,'_request_return_to_drop_zone_serialized') as dispatch:
        assert UiBackendNode.request_manual_dock(n)['error']=='parking_in_progress'
    dispatch.assert_not_called()


def test_duplicate_voice_request_executes_once_without_extending_deadline():
    gate=VoiceDepartureGate(); calls=[]
    gate.start(('cue',),lambda:calls.append(1),now_s=0,label='same',timeout_s=2)
    assert gate.start(('cue',),lambda:calls.append(2),now_s=1,label='same')==()
    gate.tick(2); gate.tick(50)
    assert calls==[1]

@pytest.mark.parametrize('event',['timeout','completion'])
def test_cancelled_voice_never_dispatches(event):
    gate=VoiceDepartureGate(); calls=[]
    gate.start(('cue',),lambda:calls.append(1),now_s=0)
    gate.on_voice_state(playing=True,current_key='cue',now_s=1)
    assert gate.cancel()
    if event=='timeout': gate.tick(50)
    else: gate.on_voice_state(playing=False,current_key='',now_s=2)
    assert calls==[] and not gate.busy

def test_real_mission_clear_invalidates_already_extracted_voice_callback():
    n=node_at_station(); n._voice_gate=VoiceDepartureGate()
    n._publish_voice_say=mock.Mock(); n.get_logger=lambda:mock.Mock()
    calls=[]
    UiBackendNode._dispatch_after_voice(n,('cue',),lambda:calls.append(1),label='request')
    late_callback=n._voice_gate._pending.on_complete
    UiBackendNode._clear_active_mission_identity(n)
    late_callback(); n._voice_gate.tick(200)
    assert calls==[] and not n._voice_gate.busy


def test_real_stop_cancels_voice_even_if_completion_arrives_late():
    n=node_at_station(); n._voice_gate=VoiceDepartureGate()
    n._publish_voice_say=mock.Mock(); n.get_logger=lambda:mock.Mock()
    n.site_names=['B1']; n.publish_mission_engage_from_destination=True
    n._state=SimpleNamespace(ws_site_states={},destination={})
    for name in ('_cancel_pending_manual_return_transition','_cancel_active_motion',
                 '_publish_mission_engage','_publish_engage','_publish_service_state','_schedule_broadcast'):
        setattr(n,name,mock.Mock())
    calls=[]
    UiBackendNode._dispatch_after_voice(n,('cue',),lambda:calls.append(1),label='request')
    late_callback=n._voice_gate._pending.on_complete
    with ExitStack() as stack:
        for name in ('_cancel_pending_charging_departure_transition','_cancel_pending_redock_after_disconnect',
                     '_cancel_pending_parking_rearm_transition','_publish_destination_dispatch_status'):
            stack.enter_context(mock.patch.object(UiBackendNode,name))
        UiBackendNode._stop_active_service_serialized(n,'regression_stop')
    late_callback(); n._voice_gate.tick(200)
    assert calls==[] and not n._voice_gate.busy
