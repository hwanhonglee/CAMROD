"""Guest information must never cover mission controls or completion dialogs."""
import json
from pathlib import Path
import re
import shutil
import subprocess

import pytest

SOURCE = (Path(__file__).resolve().parents[1]
          / 'camrod_ui_robot/assets/frontend/src/App.js').read_text(encoding='utf-8')
HELPER = SOURCE[SOURCE.index('function guestAdmissionNotice('):SOURCE.index('function robotCanCompleteMission(')]
BASE = {
    'dispatch': {'active': True, 'site': 'B1', 'generation': 1788855364299001,
                 'owner': 'guest', 'intent': 'recall'},
    'showRecall': True, 'noticeSite': 'B1', 'missionPhase': 'GOAL_RECEIVED',
    'serviceStateName': 'CHARGING', 'systemHealth': 'OK',
    'arrivalVisible': False, 'arrivedSite': None, 'executionError': '',
}


def node(script):
    if not shutil.which('node'):
        pytest.skip('Node.js required for production callback replay')
    result = subprocess.run(['node'], input=script, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


@pytest.mark.parametrize('state', ['DROP_ZONE_WAIT', 'CHARGING', 'WAITING_FOR_CHARGING'])
def test_admitted_guest_receives_nonmoving_preparation_notice(state):
    case = dict(BASE, serviceStateName=state)
    text = node(HELPER + '\nconsole.log(JSON.stringify(guestAdmissionNotice(' + json.dumps(case) + ')));')
    assert text == 'B1 사이트 호출을 접수했습니다. 출발을 준비합니다.'
    assert '이동 중' not in text


@pytest.mark.parametrize('change', [
    {'serviceStateName': 'MOVING_TO_SITE', 'missionPhase': 'DRIVING'},
    {'serviceStateName': 'RECALL_TO_SITE_ROAD', 'missionPhase': 'DRIVING'},
    {'serviceStateName': 'SITE_ENTRY'},
    {'serviceStateName': 'DEPARTING_CHARGER'},
    {'serviceStateName': 'DEPARTING_DROP_ZONE'},
    {'serviceStateName': 'GUEST_LOADING_WAIT', 'missionPhase': 'ARRIVED'},
    {'serviceStateName': 'WAITING_FOR_RETURN_REQUEST'},
    {'serviceStateName': 'RETURN_WITH_CARGO'},
    {'serviceStateName': 'RETURNING_TO_DROP_ZONE'},
    {'serviceStateName': 'DROP_ZONE_PARKING'},
    {'serviceStateName': 'OPERATOR_STOPPED'},
    {'serviceStateName': 'UNKNOWN'},
    {'missionPhase': 'PATH_PREPARING'},
    {'missionPhase': 'SAFETY_STOP'},
    {'missionPhase': 'STOPPED'},
    {'missionPhase': 'ERROR'},
    {'missionPhase': 'INITIALIZING'},
    {'missionPhase': 'UNKNOWN'},
    {'systemHealth': 'ERROR'},
    {'executionError': 'site action failed'},
    {'arrivalVisible': True},
    {'arrivedSite': 'B1'},
    {'noticeSite': 'B2'},
    {'showRecall': False, 'noticeSite': None},
])
def test_stale_call_flags_cannot_cover_progress_arrival_or_error(change):
    value = node(HELPER + '\nconsole.log(JSON.stringify(guestAdmissionNotice(' + json.dumps(dict(BASE, **change)) + ')));')
    assert value == ''


@pytest.mark.parametrize('identity', [
    {'active': False}, {'owner': 'operator'}, {'owner': 'robot'},
    {'generation': 0}, {'generation': None}, {'generation': '12'},
    {'site': ''}, {'site': 'B14'}, {'intent': ''}, {'intent': 'manual'},
])
def test_notice_requires_current_guest_identity(identity):
    case = dict(BASE, dispatch=dict(BASE['dispatch'], **identity))
    assert node(HELPER + '\nconsole.log(JSON.stringify(guestAdmissionNotice(' + json.dumps(case) + ')));') == ''


def test_real_websocket_flags_survive_but_cannot_hide_active_controls_or_final_confirmation():
    prefix = SOURCE[SOURCE.index('const SERVICE_STATE ='):SOURCE.index('// HH_260904 - Re-dock events')]
    start = SOURCE.index('const emptyBatteryReturnState =')
    prefix += SOURCE[start:SOURCE.index('// HH_260721 - Reuse one health', start)]
    start = SOURCE.index('ws.onmessage = (event) => {')
    handler = SOURCE[start:SOURCE.index('// HH_260708 - Reconnect the operator WebSocket', start)]
    setters = sorted(set(re.findall(r'\b(set[A-Z]\w*)\(', handler)))
    refs = sorted(set(re.findall(r'\b(\w+Ref)\.current', handler)) - {'wsRef'})
    setup = '\n'.join(f"const {name} = value => {{uiState.{name} = typeof value === 'function' ? value(uiState.{name}) : value;}};" for name in setters)
    setup += '\n' + '\n'.join(f'const {name} = {{current:null}};' for name in refs)
    script = prefix + '\nconst uiState = {}; const ws = {}; const wsRef = {current:ws};\n' + setup + r'''
const SITE_NAMES = Array.from({length:13}, (_,i)=>`B${i+1}`);
const connectionGeneration = 1;
wsMountedRef.current = true; wsGenerationRef.current = 1;
missionAuthorityRevisionRef.current = 0; destinationIntentRef.current = 'delivery';
batteryReturnStateRef.current = emptyBatteryReturnState();
''' + handler + r'''
const send = frame => ws.onmessage({data:JSON.stringify(frame)});
const notice = () => guestAdmissionNotice({dispatch:uiState.setMissionDispatch,
  showRecall:uiState.setShowGuestRecall, noticeSite:uiState.setGuestNavigateSite,
  missionPhase:uiState.setMissionPhase, serviceStateName:uiState.setServiceStateName,
  systemHealth:uiState.setSystemHealth, arrivalVisible:uiState.setShowArrivalComplete,
  arrivedSite:uiState.setArrivedSite, executionError:uiState.setMissionExecutionError});
send({mission_dispatch_active:true,mission_dispatch_generation:1788855364299001,
  mission_dispatch_site:'B1',mission_dispatch_owner:'guest',mission_dispatch_intent:'recall',
  service_state:13,mission_phase:'GOAL_RECEIVED',system_health:'OK',guest_navigate:'B1',guest_recall:true});
const admitted = notice(), identity = uiState.setMissionDispatch;
send({service_state:7,service_state_name:'RECALL_TO_SITE_ROAD',mission_phase:'DRIVING'});
const driving = {notice:notice(), flag:uiState.setShowGuestRecall, site:uiState.setGuestNavigateSite};
send({service_state:8,service_state_name:'GUEST_LOADING_WAIT',site:'B1',mission_phase:'ARRIVED'});
const firstWait = notice();
send({service_state:9,service_state_name:'RETURN_WITH_CARGO',mission_phase:'DRIVING',
  service_state_description:'camping_site_maneuver_controller:ROTATE_180:active'});
const turnaround = notice();
send({service_state:8,service_state_name:'GUEST_LOADING_WAIT',site:'B1',mission_phase:'ARRIVED',
  recall_final_return_ready:true,mission_execution_error:''});
const finalWait = {notice:notice(), modal:uiState.setShowArrivalComplete,
  site:uiState.setArrivedSite,final:uiState.setRecallFinalReturnReady,
  permitted:robotCanCompleteMission(uiState.setMissionDispatch,uiState.setArrivedSite,uiState.setServiceStateName)};
send({mission_phase:'SAFETY_STOP',system_health:'ERROR'});
console.log(JSON.stringify({admitted,driving,firstWait,turnaround,finalWait,safety:notice(),
  identityBefore:identity,identityAfter:uiState.setMissionDispatch}));
'''
    result = node(script)
    assert result['admitted'].startswith('B1 사이트 호출을 접수했습니다')
    assert result['driving'] == {'notice': '', 'flag': True, 'site': 'B1'}
    assert result['firstWait'] == result['turnaround'] == result['safety'] == ''
    assert result['finalWait'] == {'notice': '', 'modal': True, 'site': 'B1', 'final': True, 'permitted': True}
    assert result['identityBefore'] == result['identityAfter'] == BASE['dispatch']


def test_both_screen_headers_use_normal_flow_status_and_have_no_guest_modal():
    assert SOURCE.count('{guestAdmissionStatus}') == 2
    assert 'className="guest-recall-overlay"' not in SOURCE
    assert 'className="guest-recall-box"' not in SOURCE
    status = SOURCE.split('const guestAdmissionStatus =', 1)[1].split(') : null;', 1)[0]
    assert 'role="status"' in status and 'aria-live="polite"' in status
    assert "position: 'static'" in status and "pointerEvents: 'none'" in status
    assert 'onClick=' not in status and 'zIndex' not in status
    assert 'setTimeout' not in HELPER and 'fetch(' not in HELPER and 'CARLA' not in HELPER
    assert 'onClick={handleStopMove}' in SOURCE
    assert 'onClick={handleArrivalComplete}' in SOURCE
