"""Replay production UI handlers without ROS, browsers, or robot commands."""
import json
from pathlib import Path
import re
import shutil
import subprocess

import pytest


SOURCE = (Path(__file__).resolve().parents[1]
          / "camrod_ui_robot/assets/frontend/src/App.js").read_text(encoding="utf-8")


def replay(script):
    if not shutil.which("node"):
        pytest.skip("Node.js is required for production-handler replay")
    prefix = SOURCE[SOURCE.index("const SERVICE_STATE ="):
                    SOURCE.index("// HH_260904 - Re-dock events")]
    start = SOURCE.index("const emptyBatteryReturnState =")
    prefix += SOURCE[start:SOURCE.index("function WaitingRuntimeStatusPanel(", start)]
    start = SOURCE.index("ws.onmessage = (event) => {")
    handler = SOURCE[start:SOURCE.index("// HH_260708 - Reconnect the operator WebSocket", start)]
    start = SOURCE.index("const handleWaitingClick = () => {")
    handlers = SOURCE[start:SOURCE.index("\n  };", start) + len("\n  };")]
    start = SOURCE.index("const selectDestinationIntent = (intent) => {")
    handlers += SOURCE[start:SOURCE.index("const handleServiceDocking =", start)]
    setters = sorted(set(re.findall(r"\b(set[A-Z]\w*)\(", handler + handlers)))
    refs = sorted(set(re.findall(r"\b(\w+Ref)\.current", handler + handlers)) - {"wsRef"})
    setup = "\n".join(
        f"const {name} = value => {{uiState.{name} = typeof value === 'function' ? value(uiState.{name}) : value;}};"
        for name in setters)
    setup += "\n" + "\n".join(f"const {name} = {{current:null}};" for name in refs)
    body = prefix + "\nconst uiState = {}; const ws = {}; const wsRef = {current:ws};\n" + setup + r"""
const SITE_NAMES = Array.from({length:13}, (_,i)=>`B${i+1}`);
const connectionGeneration = 1;
const isWithinOperatingHours = () => true;
wsMountedRef.current = true; wsGenerationRef.current = 1;
missionAuthorityRevisionRef.current = 0;
destinationIntentRef.current = 'delivery';
batteryReturnStateRef.current = emptyBatteryReturnState();
serviceStateIdRef.current = SERVICE_STATE.CHARGING;
""" + handler + handlers + r"""
const send = frame => ws.onmessage({data:JSON.stringify(frame)});
const idle = () => send({mission_dispatch_active:false,mission_dispatch_generation:0,
  mission_dispatch_site:'',mission_dispatch_owner:'',mission_dispatch_intent:'',
  robot_recall_site:'',service_state:SERVICE_STATE.CHARGING});
""" + script
    result = subprocess.run(["node"], input=body, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def test_station_heartbeat_does_not_close_open_service_chooser():
    result = replay(r"""
handleWaitingClick(); idle(); idle();
console.log(JSON.stringify({waiting:uiState.setShowWaiting,
  menu:uiState.setShowServiceSelection,pinned:intentPinnedRef.current}));
""")
    assert result == {"waiting": False, "menu": True, "pinned": True}


@pytest.mark.parametrize("intent", ["delivery", "recall"])
def test_idle_identity_and_recall_replay_preserve_confirmed_service_role(intent):
    result = replay("handleWaitingClick(); activateDestinationService(" + json.dumps(intent) + r""");
idle(); idle(); idle();
console.log(JSON.stringify({waiting:uiState.setShowWaiting,
  menu:uiState.setShowServiceSelection,role:uiState.setDestinationIntent,
  roleRef:destinationIntentRef.current,pinned:intentPinnedRef.current}));
""")
    assert result == {"waiting": False, "menu": False, "role": intent,
                      "roleRef": intent, "pinned": True}


def test_new_authoritative_mission_unpins_local_choice_and_restores_actual_role():
    result = replay(r"""
handleWaitingClick(); activateDestinationService('delivery');
send({mission_dispatch_active:true,mission_dispatch_generation:42,
  mission_dispatch_site:'B2',mission_dispatch_owner:'guest',mission_dispatch_intent:'recall',
  robot_recall_site:'B2',service_state:SERVICE_STATE.RECALL_TO_SITE_ROAD});
console.log(JSON.stringify({role:uiState.setDestinationIntent,
  generation:uiState.setMissionDispatch.generation,site:uiState.setMissionDispatch.site,
  pinned:intentPinnedRef.current,waiting:uiState.setShowWaiting}));
""")
    assert result == {"role": "recall", "generation": 42, "site": "B2",
                      "pinned": False, "waiting": False}


def test_unpinned_completed_visit_returns_to_default_standby():
    result = replay(r"""
handleWaitingClick(); activateDestinationService('recall');
intentPinnedRef.current = false;
idle();
console.log(JSON.stringify({waiting:uiState.setShowWaiting,role:uiState.setDestinationIntent}));
""")
    assert result == {"waiting": True, "role": "delivery"}


def test_new_parking_preview_retains_stop_and_error_copy_priority():
    start = SOURCE.index(") : ['CHARGING'")
    block = SOURCE[start:SOURCE.index(") : displayedReturning", start)]
    assert "motionNotice?.label || parkingLifecycleStatus(" in block
    assert "motionNotice?.message || (serviceStateName" in block
    assert 'className="guest-recall-overlay"' not in SOURCE
    assert 'onClick={handleManualStop}' in SOURCE


def test_service_controls_have_distinct_real_pointer_targets():
    for hook in ("operator-service-selection-screen", "operator-back-to-services",
                 "operator-service-delivery-confirm", "operator-service-recall-confirm",
                 "operator-service-docking-confirm"):
        assert SOURCE.count(f'data-ui="{hook}"') == 1
