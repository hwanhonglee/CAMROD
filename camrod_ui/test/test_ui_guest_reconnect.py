"""Execute the actual Guest connect() in Node VM; no DOM or network exists."""
import json
from pathlib import Path
import shutil
import subprocess
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / "camrod_ui_guest" / "assets"
          / "guest_frontend" / "index.html")


class GuestReconnectTest(unittest.TestCase):
    def run_contract(self, scenario):
        node = shutil.which("node")
        if not node:
            self.skipTest("Node.js is required for actual Guest JavaScript execution")
        source = SOURCE.read_text(encoding="utf-8")
        # Include the production state declarations and exact connect() body.
        start = source.index("  let ws = null;")
        end = source.index("  function startGrace(seconds)", start)
        production = source[start:end]
        script = r"""
const vm = require('node:vm');
const assert = require('node:assert/strict');
const sockets=[], timers=new Map(), intervals=new Map();
const effects={connections:[], updates:0, grid:0};
let serial=0;
class Socket {
  static OPEN=1;
  constructor(url){this.url=url;this.readyState=0;this.sent=[];this.closed=0;sockets.push(this);}
  send(data){this.sent.push(JSON.parse(data));}
  close(){this.closed++;this.readyState=3;if(this.onclose)this.onclose();}
}
const context=vm.createContext({
  WebSocket:Socket,location:{protocol:'http:',host:'offline.invalid'},
  setConn:value=>effects.connections.push(value),
  updateUI:()=>effects.updates++,buildSiteGrid:()=>effects.grid++,
  updateBattery:()=>{},startGrace:()=>{throw Error('Unexpected grace action');},
  setTimeout:callback=>{const id=++serial;timers.set(id,{callback,cancelled:false});return id;},
  clearTimeout:id=>{if(timers.has(id))timers.get(id).cancelled=true;},
  setInterval:callback=>{const id=++serial;intervals.set(id,{callback,cancelled:false});return id;},
  clearInterval:id=>{if(intervals.has(id))intervals.get(id).cancelled=true;},
});
vm.runInContext(PRODUCTION,context);
const value=expression=>vm.runInContext(expression,context);
const connect=()=>{value('connect()');return sockets.at(-1);};
const open=socket=>{socket.readyState=Socket.OPEN;socket.onopen();};
const frame=(socket,revision,site='B1',state='GUEST_LOADING_WAIT',phase='arrived')=>
  socket.onmessage({data:JSON.stringify({sites:['B1','B2'],identity_revision:revision,
    site,request_owner:'guest',request_intent:'recall',mission_generation:100+revision,
    service_state:8,service_state_name:state,phase})});
SCENARIO
""".replace("PRODUCTION", json.dumps(production)).replace("SCENARIO", scenario)
        result = subprocess.run([node, "-e", script], capture_output=True, text=True,
                                timeout=10, check=False)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_backend_restart_revision_zero_is_accepted_on_new_current_open(self):
        self.run_contract("""
const first=connect();open(first);frame(first,79);
assert.equal(value('lastIdentityRevision'),79);
first.close();const second=connect();
assert.equal(value('lastIdentityRevision'),79); // Do not reset on construction.
open(second);assert.equal(value('lastIdentityRevision'),-1);
frame(second,0,'B2','CHARGING','ready');
assert.equal(value('lastIdentityRevision'),0);
assert.equal(value('lastDestSite'),'B2');
assert.equal(value('currentServiceStateName'),'CHARGING');
assert.equal(value('currentPhase'),'ready');
""")

    def test_same_connection_older_revision_keeps_identity_and_lifecycle(self):
        self.run_contract("""
const socket=connect();open(socket);frame(socket,79);
frame(socket,78,'B2','CHARGING','ready');
assert.equal(value('lastIdentityRevision'),79);
assert.equal(value('lastDestSite'),'B1');
assert.equal(value('currentServiceStateName'),'GUEST_LOADING_WAIT');
assert.equal(value('currentPhase'),'arrived');
""")

    def test_old_open_message_close_and_error_cannot_overwrite_current_socket(self):
        self.run_contract("""
const old=connect();open(old);frame(old,79);
const oldHeartbeat=intervals.get(value('heartbeatTimer')).callback;
const current=connect();open(current);frame(current,4,'B2','CHARGING','ready');
const heartbeat=value('heartbeatTimer'),updateCount=effects.updates,connCount=effects.connections.length;
old.onopen();old.onmessage({data:'not JSON'});
old.onmessage({data:JSON.stringify({locked:true,grace_remaining:20})});
frame(old,999);old.onclose();old.onerror();oldHeartbeat();
assert.equal(value('ws'),current);
assert.equal(value('lastIdentityRevision'),4);
assert.equal(value('lastDestSite'),'B2');
assert.equal(value('currentServiceStateName'),'CHARGING');
assert.equal(value('heartbeatTimer'),heartbeat);
assert.equal(intervals.get(heartbeat).cancelled,false);
assert.equal(effects.updates,updateCount);
assert.equal(effects.connections.length,connCount);
assert.equal(old.sent.length,0);
assert.equal(old.closed,0);
""")

    def test_stale_reconnect_timer_cannot_replace_newer_or_closed_generation(self):
        self.run_contract("""
const first=connect();open(first);first.close();
const firstTimer=timers.get(value('reconnectTimer'));
const second=connect();open(second);
assert.equal(firstTimer.cancelled,true);
firstTimer.callback();assert.equal(sockets.length,2);
second.close();const secondTimer=timers.get(value('reconnectTimer'));
firstTimer.callback();assert.equal(sockets.length,2);
assert.equal(value('ws'),null);
secondTimer.callback();assert.equal(sockets.length,3);
assert.equal(value('ws'),sockets[2]);
""")

    def test_current_heartbeat_and_close_keep_existing_behavior(self):
        self.run_contract("""
const socket=connect();open(socket);
const heartbeat=intervals.get(value('heartbeatTimer'));
heartbeat.callback();assert.deepEqual(socket.sent,[{action:'heartbeat'}]);
value('usageCompletePending=true;cancelRequestPending=true');socket.close();
assert.equal(heartbeat.cancelled,true);
assert.equal(value('ws'),null);
assert.equal(value('usageCompletePending'),false);
assert.equal(value('cancelRequestPending'),false);
assert.equal(effects.connections.at(-1),false);
assert.equal(timers.size,1);
""")

    def test_lock_or_grace_suppresses_current_reconnect_timer(self):
        self.run_contract("""
const socket=connect();open(socket);socket.close();
const timer=timers.get(value('reconnectTimer'));
value('isGrace=true');timer.callback();assert.equal(sockets.length,1);
value('isGrace=false;isLocked=true');timer.callback();assert.equal(sockets.length,1);
""")


if __name__ == "__main__":
    unittest.main()
