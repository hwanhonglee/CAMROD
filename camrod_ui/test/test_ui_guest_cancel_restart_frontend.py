"""HH_261001 - Execute Guest UI state transitions after cancel and terminal states."""

import json
from pathlib import Path
import shutil
import subprocess
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / "camrod_ui_guest" / "assets"
          / "guest_frontend" / "index.html")


class GuestCancelRestartFrontendTest(unittest.TestCase):
    def run_scenario(self, scenario: str) -> None:
        node = shutil.which("node")
        if not node:
            self.skipTest("Node.js is required for actual Guest JavaScript execution")
        html = SOURCE.read_text(encoding="utf-8")
        start = html.index("  let ws = null;")
        end = html.index("  /* ── WiFi", start)
        production = html[start:end]
        script = r"""
const assert = require('node:assert/strict');
const vm = require('node:vm');
class Element {
  constructor(id='') {
    this.id=id;this.style={};this.dataset={};this.children=[];
    this.textContent='';this.disabled=false;this.className='';this.attributes={};
    this.classes=new Set();
    this.classList={
      add:(...names)=>names.forEach(name=>this.classes.add(name)),
      remove:(...names)=>names.forEach(name=>this.classes.delete(name)),
      toggle:(name,force)=>{
        if(force===undefined)force=!this.classes.has(name);
        if(force)this.classes.add(name);else this.classes.delete(name);
      },
      contains:name=>this.classes.has(name),
    };
  }
  set innerHTML(value) {this._innerHTML=value;this.children=[];}
  get innerHTML() {return this._innerHTML||'';}
  appendChild(child) {this.children.push(child);}
  setAttribute(name,value) {this.attributes[name]=value;}
  removeAttribute(name) {delete this.attributes[name];}
}
const elements=new Map();
const element=id=>{
  if(!elements.has(id))elements.set(id,new Element(id));
  return elements.get(id);
};
const document={
  getElementById:element,
  createElement:()=>new Element(),
  querySelectorAll:selector=>selector==='.site-btn'?element('siteGrid').children:[],
};
const sockets=[];
class Socket {
  static OPEN=1;
  constructor(url){this.url=url;this.readyState=0;this.sent=[];sockets.push(this);}
  send(data){this.sent.push(JSON.parse(data));}
  close(){this.readyState=3;if(this.onclose)this.onclose();}
}
const context=vm.createContext({
  document,WebSocket:Socket,window:{confirm:()=>true},
  location:{protocol:'http:',host:'localhost'},
  setTimeout:()=>1,clearTimeout:()=>{},setInterval:()=>1,clearInterval:()=>{},
});
vm.runInContext(PRODUCTION,context);
const value=code=>vm.runInContext(code,context);
value('connect()');
const socket=sockets[0];socket.readyState=Socket.OPEN;socket.onopen();
const send=frame=>socket.onmessage({data:JSON.stringify(frame)});
const site=number=>element('siteGrid').children.find(button=>button.dataset.site===`B${number}`);
SCENARIO
""".replace("PRODUCTION", json.dumps(production)).replace("SCENARIO", scenario)
        result = subprocess.run(
            [node, "-e", script], capture_output=True, text=True,
            timeout=10, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_owned_cancel_reopens_all_sites_but_operator_stop_does_not(self) -> None:
        self.run_scenario(r"""
const stopped={sites:['B1','B2'],identity_revision:1,service_state:16,
  service_state_name:'OPERATOR_STOPPED',phase:'stopped',site:'',
  request_intent:'',request_owner:'',mission_battery_ready:true,battery:80};
send({...stopped,guest_cancel_restart_ready:false});
assert.equal(element('siteCard').style.display,'none');
send({...stopped,identity_revision:2,guest_cancel_restart_ready:true});
assert.equal(element('siteCard').style.display,'block');
assert.equal(site(1).disabled,false);assert.equal(site(2).disabled,false);
value("selectSite('B2');openConfirm();confirmNavigate()");
assert.deepEqual(socket.sent.at(-1),{action:'navigate',site:'B2'});
assert.equal(element('siteCard').style.display,'none');
send({identity_revision:3,service_state:4,service_state_name:'GUEST_RECALL_SERVICE',
  phase:'recall',site:'B2',request_intent:'recall',request_owner:'guest',
  guest_cancel_restart_ready:false});
assert.equal(element('cancelCard').style.display,'block');
value('sendCancel()');
assert.deepEqual(socket.sent.at(-1),{action:'cancel'});
assert.equal(element('cancelMissionBtn').disabled,true);
send({...stopped,identity_revision:4,guest_cancel_restart_ready:true});
assert.equal(element('siteCard').style.display,'block');
assert.equal(element('cancelCard').style.display,'none');
assert.equal(value('cancelRequestPending'),false);
value("selectSite('B1');openConfirm();confirmNavigate()");
assert.deepEqual(socket.sent.at(-1),{action:'navigate',site:'B1'});
send({...stopped,identity_revision:5,guest_cancel_restart_ready:false});
assert.equal(element('siteCard').style.display,'none');
assert.equal(value('isDispatchReady()'),false);
""")

    def test_arrival_return_parking_completion_retry_and_safety_hold(self) -> None:
        self.run_scenario(r"""
const base={sites:['B1','B2'],mission_battery_ready:true,battery:80};
send({...base,identity_revision:1,service_state:14,service_state_name:'CHARGING',
  phase:'charging',site:'',request_intent:'',request_owner:'',
  guest_cancel_restart_ready:false});
assert.equal(element('siteCard').style.display,'block');
send({...base,identity_revision:2,service_state:8,service_state_name:'GUEST_LOADING_WAIT',
  phase:'arrived',site:'B2',request_intent:'recall',request_owner:'guest'});
assert.equal(element('siteCard').style.display,'none');
assert.equal(element('completeCard').style.display,'block');
send({...base,identity_revision:3,service_state:9,service_state_name:'RETURN_WITH_CARGO',
  phase:'returning',site:'B2',request_intent:'recall',request_owner:'guest'});
assert.equal(element('completeCard').style.display,'none');
assert.equal(element('cancelCard').style.display,'block');
send({...base,identity_revision:4,service_state:11,service_state_name:'DROP_ZONE_PARKING',
  phase:'parking',site:'B2',request_intent:'recall',request_owner:'guest'});
assert.equal(element('siteCard').style.display,'none');
assert.equal(element('cancelCard').style.display,'block');
send({...base,identity_revision:5,service_state:12,service_state_name:'WAITING_FOR_CHARGING',
  phase:'waiting_for_charging',site:'',request_intent:'',request_owner:''});
assert.equal(element('siteCard').style.display,'block');
send({...base,identity_revision:6,service_state:14,service_state_name:'CHARGING',
  phase:'charging',site:'B2',request_intent:'recall',request_owner:'guest',
  mission_retryable:true});
assert.equal(element('siteCard').style.display,'block');
assert.equal(site(1).disabled,true);assert.equal(site(2).disabled,false);
assert.equal(value('selectedSite'),'B2');
send({...base,identity_revision:7,service_state:16,service_state_name:'OPERATOR_STOPPED',
  phase:'stopped',site:'',request_intent:'',request_owner:'',
  mission_retryable:false,guest_cancel_restart_ready:true,safety_hold:true});
assert.equal(element('siteCard').style.display,'none');
send({safety_hold:false});
assert.equal(element('siteCard').style.display,'block');
""")


if __name__ == "__main__":
    unittest.main()
