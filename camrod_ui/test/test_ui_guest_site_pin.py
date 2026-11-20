"""HJ_261120 - Guest recall requires the site PIN, verified by ui_guest_node."""

import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))

from camrod_ui.site_access_store import AttemptLimiter, SiteAccessStore  # noqa: E402
from camrod_ui.ui_guest_node import (  # noqa: E402
    GUEST_PIN_LOCKOUT_S,
    GUEST_PIN_MAX_FAILURES,
    guest_site_pin_error,
)

ROOT = Path(__file__).resolve().parents[1]
GUEST_HTML = ROOT / "camrod_ui_guest" / "assets" / "guest_frontend" / "index.html"
GUEST_NODE = ROOT / "runtime" / "python" / "camrod_ui" / "ui_guest_node.py"


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


@pytest.fixture
def store(tmp_path):
    access = SiteAccessStore(tmp_path / "site_access.json", ["B1", "B2"])
    access.ensure_initialized()
    access.set_pin("B1", "4827", "test")
    access.set_pin("B2", "1111", "test")
    return access


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def limiter(clock):
    return AttemptLimiter(GUEST_PIN_MAX_FAILURES, GUEST_PIN_LOCKOUT_S, clock=clock)


def test_correct_pin_admits(store, limiter):
    assert guest_site_pin_error(store, limiter, "B1", "4827") is None


@pytest.mark.parametrize("pin", [None, "", "48", "48271", "abcd", 4827])
def test_missing_or_malformed_pin_is_required_and_not_counted(store, limiter, pin):
    for _ in range(GUEST_PIN_MAX_FAILURES + 2):
        error = guest_site_pin_error(store, limiter, "B1", pin)
        assert error["error"] == "pin_required"
    assert guest_site_pin_error(store, limiter, "B1", "4827") is None


def test_wrong_pin_then_lockout_then_recovery(store, limiter, clock):
    for _ in range(GUEST_PIN_MAX_FAILURES - 1):
        error = guest_site_pin_error(store, limiter, "B1", "0000")
        assert error["error"] == "wrong_pin"
        assert error["site"] == "B1" and error["message"]
    locked = guest_site_pin_error(store, limiter, "B1", "0000")
    assert locked["error"] == "pin_locked"
    assert locked["retry_after_s"] == int(GUEST_PIN_LOCKOUT_S)
    # Even the right PIN is refused while locked.
    assert guest_site_pin_error(store, limiter, "B1", "4827")["error"] == "pin_locked"
    # The lock is per site.
    assert guest_site_pin_error(store, limiter, "B2", "1111") is None
    clock.now += GUEST_PIN_LOCKOUT_S + 1
    assert guest_site_pin_error(store, limiter, "B1", "4827") is None


def test_pin_from_another_site_is_wrong(store, limiter):
    assert guest_site_pin_error(store, limiter, "B1", "1111")["error"] == "wrong_pin"


def test_unavailable_store_fails_closed(limiter):
    assert guest_site_pin_error(None, limiter, "B1", "4827")["error"] == "pin_unavailable"

    class Broken:
        def verify_pin(self, site, pin):
            raise OSError("disk")

    assert guest_site_pin_error(Broken(), limiter, "B1", "4827")["error"] == "pin_unavailable"


def test_manager_change_is_picked_up_without_restart(store, limiter, tmp_path):
    other = SiteAccessStore(tmp_path / "site_access.json", ["B1", "B2"])
    other.set_pin("B1", "9999", "manager_ui")
    assert guest_site_pin_error(store, limiter, "B1", "9999") is None
    assert guest_site_pin_error(store, limiter, "B1", "4827")["error"] == "wrong_pin"


def test_node_gates_navigate_before_reservation_and_refuses_legacy_recall():
    source = GUEST_NODE.read_text(encoding="utf-8")
    handler = source[source.index('if action == "navigate":'):source.index('elif action == "usage_complete":')]
    gate = handler.index("guest_site_pin_error")
    reserve = handler.index("_reserve_guest_recall_request")
    publish = handler.index("_publish_navigate")
    assert gate < reserve < publish
    assert "elif pin_error is not None:" in handler
    legacy = handler[handler.index('elif action == "recall":'):]
    assert "_publish_guest_recall" not in legacy
    assert '"pin_required"' in legacy
    assert "install_lan_guard(app, node.get_logger())" in source


def test_guest_page_pin_step_and_rejection_restore():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js is required for actual Guest JavaScript execution")
    html = GUEST_HTML.read_text(encoding="utf-8")
    assert "로봇이 사이트 앞 도로에서 멈춥니다." in html
    assert 'onclick="openPinPad()"' in html
    production = html[html.index("  let ws = null;"):html.index("  /* ── Init ── */")]
    script = r"""
const assert = require('node:assert/strict');
const vm = require('node:vm');
class Element {
  constructor(id='') {
    this.id=id;this.style={};this.dataset={};this.children=[];
    this.textContent='';this.disabled=false;this.className='';this.attributes={};
    this.classes=new Set();
    this.classList={
      add:(...n)=>n.forEach(x=>this.classes.add(x)),
      remove:(...n)=>n.forEach(x=>this.classes.delete(x)),
      toggle:(x,f)=>{if(f===undefined)f=!this.classes.has(x);if(f)this.classes.add(x);else this.classes.delete(x);},
      contains:x=>this.classes.has(x),
    };
  }
  set innerHTML(v){this._h=v;this.children=[];}
  get innerHTML(){return this._h||'';}
  appendChild(c){this.children.push(c);}
  setAttribute(n,v){this.attributes[n]=v;}
  removeAttribute(n){delete this.attributes[n];}
}
const elements=new Map();
const element=id=>{if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id);};
const document={getElementById:element,createElement:()=>new Element(),
  querySelectorAll:s=>s==='.site-btn'?element('siteGrid').children:[]};
const sockets=[];
class Socket{static OPEN=1;constructor(u){this.readyState=0;this.sent=[];sockets.push(this);}
  send(d){this.sent.push(JSON.parse(d));}close(){this.readyState=3;if(this.onclose)this.onclose();}}
const context=vm.createContext({document,WebSocket:Socket,window:{confirm:()=>true},
  location:{protocol:'http:',host:'localhost'},setTimeout:()=>1,clearTimeout:()=>{},
  setInterval:()=>1,clearInterval:()=>{}});
vm.runInContext(PRODUCTION,context);
const value=c=>vm.runInContext(c,context);
value('connect()');
const socket=sockets[0];socket.readyState=Socket.OPEN;socket.onopen();
const send=f=>socket.onmessage({data:JSON.stringify(f)});
send({sites:['B1','B2'],identity_revision:1,service_state:2,service_state_name:'DROP_ZONE_WAIT',
  phase:'idle',site:'',request_intent:'',request_owner:'',mission_battery_ready:true,battery:80});
value("selectSite('B1');openConfirm();openPinPad()");
assert.equal(element('pinOverlay').classList.contains('show'),true);
value("pressPinKey('1');pressPinKey('2');pressPinKey('3')");
assert.equal(element('pinCallBtn').disabled,true);
value("confirmNavigate()");
assert.equal(socket.sent.filter(m=>m.action==='navigate').length,0);
value("pressPinKey('4')");
assert.equal(element('pinCallBtn').disabled,false);
assert.equal(element('pinCallBtn').textContent,'B1로 호출');
value("confirmNavigate()");
assert.deepEqual(socket.sent.at(-1),{action:'navigate',site:'B1',pin:'1234'});
assert.equal(element('pinOverlay').classList.contains('show'),false);
assert.equal(element('siteCard').style.display,'none');
send({error:'wrong_pin',site:'B1',message:'비밀번호가 맞지 않아요. 다시 입력해 주세요.'});
assert.equal(value('activeRequestIntent'),null);
assert.equal(value('selectedSite'),'B1');
assert.equal(element('siteCard').style.display,'block');
assert.equal(element('pinOverlay').classList.contains('show'),true);
assert.equal(element('pinError').textContent,'비밀번호가 맞지 않아요. 다시 입력해 주세요.');
assert.equal(value('pinEntry'),'');
value("pressPinKey('9');pressPinKey('9');pressPinKey('9');pressPinKey('9');confirmNavigate()");
send({error:'pin_locked',site:'B1',retry_after_s:60,message:'x'});
assert.match(element('pinError').textContent,/60초 후/);
value("closePinPad()");
assert.equal(element('pinOverlay').classList.contains('show'),false);
""".replace("PRODUCTION", json.dumps(production))
    result = subprocess.run([node, "-e", script], capture_output=True, text=True,
                            timeout=10, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
