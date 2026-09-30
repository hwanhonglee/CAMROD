"""Bounded real B7 dispatch through the UI API; stop after 5 m or 90 s.

This is a departure/integration smoke, not a completed B7 delivery certificate.
CARLA is read-only here; only CAMROD's ordinary request/stop API commands motion.
"""
import json
import math
from pathlib import Path
import time
from urllib.request import Request, urlopen
import carla

OUT = Path(__file__).parent / 'departure.json'

def request(path, post=False):
    with urlopen(Request('http://127.0.0.1:8010/' + path,
                         method='POST' if post else 'GET'), timeout=5) as response:
        return json.load(response)

c=carla.Client('127.0.0.1',2000)
c.set_timeout(5)
w=c.get_world()
w.wait_for_tick(5)
actor=next(a for a in w.get_actors() if a.type_id=='vehicle.ranger.default')
initial=actor.get_location()
report=dict(kind='bounded_departure_only', site='B7', samples=[])
try:
    report['dispatch']=request('ui/destination?site=B7&run=true',True)
    start=time.monotonic()
    for _ in range(90):
        p=actor.get_location()
        state=request('ui/state')
        sample=dict(elapsed_s=round(time.monotonic()-start,3),
                    x=p.x,y=p.y,z=p.z,
                    displacement_m=math.hypot(p.x-initial.x,p.y-initial.y),
                    state=state)
        report['samples'].append(sample)
        OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(sample['elapsed_s'],sample['displacement_m'],
              state.get('service_state_name'),state.get('ready_message'),flush=True)
        if sample['displacement_m']>5:
            report['outcome']='moved_more_than_5m_then_operator_stop'
            break
        time.sleep(1)
    else:
        report['outcome']='90s_observation_limit_then_operator_stop'
finally:
    report['stop']=request('ui/stop',True)
    OUT.write_text(json.dumps(report,ensure_ascii=False,indent=2))
