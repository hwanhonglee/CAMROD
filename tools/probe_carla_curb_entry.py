#!/usr/bin/env python3
"""HH_261007 - One bounded visible-UI B1 entry test, never a round-trip claim."""
import argparse
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/virtual_carla'))
from camping_site_matrix import OperatorBrowserClient, UIClient, RosObservation, mission_identity
from record_physical_wheel_telemetry import select_unique_ranger, capture_sample
from manual_4ws_evidence import ManualDriveBrowser


def main():
    import carla
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    # HH_261007 - Reserve fresh evidence; preserve all earlier failed runs.
    with args.output.open('x', encoding='utf-8') as stream:
        result = {'status': 'not_started', 'roundtrip_verified': False,
                  'torque_profile': 'experimental_extreme_crab_16nm', 'samples': []}
        ui = UIClient('http://127.0.0.1:8010', timeout_s=1)
        observer = browser = None
        identity = None
        started = time.monotonic()
        try:
            state = ui.state()
            if state.get('engaged') or state.get('mission_dispatch_active'):
                raise RuntimeError('An operator or mission already owns control')
            client = carla.Client('127.0.0.1', 2000)
            client.set_timeout(1)
            world = client.get_world()
            # HH_261007 - Wait for the existing pacer's first client snapshot;
            # this observer never advances the world or creates a vehicle.
            world.wait_for_tick(3)
            actor = select_unique_ranger(world)
            origin = actor.get_location()
            if actor.get_velocity().length() > .03:
                raise RuntimeError('Vehicle is not stationary')
            observer = RosObservation('ego_vehicle', expected_actor_id=actor.id)
            # HH_261007 - Leave the idle diagnostic/admin view through a normal
            # page navigation before attaching the stricter mission UI driver.
            page = ManualDriveBrowser('http://127.0.0.1:9224', 'http://127.0.0.1:8010')
            if page.snapshot().get('armed'):
                page.close()
                raise RuntimeError('Manual owner became active')
            page._call('Page.navigate', {'url': 'http://127.0.0.1:8010/'})
            page.close()
            for _ in range(20):
                observer.spin_once(.1)
                time.sleep(.1)
            browser = OperatorBrowserClient('http://127.0.0.1:9224',
                'http://127.0.0.1:8010', observation_hook=observer.pump_ui_callbacks)
            result['dispatch'] = browser.dispatch('B1', 'delivery')
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                observer.spin_once(.1)
                state = ui.state()
                candidate = mission_identity(state)
                if (state.get('mission_dispatch_active') and candidate.get('site') == 'B1'
                        and candidate.get('owner') == 'operator'
                        and candidate.get('intent') == 'delivery'):
                    identity = candidate
                    break
            if identity is None:
                raise RuntimeError('Dispatch identity not confirmed; no automatic retry')
            result['identity'] = identity
            deadline = time.monotonic() + 90
            last_phase = None
            while time.monotonic() < deadline:
                observer.spin_once(.05)
                state = ui.state()
                if mission_identity(state) != identity:
                    raise RuntimeError('Mission ownership changed; no further control')
                snap = observer.snapshot()
                sample = capture_sample(carla, actor, sequence=len(result['samples']),
                    elapsed_s=time.monotonic()-started, wall_time_ns=time.time_ns())
                phase = (snap.get('site') or {}).get('operating_state')
                result['samples'].append({'elapsed_s':time.monotonic()-started,
                    'site':snap.get('site'), 'gate':snap.get('gate'),
                    'physical':sample, 'service_state':state.get('service_state_name')})
                if phase != last_phase:
                    print('B1 phase:', phase, flush=True); last_phase = phase
                transform = sample['actor']['transform']
                r = transform['rotation_degrees']
                if abs(r['pitch']) > 18 or abs(r['roll']) > 18:
                    raise RuntimeError('Upright bound exceeded')
                if actor.get_location().distance(origin) > 10 or actor.get_velocity().length() > 2:
                    raise RuntimeError('Short-test physical motion bound exceeded')
                if state.get('mission_execution_error') or phase == 'ERROR':
                    raise RuntimeError('B1 controller reported failure')
                if phase == 'WAIT_RETURN':
                    result['status'] = 'entry_reached_wait_return'
                    break
                time.sleep(.1)
            else:
                result['status'] = 'bounded_timeout'
        except Exception as error:
            result.update(status='not_completed', error=str(error))
        finally:
            # HH_261007 - Stop only this test's still-owned mission. Never cancel
            # a replacement user mission or inject any bypass/teleport/RPC torque.
            if identity is not None:
                try:
                    if mission_identity(ui.state()) == identity:
                        result['stop'] = ui.stop()
                except Exception as error:
                    result['stop_error'] = str(error)
            if observer:
                observer.close()
            if browser:
                browser.close()
            json.dump(result, stream, ensure_ascii=False, indent=2)
            print(json.dumps({k:v for k,v in result.items()
                  if k not in ('samples','dispatch')},ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
