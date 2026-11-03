#!/usr/bin/env python3
"""HH_261103 - Verify the visible charging-screen move against real pose and contact observations."""
import json
import argparse
import os
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).parent
sys.path.insert(0, str(ROOT.parents[1] / 'service_usability_20261102/live_carla'))
from battery_observer import fetch_api, JsonlWriter, utc
from browser import LiveOperatorBrowser
from watch_return import validate_runtime


def displacement(start, end):
    dx, dy = end['x'] - start['x'], end['y'] - start['y']
    yaw = start['yaw']
    return {'forward_m': dx * math.cos(yaw) + dy * math.sin(yaw),
            'lateral_m': -dx * math.sin(yaw) + dy * math.cos(yaw),
            'station_error_m': math.hypot(end['x'] + 8.47366, end['y'] - 40.391),
            'yaw_change_rad': math.atan2(math.sin(end['yaw'] - yaw), math.cos(end['yaw'] - yaw))}


def snapshot():
    replies = {route: fetch_api('http://127.0.0.1:8010', route)
               for route in ('/ui/state', '/api/driving', '/api/charging-access')}
    assert all(reply['ok'] for reply in replies.values()), 'actual API unavailable'
    return {'utc': utc(), 'monotonic_s': time.monotonic(),
            **{route: reply['data'] for route, reply in replies.items()}}


def main():
    # HH_261103 - Replays must use the isolated simulator wrapper before any visible motion request.
    validate_runtime(os.environ)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'cargo_exit_04')
    output = parser.parse_args().output
    output.mkdir()
    writer = JsonlWriter(output / 'observations.jsonl', 16 * 1024 * 1024)
    result = {'schema': 'camrod.carla.cargo_exit.v1', 'started_utc': utc(), 'status': 'RUNNING'}
    browser = None
    try:
        before = snapshot()
        access = before['/api/charging-access']
        assert access['battery_percentage'] == 35 and access['platform_is_charging']
        assert access['platform_feedback_fresh'] and access['can_leave']
        assert not before['/ui/state']['mission_dispatch_active']
        start = before['/api/driving']['pose']
        assert start['age_s'] <= .5 and start['frame_id'] == 'map'
        assert before['/api/driving']['motion']['speed_mps'] <= .02
        result['before'] = before
        browser = LiveOperatorBrowser('http://127.0.0.1:9224', 'http://127.0.0.1:8010')
        browser.screenshot(output / '01_before.png')
        result['clicks'] = [browser._click('[data-ui="charging-access-leave"]', 'charging screen move request'),
                            browser._click('[data-ui="charging-confirm-move"]', 'confirm staging move')]
        deadline, ready_since, previous_phase = time.monotonic() + 60, None, None
        while time.monotonic() < deadline:
            row = snapshot()
            writer.write(row)
            access, driving = row['/api/charging-access'], row['/api/driving']
            phase = access.get('phase')
            if phase != previous_phase:
                browser.screenshot(output / ('phase_' + str(phase) + '.png'))
                print(json.dumps({'utc': row['utc'], 'access': access}, ensure_ascii=False), flush=True)
                previous_phase = phase
            assert phase != 'error' and not row['/ui/state'].get('mission_execution_error'), 'native move error'
            pose, motion = driving['pose'], driving['motion']
            if (phase == 'ready' and access['cargo_access_ready'] and access['platform_feedback_fresh']
                    and not access['platform_is_charging'] and motion['age_s'] <= .5
                    and motion['speed_mps'] <= .02 and pose['age_s'] <= .5):
                ready_since = ready_since or time.monotonic()
                if time.monotonic() - ready_since >= 2:
                    delta = displacement(start, pose)
                    assert delta['forward_m'] >= .30 and delta['station_error_m'] <= .21
                    assert abs(delta['yaw_change_rad']) <= .15
                    result.update(status='CARGO_ACCESS_READY_OBSERVED', after=row, measured=delta)
                    browser.screenshot(output / '02_ready.png')
                    break
            else:
                ready_since = None
            time.sleep(.5)
        else:
            raise TimeoutError('60s cargo move observation bound reached')
    except Exception as error:
        result.update(status='FAILED_OR_BLOCKED', error=str(error))
        if browser is not None:
            browser.screenshot(output / 'failure.png')
    finally:
        result['completed_utc'] = utc()
        with (output / 'result.json').open('x') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
        writer.stream.close()
        if browser is not None:
            browser.close()
        print(json.dumps({'status': result['status'], 'error': result.get('error'), 'measured': result.get('measured')}), flush=True)
    return 0 if result['status'] == 'CARGO_ACCESS_READY_OBSERVED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
