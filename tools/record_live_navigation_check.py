#!/usr/bin/env python3
"""HH_261002 - Record a bounded real UI mission without issuing any commands."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'camrod_ui/tools'))
from capture_driving_preview import Devtools


def fetch(url):
    with urlopen(url, timeout=4) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generation', required=True, type=int)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=900)
    args = parser.parse_args()
    if not 1 <= args.duration <= 1800:
        parser.error('duration must be 1..1800 seconds')
    args.output.mkdir(parents=True, exist_ok=True)
    frames = args.output / 'frames'
    frames.mkdir(exist_ok=True)
    pages = fetch('http://127.0.0.1:9224/json')
    page = next(p for p in pages if p.get('type') == 'page' and p.get('url') == 'http://127.0.0.1:8010/')
    client = Devtools(page['webSocketDebuggerUrl'])
    client.call('Runtime.discardConsoleEntries')
    client.call('Runtime.enable')
    started = time.monotonic()
    stop = False

    def terminate(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    seen_active = False
    previous = None
    previous_phase = None
    last_capture = -20
    frame = 0
    samples = 0
    distance = 0.0
    transitions = []
    exceptions = []
    last = None
    finish = 'timeout'
    try:
        with (args.output / 'live_samples.jsonl').open('w', encoding='utf-8') as stream:
            while not stop and time.monotonic() - started < args.duration:
                elapsed = time.monotonic() - started
                try:
                    snapshot = fetch('http://127.0.0.1:8010/api/driving')
                    mission = snapshot['mission']
                    if mission.get('active') and mission.get('generation') != args.generation:
                        finish = 'different_mission'; break
                    seen_active |= bool(mission.get('active'))
                    pose = snapshot.get('pose')
                    if pose and previous and pose.get('frame_id') == previous.get('frame_id'):
                        step = math.hypot(pose['x'] - previous['x'], pose['y'] - previous['y'])
                        if 0.0001 <= step <= 2:
                            distance += step
                    previous = pose
                    row = {'utc': datetime.now(timezone.utc).isoformat(), 'elapsed_s': round(elapsed, 3),
                           **{key: snapshot.get(key) for key in ('connected', 'mission', 'pose', 'motion', 'progress')}}
                    row['pose_sample_path_m'] = distance
                    phase = (mission.get('service_state_name'), mission.get('phase'))
                    if phase != previous_phase:
                        transitions.append(row)
                        print(json.dumps(row, ensure_ascii=False), flush=True)
                        previous_phase = phase
                    if elapsed - last_capture >= 10 or not mission.get('active'):
                        row['render'] = client.evaluate("JSON.parse(document.querySelector('canvas[data-navigation-state]')?.dataset.navigationState || 'null')")
                        row['area_labels'] = client.evaluate("[...document.querySelectorAll('[data-testid=driving-map-area-label]')].map(e=>e.textContent)")
                        client.capture(frames / f'{frame:04d}.png')
                        frame += 1
                        last_capture = elapsed
                    stream.write(json.dumps(row, ensure_ascii=False) + '\n')
                    stream.flush()
                    last = row
                    samples += 1
                    if seen_active and not mission.get('active'):
                        finish = 'mission_inactive'; break
                except Exception as error:
                    exceptions.append({'elapsed_s': round(elapsed, 3), 'error': str(error)[:500]})
                    print(str(error), flush=True)
                time.sleep(1)
        if stop:
            finish = 'stopped_by_recorder_owner'
    finally:
        # HH_261002 - Position sampling is approximate and distinct from the
        # known simulator speed/time-axis distance issue in the mission journal.
        report = {'evidence_type': 'real_connected_UI_and_API_observation',
                  'generation': args.generation, 'finish': finish, 'elapsed_s': time.monotonic() - started,
                  'samples': samples, 'png_frames': frame, 'pose_sample_path_m': distance,
                  'transitions': transitions, 'last': last, 'observation_errors': exceptions,
                  'runtime_exceptions': [e['params'] for e in client.events if e.get('method') == 'Runtime.exceptionThrown'],
                  'scripts': client.evaluate('[...document.scripts].map(s=>s.src).filter(Boolean)')}
        (args.output / 'observation_summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        client.socket.close()
        print(json.dumps({'finish': finish, 'samples': samples, 'frames': frame, 'path_m': distance}), flush=True)


if __name__ == '__main__':
    main()
