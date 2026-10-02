#!/usr/bin/env python3
"""HH_261002 - Capture the existing live page and GET state without robot commands."""
import argparse
import json
from pathlib import Path
import sys
from datetime import datetime, timezone
from urllib.request import urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'camrod_ui/tools'))
from capture_driving_preview import Devtools


def fetch(url):
    with urlopen(url, timeout=5) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--url', default='http://127.0.0.1:8010/')
    parser.add_argument('--debug-port', type=int, default=9224)
    parser.add_argument('--include-records', action='store_true')
    args = parser.parse_args()
    page = next(page for page in fetch(f'http://127.0.0.1:{args.debug_port}/json')
                if page.get('type') == 'page' and page.get('url') == args.url)
    snapshot = fetch(args.url.rstrip('/') + '/api/driving')
    client = Devtools(page['webSocketDebuggerUrl'])
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        client.capture(args.output.with_suffix('.png'))
        record = {'captured_utc': datetime.now(timezone.utc).isoformat(),
                  'evidence_type': 'live_page_and_received_telemetry',
                  'scenario_success_inferred': False, 'snapshot': snapshot,
                  'navigation_render_state': client.evaluate(
                      "JSON.parse(document.querySelector('canvas[data-navigation-state]')"
                      "?.dataset.navigationState || 'null')")}
        # HH_261002 - Save a read-only bounded journal view as evidence, never
        # copy or mutate the field database, and never infer distance accuracy.
        if args.include_records:
            record['mission_records'] = fetch(args.url.rstrip('/') + '/api/mission-records?limit=3')
        record['loaded_scripts'] = client.evaluate(
            'Array.from(document.scripts).map(s => s.src).filter(Boolean)')
        args.output.with_suffix('.json').write_text(json.dumps(record, ensure_ascii=False, indent=2))
        print(json.dumps({'mission': snapshot.get('mission'), 'output': str(args.output)}, ensure_ascii=False))
    finally:
        client.socket.close()


if __name__ == '__main__':
    main()
