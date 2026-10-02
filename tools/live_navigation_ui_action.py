#!/usr/bin/env python3
"""HH_261002 - Run one explicit local simulator scenario action via visible UI clicks."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/virtual_carla'))
from camping_site_matrix import OperatorBrowserClient, UIClient, mission_identity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['dispatch', 'return'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--site', default='B9')
    # HH_261002 - Exercise delivery and recall through the same visible operator
    # UI; preserve mission ownership and never synthesize arrival/completion.
    parser.add_argument('--intent', choices=['delivery', 'recall'], default='delivery')
    parser.add_argument('--final-return', action='store_true')
    args = parser.parse_args()
    client = OperatorBrowserClient('http://127.0.0.1:9224', 'http://127.0.0.1:8010', timeout_s=15)
    try:
        before = UIClient('http://127.0.0.1:8010').state()
        if args.action == 'dispatch':
            if before.get('mission_dispatch_active'):
                raise RuntimeError('A mission is already active; do not replace it')
            result = client.dispatch(args.site, args.intent)
        else:
            result = client.request_return({'mission_identity': mission_identity(before),
                                            'final_return': args.final_return})
        record = {'utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  'action': args.action, 'intent': args.intent, 'result': result,
                  'before_state': before, 'after_state': UIClient('http://127.0.0.1:8010').state()}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(record, ensure_ascii=False, indent=2))
        print(json.dumps({'action': args.action, 'result': result}, ensure_ascii=False, indent=2))
    finally:
        client.close()


if __name__ == '__main__':
    main()
