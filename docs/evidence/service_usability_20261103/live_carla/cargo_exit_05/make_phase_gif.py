#!/usr/bin/env python3
"""HH_261103 - Reuse actual04's image/metadata utilities for one labelled actual05 screenshot sequence."""
import json
from pathlib import Path
import sys
from datetime import datetime

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'battery_cycle_04'))
from make_visuals import digest, Image, ImageDraw, ImageFont, ImageOps, FONT


def main():
    output, manifest_path = HERE / 'phase_progress.gif', HERE / 'visuals_manifest.json'
    assert not output.exists() and not manifest_path.exists(), 'refuse to overwrite evidence'
    result = json.loads((HERE / 'result.json').read_text())
    observations = [json.loads(line) for line in (HERE / 'observations.jsonl').read_text().splitlines()]
    departing = next(row for row in observations if row['/api/charging-access']['phase'] == 'departing')
    ready = next(row for row in observations if row['/api/charging-access']['phase'] == 'ready')
    selected = [('01_before.png', result['before']), ('phase_departing.png', departing),
                ('phase_ready.png', ready), ('02_ready.png', result['after'])]
    frames, sources, previous = [], [], None
    font, small = ImageFont.truetype(FONT, 16), ImageFont.truetype(FONT, 15)
    for index, (name, row) in enumerate(selected, 1):
        source = HERE / name
        stamp = datetime.fromisoformat(row['utc'])
        gap = 0.0 if previous is None else (stamp - previous).total_seconds()
        assert gap >= 0
        previous = stamp
        image = ImageOps.contain(Image.open(source).convert('RGB'), (960, 1040), Image.Resampling.LANCZOS)
        canvas = Image.new('RGB', (960, 1120), '#111820')
        canvas.paste(image, ((960 - image.width) // 2, 80 + (1040 - image.height) // 2))
        draw = ImageDraw.Draw(canvas)
        access = row['/api/charging-access']
        draw.text((12, 6), 'ACTUAL05 CARGO PNG {}/4 | state observed {}'.format(index, row['utc']), font=font, fill='white')
        draw.text((12, 30), 'phase={} | charging={} | cargo_ready={}'.format(access['phase'],
                    access['platform_is_charging'], access.get('cargo_access_ready')), font=small, fill='#d3e2ef')
        draw.text((12, 54), 'Source gap +{:.3f}s | fixed screenshot playback | NOT continuous video'.format(gap),
                  font=small, fill='#ffc56a')
        frames.append(canvas.convert('P', palette=Image.Palette.ADAPTIVE, colors=128))
        sources.append({'file': name, 'sha256': digest(source), 'state_observation_utc': row['utc'],
                        'source_gap_s': gap, 'phase': access['phase'], 'charging': access['platform_is_charging'],
                        'cargo_ready': access.get('cargo_access_ready')})
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=[1800, 1800, 1800, 3200],
                   loop=0, disposal=2, optimize=True)
    with Image.open(output) as gif:
        assert gif.n_frames == 4 and gif.width <= 1280 and output.stat().st_size < 15 * 1024 * 1024
    inputs = [HERE / 'result.json', HERE / 'observations.jsonl', Path(__file__),
              HERE.parent / 'run_cargo_exit.py', HERE.parent / 'battery_cycle_04/make_visuals.py']
    manifest = {'schema': 'camrod.actual05.cargo_visual_derivatives.v1', 'sources': sources,
                'metadata_sources': [{'file': str(path.relative_to(HERE.parent)), 'sha256': digest(path)} for path in inputs],
                'command': ['python3', str(Path(__file__))], 'result_status': result['status'],
                'measured_from_result': result.get('measured'),
                'output': {'file': output.name, 'sha256': digest(output), 'bytes': output.stat().st_size,
                           'width': 960, 'height': 1120, 'frames': 4, 'playback_duration_ms': 8600},
                'limits': ['Actual05 PNGs only; no AI image or repainted UI content.',
                           'Nonuniform screenshots, not continuous motion or speed evidence.',
                           'UTC labels are API state observations immediately before the screenshot call, not camera exposure timestamps.',
                           'No claim that retest_05_desktop.mp4 includes the ready terminal; this GIF uses the actual ready PNGs.',
                           'Simulated contact feedback, not hardware charger/CAN or mechanical disconnect proof.']}
    with manifest_path.open('x') as stream:
        json.dump(manifest, stream, indent=2)
        stream.write('\n')
    print(json.dumps(manifest['output'], indent=2))


if __name__ == '__main__':
    main()
