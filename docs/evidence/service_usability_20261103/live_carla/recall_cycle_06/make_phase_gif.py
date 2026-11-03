#!/usr/bin/env python3
"""HH_261103 - Summarize six original recall06 screenshots without fabricating continuous motion."""
import json
from datetime import datetime
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'battery_cycle_04'))
from make_visuals import digest, Image, ImageDraw, ImageFont, ImageOps, FONT


def main():
    output, metadata = HERE / 'phase_progress.gif', HERE / 'visuals_manifest.json'
    assert not output.exists() and not metadata.exists()
    result = json.loads((HERE / 'result.json').read_text())
    assert len(result['photos']) == 14
    frames, sources, previous = [], [], None
    for number, index in enumerate([0, 3, 6, 7, 12, 13], 1):
        row = result['photos'][index]
        path = Path(row['image']['path']).resolve()
        assert path.parent == HERE
        stamp = datetime.fromisoformat(row['utc'])
        gap = 0.0 if previous is None else (stamp - previous).total_seconds()
        assert gap >= 0
        previous = stamp
        picture = ImageOps.contain(Image.open(path).convert('RGB'), (960, 1040), Image.Resampling.LANCZOS)
        canvas = Image.new('RGB', (960, 1120), '#111820')
        canvas.paste(picture, ((960 - picture.width) // 2, 80 + (1040 - picture.height) // 2))
        draw = ImageDraw.Draw(canvas)
        draw.text((12, 6), 'ACTUAL06 RECALL PNG {}/6 | recorded {}'.format(number, row['utc']),
                  font=ImageFont.truetype(FONT, 16), fill='white')
        draw.text((12, 30), '{} | {}'.format(row['label'], row.get('service_state')),
                  font=ImageFont.truetype(FONT, 15), fill='#d3e2ef')
        draw.text((12, 54), 'Source gap +{:.3f}s | fixed screenshot playback | NOT continuous video'.format(gap),
                  font=ImageFont.truetype(FONT, 15), fill='#ffc56a')
        frames.append(canvas.convert('P', palette=Image.Palette.ADAPTIVE, colors=128))
        sources.append({'file': path.name, 'sha256': digest(path), 'utc_from_result': row['utc'],
                        'source_gap_s': gap, 'label': row['label'], 'service_state': row.get('service_state')})
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=[1500] * 5 + [3000],
                   loop=0, disposal=2, optimize=True)
    with Image.open(output) as gif:
        assert gif.n_frames == 6 and gif.width == 960 and output.stat().st_size < 3 * 1024 * 1024
    record = {'schema': 'camrod.actual06.recall_visual_derivatives.v1', 'result_file': 'result.json',
              'result_sha256': digest(HERE / 'result.json'), 'result_status': result['status'],
              'generator_sha256': digest(Path(__file__)), 'sources': sources,
              'command': ['python3', str(Path(__file__))],
              'output': {'file': output.name, 'sha256': digest(output), 'bytes': output.stat().st_size,
                         'width': 960, 'height': 1120, 'frames': 6, 'playback_duration_ms': 10500},
              'limits': 'Six of14 actual PNGs, fixed1.5s/photo plus final3s; nonuniform gaps labelled. Not continuous motion, MP4, or physical charger proof.'}
    with metadata.open('x') as stream:
        json.dump(record, stream, indent=2)
        stream.write('\n')
    print(json.dumps(record['output']))


if __name__ == '__main__':
    main()
