#!/usr/bin/env python3
"""HH_261103 - Derive clearly labelled GIFs only from frozen actual04 media, with source hashes and exact clip offsets."""
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess

from PIL import Image, ImageDraw, ImageFont, ImageOps

HERE = Path(__file__).resolve().parent
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
START, LENGTH, WIDTH, FPS = 100, 60, 960, 5
VIDEO_WIDTH = 768


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    outputs = {name: HERE / name for name in ('phase_progress.gif', 'autodock_continuous_100_160_768.gif',
                'dock_clip_palette_768.png', 'visuals_manifest.json')}
    assert not any(path.exists() for name, path in outputs.items() if name != 'phase_progress.gif'), 'refuse to overwrite any derived evidence'
    video = HERE / 'live_desktop.mp4'
    result_path = HERE / 'result.json'
    result = json.loads(result_path.read_text())
    assert len(result['photos']) == 15
    commands = []

    def run(command):
        commands.append(command)
        return subprocess.run(command, check=True, capture_output=True, text=True).stdout

    probe = json.loads(run(['ffprobe', '-v', 'error', '-show_entries',
            'format=duration,start_time:stream=codec_name,width,height,r_frame_rate,avg_frame_rate,nb_frames:format_tags=creation_time',
            '-of', 'json', str(video)]))
    assert float(probe['format']['duration']) >= START + LENGTH
    assert probe['streams'][0]['avg_frame_rate'] == '5/1'
    sources = [{'name': 'live_desktop.mp4', 'sha256': digest(video)},
               {'name': 'result.json', 'sha256': digest(result_path)},
               {'name': 'make_visuals.py', 'sha256': digest(Path(__file__))}]
    frames, photos = [], []
    font = ImageFont.truetype(FONT, 17)
    small = ImageFont.truetype(FONT, 15)
    for index, row in enumerate(result['photos'], 1):
        source = Path(row['image']['path']).resolve()
        assert source.parent == HERE and source.is_file()
        original = Image.open(source).convert('RGB')
        image = ImageOps.contain(original, (WIDTH, 1040), Image.Resampling.LANCZOS)
        canvas = Image.new('RGB', (WIDTH, 1120), '#111820')
        canvas.paste(image, ((WIDTH - image.width) // 2, 80 + (1040 - image.height) // 2))
        draw = ImageDraw.Draw(canvas)
        message = (row.get('parking') or {}).get('message', '')
        phase = re.search(r'\bphase=(\S+)', message)
        phase = phase.group(1) if phase else 'not recorded'
        draw.text((12, 6), 'ACTUAL04 PNG SEQUENCE {}/15 | captured {}'.format(index, row['utc']), font=font, fill='white')
        draw.text((12, 30), '{} | phase={} | charging={}'.format(row.get('service_state'), phase,
                  row.get('platform_is_charging')), font=small, fill='#d3e2ef')
        draw.text((12, 54), 'Nonuniform capture intervals; fixed 1.5s/photo. NOT continuous video.', font=small, fill='#ffc56a')
        frames.append(canvas.convert('P', palette=Image.Palette.ADAPTIVE, colors=128))
        photos.append({'file': source.name, 'capture_utc_from_result': row['utc'], 'phase': phase,
                       'service_state': row.get('service_state'), 'charging': row.get('platform_is_charging'),
                       'sha256': digest(source)})
    phase_bytes = io.BytesIO()
    frames[0].save(phase_bytes, format='GIF', save_all=True, append_images=frames[1:],
                   duration=[1500] * 14 + [3000], loop=0, disposal=2, optimize=True)
    if outputs['phase_progress.gif'].exists():
        # HH_261103 - Reuse the identical prior montage rather than overwrite evidence after a size-only video retry.
        assert outputs['phase_progress.gif'].read_bytes() == phase_bytes.getvalue()
    else:
        with outputs['phase_progress.gif'].open('xb') as stream:
            stream.write(phase_bytes.getvalue())

    # HH_261103 - Keep original temporal cadence; add only a source-offset header outside the actual desktop pixels.
    visual_filter = ('fps=5,scale=768:-1:flags=lanczos,pad=iw:ih+36:0:36:black,'
        "drawtext=fontfile={}:text='ACTUAL04 continuous source +100 to +160 s | 5 fps | 1x':x=12:y=9:fontsize=15:fontcolor=white".format(FONT))
    common = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-n', '-threads', '2',
              '-ss', str(START), '-t', str(LENGTH), '-i', str(video)]
    run(common + ['-vf', visual_filter + ',palettegen=max_colors=64:stats_mode=diff',
                  '-frames:v', '1', '-update', '1', str(outputs['dock_clip_palette_768.png'])])
    run(common + ['-i', str(outputs['dock_clip_palette_768.png']), '-filter_complex_threads', '2',
                  '-filter_complex', '[0:v]' + visual_filter + '[v];[v][1:v]paletteuse=dither=none:diff_mode=rectangle',
                  '-an', '-loop', '0', str(outputs['autodock_continuous_100_160_768.gif'])])
    generated = []
    for name in ('phase_progress.gif', 'autodock_continuous_100_160_768.gif'):
        path = outputs[name]
        assert path.stat().st_size < 15 * 1024 * 1024
        with Image.open(path) as gif:
            assert gif.width <= 1280
            duration = 0
            for frame in range(gif.n_frames):
                gif.seek(frame)
                duration += gif.info.get('duration', 0)
            generated.append({'file': name, 'sha256': digest(path), 'bytes': path.stat().st_size,
                              'width': gif.width, 'height': gif.height, 'frames': gif.n_frames,
                              'playback_duration_ms': duration})
    assert generated[1]['frames'] == 300 and generated[1]['playback_duration_ms'] == 60000
    inspection = []
    for second in (100, 130, 160):
        path = HERE / ('video_at_{}s.png'.format(second))
        if path.exists():
            inspection.append({'file': path.name, 'source_offset_s': second, 'sha256': digest(path),
                'command': ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-n', '-ss', str(second),
                            '-i', str(video), '-frames:v', '1', '-vf', 'scale=960:-1', str(path)]})
    manifest = {'schema': 'camrod.actual04.visual_derivatives.v1',
                'scope': 'Frozen actual04 only; no AI images, UI modifications, fake camera frames or later-cycle footage.',
                'source_video_probe': probe, 'sources': sources, 'photos': photos,
                'source_clip': {'start_offset_s': START, 'end_offset_s': START + LENGTH,
                                'fps': FPS, 'speed': '1x', 'absolute_utc_mapping': 'not established by MP4 metadata'},
                'phase_gif_limit': 'Captured PNG montage; nonuniform source gaps; fixed 1.5 seconds/photo and final3 seconds.',
                'continuous_gif_limit': 'Continuous 60-second desktop excerpt only; scaled/color-quantized looping GIF, not full mission or mechanical charger proof.',
                'commands': commands, 'inspection_frames': inspection, 'outputs': generated,
                'rejected_intermediates': [{'file': 'autodock_continuous_100_160.gif',
                    'reason': 'First960px render exceeded15MiB; retained unmodified, not a published final GIF.',
                    'bytes': (HERE / 'autodock_continuous_100_160.gif').stat().st_size,
                    'sha256': digest(HERE / 'autodock_continuous_100_160.gif')}]}
    with outputs['visuals_manifest.json'].open('x') as stream:
        json.dump(manifest, stream, indent=2)
        stream.write('\n')
    print(json.dumps(generated, indent=2))


if __name__ == '__main__':
    main()
