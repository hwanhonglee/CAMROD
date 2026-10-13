#!/usr/bin/env python3
"""HH_261007 - Read-only surface rays near the archived forward-return shock.

No world ticking, settings, spawning, controls, teleportation or map edits.
All hits are retained; this probe does not certify drivable clearance.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    import carla
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/virtual_carla'))
    from record_physical_wheel_telemetry import capture_sample, select_unique_ranger
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(3.0)
    world = client.get_world()
    expected = 'Woraksan_camrod_b2_b4_clearance_b3safe_tag_tilt10_v224_dropzone'
    if world.get_map().name.rsplit('/', 1)[-1] != expected:
        raise RuntimeError('unexpected map')
    # A read-only wait observes the existing pacer; never call world.tick().
    world.wait_for_tick(3.0)
    actor = select_unique_ranger(world)
    before = capture_sample(carla, actor, 0, 0, time.time_ns())
    if math.sqrt(sum(v*v for v in before['actor']['linear_velocity_mps'].values())) > 0.02:
        raise RuntimeError('vehicle is moving')
    report = {'utc': datetime.now(timezone.utc).isoformat(),
              'operation': 'read_only_vertical_rays', 'map': world.get_map().name,
              'physical_before': before, 'rays': [],
              'origin_note': 'Archived ROS odometry mirrored to CARLA: (x, -y, z).',
              'limitations': 'Rays may intersect the stationary vehicle or vegetation. '
                             'Raw hits are not a drivable-surface or historical contact proof.'}
    origin = (23.275726318359375, 26.79469871520996)
    yaw = math.radians(44.24)
    started = time.monotonic()
    for index in range(-15, 16):
        station = index / 10.0
        for lateral in (-0.6, -0.3, 0.0, 0.3, 0.6):
            if time.monotonic() - started > 45:
                raise RuntimeError('probe exceeded 45-second budget')
            x = origin[0] + math.cos(yaw)*station - math.sin(yaw)*lateral
            y = origin[1] + math.sin(yaw)*station + math.cos(yaw)*lateral
            hits = world.cast_ray(carla.Location(x=x, y=y, z=5),
                                  carla.Location(x=x, y=y, z=0))
            report['rays'].append({'station_m': station, 'lateral_m': lateral,
                'x': x, 'y': y, 'hits': [
                    {'xyz_m': [h.location.x, h.location.y, h.location.z],
                     'label': str(h.label)} for h in hits]})
    report['physical_after'] = capture_sample(carla, actor, 1, time.monotonic()-started, time.time_ns())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'output': str(args.output), 'rays': len(report['rays']),
                      'scenario_pass': None, 'world_mutated': False}))


if __name__ == '__main__':
    main()
