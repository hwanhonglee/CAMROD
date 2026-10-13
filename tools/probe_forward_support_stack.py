"""HH_261007 - Passive local CARLA suspension-corridor collision audit.

Retain every hit. A lower support hit never makes a nearby upper collision
safe; this is a narrow regression check, not whole-map or driving acceptance.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time


CONTACTS = (
    ('FL_before', 23.75373, 26.87512, 2.27234),
    ('FL_after', 23.76913, 26.87914, 2.27439),
)


def evaluate_support_heights(heights, expected_z):
    """No minimum-hit shortcut: report all nearby blocking surface layers."""
    if not math.isfinite(expected_z) or any(not math.isfinite(z) for z in heights):
        raise ValueError('non-finite collision height')
    supports = [z for z in heights if abs(z - expected_z) <= .015]
    blockers = [z for z in heights if expected_z + .03 < z <= expected_z + .7]
    return {'expected_support_present': bool(supports),
            'nearby_upper_collision_z_m': blockers,
            'clear_suspension_band': bool(supports) and not blockers,
            'scope': 'two archived FL contact points; not complete path acceptance'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--map', required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('refusing to overwrite evidence')
    import carla
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(4)
    world = client.get_world()
    if world.get_map().name != args.map.removeprefix('/Game/'):
        raise RuntimeError('actual map does not match explicit selected map')
    world.wait_for_tick(3)  # Observe only; this process never owns ticking.
    actors = list(world.get_actors().filter('vehicle.ranger.default'))
    if len(actors) != 1 or actors[0].get_velocity().length() > .02:
        raise RuntimeError('expected one stationary Ranger')
    before = actors[0].get_transform()
    report = {'utc': datetime.now(timezone.utc).isoformat(), 'read_only': True,
              'map': world.get_map().name, 'actor_id': actors[0].id,
              'actor_before': str(before), 'samples': [],
              'car_hits_retained_but_not_ground': True,
              'scope': 'archived wheel suspension-support band, not obstacle clearance'}
    start = time.monotonic()
    for name, x, y, expected_z in CONTACTS:
        hits = world.cast_ray(carla.Location(x=x, y=y, z=4),
                              carla.Location(x=x, y=y, z=1))
        encoded = [{'label': str(h.label), 'xyz_m': [h.location.x, h.location.y, h.location.z]}
                   for h in hits]
        heights = [h.location.z for h in hits if h.label != carla.CityObjectLabel.Car]
        report['samples'].append({'name': name, 'xy_m': [x, y], 'expected_z_m': expected_z,
                                  'all_hits': encoded,
                                  'evaluation': evaluate_support_heights(heights, expected_z)})
    report['actor_after'] = str(actors[0].get_transform())
    report['elapsed_s'] = time.monotonic() - start
    report['clear_at_both_points'] = all(s['evaluation']['clear_suspension_band']
                                         for s in report['samples'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({'output': str(args.output),
                      'clear_at_both_points': report['clear_at_both_points']}))


if __name__ == '__main__':
    main()
