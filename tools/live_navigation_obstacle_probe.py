#!/usr/bin/env python3
"""HH_261002 - Explicit stationary CARLA test actors; never controls/ticks the Ranger.

Only actors recorded in this probe's manifest may be removed. Source world pose
and physical CARLA bounds are saved separately from perception's observed bounds.
"""
import argparse
import json
import math
from pathlib import Path
import time

import carla
import yaml


ROLE = 'camrod_navigation_object_probe_20261002'


def vector(value):
    return {axis: getattr(value, axis) for axis in ('x', 'y', 'z')}


def describe(actor):
    transform = actor.get_transform()
    return {'id': actor.id, 'type_id': actor.type_id,
            'role_name': actor.attributes.get('role_name'),
            'location': vector(transform.location),
            'rotation': {k: getattr(transform.rotation, k) for k in ('roll', 'pitch', 'yaw')},
            'physical_bbox_local_center': vector(actor.bounding_box.location),
            'physical_bbox_size': {k: getattr(actor.bounding_box.extent, k) * 2 for k in ('x', 'y', 'z')}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['spawn', 'inspect', 'remove'])
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--kind', choices=['person', 'car'], default='person')
    parser.add_argument('--forward', type=float, default=5)
    parser.add_argument('--right', type=float, default=2)
    parser.add_argument('--map-x', type=float)
    parser.add_argument('--map-y', type=float)
    parser.add_argument('--allow-distant-spawn', action='store_true')
    parser.add_argument('--yaw-deg', type=float)
    args = parser.parse_args()
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(8)
    world = client.get_world()
    # HH_261002 - A new CARLA client can initially expose an empty actor cache.
    # Observe a bridge-owned tick; never advance the synchronous world ourselves.
    world.wait_for_tick(10)
    if args.action == 'spawn':
        if args.manifest.exists():
            raise RuntimeError('Use a new manifest per spawn to preserve actor ownership')
        egos = [a for a in world.get_actors().filter('vehicle.ranger.*')
                if a.attributes.get('role_name') == 'ego_vehicle']
        if len(egos) != 1:
            raise RuntimeError(f'Expected one Ranger, got {len(egos)}')
        ego = egos[0]
        moving = ego.get_velocity().length() > 0.03
        if moving and not args.allow_distant_spawn:
            raise RuntimeError('Spawn only while Ranger is stationary')
        transform = ego.get_transform()
        front, right = transform.get_forward_vector(), transform.get_right_vector()
        point = transform.location + front * args.forward + right * args.right
        if (args.map_x is None) != (args.map_y is None):
            raise RuntimeError('Both map coordinates are required')
        if args.map_x is not None:
            alignment = yaml.safe_load((Path(__file__).resolve().parents[1] /
                'camrod_carla_adapter/config/woraksan_lane_anchor_alignment.yaml').read_text())['/**']['ros__parameters']
            dx = args.map_x - alignment['map_offset_x_m']
            dy = args.map_y - alignment['map_offset_y_m']
            yaw = alignment['map_yaw_offset_rad']
            point.x = math.cos(yaw) * dx + math.sin(yaw) * dy
            point.y = math.sin(yaw) * dx - math.cos(yaw) * dy
        if moving and (args.map_x is None or point.distance(transform.location) < 10):
            raise RuntimeError('Moving test requires explicit map point at least 10m from Ranger')
        point.z += 4
        ground = world.ground_projection(point, 20)
        if ground is None:
            raise RuntimeError('No ground surface for test actor; do not guess its height')
        point.z = ground.location.z + 0.15
        name = 'walker.pedestrian.0001' if args.kind == 'person' else 'vehicle.tesla.model3'
        bp = world.get_blueprint_library().find(name)
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', ROLE)
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'true')
        actor = world.try_spawn_actor(bp, carla.Transform(point, carla.Rotation(
            yaw=args.yaw_deg if args.yaw_deg is not None else transform.rotation.yaw + 180)))
        if actor is None:
            raise RuntimeError('Test actor spawn rejected (occupied/collision location)')
        record = {'created_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                  'purpose': 'real_CARLA_sensor_UI_test_not_ground_truth_detection_injection',
                  'ego': describe(ego), 'actor': describe(actor),
                  'requested_offset_m': {'forward': args.forward, 'right': args.right},
                  'requested_map_xy': [args.map_x, args.map_y],
                  'ground': vector(ground.location)}
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(json.dumps(record, indent=2))
        if args.kind == 'car':
            actor.apply_control(carla.VehicleControl(hand_brake=True))
        for _ in range(3):
            world.wait_for_tick(5)
        record['actor'] = describe(actor)
        args.manifest.write_text(json.dumps(record, indent=2))
        print(json.dumps(record, indent=2), flush=True)
        return
    record = json.loads(args.manifest.read_text())
    identity = record['actor']
    actor = world.get_actor(identity['id'])
    if actor is None:
        print(json.dumps({'already_absent': identity})); return
    if actor.type_id != identity['type_id'] or actor.attributes.get('role_name') != identity['role_name']:
        raise RuntimeError('Actor identity changed; refuse to touch reused ID')
    if args.action == 'inspect':
        print(json.dumps(describe(actor), indent=2)); return
    state = describe(actor)
    success = actor.destroy()
    record['removed_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    record['removed'] = success
    record['final_actor'] = state
    args.manifest.write_text(json.dumps(record, indent=2))
    print(json.dumps({'removed': success, 'actor': state}, indent=2))


if __name__ == '__main__':
    main()
