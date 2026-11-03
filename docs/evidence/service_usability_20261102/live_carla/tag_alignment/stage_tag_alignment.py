#!/usr/bin/env python3
"""Clone V224 and rigidly align only its exact 46 tag planes with dz7144."""
from __future__ import print_function

import math
import os
import sys
import traceback

ROOT = os.environ.get('RANGER_CARLA_ROOT', '/home/hong/Downloads/ranger-carla-4ws-pipeline')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import ue_stage_woraksan_v224_dropzone as previous

exact, base, unreal, StageAbort = previous.exact, previous.base, previous.unreal, previous.StageAbort
SOURCE_NAME = previous.TARGET_NAME
TARGET_NAME = 'Woraksan_v27_service_tag_20261102'
SOURCE_PACKAGE = '/Game/map_package/Maps/{0}/{0}'.format(SOURCE_NAME)
TARGET_PACKAGE = '/Game/map_package/Maps/{0}/{0}'.format(TARGET_NAME)
SOURCE_CENTER_CM = exact._add(previous.SOURCE_CENTER_CM, previous.DELTA_CM)
SOURCE_MAP_SHA256 = '1af1a6d160f8ee24a97294b104e5be9ca6aec767fc456abcf51f43a0ee4b1343'
REPORT_ROOT = os.path.join(ROOT, '.work/evidence/service_usability_tag_alignment_20261102')
APPLY_TOKEN = 'APPLY_CLONED_EXACT46_DZ7144_TAG_ALIGNMENT'
LOCATION_TOLERANCE_CM = 0.02
AXIS_TOLERANCE = 0.00001

# HH_261102 - Use the authored station and calibrated camera; do not move the robot or terrain to chase a tag.
STATION_XY = (-8.47366, 40.391)
BODY_YAW_DEG = 91.7873
REAR_CAMERA_BODY = (-0.61933, 0.0, 0.30013)
FINAL_OPTICAL_DEPTH_M = 0.4
MAP_OFFSET = (6.952184129722062, 9.490185712639594)
MAP_YAW_RAD = 8.142811119961662e-7


def geometry():
    yaw = math.radians(BODY_YAW_DEG)
    forward = (math.cos(yaw), math.sin(yaw))
    camera = tuple(STATION_XY[i] + REAR_CAMERA_BODY[0] * forward[i] for i in (0, 1))
    tag_map = tuple(camera[i] - FINAL_OPTICAL_DEPTH_M * forward[i] for i in (0, 1))
    dx, dy = tag_map[0] - MAP_OFFSET[0], tag_map[1] - MAP_OFFSET[1]
    c, s = math.cos(MAP_YAW_RAD), math.sin(MAP_YAW_RAD)
    target = (100.0 * (c * dx + s * dy), -100.0 * (-s * dx + c * dy), SOURCE_CENTER_CM[2])
    old_yaw = math.degrees(math.atan2(-0.9907780087456202, -0.13549515631956704))
    target_yaw = -BODY_YAW_DEG + math.degrees(MAP_YAW_RAD)
    delta_yaw = (target_yaw - old_yaw + 180.0) % 360.0 - 180.0
    return {'station_ros_xy_m': STATION_XY, 'body_yaw_ros_deg': BODY_YAW_DEG,
            'rear_camera_ros_xy_m': camera, 'rear_camera_body_xyz_m': REAR_CAMERA_BODY,
            'target_tag_ros_xy_m': tag_map, 'target_tag_ue_cm': target,
            'rear_optical_depth_m': FINAL_OPTICAL_DEPTH_M,
            'target_front_normal_ue_yaw_deg': target_yaw, 'rigid_ue_yaw_delta_deg': delta_yaw,
            'z_contract': 'preserve authored absolute tag Z; camera Z depends on settled vehicle pose'}


def predict(record):
    transform, basis = record['contract']['transform'], record['basis']
    value = exact.rigid_tilt_transform(transform['location_cm'], basis['forward'], basis['right'],
                                       basis['up'], SOURCE_CENTER_CM, (0.0, 0.0, 1.0),
                                       geometry()['rigid_ue_yaw_delta_deg'])
    value['location'] = exact._add(value['location'], exact._sub(geometry()['target_tag_ue_cm'], SOURCE_CENTER_CM))
    return value


def inspect(package):
    world = unreal.EditorLevelLibrary.get_editor_world()
    if base.package_from_object(base.object_path(world)) != package:
        raise StageAbort('wrong editor world')
    actors = exact._actor_contracts()
    tags = exact._tag_records(exact._tag_actor_map())
    for label, tag in tags.items():
        exact._require_tag_root_transform_alignment(tag, label,
                {'location_cm': LOCATION_TOLERANCE_CM, 'axis_component': AXIS_TOLERANCE})
    return {'actor_count': len(actors), 'all_actor_contracts': actors,
            'non_tag_actor_contracts': exact._non_tag_contracts(actors), 'tag_records': tags}


def validate(before, after, phase):
    if before['actor_count'] != after['actor_count']:
        raise StageAbort(phase + ': actor inventory changed')
    if before['non_tag_actor_contracts'] != after['non_tag_actor_contracts']:
        raise StageAbort(phase + ': non-tag contract changed')
    if set(before['tag_records']) != set(after['tag_records']) or len(after['tag_records']) != 46:
        raise StageAbort(phase + ': exact46 tag inventory changed')
    maximum_error = 0.0
    for label, original in before['tag_records'].items():
        actual = after['tag_records'][label]
        if exact._tag_stable_contract(original) != exact._tag_stable_contract(actual):
            raise StageAbort(phase + ': tag appearance/scale/collision changed ' + label)
        expected = predict(original)
        error = exact._norm(exact._sub(exact._v3(actual['contract']['transform']['location_cm']), expected['location']))
        maximum_error = max(maximum_error, error)
        if error > LOCATION_TOLERANCE_CM:
            raise StageAbort(phase + ': tag position mismatch ' + label)
        for axis in ('forward', 'right', 'up'):
            if not exact._close_vector(actual['basis'][axis], expected[axis], AXIS_TOLERANCE):
                raise StageAbort(phase + ': tag rigid orientation mismatch ' + label)
    old_pairs, new_pairs = exact._pairwise_distances(before['tag_records']), exact._pairwise_distances(after['tag_records'])
    pair_error = max(abs(value - new_pairs[key]) for key, value in old_pairs.items())
    if pair_error > LOCATION_TOLERANCE_CM:
        raise StageAbort(phase + ': tag plane lost rigidity')
    return {'phase': phase, 'tag_count': 46, 'non_tag_contracts_unchanged': True,
            'maximum_position_error_cm': maximum_error, 'maximum_pairwise_error_cm': pair_error}


def execute():
    mode = os.environ.get('RANGER_SERVICE_TAG_MODE', 'PLAN').upper()
    token = os.environ.get('RANGER_SERVICE_TAG_APPLY', '')
    if mode not in ('PLAN', 'APPLY') or (mode == 'PLAN' and token):
        raise StageAbort('invalid PLAN/APPLY mode')
    if mode == 'APPLY' and (token != APPLY_TOKEN or unreal is None):
        raise StageAbort('APPLY requires UE Python and exact token')
    output = os.environ.get('RANGER_SERVICE_TAG_REPORT', os.path.join(REPORT_ROOT, mode.lower() + '.json'))
    if os.path.lexists(output):
        raise StageAbort('report already exists; select a fresh path')
    project = previous.project_root()
    content = os.path.join(project, 'Content')
    source_paths = exact._map_sidecars(content, SOURCE_PACKAGE, SOURCE_NAME)
    targets = exact._map_sidecars(content, TARGET_PACKAGE, TARGET_NAME)
    if os.path.lexists(os.path.dirname(targets['map'])):
        raise StageAbort('target namespace already exists')
    source_files = previous.records(source_paths)
    if source_files['map']['sha256'] != SOURCE_MAP_SHA256:
        raise StageAbort('accepted V224 source map bytes drifted')
    report = {'schema': 'camrod.service_tag_alignment.stage.v1', 'mode': mode,
              'status': 'PLAN_READY', 'errors': [], 'tool_sha256': base.sha256_file(__file__),
              'source_map_package': SOURCE_PACKAGE, 'target_map_package': TARGET_PACKAGE,
              'source_files': source_files, 'target_files': targets, 'geometry': geometry(),
              'terrain_modifications': False, 'source_tag_center_ue_cm': SOURCE_CENTER_CM}
    if mode == 'PLAN':
        base.write_report(output, report)
        print('PLAN_READY report=' + output)
        return
    try:
        if os.path.realpath(str(unreal.Paths.get_project_file_path())) != os.path.join(project, 'CarlaUE4.uproject'):
            raise StageAbort('wrong Unreal project')
        if base.dirty_packages():
            raise StageAbort('editor has dirty packages before operation')
        if not unreal.EditorLevelLibrary.load_level(SOURCE_PACKAGE):
            raise StageAbort('cannot load source')
        source = inspect(SOURCE_PACKAGE)
        white = source['tag_records'][exact.TAG_WHITE_LABEL]
        if not exact._close_vector(white['contract']['transform']['location_cm'], SOURCE_CENTER_CM, LOCATION_TOLERANCE_CM):
            raise StageAbort('V224 white-base source center drifted')
        protected = {'source_' + key: value for key, value in source_paths.items()}
        for actor in source['all_actor_contracts']:
            component = actor.get('component') or {}
            for ref in [component.get('static_mesh')] + list(component.get('override_materials', [])):
                package = base.package_from_object(ref)
                if package.startswith('/Game/') and package != SOURCE_PACKAGE:
                    path = base.asset_file(content, package, '.uasset')
                    if os.path.isfile(path):
                        protected['asset:' + package] = path
        protected_before = previous.records(protected)
        baseline_dirty = base.dirty_packages()
        if set(baseline_dirty) - {SOURCE_PACKAGE}:
            raise StageAbort('source load dirtied shared assets')
        report.update(status='APPLYING', source_level_contract=source, protected_files_before=protected_before)
        base.write_report(output, report)
        if not unreal.EditorLevelLibrary.new_level_from_template(TARGET_PACKAGE, SOURCE_PACKAGE):
            raise StageAbort('clone creation failed')
        clone = inspect(TARGET_PACKAGE)
        if (previous.normalized(source['all_actor_contracts'], SOURCE_PACKAGE) !=
                previous.normalized(clone['all_actor_contracts'], TARGET_PACKAGE)):
            raise StageAbort('pristine clone differs beyond world-local ownership')
        # HH_261102 - Rotate every cell about one pivot before translation so the printed code cannot shear or mirror.
        for label, actor in sorted(exact._tag_actor_map().items()):
            prediction = predict(clone['tag_records'][label])
            actor.modify()
            if not actor.set_actor_rotation(exact._unreal_rotator(prediction['rotation']), False):
                raise StageAbort('tag rotation failed: ' + label)
            if not actor.set_actor_location(exact._unreal_vector(prediction['location']), False, False):
                raise StageAbort('tag translation failed: ' + label)
        in_memory = validate(clone, inspect(TARGET_PACKAGE), 'before_save')
        dirty_audit = base.require_dirty_delta(baseline_dirty, (TARGET_PACKAGE,), 'exact46 tag alignment')
        if not unreal.EditorLevelLibrary.save_current_level():
            raise StageAbort('target save failed')
        if not unreal.EditorLevelLibrary.load_level(SOURCE_PACKAGE) or inspect(SOURCE_PACKAGE) != source:
            raise StageAbort('source disk reload contract changed')
        if not unreal.EditorLevelLibrary.load_level(TARGET_PACKAGE):
            raise StageAbort('target disk reload failed')
        reloaded = inspect(TARGET_PACKAGE)
        disk = validate(clone, reloaded, 'after_disk_reload')
        final_dirty = base.dirty_packages()
        if set(final_dirty) - {SOURCE_PACKAGE}:
            raise StageAbort('unexpected dirty packages after reload')
        sidecars = base.copy_sidecars(source_files, targets)
        protected_after = previous.records(protected)
        if protected_after != protected_before:
            raise StageAbort('source/shared asset bytes changed')
        report.update(status='APPLIED', target_files_after=previous.records(targets),
                protected_files_after=protected_after, target_tag_records=reloaded['tag_records'],
                source_disk_reload_contract_unchanged=True, non_tag_actor_contracts_unchanged=True,
                target_disk_reload_verified=True, in_memory_validation=in_memory,
                disk_reload_validation=disk, sidecars=sidecars, dirty_audit=dirty_audit,
                dirty_packages_at_end=final_dirty)
        base.write_report(output, report)
        print('APPLIED report=' + output)
    except Exception as error:
        report.update(status='ERROR', errors=[str(error)], traceback=traceback.format_exc())
        base.write_report(output, report)
        raise


if __name__ == '__main__':
    try:
        execute()
    except BaseException:
        traceback.print_exc()
        if unreal is not None:
            os._exit(1)
        raise
