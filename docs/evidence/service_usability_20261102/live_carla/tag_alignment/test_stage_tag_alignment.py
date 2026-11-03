"""HH_261102 - Check exact scene geometry and fail-closed tag-only clone guards before UE runs."""
import copy
import json
import math
from pathlib import Path

import pytest
import stage_tag_alignment as stage


def source():
    path = Path(stage.ROOT) / '.work/evidence/v224_validation/map_dropzone_stage.json'
    records = json.loads(path.read_text())['target_tag_records']
    return {'actor_count': 47, 'non_tag_actor_contracts': [{'label': 'terrain', 'unchanged': True}],
            'tag_records': records}


def changed():
    before = source()
    after = copy.deepcopy(before)
    for label, record in after['tag_records'].items():
        prediction = stage.predict(before['tag_records'][label])
        record['contract']['transform']['location_cm'] = dict(zip('xyz', prediction['location']))
        record['contract']['transform']['rotation_deg'] = prediction['rotation']
        record['basis'] = {a: dict(zip('xyz', prediction[a])) for a in ('forward', 'right', 'up')}
    return before, after


def test_station_camera_and_tag_are_exactly_coaxial_at_point_four_optical_depth():
    value = stage.geometry()
    yaw = math.radians(stage.BODY_YAW_DEG)
    camera, tag = value['rear_camera_ros_xy_m'], value['target_tag_ros_xy_m']
    delta = tuple(tag[i] - camera[i] for i in (0, 1))
    assert delta[0] * -math.cos(yaw) + delta[1] * -math.sin(yaw) == pytest.approx(.4)
    assert delta[0] * math.sin(yaw) - delta[1] * math.cos(yaw) == pytest.approx(0, abs=1e-12)
    assert tag == pytest.approx((-8.441867901686367, 39.37216590585865), abs=1e-10)
    assert value['target_tag_ue_cm'][2] == stage.SOURCE_CENTER_CM[2]
    assert value['rigid_ue_yaw_delta_deg'] == pytest.approx(6.0000000, abs=.0001)


def test_exact46_rigid_transform_preserves_printed_code_and_upward_tilt():
    before, after = changed()
    result = stage.validate(before, after, 'test')
    assert result['maximum_pairwise_error_cm'] < 1e-9
    white = after['tag_records'][stage.exact.TAG_WHITE_LABEL]
    assert stage.exact._v3(white['contract']['transform']['location_cm']) == pytest.approx(stage.geometry()['target_tag_ue_cm'])
    normal = stage.exact._v3(white['basis']['up'])
    assert normal[2] == pytest.approx(math.sin(math.radians(10)), abs=1e-6)
    assert stage.exact.projected_yaw_degrees(normal) == pytest.approx(stage.geometry()['target_front_normal_ue_yaw_deg'], abs=.0001)
    assert len(after['tag_records']) == 46


@pytest.mark.parametrize('mutation', ['non_tag', 'inventory', 'material', 'scale', 'position', 'orientation'])
def test_forbidden_changes_fail_closed(mutation):
    before, after = changed()
    white = after['tag_records'][stage.exact.TAG_WHITE_LABEL]
    if mutation == 'non_tag':
        after['non_tag_actor_contracts'][0]['unchanged'] = False
    elif mutation == 'inventory':
        after['tag_records'].pop(stage.exact.TAG_WHITE_LABEL)
    elif mutation == 'material':
        white['contract']['component']['override_materials'] = ['wrong']
    elif mutation == 'scale':
        white['contract']['transform']['scale']['x'] += .01
    elif mutation == 'position':
        white['contract']['transform']['location_cm']['x'] += 1
    else:
        white['basis']['up']['x'] += .01
    with pytest.raises(stage.StageAbort):
        stage.validate(before, after, 'test')


def test_plan_never_mutates_scene_and_rejects_existing_report(tmp_path, monkeypatch):
    output = tmp_path / 'plan.json'
    monkeypatch.setenv('CARLA_ROOT', '/home/hong/carla')
    monkeypatch.setenv('RANGER_SERVICE_TAG_MODE', 'PLAN')
    monkeypatch.delenv('RANGER_SERVICE_TAG_APPLY', raising=False)
    monkeypatch.setenv('RANGER_SERVICE_TAG_REPORT', str(output))
    monkeypatch.setattr(stage, 'unreal', object())
    # HH_261102 - Keep the inert PLAN test repeatable after the real isolated target has been applied.
    original_paths = stage.exact._map_sidecars
    def paths(content, package, name):
        result = original_paths(content, package, name)
        if package == stage.TARGET_PACKAGE:
            return {role: str(tmp_path / 'fresh_target' / Path(path).name)
                    for role, path in result.items()}
        return result
    monkeypatch.setattr(stage.exact, '_map_sidecars', paths)
    stage.execute()
    report = json.loads(output.read_text())
    assert report['status'] == 'PLAN_READY'
    assert report['source_files']['map']['sha256'] == stage.SOURCE_MAP_SHA256
    assert report['terrain_modifications'] is False
    assert report['target_map_package'] != report['source_map_package']
    with pytest.raises(stage.StageAbort, match='report already exists'):
        stage.execute()


@pytest.mark.parametrize('mode,token', [('PLAN', 'unexpected'), ('APPLY', ''), ('APPLY', 'wrong'), ('DELETE', '')])
def test_mode_and_apply_token_fail_closed(mode, token, monkeypatch):
    monkeypatch.setenv('RANGER_SERVICE_TAG_MODE', mode)
    monkeypatch.setenv('RANGER_SERVICE_TAG_APPLY', token)
    with pytest.raises(stage.StageAbort):
        stage.execute()
