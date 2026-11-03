"""HH_261103 - Check portable paths and fail-closed scene guards without builds, launches or Git writes."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('service_scene_guard', ROOT / 'tools/verify_scene.py')
scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scene)


@pytest.mark.parametrize('name', ['setup_overlay.sh', 'overlay.sh'])
def test_shell_syntax_and_no_host_paths(name):
    path = ROOT / 'tools' / name
    subprocess.run(['bash', '-n', str(path)], check=True)
    assert '/home/' not in path.read_text()


def test_setup_archives_refs_and_checks_compatibility_before_application():
    source = (ROOT / 'tools/setup_overlay.sh').read_text()
    assert 'rev-parse "${UNIT2_REF}^{commit}"' in source
    assert '"${actual}" == "${UNIT2_SHA}"' in source
    assert '! -e "${target}"' in source
    assert 'git -C "${native_repo}" archive "${UNIT2_SHA}"' in source
    assert source.index('git apply --no-index --check') < source.index('git apply --no-index "')
    assert source.count('for patch in native_parking_compat.patch native_underlay_compat.patch') == 2
    assert 'git checkout' not in source and 'git reset' not in source


def test_build_scope_preserves_virtual_bringup():
    source = (ROOT / 'tools/overlay.sh').read_text()
    assert '/src/colcon_build.sh' in source
    assert '--packages-select camrod_control camrod_sensing camrod_ui ground_segmentation_ros2 camrod_carla_service_test' in source
    assert 'camrod_sensing/external/ground_segmentation_ros2' in source
    assert 'for package in camrod_bringup camrod_carla_adapter' in source
    assert 'validate_runtime_gates.py' in source and 'verify_scene.py' in source
    assert 'skip_gate' not in source and 'fake_sensors:=true' not in source


@pytest.fixture
def patched_build(tmp_path):
    # HH_261103 - Reconstruct only the unpatched sanitizer in a temporary copy; never run the build wrapper or alter a repo.
    text = (ROOT.parent / 'colcon_build.sh').read_text()
    start = text.index('# HH_260428 - Sanitize stale install paths')
    end = text.index('sanitize_path_var() {', start)
    text = text[:start] + ('# HH_260428 - Sanitize stale install paths from prefix vars to avoid picking up\n'
                          '# packages from a previous build that no longer exists on disk.\n') + text[end:]
    allowed = ('    if [[ "${item}" != /opt/ros/* && \\\n'
               '          "${item}" != "${WS_ROOT}/install/"* ]] && \\\n'
               '       ! _is_allowed_extra_prefix "${item}"; then\n      continue\n    fi')
    text = text.replace(allowed, '    if [[ "${item}" != /opt/ros/* && "${item}" != "${WS_ROOT}/install/"* ]]; then continue; fi')
    path = tmp_path / 'colcon_build.sh'
    path.write_text(text)
    patch = str(ROOT / 'tools/native_underlay_compat.patch')
    subprocess.run(['git', 'apply', '--no-index', '--check', patch], cwd=tmp_path, check=True)
    subprocess.run(['git', 'apply', '--no-index', patch], cwd=tmp_path, check=True)
    result = path.read_text()
    function = result[result.index('_is_allowed_extra_prefix() {'):result.index('\nsanitize_path_var() {')]
    return function


@pytest.mark.parametrize('roots,candidate,accepted', [
    ('allowed', 'allowed', True), ('allowed', 'allowed/pkg', True),
    ('allowed', 'allowed-other/pkg', False), ('allowed', 'allowed/../outside', False),
    ('', 'allowed/pkg', False), ('allowed:second', 'second/pkg', True)])
def test_underlay_patch_applies_and_explicit_roots_have_path_boundaries(patched_build, tmp_path, roots, candidate, accepted):
    env = dict(os.environ)
    env['CAMROD_EXTRA_PREFIX_ROOTS'] = ':'.join(str(tmp_path / r) for r in roots.split(':') if r)
    command = patched_build + '\n_is_allowed_extra_prefix "$1"\n'
    result = subprocess.run(['bash', '-eu', '-c', command, 'unit-test', str(tmp_path / candidate)], env=env)
    assert (result.returncode == 0) is accepted


def test_simulation_domain_and_manual_envelope_are_set_before_virtual_environment():
    source = (ROOT / 'tools/overlay.sh').read_text()
    boundary = source.index('source "${CAMROD_SRC_ROOT}/scripts/virtual_carla/env.sh"')
    prefix = source[:boundary]
    assert 'export ROS_DOMAIN_ID=188 ROS_LOCALHOST_ONLY=1 CAMROD_RECORDING_ENVIRONMENT=simulation' in prefix
    assert 'export CAMROD_MANUAL_LINEAR_LIMIT_MPS=2.0 CAMROD_MANUAL_LATERAL_LIMIT_MPS=2.0' in prefix


def test_each_new_package_has_explicit_colcon_prefix():
    source = (ROOT / 'tools/overlay.sh').read_text()
    assert 'COLCON_CURRENT_PREFIX="${test_ws}/install/${package}"' in source
    assert source.index('COLCON_CURRENT_PREFIX=') < source.index('source "${test_ws}/install/${package}/share/${package}/package.bash"')


def test_run_checks_dds_and_uses_tested_sensor_model_paths():
    source = (ROOT / 'tools/overlay.sh').read_text()
    launch = source.index('exec ros2 launch')
    assert source.index('virtual_carla_require_dds_transport') < launch
    assert 'export YOLOV9_MODEL_PATH="${CAMROD_CARLA_YOLO_MODEL_PATH}"' in source
    assert 'launch_sensor_relay:=true compressed_image_max_rate_hz:=5.0 raw_image_max_rate_hz:=10.0' in source


def receipt(tmp_path):
    artifact = tmp_path / 'unit_fixture_asset'
    artifact.write_bytes(b'unit fixture only; not a CARLA map or published receipt')
    binding = {'asset': {'path': str(artifact), 'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest()}}
    value = {'schema': 'camrod.service_charging_geometry.stage.v1', 'status': 'APPLIED', 'errors': [],
             'source_disk_reload_contract_unchanged': True, 'non_tag_actor_contracts_unchanged': True,
             'target_disk_reload_verified': True, 'native_normal_station_bytes_unchanged': True,
             'earlier_scene_bytes_unchanged': True,
             'target_map_package': '/Game/map_package/Maps/Woraksan_v27_charge_anchor_20261103/Woraksan_v27_charge_anchor_20261103',
             'charging_geometry': {'charging_behind_station_m': .60},
             'map_change': {'only_tag_actor_locations_modified': True, 'tag_actor_count_before': 46,
                            'tag_actor_count_after': 46, 'actor_count_before': 627, 'actor_count_after': 627}}
    for key in ('source_files', 'target_files_after', 'protected_files_after', 'charging_geometry_protected_files'):
        value[key] = binding
    path = tmp_path / 'unit_fixture_receipt.json'
    path.write_text(json.dumps(value))
    return path, value, artifact


def test_scene_rehash_rejects_changed_assets(tmp_path):
    path, _, artifact = receipt(tmp_path)
    assert scene.verify(path).endswith('Woraksan_v27_charge_anchor_20261103')
    artifact.write_bytes(b'drift')
    with pytest.raises(ValueError):
        scene.verify(path)


@pytest.mark.parametrize('change', ['empty_bindings', 'wrong_map', 'geometry_changed', 'missing_invariant'])
def test_scene_rejects_incomplete_or_wrong_contract(tmp_path, change):
    path, value, _ = receipt(tmp_path)
    if change == 'empty_bindings':
        value['protected_files_after'] = {}
    elif change == 'wrong_map':
        value['target_map_package'] = 'not-the-test-map'
    elif change == 'geometry_changed':
        value['map_change']['tag_actor_count_after'] = 45
    else:
        value['native_normal_station_bytes_unchanged'] = False
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        scene.verify(path)
