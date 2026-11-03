#!/usr/bin/env python3
"""HH_261103 - Rebind actual incremental checks with the existing fail-closed gate creator."""
import hashlib
import argparse
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).parent
WORKSPACE = Path('/home/hong/camrod_service_test_ws')
PIPELINE = Path('/home/hong/Downloads/ranger-carla-4ws-pipeline')
PREVIOUS = PIPELINE / '.work/evidence/service_usability_live_20261102'
EVIDENCE = PIPELINE / '.work/evidence/service_usability_live_20261103'
RUNTIME = PIPELINE / '.work/runtime/service_usability_live_20261103'
CHECKS = EVIDENCE / 'verification'
TEST_BUILD = WORKSPACE / 'log/service_usability_docking_20261103'
EXPECTED_CHANGES = {
    WORKSPACE / 'src/camrod_control/launch/parking.launch.py',
    # HH_261103 - Reinstallation strips the build RPATH; current native source and tests are checked again.
    WORKSPACE / 'install/camrod_control/lib/camrod_control/drop_zone_maneuver_controller_node',
    # HH_261103 - Reviewed cargo release fix: require charging at admission, not after its own parking handoff.
    WORKSPACE / 'src/camrod_ui/runtime/python/camrod_ui/ui_backend_node.py',
    WORKSPACE / 'install/camrod_ui/lib/python3.10/site-packages/camrod_ui/ui_backend_node.py',
}


def prior_paths(document, allowed_changes):
    paths = []
    for record in document['files'].values():
        path = Path(record['path'])
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != record['sha256'] and path not in allowed_changes:
            raise ValueError('Unreviewed source/artifact drift: ' + str(path))
        paths.append(path)
    return paths


def require_pass(path):
    suites = list(ET.parse(path).getroot().iter('testsuite'))
    if not suites or sum(int(s.get('tests', 0)) for s in suites) < 1:
        raise ValueError('No executed tests: ' + str(path))
    if any(int(s.get('failures', 0)) or int(s.get('errors', 0)) for s in suites):
        raise ValueError('Failed test suite: ' + str(path))


def main():
    global EVIDENCE, RUNTIME
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cargo-release-fix', action='store_true')
    args = parser.parse_args()
    if args.cargo_release_fix:
        EVIDENCE = EVIDENCE.with_name('service_usability_live_20261103_cargo_release')
        RUNTIME = RUNTIME.with_name('service_usability_live_20261103_cargo_release')
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    for name in ('control_ctest', 'source_contracts'):
        require_pass(CHECKS / (name + '.xml'))
    test_xml = WORKSPACE / 'build/camrod_carla_service_test/ctest_results.xml'
    require_pass(test_xml)
    build_logs = [CHECKS / 'native_control_build.log', TEST_BUILD / 'build.log']
    for log in build_logs:
        content = log.read_text()
        if not content.strip() or 'Failed <<<' in content or 'Aborted <<<' in content:
            raise ValueError('Build failure or missing output: ' + str(log))
    if 'Built target apriltag_parking_controller_node' not in build_logs[1].read_text():
        raise ValueError('Missing completed compatibility package build')
    new_package = WORKSPACE / 'src/camrod_carla_service_test'
    common = [Path(__file__), HERE / 'runtime.sh', RUNTIME / 'candidate.env', RUNTIME / 'runtime.env',
              CHECKS / 'package_resolution.json', CHECKS / 'source_contracts.xml', CHECKS / 'source_contracts.log',
              CHECKS / 'control_ctest.xml', CHECKS / 'control_ctest.log', test_xml,
              WORKSPACE / 'install/camrod_control/share/camrod_control/launch/parking.launch.py',
              WORKSPACE / 'src/camrod_control/config/parking_runtime_profiles/disabled.yaml',
              PIPELINE / '.work/evidence/service_usability_tag_alignment_20261103/apply.json']
    common += [p for p in new_package.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    common += list((WORKSPACE / 'build/camrod_carla_service_test/test_results').rglob('*.xml'))
    common += [WORKSPACE / 'install/camrod_carla_service_test/lib/camrod_carla_service_test/apriltag_parking_controller_node']
    common += list((WORKSPACE / 'install/camrod_carla_service_test/share/camrod_carla_service_test/config').glob('*.yaml'))
    common += list((WORKSPACE / 'install/camrod_carla_service_test/share/camrod_carla_service_test/launch').glob('*.py'))
    if args.cargo_release_fix:
        require_pass(HERE / 'cargo_fix_tests.xml')
        common += [HERE / 'cargo_fix_tests.xml', HERE / 'cargo_fix_tests.log',
                   HERE / 'cargo_fix_build.log', WORKSPACE / 'src/camrod_ui/test/test_charging_access_backend.py']
        source = WORKSPACE / 'src/camrod_ui/runtime/python/camrod_ui/ui_backend_node.py'
        installed = WORKSPACE / 'install/camrod_ui/lib/python3.10/site-packages/camrod_ui/ui_backend_node.py'
        assert source.read_bytes() == installed.read_bytes(), 'UI source/install drift'
    for kind, previous_name in (('offline', 'offline_tests'), ('build', 'ros_build')):
        # HH_261103 - Keep unchanged prior build/source evidence; explicitly rebind the two reviewed changes.
        previous = json.loads((PREVIOUS / (previous_name + '.json')).read_text())
        paths = prior_paths(previous, EXPECTED_CHANGES if kind == 'build' else set())
        paths += common + build_logs
        command = [sys.executable, str(PIPELINE / 'tools/create_portable_verification_report.py'),
                   'create', '--kind', kind, '--output', str(EVIDENCE / (previous_name + '.json'))]
        for index, path in enumerate(sorted(set(p.resolve(strict=True) for p in paths))):
            command += ['--file', 'file_{:04d}={}'.format(index, path)]
        subprocess.run(command, check=True)


if __name__ == '__main__':
    main()
