"""Run the real entrypoints without installing, cleaning, or cloning anything."""
# HH_260911 - Guard checkout identity and side-effect-free help on real worktrees.
import os
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[2]

@pytest.mark.parametrize('name', ['setup_camrod.sh', 'colcon_build.sh'])
def test_actual_entrypoint_uses_its_own_checkout(name, tmp_path):
    env = dict(os.environ)
    env.pop('CAMROD_BUILD_ROOT', None)
    result = subprocess.run(['bash', str(ROOT / name), '--print-paths'],
                            cwd=tmp_path, env=env, capture_output=True,
                            text=True, check=True)
    paths = dict(line.split('=', 1) for line in result.stdout.splitlines())
    assert Path(paths['SRC_ROOT']).resolve() == ROOT.resolve()
    assert Path(paths['BUILD_BASE']).parent == Path(paths['WS_ROOT'])
    assert Path(paths['INSTALL_BASE']).parent == Path(paths['WS_ROOT'])
    assert not Path(paths['BUILD_BASE']).is_relative_to(ROOT)

@pytest.mark.parametrize('name', ['setup_camrod.sh', 'colcon_build.sh'])
def test_output_override_cannot_be_inside_checkout(name, tmp_path):
    env = dict(os.environ, CAMROD_BUILD_ROOT=str(ROOT / 'audit-forbidden-output'))
    result = subprocess.run(['bash', str(ROOT / name), '--print-paths'],
                            cwd=tmp_path, env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert not (ROOT / 'audit-forbidden-output').exists()

@pytest.mark.parametrize('name', ['setup_camrod.sh', 'colcon_build.sh'])
def test_help_does_not_create_build_directories(name, tmp_path):
    output = tmp_path / 'not-created'
    env = dict(os.environ, CAMROD_BUILD_ROOT=str(output))
    result = subprocess.run(['bash', str(ROOT / name), '--help'],
                            cwd=tmp_path, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert not output.exists()

@pytest.mark.parametrize('name', ['setup_camrod.sh', 'colcon_build.sh'])
def test_explicit_output_root_keeps_source_identity(name, tmp_path):
    output = tmp_path / 'isolated-build'
    env = dict(os.environ, CAMROD_BUILD_ROOT=str(output))
    result = subprocess.run(['bash', str(ROOT / name), '--print-paths'],
                            cwd=tmp_path, env=env, capture_output=True,
                            text=True, check=True)
    paths = dict(line.split('=', 1) for line in result.stdout.splitlines())
    assert Path(paths['SRC_ROOT']).resolve() == ROOT.resolve()
    assert Path(paths['WS_ROOT']).resolve() == output.resolve()
    assert not output.exists()
