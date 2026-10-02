"""HH_261002 - Exercise staged UI publication without touching a live workspace."""

import os
from pathlib import Path
import shutil
import subprocess


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def fixture_workspace(tmp_path):
    workspace = tmp_path / "workspace"
    package = workspace / "src/camrod_ui"
    scripts = package / "scripts"
    scripts.mkdir(parents=True)
    for name in ("build_frontend.sh", "sync_frontend_build.sh"):
        shutil.copy2(SCRIPTS / name, scripts / name)
    frontend = package / "camrod_ui_robot/assets/frontend"
    source = frontend / "build"
    installed = workspace / "install/camrod_ui/share/camrod_ui/camrod_ui_robot/assets/frontend/build"
    for directory in (source, installed):
        (directory / "static/js").mkdir(parents=True)
        (directory / "index.html").write_text("old-index")
        (directory / "static/js/main.old.js").write_text("old-bundle")
    (source / "photo.png").write_bytes(b"old-image")
    (installed / "photo.png").symlink_to(source / "photo.png")
    return scripts, frontend, source, installed


def test_staged_publication_preserves_images_and_old_open_tabs(tmp_path):
    scripts, _, source, installed = fixture_workspace(tmp_path)
    stage = tmp_path / "stage"
    (stage / "static/js").mkdir(parents=True)
    (stage / "index.html").write_text('<script src="/static/js/main.new.js"></script>')
    (stage / "static/js/main.new.js").write_text("new-bundle")
    (stage / "photo.png").write_bytes(b"complete-new-image")
    result = subprocess.run(
        ["bash", str(scripts / "sync_frontend_build.sh")],
        env={**os.environ, "CAMROD_FRONTEND_BUILD_SOURCE": str(stage)},
        capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    for directory in (source, installed):
        assert (directory / "photo.png").read_bytes() == b"complete-new-image"
        assert (directory / "static/js/main.new.js").read_text() == "new-bundle"
        assert (directory / "static/js/main.old.js").read_text() == "old-bundle"
        assert (directory / "index.html").read_text() == (stage / "index.html").read_text()
        assert not list(directory.rglob(".asset-publish.*"))


def test_incomplete_build_never_replaces_running_frontend(tmp_path):
    scripts, _, source, installed = fixture_workspace(tmp_path)
    stage = tmp_path / "incomplete"
    stage.mkdir()
    result = subprocess.run(
        ["bash", str(scripts / "sync_frontend_build.sh")],
        env={**os.environ, "CAMROD_FRONTEND_BUILD_SOURCE": str(stage)},
        capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "refusing incomplete" in result.stderr
    for directory in (source, installed):
        assert (directory / "index.html").read_text() == "old-index"
        assert (directory / "photo.png").read_bytes() == b"old-image"


def test_compilation_uses_temporary_output_before_publication():
    source = (SCRIPTS / "build_frontend.sh").read_text()
    assert 'mktemp -d "${frontend_dir}/.ui-build.XXXXXX"' in source
    assert 'BUILD_PATH="${build_stage}" node ' in source
    assert source.index('BUILD_PATH="${build_stage}" node ') < source.index(
        'CAMROD_FRONTEND_BUILD_SOURCE="${build_stage}" bash ')
    assert "set -euo pipefail" in source
