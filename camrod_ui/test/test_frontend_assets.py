"""HH_261006 - Detect missing/stale navigation assets before UI installation."""

from pathlib import Path
import runpy
import shutil
import subprocess

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
VERIFIER = runpy.run_path(str(PACKAGE / "scripts/verify_frontend_assets.py"))


def _trees(tmp_path):
    frontend = tmp_path / "frontend"
    public = frontend / "public"
    (public / "models").mkdir(parents=True)
    for relative in VERIFIER["REQUIRED_ASSETS"]:
        (public / relative).write_bytes(b"portable-asset")
    (public / "logo.png").write_bytes(b"logo-image")
    build = frontend / "build"
    shutil.copytree(public, build)
    (build / "static/js").mkdir(parents=True)
    (build / "static/js/main.test.js").write_text("bundle")
    (build / "index.html").write_text('<script src="/static/js/main.test.js"></script>')
    installed = tmp_path / "installed"
    shutil.copytree(build, installed)
    return frontend, build, installed


def test_complete_portable_tree_matches_installed_bytes(tmp_path):
    assert VERIFIER["verify_assets"](*_trees(tmp_path)) == 6


@pytest.mark.parametrize("relative", VERIFIER["REQUIRED_ASSETS"] + ("logo.png",))
def test_missing_installed_asset_is_rejected(tmp_path, relative):
    frontend, build, installed = _trees(tmp_path)
    (installed / relative).unlink()
    with pytest.raises(ValueError, match="missing or empty"):
        VERIFIER["verify_assets"](frontend, build, installed)


def test_stale_texture_is_rejected_even_with_matching_js(tmp_path):
    frontend, build, installed = _trees(tmp_path)
    (installed / "models/woraksan-rear-wrap.png").write_bytes(b"old-image")
    with pytest.raises(ValueError, match="installed UI asset differs"):
        VERIFIER["verify_assets"](frontend, build, installed)


def test_stale_built_model_is_rejected(tmp_path):
    frontend, build, _ = _trees(tmp_path)
    (build / "models/ranger-navigation.glb").write_bytes(b"old-model")
    with pytest.raises(ValueError, match="built UI asset differs"):
        VERIFIER["verify_assets"](frontend, build)


def test_missing_compiled_bundle_is_rejected(tmp_path):
    frontend, build, _ = _trees(tmp_path)
    (build / "static/js/main.test.js").unlink()
    with pytest.raises(ValueError, match="missing or empty"):
        VERIFIER["verify_assets"](frontend, build)


def test_existing_optional_web_font_does_not_change_packaging_contract(tmp_path):
    frontend, build, _ = _trees(tmp_path)
    with (build / "index.html").open("a") as stream:
        stream.write('<link rel="stylesheet" href="https://example.com/optional-font.css">')
    assert VERIFIER["verify_assets"](frontend, build) == 6


def test_fresh_clone_metadata_discovery_does_not_require_build(tmp_path):
    # Metadata remains discoverable before npm, but a real build must fail.
    package = tmp_path / "camrod_ui"
    package.mkdir()
    shutil.copy2(PACKAGE / "setup.py", package / "setup.py")
    (package / "scripts").mkdir()
    shutil.copy2(PACKAGE / "scripts/verify_frontend_assets.py", package / "scripts")
    (package / "runtime/python").mkdir(parents=True)
    metadata = subprocess.run(
        ["python3", "setup.py", "--name"], cwd=package, capture_output=True, text=True,
    )
    assert metadata.returncode == 0, metadata.stderr
    assert metadata.stdout.strip() == "camrod_ui"
    build = subprocess.run(
        ["python3", "setup.py", "build_py"], cwd=package, capture_output=True, text=True,
    )
    assert build.returncode != 0
    assert "colcon_build.sh --packages-select camrod_ui" in build.stderr
