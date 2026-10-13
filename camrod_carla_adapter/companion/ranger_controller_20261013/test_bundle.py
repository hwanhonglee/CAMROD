"""HH_261013: isolated regression tests for the guarded companion patch bundle."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


SPEC = importlib.util.spec_from_file_location(
    "ranger_controller_companion_bundle", Path(__file__).with_name("bundle.py")
)
assert SPEC is not None and SPEC.loader is not None
bundle = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = bundle
SPEC.loader.exec_module(bundle)

TRACKED = ("controller.py",)
NEW = ("helpers/new_helper.py",)
BEFORE = b"def steering():\n    return 1\n"
AFTER = b"def steering():\n    return 2\n"
HELPER = b"# companion helper\nVALUE = 2\n"


def git(repo: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


@pytest.fixture
def scenario(tmp_path: Path) -> dict:
    source = tmp_path / "source"
    source.mkdir()
    git(source, "init", "-q")
    git(source, "config", "user.email", "fixture@example.invalid")
    git(source, "config", "user.name", "Fixture")
    (source / "controller.py").write_bytes(BEFORE)
    (source / "unrelated.txt").write_text("unrelated base\n")
    git(source, "add", "controller.py", "unrelated.txt")
    git(source, "commit", "-qm", "base fixture")
    base = git(source, "rev-parse", "HEAD")
    target = tmp_path / "target"
    git(tmp_path, "clone", "-q", str(source), str(target))
    git(target, "config", "user.email", "fixture@example.invalid")
    git(target, "config", "user.name", "Fixture")
    (source / "controller.py").write_bytes(AFTER)
    (source / "helpers").mkdir()
    (source / "helpers/new_helper.py").write_bytes(HELPER)
    return {"source": source, "target": target, "base": base,
            "output": tmp_path / "bundle"}


def generate(scenario: dict) -> dict:
    return bundle.generate_bundle(
        scenario["source"], scenario["output"], scenario["base"], TRACKED, NEW
    )


def snapshot(repo: Path) -> dict:
    """Include the actual index, worktree and status, not only Git's diff."""
    return {
        "head": git(repo, "rev-parse", "HEAD"),
        "status": git(repo, "status", "--porcelain=v1", "--untracked-files=all"),
        "index": (repo / ".git/index").read_bytes(),
        "files": {
            str(p.relative_to(repo)): p.read_bytes()
            for p in repo.rglob("*")
            if p.is_file() and ".git" not in p.relative_to(repo).parts
        },
    }


def test_generation_is_read_only_and_produces_bundle(scenario: dict) -> None:
    before = snapshot(scenario["source"])
    result = generate(scenario)
    assert isinstance(result, dict)
    assert snapshot(scenario["source"]) == before
    assert (scenario["output"] / "controller.patch").is_file()
    manifest = json.loads((scenario["output"] / "MANIFEST.json").read_text())
    assert isinstance(manifest, dict)
    assert scenario["base"] in json.dumps(manifest)


def test_default_check_is_ready_without_worktree_or_index_changes(scenario: dict) -> None:
    generate(scenario)
    before = snapshot(scenario["target"])
    assert bundle.check_or_apply(scenario["target"], scenario["output"]) == "READY"
    assert snapshot(scenario["target"]) == before


def test_explicit_apply_and_repeat_are_idempotent(scenario: dict) -> None:
    generate(scenario)
    target = scenario["target"]
    original_index = (target / ".git/index").read_bytes()
    assert bundle.check_or_apply(target, scenario["output"], apply=True) == "APPLIED"
    assert (target / "controller.py").read_bytes() == AFTER
    assert (target / "helpers/new_helper.py").read_bytes() == HELPER
    assert git(target, "rev-parse", "HEAD") == scenario["base"]
    assert (target / ".git/index").read_bytes() == original_index
    applied = snapshot(target)
    assert bundle.check_or_apply(target, scenario["output"]) == "ALREADY_APPLIED"
    assert bundle.check_or_apply(target, scenario["output"], apply=True) == "ALREADY_APPLIED"
    assert snapshot(target) == applied


def test_unrelated_tracked_and_untracked_dirt_is_retained(scenario: dict) -> None:
    generate(scenario)
    target = scenario["target"]
    (target / "unrelated.txt").write_text("local unrelated edit\n")
    (target / "notes.txt").write_text("untracked personal note\n")
    assert bundle.check_or_apply(target, scenario["output"]) == "READY"
    assert bundle.check_or_apply(target, scenario["output"], apply=True) == "APPLIED"
    assert (target / "unrelated.txt").read_text() == "local unrelated edit\n"
    assert (target / "notes.txt").read_text() == "untracked personal note\n"


def test_unrelated_staged_edit_and_index_are_retained(scenario: dict) -> None:
    generate(scenario)
    target = scenario["target"]
    (target / "unrelated.txt").write_text("staged unrelated edit\n")
    git(target, "add", "unrelated.txt")
    original_index = (target / ".git/index").read_bytes()
    assert bundle.check_or_apply(target, scenario["output"], apply=True) == "APPLIED"
    assert (target / ".git/index").read_bytes() == original_index
    assert git(target, "diff", "--cached", "--name-only") == "unrelated.txt"
    assert (target / "unrelated.txt").read_text() == "staged unrelated edit\n"


@pytest.mark.parametrize("apply", [False, True])
@pytest.mark.parametrize("collision", ["tracked", "new", "mixed_after"])
def test_collisions_fail_closed_without_changes(scenario: dict, apply: bool, collision: str) -> None:
    generate(scenario)
    target = scenario["target"]
    if collision == "tracked":
        (target / "controller.py").write_text("local conflicting controller\n")
    else:
        (target / "helpers").mkdir()
        (target / "helpers/new_helper.py").write_bytes(
            HELPER if collision == "mixed_after" else b"local helper collision\n"
        )
    before = snapshot(target)
    with pytest.raises(bundle.BundleError):
        bundle.check_or_apply(target, scenario["output"], apply=apply)
    assert snapshot(target) == before


@pytest.mark.parametrize("apply", [False, True])
def test_wrong_target_head_is_rejected(scenario: dict, apply: bool) -> None:
    generate(scenario)
    target = scenario["target"]
    git(target, "commit", "--allow-empty", "-qm", "different head")
    before = snapshot(target)
    with pytest.raises(bundle.BundleError):
        bundle.check_or_apply(target, scenario["output"], apply=apply)
    assert snapshot(target) == before


@pytest.mark.parametrize("apply", [False, True])
def test_targeted_staged_changes_are_rejected_even_with_base_worktree(scenario: dict, apply: bool) -> None:
    generate(scenario)
    target = scenario["target"]
    (target / "controller.py").write_bytes(AFTER)
    git(target, "add", "controller.py")
    (target / "controller.py").write_bytes(BEFORE)
    before = snapshot(target)
    with pytest.raises(bundle.BundleError):
        bundle.check_or_apply(target, scenario["output"], apply=apply)
    assert snapshot(target) == before


def test_wrong_generation_head_is_rejected(scenario: dict) -> None:
    git(scenario["source"], "commit", "--allow-empty", "-qm", "different head")
    before = snapshot(scenario["source"])
    with pytest.raises(bundle.BundleError):
        generate(scenario)
    assert snapshot(scenario["source"]) == before
    assert not (scenario["output"] / "MANIFEST.json").exists()


@pytest.mark.parametrize("damage", ["patch", "manifest", "missing_patch", "missing_manifest"])
@pytest.mark.parametrize("apply", [False, True])
def test_corrupt_or_incomplete_bundle_is_rejected(scenario: dict, damage: str, apply: bool) -> None:
    generate(scenario)
    output = scenario["output"]
    if damage == "patch":
        patch = output / "controller.patch"
        patch.write_bytes(patch.read_bytes() + b"\ncorrupted patch bytes\n")
    elif damage == "manifest":
        (output / "MANIFEST.json").write_text("{ malformed JSON")
    elif damage == "missing_patch":
        (output / "controller.patch").unlink()
    else:
        (output / "MANIFEST.json").unlink()
    before = snapshot(scenario["target"])
    with pytest.raises(bundle.BundleError):
        bundle.check_or_apply(scenario["target"], output, apply=apply)
    assert snapshot(scenario["target"]) == before


@pytest.mark.parametrize("damage", ["before_sha", "after_sha", "after_bytes", "after_mode", "empty_files"])
@pytest.mark.parametrize("apply", [False, True])
def test_manifest_file_claims_are_verified_before_target_changes(scenario: dict, damage: str, apply: bool) -> None:
    generate(scenario)
    manifest_path = scenario["output"] / "MANIFEST.json"
    manifest = json.loads(manifest_path.read_text())
    if damage == "before_sha":
        manifest["files"][0]["before"]["sha256"] = "0" * 64
    elif damage == "after_sha":
        manifest["files"][0]["after"]["sha256"] = "0" * 64
    elif damage == "after_bytes":
        manifest["files"][0]["after"]["bytes"] += 1
    elif damage == "after_mode":
        manifest["files"][0]["after"]["mode"] = "100755"
    else:
        manifest["files"] = []
    manifest_path.write_text(json.dumps(manifest))
    before = snapshot(scenario["target"])
    with pytest.raises(bundle.BundleError):
        bundle.check_or_apply(scenario["target"], scenario["output"], apply=apply)
    assert snapshot(scenario["target"]) == before


@pytest.mark.parametrize("missing", [TRACKED[0], NEW[0]])
def test_missing_selected_source_file_is_rejected(scenario: dict, missing: str) -> None:
    (scenario["source"] / missing).unlink()
    before = snapshot(scenario["source"])
    with pytest.raises(bundle.BundleError):
        generate(scenario)
    assert snapshot(scenario["source"]) == before
    assert not (scenario["output"] / "MANIFEST.json").exists()


def test_existing_output_directory_retains_unrelated_files(scenario: dict) -> None:
    scenario["output"].mkdir()
    sentinel = scenario["output"] / "sentinel.txt"
    sentinel.write_text("retain this\n")
    generate(scenario)
    assert sentinel.read_text() == "retain this\n"
    assert (scenario["output"] / "controller.patch").is_file()
    assert (scenario["output"] / "MANIFEST.json").is_file()


@pytest.mark.parametrize("existing", ["controller.patch", "MANIFEST.json"])
def test_existing_bundle_artifact_is_never_overwritten(scenario: dict, existing: str) -> None:
    scenario["output"].mkdir()
    artifact = scenario["output"] / existing
    artifact.write_text("retain this\n")
    with pytest.raises(bundle.BundleError):
        generate(scenario)
    assert artifact.read_text() == "retain this\n"
    assert sorted(p.name for p in scenario["output"].iterdir()) == [existing]


@pytest.mark.parametrize("unsafe", ["../escape.py", "/tmp/escape.py", "helpers/../../escape.py"])
@pytest.mark.parametrize("kind", ["tracked", "new"])
def test_selected_paths_cannot_escape_repository(scenario: dict, unsafe: str, kind: str) -> None:
    before = snapshot(scenario["source"])
    tracked = (unsafe,) if kind == "tracked" else TRACKED
    new = (unsafe,) if kind == "new" else NEW
    with pytest.raises(bundle.BundleError):
        bundle.generate_bundle(scenario["source"], scenario["output"], scenario["base"], tracked, new)
    assert snapshot(scenario["source"]) == before
    assert not (scenario["output"] / "MANIFEST.json").exists()


@pytest.mark.parametrize("linked", [TRACKED[0], NEW[0]])
def test_selected_source_symlink_is_rejected(scenario: dict, linked: str) -> None:
    target = scenario["source"] / linked
    target.unlink()
    target.symlink_to(scenario["source"] / "unrelated.txt")
    with pytest.raises(bundle.BundleError):
        generate(scenario)
    assert target.is_symlink()
    assert not (scenario["output"] / "MANIFEST.json").exists()


@pytest.mark.parametrize("apply", [False, True])
def test_target_symlink_is_rejected(scenario: dict, apply: bool) -> None:
    generate(scenario)
    target = scenario["target"]
    controller = target / "controller.py"
    controller.unlink()
    controller.symlink_to(target / "unrelated.txt")
    unrelated = (target / "unrelated.txt").read_bytes()
    with pytest.raises(bundle.BundleError):
        bundle.check_or_apply(target, scenario["output"], apply=apply)
    assert controller.is_symlink()
    assert (target / "unrelated.txt").read_bytes() == unrelated


@pytest.mark.parametrize("apply", [False, True])
def test_target_parent_symlink_cannot_redirect_new_file(scenario: dict, apply: bool) -> None:
    generate(scenario)
    target = scenario["target"]
    outside = target.parent / "outside"
    outside.mkdir()
    (target / "helpers").symlink_to(outside, target_is_directory=True)
    with pytest.raises(bundle.BundleError):
        bundle.check_or_apply(target, scenario["output"], apply=apply)
    assert not (outside / "new_helper.py").exists()
    assert (target / "controller.py").read_bytes() == BEFORE
