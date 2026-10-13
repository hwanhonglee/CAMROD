#!/usr/bin/env python3
"""HH_261013 - Guarded, unpublished Ranger controller companion; check by default."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


BASE = "10af3fcaf8137c7552486a7ba7e2000c203a0fe3"
PACKAGE = "ros_ws/src/carla_extended_ackermann_control/"
MODULE = PACKAGE + "src/carla_extended_ackermann_control/"
TRACKED_PATHS = (
    "bootstrap/76_create_physical_gate.sh",
    PACKAGE + "launch/carla_extended_ackermann_control.launch.py",
    PACKAGE + "launch/full_stack.launch.py",
    MODULE + "carla_extended_ackermann_control_node.py",
    MODULE + "physical_four_wheel_bridge.py",
    MODULE + "portable_gate_contract.py",
    MODULE + "wheel_torque_control.py",
    PACKAGE + "test/test_physical_four_wheel_bridge.py",
    PACKAGE + "test/test_physical_four_wheel_steering.py",
    PACKAGE + "test/test_portable_gate_contract.py",
    PACKAGE + "test/test_wheel_torque_control.py",
    "tools/carla_portable_live_acceptance.py",
    "tools/tests/test_carla_portable_live_acceptance.py",
)
NEW_PATHS = tuple(MODULE + name + ".py" for name in (
    "crab_yaw_torque_balance", "experimental_crab_yaw_hold",
    "experimental_zero_turn_yaw_integral")) + tuple(
    PACKAGE + "test/test_" + name + ".py" for name in (
        "crab_yaw_torque_balance", "experimental_crab_yaw_hold",
        "experimental_zero_turn_yaw_integral"))


class BundleError(RuntimeError):
    pass


def _git(repo, *args, data=None, env=None, codes=(0,)):
    environment = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    for key in tuple(environment):
        if key in ("GIT_DIR", "GIT_WORK_TREE", "GIT_INDEX_FILE", "GIT_OBJECT_DIRECTORY",
                   "GIT_ALTERNATE_OBJECT_DIRECTORIES") or key.startswith("GIT_CONFIG"):
            environment.pop(key)
    if env:
        environment.update(env)
    result = subprocess.run(["git", "--no-optional-locks", "-C", str(repo), *args],
                            input=data, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            env=environment, check=False, timeout=60)
    if result.returncode not in codes:
        raise BundleError(result.stderr.decode(errors="replace").strip() or "Git validation failed")
    return result.stdout


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _path(repo, name):
    path = Path(name)
    if (path.is_absolute() or not path.parts or ".." in path.parts or ".git" in path.parts
            or any(character in str(name) for character in "\n\r\t\\")):
        raise BundleError(f"Unsafe companion path: {name}")
    target = repo / path
    if any(part.is_symlink() for part in (target, *target.parents) if part != repo.parent):
        raise BundleError(f"Symlink companion path: {name}")
    if repo not in target.resolve().parents:
        raise BundleError(f"Companion path escapes repository: {name}")
    return target


def _state(path):
    if not path.exists():
        return None
    if not path.is_file():
        raise BundleError(f"Not a regular companion file: {path}")
    data = path.read_bytes()
    return dict(sha256=_sha(data), bytes=len(data),
                mode="100755" if path.stat().st_mode & 0o111 else "100644")


def _head(repo, base):
    if not re.fullmatch(r"[0-9a-f]{40}", base):
        raise BundleError("Expected full forty-character base commit")
    if _git(repo, "rev-parse", "HEAD").decode().strip() != base:
        raise BundleError(f"Repository HEAD must exactly equal {base}")
    if Path(_git(repo, "rev-parse", "--show-toplevel").decode().strip()).resolve() != repo:
        raise BundleError("Target must be the actual repository worktree root")


def _base_state(repo, base, name):
    listing = _git(repo, "ls-tree", base, "--", name)
    if not listing:
        return None
    mode, kind, _ = listing.split(b"\t", 1)[0].split()
    if kind != b"blob" or mode not in (b"100644", b"100755"):
        raise BundleError(f"Unsupported base object: {name}")
    data = _git(repo, "show", f"{base}:{name}")
    return dict(sha256=_sha(data), bytes=len(data), mode=mode.decode())


def _patch_paths(repo, patch):
    records = _git(repo, "apply", "--numstat", "-z", "-", data=patch).split(b"\0")
    return [record.split(b"\t", 2)[2].decode() for record in records if record]


def _validate_result(repo, base, patch, files):
    # HH_261013 - Both index AND newly created Git objects stay in a disposable
    # directory. The real repository's objects are read-only alternates.
    objects = Path(_git(repo, "rev-parse", "--git-path", "objects").decode().strip())
    if not objects.is_absolute():
        objects = repo / objects
    with tempfile.TemporaryDirectory(prefix="ranger-companion-index-") as scratch:
        scratch = Path(scratch)
        (scratch / "objects").mkdir()
        env = {"GIT_INDEX_FILE": str(scratch / "index"),
               "GIT_OBJECT_DIRECTORY": str(scratch / "objects"),
               "GIT_ALTERNATE_OBJECT_DIRECTORIES": str(objects.resolve())}
        _git(repo, "read-tree", base, env=env)
        _git(repo, "apply", "--cached", "--check", "-", data=patch, env=env)
        _git(repo, "apply", "--cached", "-", data=patch, env=env)
        for item in files:
            data = _git(repo, "show", ":" + item["path"], env=env)
            mode = _git(repo, "ls-files", "--stage", "--", item["path"], env=env).split()[0].decode()
            if dict(sha256=_sha(data), bytes=len(data), mode=mode) != item["after"]:
                raise BundleError("Patch output does not match manifest after identity")


def generate_bundle(repo, bundle, base=BASE, tracked_paths=TRACKED_PATHS, new_paths=NEW_PATHS):
    """HH_261013 - Only write NEW bundle artifacts; never edit source repo/index."""
    repo, bundle = Path(repo).resolve(), Path(bundle).resolve()
    _head(repo, base)
    if (bundle / "controller.patch").exists() or (bundle / "MANIFEST.json").exists():
        raise BundleError("Bundle artifacts already exist; refusing overwrite")
    names = tuple(tracked_paths) + tuple(new_paths)
    if not names or len(names) != len(set(names)):
        raise BundleError("Empty or duplicate companion selection")
    before_status = _git(repo, "status", "--porcelain=v1", "--untracked-files=all")
    files = []
    for name in names:
        current = _state(_path(repo, name))
        original = _base_state(repo, base, name)
        if current is None or (name in tracked_paths) != (original is not None):
            raise BundleError(f"Missing file or wrong tracked/new selection: {name}")
        files.append(dict(path=name, before=original, after=current))
    patch = _git(repo, "diff", "--binary", "--full-index", "--no-renames", "--no-ext-diff",
                 "--no-textconv", base, "--", *tracked_paths)
    for name in new_paths:
        patch += _git(repo, "diff", "--no-index", "--binary", "--full-index", "--no-ext-diff",
                      "--", "/dev/null", name, codes=(0, 1))
    if sorted(_patch_paths(repo, patch)) != sorted(names):
        raise BundleError("Patch must contain exactly the selected changed files")
    _validate_result(repo, base, patch, files)
    _git(repo, "apply", "--reverse", "--check", "-", data=patch)
    _head(repo, base)
    if before_status != _git(repo, "status", "--porcelain=v1", "--untracked-files=all"):
        raise BundleError("Source status changed during companion generation")
    for item in files:
        if _state(_path(repo, item["path"])) != item["after"]:
            raise BundleError("Source bytes/mode changed during companion generation")
    manifest = dict(schema="camrod.ranger_controller_companion.v1", base_commit=base,
                    upstream="https://github.com/hwanhonglee/ranger-carla-4ws-pipeline.git",
                    upstream_status="Unpublished local companion; not an upstream commit",
                    patch_sha256=_sha(patch), patch_bytes=len(patch), files=files,
                    validation="temporary-index forward check and source-worktree reverse check passed",
                    exclusions=["terrain", "maps", ".work", "raw logs", "runtime authorization"])
    bundle.mkdir(parents=True, exist_ok=True)
    with (bundle / "controller.patch").open("xb") as stream:
        stream.write(patch)
    with (bundle / "MANIFEST.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return manifest


def check_or_apply(repo, bundle, apply=False):
    """HH_261013 - Validate exact before/after bytes; apply only on explicit opt-in."""
    repo, bundle = Path(repo).resolve(), Path(bundle).resolve()
    try:
        manifest = json.loads((bundle / "MANIFEST.json").read_text())
        patch = (bundle / "controller.patch").read_bytes()
        if (manifest["schema"] != "camrod.ranger_controller_companion.v1"
                or _sha(patch) != manifest["patch_sha256"] or len(patch) != manifest["patch_bytes"]):
            raise BundleError("Bundle patch integrity mismatch")
        _head(repo, manifest["base_commit"])
        files = manifest["files"]
        names = [item["path"] for item in files]
        if not names or len(names) != len(set(names)) or sorted(_patch_paths(repo, patch)) != sorted(names):
            raise BundleError("Invalid manifest/patch file closure")
        for item in files:
            _path(repo, item["path"])
            if _base_state(repo, manifest["base_commit"], item["path"]) != item["before"]:
                raise BundleError("Manifest before identity does not match base commit")
        _validate_result(repo, manifest["base_commit"], patch, files)
        states = [_state(_path(repo, item["path"])) for item in files]
        if all(state == item["after"] for state, item in zip(states, files)):
            _git(repo, "apply", "--reverse", "--check", "-", data=patch)
            return "ALREADY_APPLIED"
        if not all(state == item["before"] for state, item in zip(states, files)):
            raise BundleError("Targeted files differ from exact base; preserving local changes")
        if _git(repo, "diff", "--cached", "--name-only", "--", *names):
            raise BundleError("Targeted staged changes exist; refusing to alter worktree")
        _git(repo, "apply", "--check", "-", data=patch)
        if not apply:
            return "READY"
        _git(repo, "apply", "-", data=patch)
        if any(_state(_path(repo, item["path"])) != item["after"] for item in files):
            raise BundleError("Post-apply verification failed; inspect target, no automatic rollback")
        return "APPLIED"
    except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
        raise BundleError(f"Invalid or unavailable companion: {error}") from error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--bundle", type=Path, default=Path(__file__).resolve().parent)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--generate", action="store_true", help="write new companion artifacts only")
    mode.add_argument("--apply", action="store_true", help="explicitly apply verified patch to target repo")
    args = parser.parse_args()
    try:
        if args.generate:
            result = generate_bundle(args.repo, args.bundle)
            print(f"GENERATED {len(result['files'])} files; external repository unchanged")
        else:
            print(check_or_apply(args.repo, args.bundle, args.apply))
        return 0
    except (BundleError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f"Companion refused: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
