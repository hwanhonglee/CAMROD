"""HH_261013 - Offline publication guards never touch original CARLA evidence."""

import copy
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import sys
from urllib.parse import unquote, urlsplit

import pytest


TOOLS = Path(__file__).resolve().parents[1]
SCRIPT = TOOLS / "publish_virtual_carla_evidence.py"
EVIDENCE = Path("docs/evidence/virtual_carla")
SELECTIONS = (
    ("B1", "v5", "B1_attempt02"),
    ("B2", "v6", "B2"),
    ("B3", "v6", "B3"),
    ("B4", "v5", "B4"),
    ("B5", "v5", "B5"),
    ("B6", "v5", "B6_attempt02"),
    ("B7", "v5", "B7_attempt02"),
    ("B8", "v5", "B8_attempt02"),
    ("B9", "v7", "B9"),
    ("B10", "v9_restart1", "B10"),
    ("B11", "v5", "B11_attempt02"),
    ("B12", "v5", "B12_attempt02"),
    ("B13", "v5", "B13_attempt02"),
)
CASE_FILES = (
    "ANALYSIS.md",
    "supervisor.json",
    "trial.json",
    "visual/representative_contact_sheet.png",
    "visual/representative_motion.gif",
    "visual/capture_manifest.json",
    "visual/sha256sums.txt",
    "visual/ffprobe.json",
    "visual/exact_commands.txt",
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def write_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")


def refresh_visual_hashes(case):
    visual = case / "visual"
    names = (
        "representative_contact_sheet.png",
        "representative_motion.gif",
        "ffprobe.json",
        "exact_commands.txt",
        "capture_manifest.json",
    )
    (visual / "sha256sums.txt").write_text(
        "".join(f"{digest((visual / name).read_bytes())}  {name}\n" for name in names),
        encoding="utf-8",
    )


def make_case(root, site, version, name):
    case = root / EVIDENCE / f"site_completion_20261012_{version}" / name
    visual = case / "visual"
    visual.mkdir(parents=True)
    actor = 100 + int(site[1:])
    write_json(case / "trial.json", {
        "schema": "camrod.carla.site_roundtrip.diagnostic.v1",
        "samples": [{"stage": "fixture", "note": "complete original sample retained"}],
        "result": {
            "status": "COMPLETED_OBSERVED",
            "roundtrip_verified": True,
            "formal_acceptance": False,
            "actor_id": actor,
            "site": site,
            "identity": {"generation": 1000 + actor, "site": site},
            "elapsed_wall_s": 20.0,
            "sampled_xy_m": 12.0,
            "sample_count": 1,
            "outbound": {"elapsed_wall_s": 8.0, "sampled_xy_m": 5.0},
            "return_leg": {"elapsed_wall_s": 12.0, "sampled_xy_m": 7.0},
            "final_observation": {
                "pose": {"x_m": -8.47366, "y_m": 40.391, "z_m": 0.0},
                "parking": {
                    "reverse": {"operating_state": "PARKED"},
                    "dispatcher": {"operating_state": "PARKED"},
                },
                "gate": {"operating_state": "STANDBY"},
                "collision_evidence": {
                    "publisher_count": 1,
                    "subscriber_and_publisher_discovered": True,
                    "event_count": 0,
                    "sampled_event_count": 0,
                    "omitted_event_count": 0,
                    "stale_queued_event_count": 0,
                },
            },
        },
    })
    write_json(case / "supervisor.json", {
        "site": site,
        "status": "finished",
        "actor_id": actor,
        "trial_exit_code": 0,
        "capture_exit_code": 0,
        "result_status": "COMPLETED_OBSERVED",
    })
    (case / "ANALYSIS.md").write_text(
        f"# {site}\n\n[Full trial](trial.json)\n\n"
        "![Capture](visual/representative_contact_sheet.png)\n",
        encoding="utf-8",
    )
    png = b"\x89PNG\r\n\x1a\n" + site.encode()
    gif = b"GIF89a" + site.encode()
    (visual / "representative_contact_sheet.png").write_bytes(png)
    (visual / "representative_motion.gif").write_bytes(gif)
    write_json(visual / "ffprobe.json", {"streams": [{"avg_frame_rate": "5/1"}]})
    (visual / "exact_commands.txt").write_text("fixture capture command\n", encoding="utf-8")
    write_json(visual / "capture_manifest.json", {
        "status": "PASS",
        "scope": {"kind": "actual X11 desktop-region recording"},
        "recording": {"capture_fps": 5},
        "derivatives": {"gif_fps": 5},
        "x11": {"same_titles_and_geometry_before_after_capture": True},
        "artifacts": {
            "contact_sheet_png": {
                "path": "representative_contact_sheet.png", "sha256": digest(png),
                "bytes": len(png),
            },
            "representative_gif": {
                "path": "representative_motion.gif", "sha256": digest(gif),
                "bytes": len(gif),
            },
        },
    })
    refresh_visual_hashes(case)
    return case


@pytest.fixture
def publisher(monkeypatch):
    # Importing this utility must stay independent of CARLA/ROS and motion APIs.
    for name in ("carla", "rclpy"):
        monkeypatch.setitem(sys.modules, name, None)
    spec = importlib.util.spec_from_file_location("publication_under_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, spec.name, module)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def evidence_tree(tmp_path):
    root = tmp_path / "source"
    receipts = tmp_path / "receipts"
    cases = [make_case(root, *selection) for selection in SELECTIONS]
    receipt = receipts / "site_completion_20261012_v5" / "ros_build.json"
    write_json(receipt, {"status": "PASS", "kind": "offline fixture receipt"})
    central = root / EVIDENCE / "site_completion_20261012_RESULTS.md"
    historical = root / EVIDENCE / "site_completion_20261012_v4" / "B5" / "trial.json"
    write_json(historical, {"status": "historical raw evidence not selected"})
    central.write_text(
        "# Selected mixed-version observations\n\n"
        f"[Selected full trial]({cases[0] / 'trial.json'})\n\n"
        f"[Receipt]({receipt})\n\n"
        "[Historical trial](site_completion_20261012_v4/B5/trial.json)\n\n"
        "[External documentation](https://example.invalid/reference)\n",
        encoding="utf-8",
    )
    audit = root / EVIDENCE / "site_completion_20261012_v9_restart1" / "FINAL_13_SITE_AUDIT.md"
    audit.write_text(
        "# Final audit\n\n[Central report](../site_completion_20261012_RESULTS.md)\n",
        encoding="utf-8",
    )
    return {
        "root": root,
        "receipts": receipts,
        "cases": cases,
        "central": central,
        "audit": audit,
        "receipt": receipt,
        "historical": historical,
        "output": root / "release",
    }


def plan_for(publisher, tree, output=None):
    return publisher.build_plan(tree["root"], tree["receipts"], output or tree["output"])


def entry_for(plan, source):
    return next(entry for entry in plan["entries"] if Path(entry["source"]) == source)


def destination_for(plan, source):
    return Path(plan["output_root"]) / entry_for(plan, source)["destination"]


def tree_bytes(root, exclude=None):
    return {
        path.relative_to(root): path.read_bytes()
        for path in root.rglob("*")
        if path.is_file() and (exclude is None or exclude not in path.parents)
    }


def test_selection_is_exactly_the_audited_mixed_version_13(publisher):
    assert tuple(publisher.SELECTIONS) == SELECTIONS


def test_default_plan_is_read_only_and_records_source_integrity(publisher, evidence_tree):
    tree = evidence_tree
    before = tree_bytes(tree["root"])
    plan = publisher.build_plan(tree["root"], tree["receipts"])
    expected_output = tree["root"] / EVIDENCE / "releases" / "20261013"
    assert Path(plan["output_root"]) == expected_output
    assert not expected_output.exists()
    assert not expected_output.parent.exists()
    assert tree_bytes(tree["root"]) == before
    trial_sources = {str(case / "trial.json") for case in tree["cases"]}
    assert trial_sources <= {str(entry["source"]) for entry in plan["entries"]}
    for entry in plan["entries"]:
        original = Path(entry["source"]).read_bytes()
        assert entry["sha256"] == digest(original)
        assert entry["size"] == len(original)
        assert entry["mode"]


def test_export_preserves_full_trials_and_selected_artifacts(publisher, evidence_tree):
    tree = evidence_tree
    before_source = tree_bytes(tree["root"])
    before_receipts = tree_bytes(tree["receipts"])
    plan = plan_for(publisher, tree)
    result = publisher.execute_plan(plan)
    assert isinstance(result, dict)
    manifest = tree["output"] / "MANIFEST.json"
    assert manifest.is_file()
    manifest_text = manifest.read_text(encoding="utf-8")
    manifest_data = json.loads(manifest_text)
    assert manifest_data["publication_complete"] is True
    assert len(manifest_data["cases"]) == 13
    for artifact in manifest_data["artifacts"]:
        published = (tree["output"] / artifact["path"]).read_bytes()
        assert artifact["sha256"] == digest(published)
        assert artifact["bytes"] == len(published)
    for case in tree["cases"]:
        original = (case / "trial.json").read_bytes()
        compressed = destination_for(plan, case / "trial.json")
        assert compressed.name == "trial.json.gz"
        assert gzip.decompress(compressed.read_bytes()) == original
        assert compressed.read_bytes()[4:8] == b"\x00\x00\x00\x00"
        assert digest(original) in manifest_text
        for name in CASE_FILES:
            if name in ("trial.json", "ANALYSIS.md"):
                continue
            source = case / name
            assert destination_for(plan, source).read_bytes() == source.read_bytes()
    assert destination_for(plan, tree["receipt"]).read_bytes() == tree["receipt"].read_bytes()
    assert tree_bytes(tree["root"], exclude=tree["output"]) == before_source
    assert tree_bytes(tree["receipts"]) == before_receipts


def test_gzip_bytes_are_deterministic_across_separate_exports(publisher, evidence_tree):
    first = plan_for(publisher, evidence_tree)
    second = plan_for(publisher, evidence_tree, evidence_tree["root"] / "second-release")
    publisher.execute_plan(first)
    publisher.execute_plan(second)
    for case in evidence_tree["cases"]:
        source = case / "trial.json"
        assert destination_for(first, source).read_bytes() == destination_for(second, source).read_bytes()


def test_rewritten_links_resolve_and_omitted_history_is_explicit(publisher, evidence_tree):
    tree = evidence_tree
    plan = plan_for(publisher, tree)
    publisher.execute_plan(plan)
    central = destination_for(plan, tree["central"])
    text = central.read_text(encoding="utf-8")
    assert "trial.json.gz" in text
    assert "LOCAL_ARCHIVE" in text
    assert "https://example.invalid/reference" in text
    assert not any(Path(entry["source"]) == tree["historical"] for entry in plan["entries"])
    for document in (central, destination_for(plan, tree["cases"][0] / "ANALYSIS.md")):
        for target in re.findall(r"\]\(([^)]+)\)", document.read_text(encoding="utf-8")):
            parsed = urlsplit(target.strip("<>"))
            if parsed.scheme or not parsed.path:
                continue
            assert not Path(unquote(parsed.path)).is_absolute()
            assert (document.parent / unquote(parsed.path)).is_file(), target


@pytest.mark.parametrize("name", CASE_FILES)
def test_missing_selected_artifact_fails_before_creating_output(publisher, evidence_tree, name):
    (evidence_tree["cases"][0] / name).unlink()
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


@pytest.mark.parametrize("key", ["central", "audit", "receipt"])
def test_missing_required_document_or_linked_receipt_fails_closed(publisher, evidence_tree, key):
    evidence_tree[key].unlink()
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


@pytest.mark.parametrize("kind", ["directory", "file"])
def test_existing_destination_is_never_overwritten(publisher, evidence_tree, kind):
    tree = evidence_tree
    plan = plan_for(publisher, tree)
    if kind == "directory":
        tree["output"].mkdir()
        sentinel = tree["output"] / "untouched.txt"
    else:
        sentinel = tree["output"]
    sentinel.write_bytes(b"existing user data")
    with pytest.raises(publisher.ExportError):
        publisher.execute_plan(plan)
    assert sentinel.read_bytes() == b"existing user data"


@pytest.mark.parametrize("operation", ["change", "delete"])
def test_changed_source_after_planning_cannot_publish_manifest(publisher, evidence_tree, operation):
    tree = evidence_tree
    plan = plan_for(publisher, tree)
    source = tree["cases"][-1] / "trial.json"
    if operation == "change":
        source.write_bytes(source.read_bytes() + b" ")
    else:
        source.unlink()
    with pytest.raises(publisher.ExportError):
        publisher.execute_plan(plan)
    assert not (tree["output"] / "MANIFEST.json").exists()


@pytest.mark.parametrize("absolute", [False, True])
def test_tampered_plan_cannot_escape_destination(publisher, evidence_tree, absolute):
    tree = evidence_tree
    plan = copy.deepcopy(plan_for(publisher, tree))
    escaped = tree["output"].parent / "escaped.json.gz"
    entry_for(plan, tree["cases"][0] / "trial.json")["destination"] = (
        str(escaped) if absolute else "../escaped.json.gz"
    )
    with pytest.raises(publisher.ExportError):
        publisher.execute_plan(plan)
    assert not escaped.exists()
    assert not (tree["output"] / "MANIFEST.json").exists()


def test_malformed_capture_manifest_is_rejected(publisher, evidence_tree):
    case = evidence_tree["cases"][0]
    (case / "visual/capture_manifest.json").write_text("{not valid JSON", encoding="utf-8")
    refresh_visual_hashes(case)
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


def test_visual_hash_mismatch_is_rejected(publisher, evidence_tree):
    visual = evidence_tree["cases"][0] / "visual"
    (visual / "representative_motion.gif").write_bytes(b"GIF89a-tampered")
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


def test_manifest_artifact_hash_mismatch_is_rejected_even_with_valid_hash_list(publisher, evidence_tree):
    case = evidence_tree["cases"][0]
    path = case / "visual/capture_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    manifest["artifacts"]["contact_sheet_png"]["sha256"] = "0" * 64
    write_json(path, manifest)
    refresh_visual_hashes(case)
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


@pytest.mark.parametrize("field,value", [
    ("status", "FAILED"),
    ("roundtrip_verified", False),
    ("formal_acceptance", True),
    ("actor_id", 99999),
])
def test_invalid_selected_trial_cannot_be_published(publisher, evidence_tree, field, value):
    path = evidence_tree["cases"][0] / "trial.json"
    trial = json.loads(path.read_text(encoding="utf-8"))
    trial["result"][field] = value
    write_json(path, trial)
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


def test_cli_defaults_to_plan_without_creating_release(publisher, evidence_tree, capsys):
    tree = evidence_tree
    status = publisher.main([
        "--source-root", str(tree["root"]),
        "--receipt-root", str(tree["receipts"]),
        "--output-root", str(tree["output"]),
    ])
    assert status == 0
    output = json.loads(capsys.readouterr().out)
    assert output["mode"] == "PLAN_ONLY"
    assert output["cases"] == 13
    assert not tree["output"].exists()


@pytest.mark.parametrize("name", ["trial.json", "supervisor.json", "visual/capture_manifest.json"])
@pytest.mark.parametrize("shape", [{}, [], None])
def test_evidence_with_invalid_structure_raises_export_error(publisher, evidence_tree, shape, name):
    case = evidence_tree["cases"][0]
    write_json(case / name, shape)
    refresh_visual_hashes(case)
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


@pytest.mark.parametrize("field,value", [
    (("status",), "FAIL"),
    (("scope", "kind"), "generated visualization"),
    (("recording", "capture_fps"), 1),
    (("derivatives", "gif_fps"), 1),
    (("x11", "same_titles_and_geometry_before_after_capture"), False),
])
def test_capture_qualification_is_enforced(publisher, evidence_tree, field, value):
    case = evidence_tree["cases"][0]
    path = case / "visual/capture_manifest.json"
    manifest = json.loads(path.read_text(encoding="utf-8"))
    item = manifest
    for component in field[:-1]:
        item = item[component]
    item[field[-1]] = value
    write_json(path, manifest)
    refresh_visual_hashes(case)
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)


@pytest.mark.parametrize("counter", [
    "event_count", "sampled_event_count", "omitted_event_count", "stale_queued_event_count",
])
def test_each_collision_counter_must_be_zero(publisher, evidence_tree, counter):
    path = evidence_tree["cases"][0] / "trial.json"
    trial = json.loads(path.read_text(encoding="utf-8"))
    trial["result"]["final_observation"]["collision_evidence"][counter] = 1
    write_json(path, trial)
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)


@pytest.mark.parametrize("operation", ["grow", "same-size", "truncate", "delete"])
def test_source_mutation_during_copy_cannot_publish_manifest(
    publisher, evidence_tree, monkeypatch, operation,
):
    tree = evidence_tree
    plan = plan_for(publisher, tree)
    source = tree["cases"][0] / "visual/representative_motion.gif"
    original_copy = publisher._copy_verified
    original_bytes = source.read_bytes()

    def mutate_then_copy(entry, target):
        if Path(entry["source"]) == source:
            if operation == "delete":
                source.unlink()
            elif operation == "grow":
                source.write_bytes(original_bytes + b"changed")
            elif operation == "same-size":
                source.write_bytes(b"X" * len(original_bytes))
            else:
                source.write_bytes(original_bytes[:2])
        return original_copy(entry, target)

    monkeypatch.setattr(publisher, "_copy_verified", mutate_then_copy)
    with pytest.raises(publisher.ExportError):
        publisher.execute_plan(plan)
    assert tree["output"].exists()
    assert not (tree["output"] / "MANIFEST.json").exists()


def test_symlinked_required_source_is_not_published(publisher, evidence_tree, tmp_path):
    source = evidence_tree["cases"][0] / "trial.json"
    replacement = tmp_path / "outside-trial.json"
    replacement.write_bytes(source.read_bytes())
    source.unlink()
    source.symlink_to(replacement)
    with pytest.raises(publisher.ExportError):
        plan_for(publisher, evidence_tree)
    assert not evidence_tree["output"].exists()


def test_tampered_plan_hash_cannot_publish_manifest(publisher, evidence_tree):
    plan = plan_for(publisher, evidence_tree)
    entry_for(plan, evidence_tree["receipt"])["sha256"] = "0" * 64
    with pytest.raises(publisher.ExportError):
        publisher.execute_plan(plan)
    assert not evidence_tree["output"].exists()


@pytest.mark.parametrize("destination", ["README.md", "MANIFEST.json", "B1/result_summary.json"])
def test_tampered_plan_cannot_replace_generated_files(publisher, evidence_tree, destination):
    plan = plan_for(publisher, evidence_tree)
    entry_for(plan, evidence_tree["receipt"])["destination"] = destination
    with pytest.raises(publisher.ExportError):
        publisher.execute_plan(plan)
    assert not evidence_tree["output"].exists()
