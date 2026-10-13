#!/usr/bin/env python3
"""HH_261013 - Publish a lossless, bounded selection of finalized CARLA evidence.

Default is a read-only plan. --execute exclusively creates a NEW directory;
original evidence, simulator state, and git are never modified.
"""

import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
from functools import wraps
from urllib.parse import unquote, urlsplit


SELECTIONS = (
    ("B1", "v5", "B1_attempt02"), ("B2", "v6", "B2"),
    ("B3", "v6", "B3"), ("B4", "v5", "B4"), ("B5", "v5", "B5"),
    ("B6", "v5", "B6_attempt02"), ("B7", "v5", "B7_attempt02"),
    ("B8", "v5", "B8_attempt02"), ("B9", "v7", "B9"),
    ("B10", "v9_restart1", "B10"), ("B11", "v5", "B11_attempt02"),
    ("B12", "v5", "B12_attempt02"), ("B13", "v5", "B13_attempt02"),
)
VISUAL_FILES = (
    "representative_contact_sheet.png", "representative_motion.gif",
    "capture_manifest.json", "sha256sums.txt", "ffprobe.json", "exact_commands.txt",
)
LINK = re.compile(r"(!?\[[^\]\n]*\])\(([^)\n]+)\)")
REFERENCE = re.compile(r"^(\s*\[[^\]\n]+\]:\s*)(\S+)\s*$", re.MULTILINE)
CHUNK = 1024 * 1024


class ExportError(RuntimeError):
    """HH_261013 - A missing, changed, unsafe, or non-final artifact fails closed."""


def _export_errors(function):
    @wraps(function)
    def checked(*args, **kwargs):
        try:
            return function(*args, **kwargs)
        except ExportError:
            raise
        except (OSError, KeyError, TypeError, ValueError, OverflowError) as error:
            raise ExportError(f"Evidence validation/export failed: {error}") from error
    return checked


def _inside(path, root):
    return path == root or root in path.parents


def _regular(path, allowed_root):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or not _inside(path.resolve(), allowed_root):
        raise ExportError(f"Missing or unsafe required source: {path}")
    return path.resolve()


def _digest(path):
    digest = hashlib.sha256()
    size = 0
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(CHUNK), b""):
            digest.update(block)
            size += len(block)
    return digest.hexdigest(), size


def _json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ExportError(f"Invalid evidence JSON: {path}: {error}") from error


def _destination(output, relative):
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts or not relative.parts:
        raise ExportError(f"Unsafe output path: {relative}")
    target = output / relative
    if not _inside(target.resolve(), output):
        raise ExportError(f"Output path escapes publication: {relative}")
    return target


def _link_target(document, raw):
    raw = raw.strip().strip("<>")
    parsed = urlsplit(raw)
    if parsed.scheme in ("http", "https", "mailto") or raw.startswith("#"):
        return None, raw
    # HH_261013 - Convert local editor :line suffixes to GitHub #L anchors.
    path_text, separator, fragment = raw.partition("#")
    line = re.search(r":(\d+)$", path_text)
    if line:
        path_text = path_text[:line.start()]
        fragment = "L" + line.group(1)
    target = Path(unquote(path_text))
    if not target.is_absolute():
        target = document.parent / target
    return target.resolve(), ("#" + fragment if fragment else "")


def _all_links(text):
    return [match.group(2) for match in LINK.finditer(text)] + [
        match.group(2) for match in REFERENCE.finditer(text)]


def _validate_case(folder, site):
    trial = _json(folder / "trial.json")
    result = trial["result"]
    final = result["final_observation"]
    supervisor = _json(folder / "supervisor.json")
    collision = final["collision_evidence"]
    pose = final["pose"]
    distance = math.hypot(pose["x_m"] + 8.47366, pose["y_m"] - 40.391)
    checks = (
        result["status"] == "COMPLETED_OBSERVED",
        result["roundtrip_verified"] is True,
        result["formal_acceptance"] is False,
        result["identity"]["site"] == site,
        supervisor["status"] == "finished",
        supervisor["trial_exit_code"] == supervisor["capture_exit_code"] == 0,
        supervisor["actor_id"] == result["actor_id"],
        final["parking"]["reverse"]["operating_state"] == "PARKED",
        final["parking"]["dispatcher"]["operating_state"] == "PARKED",
        final["gate"]["operating_state"] == "STANDBY",
        math.isfinite(distance) and distance <= 0.25,
        collision["publisher_count"] >= 1,
        collision["subscriber_and_publisher_discovered"] is True,
        all(collision[key] == 0 for key in (
            "event_count", "sampled_event_count", "omitted_event_count",
            "stale_queued_event_count")),
    )
    if not all(checks):
        raise ExportError(f"Selected case is not a finalized qualifying completion: {folder}")
    visual = folder / "visual"
    manifest = _json(visual / "capture_manifest.json")
    if not (manifest["status"] == "PASS"
            and manifest["scope"]["kind"] == "actual X11 desktop-region recording"
            and manifest["recording"]["capture_fps"] == 5
            and manifest["derivatives"]["gif_fps"] == 5
            and manifest["x11"]["same_titles_and_geometry_before_after_capture"] is True):
        raise ExportError(f"Incomplete actual-desktop visual evidence: {visual}")
    for key, name in (("contact_sheet_png", VISUAL_FILES[0]),
                      ("representative_gif", VISUAL_FILES[1])):
        item = manifest["artifacts"][key]
        actual_hash, actual_size = _digest(visual / name)
        if item["path"] != name or not actual_size or actual_hash != item["sha256"]:
            raise ExportError(f"Visual manifest hash mismatch: {visual / name}")
    listed = set()
    for line in (visual / "sha256sums.txt").read_text().splitlines():
        expected, name = line.split(None, 1)
        name = name.lstrip("*")
        if name not in VISUAL_FILES or name == "sha256sums.txt":
            raise ExportError(f"Unexpected visual hash target: {name}")
        if name in listed or _digest(visual / name)[0] != expected:
            raise ExportError(f"Visual checksum mismatch/duplicate: {visual / name}")
        listed.add(name)
    if listed != set(VISUAL_FILES) - {"sha256sums.txt"}:
        raise ExportError(f"Incomplete visual checksum list: {visual}")
    summary = {key: result[key] for key in (
        "status", "roundtrip_verified", "formal_acceptance", "actor_id", "identity",
        "elapsed_wall_s", "sampled_xy_m", "outbound", "return_leg", "sample_count")}
    summary.update(final_pose=pose, station_distance_m=distance,
                   collision_evidence=collision, source_case=folder.name)
    if any(not math.isfinite(summary[key]) or summary[key] <= 0 for key in
           ("elapsed_wall_s", "sampled_xy_m", "sample_count")):
        raise ExportError(f"Invalid selected-case physical metrics: {folder}")
    json.dumps(summary, allow_nan=False)
    return summary


@_export_errors
def build_plan(source_root, receipt_root, output_root=None):
    """HH_261013 - Resolve and hash the explicit selection without creating files."""
    root = Path(source_root).resolve()
    receipts = Path(receipt_root).resolve()
    evidence = root / "docs/evidence/virtual_carla"
    output = Path(output_root).absolute() if output_root else evidence / "releases/20261013"
    if output.exists() or output.is_symlink():
        raise ExportError(f"Publication already exists; refusing overwrite: {output}")
    if not _inside(output.resolve(), root) or output.resolve() == root:
        raise ExportError("Publication must be a new directory inside source_root")
    output = output.resolve()
    entries = {}
    source_links = {}
    destinations = set()

    def add(source, destination, mode="copy"):
        source = Path(source)
        allowed = receipts if _inside(source.resolve(), receipts) else root
        source = _regular(source, allowed)
        _destination(output, destination)
        if str(source) in entries:
            return
        if destination in destinations:
            raise ExportError(f"Publication destination collision: {destination}")
        sha, size = _digest(source)
        entries[str(source)] = dict(source=str(source), destination=destination,
                                    mode=mode, sha256=sha, size=size)
        destinations.add(destination)

    central = evidence / "site_completion_20261012_RESULTS.md"
    audit = evidence / "site_completion_20261012_v9_restart1/FINAL_13_SITE_AUDIT.md"
    add(central, "RESULTS.md", "markdown")
    add(audit, "FINAL_13_SITE_AUDIT.md", "markdown")
    cases = []
    for site, version, name in SELECTIONS:
        folder = evidence / ("site_completion_20261012_" + version) / name
        for name_in_case in ("ANALYSIS.md", "supervisor.json", "trial.json"):
            mode = "gzip" if name_in_case == "trial.json" else (
                "markdown" if name_in_case.endswith(".md") else "copy")
            add(folder / name_in_case, f"{site}/{name_in_case}" +
                (".gz" if mode == "gzip" else ""), mode)
        for name_in_visual in VISUAL_FILES:
            add(folder / "visual" / name_in_visual, f"{site}/visual/{name_in_visual}")
        summary = _validate_case(folder, site)
        summary.update(site=site, version=version,
                       original_trial_sha256=entries[str((folder / "trial.json").resolve())]["sha256"])
        cases.append(summary)

    # HH_261013 - Historical Markdown is small and preserves failures. Only
    # explicitly linked portable receipts/metadata/helpers are copied, never a
    # recursive raw-log/video/archive tree or a new live probe.
    scanned = set()
    while True:
        pending = [entry for key, entry in entries.items()
                   if entry["mode"] == "markdown" and key not in scanned]
        if not pending:
            break
        for entry in pending:
            document = Path(entry["source"])
            scanned.add(str(document))
            for raw in _all_links(document.read_text(encoding="utf-8")):
                target, _ = _link_target(document, raw)
                if target is None or str(target) in entries:
                    continue
                if _inside(target, receipts):
                    add(target, "receipts/" + target.relative_to(receipts).as_posix())
                elif _inside(target, evidence) and target.relative_to(evidence).parts[0].startswith(
                        "site_completion_20261012"):
                    if target.suffix == ".md":
                        add(target, "archive/" + target.relative_to(evidence).as_posix(), "markdown")
                    elif target.is_file() and (target.suffix in (".py", ".sh") or
                            target.name in ("live_profile.json", "live_profile_ready.json", "frozen_inputs.json")):
                        add(target, "archive/" + target.relative_to(evidence).as_posix())
                elif _inside(target, root) and not _inside(target, evidence) and target.is_file():
                    target = _regular(target, root)
                    sha, size = _digest(target)
                    source_links[str(target)] = dict(source=str(target), sha256=sha, size=size)
    return dict(schema="camrod.public_evidence.plan.v1", source_root=str(root),
                receipt_root=str(receipts), output_root=str(output),
                entries=sorted(entries.values(), key=lambda entry: entry["destination"]),
                repository_source_links=sorted(source_links.values(), key=lambda entry: entry["source"]),
                cases=cases)


def _rewrite_markdown(text, document, destination, plan, omitted):
    mapping = {entry["source"]: entry["destination"] for entry in plan["entries"]}
    root = Path(plan["source_root"])
    output = Path(plan["output_root"])
    evidence = root / "docs/evidence/virtual_carla"

    def rewrite(raw):
        target, suffix = _link_target(document, raw)
        if target is None:
            return suffix, None
        if str(target) in mapping:
            linked = output / mapping[str(target)]
        elif _inside(target, root) and not _inside(target, evidence) and target.is_file():
            # Source links remain repository-relative, not host-absolute.
            linked = target
        else:
            display = target.relative_to(root).as_posix() if _inside(target, root) else str(target)
            omitted.append(dict(document=destination, target=display))
            return None, display
        relative = os.path.relpath(linked, (output / destination).parent)
        return relative + suffix, None

    def inline(match):
        target, archive = rewrite(match.group(2))
        if archive is not None:
            label = match.group(1).lstrip("!").strip("[]")
            return f"{label} (LOCAL_ARCHIVE: `{archive}`)"
        return match.group(1) + "(" + target + ")"

    def reference(match):
        target, archive = rewrite(match.group(2))
        return (f"LOCAL_ARCHIVE: `{archive}`" if archive is not None
                else match.group(1) + target)

    return REFERENCE.sub(reference, LINK.sub(inline, text))


def _copy_verified(entry, target):
    source = Path(entry["source"])
    digest = hashlib.sha256()
    size = 0
    with source.open("rb") as incoming, target.open("xb") as raw_output:
        writer = gzip.GzipFile(filename="", mode="wb", fileobj=raw_output,
                               compresslevel=6, mtime=0) if entry["mode"] == "gzip" else raw_output
        try:
            for block in iter(lambda: incoming.read(CHUNK), b""):
                size += len(block)
                if size > entry["size"]:
                    raise ExportError(f"Source grew after planning: {source}")
                digest.update(block)
                writer.write(block)
        finally:
            if writer is not raw_output:
                writer.close()
    if size != entry["size"] or digest.hexdigest() != entry["sha256"]:
        raise ExportError(f"Source changed after planning: {source}")
    if entry["mode"] == "gzip":
        digest = hashlib.sha256()
        count = 0
        with gzip.open(target, "rb") as stream:
            for block in iter(lambda: stream.read(CHUNK), b""):
                count += len(block)
                if count > entry["size"]:
                    raise ExportError(f"Compressed output exceeds original size: {target}")
                digest.update(block)
        if count != size or digest.hexdigest() != entry["sha256"]:
            raise ExportError(f"Lossless gzip verification failed: {target}")
    elif _digest(target) != (entry["sha256"], entry["size"]):
        raise ExportError(f"Copy verification failed: {target}")


def _readme(cases):
    lines = ["# CARLA B1–B13 selected completion evidence", "",
             "<!-- HH_261013 - Curated publication; original failures and profile boundaries remain explicit. -->", "",
             "Selected historical completions: V5 ×9, V6 ×2, V7 ×1, V9 restart1 ×1. "
             "This is NOT a same-final-profile 13-site rerun or formal acceptance. "
             "No real-robot/10 km, recall, charging docking, repeatability, or full mission-archive claim.", "",
             "[Detailed report](RESULTS.md) · [Independent audit](FINAL_13_SITE_AUDIT.md) · [Manifest](MANIFEST.json)", "",
             "| Site | Profile | Total seconds | Physical sampled XY metres | Station error m | Evidence |",
             "| --- | --- | ---: | ---: | ---: | --- |"]
    for case in cases:
        site = case["site"]
        lines.append(f"| {site} | {case['version']} | {case['elapsed_wall_s']:.6f} | "
                     f"{case['sampled_xy_m']:.6f} | {case['station_distance_m']:.9f} | "
                     f"[analysis]({site}/ANALYSIS.md) · [summary]({site}/result_summary.json) · "
                     f"[raw gzip]({site}/trial.json.gz) · [PNG]({site}/visual/representative_contact_sheet.png) · "
                     f"[GIF]({site}/visual/representative_motion.gif) |")
    lines += ["", f"Selected-trial sums: **{sum(c['elapsed_wall_s'] for c in cases):.6f} s / "
              f"{sum(c['sampled_xy_m'] for c in cases):.6f} m**; not continuous endurance mileage.", "",
              "Full trial bytes are preserved in deterministic gzip (mtime=0, no embedded filename). "
              "Original files were not edited. Original and published SHA-256 values are in MANIFEST.json. "
              "For example, `gzip -cd B10/trial.json.gz | sha256sum` must equal that case's "
              "`original_trial_sha256`. Result summaries are explicitly derived extracts, not substitute raw trials.", "",
              "LOCAL_ARCHIVE labels identify omitted historical files or host-only references. "
              "They are deliberately not clickable. Historical Markdown is retained under archive/; "
              "receipts preserve their original bytes and historical host paths as provenance, not portable runtime authorization. "
              "Repository source links are relative. No raw MP4, full journal, temporary cache, or unrelated data tree is included.", ""]
    return "\n".join(lines)


@_export_errors
def execute_plan(plan):
    """HH_261013 - Exclusive publication; on failure retain incomplete output without MANIFEST."""
    output = Path(plan["output_root"])
    root = Path(plan["source_root"]).resolve()
    receipts = Path(plan["receipt_root"]).resolve()
    if output.exists() or output.is_symlink() or not _inside(output.resolve(), root) or output.resolve() == root:
        raise ExportError(f"Unsafe or existing publication: {output}")
    seen = set()
    reserved = {"README.md", "MANIFEST.json"} | {
        case["site"] + "/result_summary.json" for case in plan["cases"]}
    if [(case["site"], case["version"], case["source_case"]) for case in plan["cases"]] != list(SELECTIONS):
        raise ExportError("Plan does not contain the exact selected thirteen cases")
    json.dumps(plan["cases"], allow_nan=False)
    # Validate the complete plan before making even the publication directory.
    for entry in plan["entries"]:
        target = _destination(output, entry["destination"])
        if (target in seen or entry["destination"] in reserved
                or entry["mode"] not in ("copy", "gzip", "markdown")):
            raise ExportError(f"Invalid publication entry: {entry['destination']}")
        seen.add(target)
        source = Path(entry["source"])
        _regular(source, receipts if _inside(source.resolve(), receipts) else root)
        if _digest(source) != (entry["sha256"], entry["size"]):
            raise ExportError(f"Source changed after planning: {source}")
    for entry in plan.get("repository_source_links", []):
        source = _regular(entry["source"], root)
        if _digest(source) != (entry["sha256"], entry["size"]):
            raise ExportError(f"Linked source changed after planning: {source}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir(exist_ok=False)
    omitted = []
    for entry in plan["entries"]:
        target = _destination(output, entry["destination"])
        target.parent.mkdir(parents=True, exist_ok=True)
        if entry["mode"] == "markdown":
            source = Path(entry["source"])
            raw = source.read_bytes()
            if len(raw) != entry["size"] or hashlib.sha256(raw).hexdigest() != entry["sha256"]:
                raise ExportError(f"Markdown changed after planning: {source}")
            text = _rewrite_markdown(raw.decode("utf-8"), source,
                                     entry["destination"], plan, omitted)
            with target.open("x", encoding="utf-8") as stream:
                stream.write(text)
        else:
            _copy_verified(entry, target)
    for case in plan["cases"]:
        target = _destination(output, case["site"] + "/result_summary.json")
        with target.open("x", encoding="utf-8") as stream:
            json.dump(dict(schema="camrod.public_evidence.derived_summary.v1", **case),
                      stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
    with (output / "README.md").open("x", encoding="utf-8") as stream:
        stream.write(_readme(plan["cases"]))
    # Every published local Markdown target must exist. MANIFEST is written
    # last only after this check and a second source-integrity pass succeed.
    for document in output.rglob("*.md"):
        for raw in _all_links(document.read_text(encoding="utf-8")):
            target, _ = _link_target(document, raw)
            if target is not None and target != output / "MANIFEST.json" and not target.is_file():
                raise ExportError(f"Dangling published Markdown link: {document}: {raw}")
    for entry in plan["entries"] + plan.get("repository_source_links", []):
        if _digest(entry["source"]) != (entry["sha256"], entry["size"]):
            raise ExportError(f"Source changed during export: {entry['source']}")
    artifacts = []
    for path in sorted(output.rglob("*")):
        if path.is_file():
            sha, size = _digest(path)
            artifacts.append(dict(path=path.relative_to(output).as_posix(), sha256=sha, bytes=size))
    manifest = dict(schema="camrod.public_evidence.manifest.v1", publication_complete=True,
                    scope="selected historical delivery completions; mixed profiles; not formal acceptance",
                    source_root=str(root), cases=plan["cases"], sources=plan["entries"],
                    repository_source_links=plan.get("repository_source_links", []),
                    artifacts=artifacts, local_archive_references=omitted)
    encoded_manifest = json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    with (output / "MANIFEST.json").open("x", encoding="utf-8") as stream:
        stream.write(encoded_manifest)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--receipt-root", type=Path,
                        default=Path("/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence"))
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    try:
        plan = build_plan(args.source_root, args.receipt_root, args.output_root)
        if args.execute:
            manifest = execute_plan(plan)
            print(json.dumps(dict(mode="EXPORTED", output_root=plan["output_root"],
                                  cases=len(manifest["cases"]), files=len(manifest["artifacts"])) ))
        else:
            print(json.dumps(dict(mode="PLAN_ONLY", output_root=plan["output_root"],
                                  cases=len(plan["cases"]), sources=len(plan["entries"]),
                                  source_bytes=sum(entry["size"] for entry in plan["entries"]))))
        return 0
    except (ExportError, OSError, KeyError, TypeError, ValueError) as error:
        print(f"Publication refused: {error}; no successful MANIFEST is issued", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
