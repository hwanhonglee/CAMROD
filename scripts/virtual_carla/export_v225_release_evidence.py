#!/usr/bin/env python3
"""Export an explicit, byte-identical v2.2.5 evidence subset, never run a robot.

Original JSON paths are preserved. README/manifest provide portable archive
paths. No directory discovery, source edits/deletion, replacement exports,
image transformations, ROS, browser, git writes, or automatic mission actions.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath
import struct
import sys
import tempfile
import unittest


RUNTIME = "067568ecfe411a5cc31844fa84696220da879249"
PURE = "91158175102a2bbf86624f5c5b726e5e272f8b90"
DOCK_HELPER = "a74f898b8a216c4f36675aa5778afd3d8e6a853f168cdd2bd062e6c47238f368"
DEFAULT_SOURCE = Path("/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/evidence/v224_validation")
DEFAULT_PURE_EVIDENCE = Path("/home/hong/camrod_ws/pure_camrod_v224_verification/docs/evidence/v2_2_5_20260908")
MAX_FILE_BYTES = 99_000_000  # Strictly below GitHub's 100 MB per-file boundary.
MAX_BUNDLE_BYTES = 80 * 1024 * 1024
GROUPS = {
    "b1_guest_recall": "B1 Guest → Robot 최종 확인 → 비충전 후진주차: suite12/067 실제 strict PASS 1건",
    "optional_docking": "선택 Dock: suite13/067 실제 FAIL, STOP 관측; 충전 성공 아님",
    "ui": "Guest 재연결·설정 미가용·진단 실제 화면: 이전0854/940 bundle 시점, 현재 전체 인증 아님",
    "gnss": "GNSS 중심 수학/기존 native 회귀: 실차 전방 장착·회전 후 XY 인증 아님",
    "voice": "과거 음성 파형 감사 원본만 보존: 최신 전체 음성 검증 미완료",
    "tools": "검증 당시 TEST-only helper 사본: runtime/PURE 코드 또는 자급 실행 패키지 아님",
}


def mapping():
    """Only these exact sources may be copied. No glob or JSON path traversal."""
    rows = []

    def add(group, source, target, *, optional=False):
        rows.append({"group": group, "source_relative": source,
                     "relative_path": f"{group}/{target}", "optional": optional})

    b1 = "full_suite_v224_12/B1_guest_recall_robot_handoff"
    for source, target in (
        ("full_suite_v224_12/suite_status.json", "suite_status.json"),
        ("full_suite_v224_12/suite_plan.json", "suite_plan.json"),
        (f"{b1}/run_manifest.json", "run_manifest.json"),
        (f"{b1}/B1/site_manifest.json", "site_manifest.json"),
        (f"{b1}/B1/camping_site_matrix.json", "native.json"),
        (f"{b1}/summary/camping_site_metrics.json", "metrics.json"),
        (f"{b1}/summary/camping_site_metrics.csv", "metrics.csv"),
        (f"{b1}.strict_validation/site_evidence_collection.json", "strict.json"),
        (f"{b1}.strict_validation/site_evidence_collection.csv", "strict.csv"),
        (f"{b1}/B1/visual/capture_manifest.json", "visual/capture_manifest.json"),
        (f"{b1}/B1/visual/representative_contact_sheet.png", "visual/representative_contact_sheet.png"),
        (f"{b1}/B1/visual/representative_motion.gif", "visual/representative_motion.gif"),
        (f"{b1}/B1/terminal_robot_ui/terminal_capture.json", "ui/terminal_robot.json"),
        (f"{b1}/B1/terminal_robot_ui/terminal_robot_ui.png", "ui/terminal_robot.png"),
        (f"{b1}/B1/wheel_summary/wheel_summary.json", "wheel_summary.json"),
        (f"{b1}/B1/wheel_summary/wheel_measurements.csv", "wheel_measurements.csv"),
    ):
        add("b1_guest_recall", source, target)
    confirmations = "../camrod_camping_site_matrix_guest_usage_complete/20260908T083902Z/ui_confirmations"
    add("b1_guest_recall", f"{confirmations}/B1_g1788856685421001_first_guest.png", "ui/first_guest.png")
    add("b1_guest_recall", f"{confirmations}/B1_g1788856685421001_final_robot.png", "ui/final_robot.png")

    dock = "full_suite_v224_13/B1_optional_docking"
    for source, target in (
        ("full_suite_v224_13/suite_status.json", "suite_status.json"),
        ("full_suite_v224_13/suite_plan.json", "suite_plan.json"),
        (f"{dock}/functional/result.json", "result.json"),
        (f"{dock}/desktop/capture_manifest.json", "desktop/capture_manifest.json"),
        (f"{dock}/desktop/representative_contact_sheet.png", "desktop/representative_contact_sheet.png"),
        (f"{dock}/desktop/representative_motion.gif", "desktop/representative_motion.gif"),
        (f"{dock}/functional/01_reverse_parked_before_dock.png", "functional/01_reverse_parked_before_dock.png"),
        (f"{dock}/functional/02_rear_camera_before_dock.png", "functional/02_rear_camera_before_dock.png"),
        (f"{dock}/functional/03_actual_rear_tag_detected.png", "functional/03_actual_rear_tag_detected.png"),
        (f"{dock}/functional/04_ui_docking_in_progress.png", "functional/04_ui_docking_in_progress.png"),
        ("optional_docking_investigation/actual_rear_20260908T090835347230Z.png", "investigation/actual_rear.png"),
        ("optional_docking_investigation/actual_rear_20260908T090835347230Z.json", "investigation/actual_rear.json"),
        ("logs/optional_docking_safety.xml", "tests/safety_25.junit.xml"),
        ("logs/optional_docking_late_join.xml", "tests/late_join_74.junit.xml"),
    ):
        add("optional_docking", source, target)
    for folder, target in (("guest_reconnect_before_fix", "guest_before"),
                           ("guest_reconnect_after_restart_fixed", "guest_reconnected")):
        add("ui", f"{folder}/guest_actual.png", f"{target}/actual.png")
        add("ui", f"{folder}/observation.json", f"{target}/observation.json")
    add("ui", "diagnostic_tuning_after_fix/tuning_service_unavailable.png", "tuning/actual.png")
    add("ui", "diagnostic_tuning_after_fix/tuning_ui.json", "tuning/observation.json")
    add("ui", "diagnostic_tabs_after_ui_fixes/diagnostic_tabs.json", "diagnostics/observations.json")
    for name in ("01_camera", "02_gnss", "03_proximity", "04_trajectory", "05_perception",
                 "06_safety", "07_docking", "08_system"):
        add("ui", f"diagnostic_tabs_after_ui_fixes/{name}.png", f"diagnostics/{name}.png")
    add("gnss", "GNSS_CENTER_VALIDATION_SCOPE.md", "original_scope.md")
    add("gnss", "logs/gnss_center_scope_native.xml", "math_native.junit.xml")
    for source, target in (("native_gnss_center_scope.junit.xml", "release_math_native.junit.xml"),
                           ("native_verification_summary.json", "release_native_verification_summary.json")):
        add("gnss", source, target)
        rows[-1]["source_scope"] = "pure_release"
    add("voice", "voice_signal_audit_20260908T0600Z.json", "historical_signal_audit.json", optional=True)
    for name in ("run_optional_docking_ui.py", "run_durable_suite.py", "run_durable_suite.sh",
                 "watch_results_suite06.py", "update_results_index.py", "test_durable_suite_continuation.py",
                 "test_watch_results_suite06.py", "tests/test_optional_docking_safety.py",
                 "optional_docking_investigation/capture_rear_readonly.py"):
        add("tools", name, name)
    return rows


def safe_relative(value):
    path = PurePosixPath(value)
    if not value or path.is_absolute() or ".." in path.parts or "\\" in value or str(path) != value:
        raise ValueError(f"Unsafe archive path: {value!r}")
    return Path(*path.parts)


def digest(path):
    total = 0
    sha = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            total += len(block)
            sha.update(block)
    return {"bytes": total, "sha256": sha.hexdigest()}


def image_header(path):
    if path.suffix.lower() not in {".png", ".gif"}:
        return None
    with path.open("rb") as source:
        head = source.read(32)
    if path.suffix.lower() == ".png" and head[:8] == b"\x89PNG\r\n\x1a\n" and head[12:16] == b"IHDR":
        width, height = struct.unpack(">II", head[16:24])
    elif path.suffix.lower() == ".gif" and head[:6] in {b"GIF87a", b"GIF89a"}:
        width, height = struct.unpack("<HH", head[6:10])
    else:
        raise ValueError(f"Invalid image header: {path}")
    if width < 1 or height < 1:
        raise ValueError(f"Empty image dimensions: {path}")
    return {"width": width, "height": height, "verification": "format_header_only_not_new_visual_or_mission_acceptance"}


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def known_run_transition(expected, current):
    return (expected.get("sha256") == "72c7b9d376fe4d7879623ad31f0bceeb04f706f0b542ebd2b21cf389d7132714"
            and expected.get("bytes") == 11390
            and current.get("sha256") == "a7b32f7902601901f608ebafe984397255251740d04eabc830d3f26f4a369b5f"
            and current.get("bytes") == 11384)


def original_bindings(source_root, rows):
    """Validate selected existing acceptance and hash references, not new PASS."""
    by_name = {row["relative_path"]: row for row in rows}
    checks, notes = [], []

    def original(name):
        return load(Path(by_name[name]["source_absolute"]))

    def require(condition, label):
        if not condition:
            raise ValueError(f"Original evidence rejected: {label}")
        checks.append(label)

    def bind(record, name, label):
        row = by_name[name]
        require(isinstance(record, dict) and record.get("sha256") == row["sha256"]
                and ("bytes" not in record or record["bytes"] == row["bytes"]), label)

    run = original("b1_guest_recall/run_manifest.json")
    site = original("b1_guest_recall/site_manifest.json")
    native = original("b1_guest_recall/native.json")
    strict = original("b1_guest_recall/strict.json")
    episode = native["sites"][0]
    accepted = strict["sites"][0]
    require(run["status"] == site["status"] == native["status"] == strict["status"] == "PASS", "B1 original run/site/native/strict PASS")
    require(run["source"]["head"] == accepted["runtime_source_head"] == RUNTIME, "B1 original driver/runtime 067 identity")
    require(episode["mission_identity"] == {"site": "B1", "owner": "guest", "intent": "recall", "generation": 1788856685421001}, "B1 exact mission identity")
    require(accepted["actor_id"] == 98 and accepted["parking_completion"] == "reverse" and accepted["charging_confirmed"] is False, "B1 actor98 reverse noncharging")
    require(strict["aggregate"]["pass_count"] == 1 and strict["aggregate"]["fail_count"] == 0, "B1 one strict acceptance only")
    suite12 = original("b1_guest_recall/suite_status.json")
    require(any(item["stage"] == "B1_guest_recall_robot_handoff" and item["status"] == "PASS" for item in suite12["completed"]), "suite12 original STAGE_PASS")
    run_ref = strict["source"]["run_manifest"]
    current_run = by_name["b1_guest_recall/run_manifest.json"]
    if run_ref["sha256"] != current_run["sha256"]:
        # The checked-in runner hashes VALIDATING, then writes PASS and a new
        # updated_at_utc after strict success. Preserve both originals: do not
        # rewrite strict's hash, reconstruct a fictional historical file, or
        # silently permit arbitrary reference mismatches.
        require(known_run_transition(run_ref, current_run), "exact known VALIDATING-to-PASS run manifest transition, not generic hash bypass")
        notes.append({"kind": "KNOWN_RUN_MANIFEST_LIFECYCLE_REFERENCE_DIFFERENCE",
                      "expected_in_original_strict": run_ref,
                      "archived_final_run": {key: current_run[key] for key in ("relative_path", "bytes", "sha256")},
                      "strict_validated_utc": strict.get("validated_at_utc"),
                      "final_run_updated_utc": run.get("updated_at_utc"),
                      "source_order": "scripts/virtual_carla/run_site_evidence_matrix.sh:1535 VALIDATING; :1549 PASS after strict --allow-validating-manifest",
                      "explanation": "run_site_evidence_matrix.sh hashes VALIDATING before writing final PASS/updated_at; original historical hash is not a hash of this archived final file. Source/strict originals unchanged; full original snapshot not available."})
    else:
        bind(run_ref, "b1_guest_recall/run_manifest.json", "strict binds run manifest")
    bind({"sha256": strict["source"]["metrics_summary"]["json_sha256"]}, "b1_guest_recall/metrics.json", "strict binds metrics JSON")
    bind({"sha256": accepted["site_manifest_sha256"]}, "b1_guest_recall/site_manifest.json", "strict binds site manifest")
    bind(site["matrix_report"], "b1_guest_recall/native.json", "site binds native report")
    for key, filename in (("png", "representative_contact_sheet.png"), ("gif", "representative_motion.gif")):
        target = f"b1_guest_recall/visual/{filename}"
        bind(site["visual"][key], target, f"site binds B1 {key}")
        bind({"sha256": accepted[f"{key}_sha256"]}, target, f"strict binds B1 {key}")
    views = {(item["stage"], item["frontend"]): item for item in episode["confirmation_views"]}
    bind(views[("first", "guest")]["png"], "b1_guest_recall/ui/first_guest.png", "native binds first Guest actual PNG")
    bind(views[("final", "robot")]["png"], "b1_guest_recall/ui/final_robot.png", "native binds final Robot actual PNG")
    bind(original("b1_guest_recall/ui/terminal_robot.json")["png"], "b1_guest_recall/ui/terminal_robot.png", "terminal actual PNG hash")
    dock = original("optional_docking/result.json")
    require(dock["status"] == "FAIL" and dock["safety_stop"]["requested"] is True
            and dock["safety_stop"]["stop_observed"] is True, "Dock original FAIL and observed failure STOP")
    require(dock["test_source_provenance"]["dock_helper"]["sha256"] == DOCK_HELPER, "Dock helper version a74 distinct from runtime")
    bind(dock["test_source_provenance"]["dock_helper"], "tools/run_optional_docking_ui.py", "copied Dock helper matches executed helper")
    capture = original("optional_docking/desktop/capture_manifest.json")
    for key, filename in (("contact_sheet_png", "representative_contact_sheet.png"), ("representative_gif", "representative_motion.gif")):
        bind(capture["artifacts"][key], f"optional_docking/desktop/{filename}", f"Dock capture binds {key}")
    bind(original("optional_docking/investigation/actual_rear.json")["png"], "optional_docking/investigation/actual_rear.png", "actual later rear image hash")
    for group in ("guest_before", "guest_reconnected", "tuning"):
        bind(original(f"ui/{group}/observation.json")["png"], f"ui/{group}/actual.png", f"actual {group} image hash")
    tabs = original("ui/diagnostics/observations.json")["tabs"]
    require(len(tabs) == 8, "eight original diagnostic tabs, not eight sensor certifications")
    for item in tabs:
        filename = Path(item["png"]["path"]).name
        bind(item["png"], f"ui/diagnostics/{filename}", f"diagnostic actual image {filename}")
    # Failure never grants permission to fabricate missing completion images.
    expected_absent = ["full_suite_v224_13/B1_optional_docking/functional/05_ui_charging_complete.png",
                       "full_suite_v224_13/B1_optional_docking/functional/06_rear_camera_charging_complete.png"]
    for name in expected_absent:
        require(not (source_root / name).exists(), f"Dock completion image correctly absent: {name}")
    return checks, expected_absent, notes


def plan(source_root, pure_evidence_root=DEFAULT_PURE_EVIDENCE):
    source_root = source_root.resolve(strict=True)
    permitted_root = source_root.parent
    rows, missing = [], []
    names = set()
    for entry in mapping():
        target = safe_relative(entry["relative_path"])
        if str(target) in names:
            raise ValueError(f"Duplicate archive destination: {target}")
        names.add(str(target))
        selected_root = pure_evidence_root.resolve() if entry.get("source_scope") == "pure_release" else source_root
        allowed_root = selected_root if entry.get("source_scope") == "pure_release" else permitted_root
        source = selected_root / entry["source_relative"]
        resolved = source.resolve()
        if not resolved.is_relative_to(allowed_root):
            raise ValueError(f"Source escaped explicit evidence root: {source}")
        if not source.is_file():
            if entry["optional"]:
                missing.append({**entry, "source_absolute": str(source.absolute()), "status": "MISSING_OPTIONAL_REPORTED_NOT_SILENT"})
                continue
            raise FileNotFoundError(f"Required original not found; no export: {source}")
        info = digest(source)
        if not 0 < info["bytes"] < MAX_FILE_BYTES:
            raise ValueError(f"Source size outside permitted limit: {source}: {info['bytes']}")
        rows.append({**entry, "source_absolute": str(source.absolute()), "source_resolved": str(resolved),
                     **info, "image": image_header(source)})
    total = sum(row["bytes"] for row in rows)
    if total > MAX_BUNDLE_BYTES:
        raise ValueError(f"Bundle {total} bytes exceeds explicit {MAX_BUNDLE_BYTES} byte budget; no automatic omission")
    checks, absent, notes = original_bindings(source_root, rows)
    return {"schema": "camrod.virtual.evidence.archive.v1", "status": "VERIFIED_SOURCE_PLAN",
            "created_utc": datetime.now(timezone.utc).isoformat(), "source_root": str(source_root),
            "runtime_source_head": RUNTIME, "pure_at_runtime_head": PURE, "dock_helper_sha256": DOCK_HELPER,
            "unique_current_drive_acceptances": 1, "planned_drive_count": 39,
            "dock_status": "FAIL", "all_scenarios_passed": False, "current_audio_complete": False,
            "file_count": len(rows), "total_bytes": total, "files": rows,
            "original_reference_checks": checks, "missing_optional": missing,
            "provenance_notes": notes,
            "known_missing_due_to_dock_failure": absent,
            "excluded": ["raw MP4", "large physical wheel JSONL", "audio recordings", "tmp",
                         "older failed duplicate suites", "historical B1 delivery/Robot recall as current approval"],
            "scope": "Selected metadata hashes rechecked; not a replay of full raw-wheel/mission strict validation"}


def render_index(document):
    lines = ["# v2.2.5 가상환경 증거 묶음", "",
             "**전체 검증 미완료: 현재 runtime의 주행 승인 1/39, 선택 Dock FAIL.**", "",
             f"원본 runtime `{RUNTIME}`, 해당 PURE `{PURE}`. 릴리스/문서 커밋과 실제 실행 버전은 다릅니다.",
             "B1 Guest→Robot 최종 확인→복귀·비충전 후진주차는 suite12의 동일 1건입니다. suite13 인계를 두 번째 성공으로 세지 않습니다.",
             "B1 native 시간 868.907초 / 173.935410m; suite 준비·캡처 wall time과 구분합니다.", "",
             "원본 JSON·문서·도구는 **바이트 그대로** 복사했습니다. 원본 내부 절대경로는 당시 출처이며 새 호스트에서 유효하지 않을 수 있습니다. 아래 상대 링크와 manifest의 relative_path를 사용하세요.",
             "이미지는 기존 실제 자료입니다. exporter는 원본 hash·형식 헤더를 재확인하며 새 이미지나 성공 장면을 만들지 않습니다.",
             "전체 raw wheel/MP4/음성 녹음은 제외한 최소 배포본이므로 원본 strict 보고서 전 항목을 독립 재실행할 수 있는 전체 데이터셋은 아닙니다.", "",
             "**알려진 메타데이터 수명 차이:** strict가 기록한 run SHA는 검증 중 VALIDATING snapshot입니다. 이후 최종 PASS/updated_at 저장으로 run SHA가 바뀌었습니다. 두 원본을 그대로 보존하고 manifest.provenance_notes에 기대/현재 SHA·크기·시각을 적었습니다. site/native/metrics/media의 기존 해시는 별도로 일치 확인했습니다.", "",
             "## 핵심 확인", "",
             "- [B1 실제 GIF](b1_guest_recall/visual/representative_motion.gif) · [PNG](b1_guest_recall/visual/representative_contact_sheet.png) · [strict 원본](b1_guest_recall/strict.json)",
             "- [첫 Guest 확인](b1_guest_recall/ui/first_guest.png) · [최종 Robot 확인](b1_guest_recall/ui/final_robot.png) · [비충전 복귀 화면](b1_guest_recall/ui/terminal_robot.png)",
             "- [Dock 실패 결과](optional_docking/result.json) · [실제 GIF](optional_docking/desktop/representative_motion.gif) · [후속 실제 후방 이미지](optional_docking/investigation/actual_rear.png)",
             "- GNSS는 수식/기존 native 회귀이며 전방 실차 장착·회전 후 중심 XY 검증 완료가 아닙니다.",
             "- gnss/release_*는 PURE911의 새 릴리스 검사이며, original_scope.md/math_native.junit.xml은 과거5e 감사 범위입니다. 서로 다른 검사 실행을 중복 합산하지 않습니다.",
             "- 진단 화면은 일부 실제 렌더/수신 증거입니다. 카메라 4.75Hz는 목표10Hz 인증이 아닙니다. 설정 미가용 표시 확인은 하드웨어 설정 POST 성공이 아닙니다.",
             "- 음성 JSON은 이전 실행의 파형 감사입니다. 최신 전체 음성/도킹 성공 음성은 미검증입니다.",
             "- tools는 당시 TEST-only 도구/회귀의 보존 사본입니다. 원래 host 경로·외부 ROS/프로젝트 의존성이 있으며, 배포본 단독 실행 안내가 아닙니다. 특히 run/--run은 실제 동작 권한이 필요합니다.", "",
             "## 전체 상대 경로 목록", ""]
    for group, description in GROUPS.items():
        lines += [f"### {description}", "", "| 자료 | 크기(bytes) |", "| --- | ---: |"]
        for row in document["files"]:
            if row["group"] == group:
                path = row["relative_path"]
                lines.append(f"| [{path}]({path}) | {row['bytes']} |")
        lines.append("")
    lines += ["## 누락·검증 한계", "",
              "Dock가 실패했으므로 05_ui_charging_complete.png / 06_rear_camera_charging_complete.png는 원본에 없고 생성하지 않았습니다.",
              "dummy 상태 미수신은 None 그대로 기록하며 false로 위조하지 않습니다. tag/image 동일프레임·header freshness 전체 결합, 물리 충전기와 실차 전체 기능은 이 자료로 인증하지 않습니다.",
              f"선택 파일 {document['file_count']}개, 원본 복사 {document['total_bytes']:,} bytes. [manifest.json](manifest.json)의 출처·SHA256·명시적 누락 목록과 [SHA256SUMS](SHA256SUMS)를 확인하세요.", "",
              "검증: 저장소 루트에서 `python3 scripts/virtual_carla/export_v225_release_evidence.py verify --output docs/evidence/virtual_carla/v2_2_5_20260908`", ""]
    if document["missing_optional"]:
        lines += ["보고된 선택 원본 부재:", ""] + [f"- `{row['source_absolute']}`" for row in document["missing_optional"]] + [""]
    return "\n".join(lines)


def export(source_root, output, pure_evidence_root=DEFAULT_PURE_EVIDENCE):
    if output.exists() or output.is_symlink():
        raise FileExistsError(f"New output only; refusing to replace: {output}")
    document = plan(source_root, pure_evidence_root)  # All required paths/hash bindings checked before output creation.
    output.mkdir(parents=True, exist_ok=False)
    for row in document["files"]:
        target = output / safe_relative(row["relative_path"])
        target.parent.mkdir(parents=True, exist_ok=True)
        sha = hashlib.sha256()
        count = 0
        with Path(row["source_absolute"]).open("rb") as source, target.open("xb") as destination:
            for block in iter(lambda: source.read(1024 * 1024), b""):
                destination.write(block)
                sha.update(block)
                count += len(block)
        if sha.hexdigest() != row["sha256"] or count != row["bytes"]:
            raise RuntimeError(f"Source changed during export; retained incomplete output, no acceptance: {target}")
    document["status"] = "EXPORTED_VERBATIM"
    document["exporter_sha256"] = digest(Path(__file__))['sha256']
    with (output / "manifest.json").open("x", encoding="utf-8") as file:
        json.dump(document, file, ensure_ascii=False, indent=2)
        file.write("\n")
    with (output / "README.md").open("x", encoding="utf-8") as file:
        file.write(render_index(document))
    checksums = [{"relative_path": row["relative_path"], "sha256": row["sha256"]} for row in document["files"]]
    for name in ("manifest.json", "README.md"):
        checksums.append({"relative_path": name, **digest(output / name)})
    with (output / "SHA256SUMS").open("x", encoding="utf-8") as file:
        file.writelines(f"{row['sha256']}  {row['relative_path']}\n" for row in checksums)
    verify(output)
    return document


def verify(output):
    root = output.resolve(strict=True)
    document = load(root / "manifest.json")
    if document["status"] != "EXPORTED_VERBATIM":
        raise ValueError("Archive is incomplete")
    if document["file_count"] != len(document["files"]):
        raise ValueError("Archive file count mismatch")
    total = 0
    for row in document["files"]:
        path = root / safe_relative(row["relative_path"])
        if not path.resolve().is_relative_to(root) or path.is_symlink():
            raise ValueError(f"Archive must contain local copies, not escaping symlinks: {path}")
        actual = digest(path)
        if any(actual[key] != row[key] for key in ("bytes", "sha256")):
            raise ValueError(f"Archive checksum mismatch: {path}")
        image_header(path)
        total += actual["bytes"]
    if total != document["total_bytes"]:
        raise ValueError("Archive byte count mismatch")
    for line in (root / "SHA256SUMS").read_text().splitlines():
        checksum, relative = line.split("  ", 1)
        if digest(root / safe_relative(relative))["sha256"] != checksum:
            raise ValueError(f"Archive index checksum mismatch: {relative}")
    return document


def self_test():
    class ExportTests(unittest.TestCase):
        def test_explicit_unique_mapping(self):
            rows = mapping()
            names = [str(safe_relative(row["relative_path"])) for row in rows]
            self.assertEqual(len(names), len(set(names)))
            self.assertTrue(all(not name.endswith((".mp4", ".ogg", "physical_wheels.jsonl")) for name in names))

        def test_unsafe_archive_paths_rejected(self):
            for value in ("../escape", "/absolute", "a/../escape", "a\\b", "", "a//b"):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    safe_relative(value)

        def test_existing_export_never_replaced(self):
            with tempfile.TemporaryDirectory(prefix="camrod-export-test-") as directory:
                path = Path(directory)
                with self.assertRaises(FileExistsError):
                    export(path / "missing_source", path)

        def test_required_missing_aborts_before_output(self):
            with tempfile.TemporaryDirectory(prefix="camrod-export-test-") as directory:
                path = Path(directory)
                source = path / "source"
                source.mkdir()
                with self.assertRaises(FileNotFoundError):
                    export(source, path / "new_archive")
                self.assertFalse((path / "new_archive").exists())

        def test_digest_and_bad_image(self):
            with tempfile.TemporaryDirectory(prefix="camrod-export-test-") as directory:
                path = Path(directory) / "bad.png"
                path.write_bytes(b"not an image")
                self.assertEqual(digest(path)["bytes"], 12)
                with self.assertRaises(ValueError):
                    image_header(path)

        def test_known_run_transition_is_not_generic_mismatch_permission(self):
            expected = {"bytes": 11390, "sha256": "72c7b9d376fe4d7879623ad31f0bceeb04f706f0b542ebd2b21cf389d7132714"}
            current = {"bytes": 11384, "sha256": "a7b32f7902601901f608ebafe984397255251740d04eabc830d3f26f4a369b5f"}
            self.assertTrue(known_run_transition(expected, current))
            self.assertFalse(known_run_transition(expected, {**current, "sha256": "0" * 64}))
            self.assertFalse(known_run_transition({**expected, "bytes": 11391}, current))

    result = unittest.TextTestRunner(verbosity=1).run(unittest.defaultTestLoader.loadTestsFromTestCase(ExportTests))
    if not result.wasSuccessful():
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("plan", "export", "verify", "self-test"))
    parser.add_argument("--source-root", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--pure-evidence-root", type=Path, default=DEFAULT_PURE_EVIDENCE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.command == "self-test":
        self_test()
        return
    if args.command in {"export", "verify"} and args.output is None:
        parser.error("--output is required for export/verify")
    if args.command == "plan":
        document = plan(args.source_root, args.pure_evidence_root)
    elif args.command == "export":
        document = export(args.source_root, args.output, args.pure_evidence_root)
    else:
        document = verify(args.output)
    print(json.dumps({key: document[key] for key in ("status", "file_count", "total_bytes", "runtime_source_head",
          "unique_current_drive_acceptances", "dock_status", "missing_optional")}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
