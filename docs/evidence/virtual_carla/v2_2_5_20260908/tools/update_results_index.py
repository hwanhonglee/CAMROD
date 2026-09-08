#!/usr/bin/env python3
"""Read existing evidence only; write RESULTS.md/RESULTS_INDEX.json under HERE.

No ROS, network, subprocess, runtime edits, UI input, or mission commands.
This index preserves the producer's strict acceptance decision; it does not
replace or rerun the full evidence validator.
"""
import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
EVIDENCE = HERE.parent
CURRENT_HEAD = "067568ecfe411a5cc31844fa84696220da879249"
RUNTIME_HEAD = CURRENT_HEAD


def read_json(path):
    try:
        path = Path(path).resolve()
        path.relative_to(EVIDENCE)
        if path.stat().st_size > 20 * 1024 * 1024:
            return {}, "JSON exceeds index size limit"
        value = json.loads(path.read_text())
        return (value, "") if isinstance(value, dict) else ({}, "not a JSON object")
    except (OSError, ValueError, TypeError) as error:
        return {}, str(error)


def existing_path(path):
    if not path:
        return None
    try:
        resolved = Path(path).resolve()
        resolved.relative_to(EVIDENCE)
        return resolved if resolved.is_file() else None
    except (OSError, ValueError, TypeError):
        return None


def positive(value):
    return value if type(value) in (int, float) and math.isfinite(value) and value > 0 else None


def metadata_matches(claim):
    """Hash only small evidence metadata, never source paths or wheel recordings."""
    path = existing_path(claim.get("path"))
    if path is None or path.stat().st_size > 20 * 1024 * 1024:
        return False
    payload = path.read_bytes()
    return len(payload) == claim.get("bytes") and hashlib.sha256(payload).hexdigest() == claim.get("sha256")


def strict_binding(strict, stage, root, runtime_head):
    rows = strict.get("sites", [])
    row = rows[0] if len(rows) == 1 and isinstance(rows[0], dict) else {}
    checks = {
        "strict_report_pass": strict.get("status") == "PASS" and row.get("status") == "PASS"
            and strict.get("aggregate", {}).get("pass_count") == 1
            and strict.get("aggregate", {}).get("fail_count") == 0,
        "strict_stage_identity": row.get("site") == stage.get("site")
            and row.get("authority") == stage.get("authority")
            and row.get("mission_intent") == stage.get("intent")
            and strict.get("source", {}).get("input_root") == str(root),
        "runtime_source": row.get("runtime_source_head") == runtime_head,
    }
    return row, checks


def prior_acceptance(record, row, metadata_verified):
    reference = record.get("prior_evidence") or {}
    return bool(record.get("status") == "NOT_EXECUTED_IN_THIS_SUITE"
                and reference.get("acceptance") == "PRIOR_ACCEPTED_EVIDENCE"
                and metadata_verified and row.get("accepted_current_source")
                and row.get("source_head") == reference.get("prior_test_driver_checkout_head")
                and row.get("runtime_source_head") == (reference.get("prior_runtime_identity") or {}).get("runtime_source_head"))


def acceptance(completed, run, manifest, native, item, head, visuals_exist):
    """Native PASS alone is never a strict accepted stage."""
    checks = {
        "suite_stage_pass": completed.get("status") == "PASS",
        "run_pass": run.get("status") == "PASS",
        "site_manifest_pass": manifest.get("status") == "PASS" and manifest.get("failure_reasons") == [],
        "native_pass": native.get("status") == "PASS" and item.get("status") == "PASS",
        "current_source": (run.get("source") or {}).get("head") == head,
        "png_gif_present": visuals_exist,
    }
    return all(checks.values()), checks


def state_label(accepted, checks, completed, suite, name, run, manifest, native, item):
    if accepted:
        return "PASS(검증 완료)"
    if suite.get("failed_stage") == name or "FAIL" in {
            run.get("status"), manifest.get("status"), native.get("status"), item.get("status")}:
        return "FAIL/중단"
    if completed.get("status") == "PASS":
        return "보류(증거·소스 불일치)"
    if native.get("status") == "PASS" or item.get("status") == "PASS":
        return "검증·자료 생성 중"
    if suite.get("active_stage") == name:
        return "진행 중" if suite.get("status") == "RUNNING" else "중단/미확정"
    if manifest or native or run:
        return "미확정 기록 있음"
    return "미실행"


def inspect_stage(stage, suite_root, suite, completed, head, runtime_head):
    name, site = stage["name"], stage.get("site", "B1")
    root = suite_root / name
    run, _ = read_json(root / "run_manifest.json")
    manifest_path = root / site / "site_manifest.json"
    manifest, _ = read_json(manifest_path)
    report = manifest.get("matrix_report") or {}
    native_path = existing_path(report.get("path")) or existing_path(root / site / "camping_site_matrix.json")
    if native_path is None:
        log = existing_path(root / site / "matrix.log")
        if log:
            matches = re.findall(r"report=(/[^\s]+\.json)", log.read_text(errors="replace"))
            if matches:
                native_path = existing_path(matches[-1])
    native, native_error = read_json(native_path) if native_path else ({}, "native report not created")
    items = [item for item in native.get("sites", []) if isinstance(item, dict) and item.get("site") == site]
    item = items[0] if len(items) == 1 else {}
    visual = manifest.get("visual") or {}
    png = existing_path((visual.get("png") or {}).get("path"))
    gif = existing_path((visual.get("gif") or {}).get("path"))
    # Existing but unfinished visuals may be linked; their existence is never
    # sufficient to make a stage accepted.
    png = png or existing_path(root / site / "visual/representative_contact_sheet.png")
    gif = gif or existing_path(root / site / "visual/representative_motion.gif")
    done = completed.get(name, {})
    accepted, checks = acceptance(done, run, manifest, native, item, head, bool(png and gif))
    strict_path = suite_root / (name + ".strict_validation") / "site_evidence_collection.json"
    strict, _ = read_json(strict_path)
    strict_row, strict_checks = strict_binding(strict, stage, root, runtime_head)
    checks.update(strict_checks)
    actual_authority = manifest.get("authority") or {}
    checks["stage_identity"] = bool(
        manifest.get("site") == site and item.get("site") == site
        and run.get("selected_sites") == [site]
        and item.get("mission_intent") == stage.get("intent")
        and actual_authority.get("frontend") == stage.get("authority")
        and actual_authority.get("mission_intent") == stage.get("intent"))
    accepted = all(checks.values())
    metrics_source = manifest.get("motion_metrics") or item
    # Running reports initialize these counters to zero; do not print them as
    # measured performance. Keep even nonzero interim values out of final rows.
    metrics = {}
    if manifest.get("status") in {"PASS", "FAIL"} or item.get("status") in {"PASS", "FAIL"}:
        for key in ("elapsed_s", "outbound_duration_s", "return_duration_s",
                    "total_odom_distance_m", "outbound_distance_m", "return_distance_m"):
            metrics[key] = positive(metrics_source.get(key))
    first_return = item.get("return_response") or {}
    first_return_frame = first_return.get("frame") or {}
    first_return_ack = first_return.get("ros_ack") or {}
    return {"stage": name, "site": site, "authority": stage.get("authority"), "intent": stage.get("intent"),
            "status": state_label(accepted, checks, done, suite, name, run, manifest, native, item),
            "accepted_current_source": accepted, "accepted_prior_source": False,
            "execution_scope": "CURRENT_SUITE", "origin_suite": suite_root.name,
            "acceptance_checks": checks, "runtime_source_head": strict_row.get("runtime_source_head"),
            "source_head": (run.get("source") or {}).get("head"), "metrics": metrics,
            "strict_report": str(existing_path(strict_path) or ""),
            "site_manifest": str(existing_path(manifest_path) or ""),
            "native_report": str(native_path or ""), "native_read_error": native_error,
            "png": str(png or ""), "gif": str(gif or ""),
            "actor_id": item.get("actor_id"), "mission_identity": item.get("mission_identity") or {},
            "observed_milestones": [{key: value.get(key) for key in ("at_utc", "event", "elapsed_s")}
                                    for value in item.get("milestones", []) if isinstance(value, dict)],
            "first_return": {"accepted": first_return.get("accepted"),
                             "recall_final_return": first_return_frame.get("recall_final_return"),
                             "expected_ros_source": first_return.get("expected_ros_source"),
                             "actual_controller_source": first_return_ack.get("controller_source"),
                             "token": first_return_ack.get("token")},
            "failure_reason": item.get("failure_reason") or manifest.get("failure_reasons") or
                              (suite.get("error") if suite.get("failed_stage") == name else "")}


def unexecuted_stage(record):
    """Represent a declared skip without looking up or importing an old PASS."""
    match = re.fullmatch(r"(B(?:[1-9]|1[0-3]))_(robot_delivery|operator_recall|guest_recall_robot_handoff)", record.get("stage", ""))
    if record.get("status") != "NOT_EXECUTED_IN_THIS_SUITE" or not match:
        raise ValueError("Unknown/non-skipped stage declaration")
    site, kind = match.groups()
    return {"stage": record["stage"], "site": site,
            "authority": "guest" if kind == "guest_recall_robot_handoff" else "operator-browser",
            "intent": "delivery" if kind == "robot_delivery" else "recall",
            "status": "미실행(별도 재시험 필요)", "execution_scope": "NOT_EXECUTED_IN_THIS_SUITE",
            "origin_suite": "이번 묶음에서 제외", "accepted_current_source": False,
            "accepted_prior_source": False, "acceptance_checks": {}, "metrics": {},
            "source_head": None, "runtime_source_head": None, "site_manifest": "",
            "native_report": "", "strict_report": "", "png": "", "gif": "",
            "failure_reason": "No accepted continuation; this scenario still requires a fresh execution"}


def inspect_prior_stages(plan, suite, runtime_head):
    records = plan.get("skipped_prior_stages", [])
    if not records:
        return [], []
    if not plan.get("continuation_of"):
        rows, errors = [], []
        for record in records:
            try:
                rows.append(unexecuted_stage(record))
                if record.get("prior_evidence"):
                    errors.append(f"Ignoring unbound prior evidence for {record.get('stage')}")
            except ValueError as error:
                errors.append(str(error))
        return rows, errors
    prior_root = Path(plan.get("continuation_of") or "").resolve()
    try:
        prior_root.relative_to(HERE)
    except ValueError:
        return [], ["Continuation evidence is outside this validation root"]
    prior_plan, error = read_json(prior_root / "suite_plan.json")
    prior_suite, status_error = read_json(prior_root / "suite_status.json")
    if error or status_error:
        return [], ["Cannot read original continuation plan/status"]
    prior_stages = {stage["name"]: stage for stage in prior_plan.get("stages", [])}
    completed = {item["stage"]: item for item in prior_suite.get("completed", []) if isinstance(item, dict) and "stage" in item}
    rows, errors = [], []
    for record in records:
        name = record.get("stage")
        if record.get("prior_evidence") is None:
            try:
                rows.append(unexecuted_stage(record))
                if record not in prior_plan.get("skipped_prior_stages", []) or record not in suite.get("skipped_prior_stages", []):
                    errors.append(f"Unexecuted continuation declaration not preserved in original/current plan: {name}")
            except ValueError as error:
                errors.append(str(error))
            continue
        stage = prior_stages.get(name)
        if not stage or stage.get("kind") == "dock":
            errors.append(f"No supported original stage identity for {name}")
            continue
        reference = record.get("prior_evidence") or {}
        prior_head = reference.get("prior_test_driver_checkout_head")
        row = inspect_stage(stage, prior_root, prior_suite, completed, prior_head, runtime_head)
        claims = ("run_manifest", "site_manifest", "strict_report", "strict_checksums")
        expected_paths = {"run_manifest": prior_root / name / "run_manifest.json",
                          "site_manifest": prior_root / name / stage["site"] / "site_manifest.json",
                          "strict_report": prior_root / (name + ".strict_validation") / "site_evidence_collection.json",
                          "strict_checksums": prior_root / (name + ".strict_validation") / "SHA256SUMS"}
        metadata_verified = bool(re.fullmatch(r"[0-9a-f]{40}", prior_head or "")
                                 and record in suite.get("skipped_prior_stages", [])
                                 and reference.get("prior_stage_root") == str(prior_root / name)
                                 and all((reference.get(key) or {}).get("path") == str(expected_paths[key]) for key in claims)
                                 and all(metadata_matches(reference.get(key) or {}) for key in claims))
        accepted = prior_acceptance(record, row, metadata_verified)
        row.update(status="기존 PASS 인계(새 실행 아님)" if accepted else "인계 보류(원본 확인 필요)",
                   accepted_current_source=False, accepted_prior_source=accepted,
                   execution_scope="PRIOR_SUITE_REFERENCE", prior_metadata_verified=metadata_verified)
        rows.append(row)
    return rows, errors


def link(path, label):
    target = existing_path(path)
    return f"[{label}]({os.path.relpath(target, HERE)})" if target else "—"


def metric(value, decimals=3):
    return f"{value:.{decimals}f}" if positive(value) is not None else "—"


def failure_excerpt(reason):
    text = str(reason or "").replace("\n", " ")
    return text if len(text) <= 900 else text[:900] + " … [긴 원문은 native 보고서에 보존]"


def render(document):
    lines = ["# 최신 사이트별 검증 결과", "", f"집계 시각: {document['generated_utc']}.", "",
             f"이번 테스트 드라이버: `{document['expected_head']}`. 검증 대상 주행 runtime 기준: `{document['expected_runtime_head']}`.", "",
             f"Suite: `{document['suite']}` · 상태 `{document['suite_status']}` · 현재 단계 `{document['active_stage'] or '없음'}`.",
             f"이번 suite 새 실행 검증 완료: **{document['accepted_stage_count']} / {document['current_execution_stage_count']}개 주행 단계**. 이전 suite 승인 자료 인계: **{document['accepted_prior_stage_count']}개**(새 실행·새 PASS 수에 넣지 않음). 전체 계획은 주행 {len(document['rows'])}개이며 선택 도킹은 별도입니다.", "",
             f"이번 묶음에서 제외되어 별도 재시험이 필요한 주행: **{document['unexecuted_stage_count']}개**. 새 runtime으로 바뀌면 이전 버전의 PASS를 현재 승인으로 가져오지 않습니다.", "",
             "`PASS(검증 완료)`는 suite의 해당 STAGE_PASS, run/site manifest PASS, native PASS, strict 보고서 PASS, 테스트 드라이버와 runtime 각각의 버전 일치, 실제 PNG/GIF 존재를 모두 확인한 경우입니다. Native PASS만 있으면 완료로 승격하지 않습니다. 이 인덱스는 원본 파일을 수정하거나 전체 무결성 검사를 다시 실행하지 않습니다.", "",
             "`기존 PASS 인계(새 실행 아님)`는 continuation manifest가 명시한 이전 단계입니다. 원래 suite의 승인 기록과 소형 manifest 해시를 재확인하고 원래 PNG/GIF·버전을 그대로 연결합니다. 새 버전에서 다시 실행했다는 뜻이 아닙니다.", "",
             "미실행·진행 중 값은 `—`입니다. 실패 단계의 시간·거리가 표시되면 중단 전 관측량일 뿐 정상 왕복 성능이 아닙니다. 소스가 다른 과거 B1 결과는 아래 별도 항목으로 분리합니다.", "",
             "| 사이트 | 일반 배송·복귀 | Robot recall | Guest recall → Robot 완료 |", "| --- | --- | --- | --- |"]
    if document.get("continuation_errors"):
        lines[2:2] = ["인계 확인 경고: " + "; ".join(document["continuation_errors"]), ""]
    grouped = {}
    for row in document["rows"]:
        key = "delivery" if row["intent"] == "delivery" else "guest" if row["authority"] == "guest" else "recall"
        grouped.setdefault(row["site"], {})[key] = row["status"]
    for number in range(1, 14):
        site = f"B{number}"
        values = grouped.get(site, {})
        lines.append(f"| {site} | " + " | ".join(values.get(key, "미계획") for key in ("delivery", "recall", "guest")) + " |")
    failures = [row for row in document["rows"] if row["status"] == "FAIL/중단"]
    if failures:
        lines.extend(["", "## 실제 실패 위치와 이미 승인된 동작", "",
                      "아래는 기존 native 기록을 읽은 결과이며 새 시험이나 원인 재현을 실행한 것이 아닙니다.", ""])
    for row in failures:
        first = row.get("first_return") or {}
        if not row.get("native_report"):
            reason = row.get("failure_reason")
            scope = "출발 전 화면 준비 실패" if "Expected exactly one X11 window" in str(reason) else "임무 native 기록 없음"
            lines.extend([f"### {row['stage']}", "", scope + ". actor·임무 identity·완료 ACK가 기록되지 않아 숫자나 성공 여부를 채워 넣지 않습니다.",
                          "", f"실제 중단 원문: `{failure_excerpt(reason)}`", ""])
            continue
        identity_text = json.dumps(row["mission_identity"], ensure_ascii=False) if row.get("mission_identity") else "미기록"
        lines.extend([f"### {row['stage']}", "",
                      f"actor `{row.get('actor_id')}` · 임무 identity `{identity_text}`.", ""])
        for milestone in row.get("observed_milestones", []):
            if milestone.get("event") in {"GUEST_LOADING_WAIT with ordered WAIT_RETURN phase", "return request accepted", "fresh mission-bound RETURN source observed"}:
                lines.append(f"- `{milestone.get('at_utc')}`: {milestone.get('event')} (시작 후 {metric(milestone.get('elapsed_s'))}초).")
        if first.get("accepted") is None:
            lines.extend(["", "완료 요청·새 controller ACK: 이 실행에는 관측 기록 없음. 성공이나 제어 명령 거부로 추정하지 않습니다.", ""])
        else:
            lines.extend(["", f"첫 완료 요청 승인: `{first.get('accepted')}` · 최종 적재 확인 플래그: `{first.get('recall_final_return')}`.",
                          f"기대 ROS source: `{first.get('expected_ros_source')}`.",
                          f"실제 새 controller ACK: `{first.get('actual_controller_source')}`.", ""])
        lines.extend([f"실제 중단 원문: `{failure_excerpt(row.get('failure_reason'))}`", ""])
        if "crab-entry body-yaw precompensate_entry steady timeout" in str(row.get("failure_reason")):
            lines.extend(["이번 중단은 **CARLA에서 활성화한 전용 body-yaw `precompensate_entry`의 정착 시간 초과**입니다. 첫 recall ACK를 못 받은 실패나 일반 회전 deadband라는 이전 가설로 바꿔 적지 않습니다. 마지막 0 각속도는 오류 처리 뒤 값일 수 있으므로, 그 값만으로 시도 내내 명령이 0이었다고 판단하지 않습니다. 최종 적재 확인/출차 단계에는 도달하지 못했으므로 관련 자료 누락은 이 조기 중단과 구분해 해석합니다.", ""])
    lines.extend(["", "## 단계별 실제 기록", "",
                  "시간은 초, 거리는 m입니다. `편도 / 복귀`에는 해당 native 보고서의 구간 집계만 사용합니다.", "",
                  "| 단계 | 판정·출처 | 총 시간 | 편도 / 복귀 시간 | 총 거리 | 편도 / 복귀 거리 | 자료 |", "| --- | --- | ---: | --- | ---: | --- | --- |"])
    for row in document["rows"]:
        m = row["metrics"]
        artifacts = " · ".join(link(row[key], label) for key, label in (("png", "PNG"), ("gif", "GIF"), ("site_manifest", "manifest"), ("native_report", "native"), ("strict_report", "strict")))
        source = f"driver `{(row.get('source_head') or '미기록')[:9]}` / runtime `{(row.get('runtime_source_head') or '엄격 검증 전')[:9]}`"
        lines.append(f"| {row['stage']} | {row['status']}<br>{row['origin_suite']}<br>{source} | {metric(m.get('elapsed_s'))} | "
                     f"{metric(m.get('outbound_duration_s'))} / {metric(m.get('return_duration_s'))} | "
                     f"{metric(m.get('total_odom_distance_m'),6)} | {metric(m.get('outbound_distance_m'),6)} / {metric(m.get('return_distance_m'),6)} | {artifacts} |")
    dock = document["optional_docking"]
    lines.extend(["", "## 선택 도킹", "", f"상태: **{dock['status']}**. {link(dock['result'], '실제 결과')} · {link(dock['png'], 'PNG')} · {link(dock['gif'], 'GIF')}", "",
                  "## 이전 소스의 유효한 B1 기록 — 최신 소스 결과와 별도", ""])
    old = document["historical_b1"]
    if old:
        lines.extend([f"`{old['source_head']}`의 B1 site manifest는 `{old['status']}`입니다. **현재 `{document['expected_head'][:9]}`의 통과 수에는 포함하지 않습니다.**",
                      f"총 {metric(old['metrics'].get('elapsed_s'))}초 / {metric(old['metrics'].get('total_odom_distance_m'),6)}m. "
                      f"{link(old['manifest'], 'manifest')} · {link(old['png'], 'PNG')} · {link(old['gif'], 'GIF')}"])
    else:
        lines.append("이전 B1 manifest를 확인할 수 없습니다.")
    recent_old = document.get("historical_suite02_b1") or {}
    if recent_old:
        lines.extend(["", "### suite02 B1 배송 — 8aad runtime의 과거 자료", "",
                      f"원본 소스 `{recent_old['source_head']}` · manifest `{recent_old['status']}`. 새 runtime의 승인 수에 자동 포함하지 않습니다.",
                      f"총 {metric(recent_old['metrics'].get('elapsed_s'))}초 / {metric(recent_old['metrics'].get('total_odom_distance_m'),6)}m. "
                      f"{link(recent_old['manifest'], 'manifest')} · {link(recent_old['png'], 'PNG')} · {link(recent_old['gif'], 'GIF')}"])
    previous_failure = document.get("historical_suite05_recall") or {}
    if previous_failure and document["suite"] != "full_suite_v224_05":
        first = previous_failure.get("first_return") or {}
        lines.extend(["", "## 이전 suite05 실패 — 당시 1bfd runtime에 고정", "",
                      "actor 50 / B1 Robot recall. 첫 `recall_loading_complete` 요청과 새 mission-bound ROS ACK는 성공했으나, CARLA 전용 `precompensate_entry`가 오차 1.68°에서 1.50° 조건을 만족하지 못해 15초 timeout으로 중단됐습니다. 최신 실행의 결과가 아니며 첫 ACK 실패나 일반 회전 deadband로 재분류하지 않습니다.",
                      f"당시 드라이버 `{previous_failure.get('source_head')}` · 실제 첫 ACK `{first.get('actual_controller_source')}`.",
                      f"관측 {metric(previous_failure.get('metrics', {}).get('elapsed_s'))}초 / {metric(previous_failure.get('metrics', {}).get('total_odom_distance_m'), 6)}m는 실패 전 부분 주행량입니다.",
                      f"{link(previous_failure.get('site_manifest'), 'FAIL manifest')} · {link(previous_failure.get('native_report'), 'native')} · {link(previous_failure.get('png'), '실제 PNG')} · {link(previous_failure.get('gif'), '실제 GIF')}"])
    historical_recall = document.get("historical_suite06_recall") or {}
    if historical_recall and historical_recall.get("runtime_source_head") != document["expected_runtime_head"]:
        lines.extend(["", "## 이전 suite06 B1 Robot recall 승인 — 새 runtime 승인 아님", "",
                      f"원본 driver `{historical_recall.get('source_head')}` / runtime `{historical_recall.get('runtime_source_head')}` · 당시 `{historical_recall.get('status')}`. 새 runtime에는 자동 인계하거나 새 PASS로 집계하지 않습니다.",
                      f"원본 {metric(historical_recall.get('metrics', {}).get('elapsed_s'))}초 / {metric(historical_recall.get('metrics', {}).get('total_odom_distance_m'), 6)}m. "
                      f"{link(historical_recall.get('site_manifest'), 'manifest')} · {link(historical_recall.get('strict_report'), 'strict')} · {link(historical_recall.get('png'), '실제 PNG')} · {link(historical_recall.get('gif'), '실제 GIF')}"])
    lines.extend(["", "## 정지 상태 실제 진단 화면 — 주행 승인과 별도", "",
                  "[8개 탭 PNG·관측 제한](README.md#정지-상태-진단-8개-실제-png) · [원본 DOM·프레임·해시](diagnostic_tabs_after_suite05_settled/diagnostic_tabs.json). 카메라는 실제 전방 800×600 / 후방 960×720 렌더와 프레임 증가를 확인했습니다. 해당 캡처 표시값은 6.2 / 5.5Hz로 목표 10Hz 도달 증거가 아닙니다. 나머지 탭은 화면 수집이며 NO DATA·0개 레이어·미검증 상태를 성공으로 올리지 않습니다."])
    lines.extend(["", "## 다시 집계하기", "", "```bash",
                  f"python3 {HERE / 'update_results_index.py'} --suite {document['suite']} --expected-head {document['expected_head']} --runtime-head {document['expected_runtime_head']}", "```", "",
                  "출력은 이 폴더의 `RESULTS.md`, `RESULTS_INDEX.json` 두 파일뿐입니다. ROS/서버/UI/지도/주행 상태에는 접근하지 않고, 로컬 원본 증거 JSON과 파일 존재만 읽습니다. 원본 증거는 수정하지 않습니다.", ""])
    return "\n".join(lines)


def generate(suite_root, head, runtime_head):
    suite, error = read_json(suite_root / "suite_status.json")
    if error:
        raise RuntimeError(f"Cannot index suite status: {error}")
    plan, error = read_json(suite_root / "suite_plan.json")
    if error:
        raise RuntimeError(f"Cannot index suite plan: {error}")
    completed = {item["stage"]: item for item in suite.get("completed", []) if isinstance(item, dict) and "stage" in item}
    rows = [inspect_stage(stage, suite_root, suite, completed, head, runtime_head) for stage in plan.get("stages", []) if stage.get("kind") != "dock"]
    current_count = len(rows)
    prior_rows, continuation_errors = inspect_prior_stages(plan, suite, runtime_head)
    rows = prior_rows + rows
    dock_root = suite_root / "B1_optional_docking"
    dock_result_path = existing_path(dock_root / "functional/result.json")
    dock_result, _ = read_json(dock_result_path) if dock_result_path else ({}, "")
    dock_png = existing_path(dock_root / "desktop/representative_contact_sheet.png")
    dock_gif = existing_path(dock_root / "desktop/representative_motion.gif")
    dock_pass = completed.get("B1_optional_docking", {}).get("status") == "PASS" and dock_result.get("status") == "PASS" and dock_result.get("charging_confirmed") is True and dock_png and dock_gif
    dock_status = "PASS(단계 검증 완료)" if dock_pass else "FAIL/중단" if suite.get("failed_stage") == "B1_optional_docking" else "진행 중" if suite.get("active_stage") == "B1_optional_docking" else "미실행/미확정"
    old_path = HERE / "operator_delivery_current/B1/site_manifest.json"
    old, _ = read_json(old_path)
    old_run, _ = read_json(HERE / "operator_delivery_current/run_manifest.json")
    old_visual = old.get("visual") or {}
    historical = {"status": old.get("status"), "source_head": (old_run.get("source") or {}).get("head"), "metrics": old.get("motion_metrics") or {},
                  "manifest": str(old_path), "png": (old_visual.get("png") or {}).get("path"), "gif": (old_visual.get("gif") or {}).get("path")} if old else {}
    recent_old_root = HERE / "full_suite_v224_02/B1_robot_delivery"
    recent_old_path = recent_old_root / "B1/site_manifest.json"
    recent_old, _ = read_json(recent_old_path)
    recent_old_run, _ = read_json(recent_old_root / "run_manifest.json")
    recent_old_visual = recent_old.get("visual") or {}
    historical_recent = {"status": recent_old.get("status"), "source_head": (recent_old_run.get("source") or {}).get("head"),
                         "metrics": recent_old.get("motion_metrics") or {}, "manifest": str(recent_old_path),
                         "png": (recent_old_visual.get("png") or {}).get("path"), "gif": (recent_old_visual.get("gif") or {}).get("path")} if recent_old else {}
    previous_root = HERE / "full_suite_v224_05"
    previous_suite, _ = read_json(previous_root / "suite_status.json")
    previous_plan, _ = read_json(previous_root / "suite_plan.json")
    previous_stage = next((stage for stage in previous_plan.get("stages", [])
                           if stage.get("name") == "B1_operator_recall"), None)
    previous_head = "1bfd339d5ce1d92dc992da8c923be99ab6d477f0"
    historical_failure = inspect_stage(previous_stage, previous_root, previous_suite, {}, previous_head, previous_head) if previous_stage else {}
    old_recall_root = HERE / "full_suite_v224_06"
    old_recall_suite, _ = read_json(old_recall_root / "suite_status.json")
    old_recall_plan, _ = read_json(old_recall_root / "suite_plan.json")
    old_recall_stage = next((stage for stage in old_recall_plan.get("stages", []) if stage.get("name") == "B1_operator_recall"), None)
    old_recall_completed = {item["stage"]: item for item in old_recall_suite.get("completed", []) if isinstance(item, dict) and "stage" in item}
    old_recall_head = "9f7cd2bacda17a4581bd44ed66aaba857e89f1d4"
    old_recall_row = inspect_stage(old_recall_stage, old_recall_root, old_recall_suite, old_recall_completed, old_recall_head, old_recall_head) if old_recall_stage else {}
    historical_recall = {key: old_recall_row.get(key) for key in ("status", "source_head", "runtime_source_head", "metrics", "site_manifest", "strict_report", "png", "gif")} if old_recall_row else {}
    return {"schema": "camrod.evidence_readonly_index.v2", "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
            "suite": suite_root.name, "suite_status": suite.get("status"), "active_stage": suite.get("active_stage"),
            "expected_head": head, "expected_runtime_head": runtime_head,
            "recorded_test_driver_head": (plan.get("test_driver_identity") or {}).get("checkout_head"),
            "continuation_of": plan.get("continuation_of"), "continuation_errors": continuation_errors,
            "current_execution_stage_count": current_count,
            "unexecuted_stage_count": sum(row["execution_scope"] == "NOT_EXECUTED_IN_THIS_SUITE" for row in rows),
            "accepted_stage_count": sum(row["accepted_current_source"] for row in rows),
            "accepted_prior_stage_count": sum(row["accepted_prior_source"] for row in rows), "rows": rows,
            "optional_docking": {"status": dock_status, "result": str(dock_result_path or ""), "png": str(dock_png or ""), "gif": str(dock_gif or "")},
            "historical_b1": historical, "historical_suite02_b1": historical_recent,
            "historical_suite05_recall": historical_failure,
            "historical_suite06_recall": historical_recall}


def self_test():
    done = {"status": "PASS"}
    run = {"status": "PASS", "source": {"head": CURRENT_HEAD}}
    manifest = {"status": "PASS", "failure_reasons": []}
    native = item = {"status": "PASS"}
    assert acceptance(done, run, manifest, native, item, CURRENT_HEAD, True)[0]
    assert not acceptance({}, run, manifest, native, item, CURRENT_HEAD, True)[0]
    assert not acceptance(done, run, manifest, native, item, "78f4f0b", True)[0]
    assert not acceptance(done, run, manifest, native, item, CURRENT_HEAD, False)[0]
    assert not acceptance(done, run, {"status": "PASS", "failure_reasons": ["error"]}, native, item, CURRENT_HEAD, True)[0]
    assert state_label(False, {}, {}, {"active_stage": "B1", "status": "RUNNING"}, "B1", {}, {}, native, item) == "검증·자료 생성 중"
    assert all(positive(value) is None for value in [0, 0.0, -1, True, float("nan"), float("inf"), None])
    assert positive(747.908) == 747.908
    stage = {"name": "B1_robot_delivery", "site": "B1", "authority": "operator-browser", "intent": "delivery"}
    strict = {"status": "PASS", "aggregate": {"pass_count": 1, "fail_count": 0},
              "source": {"input_root": "/evidence/B1_robot_delivery"},
              "sites": [{"status": "PASS", "site": "B1", "authority": "operator-browser", "mission_intent": "delivery", "runtime_source_head": RUNTIME_HEAD}]}
    root = Path("/evidence/B1_robot_delivery")
    assert all(strict_binding(strict, stage, root, RUNTIME_HEAD)[1].values())
    assert not all(strict_binding(strict, stage, root, "0" * 40)[1].values())
    assert not all(strict_binding({}, stage, root, RUNTIME_HEAD)[1].values())
    assert not all(strict_binding(strict, dict(stage, site="B2"), root, RUNTIME_HEAD)[1].values())
    assert not all(strict_binding(strict, stage, Path("/different"), RUNTIME_HEAD)[1].values())
    record = {"status": "NOT_EXECUTED_IN_THIS_SUITE", "prior_evidence": {
        "acceptance": "PRIOR_ACCEPTED_EVIDENCE", "prior_test_driver_checkout_head": RUNTIME_HEAD,
        "prior_runtime_identity": {"runtime_source_head": RUNTIME_HEAD}}}
    row = {"accepted_current_source": True, "source_head": RUNTIME_HEAD, "runtime_source_head": RUNTIME_HEAD}
    assert prior_acceptance(record, row, True)
    assert not prior_acceptance(record, row, False)
    assert not prior_acceptance(record, dict(row, accepted_current_source=False), True)
    assert not prior_acceptance(record, dict(row, source_head="0" * 40), True)
    assert not prior_acceptance(record, dict(row, runtime_source_head="0" * 40), True)
    assert not prior_acceptance(dict(record, status="PASS"), row, True)
    assert not prior_acceptance({}, row, True)
    skip = {"stage": "B1_robot_delivery", "status": "NOT_EXECUTED_IN_THIS_SUITE", "prior_evidence": None}
    skipped = unexecuted_stage(skip)
    assert skipped["site"] == "B1" and skipped["intent"] == "delivery"
    assert not skipped["accepted_current_source"] and not skipped["accepted_prior_source"]
    assert skipped["metrics"] == {} and not skipped["png"] and not skipped["source_head"]
    rows, errors = inspect_prior_stages({"skipped_prior_stages": [skip]}, {}, RUNTIME_HEAD)
    assert not errors and rows == [skipped]
    rows, errors = inspect_prior_stages({"skipped_prior_stages": [dict(skip, prior_evidence=record["prior_evidence"])]}, {}, RUNTIME_HEAD)
    assert errors and not rows[0]["accepted_prior_source"]
    print("INDEX_CONTRACTS: 25 passed; no files written")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", default="full_suite_v224_13")
    parser.add_argument("--expected-head", default=CURRENT_HEAD)
    parser.add_argument("--runtime-head", default=RUNTIME_HEAD)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.suite) or not all(re.fullmatch(r"[0-9a-f]{40}", head) for head in (args.expected_head, args.runtime_head)):
        parser.error("Require a local suite directory name and full source commit SHA")
    suite_root = (HERE / args.suite).resolve()
    suite_root.relative_to(HERE)
    document = generate(suite_root, args.expected_head, args.runtime_head)
    for name, text in (("RESULTS.md", render(document)), ("RESULTS_INDEX.json", json.dumps(document, ensure_ascii=False, indent=2) + "\n")):
        path = HERE / name
        if path.is_symlink():
            raise RuntimeError(f"Refuse index symlink: {path}")
        path.write_text(text)
    print(f"{document['suite']}: {document['accepted_stage_count']}/{document['current_execution_stage_count']} newly accepted; {document['accepted_prior_stage_count']} prior accepted references; {document['active_stage']} {document['suite_status']}")


if __name__ == "__main__":
    main()
