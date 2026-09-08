#!/usr/bin/env python3
"""Independent, read-only source audit of a completed low-SOC CARLA run.

Does not import or rerun the original mixed-clock helper. Only creates NEW
independent_validation.json and README.md in the selected existing run root.
Original results, media, helpers, hashes and robot state remain untouched.
"""
import argparse
import ast
import copy
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re


def require(condition, message):
    if not condition: raise ValueError(message)


def digest(path):
    path = Path(path)
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for part in iter(lambda: stream.read(1048576), b""): sha.update(part)
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": sha.hexdigest()}


def values(event):
    return dict(re.findall(r"(?:^|\s)([a-z_]+)=([^\s]+)", event["raw"].get("message", "")))


def raw_stamp(event):
    raw = event["raw"]
    stamp = raw.get("stamp", (raw.get("header") or {}).get("stamp"))
    require(isinstance(stamp, dict), "Missing control-source timestamp")
    return int(stamp["sec"])*1000000000+int(stamp["nanosec"])


def control_sequence(data):
    low = data["soc_test_inputs"][0]
    generation = data["mission_identity"]["generation"]
    require(type(generation) is int and generation > 0, "Missing fresh mission generation")
    # This is the causal cutoff. Never compare raw CARLA simulation time with
    # wall-clock PlanningReturn/ModuleState timestamps.
    events = [e for e in data["events"] if e["received_monotonic"] > low["published_monotonic"]]
    events.sort(key=lambda e: (raw_stamp(e), e["seq"]))
    def urgent(event):
        if event["kind"] != "planning_return": return False
        tokens = event["raw"].get("source", "").split(":")
        token = next((s.split("=", 1)[1] for s in tokens if s.startswith("ui_return_token=")), "")
        return (event["raw"].get("site_name") == "B1" and tokens.count("battery_urgent_return") == 1
                and tokens.count("site=B1") == 1 and tokens.count(f"g={generation}") == 1
                and re.fullmatch(rf"g{generation}-s[0-9]+-[0-9a-f]+", token) is not None)
    predicates = [urgent,
        lambda e: e["kind"] == "dispatcher" and values(e).get("parking_method") == "reverse"
            and e["raw"].get("operating_state") == "WAITING_FOR_PARKING_OWNER",
        lambda e: e["kind"] == "reverse" and e["raw"].get("operating_state") == "REVERSE_APPROACH",
        lambda e: e["kind"] == "reverse" and e["raw"].get("operating_state") == "PARKED",
        lambda e: e["kind"] == "dispatcher" and values(e).get("parking_method") == "apriltag"
            and e["raw"].get("operating_state") == "WAITING_FOR_PARKING_OWNER",
        lambda e: e["kind"] == "apriltag" and e["raw"].get("operating_state") == "WAITING_FOR_TAG",
        lambda e: e["kind"] == "apriltag" and e["raw"].get("operating_state") == "TAG_GUIDED_REVERSE",
        lambda e: e["kind"] == "apriltag" and e["raw"].get("operating_state") == "PARKED",
        lambda e: e["kind"] == "dispatcher" and e["raw"].get("operating_state") == "PARKED"
            and values(e).get("parking_method") == "apriltag"]
    found, stamp_floor = [], 0
    for index, predicate in enumerate(predicates):
        candidate = next((e for e in events if raw_stamp(e) >= stamp_floor and predicate(e)), None)
        require(candidate is not None, f"Missing independent causal/ordered stage {index}")
        # Confirm these CONTROL source stamps are wall-clock-like in this run.
        delta = dt.datetime.fromisoformat(candidate["received_utc"]).timestamp()-raw_stamp(candidate)/1e9
        require(-.05 <= delta <= 1.0, "Control source stamp is not comparable to its wall-clock receipt")
        stamp_floor = raw_stamp(candidate)
        found.append(candidate)
    reverse, april = int(values(found[1])["attempt"]), int(values(found[4])["attempt"])
    initial_attempt = int(values({"raw": data["initial"]["snapshot"]["parking"]["dispatcher"]})["attempt"])
    require(initial_attempt < reverse < april, "Reused or stale owner generation")
    require(f"start=parking_dispatcher:attempt={reverse}:" in found[2]["raw"]["message"], "Private reverse START not bound to owner")
    require("charging=False" in found[3]["raw"]["message"], "Reverse completion was not a noncharging intermediate stage")
    for event in (found[1], found[4], found[8]):
        require(values(event).get("forced") == "false" and values(event).get("charging_required") == "true",
                "Wrong forced/automatic charging policy")
    require(values(found[8]).get("attempt") == str(april), "Final public owner generation changed")
    require("charging=true" in found[7]["raw"]["message"], "April terminal lacks charger feedback")
    return found, reverse, april


def validate(root):
    result_path, status_path = root/"functional/result.json", root/"status.json"
    manifest_path = root/"desktop/capture_manifest.json"
    originals = [result_path, status_path, manifest_path, root/"functional/observations.jsonl", root/"desktop/sha256sums.txt"]
    before = [digest(p) for p in originals]
    data, status, capture = [json.loads(p.read_text()) for p in (result_path, status_path, manifest_path)]
    require(data["status"] == "PASS" and status["status"] == "PASS" and capture["status"] == "PASS", "Original run/capture not terminal PASS")
    identity, initial = data["mission_identity"], data["initial"]
    require(identity["site"] == "B1" and identity["owner"] == "operator" and identity["intent"] == "delivery", "Wrong mission identity")
    require(identity["generation"] != initial["ui"]["mission_dispatch_generation"], "Old mission generation reused")
    require(initial["ui"]["mission_dispatch_active"] is False and initial["ui"]["engaged"] is False, "Not initially inactive")
    require(initial["extras"]["platform"]["is_charging"] is False and initial["ui"]["battery_percentage"] >= 35, "Not initially noncharging/high SOC")
    require(initial["snapshot"]["parking"]["reverse"]["operating_state"] == "PARKED", "Initial reverse PARKED missing")
    require(data["dispatch"]["transport"] == "visible_operator_page_websocket_via_cdp_input"
            and data["dispatch"]["source"] == "ws", "Dispatch not from actual Robot UI")
    require(len(data["transport_probe"]["websocket"]) == 1
            and data["transport_probe"]["websocket"][0]["frame"] == data["dispatch"]["frame"], "Unexpected extra browser command frame")
    require(not any(row.get("method") == "POST" and any(s in row.get("url", "") for s in ("/ui/manual_return", "/ui/dock"))
                    for row in data["transport_probe"]["http"]), "Explicit Return/Dock traffic present")
    low, restore_input = data["soc_test_inputs"]
    require(len(data["soc_test_inputs"]) == 2 and [low["value"], restore_input["value"]] == [.24, .80], "SOC input count/value differs")
    require(all(r["topic"] == "/camrod_carla/platform_heartbeat/soc" and r["type"] == "std_msgs/Float32"
                for r in (low, restore_input)), "Non-SOC test input")
    outbound = data["checkpoints"]["02_outbound_before_soc_stimulus"]
    require(outbound["snapshot"]["service_state"]["state_name"] == "MOVING_TO_SITE"
            and outbound["snapshot"]["carla_odometry"]["speed_mps"] > .1
            and data["outbound_distance_from_drop_zone_m"] > 5
            and outbound["ui"]["mission_dispatch_generation"] == identity["generation"], "Not genuine outbound motion before SOC input")
    found, reverse, april = control_sequence(data)
    require(found[-1]["received_monotonic"] < restore_input["published_monotonic"], "SOC restored before docking completion")
    observations = [json.loads(line) for line in (root/"functional/observations.jsonl").read_text().splitlines()]
    low_feedback = next((row for row in observations if
        isinstance(row["extras"].get("platform"), dict)
        and row["extras"]["platform"]["received_monotonic"] > low["published_monotonic"]
        and abs(row["extras"]["platform"]["battery_percentage"]-.24) < .00001), None)
    require(low_feedback is not None, "24% stimulus was not observed in platform feedback")
    require(data["final"]["service_state"]["state"] == 13 and data["final_ui"]["service_state"] == 13,
            "ROS/UI do not agree on charging")
    require(data["final"]["physical_identity"]["actor_id"] == 98, "Actor identity changed")
    restore = data["soc_restore"]
    require(restore["status"] == "RESTORED_OBSERVED" and restore["input"] == restore_input, "Missing actual restoration receipt")
    require(restore["inactive_ui"]["mission_dispatch_active"] is False and restore["inactive_ui"]["engaged"] is False,
            "Restore occurred while mission/engage active")
    stop_speed = restore["stopped_snapshot"]["carla_odometry"]["speed_mps"]
    require(math.isfinite(stop_speed) and 0 <= stop_speed < .05, "Not stopped before restoring SOC")
    require(restore["actual_platform"]["received_monotonic"] > restore_input["published_monotonic"]
            and restore["actual_platform"]["battery_state_available"] is True
            and abs(restore["actual_platform"]["battery_percentage"]-.80) < .00001
            and restore["actual_platform"]["is_charging"] is True, "Missing fresh restored 80% and charging feedback")
    # Fresh odometry receipt counters were not separately serialized at restore.
    # Check the original nearby observations/counter growth; disclose this scope.
    last, previous = observations[-1], observations[-2]
    restore_age = (dt.datetime.fromisoformat(restore_input["utc"])-dt.datetime.fromisoformat(last["utc"])).total_seconds()
    require(0 <= restore_age < 1 and last["snapshot"]["motion_metrics"]["carla_odometry_samples"] >
            previous["snapshot"]["motion_metrics"]["carla_odometry_samples"], "No fresh progressing odometry observations near restore")
    require(all(row["snapshot"]["carla_odometry"]["speed_mps"] < .05 for row in (last, previous)), "Recent odometry was moving")

    checked_media = []
    def verify_media(record, base):
        path = Path(record["path"])
        path = path if path.is_absolute() else base/path
        require(path.resolve().is_relative_to(root.resolve()), "Media path escapes selected run")
        observed = digest(path)
        require(observed["bytes"] == record["bytes"] and observed["sha256"] == record["sha256"], f"Media hash mismatch: {path}")
        with path.open("rb") as stream: magic = stream.read(8)
        require(magic.startswith(b"\x89PNG\r\n\x1a\n") or magic.startswith((b"GIF87a", b"GIF89a")), "Not PNG/GIF media")
        observed["relative_path"] = str(path.relative_to(root))
        checked_media.append(observed)
    for checkpoint in data["checkpoints"].values():
        for key in ("robot_png", "rear_rgb"):
            if key in checkpoint: verify_media(checkpoint[key], root/"functional")
    for record in capture["artifacts"].values(): verify_media(record, root/"desktop")
    for line in (root/"desktop/sha256sums.txt").read_text().splitlines():
        expected, filename = line.split(None, 1)
        path = root/"desktop"/filename.lstrip("*")
        require(path.resolve().is_relative_to((root/"desktop").resolve()), "Checksum path escape")
        require(digest(path)["sha256"] == expected, f"Desktop checksum mismatch: {filename}")
    sensor_pairs = []
    for name in ("05_apriltag_approach", "06_actual_charging_complete"):
        point = data["checkpoints"][name]
        tag, rgb = point["tag"], point["rear_rgb"]
        raw = tag["raw"]["header"]["stamp"]
        tag_ns = raw["sec"]*1000000000+raw["nanosec"]
        require(tag["raw"]["id"] == 3 and tag["raw"]["family"] == "tag36h11" and rgb["dummy_flag"] is not True,
                "Wrong tag family/id or explicitly dummy image")
        require(abs(tag_ns-rgb["stamp_ns"]) <= 250000000, "Tag/RGB same-sensor-clock pairing exceeds 250ms")
        require(tag["received_monotonic"] > found[5]["received_monotonic"], "Tag capture predates actual April START")
        sensor_pairs.append({"checkpoint": name, "tag_source_stamp_ns": tag_ns, "rgb_source_stamp_ns": rgb["stamp_ns"],
            "delta_ns": abs(tag_ns-rgb["stamp_ns"]), "dummy_flag": rgb["dummy_flag"],
            "scope": "Compare tag and RGB source stamps only; no comparison to raw CARLA odometry clock"})
    # Explicitly verify only the recorded narrow helper paths, not a repo walk.
    provenance = []
    for record in data["provenance"]["files"]:
        observed = digest(Path(record["path"]))
        require(observed == record, "Recorded helper/config/source changed since this test")
        provenance.append(observed)
    original_source = Path(provenance[0]["path"]).read_text()
    publishers = [node for node in ast.walk(ast.parse(original_source)) if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute) and node.func.attr == "create_publisher"]
    require(len(publishers) == 1 and ast.unparse(publishers[0].args[1]) == "SOC_TOPIC", "Original helper publishes more than the SOC test input")
    after = [digest(p) for p in originals]
    require(before == after, "Original evidence changed while validating")
    return {"status": "PASS", "scope": "Independent recorded functional/control/clock/media validation; no rerun",
        "checked_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "validator": digest(Path(__file__).resolve()),
        "originals_unchanged": True, "original_artifacts": before, "runtime_provenance": data["provenance"],
        "mission_identity": identity, "causality": {"clock": "same-process monotonic receipt time",
            "soc_24_published_monotonic": low["published_monotonic"],
            "urgent_request_received_monotonic": found[0]["received_monotonic"],
            "latency_s": found[0]["received_monotonic"]-low["published_monotonic"],
            "source": found[0]["raw"]["source"]},
        "control_sequence": {"clock": "PlanningReturn/ModuleState raw wall-clock stamps only", "events": found,
                             "reverse_attempt": reverse, "apriltag_attempt": april},
        "clock_domains": {"raw_carla_odometry_at_soc_ns": low["stamp_ns"],
            "helper_wall_at_soc_ns": low["helper_clock_ns"], "control_urgent_raw_ns": raw_stamp(found[0]),
            "sensor_rgb_tag": "Observed bridge-restamped 1788887... source clock; same-sensor comparison only"},
        "sensor_pairs": sensor_pairs, "soc_inputs": data["soc_test_inputs"],
        "restoration": {"status": restore["status"], "pre_restore_speed_mps": stop_speed,
            "pre_restore_active": restore["inactive_ui"]["mission_dispatch_active"],
            "pre_restore_engaged": restore["inactive_ui"]["engaged"], "latest_logged_odom_age_s": restore_age,
            "actual_80_feedback": restore["actual_platform"]},
        "metrics": {"elapsed_s": data["elapsed_from_dispatch_s"], "carla_odometry_distance_m": data["carla_odometry_distance_m"],
                    "outbound_distance_from_drop_zone_m": data["outbound_distance_from_drop_zone_m"]},
        "media_verified": checked_media, "media_count": len(checked_media),
        "limitations": ["Original helper compared simulation odometry nanoseconds with control wall-clock nanoseconds. That cutoff is not accepted here.",
            "04_actual_reverse_parked checkpoint precedes the NEW reverse PARKED event. Its PNG name is misleading and is excluded as reverse-completion evidence; original bytes/name preserved.",
            "Dummy flag was unobserved (null), not proven false; actual camera/tag topics and captured RGB are preserved.",
            "Restore odometry receipt monotonic value was not serialized separately; nearby fresh logged odometry/counter growth and stopped snapshot are cross-checked.",
            "Charging feedback is CARLA contact emulation, not physical charger hardware; no 25-35% human-confirm band or all-zone claim."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_root", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    root = args.run_root.resolve()
    if args.self_test:
        original = json.loads((root/"functional/result.json").read_text())
        control_sequence(original)
        for mutation in ("pre_stimulus_receipt", "wrong_generation", "missing_token", "old_attempt", "no_reverse_park"):
            data = copy.deepcopy(original)
            urgent = next(e for e in data["events"] if e["kind"] == "planning_return")
            if mutation == "pre_stimulus_receipt": urgent["received_monotonic"] = data["soc_test_inputs"][0]["published_monotonic"]-.1
            elif mutation == "wrong_generation": data["mission_identity"]["generation"] += 1
            elif mutation == "missing_token": urgent["raw"]["source"] = urgent["raw"]["source"].split(":ui_return_token=")[0]
            elif mutation == "old_attempt":
                for e in data["events"]:
                    if e["kind"] == "dispatcher": e["raw"]["message"] = e["raw"]["message"].replace("attempt=14", "attempt=13")
            else:
                data["events"] = [e for e in data["events"] if not (e["kind"] == "reverse" and e["raw"]["operating_state"] == "PARKED")]
            try: control_sequence(data)
            except ValueError: continue
            raise AssertionError(f"Unsafe mutation accepted: {mutation}")
        print("6 independent clock/identity/sequence self-tests PASS; no evidence written")
        return
    output, readme = root/"independent_validation.json", root/"README.md"
    require(not output.exists() and not readme.exists(), "Independent outputs must be new; no overwrite")
    report = validate(root)
    with output.open("x") as stream: json.dump(report, stream, ensure_ascii=False, indent=2)
    summary = f"""# 저SOC 자동 복귀 시험 — 독립 검증

결과: **PASS**. 원본 결과·실행 helper·PNG/GIF는 수정하거나 다시 실행하지 않았습니다.
독립 판정과 원본 해시는 [independent_validation.json](independent_validation.json)에 있습니다.

- 실제 UI B1 배송 출발 후 DropZone에서 {report['metrics']['outbound_distance_from_drop_zone_m']:.6f} m 떨어진 주행 중 CARLA SOC만 24%로 한 번 입력했습니다.
- 입력 monotonic {report['causality']['soc_24_published_monotonic']:.9f} → 자동 `battery_urgent_return` 수신 {report['causality']['urgent_request_received_monotonic']:.9f}: {report['causality']['latency_s']:.6f}초 뒤입니다. 출처의 B1·임무 generation·Return token을 검증했습니다.
- 제어 토픽의 같은 wall-clock 영역 안에서 reverse attempt {report['control_sequence']['reverse_attempt']} → 실제 비충전 PARKED → April attempt {report['control_sequence']['apriltag_attempt']} → 태그 유도 → PARKED/CHARGING을 확인했습니다.
- 시간 {report['metrics']['elapsed_s']:.6f}초, CARLA odometry 거리 {report['metrics']['carla_odometry_distance_m']:.6f} m. 충전 완료 후 정지({report['restoration']['pre_restore_speed_mps']:.12g} m/s)·임무 비활성·engage 해제를 확인하고 SOC 80%를 복원했으며 실제 피드백을 받았습니다.
- 원본 PNG/GIF {report['media_count']}개와 desktop checksum 파일을 대조했습니다.

## 원본 검증기 한계와 사진 선택

원본 helper의 low_ns는 CARLA simulation odometry({report['clock_domains']['raw_carla_odometry_at_soc_ns']})이고, 제어 토픽은 wall-clock({report['clock_domains']['control_urgent_raw_ns']})입니다. 서로 직접 비교한 원본 cutoff는 유효하지 않으므로 이 독립 검증에서는 사용하지 않았습니다. 입력→요청 인과관계는 동일 프로세스 monotonic으로, 제어 순서는 제어 토픽끼리만 비교했습니다. 태그/RGB는 실제 bridge-restamped 센서 source stamp끼리 비교했으며 raw CARLA odometry와 비교하지 않았습니다.

`functional/04_actual_reverse_parked_*`는 17:14:07 UTC에 촬영돼, 이번 임무의 실제 reverse PARKED(17:14:55 UTC)보다 이릅니다. **그 이름을 신뢰해 주차 완료 사진으로 사용하면 안 됩니다.** 원본은 보존했으며 완료 입증에는 새로운 제어 이벤트와 이후 화면만 사용합니다.

확인할 자료: [전체 실제 화면 PNG](desktop/representative_contact_sheet.png), [실제 주행 GIF](desktop/representative_motion.gif), [April 유도 중 Robot UI](functional/05_apriltag_approach_robot.png), [해당 후방 RGB](functional/05_apriltag_approach_rear_rgb.png), [충전 완료 Robot UI](functional/06_actual_charging_complete_robot.png), [충전 완료 후방 RGB](functional/06_actual_charging_complete_rear_rgb.png).

dummy flag는 미수신(null)을 그대로 유지했습니다. 충전은 CARLA contact emulation이며 실물 충전기 검증이 아닙니다. 25–35% 구간의 사용자 확인 및 전 사이트 시나리오 완료를 주장하지 않습니다. SOC 복원 직전의 독립 odometry receipt 값은 별도로 직렬화되지 않았으므로, 가까운 실제 로그의 증가한 sample count·정지 속도·당시 UI 상태를 대조했습니다.

재검증 코드: [validate_low_soc_run.py](../validate_low_soc_run.py). 결과 원본: [functional/result.json](functional/result.json).
"""
    with readme.open("x") as stream: stream.write(summary)
    print(json.dumps({"status": report["status"], "media_count": report["media_count"], "output": str(output)}, ensure_ascii=False))


if __name__ == "__main__": main()
