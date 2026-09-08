# v2.2.5 가상환경 증거 묶음

**전체 검증 미완료: 현재 runtime의 주행 승인 1/39, 선택 Dock FAIL.**

원본 runtime `067568ecfe411a5cc31844fa84696220da879249`, 해당 PURE `91158175102a2bbf86624f5c5b726e5e272f8b90`. 릴리스/문서 커밋과 실제 실행 버전은 다릅니다.
B1 Guest→Robot 최종 확인→복귀·비충전 후진주차는 suite12의 동일 1건입니다. suite13 인계를 두 번째 성공으로 세지 않습니다.
B1 native 시간 868.907초 / 173.935410m; suite 준비·캡처 wall time과 구분합니다.

원본 JSON·문서·도구는 **바이트 그대로** 복사했습니다. 원본 내부 절대경로는 당시 출처이며 새 호스트에서 유효하지 않을 수 있습니다. 아래 상대 링크와 manifest의 relative_path를 사용하세요.
이미지는 기존 실제 자료입니다. exporter는 원본 hash·형식 헤더를 재확인하며 새 이미지나 성공 장면을 만들지 않습니다.
전체 raw wheel/MP4/음성 녹음은 제외한 최소 배포본이므로 원본 strict 보고서 전 항목을 독립 재실행할 수 있는 전체 데이터셋은 아닙니다.

**알려진 메타데이터 수명 차이:** strict가 기록한 run SHA는 검증 중 VALIDATING snapshot입니다. 이후 최종 PASS/updated_at 저장으로 run SHA가 바뀌었습니다. 두 원본을 그대로 보존하고 manifest.provenance_notes에 기대/현재 SHA·크기·시각을 적었습니다. site/native/metrics/media의 기존 해시는 별도로 일치 확인했습니다.

## 핵심 확인

- [B1 실제 GIF](b1_guest_recall/visual/representative_motion.gif) · [PNG](b1_guest_recall/visual/representative_contact_sheet.png) · [strict 원본](b1_guest_recall/strict.json)
- [첫 Guest 확인](b1_guest_recall/ui/first_guest.png) · [최종 Robot 확인](b1_guest_recall/ui/final_robot.png) · [비충전 복귀 화면](b1_guest_recall/ui/terminal_robot.png)
- [Dock 실패 결과](optional_docking/result.json) · [실제 GIF](optional_docking/desktop/representative_motion.gif) · [후속 실제 후방 이미지](optional_docking/investigation/actual_rear.png)
- GNSS는 수식/기존 native 회귀이며 전방 실차 장착·회전 후 중심 XY 검증 완료가 아닙니다.
- gnss/release_*는 PURE911의 새 릴리스 검사이며, original_scope.md/math_native.junit.xml은 과거5e 감사 범위입니다. 서로 다른 검사 실행을 중복 합산하지 않습니다.
- 진단 화면은 일부 실제 렌더/수신 증거입니다. 카메라 4.75Hz는 목표10Hz 인증이 아닙니다. 설정 미가용 표시 확인은 하드웨어 설정 POST 성공이 아닙니다.
- 음성 JSON은 이전 실행의 파형 감사입니다. 최신 전체 음성/도킹 성공 음성은 미검증입니다.
- tools는 당시 TEST-only 도구/회귀의 보존 사본입니다. 원래 host 경로·외부 ROS/프로젝트 의존성이 있으며, 배포본 단독 실행 안내가 아닙니다. 특히 run/--run은 실제 동작 권한이 필요합니다.

## 전체 상대 경로 목록

### B1 Guest → Robot 최종 확인 → 비충전 후진주차: suite12/067 실제 strict PASS 1건

| 자료 | 크기(bytes) |
| --- | ---: |
| [b1_guest_recall/suite_status.json](b1_guest_recall/suite_status.json) | 4818 |
| [b1_guest_recall/suite_plan.json](b1_guest_recall/suite_plan.json) | 7631 |
| [b1_guest_recall/run_manifest.json](b1_guest_recall/run_manifest.json) | 11384 |
| [b1_guest_recall/site_manifest.json](b1_guest_recall/site_manifest.json) | 5957 |
| [b1_guest_recall/native.json](b1_guest_recall/native.json) | 1042603 |
| [b1_guest_recall/metrics.json](b1_guest_recall/metrics.json) | 2389 |
| [b1_guest_recall/metrics.csv](b1_guest_recall/metrics.csv) | 773 |
| [b1_guest_recall/strict.json](b1_guest_recall/strict.json) | 5202 |
| [b1_guest_recall/strict.csv](b1_guest_recall/strict.csv) | 921 |
| [b1_guest_recall/visual/capture_manifest.json](b1_guest_recall/visual/capture_manifest.json) | 9422 |
| [b1_guest_recall/visual/representative_contact_sheet.png](b1_guest_recall/visual/representative_contact_sheet.png) | 12702517 |
| [b1_guest_recall/visual/representative_motion.gif](b1_guest_recall/visual/representative_motion.gif) | 20256423 |
| [b1_guest_recall/ui/terminal_robot.json](b1_guest_recall/ui/terminal_robot.json) | 141932 |
| [b1_guest_recall/ui/terminal_robot.png](b1_guest_recall/ui/terminal_robot.png) | 678423 |
| [b1_guest_recall/wheel_summary.json](b1_guest_recall/wheel_summary.json) | 6153 |
| [b1_guest_recall/wheel_measurements.csv](b1_guest_recall/wheel_measurements.csv) | 1830 |
| [b1_guest_recall/ui/first_guest.png](b1_guest_recall/ui/first_guest.png) | 317418 |
| [b1_guest_recall/ui/final_robot.png](b1_guest_recall/ui/final_robot.png) | 842324 |

### 선택 Dock: suite13/067 실제 FAIL, STOP 관측; 충전 성공 아님

| 자료 | 크기(bytes) |
| --- | ---: |
| [optional_docking/suite_status.json](optional_docking/suite_status.json) | 11351 |
| [optional_docking/suite_plan.json](optional_docking/suite_plan.json) | 21974 |
| [optional_docking/result.json](optional_docking/result.json) | 43471 |
| [optional_docking/desktop/capture_manifest.json](optional_docking/desktop/capture_manifest.json) | 9239 |
| [optional_docking/desktop/representative_contact_sheet.png](optional_docking/desktop/representative_contact_sheet.png) | 8957414 |
| [optional_docking/desktop/representative_motion.gif](optional_docking/desktop/representative_motion.gif) | 6278496 |
| [optional_docking/functional/01_reverse_parked_before_dock.png](optional_docking/functional/01_reverse_parked_before_dock.png) | 678872 |
| [optional_docking/functional/02_rear_camera_before_dock.png](optional_docking/functional/02_rear_camera_before_dock.png) | 1562921 |
| [optional_docking/functional/03_actual_rear_tag_detected.png](optional_docking/functional/03_actual_rear_tag_detected.png) | 1563501 |
| [optional_docking/functional/04_ui_docking_in_progress.png](optional_docking/functional/04_ui_docking_in_progress.png) | 452341 |
| [optional_docking/investigation/actual_rear.png](optional_docking/investigation/actual_rear.png) | 1403444 |
| [optional_docking/investigation/actual_rear.json](optional_docking/investigation/actual_rear.json) | 698 |
| [optional_docking/tests/safety_25.junit.xml](optional_docking/tests/safety_25.junit.xml) | 5866 |
| [optional_docking/tests/late_join_74.junit.xml](optional_docking/tests/late_join_74.junit.xml) | 12242 |

### Guest 재연결·설정 미가용·진단 실제 화면: 이전0854/940 bundle 시점, 현재 전체 인증 아님

| 자료 | 크기(bytes) |
| --- | ---: |
| [ui/guest_before/actual.png](ui/guest_before/actual.png) | 194298 |
| [ui/guest_before/observation.json](ui/guest_before/observation.json) | 1113 |
| [ui/guest_reconnected/actual.png](ui/guest_reconnected/actual.png) | 230036 |
| [ui/guest_reconnected/observation.json](ui/guest_reconnected/observation.json) | 1294 |
| [ui/tuning/actual.png](ui/tuning/actual.png) | 112737 |
| [ui/tuning/observation.json](ui/tuning/observation.json) | 3628 |
| [ui/diagnostics/observations.json](ui/diagnostics/observations.json) | 31659 |
| [ui/diagnostics/01_camera.png](ui/diagnostics/01_camera.png) | 2134200 |
| [ui/diagnostics/02_gnss.png](ui/diagnostics/02_gnss.png) | 234724 |
| [ui/diagnostics/03_proximity.png](ui/diagnostics/03_proximity.png) | 249684 |
| [ui/diagnostics/04_trajectory.png](ui/diagnostics/04_trajectory.png) | 188897 |
| [ui/diagnostics/05_perception.png](ui/diagnostics/05_perception.png) | 245471 |
| [ui/diagnostics/06_safety.png](ui/diagnostics/06_safety.png) | 340645 |
| [ui/diagnostics/07_docking.png](ui/diagnostics/07_docking.png) | 604060 |
| [ui/diagnostics/08_system.png](ui/diagnostics/08_system.png) | 112737 |

### GNSS 중심 수학/기존 native 회귀: 실차 전방 장착·회전 후 XY 인증 아님

| 자료 | 크기(bytes) |
| --- | ---: |
| [gnss/original_scope.md](gnss/original_scope.md) | 9855 |
| [gnss/math_native.junit.xml](gnss/math_native.junit.xml) | 3233 |
| [gnss/release_math_native.junit.xml](gnss/release_math_native.junit.xml) | 3233 |
| [gnss/release_native_verification_summary.json](gnss/release_native_verification_summary.json) | 10396 |

### 과거 음성 파형 감사 원본만 보존: 최신 전체 음성 검증 미완료

| 자료 | 크기(bytes) |
| --- | ---: |
| [voice/historical_signal_audit.json](voice/historical_signal_audit.json) | 16213 |

### 검증 당시 TEST-only helper 사본: runtime/PURE 코드 또는 자급 실행 패키지 아님

| 자료 | 크기(bytes) |
| --- | ---: |
| [tools/run_optional_docking_ui.py](tools/run_optional_docking_ui.py) | 18144 |
| [tools/run_durable_suite.py](tools/run_durable_suite.py) | 32659 |
| [tools/run_durable_suite.sh](tools/run_durable_suite.sh) | 1843 |
| [tools/watch_results_suite06.py](tools/watch_results_suite06.py) | 8236 |
| [tools/update_results_index.py](tools/update_results_index.py) | 36798 |
| [tools/test_durable_suite_continuation.py](tools/test_durable_suite_continuation.py) | 19096 |
| [tools/test_watch_results_suite06.py](tools/test_watch_results_suite06.py) | 12301 |
| [tools/tests/test_optional_docking_safety.py](tools/tests/test_optional_docking_safety.py) | 18290 |
| [tools/optional_docking_investigation/capture_rear_readonly.py](tools/optional_docking_investigation/capture_rear_readonly.py) | 2531 |

## 누락·검증 한계

Dock가 실패했으므로 05_ui_charging_complete.png / 06_rear_camera_charging_complete.png는 원본에 없고 생성하지 않았습니다.
dummy 상태 미수신은 None 그대로 기록하며 false로 위조하지 않습니다. tag/image 동일프레임·header freshness 전체 결합, 물리 충전기와 실차 전체 기능은 이 자료로 인증하지 않습니다.
선택 파일 61개, 원본 복사 61,917,961 bytes. [manifest.json](manifest.json)의 출처·SHA256·명시적 누락 목록과 [SHA256SUMS](SHA256SUMS)를 확인하세요.

검증: 저장소 루트에서 `python3 scripts/virtual_carla/export_v225_release_evidence.py verify --output docs/evidence/virtual_carla/v2_2_5_20260908`
