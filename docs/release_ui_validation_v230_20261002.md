# v2.3.0 UI·음성 변경 검증 기록 (2026-10-02)

이 문서는 **커밋·푸시 전 정적/단위 테스트 결과**다. 실행 중인 CARLA 미션, ROS 프로세스, Git 참조는 변경하지 않았다. 실제 주행·화면 검증 및 릴리스 승인은 별도로 기록해야 한다.

## 실행한 검증

| 범위 | 실행 명령 | 결과 |
| --- | --- | --- |
| Robot UI 프런트엔드 전체 | `cd camrod_ui/camrod_ui_robot/assets/frontend && CI=true ./node_modules/.bin/react-scripts test --watch=false --runInBand` | 10 suites, **191 tests passed**, 0 failed |
| UI·음성 Python 전체 | `python3 -m pytest -q camrod_ui/test camrod_voice/test` | **903 passed**, 0 failed (8.35 s) |
| 배송/호출/복귀 기록, CAN, 정책, 주행 표시 중심의 선택 Python 테스트 | `python3 -m pytest -q camrod_ui/test/test_driving_snapshot.py camrod_ui/test/test_ui_backend_stop.py camrod_ui/test/test_ui_guest_cancel_restart_frontend.py camrod_ui/test/test_mission_journal.py camrod_ui/test/test_mission_recorder_node.py camrod_ui/test/test_mission_recording_bridge.py camrod_ui/test/test_mission_recording_integration.py camrod_ui/test/test_raw_can_capture.py camrod_ui/test/test_service_metrics.py camrod_ui/test/test_service_metrics_migration.py camrod_ui/test/test_battery_return_policy.py camrod_ui/test/test_manual_drive_policy.py camrod_voice/test/test_voice_event_policy.py camrod_voice/test/test_parking_voice_contract.py` | **571 passed**, 0 failed (3.88 s); 위 전체 실행에 포함 |

## `HH_YYMMDD -` 영어 주석 감사

- 현재 변경된 UI·perception·voice·control 실행 코드에는 해당 형식의 주석이 있다. 발견된 `HH_` 표기에는 잘못된 날짜/구분자 형식이 없었다.
- 최초 감사에서 [DrivingPreview.css](../camrod_ui/camrod_ui_robot/assets/frontend/src/DrivingPreview.css)와 [navigationMath.test.js](../camrod_ui/camrod_ui_robot/assets/frontend/src/navigationMath.test.js)의 날짜 주석 누락을 발견했다. 이후 실제 작성일을 보존한 `HH_261001 -` 영어 주석을 두 소스에 추가했고, develop 이식본에도 반영했다.
- 기존 [TelemetryWorkspace.js](../camrod_ui/camrod_ui_robot/assets/frontend/src/TelemetryWorkspace.js)와 [App.js](../camrod_ui/camrod_ui_robot/assets/frontend/src/App.js)의 `HH_260909` 영어 주석 각 1곳에는 한국어 UI 문자열 `도킹`, `충전`이 인용되어 있다. 한국어 설명문이 아니라 UI 라벨 인용이므로 기능 누락으로 보지 않는다.

## 아직 검증하지 않은 범위

최종 추가 검증: 정지 확인창의 미션 세대값 보호를 보강한 뒤 프런트엔드 전체는
양쪽 브랜치에서 **194 passed**로 증가했다. 순수 develop Python(UI·voice·기록 launch)은
**789 passed**, 최종 빌드는 develop `main.11090c1b.js`, virtual/carla `main.2c03cd74.js`로 성공했다.
실제 B9 시험은 [최종 검증 기록](release_v230_verification_20261002.md)에 별도로 정리한다.

- C++ control 현재 소스의 실행 테스트는 이번 감사에서 하지 않았다. 기존 `build/camrod_control/test_control_policies` 바이너리(2026-09-30 18:54 KST)가 `camrod_control/test/test_control_policies.cpp` 소스(2026-10-01 15:56 KST)보다 오래되어, 해당 바이너리 실행을 최신 소스 통과로 주장할 수 없다. CARLA 실행 중 부하를 피하기 위해 재빌드하지 않았다.
- 이 테스트는 UI, 음성, 기록 로직의 단위/계약 검증이며 CARLA 주행, 브라우저 실화면, 센서 성능 또는 실제 로봇 CAN 데이터 보존을 증명하지 않는다.
