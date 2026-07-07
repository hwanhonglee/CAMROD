---
name: light-indicator-feature
description: 전조등/방향지시등(WS2815+Arduino Nano) 기능의 확정된 설계 결정 사항
metadata: 
  node_type: memory
  type: project
  originSessionId: 874cb916-2ddd-4e00-a4fe-2c0616836bfd
---

전조등 + 방향지시 인디케이터 기능 개발 (2026-07-07 결정 완료, camrod_platform 하위 구현 예정).

**확정 결정:**
1. **방향 판정 = B안(태그 조회)**: `/planning/route_lanelet_ids` (Int64MultiArray, transient_local, lanelet_route_planner가 발행)의 lanelet ID를 Lanelet2 지도의 `turn_direction` 태그(left/right/straight)로 조회. 다음 left/right 구간 시작까지 거리 D ≤ D_pre(기본 12m, 파라미터)면 점등, 통과 후 소등. **지도 태그 검증됨**: c_track_test_moved.osm = 532 lanelets 중 454 태그(straight 324/left 53/right 77), 분기점 36곳; copy_park.osm = 30 중 28 태그. A안(로컬 경로 곡률)은 미구현 — 그리드 폴백 플래너 주행 중엔 소등 유지.
2. 비상등 = ERROR_STOP(`/planning/state_machine/state`) / estop(`/platform/status/estop`, `/planning/state_machine/estop`)만. 우선순위 HAZARD > 좌/우 > OFF.
3. 통신 두절(하트비트 타임아웃) 시 MCU가 **비상등 점멸**. 단 부팅 부작용 방지: MCU는 최초 유효 프레임 수신 이후부터만 두절 판정 시작.
4. 크랩(게걸음) 기동 중 = **이동 방향 지시등** (site_maneuver 활성 + cmd_vel linear.y 부호).
5. 깜빡임은 MCU(Arduino Nano) 자율 생성(약 1.25Hz, 호박색); PC는 모드+하트비트만 JSON으로 전송.
6. 배선: Relay IN=**D12**(전조등; D13→D12 변경으로 부트로더 점멸 문제 해소), 좌 WS2815 D5(데이터)/D6(백업), 우 D7(데이터)/D8(백업). WS2815=12V, 백업 데이터라인 있음. LED 개수는 스케치 #define.
7. 시리얼: JSON over Serial. **`/dev/serial/by-id/` 경로 필수** — Nano(CH340)가 ttyUSB로 잡히면 레이더 CH9344 포트 열거와 충돌 위험 ([[field-baseline-and-conventions]]의 레이더 USB 포트맵 참조).
8. 전조등: UI 버튼 → FastAPI → 신규 토픽(예: /platform/headlight/command) → MCU.
9. 구조: camrod_platform에 light_controller_node(모드 결정) + mcu_serial_bridge_node(시리얼) 분리 구성.
10. **구현 언어 = C++ 통일** (platform 기존 노드 3개 전부 C++). 시리얼=POSIX termios(무의존성), JSON=nlohmann/json.hpp(시스템 설치 + nav2_smac_planner 사용 선례), 테스트=gtest(판정 로직은 ROS 비의존 클래스로 분리).
11. **B안 데이터 공급 = lanelet_route_planner 확장 (코드 검증 완료)**: createPlan 루프(205-220행)가 lanelet별 호길이(segment_start_s/end_s)를 이미 계산 → 누적 + `attributeOr("turn_direction","")` 조회로 `/planning/route_turn_segments`(Float32MultiArray, transient_local) 발행. route_lanelet_ids 발행(HH_260619)과 동일 패턴의 additive 변경(~40-60줄). **동기화 가드**: 배열 [0]=전체 경로 길이, light_controller가 global_path 길이와 ±1m 대조, 불일치 사이클은 소등.

JSON 스키마는 작게 유지 (Nano SRAM 2KB): PC→MCU `{"h":1,"i":"L","q":42}`, MCU→PC ack `{"a":42,...}`.
설명 자료(저장소 내로 이동됨): `src/camrod_platform/docs/turn-signal-explainer.html` (A/B안 그림 비교), `src/camrod_platform/docs/lights-design-doc.html` (구현 설계서+TODO 10단계) — git 미커밋 상태, 커밋은 사용자 지시 대기. See [[safety-critical-path]], [[workspace-layout]].
