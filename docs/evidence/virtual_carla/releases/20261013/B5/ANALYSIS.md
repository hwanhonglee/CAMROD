# B5 V5 완료 관측

<!-- HH_261012 - Record only finalized evidence from the unchanged V5 run. -->

[trial.json](trial.json.gz)의 결과는 `COMPLETED_OBSERVED`, `roundtrip_verified=true`이다. [supervisor.json](supervisor.json)은 `finished`, trial/capture 종료 코드는 모두 `0`, [capture manifest](visual/capture_manifest.json)는 `PASS`이다. 이는 CARLA 진단 시나리오의 완료 관측이며 `formal_acceptance=false`이다. 전체 사이트 또는 실제 로봇의 안전 인증을 뜻하지 않는다.

- 실제 주행: 2026-10-12 **12:42:52.144–12:49:12.860 KST**. 캡처 후처리까지 12:50:14.472 KST에 종료되었다.
- CARLA actor `44`, mission generation `1791776563087001`, `delivery`, 기존 서비스 방식 `turnaround` 유지.
- 총 **380.746 s / 215.389545 m**, outbound **154.229 s / 66.533315 m**, return **226.314 s / 148.856230 m**. 거리는 1,638개 기록 샘플에 기반한 물리 actor XY 이동거리이며, UI 누적 거리나 계획 경로 길이와 다르다.
- 최종 localization XY `(-8.66077952623058, 40.496693416500314)`. 고정 station `(-8.47366, 40.391)`과의 유클리드 거리는 **0.21490652709553965 m**, 변경하지 않은 native 최종 주차 허용치 **0.25 m** 이내이다.
- 최종 reverse parking/dispatcher `PARKED`, site `DONE`, gate `STANDBY`. `/carla/ego_vehicle/collision` publisher 1개 발견, 기록된 collision event **0**, 누락/오래된 큐 이벤트도 0이다. 이것만으로 미관측 접촉까지 부정하지 않는다.

## 두 번째 사이트 내 회전 및 전진 복귀 증거

기록된 순서는 `IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다. 첫 180도 서비스 회전을 없애지 않았으며, 사용자 Return 이후 두 번째 사이트 내 회전으로 진입 당시 진행 방향을 복원한 뒤 crab exit와 정방향 도로 루프를 수행했다.

`camrod-sites-20261012-v5-camrod.service`의 **12:42:52–12:49:14 KST** 범위 journal과 trial 상태를 교차 확인했다. 주요 journal 시각은 다음과 같다.

| 시각 KST | 실제 관측 |
| --- | --- |
| 12:45:27.635 | `ALIGN_RETRACE_YAW`, `return_route_policy=forward_loop`, centering tolerance `0.010 m` |
| 12:45:32.426 / 38.226 / 43.126 / 48.026 / 54.126 | `pivot paused` recovery **1/10 → 5/10**, 각 trigger error 약 `0.025–0.026 m`; 남은 yaw target 및 전체 phase deadline 보존 |
| 12:46:00.326 | `CRAB_OUT`: 사이트 안에서 outbound yaw 확인 완료 |
| 12:46:08.126 | `DONE`: live lanelet handoff `(17.48,4.41)`, projection `(17.37,4.36)`, lateral error `0.11 m` |
| 12:46:08.132 | planner가 `mode=forward_one_way`, source `camping_site_maneuver_controller:done_live_lanelet:site_policy_roadside_forward` 인증 |
| 12:46:08.252 | `start_ll=2192`, `goal_ll=751`, `mode=one_way`, `reverse_source=roadside_forward`, heading error `2.8°`, **21 lanelets / 143.97 m** 복귀 경로 생성 |

두 번째 회전 단계의 journal 경과는 약 **32.690 s**로 전체 90 s 한도 이내이고, 최종 상태에도 recenter **5/10**이 남아 있다. 최초 precenter 샘플의 center error는 `0.069886882 m`였다. 최초 precenter를 제외한 실제 회전 및 중간 recenter 샘플에서 catalog center `(20.1356,5.56877)`와 localization XY의 최대 거리는 **0.026773872 m**로 `0.05 m` guard 이내였다. 이는 이산 샘플의 최대치이며 연속시간 최대치라는 주장은 아니다. 마지막 회전 샘플의 yaw error는 `2.41°`; 상태에 기록된 마지막 precise-center 복원 error는 `0.008 m`였다. 장애물 삭제, 비용 정지 우회, 최종 주차 허용치 확장은 하지 않았다.

이 B5 완료는 이전 short-return cost hold와 구분되는 새 실행 증거다. 과거 실패 기록은 그대로 보존하며, 같은 frozen profile의 다른 사이트 결과를 이 결과로 대신하지 않는다.

## 실제 시각 증거

[대표 PNG](visual/representative_contact_sheet.png)를 직접 확인했다. 출발, 사이트 진입, 물리 도로 복귀와 최종 주차/서비스 화면이 포함되어 있다. [대표 GIF](visual/representative_motion.gif), [capture manifest](visual/capture_manifest.json)에 파생 파일과 해시가 있다. 캡처 길이는 **386.4 s**이며 원본 MP4는 설정에 따라 파생 파일 생성 후 제거되었으므로 존재하지 않는 동영상에 링크하지 않는다.
