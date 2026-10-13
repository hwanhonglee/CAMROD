# B2 V5 attempt02 완료 관측

<!-- HH_261012 - Finalized trial, supervisor and capture evidence only. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B2_attempt02/trial.json`): **`COMPLETED_OBSERVED`**, `roundtrip_verified=true`, `formal_acceptance=false`. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B2_attempt02/supervisor.json`)은 `finished`, trial/capture 종료 코드 모두 `0`; capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B2_attempt02/visual/capture_manifest.json`)는 `PASS`이다. 동일 frozen V5 profile의 CARLA 진단 결과이며 전체 사이트 또는 실제 로봇 인증을 의미하지 않는다. 이전 cohort의 SIGTERM 원인은 입증되지 않았으며, 이 사이트는 재개 cohort의 `attempt02` 이름으로 실행되었다.

- 실제 주행: **2026-10-12 13:28:12.605–13:34:45.468 KST**; 캡처 후처리 완료 **13:35:47.756 KST**.
- Actor **140**, mission generation **1791779285440001**, delivery / 기존 **turnaround** 유지.
- 총 **392.895 s / 215.995968 m**; outbound **172.391 s / 78.557179 m**; return **220.355 s / 137.438789 m**. 거리는 물리 actor XY의 1,698개 기록 샘플에 기반하며 계획 거리나 UI 누적 거리와 구분한다.
- 최종 localization XY **(-8.653484992230196, 40.53710716185806)**. 고정 station **(-8.47366, 40.391)**과의 정확한 XY 거리 **0.23169879278236977 m < 변경하지 않은 native 0.25 m**.
- 최종 reverse parking `PARKED`, site `DONE`, gate `STANDBY`. Collision publisher 1개 발견; 기록 이벤트 **0**, 누락/오래된 큐 이벤트 **0**. 미관측 접촉까지 부정하는 증거는 아니다.

## 실제 복귀 동작

기록 순서는 `IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다. 기존 서비스 회전과 대기 후 사용자 Return을 받아 두 번째 사이트 내 회전, crab exit, 정방향 도로 복귀를 완료했다. 중간 recenter는 **7/10**이었다.

`camrod-sites-20261012-v5-camrod.service`의 **13:28:12–13:34:47 KST** 범위를 조회한 journal 근거:

- **13:31:06.256**: `ALIGN_RETRACE_YAW`, `return_route_policy=forward_loop`, center tolerance `0.010 m`.
- **13:31:11.747–13:31:42.546**: bounded recenter **1/10 → 7/10**; trigger `0.025–0.028 m`, 남은 yaw target과 전체 phase deadline 보존.
- **13:31:48.046**: outbound yaw 확인 후 `CRAB_OUT` — 두 번째 회전 단계 약 **41.790 s**, 전체 90 s 한도 이내.
- **13:31:55.847–55.849**: live lanelet handoff lateral error `0.14 m`; planner가 `forward_one_way`, source `camping_site_maneuver_controller:done_live_lanelet:site_policy_roadside_forward` 인증.
- **13:31:55.973**: `start_ll=2193`, `goal_ll=751`, `mode=one_way`, `reverse_source=roadside_forward`, heading error **0.7°**, **20 lanelets / 132.26 m** 계획 생성.

장애물 삭제, 정지/비용 우회 또는 native 최종 주차 허용치 확대 없이 완료되었다. 과거 실행의 실패·중단 증거를 이 결과로 덮어쓰지 않는다.

## 실제 시각 증거

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B2_attempt02/visual/representative_contact_sheet.png`)를 직접 확인했다. 출발·사이트 구간·복귀·최종 주차/서비스 화면을 포함한다. 대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B2_attempt02/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B2_attempt02/visual/capture_manifest.json`)에 파일과 해시가 보존되어 있다. 캡처 길이 **398.8 s**; 원본 MP4는 정상 파생 후 제거되었으므로 링크하지 않는다.
