# B6 V5 attempt02 완료 관측

<!-- HH_261012 - Finalized trial, supervisor, controller journal and actual capture evidence only. -->

[trial.json](trial.json.gz): **`COMPLETED_OBSERVED`**, `roundtrip_verified=true`, `formal_acceptance=false`. [supervisor.json](supervisor.json)은 `finished`, trial/capture 종료 코드 모두 `0`; [capture manifest](visual/capture_manifest.json)는 `PASS`이다. 동일 frozen V5 profile의 CARLA 진단 결과이며 실제 로봇 인증을 의미하지 않는다.

- 실제 주행: **2026-10-12 13:41:04.469–13:47:11.447 KST**; 캡처 후처리 완료 **13:48:11.577 KST**.
- Actor **172**, mission generation **1791780055444001**, delivery / 기존 **turnaround** 유지.
- 총 **367.014864 s / 215.693355 m**; outbound **153.548430 s / 65.388667 m**; return **213.248010 s / 150.304688 m**. 거리는 물리 actor body-XY의 **1,581개** 기록 샘플에 기반한다. UI 속도 적분 누적 거리·계획 거리와는 다른 측정치다([거리 정의](../B4/ANALYSIS.md)).
- 최종 localization XY **(-8.657961384176645, 40.52073810977498)**. 고정 station **(-8.47366, 40.391)**까지 **0.22538628471451527 m < 변경하지 않은 native 0.25 m**이다. 별도 drop-zone 접근 허용치 `0.05 m`와 혼동하지 않는다.
- 최종 reverse/dispatcher parking `PARKED`, site `DONE`, gate `STANDBY`. Collision publisher **1개**, 기록 이벤트 **0**, 누락/오래된 큐 이벤트 **0**. 미관측 접촉까지 부정하는 증거는 아니다.

## 실제 복귀 동작

단계는 `IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다. `camrod-sites-20261012-v5-camrod.service`의 **13:41:04–13:47:13 KST** journal에서 확인했다.

- **13:43:39.299**: 사용자 Return 후 `ALIGN_RETRACE_YAW`, `return_route_policy=forward_loop`, 정밀 center target **0.010 m**.
- **13:43:49.375**: bounded recenter **1/10**, trigger error **0.027 m**. 기존 yaw target과 전체 phase deadline을 보존했다. V5의 active-turn guard **0.05 m**, 회전 단계 **90 s**, 각 centering **15 s** 한도는 그대로다.
- **13:43:56.475**: outbound yaw 확인 후 `CRAB_OUT`. 두 번째 회전 단계는 약 **17.176 s**, 추가 recenter 없이 완료했다.
- **13:44:04.575–04.576**: live lanelet handoff lateral error **0.08 m**, original-anchor error **0.13 m**; planner가 `forward_one_way`, source `camping_site_maneuver_controller:done_live_lanelet:site_policy_roadside_forward`를 인증했다.
- **13:44:04.704**: `start_ll=2192`, `goal_ll=751`, `mode=one_way`, `reverse_source=roadside_forward`, heading error **3.6°**, **21 lanelets / 145.41 m** 정방향 복귀 계획.

최종 관측까지 gate sequence는 `STANDBY → ENABLED → STANDBY → ENABLED → STANDBY`이며 `SAFETY_HOLD` 기록은 없다. 이 완료 결과는 다른 사이트의 실패·중단 기록을 대체하지 않는다.

## 실제 시각 증거

[대표 PNG](visual/representative_contact_sheet.png)를 직접 확인했다. 출발, B6 진입, 도로 복귀, 최종 주차 및 서비스 화면이 함께 기록되어 있다. 화면의 번역 팝업도 원본 그대로 보존했다. [대표 GIF](visual/representative_motion.gif), [capture manifest](visual/capture_manifest.json)에 파생 파일과 해시가 있다. 캡처는 **372.8 s / 5 fps**이며 원본 MP4는 정상 파생 후 제거되어 링크하지 않는다. 시각 자료만으로 물리 actor·안전 상태·성공을 판정하지 않고 위 telemetry와 함께 해석한다.
