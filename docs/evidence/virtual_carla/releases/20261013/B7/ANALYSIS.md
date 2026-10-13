# B7 V5 attempt02 완료 관측

<!-- HH_261012 - Finalized trial, supervisor, controller journal and actual capture evidence only. -->

[trial.json](trial.json.gz): **`COMPLETED_OBSERVED`**, `roundtrip_verified=true`, `formal_acceptance=false`. [supervisor.json](supervisor.json)은 `finished`, trial/capture 종료 코드 모두 `0`; [capture manifest](visual/capture_manifest.json)는 `PASS`이다. 동일 frozen V5 profile의 CARLA 진단 결과이며 실제 로봇 인증을 의미하지 않는다.

- 실제 주행: **2026-10-12 13:49:05.099–13:55:23.678 KST**; 캡처 후처리 완료 **13:56:23.728 KST**.
- Actor **188**, mission generation **1791780536997001**, delivery / 기존 **turnaround** 유지.
- 총 **378.606991 s / 215.203380 m**; outbound **146.459080 s / 60.612064 m**; return **231.890415 s / 154.591317 m**. 물리 actor body-XY의 **1,631개** 기록 샘플 기준이며 UI 속도 적분 누적 거리·계획 거리와 구분한다([거리 정의](../B4/ANALYSIS.md)).
- 최종 localization XY **(-8.656070990208734, 40.53525072478929)**에서 station **(-8.47366, 40.391)**까지 **0.23255545779698997 m < 변경하지 않은 native 0.25 m**. 별도 drop-zone 접근 허용치 `0.05 m`와는 다른 판정이다.
- 최종 reverse/dispatcher parking `PARKED`, site `DONE`, gate `STANDBY`. Collision publisher **1개**, 기록 이벤트 **0**, 누락/오래된 큐 이벤트 **0**. 미관측 접촉까지 부정하는 증거는 아니다.

## 실제 복귀 동작

단계는 `IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다. `camrod-sites-20261012-v5-camrod.service`의 **13:48:12–13:56:24 KST** journal 근거:

- **13:51:32.920**: 사용자 Return 후 `ALIGN_RETRACE_YAW`, `return_route_policy=forward_loop`, center target **0.010 m**.
- **13:51:36.668–13:51:57.068**: bounded recenter **1/10 → 5/10**, trigger error **0.025–0.026 m**. 기존 yaw target과 전체 phase deadline을 보존했다. Active-turn guard **0.05 m**, 회전 단계 **90 s**, 각 centering **15 s** 한도는 변경하지 않았다.
- **13:52:03.968**: outbound yaw 확인 후 `CRAB_OUT`; 두 번째 회전 단계 약 **31.049 s**.
- **13:52:11.768–11.769**: live lanelet handoff lateral error **0.12 m**, original-anchor error **0.13 m**. Planner가 `forward_one_way`, source `camping_site_maneuver_controller:done_live_lanelet:site_policy_roadside_forward`를 인증했다.
- **13:52:11.895**: `start_ll=2192`, `goal_ll=751`, `mode=one_way`, `reverse_source=roadside_forward`, heading error **3.1°**, **21 lanelets / 149.77 m** 복귀 계획.

기록된 gate sequence는 `STANDBY → ENABLED → STANDBY → ENABLED → STANDBY`이며 `SAFETY_HOLD`는 없다. 다른 사이트의 실패·중단 기록은 이 완료 결과와 별개로 유지한다.

## 실제 시각 증거

[대표 PNG](visual/representative_contact_sheet.png)를 직접 확인했다. B7 배달 선택, 사이트 도착/Return 확인, 도로 복귀, 최종 주차/서비스 화면이 보인다. 번역 팝업을 포함해 원본 화면 그대로다. [대표 GIF](visual/representative_motion.gif), [capture manifest](visual/capture_manifest.json)에 파생 파일과 해시가 보존되어 있다. 캡처 **384.2 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되어 링크하지 않는다. 시각 자료만으로 성공을 판정하지 않고 위 telemetry와 함께 해석한다.
