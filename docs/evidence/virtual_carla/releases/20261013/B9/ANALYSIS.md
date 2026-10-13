# B9 V7 완료 관측: 실제 0.015/0.035 m profile과 hard guard 유지

<!-- HH_261012 - Finalized physical roundtrip; actual profile/status and sampled guard evidence. -->

[trial.json](trial.json.gz)은 **`COMPLETED_OBSERVED`**, `roundtrip_verified=true`, `formal_acceptance=false`다. [supervisor.json](supervisor.json)은 `finished`, trial/capture 종료 코드 **0/0**; [capture manifest](visual/capture_manifest.json)는 `PASS`다. [V5 최초 centering timeout](../archive/site_completion_20261012_v5/B9_attempt02/ANALYSIS.md), [V6 recenter 10회 제한 실패](../archive/site_completion_20261012_v6/B9/ANALYSIS.md)는 그대로 보존한다. 이번 한 번의 CARLA 완료는 실제 로봇 인증이나 반복 성공률을 의미하지 않는다.

- 실제 주행 **2026-10-12 15:05:19.971–15:12:00.949 KST**; 캡처 후처리 완료 **15:13:03.113 KST**.
- Actor **316**, mission generation **1791785015292001**, delivery / 기존 **turnaround** 유지.
- 총 **401.002762 s / 215.956555 m**; outbound **139.218789 s / 53.720730 m**; return **261.566569 s / 162.235825 m**. 물리 actor body-XY **1,730개** 기록 샘플 기준이며 UI 속도 적분 거리·계획 거리와 구분한다([정의](../B4/ANALYSIS.md)).
- 최종 localization XY **(-8.64630228329763,40.50406226238638)**에서 station **(-8.47366,40.391)**까지 **0.20636965173722854 m < 변경하지 않은 native 0.25 m**. 별도 drop-zone 접근 허용치 `0.05 m`와는 다른 판정이다.
- 최종 reverse/dispatcher parking `PARKED`, site `DONE`, gate `STANDBY`, 명령 `(0,0,0)`. Collision publisher **1개**, 기록·누락·오래된 큐 이벤트 각각 **0**.

## 설정 파일이 아닌 실제 선택 profile

`camrod-sites-20261012-v7-camrod.service`의 **15:05:12–15:13:04 KST** journal에서 **15:07:40.472898** `ALIGN_RETRACE_YAW`가 `centering_tolerance_m=0.015 recenter_trigger_m=0.035`를 보고했다. 열 번의 bounded recenter 로그에도 `target_m=0.015 trigger_m=0.035`가 반복된다. 최종 기록된 `site.message`에도 `site_forward_return_centering_tolerance_m=0.015`, `site_forward_return_recenter_trigger_m=0.035`가 남아 있다. 즉 YAML 존재만이 아니라 **실제 활성 profile**을 확인했다.

새 `forward_loop precise centering lateral realign` 로그는 위 전체 구간에서 **0회**다. 따라서 V7의 B9 전용 convergence profile로 완료했지만, 이번 실행이 lateral-realign 분기 자체의 동적 실행 시험이었다고 주장하지 않는다.

## 회전·recenter와 유지된 경계

- Return-heading 시작 **15:07:40.472898** → `CRAB_OUT` **15:08:31.346350**: **50.873452 s < 90 s**.
- Bounded recenter는 **15:07:47.752262의 1/10 → 15:08:27.346998의 10/10**, 기록 trigger radial **0.036–0.040 m**. **10회 한도 자체는 늘리지 않았다.** 마지막 recovery 뒤 추가 pause 없이 목표 yaw를 확인하고 종료했으므로 한도 여유는 없었다.
- 최초 phase 시작부터 첫 recovery까지 **7.279364 s**, 이후 각 recovery 시작부터 다음 recovery 또는 `CRAB_OUT`까지 최대 **4.799348 s**다. 이 구간은 centering과 뒤따르는 회전을 모두 포함하는 상한이며 각 centering **15 s**를 연장한 결과가 아니다.
- 첫 nonzero yaw 명령 샘플 **15:07:45.631869**부터 `CRAB_OUT` 직전까지 localization radial 최대는 **0.04364422643493456 m**(15:08:05.363130), 변경하지 않은 active **0.05 m guard**보다 약 **6.356 mm** 작다. 명령·pose 기록은 비동기 최대 5 Hz 샘플이므로 이 수치를 연속시간 최대치로 주장하지 않는다.

정밀 완료 목표와 proactive recenter trigger만 B9 profile의 **0.015/0.035 m**로 선택되었다. 실제 위치 hard guard **0.05 m**, 각 centering **15 s**, 전체 heading **90 s**, recovery **10회**, native 최종 parking **0.25 m**는 그대로다. Source presence만으로 안전성이나 충분한 반복 margin을 인증하지 않는다.

## 실제 정방향 도로 복귀

단계는 `IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다.

- **15:08:38.847298–38.847954**: live lanelet handoff lateral error **0.14 m**, original-anchor error **0.15 m**. Planner가 `forward_one_way`, source `camping_site_maneuver_controller:done_live_lanelet:site_policy_roadside_forward`를 인증했다.
- **15:08:38.977241**: `start_ll=2136`, `goal_ll=751`, `mode=one_way`, `reverse_source=roadside_forward`, heading error **2.9°**, **23 lanelets / 156.82 m** 복귀 계획.

최종 `return_published=true`, `return_ack=true`. 기록된 gate sequence에 `SAFETY_HOLD`는 없다. 실제 station parking까지 완료했으며 계획 생성 또는 사이트 탈출만을 PASS로 집계하지 않았다.

## 실제 시각 증거

[대표 PNG](visual/representative_contact_sheet.png)를 직접 확인했다. B9 선택, 도로 주행·사이트 내 회전·복귀·최종 주차/서비스 화면을 원본 그대로 보존했다. 번역 팝업도 편집하지 않았다. [대표 GIF](visual/representative_motion.gif), [capture manifest](visual/capture_manifest.json)에 파생 파일·해시가 있다. 캡처 **406.4 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되었다. 시각 자료는 위 telemetry 및 journal과 함께 해석한다.
