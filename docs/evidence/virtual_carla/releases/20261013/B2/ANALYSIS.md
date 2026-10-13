# B2 V6 완료 관측: 회귀 실행 통과, 새 lateral realign 분기 0회

<!-- HH_261012 - Separate scenario completion under the new binary from actual new-branch coverage. -->

[trial.json](trial.json.gz)은 **`COMPLETED_OBSERVED`**, `roundtrip_verified=true`, `formal_acceptance=false`다. [supervisor.json](supervisor.json)은 `finished`, trial/capture 종료 코드 **0/0**; [capture manifest](visual/capture_manifest.json)는 `PASS`다. [V5 B2](../archive/site_completion_20261012_v5/B2_attempt02/ANALYSIS.md)에 이어 V6 binary에서도 동일 시나리오를 완료했다. 그러나 이번 B2 실행의 새 lateral realign marker는 **0회**이므로, 새 분기의 실제 보정 성공을 입증하는 시험으로 분류하지 않는다.

- 실제 주행 **2026-10-12 14:43:22.095–14:49:51.199 KST**; 캡처 후처리 완료 **14:50:53.528 KST**.
- Actor **300**, mission generation **1791783794049001**, delivery / **turnaround**.
- 총 **389.133823 s / 216.251340 m**; outbound **172.031062 s / 78.687330 m**; return **216.962828 s / 137.564010 m**. 물리 actor body-XY **1,678개** 기록 샘플 기준이며 UI 속도 적분 거리·계획 거리와 구분한다([정의](../B4/ANALYSIS.md)).
- 최종 localization XY **(-8.663301008021557,40.504278719909706)**에서 station **(-8.47366,40.391)**까지 **0.22089766931277074 m < 변경하지 않은 native 0.25 m**. 별도의 drop-zone 접근 허용치 `0.05 m`와 혼동하지 않는다.
- 최종 reverse/dispatcher parking `PARKED`, site `DONE`, gate `STANDBY`, 명령 `(0,0,0)`. Collision publisher **1개**, 기록·누락·오래된 큐 이벤트 각각 **0**. 미관측 접촉까지 부정하는 증거는 아니다.

## 실제 return-heading과 도로 복귀

단계는 `IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다. `camrod-sites-20261012-v6-camrod.service`의 **14:42:28–14:50:54 KST** journal을 조회했다.

- **14:46:15.337853**: 사용자 Return 후 `ALIGN_RETRACE_YAW`, `forward_loop`, center target **0.010 m**.
- **14:46:23.715683–14:46:46.515378**: bounded recenter **1/10 → 6/10**, trigger error **0.025–0.028 m**. Active guard **0.05 m**, 각 centering **15 s**, 전체 heading **90 s**를 변경하지 않았다.
- **14:46:53.915401**: outbound yaw 확인 후 `CRAB_OUT`. Heading phase **38.577548 s**.
- **14:47:01.715892–01.716844**: live lanelet handoff lateral error **0.17 m**, original-anchor error **0.18 m**; planner가 `forward_one_way`, source `camping_site_maneuver_controller:done_live_lanelet:site_policy_roadside_forward`를 인증했다.
- **14:47:01.842548**: `start_ll=2193`, `goal_ll=751`, `mode=one_way`, `reverse_source=roadside_forward`, heading error **3.4°**, **20 lanelets / 132.34 m** 복귀 계획 생성.

위 전체 journal 구간의 `forward_loop precise centering lateral realign` count는 **0**이다. V6 B9/B10에서는 실제 분기 실행이 관측됐지만 두 실행 모두 다른 경계에서 실패했다([B9](../archive/site_completion_20261012_v6/B9/ANALYSIS.md), [B10](../archive/site_completion_20261012_v6/B10/ANALYSIS.md)). B2 완료로 그 실패를 덮어쓰거나 V6 전체 사이트 완료를 주장하지 않는다. 기록된 gate sequence에도 `SAFETY_HOLD`는 없다.

## 실제 시각 증거

[대표 PNG](visual/representative_contact_sheet.png)를 직접 확인했다. B2 배달 선택, 도로 주행·사이트 진입·복귀·최종 주차와 서비스 화면이 보인다. 번역 팝업을 포함해 원본 그대로다. [대표 GIF](visual/representative_motion.gif), [capture manifest](visual/capture_manifest.json)에 파생 파일·해시가 있다. 캡처 **394.4 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되었다. 시각 자료만으로 성공을 판단하지 않고 위 telemetry 및 journal과 함께 해석한다.
