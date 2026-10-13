# B9 V5 attempt02 실패: 두 번째 회전 전 정밀 중심 맞춤 시간 초과

<!-- HH_261012 - Preserve the finalized failure; capture PASS is not trial PASS. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/trial.json`)은 **`FAILED`**, `roundtrip_verified=false`, `formal_acceptance=false`이다. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/supervisor.json`)은 `finished`, trial 종료 코드 **1**, capture 종료 코드 **0**. Capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/visual/capture_manifest.json`)의 `PASS`는 캡처 성공만 뜻한다. 동일 frozen V5 profile의 실제 controller 실패이며, 원인이 입증되지 않은 앞선 cohort의 SIGTERM 중단과 구분한다.

- 실제 주행: **2026-10-12 14:05:24.433–14:07:58.692 KST**; 캡처 후처리 완료 **14:08:24.169 KST**.
- Actor **220**, mission generation **1791781516415001**, delivery / **turnaround**.
- 총 **154.290656 s / 55.416497 m**; outbound **137.896566 s / 53.931073 m**; 물리 actor body-XY **671개** 기록 샘플. `return_leg=null`로 완료 복귀 지표는 없다. Total − outbound 차이 **16.394090 s / 1.485424 m**는 실패 전 잔여 구간이며 완료 복귀로 집계하지 않는다. UI 속도 적분 거리는 다른 측정치다([거리 정의](../../../B4/ANALYSIS.md)).
- 최종 localization XY **(15.252560508928237, 17.347951097513768)**. Station **(-8.47366, 40.391)**까지 **33.07439557060202 m**이며 최종 parking 단계에 도달하지 않았다. 변경하지 않은 native **0.25 m** 주차 기준을 달성하지 못했다.
- 최종 site `ERROR`, reverse/dispatcher `IDLE`, gate `ENABLED`. Collision publisher **1개**, 기록 이벤트 **0**, 누락/오래된 큐 이벤트 **0**. 접촉 이벤트가 없다는 기록과 임무 실패는 별개다.

## 첫 실패 지점

`IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → ERROR`. 서비스 회전과 사용자 Return 승인은 통과했지만 **두 번째 회전 전 초기 precenter**가 완료되지 않았다. Bounded pivot recenter는 **0/10**이며 yaw **114.54°**, target **-68.22°**, 남은 yaw **177.24°**였다. `CRAB_OUT`, `DONE`, 도로 복귀 handoff 및 `return_published`/`return_ack`는 없다.

`camrod-sites-20261012-v5-camrod.service`의 **14:07:43–14:07:59 KST** journal:

| 시각 KST | 관측 |
| --- | --- |
| 14:07:43.555 | precenter 시작, current `(15.153,17.561)`, target `(15.233,17.272)`, radial 약 `0.300 m`, 최대 명령 속도 `0.120 m/s`, timeout **15 s** |
| 14:07:44.816 | lateral → `steering_settle` |
| 14:07:46.016 | `longitudinal` 진입 |
| 14:07:58.617 | `ERROR`: `forward_loop in-site heading preparation exceeded site position bound or steady centering/turn timeout` |

Embedded log 시각 기준 precenter 시작부터 ERROR까지 **15.062724 s**이다. 최종 site center `(15.2328,17.2718)` 잔차는 약 **0.078673 m**, 초기 centering admission **0.65 m** 이내다. 따라서 직접 실패 조건은 **15 s centering deadline**이며, 아직 시작하지 않은 두 번째 pivot의 **0.05 m** guard 또는 전체 **90 s** deadline이 아니다. 제한은 완화하지 않았다.

## B3와 공통점 및 다른 점

기록된 localization XY와 반올림된 yaw로 body 축 잔차를 재계산하면, **14:07:49.693918 KST**(precenter 시작 후 **6.140112 s**)에 longitudinal **-0.001388 m**, lateral **+0.020132 m**, radial **0.020179 m**였다. Lateral 단계를 완료 처리한 뒤 longitudinal만 보정해서는 `0.010 m` radial target에 도달할 수 없는 상태가 실제로 있었다. 기존 15 s deadline까지 약 **8.860 s**가 남았으며, 제안된 full-target lateral 재확인 조건을 만족한 샘플은 **4개**다.

그러나 B9는 그 이후 **longitudinal 진동/잔차 증가도 동반**했다. 마지막 물리 샘플의 근사 body-longitudinal/lateral 잔차는 **-0.075704 / +0.049325 m**, radial **0.090355 m**였다. Longitudinal 구간의 물리 lateral drift 약 **39.218 mm**도 관측되었다. 마지막 샘플의 실제 body-longitudinal 속도는 약 **-0.2832 m/s**, 명령은 **-0.12 m/s**, FL external drive torque는 약 **-1.744 Nm**였다. 따라서 B3와 공통된 횡방향 latch 문제는 보이지만, 같은 후보 수정이 B9를 실제로 완료시킨다고 단정하지 않는다.

후속 후보는 optional `forward_loop` 정밀 centering에서 radial 미완료, longitudinal stage, `abs(longitudinal) ≤ 0.010 m`, `abs(lateral) > 0.010 m`일 때만 lateral 단계를 다시 확인하는 것이다. 이 V5 결과에는 적용되지 않았다. Deadline·yaw target·recovery count를 초기화하거나 torque/허용치를 확대하지 않는 테스트와 **B9 실제 재검증**이 필요하다.

## 보존된 실패 시각 증거

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/visual/representative_contact_sheet.png`)를 직접 확인했다. 출발, B9 진입, 사이트 내 정지 및 마지막 `Operator Stopped` 화면을 보존했다. 마지막 화면은 실패 후 상태이며 위 controller timeout의 원인을 operator stop으로 바꾸지 않는다. 번역 팝업도 원본 그대로다. 대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/visual/capture_manifest.json`)에 파생 파일과 해시가 있다. 캡처 **160.2 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되었다. 이 캡처를 성공 증거로 집계하지 않는다.
