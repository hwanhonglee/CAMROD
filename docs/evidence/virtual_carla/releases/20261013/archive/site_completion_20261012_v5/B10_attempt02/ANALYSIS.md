# B10 V5 attempt02 실패: 두 번째 recenter 중 0.05 m 위치 guard 초과

<!-- HH_261012 - Preserve the finalized guard failure; distinguish it from B3/B9 centering timeouts. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/trial.json`)은 **`FAILED`**, `roundtrip_verified=false`, `formal_acceptance=false`이다. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/supervisor.json`)은 `finished`, trial 종료 코드 **1**, capture 종료 코드 **0**. Capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/visual/capture_manifest.json`)의 `PASS`는 캡처 성공만 뜻한다. 동일 frozen V5 profile의 실제 controller 실패다.

- 실제 주행: **2026-10-12 14:09:17.716–14:11:54.607 KST**; 캡처 후처리 완료 **14:12:19.730 KST**.
- Actor **236**, mission generation **1791781749485001**, delivery / **turnaround**.
- 총 **156.918426 s / 53.275844 m**; outbound **137.015970 s / 52.686118 m**; 물리 actor body-XY **683개** 기록 샘플. `return_leg=null`로 완료 복귀 지표는 없다. Total − outbound 차이 **19.902456 s / 0.589726 m**는 실패 전 잔여 구간이며 완료 복귀로 집계하지 않는다. UI 속도 적분 거리와는 구분한다([거리 정의](../../../B4/ANALYSIS.md)).
- 최종 localization XY **(9.005765469736549, 16.53943204880288)**. Station **(-8.47366,40.391)**까지 **29.570722150171584 m**이며 최종 parking 단계에 도달하지 않았다. 변경하지 않은 native **0.25 m** 주차 기준을 달성하지 못했다.
- 최종 site `ERROR`, reverse/dispatcher `IDLE`, gate `ENABLED`, 최종 명령 `(0,0,0)`. Collision publisher **1개**, 기록 이벤트 **0**, 누락/오래된 큐 이벤트 **0**. 접촉 이벤트가 없다는 기록과 임무 실패는 별개다.

## B3/B9와 다른 직접 실패 조건

`IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → ERROR`. B10은 **최초 정밀 precenter를 통과하고 두 번째 회전을 실제로 시작**했다. 이후 recovery **2/10**의 longitudinal 보정 중 위치 guard를 초과했다. ERROR 진입 시 내부 count가 `0`으로 clear되므로 최종 count만 보고 recovery가 없었다고 판단하면 안 된다. `CRAB_OUT`, `DONE`, 도로 복귀 handoff 및 `return_published`/`return_ack`는 없다.

`camrod-sites-20261012-v5-camrod.service`의 **14:11:36–14:11:55 KST** journal과 물리 샘플:

| 시각 KST | 관측 |
| --- | --- |
| 14:11:36.020 | Return precenter 시작, target `(9.0187,16.4852)`, radial 약 `0.283 m`, 목표 **0.010 m** |
| 14:11:40.227 | 초기 precenter `longitudinal` 단계 진입 |
| 14:11:44.181468 | 첫 yaw 명령 **-0.45 rad/s**, radial **0.003014124 m** |
| 14:11:45.328 | bounded recenter **1/10**, trigger 약 **0.028 m** |
| 14:11:50.228 | bounded recenter **2/10**, trigger 약 **0.027 m** |
| 14:11:52.827 | recovery 2 `longitudinal` 진입 |
| 14:11:54.519469 | 기록된 radial **0.052355253 m > 0.05 m** |
| 14:11:54.528 | `ERROR`: `forward_loop in-site heading preparation exceeded site position bound or steady centering/turn timeout` |

최종 site-center 잔차는 **0.055753180989914775 m**였다. ERROR는 recovery 2 시작 후 **4.300 s**, 전체 return-heading phase **18.507 s**에 발생했다. 따라서 **active recenter의 변경하지 않은 0.05 m 위치 guard**가 직접 조건이며, 각 centering **15 s**, 전체 회전 **90 s**, recovery **10회** 한도 초과가 아니다. 최초 precenter의 `0.65 m` admission envelope를 active recovery에 적용하지 않았다. 남은 yaw는 **-153.08°**였다.

## 횡방향 잔차와 후속 후보의 한계

기록된 localization XY와 반올림된 yaw로 body 축 잔차를 재계산하면, recovery 1의 **14:11:49.110084 KST**에 longitudinal **+0.004970 m**, lateral **-0.010009 m**, radial **0.011175 m**였다. Recovery 2의 **14:11:53.583036 KST**에도 longitudinal **-0.002358 m**, lateral **-0.011521 m**, radial **0.011760 m**로 lateral 완료 latch 후 정밀 radial 목표 밖에 있었다. 제안된 full-target lateral 재확인 조건을 만족한 샘플은 **두 recovery에 걸쳐 5개**다.

마지막 샘플에서는 longitudinal **-0.049101 m**, lateral **-0.018170 m**로 커졌다. Recovery 2의 실제 body-longitudinal 속도는 약 **-0.0577 → +0.1165 m/s**로 방향이 바뀌며, 단순 정지 횡잔차만이 아니라 longitudinal 진동/위치 이탈도 관측된다. Recovery 2의 첫 재확인 기회부터 guard ERROR까지 약 **0.9445 s**뿐이다. 따라서 lateral 재확인 후보가 더 일찍 개입할 가능성은 있지만, **실제 B10 완료나 guard 내 정지를 보장하는 증거는 아니다**.

V5에는 후보 수정이 적용되지 않았다. 후속 후보는 optional `forward_loop`의 longitudinal stage에서 radial 미완료, `abs(longitudinal) ≤ 0.010 m`, `abs(lateral) > 0.010 m`일 때만 bounded lateral 보정을 재개하는 범위다. 기존 위치 guard·deadline·yaw target·recovery count를 유지한 테스트 및 **B10 실제 재검증**이 필요하다. 이 실패를 시간 연장이나 guard 완화로 통과 처리하지 않는다.

## 보존된 실패 시각 증거

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/visual/representative_contact_sheet.png`)를 직접 확인했다. 출발, B10 진입, 사이트 내 정지 및 마지막 `Operator Stopped` 화면이 보존되어 있다. 마지막 화면은 실패 후 상태이며 위 controller guard 실패의 원인을 operator stop으로 바꾸지 않는다. 번역 팝업도 원본 그대로다. 대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/visual/capture_manifest.json`)에 파생 파일과 해시가 있다. 캡처 **163.2 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되었다. 이 캡처를 성공 증거로 집계하지 않는다.
