# CARLA 사이트 왕복 V5 결과

<!-- HH_261012 - User-readable observed results only; update after finalized trial, supervisor and capture evidence. -->

**V5 최종 분류: 동일 고정 프로필로 13곳을 시험했고, 10곳 성공·B3/B9/B10 3곳 실패다. 전체 시나리오 성공은 아니다.**
2026-10-12 14:14 KST 기준 모든 사이트의 trial, supervisor, 영상 후처리가 종료됐다. 아래 표는 성공한 실행만 포함하며 실패 세 곳은 별도로 보존한다. 자동 집계는 summary.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/summary.json`)이다.

B3는 trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/trial.json`)의 `roundtrip_verified=false`, supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/supervisor.json`)의 probe/capture 종료 코드 **1/0**으로 실패가 확정됐다. [상세 분석](B3_attempt02/ANALYSIS.md): **179.946 s / 73.598 m** 주행 후, 두 번째 회전 전 최초 정밀 precenter가 **15.012 s**에 시간 초과했다. 마지막 중심 오차 **0.021116 m**에서 횡방향 잔차가 남아 0.01 m 목표에 수렴하지 못했다. **0.05 m pivot guard나 전체 90 s 한도 초과가 아니며**, 중간 교정 count도 0/10이다. 완료된 복귀·최종 reverse 주차가 없으므로 **최종 주차 오차나 성공 합계에 포함하지 않는다.** Capture PASS는 캡처 성공만 뜻한다. 횡방향 잔차 재확인의 [후속 수정·회귀시험 제안](B3_attempt02/BOUNDED_CENTERING_PROPOSAL.md)은 V5에 적용하지 않았다. 향후 다른 gate/profile의 결과를 이 표와 섞어 **한 프로필 13/13 성공**이라고 주장하지 않는다.

B9도 trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/trial.json`) `FAILED`, supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B9_attempt02/supervisor.json`) probe/capture **1/0**으로 확정됐다. [상세 분석](B9_attempt02/ANALYSIS.md): **154.291 s / 55.416 m** 이후 최초 centering이 **15.063 s**에 시간 초과했다. 최종 중심 오차 **0.078673 m**는 초기 admission 0.65 m 이내이며, 아직 두 번째 pivot 전이므로 active-turn 0.05 m guard 위반이 아니다. 중간 교정 count는 0/10, 완료된 return·최종 주차가 없다. 시작 후 6.140 s에 longitudinal 약 −0.0014 m인데 lateral 약 +0.0201 m가 남아 B3의 수정 후보 조건에 해당했지만, 이후 종방향 진동·drift도 관측됐다. **동일 후보로 해결될지는 실제 재시험 전까지 미확인**이다. 성공 합계에 포함하지 않는다.

B10 역시 trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/trial.json`) `FAILED`, supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B10_attempt02/supervisor.json`) probe/capture **1/0**이다. [상세 분석](B10_attempt02/ANALYSIS.md): **156.918 s / 53.276 m** 이후 **회전 중 두 번째 재중심 교정의 0.05 m 위치 guard**를 넘어 정지했다. 초기 precenter는 통과했으며, 오류 직전 중심 반경 **0.052355 m**, 최종 **0.055753 m**였다. 두 번째 교정은 약 4.3 s, 전체 heading 준비는 18.507 s이므로 B3/B9의 최초 15 s timeout과 원인이 다르다. 실제 중간 교정은 **2/10**이고 ERROR 전환 시 상태가 초기화되어 최종 count=0으로 보인다. 완료된 복귀·최종 주차가 없으며 캡처 PASS를 왕복 성공으로 집계하지 않는다. 후속 후보 조건이 관측됐어도 **B10의 별도 실제 재시험 없이 해결을 주장하지 않는다.**

## 완료된 실제 시뮬레이션 결과

아래 전부 `COMPLETED_OBSERVED`, `roundtrip_verified=true`, reverse `PARKED`이다.
각 probe/capture 종료 코드는 0/0, 실제 데스크톱 **5 fps PNG/GIF 증거 manifest는 PASS**다.
최종 오차는 station `(-8.47366, 40.391)`과 localization XY의 거리이며, 허용치는 기존 **0.25 m**다.

| 사이트·상세 분석 | 전체 시간 s | 물리 body-XY 거리 m | 복귀 시간 s / 거리 m | 최종 주차 XY 오차 m | 원본 기록 |
| --- | ---: | ---: | ---: | ---: | --- |
| [B1](../../B1/ANALYSIS.md) | 369.237 | 215.386 | 198.185 / 136.348 | 0.225141 | [trial](../../B1/trial.json.gz) |
| [B2](B2_attempt02/ANALYSIS.md) | 392.895 | 215.996 | 220.355 / 137.439 | 0.231699 | trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B2_attempt02/trial.json`) |
| [B4](../../B4/ANALYSIS.md) | 394.764 | 216.713 | 228.744 / 144.601 | 0.225581 | [trial](../../B4/trial.json.gz) |
| [B5](../../B5/ANALYSIS.md) | 380.746 | 215.390 | 226.314 / 148.856 | 0.214907 | [trial](../../B5/trial.json.gz) |
| [B6](../../B6/ANALYSIS.md) | 367.015 | 215.693 | 213.248 / 150.305 | 0.225386 | [trial](../../B6/trial.json.gz) |
| [B7](../../B7/ANALYSIS.md) | 378.607 | 215.203 | 231.890 / 154.591 | 0.232555 | [trial](../../B7/trial.json.gz) |
| [B8](../../B8/ANALYSIS.md) | 373.202 | 215.555 | 229.735 / 156.997 | 0.231757 | [trial](../../B8/trial.json.gz) |
| [B11](../../B11/ANALYSIS.md) | 323.298 | 209.620 | 215.404 / 163.851 | 0.221056 | [trial](../../B11/trial.json.gz) |
| [B12](../../B12/ANALYSIS.md) | 321.670 | 209.833 | 215.082 / 166.077 | 0.233476 | [trial](../../B12/trial.json.gz) |
| [B13](../../B13/ANALYSIS.md) | 320.968 | 209.764 | 221.275 / 170.785 | 0.227488 | [trial](../../B13/trial.json.gz) |

완료 10건의 물리 표본 거리 합은 **2,139.153 m**다. **실물 로봇 검증 주행거리나 인증 km가 아니다.**
기록된 충돌 이벤트는 모두 0건이며 publisher 발견도 확인했다. 이것이 미관측 접촉까지 없었음을 보장하지는 않는다.
사이트별 상세 분석에 outbound/return 구간, 상태 전이, 실제 [PNG 예시](../../B4/visual/representative_contact_sheet.png)·[GIF 예시](../../B4/visual/representative_motion.gif)와 manifest 링크가 있다. 원본 MP4는 파생 파일 생성·검증 후 정상 제거되었으며 존재하지 않는 영상에 링크하지 않는다.

## 무엇을 바꿨고 어디에 적용되는가

**사이트마다 설정을 바꾸지 않았다.** [고정 프로필](profile.sh), [실제 노드 파라미터 PASS](live_profile.json), [3,663개 frozen 입력](frozen_inputs.json)이 공통이다. 기존 실물 CAMROD 기본값을 일괄 변경한 것이 아니라, CARLA 사이트 프로필의 명시적 선택과 그 선택을 처리하는 분기를 추가·조정했다.

1. **작은 제자리 회전 명령의 yaw 응답:** CARLA Ranger의 `ExperimentalZeroTurnYawIntegral.update()` (LOCAL_ARCHIVE: `/home/hong/Downloads/ranger-carla-4ws-pipeline/ros_ws/src/carla_extended_ackermann_control/src/carla_extended_ackermann_control/experimental_zero_turn_yaw_integral.py`)와 `_zero_turn_yaw_integral_torque()` (LOCAL_ARCHIVE: `/home/hong/Downloads/ranger-carla-4ws-pipeline/ros_ws/src/carla_extended_ackermann_control/src/carla_extended_ackermann_control/carla_extended_ackermann_control_node.py`)가 작은 ZERO_TURN yaw-rate 오차에 제한된 적분 보상을 적용한다. 이 V5 전체 사이트는 PI=true, Ki=2.0이다. 기존 **2 N·m 회전 토크 상한**과 anti-windup, stale/interlock/정지 시 reset을 유지한다. 일반 [full launch](../../../../../../../camrod_carla_adapter/launch/camrod_carla_full.launch.py)는 기본 OFF이며 [사이트 wrapper](../../../../../../../camrod_carla_adapter/launch/camrod_carla_develop_site_geometry.launch.py)만 명시적으로 ON이다.

2. **B1–B10의 복귀 경로:** [CARLA 전용 catalog](../../../../../../../camrod_carla_adapter/config/camping_sites_carla_forward_return.yaml)의 `return_route_policy: forward_loop`로 기존 turnaround 서비스 회전·적재 대기를 유지한 뒤, Return 승인 후 **사이트 안에서 진입 당시 진행 방향으로 다시 회전 → crab exit → 정방향 도로 루프**를 사용한다. [사이트 controller](../../../../../../../camrod_control/src/camping_site_maneuver_controller_node.cpp)의 `beginReturnExit()`, `beginRotationEntryCentering()`, `beginSiteForwardReturnRecentering()`, `onTimer()`가 이를 수행한다. `publishReturnRequest()`가 실제 자세·handoff 완료를 확인한 뒤 `returnRequestSource()`의 인증 marker로 기존 planner의 one-way 분기를 선택한다. catalog 기본은 `native`이며 B11–B13의 기존 roadside 정책과 battery-urgent 분기는 보존한다. 이전 짧은 복귀 경로의 fusion cost hold는 **새 경로로 완료한 것**이지, 이전 물체 분류의 근본 원인을 해결했다고 주장하는 것은 아니다.

3. **B11–B13의 마지막 주차 접근:** [`develop_site_geometry_control_params()`](../../../../../../../camrod_carla_adapter/launch/camrod_carla_develop_site_geometry.launch.py#L344)가 정확히 drop-zone 접근점 허용치 한 항목만 **0.20→0.05 m**로 생성 YAML에서 덮어쓴다. [native control.yaml](../../../../../../../camrod_control/config/control.yaml)은 **0.20 m**, [최종 reverse 주차 허용치](../../../../../../../camrod_control/config/parking.yaml)는 **0.25 m** 그대로다. 전진 루프 도착 방향에서 약 0.18–0.20 m 잔차를 실제로 줄인 뒤 회전하므로, 이번 B11–B13은 약 45 mm의 불리한 pivot 이동에도 최종 주차에 성공했다. **접근점 잔차 0.037–0.045 m를 최종 station 주차 오차로 혼동하지 않는다.** 가장 작은 이번 여유는 B12의 **16.524 mm**이며 일반적인 반복 신뢰성 보장은 아니다.

### 두 번째 사이트 회전의 고정 한계

회전 중 위치 guard **0.05 m**, 회전 전·재중심 목표 **0.01 m**, 재중심 시작 기준 **0.025 m 초과**, 중간 교정 최대 **10회**, 각 교정 **15 s**, 최초 준비부터 모든 교정을 합친 전체 단계 **90 s**다. 재교정할 때 전체 deadline이나 남은 yaw 목표를 초기화하지 않는다. 첫 precenter만 기존 **0.65 m admission 범위**를 사용하며, 회전 시작 후에는 교정도 0.05 m guard 안에서 수행한다. V4 대비 V5의 직접 변경은 교정 횟수 **4→10**이며 위치·시간·토크 한계를 넓히지 않았다. B4는 실제 7/10회 교정, 40.619 s에 두 번째 heading 준비를 마쳤다.

이번 수정/V5 실행에서 **배경 물체 삭제, 지형·도로 mesh 재편집은 하지 않았다.** 기존 승인된 CARLA 맵·물리 자산 및 사이트 프로필을 유지했다. 위험 정지를 새로 무시하거나 주차 최종 허용치를 확대하지 않았다. 기존 사이트 overlay의 범위는 live-profile과 wrapper에 명시되어 있으며, 모든 CARLA overlay가 native 설정과 같다고 주장하지 않는다.

## 거리 두 종류를 구분한다

이 표의 거리는 CARLA physics의 원자적 `body_transform.location_m`을 최대 5 Hz로 읽어 같은 frame 중복을 제외한 **연속 XY 위치 차이의 합**이다. 시간은 wall time이다. UI는 `/platform/status` 속도 크기를 **수신 wall/ROS timestamp** 간격으로 적분하므로 같은 수치가 아니다.

[B4 상세 재계산](../../B4/ANALYSIS.md)에서 물리 body-XY 합은 **216.713311 m**, UI 최종 DB는 **227.937939 m**로 **11.224628 m, 5.17948%** 차이가 난다. 화면 한 장의 약 227.3 m는 최종 DB 값과도 구분한다. 같은 physics 속도를 wall 시간으로 적분하면 228.185 m, simulation frame 시간으로 적분하면 216.270 m여서 시계 차이가 주된 설명이다. UI 값을 물리 거리로 바꾸어 쓰거나 실물 mileage로 제시하지 않았다.

## 빌드·물리 검증과 중단 기록

V5는 **동일 서버 부팅에서 Ranger 물리 계층이 바뀌지 않아** V4 restart1의 [raw 4WS 물리 검증 receipt](../../receipts/site_completion_20261012_v4_restart1/portable_live_4ws_acceptance.json)를 재사용했다. V5에서 raw 물리 시험을 새로 수행했다고 표현하지 않는다. 대신 실제 CAMROD/Ranger [빌드 receipt](../../receipts/site_completion_20261012_v5/ros_build.json), [offline receipt](../../receipts/site_completion_20261012_v5/offline_tests.json), source/runtime 결합 [새 gate](../../receipts/site_completion_20261012_v5/ranger_physical_4ws_acceptance_gate.json)를 생성·검증했다. Offline pytest **920 passed / 3 skipped**, 별도 catalog **18 passed**, targeted controller ctest PASS이며 source·artifact 재해시 검증도 PASS다. Gate SHA-256은 `155733157023793752263a8201894ff55b7ccba4d45c3aa67fa6239c4009d727`이다.

[이전 V5 B11](B11/ANALYSIS.md)은 도로 복귀 중 SIGTERM으로 중단됐고 **신호 원인은 입증되지 않았다**. 주차 단계에 도달하지 않았으므로 성공 건수·완료 주행 합계에 포함하지 않는다. 이후 동일 frozen 설정을 persistent supervisor에서 실행한 `attempt02` 결과를 사용했다. 따라서 전체 보존 기록은 **14 attempts = 성공 10 + controller 실패 3 + SIGTERM 중단 1**이고, 최신 고유 사이트 결과는 **13곳 = 성공 10 + 실패 3**이다. 이전 V1–V4 실패·준비 중단 기록도 별도 디렉터리에 보존했으며 성공으로 재분류하지 않았다.

**검증 범위는 UI 배송 출발 → 사이트 서비스 → 사용자 Return → 도로 복귀 → reverse PARKED까지다.** Recall 전체 분기, 저전압 긴급 복귀 전체 분기, AprilTag 정밀 docking·실제 충전 접점, 반복 신뢰성, 실물 로봇 검증까지 완료한 것은 아니다. Trial의 `formal_acceptance=false`를 유지한다. 이 결과 문서 작성으로 source/config/runtime를 변경하거나 push하지 않았다.
