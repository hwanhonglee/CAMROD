# 사이트 13곳 배송·복귀·주차 관측 완료 — 버전별 표적 회귀 결과

<!-- HH_261012 - Keep versioned full-cohort, targeted regression, and pre-trial infrastructure evidence separate; never infer a single-profile all-site pass. -->

**B1–B13 각 사이트의 배송→사이트 서비스→전진 복귀→reverse 주차 완료를 실제 관측했다.** 마지막 B10은 **V9_restart1**에서 2026-10-12 **17:01:31 KST** `PARKED`, **17:02:32 KST** 영상 후처리까지 완료했다. 선택한 13건 모두 `COMPLETED_OBSERVED`, `roundtrip_verified=true`, trial/capture **0/0**이며 최종 station XY는 **0.206370–0.233476 m < 기존 0.25 m**다.

다만 **동일한 최종 V9 소스로 13곳 전부를 재주행한 결과는 아니다.** 아래 선택은 **V5 9곳 + V6 2곳 + V7 1곳 + V9_restart1 1곳**이다. [V5 전수](archive/site_completion_20261012_v5/RESULTS.md) 10성공·3실패 이후 실패 사이트와 B2를 표적 재시험한 결과이며, V5/V6/V7/V8의 실패·중단은 그대로 보존한다. Repeatability, 실차 및 recall/docking 전체 acceptance로 확대하지 않는다.

최종 B10은 **CARLA B10만 교정 한도 10→20회**로 바꾸고 실제 **13/20회**를 사용했다. Active 위치 **0.05 m**, 교정별 **15 s**, 전체 heading **90 s**, 최종 주차 **0.25 m**는 유지했다. GPU 장애로 중단된 최초 V8/V9 bootstrap은 실제 mission 전의 infrastructure 사건이며 controller 실패로 집계하지 않는다([V9 서버 사건](archive/site_completion_20261012_v9/SERVER_RESTART.md)).

## 사이트별 실제 확인 상태

아래는 사이트마다 선택한 **실제 성공 실행 1건**이다. 시간은 wall seconds, 거리는 physical body-XY 표본 경로 길이이며 UI 속도 적분 값이 아니다. 오차는 고정 station `(-8.47366,40.391)` 기준 최종 localization XY 거리다. 각 분석에서 outbound/return 세부 지표와 trial·supervisor·PNG/GIF·manifest를 확인할 수 있다.

| 사이트 | 실제 버전 | 전체 s / body-XY m | 복귀 s / body-XY m | 최종 주차 XY m | 증거 |
| --- | --- | ---: | ---: | ---: | --- |
| B1 | V5 | 369.237 / 215.386 | 198.185 / 136.348 | 0.225141 | [분석](B1/ANALYSIS.md) · [PNG](B1/visual/representative_contact_sheet.png) · [GIF](B1/visual/representative_motion.gif) |
| B2 | V6 | 389.134 / 216.251 | 216.963 / 137.564 | 0.220898 | [분석](B2/ANALYSIS.md) · [PNG](B2/visual/representative_contact_sheet.png) · [GIF](B2/visual/representative_motion.gif) |
| B3 | V6 | 397.796 / 215.958 | 234.483 / 143.186 | 0.227135 | [분석](B3/ANALYSIS.md) · [PNG](B3/visual/representative_contact_sheet.png) · [GIF](B3/visual/representative_motion.gif) |
| B4 | V5 | 394.764 / 216.713 | 228.744 / 144.601 | 0.225581 | [분석](B4/ANALYSIS.md) · [PNG](B4/visual/representative_contact_sheet.png) · [GIF](B4/visual/representative_motion.gif) |
| B5 | V5 | 380.746 / 215.390 | 226.314 / 148.856 | 0.214907 | [분석](B5/ANALYSIS.md) · [PNG](B5/visual/representative_contact_sheet.png) · [GIF](B5/visual/representative_motion.gif) |
| B6 | V5 | 367.015 / 215.693 | 213.248 / 150.305 | 0.225386 | [분석](B6/ANALYSIS.md) · [PNG](B6/visual/representative_contact_sheet.png) · [GIF](B6/visual/representative_motion.gif) |
| B7 | V5 | 378.607 / 215.203 | 231.890 / 154.591 | 0.232555 | [분석](B7/ANALYSIS.md) · [PNG](B7/visual/representative_contact_sheet.png) · [GIF](B7/visual/representative_motion.gif) |
| B8 | V5 | 373.202 / 215.555 | 229.735 / 156.997 | 0.231757 | [분석](B8/ANALYSIS.md) · [PNG](B8/visual/representative_contact_sheet.png) · [GIF](B8/visual/representative_motion.gif) |
| B9 | V7 | 401.003 / 215.957 | 261.567 / 162.236 | 0.206370 | [분석](B9/ANALYSIS.md) · [PNG](B9/visual/representative_contact_sheet.png) · [GIF](B9/visual/representative_motion.gif) |
| B10 | V9 restart1 | 409.820 / 217.426 | 272.260 / 164.685 | 0.221309 | [분석](B10/ANALYSIS.md) · [PNG](B10/visual/representative_contact_sheet.png) · [GIF](B10/visual/representative_motion.gif) |
| B11 | V5 | 323.298 / 209.620 | 215.404 / 163.851 | 0.221056 | [분석](B11/ANALYSIS.md) · [PNG](B11/visual/representative_contact_sheet.png) · [GIF](B11/visual/representative_motion.gif) |
| B12 | V5 | 321.670 / 209.833 | 215.082 / 166.077 | 0.233476 | [분석](B12/ANALYSIS.md) · [PNG](B12/visual/representative_contact_sheet.png) · [GIF](B12/visual/representative_motion.gif) |
| B13 | V5 | 320.968 / 209.764 | 221.275 / 170.785 | 0.227488 | [분석](B13/ANALYSIS.md) · [PNG](B13/visual/representative_contact_sheet.png) · [GIF](B13/visual/representative_motion.gif) |

선택한 13건의 원시 값 합은 **4,827.260 s(약 80.45분) / 2,788.749 m**다. Outbound 합은 **1,859.790 s / 788.668 m**, return 합은 **2,965.148 s / 2,000.081 m**이며 시간 구간 경계 때문에 outbound+return 시간이 전체 시간과 조금 다르다. 서로 다른 실행을 더한 값이지 연속 주행 시간이 아니다. **시뮬레이션 표본 거리 약 2.789 km**로, 실차 거리·UI 적분 거리·10 km 인증으로 표현하지 않는다.

실제 데스크톱 캡처는 모두 **5 fps**다. 선택한 성공 사례들과 아래 보존된 실패 사례들을 혼합 집계하지 않는다. 실패 실행의 **capture PASS는 영상 생성 성공일 뿐 임무 성공이 아니다.**

[최종 13곳 독립 감사](FINAL_13_SITE_AUDIT.md)도 같은 원시 합계를 확인했다. 13개 trial의 status·actor identity·reverse/dispatcher `PARKED`·gate `STANDBY`, collision publisher 발견 및 0건을 확인했고, 13개 실제 capture 검증과 **65개 보존 파일 SHA-256** 대조가 모두 통과했다. 이는 과거 기록을 현재 binary와 동일하다고 인증한 것이 아니다.

### 마지막 B10의 실제 완료 근거

[V9_restart1 B10 분석](B10/ANALYSIS.md)은 실제 profile **center 0.015 m / trigger 0.035 m / dwell 0.400 s / limit 20**을 확인했다. 기존 10회를 넘는 **11·12·13번째 교정**을 실제 사용하고 **58.940187 s**에 사이트 내부 heading 복원을 완료했다. 새 lateral-realign은 **2회**였으며, 교정과 뒤따르는 yaw를 포함한 구간 최대 **7.901026 s < 15 s**, 첫 yaw 이후 표본 최대 반경 **0.046313096 m < 0.05 m**였다. 약 **3.687 mm** 표본 여유는 연속시간 안전 보증이 아니다.

이후 실제 live-lanelet handoff로 **23 lanelets / 158.52 m** 전진 경로를 승인받았고 **17:01:31.355308 KST** reverse `PARKED`가 기록됐다. 최종 station 오차 **0.221308763 m**는 별도 approach 잔차 **0.042 m**와 구분한다. [Supervisor](B10/supervisor.json)는 **0/0**, [capture](B10/visual/capture_manifest.json)는 **415.4 s / 5 fps PASS**다. 위 결과는 이 실행의 완료 증거이지 반복 신뢰성 보증이 아니다.

### 보존된 실패·표적 회귀 이력

V9 이전 **V8_restart1 B10 실패**도 그대로 남긴다. 실제 trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/trial.json`)은 **190.248 s / 54.221 m**, outbound **137.778 s / 52.907 m**, `roundtrip_verified=false`와 복귀·최종 주차 미완료를 기록한다. 실제 profile **0.015/0.035 m·0.400 s**와 lateral-realign 4회가 관측됐지만 pivot recenter **10/10회** 한도에 도달했다. Supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/supervisor.json`)는 trial/capture **1/0**, capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/capture_manifest.json`)는 **195.8 s·5 fps PASS**다. 실제 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/representative_contact_sheet.png`)·GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/representative_motion.gif`)는 실패 실행의 영상 증거이며 주차 성공 영상이 아니다.

[V8_restart1 확정 상세 분석](archive/site_completion_20261012_v8_restart1/B10/ANALYSIS.md)에서 전체 heading 단계는 **51.213 s**, 첫 yaw 이후 표본 최대 반경은 **0.045088 m < 0.05 m**였다. 따라서 직접 종료는 **10회 소진**이며 15 s 교정 timeout·90 s 전체 timeout·위치 guard 초과가 아니다. 15개 실제 dwell 전환은 **0.400001–0.500480 s**, 같은 실행의 최초 native service-turn dwell은 **1.200036 s**였다. 모든 교정 후 yaw가 재개됐지만 **65.32°**가 남아 도로 복귀에 도달하지 못했다. 표본상 약 **4.912 mm** guard 여유는 연속시간 안전 보증이 아니며, 이후 회전 진척을 외삽해 성공으로 계산하지 않는다.

[독립적인 기존 12곳 증거 재검사](archive/site_completion_20261012_v8/PRIOR_SITE_AUDIT.md)는 V5 9곳·V6 2곳·V7 1곳의 trial 완료, supervisor 0/0, reverse/dispatcher `PARKED`, gate `STANDBY`, collision publisher 발견 및 0건을 확인했다. 기존 보존 시각자료 **60개 파일의 SHA-256**과 12개 capture 검증도 통과했다. 이는 과거 증거의 구조·해시 재검사이지 V8에서 12곳을 다시 주행하거나 새로 영상을 촬영한 결과가 아니다.

### V6 확정 완료 지표

| 실제 실행 | 전체 시간 s / 물리 body-XY m | 복귀 시간 s / 거리 m | 새 lateral-realign 직접 관측 | 증거 |
| --- | ---: | ---: | --- | --- |
| B3 | 397.796 / 215.958 | 234.483 / 143.186 | 0회 | [trial](B3/trial.json.gz) · [분석·영상](B3/ANALYSIS.md) |
| B2 | 389.134 / 216.251 | 216.963 / 137.564 | 0회 | [trial](B2/trial.json.gz) · [분석·영상](B2/ANALYSIS.md) |

B3는 probe/capture **0/0**, 5 fps capture PASS, reverse `PARKED`, 기록된 collision 0건이다. 두 번째 회전은 **47.535 s, 기존 pivot recenter 9/10회**로 완료했다. 다만 **새 `lateral realign` 분기는 이 실행에서 0회**였다. 따라서 새 binary의 B3 왕복 완료는 확인했지만, 새 분기가 물리적으로 작동해 V5 실패를 해결했다는 직접 인과 증거로 해석하지 않는다. 단위 테스트와 실제 분기 실행 증거도 구분한다.

B2도 probe/capture **0/0**, 5 fps capture PASS, reverse `PARKED`, 기록된 collision 0건이다. 두 번째 heading 준비 **38.578 s**, 기존 recenter **6/10회**였고 새 lateral-realign은 **0회**였다. 이는 새 binary의 회귀 호환성 관측이지 새 분기의 실제 실행 증거는 아니다.

**V6 B9 실패:** **220.645 s / 55.093 m** 주행했고 완료된 복귀·주차는 없다. [실제 lateral-realign 11회](archive/site_completion_20261012_v6/B9/ANALYSIS.md)(초기 1회, pivot recovery 2/9/10에서 4/2/4회)를 확인했다. 초기 precenter는 통과했지만 이후 기존 **10/10회 pivot recenter 한도**를 소진했다. 전체 heading 단계 **80.417 s**, 남은 yaw **48.93°**로 90 s timeout이 아니다. 첫 yaw 이후 표본 최대 반경도 **0.032527 m < 0.05 m**여서 위치 guard 초과와 구분한다. V5 B9의 초기 15 s timeout과 다른 실패다. Supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/supervisor.json`)는 trial/capture **1/0**, capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/visual/capture_manifest.json`)는 PASS다. 새 분기가 실행됐다는 사실만으로 전체 회전·복귀·주차 해결을 주장하지 않는다.

**V6 B10도 실패:** [상세 분석](archive/site_completion_20261012_v6/B10/ANALYSIS.md)과 trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/trial.json`)은 **173.304 s / 53.719 m** 후 **세 번째 recenter의 15.000 s timeout**을 확인했다. 새 lateral-realign은 모두 세 번째 교정에서 **4회** 실행됐다. 첫 yaw 이후 표본 최대 반경 **0.039614 m < 0.05 m**, 최종 반경 **0.019982 m**로 V5의 guard 초과는 이번에 재현되지 않았지만, 목표 0.01 m에 수렴하지 못했다. 전체 heading 단계는 **33.518 s**로 90 s 한도 초과도 아니다. `return_leg=null`; supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/supervisor.json`)는 trial/capture **1/0**, 캡처만 PASS다. 실제 분기 실행과 일부 실패 조건 개선을 전체 복귀·주차 해결로 표현하지 않는다.

### V7 확정 완료 지표

| 실제 실행 | 전체 시간 s / 물리 body-XY m | 복귀 시간 s / 거리 m | 최종 주차 XY m | 증거 |
| --- | ---: | ---: | ---: | --- |
| B9 | 401.003 / 215.957 | 261.567 / 162.236 | 0.206370 | [trial](B9/trial.json.gz) · [분석·영상](B9/ANALYSIS.md) |

V7 B9는 probe/capture **0/0**, reverse `PARKED`, 기록된 collision 0건이다. 실제 상태·로그에서 **0.015/0.035 m profile**을 확인했다. Heading 준비 **50.873 s**, recenter **10/10회**, 첫 yaw 이후 표본 최대 반경 **0.043644 m < 0.05 m**다. 횟수 한도 여유는 없었고 위치 여유도 표본 기준 약 **6.356 mm**이므로 반복 신뢰성을 보장하지 않는다. 새 lateral-realign은 **0회**였다. B10의 완료를 대신하지 않으며 이전 V5/V6 실패도 그대로 보존한다.

**V7 B10 실패:** [확정 분석](archive/site_completion_20261012_v7/B10/ANALYSIS.md)은 **175.276 s / 53.458 m**, 완료된 복귀·주차 없음, probe/capture **1/0**을 기록한다. 실제 선택값은 **0.015/0.035 m**였고 네 번째 recenter에서 target 0.015 m의 lateral-realign이 **4회** 실행됐다. 그러나 해당 교정의 **15.000 s timeout**으로 종료됐다(전체 phase 36.363 s). 첫 yaw 이후 표본 최대 반경 **0.042414 m < 0.05 m**, 최종 **0.021485 m**로 hard guard는 지켰지만 목표 안에 수렴하지 못했다. ERROR 시 profile/count가 기본값으로 clear되므로 최종 0.010/0.025·count0을 실행 중 profile 오선택으로 해석하지 않는다. **당시 V7에서는 B10이 미해결이었고, 이후 V9_restart1에서 완료했다.**

## 수정 내용과 적용 범위

V5까지 적용한 CARLA 전용 변경은 [상세 보고서](archive/site_completion_20261012_v5/RESULTS.md)에 보존했다.

- **작은 제자리 yaw 응답:** `ExperimentalZeroTurnYawIntegral.update()`와 `_zero_turn_yaw_integral_torque()`의 제한된 PI 보상(`HH_261008`). 사이트 wrapper에서는 ON, Ki=2.0이며 일반 full launch의 기본은 OFF다. 기존 회전 토크 2 N·m 상한과 정지·stale·interlock reset을 유지한다.
- **정방향 복귀:** [CARLA catalog](../../../../../camrod_carla_adapter/config/camping_sites_carla_forward_return.yaml)의 B1–B10 `return_route_policy: forward_loop`와 사이트 controller의 `beginReturnExit()`, `beginSiteForwardReturnRecentering()`, `publishReturnRequest()` 등(`HH_261012`). 기존 서비스 180도 회전·적재 확인 후 사이트 안에서 진입 진행 방향을 복원하고 도로의 정방향 루프로 복귀한다. Native 기본 정책과 B11–B13 roadside 정책은 유지한다. 이전 short-return fusion 정지 경로를 새 경로로 완료한 것이며, 기존 물체 분류 원인을 해결했다고 확대 해석하지 않는다.
- **마지막 주차 접근:** [`develop_site_geometry_control_params()`](../../../../../camrod_carla_adapter/launch/camrod_carla_develop_site_geometry.launch.py#L344)의 생성 YAML 한 항목만 접근점 허용치 **0.20→0.05 m**로 조인다(`HH_261008`). Native control.yaml은 0.20 m, 최종 reverse 주차 기준은 0.25 m 그대로다. 접근점 오차를 최종 station 주차 오차로 제시하지 않는다.

### V6: 횡방향 잔차를 다시 확인하는 제한된 보정

V5 B3/B9는 lateral 완료를 기억한 채 longitudinal만 보정하다가, 남아 있는 횡방향 잔차 때문에 0.01 m 반경 목표에 수렴하지 못하고 초기 centering 15 s 한도에 도달했다. B10은 초기 centering을 통과했지만 **두 번째 회전 중 교정**에서 위치가 0.05 m 밖으로 나가 정지했다. B10의 직접 실패는 시간 초과가 아니며, ERROR 이후 표시 count=0은 실제 2/10회를 초기화한 값이다.

[최종 후보 설계](archive/site_completion_20261012_v5/B3_attempt02/BOUNDED_CENTERING_PROPOSAL.md)에 따라 production 변경은 [사이트 controller](../../../../../camrod_control/src/camping_site_maneuver_controller_node.cpp)의 **`publishRotationEntryCentering()` 한 함수**에 한정한다(`HH_261012`, English source comments). 아래를 **모두** 만족할 때만 기존 lateral 단계로 돌아간다.

1. 명시적 `forward_loop` episode와 정밀 centering이 모두 활성 상태이고 현재 stage가 `longitudinal`이다.
2. Fresh finite pose·guard·gain·speed를 사용하고 기존 radial 성공 조건을 아직 만족하지 않았다.
3. `abs(longitudinal) <= precise_target`이고 **`abs(lateral) > precise_target`**이다. 현재 target은 `min(0.01, 0.2 × pivot_guard)`이므로 **전체 0.01 m**가 trigger이며 폐기한 0.005 m trigger가 아니다.
4. 횡잔차가 기존 sequencer의 latch 한계 이내다: `min(native_transition_tolerance, 0.5 × precise_target) + native_hysteresis`, 현재 약 **0.105 m**. 이 수치는 새 이동 허용 범위가 아니며, active pivot의 **0.05 m 위치 guard가 계속 우선**한다. 기존 latch 한계 밖 오류도 우회하지 않는다.

재설정하는 것은 **sequencer stage와 내부 stage timestamp만**이다. XY/yaw 목표, mission 승인, 최초 centering 시작 시각, 전체 단계 시작 시각, 교정 횟수는 보존한다. 기존 radial 성공이 먼저 판정되며, native/urgent 경로와 sequencer 공용 helper는 바꾸지 않는다. 별도 parameter·사이트별 예외, 양축 동시 명령, 추가 회전 명령을 넣지 않는다.

V6에서 유지한 제한은 **pivot guard 0.05 m / precise target 0.01 m / pivot 재중심 trigger 0.025 m 초과 / 최대 10회 / 교정별 15 s / 전체 90 s**다. 기존 steering settle 1.2 s, 초기 precenter admission 0.65 m도 유지했다. 위 실주행 결과처럼 초기 단계 일부는 개선됐지만 B9/B10의 전체 회전·복귀는 완료하지 못했다.

### V7 — B9/B10 전용 profile 적용 후 B9 성공·B10 실패

V6 종료 후 적용한 V7은 **CARLA B9/B10의 해당 mission만** 정밀 중심 목표를 **0.010→0.015 m**, pivot 재중심 trigger를 **0.025→0.035 m**로 선택하게 한다. [CARLA catalog](../../../../../camrod_carla_adapter/config/camping_sites_carla_forward_return.yaml)의 해당 두 사이트에만 `forward_return_centering_tolerance_m: 0.015`, `forward_return_recenter_trigger_m: 0.035`를 둔다. 사이트 controller의 `configureSiteForwardReturnProfile()`이 출발 전 profile을 검증하고, `siteForwardReturnRecenterTriggerM()`과 기존 target getter가 mission-local 값을 사용하며 `clearSiteForwardReturn()`은 기본값을 복원한다(`HH_261012`, English source comments).

이는 내부 목표·trigger의 실제 변경이며 그대로 유지했다고 표현하지 않는다. 다른 사이트 기본 목표 0.01 m·trigger 0.025 m, native/urgent 동작은 유지하는 범위다. **위치 guard 0.05 m, 교정 15 s, 전체 90 s, 최대 10회, 최종 reverse 주차 0.25 m**는 바꾸지 않았다. 실제 빌드·테스트·새 gate는 통과했지만 위 주행 결과처럼 B9만 완료했고 B10 실패는 남았다.

### V8 — B10 전용 steering dwell 후보, 실제 재시험 실패

[관측 근거와 후보 범위](archive/site_completion_20261012_v8/CANDIDATE.md)에 따라 **CARLA B10의 명시적 forward-return mission에만** `forward_return_steering_settle_s: 0.4`를 선택한다. Lateral 교정 후 wheel을 0도로 복원하며 zero command를 유지하는 dwell을 **1.2→0.4 s**로 줄이는 변경이다. V7 B10에서는 이 대기 중 실제 body 횡속도 약 0.007–0.010 m/s와 횡잔차 재증가가 관측됐다. 한 실행의 wheel/body 표본에서 얻은 후보이며 모든 상황의 settling 시간이나 하드웨어 안전 인증은 아니다.

`siteForwardReturnSteeringSettleS()`가 mission-local 선택값을 읽고 `configureSiteForwardReturnProfile()`이 유한한 양수·native dwell 이하인지 기존 profile 조건과 함께 검증한다. `publishRotationEntryCentering()`에서 선택값을 사용하며 mission 종료·취소·오류 시 기존 clear 경로로 기본 동작을 복원한다(`HH_261012`, English source comments). **B9·기타 사이트와 native omitted-field dwell 1.2 s는 유지**한다. B10 target/trigger **0.015/0.035 m**, active hard guard **0.05 m**, 교정별 **15 s**, 전체 **90 s**, 최대 **10회**, 최종 주차 **0.25 m**도 그대로다. 실제 0.400 s 활성값은 관측됐지만 전체 heading·복귀·주차를 완료하지 못했으므로 이 후보를 해결 완료로 표시하지 않는다.

### V9 — B10 전용 최대 교정 횟수 변경, 실제 13회 사용 후 완료

V8에서 매 교정 뒤 같은 방향의 yaw 진척은 있었으나 10회 뒤에도 65.32°가 남았다. V9은 **CARLA B10에만** `forward_return_recentering_limit: 20`을 추가한다. `siteForwardReturnRecenteringLimit()`와 `beginSiteForwardReturnRecentering()`이 mission-local 한도를 사용하고 `configureSiteForwardReturnProfile()`이 정수 **1–20**을 검증한다(`HH_261012`, English source comments). 다른 사이트의 omitted-field **10회** 및 native 경로는 유지한다.

**B10 교정 횟수는 10→20회로 변경**하지만 위치 hard guard **0.05 m**, 교정별 **15 s**, 전체 heading **90 s**는 유지한다. Target/trigger **0.015/0.035 m**, dwell **0.4 s**, 최종 reverse 주차 **0.25 m**도 V8과 같다. 이번에는 실제 13회·58.940187 s로 heading을 완료한 뒤 도로 복귀·주차까지 확인했다. 이 실제 결과를 V8 진척률의 외삽이나 모든 반복 실행의 90 s 내 완료 보장으로 바꾸어 표현하지 않는다.

V9_restart1은 GPU 장애 이후 [server launcher](../../../../../scripts/virtual_carla/run.sh#L91)에 추가된 선택 옵션 **`CAMROD_CARLA_DISABLE_GPU_PROFILER=true`**도 사용한다. 렌더링을 끄는 것이 아니라 **GPU statistics timestamp 수집만 `r.GPUStatsEnabled 0`으로 끄는 설정**이며, 같은 화면 렌더링·30 FPS·원본 맵을 유지한다. `camrod-sites-20261012-v9-r1-server.service` journal에서 **16:50:54 KST** command line과 실제 `r.GPUStatsEnabled = 0`을 확인했다. 옵션 unset/false의 기존 인자 동작을 보존하는 17개 테스트를 추가했다. 이는 관측된 profiling 경로를 피하려는 infrastructure 조치로서 **GPU/driver의 근본 원인 해결이나 장기 안정성을 입증하지 않는다**. 최초 V9의 actor 0·`ABSENCE_VERIFIED`와 trial 없음은 controller 실패와 분리한다.

## 버전별 빌드·테스트·물리 gate

| 구분 | 실제 확인된 검증 상태 |
| --- | --- |
| V5 | [빌드 PASS](receipts/site_completion_20261012_v5/ros_build.json), [offline PASS](receipts/site_completion_20261012_v5/offline_tests.json): pytest 920 passed / 3 skipped, catalog 18 passed, targeted controller ctest PASS. [실제 파라미터 PASS](archive/site_completion_20261012_v5/live_profile.json), [3,663개 frozen 입력](archive/site_completion_20261012_v5/frozen_inputs.json). |
| V5 gate | [source-bound gate](receipts/site_completion_20261012_v5/ranger_physical_4ws_acceptance_gate.json) SHA-256 `155733157023793752263a8201894ff55b7ccba4d45c3aa67fa6239c4009d727`. 변경 없는 동일 서버 부팅의 기존 [raw 4WS receipt](receipts/site_completion_20261012_v4_restart1/portable_live_4ws_acceptance.json)를 재사용하되 CAMROD 빌드·offline·source 결합 gate는 새로 확인했다. |
| V6 | [실제 빌드 PASS](receipts/site_completion_20261012_v6/ros_build.json), [offline PASS](receipts/site_completion_20261012_v6/offline_tests.json): focused GTest **54 passed**(9개 추가, 실제 XML 14:20:37 KST), pytest **920 passed / 3 skipped**, 별도 [catalog+harness 24 passed](receipts/site_completion_20261012_v6/verification/initial/catalog_harness.xml). 마지막 24개는 freeze 이후 저장한 추가 실제 receipt이며 기존 gate artifact가 아니고 gate도 변경하지 않았다. [20개 노드 live 파라미터 PASS](archive/site_completion_20261012_v6/live_profile.json), [3,662개 frozen 입력](archive/site_completion_20261012_v6/frozen_inputs.json). |
| V6 gate | [새 source-bound gate](receipts/site_completion_20261012_v6/ranger_physical_4ws_acceptance_gate.json) SHA-256 **`07d38ccc60e4920340107021b0a2c97a849e6b4919c4c7d03ec7e28fcfe10c16`**, deep validation PASS. 동일 raw 4WS receipt를 재사용하지만 V6 controller source·binary와 실제 새 build/offline receipt를 결합한 별도 gate다. |
| V7 | retry02 [실제 빌드 PASS](receipts/site_completion_20261012_v7/ros_build.json), [offline PASS](receipts/site_completion_20261012_v7/offline_tests.json): focused GTest **65 passed**(실제 XML 15:01:25 KST), pytest **920 passed / 3 skipped**, 별도 [catalog+harness 37 PASS](receipts/site_completion_20261012_v7/verification/initial/catalog_harness.xml). [20개 노드 live PASS](archive/site_completion_20261012_v7/live_profile.json), [3,662개 frozen 입력](archive/site_completion_20261012_v7/frozen_inputs.json). |
| V7 gate | [별도 source-bound gate](receipts/site_completion_20261012_v7/ranger_physical_4ws_acceptance_gate.json) VERIFIED·deep validation PASS, SHA-256 **`feb3de0673729a54b3272492222e345d9302071a7503f59496e9e047ac7ed26b`**. V7 source SHA `1effa3655c4f52b593fee624c371fc6fbed7f63ce445675a7d46556c5c36ebb3`, binary SHA `bbdb656e9790c2c76fd03c6ab6257b0ca8ebe5b3163a1d645259026d39d8163b`, catalog SHA `01384c7eec6d0e876857a1cb2552f4a258325738ab9d60a1416331dd9c92e5aa`는 build receipt에 결합되어 있다. |
| V8_restart1 | [새 빌드 PASS](receipts/site_completion_20261012_v8_restart1/ros_build.json), [새 offline PASS](receipts/site_completion_20261012_v8_restart1/offline_tests.json): focused GTest **71 passed**(실제 XML 15:46:12 KST), pytest **920 passed / 3 skipped**, 별도 [catalog+harness retry01 50 PASS](receipts/site_completion_20261012_v8_restart1/catalog_harness_retry01.xml). [20개 노드 live PASS](archive/site_completion_20261012_v8_restart1/live_profile.json), [3,662개 frozen 입력](archive/site_completion_20261012_v8_restart1/frozen_inputs.json). |
| V8_restart1 gate | [새 source-bound gate](receipts/site_completion_20261012_v8_restart1/ranger_physical_4ws_acceptance_gate.json) VERIFIED, SHA-256 **`c783494c146eda0e616df668d98f256d206d354e4dd42ac3c0ee221e5acee840`**. 새 서버 PID **3526820**에서 [새 raw 물리 acceptance PASS](receipts/site_completion_20261012_v8_restart1/portable_live_4ws_acceptance.json)(15:47:35–15:47:40 KST, 8개 fresh-actor 방향 시험 + 2개 동일 actor 정지 전환)을 결합했다. Raw SHA **`c87d87f719c545a634c66fff7ec3316f39e6497f6c30926a2f5d1f0aa39cc68b`**이며 이전 서버 raw receipt를 재사용한 gate가 아니다. |
| V9_restart1 | [새 빌드 PASS](receipts/site_completion_20261012_v9_restart1/ros_build.json), [새 offline PASS](receipts/site_completion_20261012_v9_restart1/offline_tests.json): focused GTest **78 passed**(실제 XML 16:49:29 KST), pytest **937 passed / 3 skipped**(GPU profiler 옵션 17개 포함), 별도 [catalog+harness 63 PASS](receipts/site_completion_20261012_v9_restart1/catalog_harness.xml), [3,662개 frozen 입력](archive/site_completion_20261012_v9_restart1/frozen_inputs.json), [20개 노드 live PASS](archive/site_completion_20261012_v9_restart1/live_profile.json). 실제 B10 복귀·주차와 capture 0/0도 확정됐다. 실행 담당 root는 완료 후 freeze UNCHANGED·gate deep validation PASS를 다시 확인했다. |
| V9_restart1 gate | [새 source-bound gate](receipts/site_completion_20261012_v9_restart1/ranger_physical_4ws_acceptance_gate.json) VERIFIED·deep validation PASS, SHA-256 **`00fd65560116523478830575f50cad7ebc5ba07b693b750b58d5620cb87a795e`**. 새 서버 PID **3581434**의 [새 raw 물리 acceptance PASS](receipts/site_completion_20261012_v9_restart1/portable_live_4ws_acceptance.json)(16:51:40–16:51:44 KST)를 결합했다. Raw SHA **`38e6a239b82c3719a4f8a9ab2b2abff9ec2a477cb11fe217db3272ba3913cdf4`**이며, 중단된 최초 V9의 이전 서버 raw 재사용 계획과 다르다. |

V7 초기 [C++ 실행 64 PASS / 1 FAIL](receipts/site_completion_20261012_v7/verification/initial/control_tests.log) 기록도 보존한다. Direct YAML load fixture의 timeout=0을 90 s로 설정하고 malformed-profile 거부 검사를 강화한 뒤 retry02에서 통과했다. 이 테스트 보정에서 production code는 바꾸지 않았으며 초기 실패를 삭제하거나 처음부터 PASS였다고 표시하지 않는다.

V8_restart1의 [첫 catalog+harness 실행 49 PASS / 1 FAIL](receipts/site_completion_20261012_v8_restart1/catalog_harness.xml)도 보존한다. 복제한 harness에 남아 있던 `_restart1` 경로 금지 assertion이 새 경로와 충돌한 기록이며 retry01에서는 50개 모두 통과했다. 이 기록을 실제 B10 임무 실패로 세지 않는다. V8_restart1 build receipt의 controller source SHA는 `f4e6eccf9d5e5be0b5cd3a9762cdcaa9644ce2a73182a2d9588d7cb682b9ea17`, binary SHA는 `8b77f27a4eb9baa0560509c379427e0edf59972605e1854c3d18aaa594f5ee7b`, catalog SHA는 `c2c370a2c3763bc13e018dece8c399b7540fcf20eb1700c829e6b83baa8192a5`다.

V9_restart1 build receipt의 controller source SHA는 `ac03a8f4d4e22f3888b7289fa635b9724dc99301d37f541dfa41bc4a78539015`, binary SHA는 `5000af6588b0458acc31933ed2dca411df233f4d776a422b36251a73dbda51a3`, catalog SHA는 `9c09a7ac10c350d75a8c4ddfdde240ed46c7c4738d271e8111625b1446c01fbc`다. V8/V9의 source·gate·서버 부팅을 서로 바꿔 쓰지 않는다.

V5/V6/V7은 기존 **CARLA 서버 PID 3296973**에서 같은 raw 물리 receipt(SHA `e0c7730e351c3f9e9338895ee873cbb61acbea2d1dfea5e0c4a2cfce12e18cb2`)를 사용하되, 변경된 CAMROD source·binary·catalog에 맞춰 각 버전 gate를 새로 검증했다. V6 source SHA는 `d79faa78b4da5983f07b281a7751a9b950f7ba5a7e9febeb4e6f0f5848c55bae`, binary SHA는 `b88995f965697fba90a3e3fd2d384797b894e268fc4d85087e4f1cccd4c61718`이며 해당 build receipt에 포함된다. **V8 준비 중 15:42:11 KST에 기존 서버가 종료**돼 V8_restart1은 위 새 서버·새 raw 검증으로 구분한다. 동일 원본 맵을 사용하며 이번 실패 수정 때문에 배경 물체를 삭제하거나 지형·도로 mesh를 바꾸지 않았다. 기존 raw 재사용과 새 부팅에서 실제 수행한 raw 시험을 혼동하지 않는다.

## 거리·실패 기록·검증 한계

V5 성공 10건의 **물리 body-XY 표본 거리 합은 2,139.153 m**다. CARLA actor의 같은 physics frame 중복을 제외한 연속 XY 위치 차이 합이며 **시뮬레이션 진단 거리**다. 실물 10 km 검증이나 인증 mileage가 아니다. UI는 platform 속도를 수신 wall/ROS timestamp 간격으로 적분하므로 같은 수치가 아니다. [B4 실측 비교](B4/ANALYSIS.md)는 body-XY **216.713311 m**, UI 최종 DB **227.937939 m**로 5.17948% 차이를 보였고, simulation 시간과 수신 wall 시간의 차이가 주된 설명이다.

[이전 V5 B11 SIGTERM 중단](archive/site_completion_20261012_v5/B11/ANALYSIS.md)은 원인이 입증되지 않은 별도 기록이다. 완료 결과로 계산하지 않는다. V5 보존 기록은 **14 attempts: 성공 10, controller 실패 3, SIGTERM 중단 1**이며 최신 고유 사이트 결과는 **13곳: 성공 10, 실패 3**이다. 이전 실패·중단 증거를 덮어쓰거나 V6 성공으로 재분류하지 않는다.

검증 범위는 **UI 배송 → 사이트 서비스 → 사용자 Return → 전진 도로 복귀 → reverse PARKED**다. Recall 전체 분기, 저전압 긴급 복귀 전체 분기, AprilTag 정밀 docking·실제 충전 접점, 반복 신뢰성, 실제 로봇 검증까지 완료한 것은 아니다. `formal_acceptance=false`를 유지한다. 변경된 실제 source/helper에는 범위·이유를 설명하는 English `HH_261008`/`HH_261012` inline 주석을 두었다. 자동 Git commit/push는 수행하지 않았다.
