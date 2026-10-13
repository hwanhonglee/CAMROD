# B4 V5 — 7회 재중심 후 forward-loop 왕복·최종 주차 완료 관측

<!-- HH_261012 - Report the completed physical roundtrip with exact bounds, distinct approach/final parking errors, immutable prior failures, and no formal/hardware acceptance claim. -->

## 최종 결과

[trial](trial.json.gz): **COMPLETED_OBSERVED**, **`roundtrip_verified=true`**, `formal_acceptance=false`. actor **28**, 2026-10-12 **03:33:05~03:39:40 UTC**, wall **394.764초**, 위치 표본 누적 거리 **216.713m**, 표본 **1,703개**.

[supervisor](supervisor.json)는 **03:40:42 UTC**에 종료됐고 probe/capture exit 모두 **0**이다. [영상 manifest](visual/capture_manifest.json)도 최종 **PASS**다. 최종 서비스 **DROP_ZONE_WAIT**, reverse controller와 parking dispatcher 모두 **PARKED**, cmd_vel 모든 성분 **0**, gate **STANDBY**, platform drive-enable=false를 관측했다. 충돌 publisher/subscriber가 발견된 상태에서 기록된 충돌 이벤트 **0건**이며, 모든 접촉 부재를 보장하는 수치는 아니다.

실제 영상 증거: [PNG contact sheet](visual/representative_contact_sheet.png), [GIF motion](visual/representative_motion.gif). PNG를 직접 열어 outbound·사이트 내부·짐을 싣고 forward-loop 복귀·Drop Zone 최종 대기 화면을 확인했다. 성긴 대표 장면만으로 모든 recovery tick을 입증하지 않으며, 아래 수치·상태 판정은 trial과 실제 노드 로그에 근거한다.

## 사이트 내 두 번째 회전: 7/10 recovery로 완료

실제 단계는 `ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다. [V4](../archive/site_completion_20261012_v4/B4/ANALYSIS.md)의 recovery 4회 한도 소진을 보존하고, V5에서 최대 회수를 **4→10**으로 변경한 후보를 실제 시험했다. **회전·진행 중 재중심 위치 guard 0.05m, 재중심 목표 0.01m, 각 재중심 15초, 전체 heading 준비 90초**는 유지했다.

실제 사이트 노드 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012_v5/camping_site_maneuver_controller_node_3317019_1791775918958.log`):

| 사건 | UNIX 시각 s | 시작 중심 거리 m (노드 반올림 값) |
| --- | ---: | ---: |
| heading 준비 시작 | 1791776152.881448 | — |
| recovery 1/10 | 1791776158.802220 | 0.026 |
| recovery 2/10 | 1791776163.400239 | 0.026 |
| recovery 3/10 | 1791776168.200458 | 0.026 |
| recovery 4/10 | 1791776172.500251 | 0.029 |
| recovery 5/10 | 1791776176.700369 | 0.027 |
| recovery 6/10 | 1791776181.000792 | 0.026 |
| recovery 7/10 | 1791776186.201326 | 0.026 |
| heading 완료·CRAB_OUT | 1791776193.500591 | — |
| lanelet handoff·DONE | 1791776201.400958 | — |

heading 준비 전체 **40.619초**로 90초 이내이며, 일곱 번 recovery 동안 phase deadline을 다시 시작하지 않았다. 반복 centering timeout·위치 guard·회수 한도 오류는 없었다.

고정 B4 중심 **(16.3537, −1.41168)m**에 대한 첫 return 표본의 오차는 **0.199286m**였다. 이는 첫 서비스 회전 뒤의 **초기 재중심 admission** 구간으로, 기존 0.65m envelope 안이며 회전 중 0.05m guard 구간과 혼동하지 않는다. 이후 첫 yaw 명령 표본(wall **171.816초**)은 중심 오차 **0.005504612m**였다. 이때부터 마지막 heading 준비 표본까지의 최대 오차는 **0.031330878m**, 즉 0.05m 안이다. 표본 최대값은 모든 physics tick의 최대값과 동일하지 않다.

마지막 heading 준비 상태는 목표 −67.43°, 현재 −64.30°, 남은 오차 **−3.13°**의 표본이며, 이후 노드가 heading 완료를 확인하고 CRAB_OUT으로 전환했다. DONE 이후 도로 곡선에서 현재 yaw가 바뀌는 것은 정상적인 road return이며, 이전 heading 완료 판정의 오차로 재해석하지 않는다.

## 기존 yaw 문제와 짧은 복귀 경로 안전 정지

entry yaw 정렬은 완료했다. 이번 nominal 시작 오차는 **3.82°**로, 10/08 baseline (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_regression_20261008/B4/ANALYSIS.md`)의 6.31°와 다르다. 따라서 동일 초기 조건의 A/B 재현이라고 주장하지 않는다. 후속 body-yaw 정렬에서 실제 작은 yaw PI 작동을 관측했고, entry 구간 **30개** 표본(wall **127.998~134.737초**)의 외부 wheel torque 절댓값 최대는 **0.788Nm**, 실제 PI 로그 적분 항 절댓값 최대는 **0.330Nm**였다.

[10/12 첫 프로필 B4](../archive/site_completion_20261012/B4/ANALYSIS.md)는 yaw 통과 뒤 짧은 복귀 경로에서 `dynamic_front_path:fusion` 안전 정지가 지속됐다. **이번 fixed forward-loop 경로의 전체 trial에는 SAFETY_HOLD 표본이 0개**이며 도로 복귀와 주차를 완료했다. 이는 새 경로의 실제 완료 증거이지, 이전 짧은 경로의 fusion 물체 분류 근본 원인이 해결됐다는 증거는 아니다. 이전 실패를 삭제·성공으로 재분류하거나 감지·정지 임계값을 무시하지 않았다.

## 최종 주차 오차: 접근점과 station을 구분

최종 ROS pose는 **(−8.6560160420, 40.5237896108)m**다.

| 항목 | 목표·기준 | 관측 오차 | 적용 허용치 |
| --- | --- | ---: | ---: |
| Drop Zone 사전 접근점 정렬 | drop-zone controller의 snapped parking approach point | **0.047m** (진단 반올림) | **0.05m** |
| 최종 reverse 주차 | station **(−8.47366, 40.391)m** | **0.225581043m** (XY 거리) | 기존 **0.25m** |

station 좌표·축은 [drop_zones.yaml](../../../../../../camrod_bringup/config/map/drop_zones.yaml), 기존 reverse 허용치는 [parking.yaml](../../../../../../camrod_control/config/parking.yaml)의 `station_axis_tolerance_m=0.25`다. 최종 XY 거리는 허용치 안에 약 **24.419mm** 여유가 있다. **0.047m 접근점 오차를 최종 station 주차 오차로 제시하지 않는다.** 최종 노드 판정도 `station XY goal reached`, reverse/dispatcher `PARKED`였다.

## 거리 수치 정의: UI 적분 거리와 물리 body-XY 합은 다르다

<!-- HH_261012 - Explain distinct distance definitions and clock domains without rewriting evidence or treating UI mileage as certified physical distance. -->

대표 UI 화면에서 읽은 약 **227.3m**와 trial의 **216.713m**를 같은 계측량으로 취급하지 않는다. 저장된 service metrics DB (LOCAL_ARCHIVE: `/home/hong/camrod_ws/state/virtual_carla/site_completion_20261012_v5/camrod/service_metrics.sqlite3`)를 SQLite `mode=ro`로 조회한 B4 최종 `completed` record(`svc-20261012T033318-9347f6a7`)는 **227.937939227667m**다. 화면의 단일 snapshot과 durable 최종 record 차이의 정확한 갱신 시점은 이 자료만으로 확정하지 않으며, 227.3m를 최종 DB 값이라고 전사하지 않는다.

| 수치 | 실제 계산 정의 |
| --- | --- |
| **216.713311084026m** | [PhysicalDistance.observe](../../../../../../tools/probe_carla_site_roundtrip.py)의 원자적 CARLA physics `body_transform.location_m` 연속 XY 위치 차이 `Σhypot(Δx,Δy)`. 최대 5Hz 표본, 같은 world frame은 중복 가산하지 않음. |
| **227.937939227667m** | [ServiceMetricsTracker.observe_velocity](../../../../../../camrod_ui/runtime/python/camrod_ui/service_metrics.py)의 `/platform/status` 평면 속도 크기 `hypot(vx,vy)`를 `velocity.header.stamp` 간격으로 사다리꼴 적분. 실제 설정은 최소 속도 0.03m/s, 최대 3m/s, 최대 표본 간격 2초. |

실제 B4 platform telemetry 1,910개 표본 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/state/virtual_carla/site_completion_20261012_v5/mission_records/2026-10-12/001_B4_delivery_fd8cbd00/telemetry_000001.jsonl`)을 위 UI 공식으로 읽기 전용 재계산하면 **227.93793922766665m**로 DB와 정확히 일치한다. 유효하지 않은 간격으로 제외된 표본은 없었다. 설정값은 [실제 live-profile dump](../archive/site_completion_20261012_v5/live_profile.json)의 `/ui_backend` 값과 확인했다.

시계 경로도 다르다. [CARLA feedback 설정](../../../../../../camrod_carla_adapter/config/feedback_bridge.yaml)은 `stamp_with_reception_time=true`, [adapter launch](../../../../../../camrod_carla_adapter/launch/adapter.launch.py)는 `use_sim_time=false`다. [feedback bridge](../../../../../../camrod_carla_adapter/src/camrod_carla_adapter/feedback_bridge_node.py)는 CARLA 속도값은 유지하되 stamp를 수신 wall/ROS clock으로 다시 찍어 `/odom`으로 내보낸다. [platform bridge](../../../../../../camrod_platform/src/ranger_platform_bridge_node.cpp)가 이를 `/platform/status.velocity`로 전달하고 [UI backend](../../../../../../camrod_ui/runtime/python/camrod_ui/ui_backend_node.py)가 해당 stamp로 적분한다. 저장된 실제 첫 sample stamp **1791775998.143664s**도 수신 epoch 시각이며, 이 CARLA 실행의 속도는 **시뮬레이션 platform telemetry**이지 실물 SocketCAN mileage가 아니다.

같은 trial의 physics 평면 속도 표본을 비교 재계산하면, **wall 시간 간격**으로는 **228.185330m**, **CARLA frame×0.05초** 간격으로는 **216.270492m**다. 후자는 body-XY 합 216.713311m에 가깝고 전자는 UI 적분 227.937939m에 가깝다. trial wall은 **394.764초**, 기록된 누적 simulation 시간은 **374.050초**다. 따라서 **시뮬레이션 속도에 수신 wall 시간을 곱하는 시계 차이가 약 5.18% 거리 차이의 주된 설명**이다. 나머지 작은 차이는 표본 주기·구간 경계·속도 cutoff·곡선의 위치차 합과 속도 적분 차이를 포함하며, 개별 오차 성분까지 분리한 것은 아니다.

최종 DB와 trial의 차이는 **11.224628m (trial 대비 5.17948%)**다. 어느 값을 다른 값으로 덮어쓰거나 두 값을 같은 인증 거리라고 주장하지 않는다. 이 문서의 216.713m는 계속 **secondary sampled physical body-XY distance**이며 formal mileage certification이 아니다. UI·번역 팝업·capture는 관측된 그대로 보존했고 source/runtime는 수정하지 않았다.

## 소스·물리 증거와 범위

- trial physical gate SHA-256: `155733157023793752263a8201894ff55b7ccba4d45c3aa67fa6239c4009d727`.
- [frozen inputs](../archive/site_completion_20261012_v5/frozen_inputs.json): 영상 최종화 후 read-only `check` 결과 **unchanged=true**, differences=[]였다.
- 사이트 노드 로그 SHA-256: `dc67257e23ef3ac3f0387ee55caab0846fd496a375ba98bf8f5136c1d8ff58ad`.
- PI controller 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012_v5/python3_3316260_1791775915224.log`) SHA-256: `43189e174c4c7110d1fb1ab367ce2e2a0d47c1411bc058e6957e1d10beb0c3d4`.

Ranger 물리 controller가 바뀌지 않은 같은 서버의 raw physical receipt를 source-bound V5 gate에서 재사용한 것이며, 새 raw 시험을 했다고 주장하지 않는다. 이번 결과는 **고정 V5 프로필의 B4 한 차례 실제 왕복 완료 관측**이다. 나머지 12개 사이트·반복 신뢰성·하드웨어·formal acceptance까지 완료됐다는 뜻이 아니다. 이 분석자는 읽기 전용 관측과 최종 문서 작성만 수행했으며 frozen source/runtime·차량 API·안전 설정은 변경하지 않았다.
