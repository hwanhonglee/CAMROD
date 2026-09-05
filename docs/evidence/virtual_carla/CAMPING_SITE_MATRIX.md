# B1~B13 캠핑 사이트 왕복 검증 매트릭스

이 문서는 `Drop Zone → B1~B13 각 사이트 → Drop Zone 주차·충전` 결과를 한곳에서
비교하는 기준표다. **ROS 2 controller simulation**, **실제 CARLA live run**, **화면
PNG/GIF**는 서로 다른 증거 등급이며 한 열의 PASS를 다른 열로 확대 해석하지 않는다.

## 현재 보유 결과

`CARLA tuned` 열은 현재 기본 명령의 성공 증거가 아니다. 2026-08-31 당시
Woraksan 보정 OSM, reverse-return, 복구·속도·경계·UI 튜닝이 넣어간 구성을
현재의 두 프로필 기준으로 재분류한 결과다. 당시 실행 명령이 현재 새로 생긴
`camrod-tuned` subcommand였다는 뜻은 아니다. `develop-parity` 열은 현재
`./scripts/virtual_carla/run.sh camrod`를 새로 실행해야만 채울 수 있다.

| 사이트 | 운용 정책 | 활성 goal-snap `(x, y, yaw°)` | 기존 ROS 2 sim | 과거 Woraksan-tuned CARLA | 현재 develop-parity CARLA |
|---|---|---|---|---|---|
| B1 | turnaround | `(22.358, -6.908, -63.25)` | PASS | **PASS (2026-08-31)** | **PENDING** |
| B2 | turnaround | `(22.455, -7.101, -63.25)` | PASS | PENDING | **PENDING** |
| B3 | turnaround | `(19.772, -1.223, -65.29)` | PASS | PENDING | **PENDING** |
| B4 | turnaround | `(19.307, -0.153, -67.43)` | PASS | PENDING | **PENDING** |
| B5 | turnaround | `(17.383, 4.336, -66.90)` | PASS | PENDING | **PENDING** |
| B6 | turnaround | `(16.786, 5.716, -66.42)` | PASS | PENDING | **PENDING** |
| B7 | turnaround | `(15.085, 9.670, -66.94)` | PASS | PENDING | **PENDING** |
| B8 | turnaround | `(14.147, 11.828, -66.92)` | PASS | PENDING | **PENDING** |
| B9 | turnaround | `(12.680, 15.667, -68.22)` | PASS | PENDING | **PENDING** |
| B10 | turnaround | `(11.891, 17.728, -69.18)` | PASS | PENDING | **PENDING** |
| B11 | roadside_stop | `(10.822, 20.516, -68.78)` | PASS + full-cycle sim PASS | PENDING | **PENDING** |
| B12 | roadside_stop | `(10.103, 23.094, -78.44)` | PASS | **PASS (2026-08-31)** | **PENDING** |
| B13 | roadside_stop | `(9.626, 27.419, -91.18)` | PASS | PENDING | **PENDING** |

Drop Zone 기준점은 `(-14.2347, 39.7863, -82.2127°)`다. 표의 goal-snap은 semantic
area 중심이 아니라 lanelet 경로가 실제로 사용하는 활성 snapped target이며
`camrod_planning/test/test_active_campsite_geometry.cpp`가 현재 map 계약으로 고정한다.

## 정책과 프로필의 차이

- B1~B10 `turnaround`:
  `CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`
- B11~B13 `roadside_stop`:
  `CRAB_IN → UNLOAD_WAIT → WAIT_RETURN → CRAB_OUT → DONE`
- develop-parity `camrod`는 develop의 BT·controller·goal source를 그대로 유지한다.
  특히 B11~B13 `roadside_stop`의 현장 zero-turn을 금지하고 develop의
  forward one-way return 정책을 유지한다.
- `camrod-tuned`는 `RPPReverse`, reverse auxiliary goal, reverse lanelet 검사와
  Woraksan 복귀 overlay를 명시적으로 켠다. 아래 B1/B12 과거 실행의
  `reverse_shortest` 결과는 이 tuned 열에만 해당한다.
- parity는 recovery breakaway, sim planning/localization/parking profile, docking rear
  fallback, Return site-exit re-arm을 끄고 실행한다. tuned matrix는 이 항목들을
  켜므로, 같은 사이트 이름이어도 parity 결과로 승계하지 않는다.

## 기존 데이터의 정확한 범위

기존 B1~B13 결과는
[`../../assets/module-guides/bringup/test-results/camping-site-full-return-20260819`](../../assets/module-guides/bringup/test-results/camping-site-full-return-20260819/)
에 있다. `camping_site_1.json`~`camping_site_13.json`, 정책 요약 PNG와 phase GIF가
모두 있으며 13/13 controller/policy simulation이 PASS다. B11에는 별도의 full service
cycle sim JSON이 있다. 이 데이터는 AMD64 ROS 2 simulation이며 CARLA Ranger actor가
실제 맵을 왕복했다는 증거가 아니다.

B12의 CARLA 자료는
[`camrod_carla_b12_round_trip_e2e_20260831.json`](camrod_carla_b12_round_trip_e2e_20260831.json)과
[`07_b12_round_trip_e2e.png`](07_b12_round_trip_e2e.png),
[`07_b12_round_trip_e2e.gif`](07_b12_round_trip_e2e.gif)다. 해당 run은 onscreen
CARLA와 production UI에서 수동주행·teleport 없이 복귀와 충전까지 완료했지만,
후속 source refresh와 develop-parity 분리 이전의 역사 tuned 결과다.

## 역사 B1 live CARLA tuned 결과

최종 보고서는
[`camrod_carla_b1_round_trip_final_20260831.json`](camrod_carla_b1_round_trip_final_20260831.json),
센서 감사는
[`camrod_carla_b1_sensor_source_audit_20260831.json`](camrod_carla_b1_sensor_source_audit_20260831.json),
화면 증거는
[`08_b1_round_trip_live_20260831.png`](08_b1_round_trip_live_20260831.png)와
[`08_b1_round_trip_live_20260831.gif`](08_b1_round_trip_live_20260831.gif)다.

| 항목 | 결과 |
|---|---:|
| 전체 결과 | PASS |
| 실행 시간 | 710.787초 / 900초 제한 |
| B1 도착 오차 | 0.094504 m |
| Drop Zone 복귀 오차 | 0.306233 m / 3.0 m 허용 |
| CARLA odometry 누적 거리 | 170.222831 m |
| `/cmd_vel` / odometry / physical sample | 13,570 / 13,462 / 20,194 |
| 최종 상태 | PARKED + CHARGING |
| 최종 속도 | 약 `7.45e-8 m/s` |
| 물리 actor/backend | actor 130 / `PHYSX_FOUR_WHEEL_STEERING` |
| wheel torque safety cap | 20 Nm |
| sensor source audit | 32/32 stream, 13/13 actor, failure 0 |
| pose teleport / fake sensor | false / false |

실제 관측 순서는 다음과 같다.

```text
MOVING_TO_SITE
→ ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180
→ UNLOAD_WAIT → WAIT_RETURN
→ ALIGN_RETRACE_YAW → CRAB_OUT → DONE
→ reverse_shortest + RPPReverse 복귀
→ ALIGN_PARKING_YAW → REVERSE_APPROACH
→ WAIT_FOR_CHARGING → PARKED → CHARGING
```

UI dispatch는 0.995초, B1 `WAIT_RETURN`은 246.374초, Return 승인은 246.390초,
Drop Zone `WAITING_FOR_CHARGING`은 699.273초, `PARKED + CHARGING`은 710.774초에
관측됐다. 동일 actor 130과 physical gate/backend/hash가 전 구간 유지됐고 실제 sensor
actor는 RGB camera 2, LiDAR 1, radar 7, GNSS 2, IMU 1로 총 13개다. 모든 실제 source
publisher는 `carla_ros_bridge`였으며 canonical UI topic은 지정 relay/filter 노드에서만
나왔다.

이 실행은 **B1만** 대상으로 했으므로 tuned B2~B11·B13은 계속 PENDING이고,
B12는 후속 수정 이전 역사 tuned PASS다. 현재 develop-parity는 B1과 B12를
포함해 **B1~B13 모두 PENDING**이다. `WAITING_FOR_CHARGING` 직후 matrix sample에서 parking
상태가 잠깐 `REVERSE_APPROACH`로 보이는 것은 비동기 observation 시점 차이며 11.501초 뒤
`PARKED + CHARGING`으로 수렴했다. 최종 campsite `DONE` 메시지의 yaw 값은 이미 B1을 떠난
뒤 남은 현장 context이므로 복귀 오차는 `drop_zone_error_m=0.306233`을 사용한다.

## 새 CARLA 매트릭스가 기록해야 하는 항목

새 live runner는 fake sensor, `/initialpose`, actor teleport와 직접 `/cmd_vel` 발행을
사용하지 않는다. production backend의 `POST /ui/destination`과
`POST /ui/manual_return`만 사용하고 다음을 사이트별로 기록한다.

- dispatch 응답과 `WAITING_FOR_RETURN_REQUEST` 도달 시간
- 실제 site phase 순서와 Return 요청 응답
- Drop Zone parking, `WAITING_FOR_CHARGING`, `CHARGING` 도달 시간
- 시작·사이트·복귀 시 `/localization/pose`와 CARLA odometry, 최종 Drop Zone 오차
- 움직임 거리와 `/control/cmd_vel` 관측
- 동일 CARLA Ranger actor ID, physical 4WS ready/gate hash의 run 중 불변성
- 실제 CARLA sensor source audit 결과와 fake/dummy publisher 부재
- 중단·실패 시 완료 사이트, 실패 milestone, 미실행 사이트

## 실행 명령

계획 확인은 완전한 read-only다. 파일·디렉터리를 만들거나 ROS/UI 명령을 보내지 않는다.

```bash
cd /home/hong/camrod_ws/src
export RANGER_CARLA_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline
./scripts/virtual_carla/run.sh camping-sites-plan
```

최신 gate가 검증되고 `server → bridge → pacer → spawn`이 실행 중일 때
먼저 검증할 프로필을 하나만 선택한다. 최신 develop 알고리즘 동등성 재검증은
다음이 기준이다.

```bash
./scripts/virtual_carla/run.sh camrod
```

2026-08-31 보정 구성을 재현하는 별도 실험에서만 다음을 쓴다.

```bash
./scripts/virtual_carla/run.sh camrod-tuned
```

그 다음 별도 terminal에서 활성 프로필을 변경하지 않고 B1~B13을 순차 실행한다.

```bash
./scripts/virtual_carla/run.sh camping-sites
```

일부 사이트를 먼저 확인하려면 같은 실행기를 환경변수로 제한한다.

```bash
CAMROD_CARLA_CAMPING_SITES=B1,B11,B12,B13 \
  ./scripts/virtual_carla/run.sh camping-sites
```

기본 milestone timeout은 사이트별 900초다. 최초 spawn은 Drop Zone 중심이 아니라
lanelet 751 위의 3.8 m 전방 안전 지점이므로 시작 전용 허용 오차는 5.0 m이고, 실제
복귀·주차의 최종 Drop Zone 허용 오차는 더 엄격한 3.0 m다. 각각
`CAMROD_CARLA_MATRIX_PHASE_TIMEOUT_S`,
`CAMROD_CARLA_MATRIX_START_DROP_ZONE_TOLERANCE_M`,
`CAMROD_CARLA_MATRIX_DROP_ZONE_TOLERANCE_M`로 설정할 수 있다. 최종 허용 오차를
늘려 실패를 숨기는 용도로 사용하지 않는다.

`/service/state`는 event-driven volatile 토픽이라 bringup 이후 observer가 붙으면 최초
`DROP_ZONE_WAIT` 한 건을 놓칠 수 있다. 첫 dispatch에 한해서만 이 경우를 허용하며,
대신 physical 4WS actor/gate, `STANDBY` 제어 gate, CARLA 정지 속도(0.05 m/s 이하),
Drop Zone-front spawn 위치를 동시에 요구한다. dispatch 이후와 두 번째 사이트부터는
서비스 상태 전이를 기존과 동일하게 필수로 관측한다.

## 생성 데이터

실행마다 다음 새 디렉터리를 만들며 기존 결과를 덮어쓰지 않는다.

```text
$RANGER_EVIDENCE_ROOT/camrod_camping_site_matrix/<UTC>/
├── sensor_source_audit.json
└── camping_site_matrix.json
```

`sensor_source_audit.json`은 주행 직전 실제 CARLA source/UI 36개 stream과 CARLA sensor
actor 13개의 감사 결과를 묶는다. 성공했다면 fake/dummy publisher가 없음을
보여주지만, 그 결과만으로 mission/parity PASS가 되지는 않는다. matrix 보고서는 각
milestone마다 atomic 갱신되므로 긴 전체 실행을 Ctrl+C로 중단해도 완료 사이트와 마지막
관측을 보존한다. 오류나 actor/gate identity 변화 시 `/ui/stop`을 호출하고 이후 사이트는
`NOT_ATTEMPTED`로 둔다.

B12 역사 실행은 약 14분이었으므로 13개 전체는 맵 경로와 주차 시간에 따라 수 시간이
걸릴 수 있다. console은 사이트 번호와 dispatch, WAIT_RETURN, Return, 주차/충전 시점을
즉시 출력한다. PNG/GIF는 별도
[`capture_ui_evidence.sh`](../../../scripts/virtual_carla/capture_ui_evidence.sh)로 실제
CARLA/UI 창을 동시에 녹화해야 하며, matrix JSON의 UTC milestone과 맞춰 파생물을 만든다.
B1~B13 사이트별 장시간 캡처는
`--capture-fps 1 --retain-source-video false --allow-short-capture true`를 쓰면 matrix
완료 직후 같은 캡처 terminal의 ffmpeg에 `q`를 입력해 실제 길이(최소 12초)로 PNG/GIF를
만들고, ffprobe/manifest/SHA 목록은 남기면서 임시 MP4만 파생 완료 후 제거할 수 있다.
manifest와 `exact_commands.txt`는 요청/실제 시간 및 `early_finalized=true`를 기록한다.
`--allow-short-capture` 기본값 `false`는 요청 길이 strict 검증을 유지하고,
`--retain-source-video` 기본값 `true`는 원본 MP4도 유지한다. 제거 모드에서도 녹화 중
원본을 담을 순간 여유 공간은 필요하다.

2026-08-31 host에서는 gate 두 개와 실제 sensor source audit가 모두 검증된 뒤
현재 `camrod-tuned`로 분류한 구성의 B1 matrix가 PASS했다. 그 결과만
과거 tuned B1 열에 반영했으며, 실제 실행하지 않은 나머지 tuned 사이트와
현재 develop-parity 전체를 PASS로 바꾸지 않는다.
