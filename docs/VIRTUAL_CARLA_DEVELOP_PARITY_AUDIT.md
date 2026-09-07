# `virtual/carla` develop-parity 전수 감사

이 문서는 CAMROD 최신 `develop`을 CARLA와 연결하는 과정에서 변경된
코드가 단순 I/O adapter인지, 알고리즘·안전 기본값을 바꾸는 튜닝인지를
분리한 감사 기록이다. 2026-09-01 초기 감사 이후 2026-09-07에
원격 `develop` ancestry, 현재 v27 runtime 계약, 실제 v27 전 구역 주행 증빙을
다시 대조했다.

핵심 결론은 다음과 같다.

- 기본 `./scripts/virtual_carla/run.sh camrod`는 `origin/develop`의 알고리즘,
  공통 YAML 기본값, BT, parking/localization/safety 정책을 유지하고 CARLA
  sensor/actuator 경계만 opt-in으로 연다.
- `./scripts/virtual_carla/run.sh camrod-tuned`는 과거 Woraksan B1/B12 실행에
  사용된 map, reverse-return, recovery, speed/radius, 경계, UI 조정값을
  격리한 재현용 프로필이다.
- 현재 committed `virtual/carla` HEAD는 `origin/develop`
  `b194614b45d24f9a1a42b5c393417a97a28d9d7e`보다 **behind 0**이고,
  해당 commit 전체를 ancestry로 포함한다.
- `current/{operator_delivery,operator_recall,guest_recall}`은 exact
  `develop-plus-carla-site-geometry-v27`에서 새로 수집한 B1~B13 실제 mission
  증빙이다. 세 경로 모두 13/13 PASS, collision 0, 전 wheel 접지,
  `PARKED + CHARGING`을 확인했다.
- 각 bundle은 실행 시점의 source HEAD를 보존한다. Operator delivery,
  Operator recall, Guest는 각각 `2d03069...`, `504126f...`, `7ce1c7b...`이며,
  모든 E2E가 단일 byte-identical HEAD였다고 합치지 않는다.
- 현재 validator는 exact v27 profile, authoritative parameters, source/install
  fingerprint, map/actor/sensor/wheel/visual hash를 검증한다. Guest는 runner가
  최종 PASS를 기록한 뒤 post-final strict validation도 다시 실행했다.
- 2026-08-31 B1/B12 PNG/GIF/JSON은 **과거 tuned PASS**다. 이 문서에
  남아 있는 `9756edf...`/`PENDING`은 2026-09-01 스모크의 역사 값일
  뿐 현재 코드나 현재 mission 판정이 아니다.
- `audit-sensors` 성공은 CARLA actor/topic 소유권과 freshness를 보여주지만,
  mission 완주, develop parity, 실차 안전을 대신하지 않는다.

## 1. 기준 식별자와 브랜치 경계

| 항목 | 감사 기준 |
|---|---|
| 최종 재감사일 | 2026-09-07 KST |
| current core baseline | `origin/develop` `b194614b45d24f9a1a42b5c393417a97a28d9d7e` |
| Guest 최종 실행 HEAD | `virtual/carla` `7ce1c7b902c0e0d53299849cbcd728536d197c7d` |
| ancestry | `merge-base(7ce1c7b, origin/develop)=b194614...`; `origin/develop...7ce1c7b` = `0 43`, 즉 behind 0 / ahead 43 |
| 재감사 시 `origin/virtual/carla` | `805e4df898680bf3a022a0ed3deb2713e4bbc906` (로컬 HEAD의 후속 push는 별도 확인) |
| 역사 2026-09-01 baseline | `origin/develop` `9756edf2d7aebf59cf8b635b5d8f5982bf6211aa`, CARLA ref `c3646a89824fb075da58727d216a56ae0124b890` |
| 작업 브랜치 경계 | 최종 산출물은 `virtual/carla`에만 통합; `develop`에 CARLA 변경을 추가하지 않음 |

비교의 출발점은 파일명의 `latest` 또는 과거 스크린샷이 아니라
현재 `origin/develop` `b194614...`이다. `v2.2.1` 과거 발행 문서에 CARLA 변경을
소급 편입하지 않고 현재 CARLA overlay는 본 문서와
`docs/virtual_carla.md`에만 기록한다.

## 2. 분류 기준

| 분류 | 허용 범위 | 예 |
|---|---|---|
| 유지 | `origin/develop`의 기본 알고리즘/수치/토픽을 그대로 사용 | develop map, BT, safety threshold, parking/localization profile |
| 연결 추가 | 실차 device 대신 CARLA actor를 같은 canonical ROS 경계에 연결 | Image, PointCloud2, GNSS, IMU, radar, odometry relay; Twist→4WS |
| 기본-off hook | ordinary develop 실행에서 `false`, 빈 topic, 빈 overlay로 작동하는 선택 기능 | external simulator owner, CARLA manual topic, reverse/recovery hook |
| 튜닝 격리 | 맵/지형/물리 반응을 성공시키기 위한 develop 비동일 수치 | Woraksan OSM, speed `1.0`, radius `0.82`, threshold `100`, reverse-return |
| 증거 | 특정 commit/profile/host에서 실제로 관찰한 범위만 PASS | bundle별 v27 source HEAD와 strict 결과를 따로 보존; 서로 다른 실행 snapshot을 단일 HEAD라고 주장하지 않음 |

## 3. 패키지별 전수 감사

| 패키지/영역 | develop 대비 유지 | CARLA 연결을 위한 추가 | CARLA 전용 프로필에 격리한 내용 |
|---|---|---|---|
| `camrod_carla_adapter` | develop에 없던 신규 패키지이므로 core 대체가 아님 | Twist→extended 4WS, CARLA feedback/GNSS/IMU/camera/LiDAR/radar relay, step pacer, identity/gate/source audit | 보정 OSM, reverse Nav2/parking/input-adapter YAML, tuned BT와 wrapper |
| `camrod_bringup` | ordinary `sim`/hardware 기본 owner, fake-sim 기본, planning/localization/parking profile 선택은 develop 유지 | `external_simulator`, external camera owner, fake publisher 세부 비활성, sparse runtime overlay, CARLA diagnostics 선택 인자 | tuned/site-geometry wrapper가 각각 reverse/recovery/parking/map 인자를 명시 주입 |
| `camrod_control` | heading/lateral/parking/safety 공통 YAML 값은 develop 값; 추가 recovery/breakaway/manual-charging-departure 스위치 기본 `false` | CARLA 수동 명령을 기존 final safety gate 앞의 후보 source로 중재; Ackermann 반경·stale watchdog | plant torque recovery breakaway, path-relative recovery, zero-hold limit pause, corrective yaw, reverse lanelet, speed/radius/경계 수치; site-geometry에서만 0.35 s 수동 충전 이탈 lease |
| `camrod_planning` | develop BT, controller, goal source, active-goal reissue 기본 정책 유지 | 빈 topic/false로 기본 비활성된 reverse auxiliary goal·pose-jump·reissue hook | `RPPReverse`, `/planning/auto_reverse_goal_raw`, CARLA reverse BT/Nav2 overlay |
| `camrod_perception` | production YOLO + camera-LiDAR fusion이 semantic obstacle를 소유; 공통 perception 수치는 유지 | external canonical front camera를 local device 대신 선택, CARLA 장착 차이만 sparse extrinsic overlay | 없음. 의미론 obstacle 경계를 raw LiDAR cost로 바꾸지 않음 |
| `camrod_localization` | production input adapter/EKF 경로, ground-truth localization/TF direct publish `false` | CARLA odometry·GNSS·IMU를 production-shaped ROS 메시지로 변환; 지원하지 않는 RTK/DOP 값은 생성하지 않음 | 과거 CARLA input-adapter overlay는 tuned wrapper에서만 선택 |
| `camrod_system` | ordinary diagnostics QoS/threshold와 root DDS config 유지 | rendered 부하에 맞춘 `carla` rate/staleness profile, production `obstacle_fusion`에 맞춘 reliable QoS, 대용량 camera fragment용 별도 CycloneDDS XML | 없음; sensor 기준을 통과시키기 위해 fake rate를 올리지 않음 |
| `camrod_ui` | ordinary UI topic/default view; CARLA manual topic이 비어 있으면 추가 조작기는 no-op | CARLA composition이 명시한 manual topic에서만 panel/WebSocket/lease; CARLA overlay의 TF transform을 켜 actual camera/GNSS/LiDAR/radar를 UI fixed frame에 표시 | 높은 manual envelope/긴 lease는 tuned; docking rear fallback은 tuned과 site-geometry v27; Return re-arm은 site-geometry v27에 명시적으로 격리 |
| `camrod_platform` | hardware driver 코드와 ordinary platform owner 유지 | CARLA full에서 hardware driver off, `ranger_platform_bridge` on; adapter의 BMS/platform heartbeat는 sensor truth가 아님 | Drop Zone contact emulator는 tuned과 site-geometry v27에서만 on |
| map | `run.sh camrod`는 root `lanelet2_maps.osm` | CARLA world↔CAMROD map SE(2) alignment과 exact spawn cohort | tuned의 `woraksan_carla_lanelet2.osm`과 site-geometry의 version-pinned Woraksan v15 map을 각 wrapper가 선택 |
| `scripts/virtual_carla` | ordinary `setup_camrod.sh`, `colcon_build.sh`를 대체하지 않음 | 외부 prefix, gate, server→bridge→pacer→spawn→CAMROD, sensor audit/capture/matrix orchestration | `camrod-tuned` subcommand만 tuned launch를 선택 |

이 표의 “유지”는 CARLA 브랜치에 core 파일 차이가 전혀 없다는 뜻이 아니다.
external owner와 sparse overlay를 선택하기 위한 launch/source hook은 추가됐다. 판별
기준은 ordinary develop 실행에서 그 hook이 false/빈 값이고, parity full
launch에서도 develop 공통 파라미터를 다른 수치로 덮지 않는지이다.

### 3.1 핵심 구현 파일

| 역할 | 주요 파일 |
|---|---|
| parity/tuned composition | `camrod_carla_adapter/launch/camrod_carla_full.launch.py`, `camrod_carla_woraksan_tuned.launch.py` |
| CARLA 명령/feedback | `camrod_carla_adapter/src/camrod_carla_adapter/command_mapping.py`, `twist_to_4ws_node.py`, `feedback_bridge_node.py`, `feedback_mapping.py` |
| sensor relay/audit | `sensor_relay_node.py`, `radar_relay_node.py`, `lidar_filter_node.py`, `sensor_source_audit.py`, `sensor_source_audit_node.py` |
| external-owner bringup | `camrod_bringup/launch/_bringup_impl.py`, `config/bringup/launch_defaults.yaml`, `launch/fake_sensors.launch.py` |
| control opt-in hook | `camrod_control/src/cmd_vel_safety_gate_node.cpp`, `route_safety_recovery_controller_node.cpp`, `camping_site_maneuver_controller_node.cpp`, `include/camrod_control/manual_charging_departure_authorization.hpp` |
| planning opt-in hook | `camrod_planning/launch/planning.launch.py`, `nav2_lanelet.launch.py`, `src/goal_snapper_node.cpp`, `scripts/planning_state_machine_node.py` |
| perception ownership | `camrod_perception/launch/perception.launch.py`, `obstacle_fusion.launch.py`, `src/obstacle_lidar_node.cpp`, `camrod_carla_adapter/config/perception_carla.yaml`; CARLA host-local model은 `scripts/virtual_carla/prepare_yolo_engine.sh` |
| UI CARLA operation | `camrod_ui/camrod_ui_robot/launch/ui.launch.py`, `assets/frontend/src/ManualDrivePanel.js`, Guest frontend, `runtime/python/camrod_ui/ui_backend_node.py`, `ui_guest_node.py` |
| v27 launch/evidence | `camrod_carla_adapter/launch/camrod_carla_develop_site_geometry.launch.py`, `scripts/virtual_carla/run_site_evidence_matrix.sh`, `validate_site_evidence_collection.py`, `curate_final_evidence.py` |
| CARLA diagnostics/DDS | `camrod_system/config/diagnostics/carla`, `camrod_system/config/cyclonedds_carla.xml`, `camrod_system/src/diagnostics/perception_obstacle_checker_node.cpp` |
| operator scripts | `scripts/virtual_carla/env.sh`, `run.sh`, `setup.sh`, `build.sh`, `prepare_yolo_engine.sh`, `test.sh`, `camping_site_matrix.py` |

## 4. 실제 CARLA sensor 데이터 경로

full-sensor spawn은 ego에 RGB camera 2, LiDAR 1, radar 7, GNSS 2, IMU 1을
붙인다. 이 13개 actor 외에 collision, lane-invasion, odometry/control pseudo sensor가
있지만 UI-visible sensor 13개 계약에는 포함하지 않는다.

```text
CARLA actor 13개
  ├─ front RGB /carla/ego_vehicle/rgb_view/*
  │    → sensor relay → /sensing/camera/econ_front/*
  │    → production YOLO (official v9-s ONNX에서 이 host용으로 생성한 engine)
  │    → /perception/camera/detections_2d
  │
  ├─ rear RGB /carla/ego_vehicle/rgb_rear/*
  │    → sensor relay → /sensing/camera/econ_rear/*
  │    → production AprilTag parking detector
  │
  ├─ lidar_front /carla/ego_vehicle/lidar_front
  │    → /sensing/lidar/vanjee/points_raw
  │    → road-plane/self-return filter → /sensing/lidar/points_filtered
  │    ├─ Euclidean bbox/cluster → /perception/lidar/cluster_points (visual only)
  │    └─ front detections + production fusion → /perception/obstacles
  │                                          → semantic LiDAR cost raster
  │
  ├─ radar 7 → PointCloud2.Range → channel AvgRange/Range
  └─ GNSS 2 + IMU + odometry → CAMROD canonical sensor/platform inputs
```

### 4.1 fake output 비활성

CARLA full composition은 `enable_fake_sensors=false`로 fake sensor node 자체를 실행하지
않고, fake GNSS/IMU/LiDAR/radar/velocity/dummy-grid publisher 세부 플래그도 모두
`false`로 전달한다. front/rear external camera owner를 `true`로 선택해 local
V4L2/GStreamer capture를 시작하지 않고 CARLA relay가 같은 canonical topic을
소유한다.

`carla_platform_heartbeat_node` 및 tuned/site-geometry 전용 charging contact emulator는 실차 CAN/BMS가
없는 시뮬레이션 plant 상태를 제공한다. 이를 camera/LiDAR/GNSS/IMU/radar
계측치나 실차 CAN acceptance로 부르지 않는다.

### 4.2 LiDAR와 perception 소유권

CARLA LiDAR는 기계식 360° sensor가 아니다. JSON 계약은 수평
`-60..+60°`(120°), 수직 `-25..+10°`(35°), 16 channel, 60,000 points/s,
50 m range, `sensor_tick=0.1 s`인 **전방 고정 CARLA ray-cast solid-state
근사**다. `rotation_frequency=20`은 고정 부채꼴의 CARLA sampling 속성이지
sensor가 360°로 회전한다는 뜻이 아니다. 실물 sensor의 beam pattern,
intensity, noise, multipath까지 동등하다고 주장하지 않는다.

`obstacle_lidar_node`가 만든 Euclidean cluster/bbox는 진단·시각화 자료다.
cluster cloud는 `/perception/lidar/cluster_points`로 나가며 class 없는 점을
`/perception/obstacles`로 복제하지 않는다. semantic topic
`/perception/obstacles`의 단일 owner는 production camera-LiDAR fusion이다. 유효하고
신선한 YOLO class detection과 결합된 LiDAR 점만 fusion output이 되고, 이
semantic cloud를 develop의 LiDAR cost raster가 소비한다. 따라서 노면,
차체 self-return, class 없는 raw cluster가 임의 cost로 승격되면 정상이 아니다.

운영 UI도 이 안전 경계를 그대로 표시한다. 일반/develop 실행은 기존 호환성을 위해
raw bbox overlay opt-in 기능을 보존하지만, CARLA full launch는
`operator_telemetry_raw_lidar_bbox_overlay_enabled=false`를 기본값으로 전달한다. 따라서
CARLA UI의 `Semantic fusion cost`와 `Semantic obstacle cloud`는 fusion 결과만 보여준다.
raw `/perception/lidar/bboxes`는 제어 입력이 아닌 시각화 자료이며, 명시적으로 켤 때만
`Raw LiDAR bbox · visual only`로 표시된다.

source ownership audit에서도 이 경계를 exact contract로 검사한다. key는
`ui.perception.obstacles`, topic은 `/perception/obstacles`, 기대 publisher는
`obstacle_fusion`이다. CARLA perception diagnostic checker는 YOLO detection과 production
fusion publisher를 둘 다 감시하고, fusion에는 `reliable` QoS를 사용한다.
CARLA에서만 적용하는 `2 Hz`/`3 s`는 rendered alive/stale 하한이지
actor JSON의 `0.1 s` sensor tick이 10 Hz로 달성됐다는 성능 증거가 아니다.

CARLA에서만 camera/LiDAR 장착 차이를
`camrod_carla_adapter/config/perception_carla.yaml`의 sparse translation
`(0.0, +0.00001, -0.09970) m`로 덮는다. production perception 공통 수치를
시뮬레이션에 맞추어 바꾸지 않는다.

특히 일반 `camrod`는 최신 develop의 YOLO `min_confidence=0.50`을 보존한다.
Woraksan 전 구역 증빙용 `camrod-site-geometry`만 별도
`perception_carla_site_geometry.yaml`을 명시적으로 선택하며, 그 파일에는 같은
extrinsic과 실제 foliage 오검출 cohort에서 검증한 `min_confidence=0.95`가 함께 있다.
즉 0.95는 CARLA 전체 기본값이 아니라 site-geometry 프로필의 추적 가능한 opt-in이다.

## 5. 런타임 프로필 파라미터와 v27 격리 경계

아래는 편의상 주요 차이를 정리한 표다. 실제 실행 시에는 source 파일만
보지 말고 install을 새로 build한 뒤 `ros2 launch ... --show-args`, node parameter
dump, runtime report에서 최종 값을 같이 저장한다.

| 항목 | `camrod` develop-parity | `camrod-tuned` |
|---|---:|---:|
| lanelet map | root `lanelet2_maps.osm` | `woraksan_carla_lanelet2.osm` |
| charging contact emulator | `false` | `true` |
| recovery breakaway torque authority | `false` | `true` |
| sim planning/localization/parking profile | `false / false / false` | `true / true / true` |
| manual linear/lateral/angular ceiling | `0.20 / 0.20 / 0.20` | `1.40 / 1.00 / 0.7853` |
| UI manual lease | `0.25 s` | `0.75 s` |
| final command speed scale | `0.5` | `1.0` |
| minimum Ackermann radius override | `0.0` (disabled) | `0.82 m` |
| cost/lanelet/current threshold | `85 / 85 / 85` | `100 / 100 / 100` |
| reverse lanelet check | `false` | `true` |
| roadside reverse return | `false` | `true` |
| roadside reverse handoff | `0.03 m` | `0.10 m` |
| reverse Nav2 controller/topic | empty / empty | `RPPReverse` / `/planning/auto_reverse_goal_raw` |
| path-relative recovery | `false` | `true` |
| zero-hold pauses limits | `false` | `true` |
| corrective yaw beyond limit | `false` | `true` |
| active goal reissue | `false` | `true` |
| crab approach slowdown/min speed | `0.0 / 0.0` | `1.0 m / 0.12 m/s` |
| rotate-180 timeout | `0.0` (develop semantics) | `60.0 s` |
| campsite entry position tolerance | `0.15 m` | `0.05 m` |
| Nav2/parking/input adapter overlay | disabled/develop files | reverse-return, CARLA parking, historical input adapter |
| UI TF transform | `true` | `true` |
| UI TF latest fallback | `0.0 s` | `0.075 s` |
| UI camera raw fallback | `true` | `false` |
| docking rear-camera fallback | `false` | `true` |
| campsite Return site-exit re-arm | `false` | `true` |

`camrod-site-geometry` v27은 위 `camrod-tuned`의 별칭이 아니며, 단순
docking rear-camera fallback 하나만 켜는 프로필도 아니다. 현재
`camrod_carla_develop_site_geometry.launch.py`가 공유 full launch 위에
명시적으로 올리는 CARLA-only opt-in은 다음과 같다.

| v27 site-geometry opt-in 분류 | exact 효과 |
|---|---|
| map/perception/parking overlay | Woraksan site-geometry v15 map; YOLO `min_confidence=0.95`; CARLA AprilTag/parking parameter file; bounded lateral retry `2` 회, forward `0.8 m` at `0.20 m/s`, timeout `30 s`, lateral gain `2.7` |
| 속도 분할 | final gate `speed_scale=1.0`; RPP/RotationShim은 overlay에서 `0.555556 m/s`로 pre-limit해 일반 Nav2 cruise를 두 배로 올리지 않음 |
| Return 권한/충전 | `return_site_exit_rearm_enabled=true`; CARLA charger/BMS 대신 verified stopped contact를 쓰는 charging-contact emulator; `allow_manual_departure_while_charging=true`, accepted manual heartbeat lease `0.35 s` |
| 물리 recovery lease | authenticated low-speed `recovery_breakaway_enable=true`; rotation stall lease `true`, fresh-status timeout `1.25 s` |
| 경로 상대 recovery | unsnapped `/localization/pose`로 pose jump 확인; path-relative recovery `true`; center re-entry `0.15 m`; zero-hold가 limit clock을 일시정지; 한계 밖 corrective yaw 허용 |
| roadside reverse | `roadside_reverse_return_enable=true`, handoff `0.03 m`, `/planning/auto_reverse_goal_raw`, `RPPReverse`, reverse lanelet corridor check |
| campsite safety phase lease | static/lanelet bypass 목록에 `ALIGN_OUTBOUND_LANE_YAW`를 포함한 `ALIGN_ENTRY_YAW,REVERSE_IN,CRAB_IN,ROTATE_180,ALIGN_RETRACE_YAW,ALIGN_OUTBOUND_LANE_YAW,REVERSE_OUT,CRAB_OUT`; 이 phase는 production/default 목록에는 없음 |
| return completion | planning-state-machine handoff `return_goal_reached_distance_m=0.35`; Nav2 success latch나 parking completion tolerance를 늘리지 않음 |
| footprint/cost latch | simulator/map planning envelope의 `lanelet_safety_footprint_enable=false`; fabrication-size body hard stop은 유지; merged clear는 trigger source로 확인하고 merged dynamic label은 `radar`로 제한 |
| goal lifecycle | route recovery 후 동일 active goal의 bounded reissue `true`; obsolete entry-anchor centering은 initial-error `0.0`으로 계속 비활성 |
| campsite maneuver | crab slowdown `1.0 m`/최속 `0.12 m/s`; rotate timeout `90 s`, max angular `0.45 rad/s`; entry/rotate tolerance `0.05 m`, rotate centering initial bound `0.65 m`; CRAB_IN yaw compensation `2.0°`, settle tolerance `1.5°`/`15 s`; CRAB_OUT yaw recovery `8°`, `8` attempts, global `90 s` |
| UI docking 표시 | AprilTag debug가 없거나 stale일 때 실제 CARLA rear RGB를 “대체 영상”으로 보이는 `operator_telemetry_docking_rear_camera_fallback_enabled=true`; detector 결과로 위장하지 않음 |

위 값은 site-geometry wrapper를 선택한 시험에만 유효하다. ordinary
production/develop과 공유 `camrod_carla_full` launch의 대응 기본값은
`false`, 빈 topic/overlay, 또는 develop 원본 수치로 유지된다.

수동 충전 이탈 lease는 charging block만 제거하며 E-stop, input freshness, command
mode/error, critical SOC, cost/lanelet/collision watchdog을 우회하지 않는다. dedicated
manual input, manual owner/engage, drive-enable, charging, battery-ready, no-site-maneuver가
동시에 맞아야 하고 수락된 heartbeat만 lease를 갱신한다. runtime 설정은 두 파라미터를
원자적으로 검증하며 유효한 변경도 기존 lease와 출력을 zero로 초기화한다.

runtime profile은 문자열까지 exact 계약이다. 현재
`validate_site_evidence_collection.py`는
`develop-plus-carla-site-geometry-v27`과 authoritative v27 parameter set을 요구하며,
보존된 v26 runtime audit을 새 v27 검증으로 인정하지 않는다.

위 프로필들이 공통으로 사용하는 CARLA I/O 추가는 physical 4WS backend,
external odometry/sensor owner, production YOLO/fusion, CARLA diagnostics, CARLA mount extrinsic,
UI fixed-frame TF transform, manual command topic, sensor preflight/source audit이다. 이 공통 분은 맵 성공용
알고리즘 튜닝이 아니라 simulator I/O 구성이다.

## 6. 실행 순서

각 프로세스를 별도 terminal에서 실행하며 모든 terminal은 같은
`RANGER_CARLA_ROOT`, `ROS_DOMAIN_ID`, RMW/DDS 구성을 사용한다.

```bash
cd /home/hong/camrod_ws/src
export RANGER_CARLA_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline
export CARLA_RENDER_MODE=onscreen

./scripts/virtual_carla/prepare_yolo_engine.sh --verify-only --print-path
./scripts/virtual_carla/site_access.sh doctor
./scripts/virtual_carla/site_access.sh commands
```

CARLA runner는 검증된 `$CAMROD_CARLA_YOLO_MODEL_PATH`를 자식 launch의
`YOLOV9_MODEL_PATH`로만 전달한다. 따라서 일반 develop/실차 launch의 packaged
`epoch74_step151350.vec2box.sim.engine` 기본은 변경되지 않는다. TensorRT plan은
TensorRT/GPU 종속이므로 Git artifact가 아니라 `$RANGER_WORK_ROOT/camrod/model_cache`의
untracked 산출물이다. NullRHI/control-only에서는 sensor relay, YOLO와 fusion을 모두
끄며 이 결과를 camera/perception PASS로 사용하지 않는다.

그 다음 terminal 1~4에서 순서대로 실행한다.

```bash
./scripts/virtual_carla/site_access.sh server
./scripts/virtual_carla/site_access.sh bridge
./scripts/virtual_carla/site_access.sh pacer
./scripts/virtual_carla/site_access.sh spawn
```

terminal 5에서 검증 목적에 맞는 하나만 선택한다.

```bash
# 최신 develop parity 재검증
./scripts/virtual_carla/site_access.sh camrod

# 최신 develop + exact v27 전 구역 mission 증거 생성
./scripts/virtual_carla/site_access.sh camrod-site-geometry

# 또는: 2026-08-31 Woraksan 튜닝 재현
./scripts/virtual_carla/site_access.sh camrod-tuned
```

sensor 소유권은 UI에 값이 보이는 것만으로 판정하지 않는다.

```bash
./scripts/virtual_carla/site_access.sh audit-sensors
```

B1~B13 mission runner는 현재 실행 중인 프로필을 변경하지 않는 observer/UI
dispatcher다. 새 v27 승인 증거는 반드시 위 `camrod-site-geometry`가
실행 중일 때 별도 terminal에서 수집한다. `camrod`는 core parity,
`camrod-tuned`는 역사 재현용이므로 그 결과를 v27 전 구역 승인으로
라벨하지 않는다.

```bash
./scripts/virtual_carla/site_access.sh camping-sites-plan
./scripts/virtual_carla/site_access.sh camping-sites
```

프로필, map SHA, gate SHA, actor ID, parameter dump, sensor audit, mission JSON을 같은
run directory에 묶지 않으면 B1/B12 과거 결과와 새 parity 결과를 구별할 수
없다.

## 7. 검증 상태와 한계

| 검증 항목 | 판정 | 해석 경계 |
|---|---|---|
| Git ancestry | `origin/develop=b194614...`, Guest 실행 HEAD `7ce1c7b...`, merge-base 동일, `origin/develop...7ce1c7b=0 43` | latest develop보다 behind 0. 문서/evidence commit과 최종 push SHA는 E2E 실행 SHA 뒤에 생김 |
| v27 Operator delivery | B1~B13 `13/13 PASS`, `7,840.705 s`, `1,786.452243 m` | 실행 HEAD `2d03069...`; production destination + Return 경로 |
| v27 Operator typed recall | B1~B13 `13/13 PASS`, `7,669.826 s`, `1,723.057324 m` | 실행 HEAD `504126f...`; out 좌표에서 즉시 복귀 경로를 만드는 최신 recall 경로 포함 |
| v27 visible Guest recall | B1~B13 `13/13 PASS`, `7,803.845 s`, `1,722.789852 m` | 실행 HEAD `7ce1c7b...`; visible page `navigate -> usage_complete`, site/nonce-qualified source, post-final strict PASS |
| current source/package/unit/launch contract | `scripts/virtual_carla/test.sh` 전체 PASS; focused matcher/wrapper `97/97`, adapter `488/488`, bringup `347/347`, control `207/207` | offline/contract 시험이며 live E2E를 대신하지 않음 |
| current sensor source | stream `36/36`, CARLA actor `13/13`, fake/dummy owner 0 | actual topic/actor/freshness 증거이며 실물 sensor beam/noise 동등성은 아님 |
| visible manual 4WS | straight/turn/crab/zero-turn `4/4 PASS`, collision 0, 전 wheel 접지 | live bundle에는 source HEAD binding이 없음. 실행 뒤 동일 lease를 `413b9d...`에서 policy refactor하고 unit test한 사실을 E2E와 분리 |
| 2026-08-31 tuned B1/B12 | 역사 tuned PASS | 나머지 tuned 사이트는 당시 `PENDING`; 현재 v27 판정이 아님 |
| actual wheel state | 39개 mission과 4개 manual scenario에 wheel summary/PNG, raw hash가 있음 | 큐레이션은 raw JSONL 자체가 아니라 manifest의 원본 hash와 파생 요약을 보존 |
| LiDAR model | CARLA 전방 120° ray-cast 근사 | 실물 solid-state beam/noise/multipath 동등성 미검증 |
| 실차/Jetson | 본 CARLA 감사 범위 밖 | CARLA PASS를 FIELD-PASS로 전환하지 않음 |

`docs/evidence/virtual_carla/current/`는 이제 exact-v27 세 mission bundle과 manual
bundle만 담는다. 각 mission site의 runtime audit, sensor-source audit, native matrix,
physical wheel, 실제 X11 PNG/GIF를 strict validator가 교차 검증했고 curator가 일반 파일로
복사했다. 총 PNG 86개, GIF 43개이며 root `SHA256SUMS` 528개가 모두 일치한다.

Operator 자료 뒤 추가된 변경은 경로가 분리된다. recall 수정은 recall bundle 실행 전에
포함됐고, 이후는 manual charging departure lease/refactor, Guest 선택-site snapshot,
Guest return-source evidence matcher다. 따라서 Operator E2E를 현재 경로 동작 증거로
사용하되 Guest HEAD에서 다시 실행한 것처럼 소급 표기하지 않는다. 세부 commit과 결과는
[`VIRTUAL_CARLA_FINAL_VALIDATION.md`](VIRTUAL_CARLA_FINAL_VALIDATION.md)에 고정한다.

## 8. 재감사 명령

```bash
cd /home/hong/camrod_ws/src

git rev-parse origin/develop origin/virtual/carla HEAD
git diff --name-status origin/develop -- \
  camrod_bringup camrod_control camrod_perception camrod_planning \
  camrod_system camrod_ui camrod_carla_adapter scripts/virtual_carla

git diff --exit-code origin/develop -- \
  docs/V2_2_1_RELEASE_NOTES.md DONE.txt

rg -n 'camrod-tuned|develop-parity|PENDING|/perception/obstacles|cluster_points|fake' \
  README.md docs/virtual_carla.md docs/VIRTUAL_CARLA_DEVELOP_PARITY_AUDIT.md \
  docs/evidence/virtual_carla
```

마지막 `git diff --exit-code`가 0이어야 v2.2.1 과거 문서에 CARLA 설명을
소급 편입하지 않았음을 확인할 수 있다. 새 parity commit SHA와 실제
build/test/live report는 최종 통합·push 시 이 감사의 후속 기록으로 남긴다.
