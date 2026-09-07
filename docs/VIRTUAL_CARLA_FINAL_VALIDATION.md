# CAMROD × CARLA v27 최종 검증 보고서

이 문서는 Woraksan v15 site-access 환경에서 수행한 CAMROD `virtual/carla`
최종 검증 결과, 실제 화면 자료, 실행 시점의 소프트웨어 provenance, 재실행 절차를
한곳에 고정한다. 승인 자료 인덱스는
[`docs/evidence/virtual_carla/current/`](evidence/virtual_carla/current/)이며,
아래 수치는 큐레이션된 manifest와 strict validation 결과에서 다시 대조했다.

## 1. 최종 판정

검증된 세 가지 B1~B13 왕복 경로는 모두 `13/13 PASS`다. 합계는 39개 사이트
왕복, `23,314.376 s`, `5,232.299419 m`이며 모든 왕복에서 다음 acceptance를
확인했다.

- 충돌 event `0`
- FL/FR/RL/RR 네 wheel이 모든 보존 sample에서 접지
- 마지막 속도 `0.05 m/s` 이하
- Drop Zone 복귀 후 `PARKED=true`, `CHARGING=true`
- actor type `vehicle.ranger.default`, role `ego_vehicle`
- actual CarlaUE4와 실제 표시 UI를 X11에서 캡처한 PNG/GIF 존재
- `PHYSX_FOUR_WHEEL_STEERING` backend, patched CARLA physical gate 및 actor identity 일치

| 경로 | 권한/동작 | 결과 | 전체 | 출발 | 복귀 | 실행 source HEAD | 승인 자료 |
|---|---|---:|---:|---:|---:|---|---|
| Operator delivery + Return | visible Robot UI delivery | 13/13 PASS | 7,840.705 s / 1,786.452243 m | 2,789.613 s / 889.573874 m | 5,051.089 s / 896.878369 m | `2d03069d1fca84c2a23597d03bc4258b9c2d3399` | [operator_delivery](evidence/virtual_carla/current/operator_delivery/) |
| Operator recall + Return | visible Robot UI recall | 13/13 PASS | 7,669.826 s / 1,723.057324 m | 2,573.926 s / 858.350562 m | 5,095.902 s / 864.706762 m | `504126f11cc37d355b2404d21601a111acf5fd07` | [operator_recall](evidence/virtual_carla/current/operator_recall/) |
| Guest recall + 이용 완료 | visible Guest UI WebSocket | 13/13 PASS | 7,803.845 s / 1,722.789852 m | 2,591.052 s / 858.190109 m | 5,212.793 s / 864.599743 m | `7ce1c7b902c0e0d53299849cbcd728536d197c7d` | [guest_recall](evidence/virtual_carla/current/guest_recall/) |
| Robot UI manual 4WS | straight/turn/crab/zero-turn | 4/4 PASS | 아래 5절 참조 | - | - | bundle에 top-level Git HEAD가 없으므로 임의 지정하지 않음 | [manual_4ws](evidence/virtual_carla/current/manual_4ws/) |

각 mission bundle에는 사이트당 contact-sheet PNG 1개, wheel-summary PNG 1개,
motion GIF 1개가 있다. 즉 mission별 `PNG 26 + GIF 13`, 세 mission 전체
`PNG 78 + GIF 39`다. manual bundle의 `PNG 8 + GIF 4`를 합치면 현재 승인
인덱스에는 `PNG 86 + GIF 43`이 있다. 제어 명령으로 합성한 도식이 아니라 표시
중인 CarlaUE4와 Robot/Guest UI의 실제 화면 녹화에서 파생한 자료다.

## 2. 실행 snapshot과 최신 develop 관계

비교 기준은 `origin/develop`
`b194614b45d24f9a1a42b5c393417a97a28d9d7e`다. 세 mission 실행 commit은 모두 이
develop commit을 정확한 ancestor로 포함한다.

| 실행 snapshot | `origin/develop...snapshot` | 의미 |
|---|---:|---|
| Operator delivery `2d03069...` | `0 32` | develop보다 behind 0, CARLA overlay 32 commit ahead |
| Operator recall `504126f...` | `0 34` | develop보다 behind 0, recall 변경을 포함한 34 commit ahead |
| Guest recall `7ce1c7b...` | `0 43` | develop보다 behind 0, manual/Guest/evidence 후속 변경까지 포함한 43 commit ahead |

세 결과를 하나의 byte-identical HEAD에서 실행했다고 주장하지 않는다. 실행 순서상
Operator delivery 뒤에 recall 처리 변경이 들어갔고, Operator recall 뒤에는 manual
4WS와 Guest UI/evidence 변경이 추가됐다. 각 bundle의 runtime audit가 기록한 source
HEAD가 해당 결과의 권위 있는 실행 식별자다.

주요 후속 commit과 영향 범위는 다음과 같다.

| commit | 변경 | 앞선 증거에 대한 해석 |
|---|---|---|
| `aeb191aed` | deferred recall admission을 evidence가 관찰하도록 보강 | delivery 결과를 recall 검증으로 확대하지 않음 |
| `504126f11` | backend 재시작을 지나도 recall route anchor를 보존 | Operator recall 실행에 포함됨 |
| `e74bbc031` | live CARLA 충전 상태에서 전용 manual 명령으로 안전하게 출발할 수 있는 lease 추가 | mission 경로가 아니라 manual 경로용 opt-in |
| `413b9dfb2` | manual charger lease 정책을 독립 정책 객체로 만들고 atomic runtime update를 시험 가능하게 정리 | Guest 실행에는 포함; 이전 mission 수치를 소급 변경하지 않음 |
| `ddd2b0cbb`~`136853c67` | visible manual UI 축 해석, actual steering 판정, zero-torque crossing 판정 보강 | manual 증거 판정 경계만 강화 |
| `077999ac3` | Guest 목적 선택 후 UI가 갱신되어도 `dispatchSite` snapshot을 보존해 `{site:null}` 전송 방지 | Guest B1~B13 실행에 포함 |
| `7ce1c7b90` | legacy `guest:usage_complete`와 현재 `guest:usage_complete:site=B#:g=N`을 site/generation까지 엄격 검증 | Guest 최종 실행 및 post-final strict validator에 포함 |

manual 큐레이션 manifest는 actor, UI 입력, ROS/physical 관찰과 artifact hash를
기록하지만 top-level Git HEAD를 기록하지 않는다. 따라서 이 보고서는 manual 자료를
`7ce1c7b...`와 byte-identical이라고 추정하지 않고, bundle 자체의 4/4 acceptance와
시나리오 manifest를 검증 근거로 사용한다.

## 3. 전체 제어·센서 아키텍처

```text
CARLA camera/LiDAR/GNSS/IMU/radar/odometry actors
    -> carla_ros_bridge
    -> camrod_carla_adapter relay / frame·type·map alignment
    -> CAMROD canonical sensor and odometry topics
    -> localization + production YOLO/camera-LiDAR fusion + planning
    -> nav2_cmd_vel 또는 전용 manual_cmd_vel
    -> CAMROD cmd_vel safety gate (/control/cmd_vel_ros)
    -> Twist-to-4WS mapping + extended Ackermann controller
    -> patched CARLA FL/FR/RL/RR steer-angle and drive-torque API
    -> physical status / wheel telemetry / odometry feedback

Robot UI / Guest UI
    -> production HTTP·WebSocket boundary
    -> typed delivery/recall/Return 또는 dedicated manual source
    -> 위 CAMROD planning/safety 경로
```

핵심 원칙은 CARLA가 CAMROD 알고리즘을 대체하지 않는다는 것이다. 실제 CARLA actor의
메시지를 CAMROD canonical 경계로 변환하고, CAMROD 최종 safety-gated Twist를 Ranger
physical 4WS 명령으로 변환한다. ordinary develop/hardware 실행의 기본 owner와 파라미터는
유지하며, 외부 simulator 및 Woraksan 지형 적응값은 명시적인 opt-in wrapper에서만 켠다.

### 3.1 컴포넌트별 변경과 격리

| 컴포넌트 | develop에서 유지한 것 | CARLA 연결/검증을 위해 추가한 것 | 격리 및 안전 경계 |
|---|---|---|---|
| `camrod_carla_adapter` | CAMROD core package를 대체하지 않음 | Twist→extended physical 4WS, CARLA odometry/GNSS/IMU/camera/LiDAR/radar relay, 20 Hz pacer, gate/actor/source audit | patched CARLA와 두 physical gate가 없으면 nonzero command를 fail closed |
| `camrod_bringup` | ordinary sim/hardware owner와 fake-sim 기본 동작 | `external_simulator`, external camera/sensor owner, fake publisher 선택 비활성, sparse CARLA overlay | 관련 기본값은 false/빈 overlay; CARLA full launch만 명시 활성화 |
| `camrod_planning` | 최신 develop BT, controller, destination/recall 상태기계 | current pose 기반 roadside return, reverse auxiliary goal, pose-jump/reissue hook | CARLA reverse/site hook은 기본 off; site-geometry wrapper에서만 활성화 |
| `camrod_control` | engage/mission/estop, freshness, mode, SOC, obstacle와 watchdog safety gate | 전용 manual source 중재, physical 4WS mapping, bounded recovery, campsite maneuver/Return re-arm hook | recovery·re-arm·charger departure는 기본 off이며 v27 site wrapper만 명시 선택 |
| `camrod_perception` | production YOLO + camera-LiDAR fusion이 `/perception/obstacles`의 의미론적 owner | 실제 CARLA RGB/ray-cast LiDAR relay, 장착 extrinsic과 ground/self-return filter | raw LiDAR bbox/cluster는 visual-only; class 없는 raw cluster를 안전 cost로 승격하지 않음 |
| `camrod_localization` / `camrod_platform` | production input adapter/EKF와 platform status 경계 | CARLA odometry·dual GNSS·IMU를 production-shaped topic으로 변환, CARLA platform bridge | 지원하지 않는 RTK/DOP/satellite 정보를 생성하지 않음; fake sensor truth 사용 안 함 |
| `camrod_system` | production diagnostic 구조 | rendered 부하에 맞춘 CARLA rate/staleness 및 reliable fusion QoS, camera용 CycloneDDS transport | 검사 통과를 위해 fake rate/message를 만들지 않음 |
| `camrod_ui` | production Robot UI/Guest UI와 기존 topic 계약 | CARLA actual sensor overlay, dedicated manual panel/WebSocket, Guest site-qualified 이용완료 source | CARLA manual topic이 비면 no-op; docking rear fallback은 실제 rear RGB임을 명시하며 detector 결과로 위장하지 않음 |
| `scripts/virtual_carla` | 일반 setup/build script를 대체하지 않음 | doctor, lifecycle, sensor audit, visible UI automation, actual X11 capture, site matrix, strict validator, curation | map/profile/source/actor/hash가 다르면 증거 생성 전에 중단 |
| map/profile | 일반 `run.sh camrod`는 develop map/기본 수치 | v15 UE map, lanelet/map SE(2), site-geometry parameter cohort | `site_access.sh`가 `woraksan-camrod-site-geometry-v15`를 self-pin; 원본 map/develop 기본을 변경하지 않음 |

### 3.2 최신 recall 동작

사이트 `IN` 시점의 과거 XY로 돌아가야만 복귀 route가 생기는 방식에 의존하지 않는다.
현재 campsite controller는 `CRAB_OUT`에서 fresh live lanelet projection/current XY를
기준으로 route handoff를 수행하고, 과거 entry snap의 longitudinal 거리는 무시한다.
공통 설정의 `enable_live_lanelet_return_handoff=true`, handoff 거리 `0.15 m`, 정지
hold `1.20 s`가 이 경계를 만든다. `504126f11`은 backend가 중간에 재시작된 경우에도
필요한 recall route anchor가 사라지지 않도록 보강했다.

v27 site-geometry 프로필은 이 develop 동작 위에 roadside reverse return, Return 전
command-gate re-arm, active goal bounded reissue와 campsite CRAB_OUT을 추가한다. 이것은
CARLA 지형/plant 적응용이며 ordinary develop/shared full launch에서는 기본 off다.

### 3.3 manual 충전 출발 lease

충전 중 수동주행이 영구 차단되거나 반대로 충전 safety 전체를 우회하지 않도록
`manual_charging_departure_authorization.hpp`와 safety-gate 통합을 추가했다.

- site-geometry에서만 enabled, ordinary develop/shared full 기본값은 `false`
- lease `0.35 s`; accepted dedicated manual heartbeat만 갱신
- manual command owner active, manual engaged, mission disengaged, drive enabled,
  campsite maneuver 비활성, charging 및 battery-ready를 모두 요구
- 활성 동안 `charging` 차단 이유만 제거
- estop, command freshness, mode/error, critical SOC, obstacle/cost, watchdog은 계속 적용
- runtime parameter set은 전체 config를 먼저 검증한 뒤 atomic 적용; invalid update는
  거부하고 설정 변경 시 lease reset 및 zero command
- UI WebSocket의 press/release lease(`0.25/0.75 s`)와 safety input stale timeout
  (`0.35 s`)은 서로 다른 계층이며 혼동하지 않음

## 4. Guest B1~B13 상세 결과

아래 값은
[`guest_recall/summary/camping_site_metrics.md`](evidence/virtual_carla/current/guest_recall/summary/camping_site_metrics.md)의
explicit duration/distance split이다. 모든 행은 최종 `CHARGING`, parked/charging true,
collision 0, 네 wheel 접지이며 actor는 `66`이다.

| Site | 결과 | 총 시간 s | 출발 s | 복귀 s | 총 거리 m | 출발 m | 복귀 m | Drop Zone 오차 m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| B1 | PASS | 753.340 | 256.474 | 496.866 | 167.556243 | 83.566454 | 83.989789 | 0.103478 |
| B2 | PASS | 728.370 | 243.707 | 484.663 | 168.201701 | 83.834474 | 84.367227 | 0.098705 |
| B3 | PASS | 701.556 | 230.306 | 471.250 | 155.066164 | 77.388388 | 77.677776 | 0.099766 |
| B4 | PASS | 681.406 | 225.470 | 455.936 | 152.919572 | 76.123871 | 76.795701 | 0.100456 |
| B5 | PASS | 634.300 | 210.982 | 423.318 | 143.354755 | 71.426044 | 71.928711 | 0.104768 |
| B6 | PASS | 640.017 | 206.105 | 433.912 | 140.204180 | 69.766702 | 70.437478 | 0.107524 |
| B7 | PASS | 579.192 | 196.802 | 382.390 | 131.381479 | 65.434520 | 65.946959 | 0.103293 |
| B8 | PASS | 568.099 | 190.614 | 377.485 | 126.714500 | 63.130369 | 63.584131 | 0.097859 |
| B9 | PASS | 533.085 | 178.998 | 354.087 | 118.417239 | 58.937843 | 59.479396 | 0.106701 |
| B10 | PASS | 517.525 | 174.369 | 343.156 | 114.257212 | 56.840778 | 57.416434 | 0.112791 |
| B11 | PASS | 507.260 | 166.808 | 340.452 | 108.103327 | 53.843864 | 54.259463 | 0.104551 |
| B12 | PASS | 477.254 | 160.111 | 317.143 | 102.552748 | 51.084927 | 51.467821 | 0.101666 |
| B13 | PASS | 482.441 | 150.306 | 332.135 | 94.060732 | 46.811875 | 47.248857 | 0.102230 |
| **합계** | **13/13** | **7,803.845** | **2,591.052** | **5,212.793** | **1,722.789852** | **858.190109** | **864.599743** | - |

Guest Return은 단순 문자열 spoof가 아니다. 실제 source는 예를 들어
`guest:usage_complete:site=B2:g=<positive-generation>` 형태이며, validator가 현재
시험 site와 일치하는 `B1..B13`, 양의 generation, 실제 UI/ROS event chain을 확인한다.
잘못된 site, `g=0`, malformed source 또는 다른 authority는 거부한다.

각 사이트 화면은 `guest_recall/sites/<SITE>/visual/`에 있다. 예를 들어 B2는
[contact sheet](evidence/virtual_carla/current/guest_recall/sites/B2/visual/representative_contact_sheet.png),
[motion GIF](evidence/virtual_carla/current/guest_recall/sites/B2/visual/representative_motion.gif),
[wheel summary](evidence/virtual_carla/current/guest_recall/sites/B2/wheel_summary/wheel_summary.png)로
바로 확인할 수 있다.

### 4.1 B2 경사/턱 판정

B2는 지형을 삭제하거나 actor를 순간이동해 통과시킨 결과가 아니다. raw physical-wheel
telemetry의 경사 구간을 별도로 검사한 결과는 다음과 같다.

| 구간 | 최대 절대 pitch | 4-wheel 접지 | 경사 구간 최대 external drive torque | 속도/정지 판정 |
|---|---:|---:|---:|---|
| 출발 | `11.62°` | 4/4 | 약 `8.052 N·m/wheel` | 경사 진행 중 `0.365 m/s` 이상을 유지, 지속 stall 없음 |
| 복귀 | `11.60°` | 4/4 | 약 `7.471 N·m/wheel` | 경사 구간 최저 `0.129 m/s`, `v < 0.02 m/s`가 `0.1 s` 이상 지속된 구간 없음 |

따라서 이 accepted run에서 B2 실패 원인은 토크 부족으로 관찰되지 않았고,
low-speed breakaway/recovery가 발동하지도 않았다. 전체 B2 wheel stream은 7,804 sample,
관찰 rate 약 10 Hz이고 in-air count는 모든 wheel에서 0이다. Git에는 대용량 raw JSONL을
넣지 않았지만 [wheel summary JSON](evidence/virtual_carla/current/guest_recall/sites/B2/wheel_summary/wheel_summary.json)과
physical manifest에 raw file의 byte 수, line 수, SHA-256이 남아 있다.

## 5. visible Robot UI manual 4WS

manual 시험은 API로 Twist를 직접 주입한 것이 아니다. 화면에 표시된 production Robot
UI의 실제 CDP pointer/keyboard event로 positive → zero → inverse → zero 순서를 만들고,
ROS subscriber와 patched CARLA wheel readback은 관찰에만 사용했다. 네 시나리오 모두
collision 0, 4/4 접지, strict physical sequence와 exact zero cut을 통과했다.

| 시나리오 | 정방향 physical 결과 | 역방향 physical 결과 | 왕복 오차 | actual steering 일치 | 4-wheel torque 동시 활성 sample |
|---|---|---|---|---:|---:|
| straight | 0.5645 m, yaw -0.095° | 0.4881 m, yaw -0.081° | 0.0717 m / 0.170° | 100% / 100% | 76 / 76 |
| turn | 0.4711 m, yaw -27.508° | 0.4553 m, yaw +26.977° | 0.0820 m / 0.581° | 100% / 100% | 74 / 73 |
| crab | lateral 0.5125 m, yaw -0.212° | lateral 0.4553 m, yaw -0.005° | 0.0545 m / 0.207° | 94.29% / 97.14% | 64 / 65 |
| zero-turn | path 0.0417 m, yaw -28.831° | path 0.0400 m, yaw +29.906° | 0.0090 m / 0.594° | 100% / 100% | 58 / 62 |

자료는 [manual 4WS 보고서](evidence/virtual_carla/current/manual_4ws/manual_4ws_report.md)와
각 `scenarios/{straight,turn,crab,zero_turn}/visual/` 및 `wheel_summary/`에 있다.
straight/turn/crab/zero-turn 각각 contact sheet, motion GIF, wheel-summary PNG를 제공한다.

## 6. 실제 CARLA 센서와 perception 경계

rendered sensor source audit 결과는 stream `36/36`, CARLA actor `13/13`, failed 0이다.
actor 구성은 camera 2, LiDAR 1, GNSS 2, IMU 1, radar 7이다. UI-visible canonical
camera/LiDAR/GNSS/IMU/radar 경계는 실제 CARLA actor가 소유하며 fake/dummy owner는 0이다.

LiDAR는 실물 solid-state와 동일하다고 주장하지 않는다. 현재 CARLA actor는 전방 고정
120° ray-cast 근사이며, relay 후 ground plane과 Ranger self-return을 제거한다.
`/perception/lidar/cluster_points`와 raw bbox는 visual-only다. 실제 안전 경계
`/perception/obstacles`와 semantic cost는 develop과 같은 production YOLO +
camera-LiDAR fusion만 소유하므로, class 없는 LiDAR cluster가 임의 장애물로 승격되지
않는다. 맑은 장면에서 semantic obstacle가 0점일 수는 있지만 CARLA raw source가
stale/empty이면 audit가 실패한다.

CARLA GNSS/IMU를 hardware-shaped message로 표현할 때도 simulator가 제공하지 않는
satellite 수, DOP, RTK carrier solution을 만들어내지 않는다. rear docking fallback은
실제 CARLA rear RGB를 보여 주는 표시 fallback일 뿐 AprilTag detection으로 위장하지
않는다.

## 7. strict validation과 자료 무결성

Guest 최종 strict report는
[`site_evidence_collection.json`](evidence/virtual_carla/current/guest_recall/provenance/strict_validation/site_evidence_collection.json)이며
SHA-256은
`9a3f7dc5c24101abff3b9a30465423b6497476815ec0103b1dc93ae8fa2bfaac`다.
검증 결과는 `status=PASS`, `runner_status_pass=true`, `runner_prepass_validating=false`다.
즉 실행 도중의 임시 prepass가 아니라 13개가 끝난 뒤 final manifest를 다시 읽어
검증한 결과다.

Operator delivery strict JSON SHA-256은
`06ed5e0e80ef415f026b801c186f3057620432494767f4278d6a9632299f25fe`다.
Operator recall은 실제 실행을 B1~B5와 B6~B13 두 part로 나누었고 strict JSON
SHA-256은 각각
`8bac1fb053a090d6f80d9118cabbee420fcbff87d95ab7b300e943ffc82e32cf`,
`f0c381d9808471ff25e483374f25eaba8621a046eb88508e330a4a8a5cfbe5e2`다.
큐레이터가 site set의 중복/누락 없이 두 part를 13개로 합쳤다.

다음 명령은 큐레이션 시점에 생성한 root allowlist hash를 전부 확인한다.

```bash
cd /home/hong/camrod_ws/src/docs/evidence/virtual_carla/current
sha256sum -c SHA256SUMS

sha256sum guest_recall/provenance/strict_validation/site_evidence_collection.json
jq '{status, aggregate, validation}' \
  guest_recall/provenance/strict_validation/site_evidence_collection.json
```

개별 bundle 내부에서 다시 확인하려면 다음을 실행한다.

```bash
for bundle in operator_delivery operator_recall guest_recall manual_4ws; do
  (cd "/home/hong/camrod_ws/src/docs/evidence/virtual_carla/current/${bundle}" && \
    sha256sum -c SHA256SUMS)
done
```

## 8. 환경 준비와 정확한 실행 순서

승인 조합은 ROS 2 Humble, CARLA 0.9.15 source checkout, UE 4.26.x, Python 3.10,
Woraksan v15 site-access map이다. STEP→FBX→rigged FBX→Unreal Blueprint/PhysicsAsset는
독립 `ranger-carla-4ws-pipeline` 저장소가 준비하고, 이 런타임 절차에서는 이미 gate를
통과한 `vehicle.ranger.default`를 사용한다.

모든 terminal에 같은 환경을 적용한다.

```bash
cd /home/hong/camrod_ws/src
export RANGER_CARLA_ROOT=/home/hong/Downloads/ranger-carla-4ws-pipeline
export CARLA_ROOT=/home/hong/carla
export UE_ROOT=/home/hong/UnrealEngine_4.26
export RANGER_UE_ROOT=/home/hong/UnrealEngine_4.26
export CARLA_RENDER_MODE=onscreen
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority

./scripts/virtual_carla/prepare_yolo_engine.sh --verify-only --print-path
./scripts/virtual_carla/site_access.sh doctor
./scripts/virtual_carla/site_access.sh commands
```

그 다음 각 명령을 별도 terminal에서 순서대로 실행한다. `site_access.sh`는 v15
map/profile identity를 스스로 고정하며, 충돌하는 `CARLA_UE_MAP`, `CARLA_TOWN`,
profile override가 있으면 시작 전에 거부한다.

```bash
# Terminal 1
./scripts/virtual_carla/site_access.sh server

# Terminal 2
./scripts/virtual_carla/site_access.sh bridge

# Terminal 3
./scripts/virtual_carla/site_access.sh pacer

# Terminal 4
./scripts/virtual_carla/site_access.sh spawn

# Terminal 5
export CAMROD_ENABLE_OPERATOR_WINDOW=false
./scripts/virtual_carla/site_access.sh camrod-site-geometry
```

CAMROD가 healthy가 된 뒤 필요한 표시 UI 하나를 별도 terminal에서 연다.

```bash
# Robot UI 시나리오
./scripts/virtual_carla/site_access.sh operator-ui

# 또는 Guest 시나리오
./scripts/virtual_carla/site_access.sh guest-ui

# source/topic/actor 검증
./scripts/virtual_carla/site_access.sh audit-sensors

# terminal keyboard는 UI manual의 fallback이며 TTY에서만 실행
./scripts/virtual_carla/site_access.sh manual
```

`server`, `bridge`, `pacer`, `spawn`, `camrod-site-geometry`는 vehicle motion goal을
자동 전송하지 않는다. UI 또는 mission dispatcher를 명시 실행한 때만 움직인다.
종료는 manual/evidence → UI → CAMROD → spawn → pacer → bridge → server의 역순으로
수행한다.

## 9. PNG/GIF를 포함한 evidence 재수집 명령

현재 승인 디렉터리를 덮어쓰지 말고 항상 비어 있는 새 output root를 사용한다. 아래
옵션은 승인 실행과 같은 capture `1 fps`, GIF `8 fps`, derived width `1280`, wheel
telemetry `10 Hz`, source MP4 미보존 계약이다.

```bash
cd /home/hong/camrod_ws/src
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
SITES=B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11,B12,B13

# visible Robot UI: delivery + Return
./scripts/virtual_carla/run_site_evidence_matrix.sh run \
  --authority operator-browser --mission-intent delivery --sites "${SITES}" \
  --output-root /absolute/new/evidence/operator_delivery \
  --phase-timeout-s 900 --capture-fps 1 --gif-fps 8 \
  --derived-width 1280 --wheel-rate-hz 10 --retain-source-video false \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority

# visible Robot UI: recall + Return
./scripts/virtual_carla/run_site_evidence_matrix.sh run \
  --authority operator-browser --mission-intent recall --sites "${SITES}" \
  --output-root /absolute/new/evidence/operator_recall \
  --phase-timeout-s 900 --capture-fps 1 --gif-fps 8 \
  --derived-width 1280 --wheel-rate-hz 10 --retain-source-video false \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority

# visible Guest UI: recall + 이용 완료
./scripts/virtual_carla/run_site_evidence_matrix.sh run \
  --authority guest --mission-intent recall --sites "${SITES}" \
  --output-root /absolute/new/evidence/guest_recall \
  --phase-timeout-s 900 --capture-fps 1 --gif-fps 8 \
  --derived-width 1280 --wheel-rate-hz 10 --retain-source-video false \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority
```

실행 전에 명령과 runtime identity만 검사하려면 `run` 대신 `plan`을 사용한다. Guest
matrix 전에 `guest-ui`, Operator matrix 전에 `operator-ui`가 표시 중이고 CDP port가
ready여야 한다. matrix는 보이는 DOM에 pointer/keyboard event를 전달하며 REST/ROS를
우회해 motion을 직접 만들지 않는다.

offline source/package 계약은 다음으로 재검사한다.

```bash
cd /home/hong/camrod_ws/src
./scripts/virtual_carla/test.sh
```

최종 실행에서 확인한 핵심 결과는 runner/offline pytest `142 passed`,
`camrod_carla_adapter` `488 passed`, `camrod_bringup` `347 passed`,
`camrod_control` `207 passed`이며 해당 실행에서 failure/error는 0이었다. 이 unit/contract
결과는 실제 mission E2E를 대신하지 않고, 위 strict mission·sensor·manual 자료와 함께
해석한다.

## 10. 자료 위치와 읽는 순서

1. 전체 요약: [current README](evidence/virtual_carla/current/README.md)
2. Operator delivery: [bundle README](evidence/virtual_carla/current/operator_delivery/README.md)
3. Operator recall: [bundle README](evidence/virtual_carla/current/operator_recall/README.md)
4. Guest recall: [bundle README](evidence/virtual_carla/current/guest_recall/README.md)
5. Guest 시간·거리: [metrics table](evidence/virtual_carla/current/guest_recall/summary/camping_site_metrics.md)
6. Guest strict validator: [strict report](evidence/virtual_carla/current/guest_recall/provenance/strict_validation/site_evidence_collection.md)
7. manual 4WS: [manual report](evidence/virtual_carla/current/manual_4ws/manual_4ws_report.md)
8. develop 대비 package 전수 감사: [parity audit](VIRTUAL_CARLA_DEVELOP_PARITY_AUDIT.md)
9. 전체 환경/실행 설명: [virtual CARLA guide](virtual_carla.md)

각 사이트의 `camping_site_matrix.json`은 state/phase/source/거리/시간/충돌을,
`site_manifest.json`은 artifact identity를, `wheel_summary/`는 FL/FR/RL/RR physical
readback을, `visual/`은 실제 PNG/GIF와 capture manifest를 제공한다. raw JSONL/MP4/log는
Git 용량을 위해 제외했으며, 그 원본의 SHA-256과 byte/line count는 manifest에 남겼다.

## 11. 승인 범위의 한계

- 이 결과는 `virtual/carla`의 Woraksan v15 CARLA 검증이다. 실차, Jetson 배포,
  실제 타이어/토양, 실제 solid-state LiDAR beam/noise/multipath에 대한 field PASS가 아니다.
- CARLA actor ID는 실행 lifecycle에 따라 달라질 수 있다. Operator/manual 자료의 actor
  `50`과 Guest 자료의 actor `66`이 다른 것은 별도 clean spawn lifecycle의 결과이며,
  각 bundle 내부에서는 actor/type/role이 일관된다.
- 세 mission은 실행 HEAD가 다르다. 최신 Guest snapshot이 앞선 Operator 화면 자료를
  소급해 다시 생성한 것으로 표현하지 않는다.
- `camrod-site-geometry`의 map/perception/parking/traction/Return opt-in은 ordinary
  develop 기본이 아니다. `run.sh camrod` 결과와 v27 site acceptance를 섞지 않는다.
- sensor `36/36`은 ownership, type, payload freshness/validity 및 actor attachment PASS다.
  센서의 실물과의 물리적 동등성까지 뜻하지 않는다.
- UI의 target steering gauge만으로 actual wheel 추종을 주장하지 않는다. manual/site
  bundle의 patched CARLA physical telemetry 및 wheel manifest를 acceptance 근거로 쓴다.

이 경계 안에서 최신 develop baseline을 포함한 CAMROD 알고리즘, production Robot/Guest
UI, 실제 CARLA sensor actor, physical Ranger 4WS backend의 B1~B13 delivery/recall/Guest
이용완료 및 Drop Zone 복귀는 최종 PASS다.
