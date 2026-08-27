# CAMROD `virtual/carla` 환경·빌드·실행 가이드

이 문서는 독립 Ranger/CARLA 자산 저장소에서 만든 4륜 조향 로봇을 CARLA에
올리고, CAMROD `virtual/carla` 브랜치의 알고리즘·production UI와 연결하는
재현 가능한 절차를 설명한다.

CAMROD 저장소는 STEP/FBX/Unreal 자산을 다시 만들지 않는다. 반대로 Ranger
저장소는 CAMROD 알고리즘 소스를 복제하지 않는다. 두 저장소는
`RANGER_CARLA_ROOT`와 각각의 ROS install prefix만으로 연결하며 Git submodule을
사용하지 않는다.

## 1. 저장소별 책임

### Ranger/CARLA 자산 저장소

독립 `ranger-carla-4ws-pipeline` 저장소는 다음 준비 단계를 소유한다.

1. `RANGER.step` CAD 원본과 provenance 보존
2. 승인된 CAD 도구에서 STEP을 tessellate/export해 raw FBX checkpoint 생성. 이 수동
   checkpoint는 현재 bootstrap이 자동화하거나 성공으로 추정하지 않음
3. Blender Python의 `asset_pipeline/convert_scripts/01_rig_ranger_for_carla.py`가 raw FBX를
   입력으로 받아 chassis와 네 wheel bone을 구성하고 rigged FBX 생성
4. `asset_pipeline/convert_scripts/02_verify_rigged_fbx.py`로 bone, transform, wheel 축과 FBX
   계약 검증
5. Unreal Python/C++ 도구로 skeletal mesh, PhysicsAsset, wheel class와
   `vehicle.ranger.default` Blueprint 생성·검증
6. CARLA 0.9.15/UE 4.26의 물리 4륜 조향 API와 Ranger ROS 메시지·제어기 빌드
7. 현재 binary/egg/Blueprint와 일치하는 fail-closed gate 생성

STEP→FBX와 FBX→Blueprint는 CARLA 런타임 때 반복하지 않지만, 실행 가능한 로봇
자산을 만드는 필수 준비 단계다. 이 단계의 현재 build/run 계약은 Ranger 저장소의
`docs/BUILD_AND_RUN.md`, runtime gate 계약은 `docs/RUNTIME_GATE_CONTRACT.md`, 전체
구성은 루트 README를 따른다.

### CAMROD `virtual/carla` 브랜치

이 저장소는 다음 런타임 통합을 소유한다.

- `camrod_carla_adapter`: CAMROD Twist, CARLA extended 4WS command, odometry,
  GNSS/IMU, camera/LiDAR topic 경계
- `camrod_carla_adapter/launch/camrod_carla_full.launch.py`: Ranger physical
  controller + adapter + 전체 CAMROD bringup + production UI 구성
- `scripts/virtual_carla`: portable 환경, setup, build, test, 명시적 실행 진입점
- `colcon_build.sh`: CAMROD 전체 colcon build 및 production UI frontend build

## 2. 호환성 범위와 백엔드 프로파일

이 절은 2026-08-27의 `origin/develop`과 `virtual/carla`를 비교한 호환성
감사 결과다. 여기서 `ACKERMANN_ONLY`, `PLANAR_TWIST`, `PHYSICAL_4WS`는
**호환성 분류 이름**이지 현재 `run.sh`의 subcommand나 launch enum이 아니다.
현재 checked-in 실행 스크립트가 구현하는 것은 `PHYSICAL_4WS`뿐이며, 실제
controller launch 값은 `extended_mode_backend:=PHYSX_FOUR_WHEEL_STEERING`이다.
다른 두 분류 이름을 현재 CLI에 전달하면 안 된다.

### 2.1 감사 기준과 core/overlay 원칙

| 대상 | 감사 기준 | 확인된 범위 |
|---|---|---|
| CAMROD core | `origin/develop` `9756edf2d7aebf59cf8b635b5d8f5982bf6211aa` | 아래 표준 ROS 경계는 존재하지만 `camrod_carla_adapter`와 `scripts/virtual_carla`는 없음 |
| CARLA overlay | `virtual/carla` 작업 트리(병합 전 원격 기준 `0b4adc7be455f280ee2340faca4c5288fed34ccf`) | 위 `origin/develop`을 병합하고 별도 adapter package, opt-in external-simulator bringup, physical 4WS runner를 추가 |
| ROS bridge pin | upstream `c596934b430173a5713bc1ac191ff23ae8df9686` + Ranger patch set | stock topic/message 경계와 Ranger actor/PhysX lifecycle patch를 함께 사용 |
| 현재 검증 조합 | ROS 2 Humble, CARLA 0.9.15 source, UE 4.26.x, Python 3.10 | 다른 ROS/CARLA/UE 조합은 이 문서가 승인하지 않음 |

통합 원칙은 CAMROD core의 기본 topic과 알고리즘을 CARLA 전용으로 바꾸지 않고,
시뮬레이터 변환을 adapter overlay에서 끝내는 것이다. `virtual/carla`의 core 파일이
`origin/develop`과 byte-for-byte 동일한 것은 아니다. external simulator를 위해
`external_simulator`, external odometry와 fake-sensor rate 인자가 추가돼 있지만,
ordinary bringup 기본값은 각각 `false`, 내부 `cmd_vel` motion source와 기존 platform
status owner를 유지한다. 즉 **기본 동작은 유지하고 CARLA full launch가 opt-in 인자를
명시적으로 켠다**는 계약이다.

병합 감사에서 최신 `develop`의 중복 parking YAML 두 개가 서로 달라 자체 mirror
계약이 실패하는 문제도 확인했다. 이 브랜치는 실제 full bringup의 최신 heading/lateral
gain(`0.9`/`1.5`)을 두 파일에 동일하게 유지하고, C++ 기본값과 robot-center frame
계약에 맞지 않던 deprecated `parked_distance_from_tag_m`만 `0.943`으로 동기화한다.
이는 CARLA 전용 tuning이 아니며 두 실행 경로의 upstream 설정 drift 수정이다.

`origin/develop`에서 확인한 stable ROS 경계는 다음과 같다.

| 용도 | CAMROD topic | ROS type | 소유자/조건 |
|---|---|---|---|
| Nav2 후보 명령 | `/control/nav2_cmd_vel_ros` | `geometry_msgs/msg/Twist` | planning → safety gate; simulator가 직접 구독하면 gate를 우회함 |
| 최종 actuator 명령 | `/control/cmd_vel_ros` | `geometry_msgs/msg/Twist` | safety gate 출력; profile adapter가 여기서만 분기 |
| 표준 odometry 입력 | `/odom` | `nav_msgs/msg/Odometry` | external simulator 또는 platform driver → `ranger_platform_bridge` |
| 정규화 platform odometry | `/platform/status/odometry` | `avg_msgs/msg/AvgOdometry` | CAMROD 내부 경계; `ranger_platform_bridge`가 `/odom`을 변환 |
| IMU | `/sensing/imu/data_ros` | `sensor_msgs/msg/Imu` | frame, covariance, clock와 freshness를 함께 맞춰야 함 |
| GNSS fix | `/sensing/gnss/ublox_gps_node/fix` | `sensor_msgs/msg/NavSatFix` | datum/좌표계와 covariance가 필요함 |
| raw LiDAR | `/sensing/lidar/vanjee/points_raw` | `sensor_msgs/msg/PointCloud2` | point fields, frame/TF와 rate를 검증해야 함 |
| camera | `/sensing/camera/econ_{front,rear}/*` | `sensor_msgs/msg/Image`, `CameraInfo` 또는 `CompressedImage` | production consumer가 요구하는 raw/rect/compressed 형태가 서로 다름 |

### 2.2 백엔드 호환성 행렬

| 프로파일 | command mapping | feedback mapping | stock CARLA/ROS bridge | 현재 구현 및 제한 |
|---|---|---|---|---|
| `ACKERMANN_ONLY` | final `Twist` → `ackermann_msgs/msg/AckermannDrive` → `/carla/<role>/ackermann_cmd` | `/carla/<role>/odometry` `nav_msgs/msg/Odometry`; controller는 `/vehicle_status` `carla_msgs/msg/CarlaEgoVehicleStatus`와 `/vehicle_info` `carla_msgs/msg/CarlaEgoVehicleInfo`도 사용 | **가능**. stock `carla_ackermann_control`의 단일 steering/throttle/brake 경로 | 이 저장소에는 해당 Twist→stock-Ackermann adapter/runner가 아직 없음. `linear.y`, crab, zero-turn, pivot, 독립 rear steer와 wheel별 torque/telemetry는 표현할 수 없음 |
| `PLANAR_TWIST` | `/control/cmd_vel_ros`의 `linear.x`, `linear.y`, `angular.z`를 simulator-native planar base에 전달 | map-aligned `nav_msgs/msg/Odometry`와 필요한 TF/sensor를 `/odom` 및 CAMROD 표준 경계로 반환 | **stock wheeled CARLA에는 불가**. stock bridge에는 holonomic vehicle용 planar `Twist` actuator가 없음 | 범용 simulator용 계약일 뿐, 현재 CARLA runner는 없음. simulator가 lateral translation과 in-place yaw를 명시적으로 지원할 때만 crab/zero-turn의 kinematic 시험 가능; wheel 물리 4WS 증거는 아님 |
| `PHYSICAL_4WS` | final `Twist` → `carla_extended_ackermann_msgs/msg/ExtendedAckermannDrive` → `PhysicalFourWheelControl` → custom CARLA wheel API | standard odometry/sensor topics + `/carla/<role>/physical_four_wheel_status`; startup에 read-only PhysX substep snapshot 확인 | **제어는 불가**. stock bridge topic은 feedback에 재사용할 수 있지만 unmodified CARLA/bridge는 physical gate를 통과하지 못함 | 현재 `run.sh`가 구현한 유일한 profile. exact Ranger Blueprint, patched bridge/CARLA, custom messages/controller, 두 runtime gate가 모두 필요. actual wheel angle/torque의 continuous ROS feedback은 현재 없음 |

stock Ackermann은 “CARLA와 CAMROD가 연결된다”는 최소 주행 경로이지 Ranger의
4WS 동작과 동등하지 않다. `ackermann_msgs/msg/AckermannDrive`에는 lateral velocity,
rear steering, drive mode 또는 wheel별 torque가 없다. 따라서 `linear.y`가 deadband를
넘거나 CAMROD semantics가 crab/zero-turn/pivot을 요구하면 stop을 내고 diagnostic을
올려야 한다. 이를 front-steer 포화값으로 근사하거나 일반 회전으로 조용히 바꾸면
안 된다. 허용된 Dual-Ackermann 구간도 stock front-steer 차량의 wheelbase, steering
sign, 속도/곡률 한계로 별도 변환·시험해야 한다.

`PLANAR_TWIST`는 ROS message 수준에서 가장 단순하지만 CARLA physical vehicle의
보편 기능이 아니다. actor pose를 순간이동시키거나 physics를 우회해 `Twist`를
흉내 낸 결과는 navigation 알고리즘의 kinematic 시험에는 쓸 수 있어도
`PHYSICAL_4WS` 또는 4WD traction 시험으로 기록하면 안 된다.

### 2.3 profile별 필수 mapping

모든 profile은 다음 연결을 명시적으로 소유해야 한다. topic 이름이 같아도 type,
frame, clock 또는 QoS가 다르면 호환으로 간주하지 않는다.

| 경계 | source → destination | 필수 조건 |
|---|---|---|
| 안전 명령 | Nav2 `/control/nav2_cmd_vel_ros` 또는 CARLA 전용 수동 `/control/manual_cmd_vel_ros` → CAMROD safety gate → `/control/cmd_vel_ros` | engage/mission/estop, command owner, stale timeout과 zero publish를 유지; actuator adapter를 후보 topic에 직접 연결하지 않음 |
| `ACKERMANN_ONLY` actuation | `/control/cmd_vel_ros` `Twist` → **추가 구현할 adapter** → `/carla/<role>/ackermann_cmd` `AckermannDrive` | only supported Dual-Ackermann input; `linear.y` 및 zero-turn/pivot 분류는 reject+stop; current `twist_to_4ws_node.py`는 message type이 달라 재사용/단순 remap할 수 없음 |
| `PLANAR_TWIST` actuation | `/control/cmd_vel_ros` `Twist` → simulator planar command | simulator가 지원 축, body frame, 단위와 sign을 preflight에서 광고/검증; capability가 없으면 nonzero publish 금지 |
| `PHYSICAL_4WS` actuation | `/control/cmd_vel_ros` → `/carla/<role>/extended_ackermann_cmd` → `/carla/<role>/physical_four_wheel_cmd` | `ExtendedAckermannDrive`/`PhysicalFourWheelControl` exact type, monotonically increasing sequence, `PhysicalFourWheelStatus` acknowledgement와 watchdog 필요 |
| odometry | simulator odom → map alignment → `/odom` `nav_msgs/msg/Odometry` → `/platform/status/odometry` `AvgOdometry` | pose와 twist가 finite, quaternion normalized, `map`/`odom`/`robot_center_link` ownership이 단일하며 covariance와 freshness가 유효해야 함 |
| identity | configured actor/robot → `<role>` topic namespace | 모든 CARLA topics가 같은 role을 사용. physical profile은 정확히 하나의 `vehicle.ranger.default`와 일치하는 actor ID까지 요구 |
| sensor | simulator Image/CameraInfo/PointCloud2/Imu/NavSatFix → CAMROD canonical topics | calibration, optical/body frames, TF, timestamp, QoS, field layout와 consumer rate를 각각 검증 |
| map | simulator world pose → `CAMROD_MAP_ALIGNMENT_FILE` → `CAMROD_LANELET_MAP` | spawn pose, SE(2) alignment와 lanelet map을 한 cohort로 승인; 다른 map/spawn에 기존 alignment 재사용 금지 |

`sensor_relay_node.py`는 CARLA Image/CameraInfo/PointCloud2를 CAMROD canonical topic과
frame으로 copy/restamp한다. CARLA RGB camera는 distortion을 설정하지 않은 pinhole
sensor이므로 동일 payload를 `image_raw`와 `image_rect` 경계에 제공한다. CARLA 전용
relay 노드가 같은 callback 안에서 JPEG quality 80으로 front
`image_rect/compressed`와 rear `image_raw/compressed`도 만든다. 별도 republish 프로세스가
없어 원시·압축 frame의 수명과 종료 순서도 하나로 유지된다. 실제 렌즈 calibration이나
distortion이 있는 다른 camera에 이 무변환 rectified 계약을 재사용하면 안 된다.
LiDAR relay는 point field 자체를 변환하지 않으며, canonical mount를 사용한 visual
profile에서 header를 `lidar_link`로 고정한다. current full launch는 CARLA IMU/GNSS
relay를 꺼 두고 map-aligned `/odom`에서 10 Hz fake GNSS/IMU/readiness data를 파생한다.
relay의 raw/JPEG/LiDAR stale 또는 JPEG 변환 오류는 전용 status와 표준 `/diagnostics`에
동시에 발행한다.

### 2.4 capability fail-closed 계약

backend 선택은 자동 fallback이 아니다. 시작 전에 profile을 하나 고정하고 아래
capability가 전부 확인된 뒤에만 nonzero command를 허용한다.

- `ACKERMANN_ONLY`: exact `/ackermann_cmd` type과 subscriber, `/vehicle_status`,
  `/vehicle_info`, fresh odometry, 차량 geometry/steer limit를 확인한다. unsupported
  planar component나 4WS mode가 오면 zero를 발행하고 상태를 ERROR로 만든다.
- `PLANAR_TWIST`: simulator가 `linear.x`, `linear.y`, `angular.z` 각각과 body-frame
  convention을 지원한다고 명시해야 한다. 표준 capability handshake가 없으므로
  deployment config와 bounded preflight test가 둘 다 없으면 fail closed한다.
- `PHYSICAL_4WS`: baseline/physical manifest deep validation, gate-bound egg/libcarla,
  exact Blueprint/role/단일 actor, `set_wheel_physics_steer_angles_and_drive_torques`,
  `reset_wheel_physics_steer_angles`, `get_wheel_physics_telemetry`, PhysX substep
  telemetry와 `/physical_four_wheel_status`의 `ready=true`, accepted gate,
  `independent_wheel_drive_available=true`를 모두 요구한다.
- 어떤 profile도 feedback가 stale하거나 command/type/frame validation이 실패했을 때
  마지막 nonzero 명령을 replay하지 않는다. zero publish와 backend별 brake/reset을
  수행하고 operator가 원인을 해결한 뒤 명시적으로 다시 승인한다.
- `PHYSICAL_4WS` 요청이 실패해도 `ACKERMANN_ONLY`로 내려가 움직이지 않는다. 그런
  fallback은 Ranger 4WS 검증 범위를 바꾸므로 별도 operator 선택과 별도 test report가
  필요하다.

### 2.5 hard dependency 경계

| 프로파일 | 반드시 필요한 것 | 필요하지 않거나 현재 없는 것 |
|---|---|---|
| `ACKERMANN_ONLY` | audited CARLA/bridge version pair, `ackermann_msgs`, `carla_msgs`, stock `carla_ackermann_control`, ego role, odometry pseudo-sensor, 별도 Twist→Ackermann adapter와 profile preflight | custom wheel RPC와 physical gate는 필요 없음. 그러나 이 저장소에는 adapter/runner가 아직 없음 |
| `PLANAR_TWIST` | ROS 2 `geometry_msgs/msg/Twist`, map-aligned `nav_msgs/msg/Odometry`, TF/sensor contract를 제공하는 simulator plugin과 명시적 capability config | CARLA, UE, Ackermann 또는 Ranger wheel API는 필수가 아님. 현재 이 profile용 runner/preflight는 없음 |
| `PHYSICAL_4WS` | Ranger pipeline의 exact UE/CARLA/Blueprint, patched ROS bridge, gate-bound Python egg/libcarla, custom message/controller/rqt overlay, baseline+physical gates, aligned map/spawn, live physical status | stock packaged CARLA나 unmodified upstream bridge로 대체할 수 없음 |

현재 `camrod_carla_adapter/package.xml`은
`carla_extended_ackermann_control`과 `carla_extended_ackermann_msgs`를 unconditional
runtime dependency로 선언하고, `run.sh doctor`도 physical gates와 Ranger packages를
항상 요구한다. 따라서 feedback node 일부가 standard message만 쓰더라도 현재 package와
CLI 전체를 stock-only 또는 planar generic adapter라고 볼 수 없다. 두 profile을 실제로
지원하려면 physical package와 분리된 adapter package/launch, profile별 dependency와
preflight를 추가해야 한다.

### 2.6 current `virtual/carla` bringup의 고정 전제

현재 `camrod_carla_full.launch.py`와 `run.sh`는 generic launcher가 아니라 다음을
명시한 하나의 deployment profile이다.

- Ubuntu 22.04/ROS 2 Humble/Python 3.10, CARLA 0.9.15 source와 UE 4.26.x
- patched ROS bridge pin과 Ranger ROS overlay의
  `carla_extended_ackermann_{msgs,control}`, `rqt_extended_ackermann`
- `vehicle.ranger.default`, role `ego_vehicle`, endpoint `127.0.0.1:2000`; physical
  bridge는 이 endpoint 외 값을 현재 거부함
- `sim=true`, `external_simulator=true`, external odometry `/odom`, timeout `0.5 s`
- hardware Ranger driver off, `ranger_platform_bridge` on, fake platform status owner off
- CAMROD readiness용 fake sensor rate `10 Hz`; platform heartbeat의 `5 Hz`와 별도
- adapter node는 wall/reception time(`use_sim_time=false`)을 사용하고 CARLA bridge는
  synchronous `0.05 s` step을 기본 사용하므로 모든 terminal의 clock 선택을 섞지 않음
- Woraksan CARLA map, lanelet map, checked-in single-anchor SE(2) alignment와 exact
  spawn pose. 다른 map은 alignment 재승인이 필요함
- production UI에는 Node.js/npm build가 추가로 필요하지만 actuator ROS 경계 자체의
  필수 dependency는 아님

`ROS_DOMAIN_ID=188`, `rmw_cyclonedds_cpp`, synchronous mode와 UI port `8010`은 현재
script 기본값이다. 프로토콜의 보편 상수는 아니지만 같은 run의 모든 terminal에서
일치해야 한다. 이 pin 또는 topic/type 계약을 바꾼 조합은 `doctor`의 physical PASS나
과거 evidence를 재사용하지 말고 profile별 build/interface/live 시험을 새로 수행한다.

## 3. PHYSICAL_4WS 전체 데이터 흐름

```text
UE 4.26 / CARLA 0.9.15 custom map
  ├─ vehicle.ranger.default Blueprint + four-wheel PhysX
  └─ CARLA Python physical-4WD v2 API
            │
            ├── pinned carla_ros_bridge ── camera/LiDAR/IMU/GNSS/odometry
            │                              │
            │                     camrod_carla_adapter
            │                              │
            │       CAMROD sensing/localization/map/planning/control/UI
            │                              │
            └── physical_four_wheel_bridge ← extended 4WS command
                         ↑
             carla_extended_ackermann_control
                         ↑
        /carla/<role>/extended_ackermann_cmd
                         ↑
          twist_to_4ws ← /control/cmd_vel_ros
```

핵심 제어 경로는 다음과 같다.

1. CAMROD planning/control이 `/control/cmd_vel_ros`를 생성한다.
2. `twist_to_4ws_node.py`가 planar Twist를 Dual Ackermann, crab 또는 zero-turn
   명령으로 변환한다. lateral/yaw deadband, 조향·속도 제한과 watchdog은
   `command_mapping.py` 계약을 따른다.
3. Ranger `carla_extended_ackermann_control`이 extended command를 wheel별 목표로
   계산한다.
4. `physical_four_wheel_bridge`가 gate로 승인된 CARLA Python egg의
   `set_wheel_physics_steer_angles_and_drive_torques` API를 사용한다.
5. CARLA odometry와 controller/physical bridge status가 다시 CAMROD로 들어온다.
   bridge는 시작 시 read-only PhysX substep telemetry를 한 번 확인하지만, 현재 ROS
   message는 네 wheel의 실제 steering angle/torque를 연속 측정해 발행하지 않는다.

`camrod_carla_full.launch.py`는 CARLA server, ROS bridge 또는 actor를 소유하지
않는다. 프로세스 수명과 실패 범위를 분리하기 위해 네 단계를 각각 다른 터미널에서
시작한다.

## 4. 기준 환경

검증 기준 조합은 다음과 같다.

- Ubuntu 22.04, x86_64
- ROS 2 Humble
- Python 3.10
- CARLA 0.9.15 source build와 프로젝트에 포함된 custom Woraksan map
- Unreal Engine 4.26.x
- Humble용 `carla_ros_bridge` pin `c596934`와 Ranger lifecycle patch set
- Node.js/npm: CAMROD production UI frontend build용
- rendered 시험: 정상 NVIDIA driver와 Vulkan

`nullrhi`는 GPU 없이 physics/control/Nav2 경로를 진단할 때만 사용한다. RGB,
camera UI, YOLO/perception 또는 시각 품질을 검증하지 못한다. `offscreen`도 실제
렌더링이므로 NVIDIA/Vulkan이 필요하다.

## 5. portable 디렉터리 계약

Ranger bootstrap이 기본적으로 만드는 런타임 트리는 다음과 같다.

```text
$RANGER_CARLA_ROOT/
  config/environment.env             # machine-local, untracked selections
  ros_ws/                             # Ranger 4WS ROS packages
  .work/
    src/carla/                        # custom CARLA source/project
    src/UnrealEngine_4.26/            # UE 4.26
    ros-bridge-ws/                    # pinned bridge + Ranger lifecycle patch
    evidence/
      ranger_ros_backend_gate.json
      ranger_physical_4ws_acceptance_gate.json
```

`scripts/virtual_carla/env.sh`는 `config/environment.env`가 있으면 먼저 읽는다.
호출자가 export한 값은 그 파일보다 우선한다. 주요 변수는 다음과 같다.

| 변수 | 기본값/역할 |
|---|---|
| `RANGER_CARLA_ROOT` | 유일한 필수 cross-repository anchor |
| `RANGER_WORK_ROOT` | `$RANGER_CARLA_ROOT/.work` |
| `RANGER_ROS_WS` | `$RANGER_CARLA_ROOT/ros_ws`; 과거 checkout은 `carla_ws` 호환 |
| `CARLA_ROOT` | `$RANGER_WORK_ROOT/src/carla` |
| `RANGER_UE_ROOT` / `UE_ROOT` | bootstrap이 선택한 licensed UE checkout; 기본은 `$RANGER_WORK_ROOT/src/UnrealEngine_4.26` |
| `RANGER_ROS_BRIDGE_WS` / `CARLA_ROS_BRIDGE_WS` | `$RANGER_WORK_ROOT/ros-bridge-ws`; 같은 install을 가리키는 alias |
| `RANGER_ROS_BRIDGE_SOURCE` | `$RANGER_ROS_BRIDGE_WS/src/ros-bridge`; actor authorizer가 재검증하는 source checkout |
| `RANGER_EVIDENCE_ROOT` | `$RANGER_WORK_ROOT/evidence` |
| `RANGER_CARLA_PYTHON_EGG` | Ranger gate가 고정한 CARLA egg |
| `CARLA_PYTHON_EGG` | 위 egg와 동일해야 하는 CAMROD launch alias |
| `RANGER_PYTHON_EGG_CACHE` | 실행마다 새로 만든 빈 절대 경로 |
| `RANGER_SPAWN_FILE` | actor/sensor JSON; rendered mode는 정렬된 `ranger_spawn_camrod_full_sensors.json`, `nullrhi`는 control-only smoke가 기본 |
| `CAMROD_LANELET_MAP` | CAMROD lanelet2 `.osm` |
| `CAMROD_MAP_ALIGNMENT_FILE` | CARLA↔CAMROD SE(2) alignment YAML |
| `CARLA_RENDER_MODE` | `offscreen`, `onscreen`, `nullrhi` |

portable gate 파일이 없으면 실행은 실패한다. `reports/`의 과거 timestamped JSON을
자동 fallback으로 쓰지 않는다. CARLA/UE binary, Blueprint, physical controller,
Python egg를 다시 빌드하거나 위치를 바꿨다면 Ranger 저장소에서 gate를 새로 만들고
검증해야 한다. egg 별칭 또는 해시가 다르면 fail closed가 정상이다.

## 6. 최초 환경 준비와 빌드

먼저 Ranger 저장소의 `docs/BUILD_AND_RUN.md`에 따라 UE, custom CARLA, ROS bridge,
`ros_ws/src`, portable gate와 `config/environment.env`를 준비한다. 기본 work root는
repository 내부의 `$RANGER_CARLA_ROOT/.work`이며, 더 큰 build disk가 필요할 때만
`RANGER_WORK_ROOT`를 절대경로로 override한다. 이 작업은 매우 크고 오래 걸릴 수
있으며 CAMROD 스크립트는 이를 대신 다운로드하거나 컴파일하지 않는다.

```bash
cd /data/ranger-carla-4ws-pipeline
export RANGER_CARLA_ROOT="$(pwd)"
cp -n config/environment.example config/environment.env
# licensed RANGER_UE_ROOT와 필요 시 큰 disk의 RANGER_WORK_ROOT를 편집한다.
${EDITOR:-vi} config/environment.env
./bootstrap.sh doctor
./bootstrap.sh plan
./bootstrap.sh install-host
# 동일 pin의 UE에 Ranger patch가 이미 적용된 기존 checkout이면 먼저:
# ./bootstrap.sh adopt-ue
./bootstrap.sh fetch
./bootstrap.sh build
./bootstrap.sh test
./bootstrap.sh prepare-runtime
./bootstrap.sh gate-plan
# gate-plan이 출력한 UE audit, drive, 6 PNG + 사람의 review, baseline gate,
# live 4WS acceptance, physical gate 순서를 전부 수행한 뒤:
./bootstrap.sh create-baseline
./bootstrap.sh verify-baseline
./bootstrap.sh create-gate
./bootstrap.sh verify-gate
./bootstrap.sh verify-runtime
```

그 다음 CAMROD source workspace에서 실행한다.

```bash
cd /data/camrod_ws/src
git switch virtual/carla
# ~/.bashrc 등에 남은 legacy CARLA package 경로가 config보다 우선하지 않게 한다.
unset CARLA_ROOT UE_ROOT RANGER_UE_ROOT CARLA_PYTHON_EGG \
  RANGER_CARLA_PYTHON_EGG CARLA_ROS_BRIDGE_WS RANGER_ROS_BRIDGE_WS
export RANGER_CARLA_ROOT=/data/ranger-carla-4ws-pipeline
```

의존성 setup:

```bash
./scripts/virtual_carla/setup.sh
```

`setup.sh`는 기존 `setup_camrod.sh`를 안전하게 호출한다. 이미 선택한 bridge/Ranger
prefix가 다른 ambient Autoware/CARLA overlay에 가려지지 않았는지 호출 전후에
확인한다. 시스템 패키지 설치를 제외하려면 다음처럼 rosdep만 건너뛸 수 있다.

```bash
./scripts/virtual_carla/setup.sh --no-rosdep
```

ROS build:

```bash
./scripts/virtual_carla/build.sh
```

기본 build 순서는 고정되어 있다.

1. `$RANGER_ROS_WS/src`의 `carla_extended_ackermann_msgs`
2. `carla_extended_ackermann_control`
3. `rqt_extended_ackermann`
4. CAMROD의 canonical `colcon_build.sh`
5. `camrod_ui` npm frontend와 `camrod_carla_adapter`를 포함한 CAMROD install

단계별 재빌드는 다음과 같다.

```bash
./scripts/virtual_carla/build.sh --ranger-only
./scripts/virtual_carla/build.sh --camrod-only
./scripts/virtual_carla/build.sh --camrod-only \
  --packages-up-to camrod_carla_adapter camrod_ui
```

CAMROD build wrapper는 `CAMROD_EXTRA_PREFIX_ROOTS`에 명시된 bridge와 Ranger install
하위만 보존한다. 임의의 ambient overlay는 제거한다. 따라서 터미널에 다른 Autoware
workspace가 source되어 있어도 package가 조용히 잘못 resolve되는 것을 방지한다.

## 7. 오프라인 시험

빠른 source/스크립트 계약 시험:

```bash
./scripts/virtual_carla/test.sh --source-only
```

adapter만 시험:

```bash
./scripts/virtual_carla/test.sh --adapter-only
```

CARLA 통합 관련 Ranger와 CAMROD package 시험:

```bash
./scripts/virtual_carla/test.sh
```

현재 build된 CAMROD 전체 시험:

```bash
./scripts/virtual_carla/test.sh --all-camrod
```

이 시험들은 server를 시작하거나 actor를 spawn하거나 motion/Nav2 goal을 보내지
않는다. shell 문법, host 절대경로 금지, gate alias, launch forwarding, 4WS mapping,
watchdog, feedback, 관련 CAMROD regression을 확인한다. 또한 `.gitmodules`와 Git
mode `160000` 항목이 CAMROD에 다시 들어오면 실패한다.

## 8. 런타임 사전 점검

경로와 package graph를 확인한다.

```bash
export RANGER_CARLA_ROOT=/data/ranger-carla-4ws-pipeline
export CARLA_RENDER_MODE=offscreen
./scripts/virtual_carla/run.sh doctor
```

`doctor`는 다음을 확인하고 하나라도 틀리면 non-zero로 종료한다.

resolved 환경과 `caller environment > RANGER_ENV_FILE > derived defaults` 우선순위를
오류보다 먼저 출력한다. template 경로나 packaged `CarlaUE4.sh` 경로가 source
checkout 자리에 들어오면 configuration 단계에서 중단한다. 필수 파일이나 새 gate가
없으면 뒤의 JSON/ROS/Python API/GPU 검사를 건너뛰므로, 같은 원인의 연쇄 오류나 Python
traceback을 정상 진단으로 오해하지 않는다.

- UE4Editor, CarlaUE4 project와 custom map `.umap`
- Ranger spawn JSON과 `ego_vehicle` role
- bridge, spawn, Ranger controller/rqt, CAMROD adapter/UI package prefix
- portable baseline/physical gate를 Ranger 저장소의 canonical Python
  validator로 재검증한 deep binding
- gate-bound CARLA egg와 physical 4WD v2 Python method
- lanelet map, SE(2) alignment, CAMROD launch defaults
- rendered mode의 NVIDIA 상태. `vulkaninfo`가 설치돼 있으면 Vulkan summary도
  필수 통과하며, 도구 자체가 없을 때만 경고 후 생략
- CARLA TCP port가 비었는지 또는 이미 사용 중인지

GPU 고장 상태에서 control-only 진단을 선택했다면 명시적으로 다음을 사용한다.

```bash
export CARLA_RENDER_MODE=nullrhi
export CARLA_WAIT_FOR_CONTROL_COMMAND=True
./scripts/virtual_carla/run.sh doctor
```

이 선택으로 camera/perception PASS를 주장하면 안 된다.

## 9. 실행 순서

복사 가능한 현재 환경 명령을 먼저 볼 수 있다. 이 명령은 아무 프로세스도 시작하지
않는다.

```bash
./scripts/virtual_carla/run.sh commands
```

각각 새 터미널에서 같은 `RANGER_CARLA_ROOT`와 필요한 override를 export한 뒤 다음
순서로 실행한다.

Terminal 1 — CARLA server:

```bash
./scripts/virtual_carla/run.sh server
```

`offscreen`은 `-RenderOffScreen`, `onscreen`은 직접 볼 수 있는 UE/CARLA window,
`nullrhi`는 `-nullrhi`를 선택한다. 이미 CARLA port가 listen 중이면 두 번째 server
시작을 거부한다.

Terminal 2 — pinned CARLA ROS bridge:

```bash
./scripts/virtual_carla/run.sh bridge
```

Rendered mode는 sensor payload가 CAMROD보다 먼저 준비돼야 하므로
`CARLA_SYNCHRONOUS_MODE=True`, `CARLA_WAIT_FOR_CONTROL_COMMAND=False`,
`0 < CARLA_FIXED_DELTA_SECONDS <= 0.1`을 강제한다. 예전 control-only 명령의
`CARLA_WAIT_FOR_CONTROL_COMMAND=True`를 rendered 시험에 재사용하면 시작 전 sensor
preflight와 tick이 서로 기다리므로 bridge 단계에서 거부한다.

Terminal 3 — Ranger actor와 sensor:

```bash
./scripts/virtual_carla/run.sh spawn
```

`offscreen`/`onscreen`의 기본 `RANGER_SPAWN_FILE`은 checked-in
`ranger_spawn_camrod_full_sensors.json`이다. 검증된 Woraksan ego pose는 control-only와
동일하고, CAMROD URDF 장착 좌표의 `rgb_view`, `rgb_rear`, `lidar_front`를 각각 10 Hz로
생성한다. `nullrhi`는 렌더링 센서를 검증할 수 없으므로 control-only JSON을 선택하고
CAMROD sensor relay도 시작하지 않는다.
다른 JSON으로 override하려면 accepted control sensor 전체(collision, lane invasion,
GNSS, IMU, odometry, control), 세 visual sensor의 type/rate/장착값, exact ego pose의
spawn/alignment cohort를 모두 만족해야 한다. generic Ranger sensor-suite의 `(0,0)`
pose로 단순 교체하고 기존 alignment로 navigation 성공을 주장해서는 안 된다.

Terminal 4 — physical 4WS + full CAMROD + production UI:

```bash
./scripts/virtual_carla/run.sh camrod
```

egg cache를 지정하지 않으면 `camrod`가 실행 전용의 새 빈 절대 경로를 만든다. 이미
내용이 있는 cache는 거부한다. 이 단계는 controller와 전체 알고리즘/UI를 시작하지만
motion을 자동 전송하지 않는다. rendered mode에서는 시작 전에 CARLA front/rear image,
두 CameraInfo와 LiDAR가 각각 1초 이상 관찰 구간에서 최소 8 Hz payload를 유지하는지도
bounded preflight로 확인한다. topic 이름만 존재하거나 one-shot/5 Hz이면 UI를 띄우지
않고 각 stream의 측정 rate를 출력한다.

선택 Terminal 5 — CAMROD safety gate를 통과하는 수동 키보드 조작:

```bash
./scripts/virtual_carla/run.sh manual
```

`manual`은 다음 조건을 모두 다시 확인한 뒤에만
`teleop_twist_keyboard`를 CARLA에서만 활성화되는 전용 입력
`/control/manual_cmd_vel_ros`로 remap한다. 첫 수동 Twist가 도착하기 전에는 기존
UI/Nav2 수동 goal 경로가 유지되며, 첫 Twist 이후에는 safety gate가 수동 입력 소유권을
고정해 Nav2/일반 raw 명령이 수동 명령을 덮어쓰지 못하게 한다.

- portable baseline/physical gate deep validation
- spawn JSON의 정확한 `vehicle.ranger.default`/role 계약
- CARLA world에 정확히 하나인 현재 Ranger actor
- `GET /ui/health`의 `ok=true`와 `GET /ui/state`의 `ready=true`,
  `engaged=false`, `mission_phase=READY`, `mission_source=none`
- `/carla/<role>/physical_four_wheel_status`의 `ready`, gate, PhysX substep,
  independent wheel drive, backend와 actor ID
- 현재 CARLA Ranger actor ID와 physical 4WS bridge가 고정한 actor ID가 정확히 동일함

명령 자체는 engage 또는 goal을 발행하지 않고, 키를 누르기 전에는 주행 명령도
발행하지 않는다. UI에서 먼저 기존 goal을 **STOP**으로 취소한 뒤, 마지막 페이지의
독립 **ENGAGE**를 눌러 수동 조작을 승인한다. B1~B13 목적지 버튼은 ENGAGE가 아니다.
mission ENGAGE가 켜지거나 독립 ENGAGE가 해제되면 safety gate는 즉시 zero를 내고 수동
소유권을 해제한다. 기본 속도는 `0.20 m/s`, 회전 속도는 `0.20 rad/s`다.
대각 조향 키의 `speed / turn = 1.0 m`는 Ranger 최소 회전 반경
`0.810330349 m`보다 크므로 Dual-Ackermann으로 분류된다. `j`/`l`처럼 선속도가
0인 키만 제자리 회전으로 분류된다.

| 키 | 동작 |
|---|---|
| `i` / `,` | 직진 / 후진 |
| `u`, `o` / `m`, `.` | 전진·후진 조향 |
| `j` / `l` | 제자리 좌회전 / 우회전 |
| `Shift+J` / `Shift+L` | 좌 crab / 우 crab |
| `k` 또는 미지정 키 | 정지 |
| `Ctrl-C` | zero를 보내고 teleop 종료 |

CARLA의 `PythonAPI/examples/manual_control.py`는 기존 ego에 attach하지 않고 새 actor와
camera/GNSS/IMU를 spawn하며 `VehicleControl.apply_control()`을 직접 호출한다. 따라서
동일 role actor 중복과 physical 4WS/CAMROD safety gate 우회를 만들 수 있으므로 이
full-stack 시험에는 사용하지 않는다.

선택 Terminal 6 — controller target/status monitor:

```bash
source ./scripts/virtual_carla/env.sh
virtual_carla_source_ros true true
ros2 run rqt_extended_ackermann rqt_extended_ackermann
```

이 rqt 화면의 wheel schematic과 front/rear steering gauge는
`ExtendedControlInfo.target_*`와 normalized controller output을 표시한다. 현재 widget은
front/rear gauge의 current와 target 양쪽에 같은 target 값을 넣으므로 actual wheel
encoder/PhysX angle 측정 화면이 아니다. speed/acceleration은 CARLA status/odometry에서
오지만 wheel별 actual torque도 표시하지 않는다. 따라서 이 UI만으로 “각 wheel의 실제
조향·torque가 명령을 추종했다”는 acceptance를 만들면 안 된다.

종료는 Terminal 6 → 5 → 4 → 3 → 2 → 1의 역순이다.

## 10. UI와 기능 확인

`camrod`가 정상 기동하면 기존 CAMROD production UI backend/frontend를 그대로
사용한다.

```text
http://127.0.0.1:8010
```

`CAMROD_ENABLE_OPERATOR_WINDOW=true`이면 같은 URL을 로봇 operator window로도
연다. 포트를 바꾸려면 `CAMROD_UI_PORT`와 `CAMROD_UI_URL`을 함께 맞춘다.

최소 관찰 항목은 다음과 같다.

- CARLA status와 ego odometry가 계속 갱신됨
- physical four-wheel bridge가 gate/egg/actor를 모두 승인함
- `/control/cmd_vel_ros`가 adapter watchdog 범위에서 처리됨
- `/carla/ego_vehicle/extended_ackermann_cmd`, controller target mode와 physical bridge
  sequence/status가 일치함. 이는 actual per-wheel angle/torque 측정과 구분함
- CAMROD map/localization/planning/control diagnostics가 stale/fault가 아님
- rendered full-sensor 시험에서 front/rear RGB와 LiDAR relay가 갱신됨
- CAMROD UI가 HTTP 200이고 현재 모듈 상태를 표시함

운영자가 UI 또는 Nav2 goal로 주행 시험을 시작하기 전 physical bridge의 armed 상태와
정지 watchdog을 먼저 확인한다. 이 저장소의 실행 스크립트는 안전상 직진, 회전,
crab, zero-turn 명령을 자동으로 보내지 않는다. `manual`도 키 입력을 기다리며 engage를
자동으로 켜지 않는다.

CAMROD의 GNSS/lidar/perception readiness 계약은 10 Hz다. CARLA full launch는 fake
sensor rate를 `sim_fake_sensor_publish_rate_hz=10.0`으로 명시하고, fake-sensor include는
heartbeat의 5 Hz launch 인자와 충돌하지 않는 전용 인자명을 사용한다. 실행 중 rate를
바꾸는 경우에도 fake sensor node는 기존 rclpy timer의 period를 함께 변경한다. 이전
빌드로 이미 떠 있는 프로세스에는 새 callback이 로드되지 않으므로 source 수정 후에는
CAMROD를 재빌드·재시작하고 `ros2 topic hz`로 새 주기를 확인한다.

## 11. 핵심 코드 위치

| 역할 | 파일/패키지 |
|---|---|
| Twist→4WS mode/limit/watchdog | `camrod_carla_adapter/src/camrod_carla_adapter/command_mapping.py`, `twist_to_4ws_node.py` |
| CARLA odometry/좌표 alignment | `feedback_bridge_node.py`, `config/woraksan_lane_anchor_alignment.yaml` |
| camera/LiDAR topic 변환 | `sensor_relay_node.py`, `sensor_relay.launch.py` |
| aligned visual sensor spawn/preflight | `config/ranger_spawn_camrod_full_sensors.json`, `scripts/virtual_carla/check_carla_sensor_streams.py` |
| simulated Ranger 상태 heartbeat | `carla_platform_heartbeat_node.py` |
| 전체 CAMROD/UI 조합 | `launch/camrod_carla_full.launch.py` |
| 부분 map/localization/planning 조합 | `launch/camrod_carla.launch.py` |
| wheel별 PhysX 제어 | Ranger ROS의 `carla_extended_ackermann_control` 및 custom CARLA API |
| CAMROD 알고리즘 | `camrod_sensing`, `camrod_localization`, `camrod_map`, `camrod_planning`, `camrod_control` |
| production UI | `camrod_ui`; frontend는 `colcon_build.sh`가 npm build |

## 12. 자주 발생하는 실패

- **package가 다른 workspace로 resolve됨**: 새 터미널에서 wrapper를 사용한다.
  `doctor`는 예상 install root 밖의 package를 거부한다.
- **portable gate 없음**: Ranger bootstrap/gate 생성 단계를 다시 수행한다. 과거
  timestamped report를 복사해 우회하지 않는다.
- **egg mismatch 또는 physical 4WD method 없음**: custom CARLA PythonAPI를 다시
  빌드하고 portable gate를 재생성한다.
- **CARLA는 켜졌지만 actor가 없음**: bridge 다음에 spawn을 실행하고 JSON의 actor
  `id`가 `CARLA_ROLE_NAME`과 같은지 확인한다.
- **actor는 생겼지만 physical status가 interlocked**: actor보다 CAMROD를 먼저 띄운
  경우다. CAMROD terminal만 종료하고 `server → bridge → spawn → camrod` 순서로 다시
  시작한다. `run.sh manual`은 `ready=false` 상태를 거부한다.
- **spawn만 재실행한 뒤 제어가 안 됨**: 새 actor ID와 기존 physical bridge binding이
  달라진 상태다. 실행 중인 CAMROD가 있으면 `spawn`을 다시 실행하지 않는다. 전체를
  역순으로 종료하고 정방향으로 재시작한다. `spawn`은 type과 무관하게 같은 role을 쓰는
  기존 actor가 하나라도 있으면 거부하고, `manual`은 두 actor ID가 다르면 fail closed한다.
- **UI readiness가 5 Hz/10 Hz 사이에서 흔들림**: 최신 source를 build한 뒤 CAMROD를
  재시작한다. heartbeat의 5 Hz가 fake sensors에 유출되던 launch-scope 충돌은 전용
  `fake_sensor_publish_rate_hz` 인자로 차단돼 있다.
- **UI는 보이나 camera가 stale/NO FRAME**: rendered 기본 visual JSON인지 확인하고,
  `camrod` preflight가 열거한 `/carla/ego_vehicle/{rgb_view,rgb_rear,lidar_front}` payload를
  확인한다. NullRHI/control-only 결과로 camera PASS를 주장하지 않는다.
- **rendered server 시작 실패**: `nvidia-smi -L`과 `vulkaninfo --summary`를 먼저
  고친다. `nullrhi` 결과로 rendered 검증을 대체하지 않는다.
- **map에서 경로가 어긋남**: CARLA spawn pose, lanelet map과 SE(2) alignment를 한
  세트로 다시 검증한다.

## 13. 증거 범위

[virtual CARLA 증거 인덱스](evidence/virtual_carla/README.md)에 PNG/GIF, UI
screenshot과 live report가 정리되어 있다.

- 시나리오 PNG/GIF는 제어·자세 데이터를 이용해 생성한 **기술 시각화**다. 실제
  CARLA camera 영상으로 간주하지 않는다.
- `camrod_carla_ui_latest_develop_20260825.png`는 당시 실행한 production UI의 실제
  screenshot이다.
- live/full-test JSON은 당시 프로세스와 topic을 관찰한 실제 보고서다.
- 이 과거 증거는 새 checkout의 gate 또는 새 rendered full-sensor 시험을 자동으로
  승인하지 않는다. 현재 실행 결과는 새 gate와 새 runtime report로 별도 기록한다.
