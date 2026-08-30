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
  실제 CARLA GNSS/IMU, camera/LiDAR/7채널 radar topic 경계와 source audit,
  gate-bound bridge의 공식 `/carla/control`을 사용하는 20 Hz step pacer
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
`external_simulator`, external odometry와 sensor-owner 선택 인자가 추가돼 있지만,
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
| radar | `/sensing/radar/{front1,front2,left1,left2,right1,right2,rear}/range` | `avg_msgs/msg/AvgRange` | 각 CARLA radar cloud에서 가장 가까운 유효 detection을 선택함 |

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

`woraksan_lane_anchor_alignment.yaml`은 Woraksan XODR의 Transverse Mercator
`geoReference`와 CAMROD lanelet 지도의 LocalCartesian 원점을 변환해 얻은 전역 SE(2)다.
따라서 특정 actor의 nominal root나 한 번 측정한 settle pose에 맞춘 1점 보정이 아니다.
현재 JSON spawn은 XODR Road 12/lane 2/s=2.0의 도로 중앙이며 변환 후 lanelet 751 내부,
drop zone 중심에서 약 3.8 m 앞이다. Blueprint root/rig를 다시 만들면 spawn 후 실제
odometry, 충돌, 차체 footprint를 다시 검증하되 전역 지도 정합을 actor drift에 맞춰
변형하거나 lanelet threshold를 낮춰 문제를 숨기면 안 된다.

`sensor_relay_node.py`는 CARLA Image/CameraInfo/PointCloud2를 CAMROD canonical topic과
frame으로 copy/restamp한다. CARLA RGB camera는 distortion을 설정하지 않은 pinhole
sensor이므로 동일 payload를 `image_raw`와 `image_rect` 경계에 제공한다. CARLA 전용
relay 노드가 같은 callback 안에서 JPEG quality 80으로 front
`image_rect/compressed`와 rear `image_raw/compressed`도 만든다. 별도 republish 프로세스가
없어 원시·압축 frame의 수명과 종료 순서도 하나로 유지된다. 실제 렌즈 calibration이나
distortion이 있는 다른 camera에 이 무변환 rectified 계약을 재사용하면 안 된다.
LiDAR relay는 point field 자체를 변환하지 않으며, canonical mount를 사용한 visual
profile에서 header를 `lidar_link`로 고정하고 실제 CARLA ray-cast cloud를 raw topic에만
전달한다. 그 뒤 CARLA 전용 `carla_lidar_filter`가 실제 점으로 local road plane을 robust
fit하고 plane보다 `0.08 m` 이상 높은 점만 filtered cloud로 남긴다. Ranger query
geometry에서 반복 측정한 좌우 self-return box도 이 단계에서만 제외하며, 정면
`|y| < 0.59 m` corridor는 유지한다. `obstacle_lidar_node`는 이 실제 nonground cloud를
Euclidean cluster해 `/perception/obstacles`를 소유한다. 즉 raw, filtered, obstacle은
같은 가짜 cloud의 alias가 아니라 하나의 CARLA 측정에서 순차 파생된 세 단계다. 맑은
장면에서 filtered/obstacle heartbeat가 0 points인 것은 정상일 수 있지만 CARLA raw
cloud까지 비어 있으면 실패다. CARLA full launch에서는 대응 fake/dummy publisher를
비활성화한다. relay의 raw/JPEG/LiDAR stale 또는 JPEG 변환 오류는 전용 status와 표준
`/diagnostics`에 동시에 발행한다.

실차 기본 `lidar/cost_grid.yaml`은 camera-LiDAR semantic association을 거친 obstacle
cloud를 전제로 한다. CARLA composition은 classifier가 꺼진 상태에서 위 Euclidean
cluster cloud를 사용하므로 `carla_lidar_cost_grid.yaml`을 safety raster에만 적용한다.
노면 제거가 앞 단계에서 끝났기 때문에 과거 raw-cloud `cloud_min_z_m=0.15` workaround는
제거했고, 낮은 실제 장애물을 보존하도록 production과 같은 `-0.55 m` lower bound를
복원했다. 이 전처리와 raster 설정은 CARLA profile에만 적용되며 일반 CAMROD/실차
parameter는 수정하지 않는다.

CARLA full launch는 `gnss`, `gnss_right`, `imu` actor를 실제 source로 사용한다.
`feedback_bridge_node.py`가 두 `NavSatFix`, CARLA odometry와 IMU를 CAMROD의 표준 fix/IMU와
hardware-shaped `NavPVT`, `NavCOV`, `NavRELPOSNED9` 화면 형식으로 변환한다. 이는 실제
CARLA 측정의 **표현 변환**이며 satellite 수, DOP, differential correction 또는 RTK
carrier solution을 만들어내지 않는다. 지원하지 않는 값은 0/no-solution으로 남는다.
7개 CARLA radar actor의 `PointCloud2.Range`는 `radar_relay_node.py`가 채널별 최근접
`AvgRange`/`sensor_msgs/Range`로 변환하며, target이 없을 때도 physical driver와 같은
`max_range + 1 mm` heartbeat를 보내 stale과 no-target을 구분한다.

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
- UI-visible fake GNSS/IMU/LiDAR/radar/velocity/dummy-grid owner는 모두 off. camera 2,
  LiDAR 1, GNSS 2, IMU 1, radar 7의 실제 CARLA actor 13개가 source임
- `lidar_front`는 360° 회전형이 아니라 전방 고정 solid-state 근사 profile이다.
  수평 FOV 120°(`-60..+60°`), 수직 FOV 35°(`-25..+10°`), 16 channel,
  60,000 points/s, `sensor_tick=0.1 s`를 사용한다. JSON의
  `rotation_frequency=20` 은 CARLA ray-cast가 각 frame에 고정 120° sector를 빠짐없이
  sampling하도록 맞추는 내부 속성이며, sensor actor가 360° 회전한다는 뜻이
  아니다. 실물 solid-state의 beam pattern·intensity·noise를 정확히 복제하는
  model은 아니고 FOV/cadence 수준의 근사임을 증거에 명시한다.
- rendered 부하를 고려한 CARLA diagnostics 기준은 `2 Hz`, sample stale 한계는 `3 s`;
  actor JSON의 sensor tick은 `0.1 s`로 구성됨
- adapter node는 wall/reception time(`use_sim_time=false`)을 사용하고 CARLA bridge는
  synchronous `0.05 s` step을 기본 사용하므로 모든 terminal의 clock 선택을 섞지 않음
- Woraksan CARLA map, lanelet map, georeference-derived SE(2) alignment와 exact
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
            ├── pinned carla_ros_bridge ── camera/LiDAR/7 radar/2 GNSS/IMU/odometry
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
| `CARLA_RENDER_MAX_FPS` | rendered Unreal 창 FPS 상한; 기본 `30`. 차량이 없어도 GPU를 100% 점유하는 무제한 렌더를 방지 |
| `CAMROD_CARLA_STEP_PACING` | rendered 기본 `True`; 기존 bridge를 수정하지 않고 `/carla/control`의 `PAUSE`/`STEP_ONCE`로 CARLA를 1x wall time에 맞춤. `nullrhi` 기본 `False` |
| `CAMROD_CARLA_STEP_PERIOD_SECONDS` | 기본 `0.05 s`; `CARLA_FIXED_DELTA_SECONDS`와 0.0001 s 이내로 같아야 함 |
| `CAMROD_CYCLONEDDS_CONFIG` | raw camera fragment용 checked-in `cyclonedds.xml` |
| `CAMROD_CARLA_SENSOR_MIN_RATE_HZ` / `CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS` | rendered preflight 기본 `2.0 Hz` / `3.0 s` |
| `CAMROD_CARLA_COMPRESSED_IMAGE_MAX_RATE_HZ` | UI용 JPEG wall-clock 상한; 기본 `5 Hz`, 구독자가 없으면 인코딩 자체를 생략 |
| `CAMROD_CARLA_RAW_IMAGE_MAX_RATE_HZ` | 실제 raw camera consumer가 있을 때의 wall-clock 상한; 기본 `10 Hz`, 구독자가 없으면 payload publish를 생략 |
| `CAMROD_MANUAL_{LINEAR,LATERAL,ANGULAR}_LIMIT_*` | UI slider 100%의 CARLA 수동 명령 envelope; 기본 `1.40 m/s`, `1.00 m/s`, `0.7853 rad/s` |
| `CAMROD_MANUAL_DEADMAN_TIMEOUT_S` | CARLA UI heartbeat lease 기본 `0.75 s`; 물리 명령 정지는 별도 safety/adapter `0.35 s` watchdog이 계속 담당 |

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

800x600 raw camera frame은 DDS UDP fragment가 크므로 최초 한 번 host socket buffer를
20 MiB 이상으로 설정한다. 이 값보다 작으면 작은 `CameraInfo`는 보이면서 실제 image만
멈출 수 있고, `doctor`, `bridge`, `spawn`, `camrod`, `manual`, `audit-sensors`가 모두
fail closed한다.

```bash
sudo tee /etc/sysctl.d/99-camrod-carla-dds.conf >/dev/null <<'EOF'
net.core.rmem_max=20971520
net.core.wmem_max=20971520
EOF
sudo sysctl --system
sysctl net.core.rmem_max net.core.wmem_max
```

설정 후에는 이전 ROS/CARLA terminal을 전부 종료하고 새 terminal에서 같은
`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI`를 사용한다.

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
시작을 거부한다. rendered 실행은 `t.MaxFPS=30`을 기본 적용한다. 이는 CARLA
simulation tick을 바꾸는 값이 아니라 Unreal 화면 렌더 상한이며, operator UI와
sensor 처리를 위한 GPU 여유를 남긴다.

Terminal 2 — pinned CARLA ROS bridge:

```bash
./scripts/virtual_carla/run.sh bridge
```

Rendered mode는 sensor payload가 CAMROD보다 먼저 준비돼야 하므로
`CARLA_SYNCHRONOUS_MODE=True`, `CARLA_WAIT_FOR_CONTROL_COMMAND=False`,
`0 < CARLA_FIXED_DELTA_SECONDS <= 0.1`을 강제한다. `fixed_delta_seconds`는 한 tick의
시뮬레이션 시간만 정하며 벽시계 속도를 제한하지 않는다. pacer는 bridge의 public
`/carla/control` 상태 전이 의미를 바꾸지 않고 unpaced active-sync bridge를 제어한다.
실행되는 bridge는 upstream pin에 Ranger lifecycle/world-step/종료 patch를 적용한 뒤
physical gate가 정확한 source hash를 검증한 binary다. 예전 control-only 명령의
`CARLA_WAIT_FOR_CONTROL_COMMAND=True`를 rendered 시험에 재사용하면 시작 전 sensor
preflight와 tick이 서로 기다리므로 bridge 단계에서 거부한다.
runner는 이 exact-patched `bridge` executable을 직접 실행하고 logger를 WARN으로
제한한다. 공식 STEP_ONCE 계약은 매 frame마다 `PAUSED`/`Execute single step`을 INFO로
기록하므로, 20 Hz 장시간 시험에서 콘솔과 ROS log가 제어 주기를 방해하는 것을 막기
위한 실행 옵션이며 제어 동작을 바꾸지 않는다.

Terminal 3 — CAMROD wall-time step pacer:

```bash
./scripts/virtual_carla/run.sh pacer
```

pacer는 bridge의 public `CarlaControl` 계약만 사용한다. 먼저 반복 `PAUSE`로 실제
PAUSED ACK를 확보한 뒤 `STEP_ONCE`를 한 번 보내고, `/carla/status`에서 정확한
`frame N → N+1 → PAUSED`를 확인한다. 다음 step의 기준은 직전 `STEP_ONCE` 명령
시각에서 50 ms 뒤다. 즉 해당 frame 처리에 20 ms가 걸렸으면 30 ms만 더 기다리고,
50 ms보다 오래 걸렸으면 다음 단일 step은 즉시 가능하다. 과거 deadline을 누적하지
않고 outstanding step도 항상 하나뿐이므로 느린 frame 뒤에 catch-up tick을 몰아서
보내지는 않는다. 외부 `PLAY`, frame jump,
`fixed_delta_seconds` 변경, status/step timeout은 모두 PAUSE로 fail closed한다.
일반 PAUSED status freshness는 `0.5 s`이지만, gate 검증과 PhysX 활성화가 포함된
Ranger 최초 spawn은 약 2초가 걸린다. 이어지는 full-sensor 최초 frame에서는 upstream
bridge가 새로 등록된 13개 CARLA sensor를 각각 최대 1초씩 순차 확인하므로 실제로
약 13초가 걸렸다. 따라서 outstanding step ACK에만 별도 `20.0 s` 상한을 적용한다.
이 구간에도 두 번째 STEP은 발행하지 않고 health는 ready가 아니다.
`spawn`은 `/virtual_carla/step_pacer/health`가 20 Hz·fresh status·PAUSED ACK를
능동 확인하기 전에는 실행되지 않는다.

Terminal 4 — Ranger actor와 sensor:

```bash
./scripts/virtual_carla/run.sh spawn
```

`offscreen`/`onscreen`의 기본 `RANGER_SPAWN_FILE`은 checked-in
`ranger_spawn_camrod_full_sensors.json`이다. 검증된 Woraksan ego pose는 control-only와
동일하고, CAMROD 장착 좌표의 camera 2개, LiDAR 1개, radar 7개, 좌/우 GNSS 2개와 IMU
1개를 각각 CARLA actor로 생성한다. actor JSON의 주기는 `0.1 s`다. `nullrhi`는 렌더링
센서를 검증할 수 없으므로 control-only JSON을 선택하고 CAMROD sensor relay도 시작하지
않는다. exact ego pose는 ROS/XODR 좌표
`(-20.6725482941, 33.9517669678, 3.0634040833, yaw=6.8785247803 deg)`이며,
Road 12/lane 2/s=2.0에서 노면보다 4 m 위에 생성되어 lanelet 751의 drop-zone 출구 앞에
안착한다.
다른 JSON으로 override하려면 accepted control sensor 전체(collision, lane invasion,
GNSS, IMU, odometry, control), 위 13개 UI sensor의 type/rate/장착값, exact ego pose의
spawn/alignment cohort를 모두 만족해야 한다. generic Ranger sensor-suite의 `(0,0)`
pose로 단순 교체하고 기존 alignment로 navigation 성공을 주장해서는 안 된다.

Terminal 5 — CAMROD algorithms와 production UI:

```bash
./scripts/virtual_carla/run.sh camrod
```

종료는 단순 역순이 아니다. `camrod → spawn → bridge → pacer → server` 순서로
종료한다. bridge 종료 전에 pacer의 release service를 호출하면 PAUSED bridge가
`PLAY`를 받아 가장 명시적으로 종료된다.

```bash
ros2 service call /virtual_carla/step_pacer/release std_srvs/srv/Trigger '{}'
```

pacer가 active인 상태에서 release를 생략하더라도 patched bridge의 PAUSE wait는
0.1초 timed get과 shutdown flag를 사용하고 world tick 직전에 종료를 재확인한다.
따라서 bridge는 Ctrl-C 한 번으로 끝나야 한다. 2026-08-31 live 20 Hz 경합 회귀
시험은 한 번의 Ctrl-C 뒤 0.24초, exit code 0을 기록했으며, 두 번째 Ctrl-C나 강제
종료는 사용하지 않았다. pacer를 bridge보다 먼저 Ctrl-C로 종료하면 정상 종료 경로가
PLAY를 발행하므로, bridge를 중단할 때까지 simulation이 다시 무제한 속도로 진행할 수
있다.

egg cache를 지정하지 않으면 Terminal 5의 `camrod`가 실행 전용의 새 빈 절대 경로를 만든다. 이미
내용이 있는 cache는 거부한다. 이 단계는 controller와 전체 알고리즘/UI를 시작하지만
motion을 자동 전송하지 않는다. rendered mode에서는 시작 전에 CARLA front/rear image,
두 CameraInfo와 LiDAR가 각각 1초 이상 관찰 구간에서 최소 `2 Hz` payload를 유지하고
최신 sample이 `3 s`보다 오래되지 않았는지도 bounded preflight로 확인한다. topic 이름만
존재하거나 one-shot이면 UI를 띄우지 않고 각 stream의 측정 rate와 age를 출력한다.
필요하면 두 기준을 `CAMROD_CARLA_SENSOR_MIN_RATE_HZ`와
`CAMROD_CARLA_SENSOR_MAX_SAMPLE_AGE_SECONDS`로 더 엄격하게 올릴 수 있다.

CAMROD UI 수동주행(권장):

1. `http://127.0.0.1:8010`을 열고 관리자 진단 화면의 **카메라** 탭으로 이동한다.
   현재 기본 UI에서는 화면 오른쪽 끝을 1.5초 누른 뒤 관리자 인증을 거친다.
2. 모든 텔레메트리 화면 아래에 유지되는 **CARLA 수동주행** 도크에서 **제어 열기**를
   누른 뒤 **수동주행 시작**을 한 번 누른다. 도크는 기본적으로 한 줄 상태 바로 접혀
   있어 카메라·도킹·안전 화면을 가리지 않는다. ARM 이후 다시 접어도 키보드 제어와
   WebSocket 세션은 유지되며, 접힌 바에서 현재 모드와 X/Y/Yaw 명령을 확인할 수 있다.
   backend가 진행 중 Nav2 goal을 취소하고 mission engage를 닫은 뒤 manual engage와
   platform drive-enable을 연다. 별도로 B1~B13이나 메인 ENGAGE를 누르지 않는다.
   서버 응답 전에는 패널이 **ARMING**을 표시한다. 이때 새로 누른 방향 키는 보관하지만
   `armed=true` 응답 전에는 drive frame을 보내지 않는다. ARM 요청 전부터 누르고 있던
   키의 OS repeat는 무시하므로 예상하지 않은 승인 직후 출발도 막는다.
3. **속도** slider로 10–100%를 선택한다. 기본 CARLA envelope에서 100%는 전후진
   `1.40 m/s`, crab `1.00 m/s`, yaw `0.7853 rad/s`이고 현재 선택값과 실제 전송값이
   패널에 함께 표시된다. slider는 server가 정한 envelope를 줄일 수만 있다.
4. 모드 버튼을 따로 누르지 않는다. UI가 held key를 제어기 입력 모드로 자동 분류한다.
   `W/S + A/D`는 Dual-Ackermann, `A/D` 단독은 zero-turn, `Z/C` 단독은 Crab이다.
   분류된 모드가 바뀌면 기존 모드의 exact zero를 먼저 보내고 다음 100 ms heartbeat부터
   새 모드 non-zero를 보낸다. Crab 키와 전후진/회전 키를 섞으면 우선순위로 추측하지
   않고 exact zero로 차단한다. 버튼/key-up 즉시 zero가 전송된다. zero 상태에서도 10 Hz
   heartbeat가 UI 권한 lease를
   갱신하며, ARM 직후 첫 frame 전부터 deadline이 시작된다. 일반 CAMROD UI lease는
   `0.25 s`, 고부하 CARLA composition은 `0.75 s`이다. 해당 시간이 지난 늦은 frame은
   timer callback이 지연돼도 lease를 되살리지 못하고 재-ARM을 요구한다. 어느 경우에도
   control/adapter의 독립 `0.35 s` stale-command watchdog이 실제 non-zero 출력을 먼저
   정지시킨다.

| 자동 분류 모드 | UI 키/버튼 | 동작 |
|---|---|---|
| **직진·조향** | `W` / `S`, `↑` / `↓` | 전진 / 후진 |
| **직진·조향** | 전후진 중 `A` / `D`, `←` / `→` | 기본 100% envelope에서 약 1.78 m 반경의 Dual-Ackermann 좌/우 조향 |
| **제자리회전** | `A` / `D`, `←` / `→` 단독 | 제자리 반시계 / 시계 회전 |
| **Crab** | `Z` / `C` 단독 | 좌 / 우 횡이동; 전후진·yaw 키와 혼합하면 zero |
| 모든 모드 | `Space` | 즉시 zero, 수동 권한은 유지 |
| 모든 모드 | `Esc` 또는 **수동주행 종료** | zero 후 수동 권한 해제 |

도킹 탭은 최신 AprilTag debug 영상이 있으면 이를 우선 표시한다. CARLA 맵에 태그가
배치되지 않아 detector 영상이 없거나 stale이면 `/sensing/camera/econ_rear/`의 실제 CARLA
후방 카메라로 자동 전환하며, 화면 제목과 source pill에 현재 영상 출처를 표시한다.

브라우저 focus 상실은 held key를 비우고 exact zero를 보내되 ARM은 유지하므로 로그나
CARLA 창을 잠깐 확인한 뒤 새 key-down으로 바로 재개할 수 있다. 브라우저 page hide,
숨김, WebSocket 끊김, 운영자 workspace 종료에는 zero 후 권한을 해제하며 재연결 뒤
자동 재출발하지 않는다. 수동 패널과 전용 WebSocket은 Camera/Docking/안전 탭 전환에도
계속 mount되어 영상 탭 변경만으로 ARM이 풀리지 않는다. 이 패널은 CARLA composition이
`/control/manual_cmd_vel_ros`를 명시한 경우에만 나타난다. 일반 `develop`/실차 CAMROD의
UI와 기본 topic 계약은 바뀌지 않는다. 수동 입력도 E-stop, lanelet 차체 경계,
동적 장애물과 충돌 cost, stale-command 검사를 그대로 통과해야 한다. 따라서 화면에
`lanelet_physical_body_cost`가 표시되면 조작기가 고장 난 것이 아니라 안전 게이트가
의도적으로 zero를 내는 상태다. slider가 높아도 마지막 safety gate 또는 4WS mapping
limit가 실제 출력을 더 낮추거나 zero로 만들 수 있다.

선택 Terminal 6 — UI를 쓸 수 없을 때의 터미널 키보드 fallback:

```bash
./scripts/virtual_carla/run.sh manual
```

`manual`은 다음 조건을 모두 다시 확인한 뒤에만 `teleop_twist_keyboard`를 CARLA 전용
입력 `/control/manual_cmd_vel_ros`로 remap한다. 첫 수동 Twist가 도착하기 전에는 기존
UI/Nav2 goal 경로가 유지되며, 첫 Twist 이후에는 safety gate가 수동 입력 소유권을
고정해 Nav2/일반 raw 명령이 수동 명령을 덮어쓰지 못하게 한다.

- portable baseline/physical gate deep validation
- spawn JSON의 정확한 `vehicle.ranger.default`/role 계약
- CARLA world에 정확히 하나인 현재 Ranger actor
- `GET /ui/health`의 `ok=true`와 `GET /ui/state`의 `ready=true`,
  `engaged=false`, `mission_phase=READY`, `mission_source=none`
- `/carla/<role>/physical_four_wheel_status`의 `ready`, gate, PhysX substep,
  independent wheel drive, backend와 actor ID
- 현재 CARLA Ranger actor ID와 physical 4WS bridge가 고정한 actor ID가 정확히 동일함

터미널 fallback 자체는 engage 또는 goal을 발행하지 않고, 키를 누르기 전에는 주행
명령도 발행하지 않는다. UI에서 먼저 기존 goal을 **STOP**으로 취소한 뒤, 마지막
페이지의 독립 **ENGAGE**를 눌러 수동 조작을 승인한다. B1~B13 목적지 버튼은 ENGAGE가
아니다. 위의 카메라 탭 수동 패널은 이 순서를 한 번의 arm 동작으로 처리한다.
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
- rendered full-sensor 시험에서 front/rear RGB, raw/filtered/obstacle LiDAR, 좌/우 GNSS,
  IMU와 7 radar relay가 갱신됨
- CAMROD UI가 HTTP 200이고 현재 모듈 상태를 표시함

운영자가 UI 또는 Nav2 goal로 주행 시험을 시작하기 전 physical bridge의 armed 상태와
정지 watchdog을 먼저 확인한다. 이 저장소의 실행 스크립트는 안전상 직진, 회전,
crab, zero-turn 명령을 자동으로 보내지 않는다. `manual`도 키 입력을 기다리며 engage를
자동으로 켜지 않는다.

UI에서 camera, GNSS/IMU, proximity/radar 화면을 연 뒤 별도 terminal에서 source ownership을
검증한다.

```bash
./scripts/virtual_carla/run.sh audit-sensors
```

성공 첫 줄은 다음 형식이다.

```text
CARLA_SENSOR_SOURCE_AUDIT PASS streams=32 failed=0 actors=13 actor_failed=0 ...
```

이 audit는 13개 CARLA source stream과 UI가 직접 소비하는 canonical stream 19개의
publisher type, 단일 ownership, payload freshness/유효성을 확인하고, camera 2 + LiDAR 1 +
GNSS 2 + IMU 1 + radar 7 actor가 정확히 하나씩 `ego_vehicle`에 attach됐는지 CARLA API로
확인한다. camera 두 `CameraInfo`의 dimensions/calibration/rate는 `camrod` 시작 전 visual
preflight가 별도로 검증한다. fake/dummy publisher, 중복 publisher, 잘못된 type, stale/empty
payload 또는 actor 누락이 하나라도 있으면 FAIL이며, 이 경우 UI 화면만 보고 sensor 연동
완료로 판정하지 않는다.

## 11. 핵심 코드 위치

| 역할 | 파일/패키지 |
|---|---|
| Twist→4WS mode/limit/watchdog | `camrod_carla_adapter/src/camrod_carla_adapter/command_mapping.py`, `twist_to_4ws_node.py` |
| CARLA Nav2 최종 속도/반경 경계 | `camrod_carla_adapter/config/nav2_carla_reverse_return.yaml`, `camrod_control/include/camrod_control/ackermann_turn_radius_constraint.hpp`, `camrod_control/src/cmd_vel_safety_gate_node.cpp` |
| CARLA odometry/좌표 alignment와 실제 GNSS/IMU | `feedback_bridge_node.py`, `gnss_compat.py`, `config/woraksan_lane_anchor_alignment.yaml` |
| 실제 camera/LiDAR topic 변환 | `sensor_relay_node.py`, `sensor_relay.launch.py` |
| 실제 7채널 radar 변환 | `radar_mapping.py`, `radar_relay_node.py`, `config/radar_relay.yaml` |
| 전체 sensor spawn/preflight/source proof | `config/ranger_spawn_camrod_full_sensors.json`, `scripts/virtual_carla/check_carla_sensor_streams.py`, `sensor_source_audit_node.py` |
| simulated Ranger 상태 heartbeat/Drop Zone 접점 | `carla_platform_heartbeat_node.py`, `charging_contact_emulator_node.py` |
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
- **CameraInfo는 보이지만 image가 멈춤**: `sysctl net.core.rmem_max net.core.wmem_max`가
  둘 다 `20971520` 이상인지 확인한다. `/etc/sysctl.d/99-camrod-carla-dds.conf` 적용 뒤
  기존 ROS/CARLA process를 전부 종료하고 새 terminal에서 정방향으로 다시 시작한다.
- **목적지를 누르자 이동거리 0 m에서 `lanelet_physical_body_cost`로 정지**: 현재
  `/localization/pose` 중심만 보지 말고 0.05 m lanelet safety grid 안의 전체 물리 차체를
  확인한다. 구버전 single-anchor calibration은 drop zone과 약 86.6 m 떨어진 lanelet
  4584에 맞춰져 있으므로 사용하지 않는다. 최신 georeference alignment와 Road 12/lane
  2 spawn을 build한 뒤 전체 stack을 재시작하고, settle된 실제 차체 sample이 모두 cost
  0이며 route start가 lanelet 751인지 확인한다.
- **spawn만 재실행한 뒤 제어가 안 됨**: 새 actor ID와 기존 physical bridge binding이
  달라진 상태다. 실행 중인 CAMROD가 있으면 `spawn`을 다시 실행하지 않는다. 전체를
  역순으로 종료하고 정방향으로 재시작한다. `spawn`은 type과 무관하게 같은 role을 쓰는
  기존 actor가 하나라도 있으면 거부하고, `manual`은 두 actor ID가 다르면 fail closed한다.
- **UI readiness가 sensor rate 때문에 흔들림**: CARLA profile 기준은 `2 Hz`, stale
  한계는 `3 s`다. 최신 source를 build/restart하고 `audit-sensors`에서 정확히 어느 실제
  source/canonical stream이 느리거나 stale인지 확인한다. fake sensor rate를 올려서
  우회하지 않는다.
- **UI는 보이나 camera가 stale/NO FRAME**: rendered 기본 visual JSON인지 확인하고,
  `camrod` preflight가 열거한 `/carla/ego_vehicle/{rgb_view,rgb_rear,lidar_front}` payload를
  확인한다. NullRHI/control-only 결과로 camera PASS를 주장하지 않는다.
- **UI sensor 값은 보이지만 실제 CARLA source인지 불명확**: `run.sh audit-sensors`의
  `streams=32 failed=0 actors=13 actor_failed=0` PASS를 요구한다. fake/dummy 또는 중복
  publisher가 발견되면 해당 channel은 실패가 정상이다.
- **rendered server 시작 실패**: `nvidia-smi -L`과 `vulkaninfo --summary`를 먼저
  고친다. `nullrhi` 결과로 rendered 검증을 대체하지 않는다.
- **map에서 경로가 어긋남**: CARLA spawn pose, lanelet map과 SE(2) alignment를 한
  세트로 다시 검증한다.
- **Drop Zone에 주차한 actor를 유지한 채 CAMROD만 재시작한 뒤 UI가 바로 경로를
  보내고 safety gate가 막음**: reverse parking controller의 `PARKED`는 메모리 상태라
  process 재시작 시 `IDLE`로 돌아간다. CARLA 전용 charging-contact emulator는 이때만
  실측 위치가 Drop Zone 중심 `0.35 m` 이내이고, 실측 속도가 `0.05 m/s` 이하이며,
  pose/odometry가 각각 `0.5 s` 이내이고, reverse parking status `IDLE`과 planning
  `WAIT_DZ(2)`가 모두 `2.0 s` 이내인 상태를 `1.0 s` 연속 확인한 뒤 접점을 복원한다.
  `RUNNING` 등 mission-active planning state, stale/missing state, 이동, 위치 이탈 중에는
  항상 false다. 따라서 UI는 복원된 charging edge로 `CHARGING`을 재구성하고 기존
  charging dwell → Drop Zone station exit → site goal 순서를 그대로 사용한다. 이 복구는
  `camrod_carla_full.launch.py`에만 포함되며 production CAN/BMS와 일반 CAMROD 설정은
  변경하지 않는다.
- **B12 진행률이 경사에서 고정되어 복귀 버튼이 활성화되지 않음**: 복귀 API의
  문제가 아니라 아직 `WAITING_FOR_RETURN_REQUEST`에 도달하지 못한 상태일 수 있다.
  이전 full launch는 production `speed_scale=0.5`를 상속해 CARLA Nav2의 `0.20 m/s`를
  최종 `0.10 m/s`로 다시 줄였고, 약 5° 경사에서 휘 토크가 중력 보상 수준에 머물렀다.
  최신 full launch는 `CAMROD_CARLA_CMD_VEL_GATE_SPEED_SCALE=1.0`을 CARLA 조합에만
  주입해 경로 속도 `0.20 m/s`를 그대로 유지한다. 또한 최종 `abs(vx/wz)`가
  adapter 경계 `0.810330349 m` 바로 아래이면 command mode가 `ACKERMANN`과 `ZERO_TURN`
  사이에서 흔들릴 수 있으므로 full launch만 final Nav2 반경 `0.82 m`를 주입한다.
  production/default YAML은 각각 `speed_scale=0.5`, 반경 옵션 `0.0`을 그대로 유지하므로 실제 로봇,
  수동주행, 캠핑 maneuver, CRAB과 명시적 ZERO_TURN은 바뀌지 않는다. 수정 확인은
  gate parameter `speed_scale=1.0`, adapter status의 연속 `ACKERMANN`, 그리고 B12 도착
  상태 `UNLOAD_WAIT`/`WAITING_FOR_RETURN_REQUEST`를 함께 요구한다.
  B12 복귀 경사에서 이전 약 `5.00/4.90 N·m` 명령은 40초 이상 실제 진행
  `0.002-0.011 m/s`, 12초간 약 2 mm에 머물러 실패했다. Ranger controller는 일반
  `3.0 N·m` launch floor와 목표 `-0.20 m/s`를 바꾸지 않고, fresh·authorized·steering-
  converged ACKERMANN/CRAB translation이 `0.015 m/s` 이하로 실제 정지했을 때만
  `0.5 s` 후 total-scalar floor `5.5 N·m`, `1.0 s` 후 `6.0 N·m`를 적용한다. 이 값은
  grade feedforward에 더하지 않으며 translation cap `8.0 N·m`, bridge 절대 cap
  `20.0 N·m` 안에 있다. 진행이 `0.015→0.045 m/s`가 되면 연속적으로 해제하고,
  stop/sign/mode/권한/stale/nonfinite/time-regression에는 즉시 reset한다. 2026-08-31
  최종 왕복에서는 경사에서 실제 stall이 발생하지 않아 이 floor가 발동하지 않았으므로,
  그 실행은 전체 경로 성공 증거이지 `5.5/6.0 N·m` live 발동 증거로 과장하지 않는다.

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

### 13.1 2026-08-31 rendered B12 왕복 결과

정상 NVIDIA/Vulkan onscreen 실행에서 actor 50
`vehicle.ranger.default/ego_vehicle`를 Drop Zone 앞 lanelet 751에 소환하고, UI의 B12
dispatch와 도착 후 Return 요청만 한 번씩 사용했다. 수동주행 명령이나 pose teleport는
사용하지 않았다. 결과는 다음 순서로 완료됐다.

| KST 시각 | 상태 |
|---|---|
| 03:55:13 | B12 dispatch, 47.02 m outbound route 시작 |
| 04:01:01 | `CRAB_IN → UNLOAD_WAIT → WAITING_FOR_RETURN_REQUEST` 도착 |
| 04:03:01 | UI Return 수락, `CRAB_OUT` 후 47.32 m `reverse_shortest` route 시작 |
| 04:07:43 | Drop Zone route goal, parking yaw alignment 시작 |
| 04:08:10 | `WAITING_FOR_CHARGING`; station distance `0.251 m`, speed `0.000 m/s` |
| 04:08:21 | 51개 distinct frame/10.00 s charging evidence 후 `CHARGING(13)`/`PARKED` |

최종 CARLA ROS odometry `(-21.0796356, 30.5216637)`에 checked-in SE(2)
`offset=(6.9521841, 9.4901857)`, `yaw=8.142811e-7 rad`를 적용하면 CAMROD metric pose
`(-14.1274763, 40.0118322)`가 된다. 이는 `/camrod_carla/metric_pose`, GNSS pose,
localization input, EKF/UI 위치와 수치상 일치한다. 서로 다른 좌표를 직접 비교해
명령 적분이나 fake localization으로 오판하면 안 된다.

`run.sh audit-sensors`는 source/UI stream `32/32`, 실제 CARLA actor `13/13`을 통과했다.
front/rear camera, 전방 고정 120° solid-state LiDAR 근사, dual GNSS, IMU와 7 radar가
모두 CARLA actor에서 시작하며 fake sensor owner는 UI-visible 경계를 소유하지 않았다.
실행 중 lanelet footprint contact는 bounded automatic hold/release로 복구됐고, 정차 중
약 2초의 IMU/localization dropout에는 safety gate가 nonzero command를 차단한 뒤 자동
복구했다. 두 사건 모두 operator 주행 개입 없이 종료됐지만 새 환경에서 재현될 경우
무시하거나 threshold를 낮추지 말고 원인을 별도 조사한다.

실제 렌더 화면은 `07_b12_round_trip_e2e.png`와
`07_b12_round_trip_e2e.gif`, 정확한 gate/source/test 범위와 제한은 같은 폴더의 dated
E2E JSON에 기록한다. 원본 857초 MP4와 ROS log는 크기와 host-local 경로 때문에 Git에
넣지 않으며, 압축 PNG/GIF는 해당 원본에서 직접 추출했다.
