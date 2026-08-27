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

## 2. 전체 데이터 흐름

```text
UE 4.26 / CARLA 0.9.15 custom map
  ├─ vehicle.ranger.default Blueprint + four-wheel PhysX
  └─ CARLA Python physical-4WD v2 API
            │
            ├── carla_ros_bridge ── camera/LiDAR/IMU/GNSS/odometry
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
5. 네 wheel의 실제 steering/torque telemetry와 CARLA odometry가 다시 CAMROD로
   들어온다.

`camrod_carla_full.launch.py`는 CARLA server, ROS bridge 또는 actor를 소유하지
않는다. 프로세스 수명과 실패 범위를 분리하기 위해 네 단계를 각각 다른 터미널에서
시작한다.

## 3. 기준 환경

검증 기준 조합은 다음과 같다.

- Ubuntu 22.04, x86_64
- ROS 2 Humble
- Python 3.10
- CARLA 0.9.15 source build와 프로젝트에 포함된 custom Woraksan map
- Unreal Engine 4.26.x
- Humble용 `carla_ros_bridge`
- Node.js/npm: CAMROD production UI frontend build용
- rendered 시험: 정상 NVIDIA driver와 Vulkan

`nullrhi`는 GPU 없이 physics/control/Nav2 경로를 진단할 때만 사용한다. RGB,
camera UI, YOLO/perception 또는 시각 품질을 검증하지 못한다. `offscreen`도 실제
렌더링이므로 NVIDIA/Vulkan이 필요하다.

## 4. portable 디렉터리 계약

Ranger bootstrap이 기본적으로 만드는 런타임 트리는 다음과 같다.

```text
$RANGER_CARLA_ROOT/
  config/environment.env             # machine-local, untracked selections
  ros_ws/                             # Ranger 4WS ROS packages
  .work/
    src/carla/                        # custom CARLA source/project
    src/UnrealEngine_4.26/            # UE 4.26
    ros-bridge-ws/                    # standard CARLA ROS bridge
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
| `RANGER_SPAWN_FILE` | actor/sensor JSON; 기본은 control-only smoke 구성 |
| `CAMROD_LANELET_MAP` | CAMROD lanelet2 `.osm` |
| `CAMROD_MAP_ALIGNMENT_FILE` | CARLA↔CAMROD SE(2) alignment YAML |
| `CARLA_RENDER_MODE` | `offscreen`, `onscreen`, `nullrhi` |

portable gate 파일이 없으면 실행은 실패한다. `reports/`의 과거 timestamped JSON을
자동 fallback으로 쓰지 않는다. CARLA/UE binary, Blueprint, physical controller,
Python egg를 다시 빌드하거나 위치를 바꿨다면 Ranger 저장소에서 gate를 새로 만들고
검증해야 한다. egg 별칭 또는 해시가 다르면 fail closed가 정상이다.

## 5. 최초 환경 준비와 빌드

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

## 6. 오프라인 시험

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

## 7. 런타임 사전 점검

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

## 8. 실행 순서

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

Terminal 2 — standard ROS bridge:

```bash
./scripts/virtual_carla/run.sh bridge
```

Terminal 3 — Ranger actor와 sensor:

```bash
./scripts/virtual_carla/run.sh spawn
```

기본 `RANGER_SPAWN_FILE`은 checked-in control-only smoke 구성이다. 현재 저장소에는
Woraksan CAMROD pose에 정렬된 full-sensor JSON이 없다. RGB/LiDAR와 production
perception 시험용 파일은 ego pose와 SE(2) alignment를 함께 검증해 별도 생성해야 한다.
generic Ranger sensor-suite JSON으로 단순 교체하거나 spawn pose를 바꾸고 기존
alignment로 navigation 성공을 주장해서는 안 된다.

Terminal 4 — physical 4WS + full CAMROD + production UI:

```bash
./scripts/virtual_carla/run.sh camrod
```

egg cache를 지정하지 않으면 `camrod`가 실행 전용의 새 빈 절대 경로를 만든다. 이미
내용이 있는 cache는 거부한다. 이 단계는 controller와 전체 알고리즘/UI를 시작하지만
motion을 자동 전송하지 않는다.

선택 Terminal 5 — CAMROD safety gate를 통과하는 수동 키보드 조작:

```bash
./scripts/virtual_carla/run.sh manual
```

`manual`은 다음 조건을 모두 다시 확인한 뒤에만
`teleop_twist_keyboard`를 `/control/nav2_cmd_vel_ros`로 remap한다.

- portable baseline/physical gate deep validation
- spawn JSON의 정확한 `vehicle.ranger.default`/role 계약
- CARLA world에 정확히 하나인 현재 Ranger actor
- `GET /ui/health`의 `ok=true`와 `GET /ui/state`의 `ready=true`,
  `engaged=false`, `mission_phase=READY`, `mission_source=none`
- `/carla/<role>/physical_four_wheel_status`의 `ready`, gate, PhysX substep,
  independent wheel drive, backend와 actor ID

명령 자체는 engage 또는 goal을 발행하지 않고, 키를 누르기 전에는 주행 명령도
발행하지 않는다. UI에서 먼저 기존 goal을 **STOP**으로 취소한 뒤 **ENGAGE**를 눌러
수동 조작을 승인한다. 기본 속도는 `0.20 m/s`, 회전 속도는 `0.20 rad/s`다.
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

선택 Terminal 6 — wheel telemetry monitor:

```bash
source ./scripts/virtual_carla/env.sh
virtual_carla_source_ros true true
ros2 run rqt_extended_ackermann rqt_extended_ackermann
```

종료는 Terminal 6 → 5 → 4 → 3 → 2 → 1의 역순이다.

## 9. UI와 기능 확인

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
- `/carla/ego_vehicle/extended_ackermann_cmd`와 wheel telemetry mode가 일치함
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

## 10. 핵심 코드 위치

| 역할 | 파일/패키지 |
|---|---|
| Twist→4WS mode/limit/watchdog | `camrod_carla_adapter/src/camrod_carla_adapter/command_mapping.py`, `twist_to_4ws_node.py` |
| CARLA odometry/좌표 alignment | `feedback_bridge_node.py`, `config/woraksan_lane_anchor_alignment.yaml` |
| camera/LiDAR topic 변환 | `sensor_relay_node.py`, `sensor_relay.launch.py` |
| simulated Ranger 상태 heartbeat | `carla_platform_heartbeat_node.py` |
| 전체 CAMROD/UI 조합 | `launch/camrod_carla_full.launch.py` |
| 부분 map/localization/planning 조합 | `launch/camrod_carla.launch.py` |
| wheel별 PhysX 제어 | Ranger ROS의 `carla_extended_ackermann_control` 및 custom CARLA API |
| CAMROD 알고리즘 | `camrod_sensing`, `camrod_localization`, `camrod_map`, `camrod_planning`, `camrod_control` |
| production UI | `camrod_ui`; frontend는 `colcon_build.sh`가 npm build |

## 11. 자주 발생하는 실패

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
- **UI readiness가 5 Hz/10 Hz 사이에서 흔들림**: 최신 source를 build한 뒤 CAMROD를
  재시작한다. heartbeat의 5 Hz가 fake sensors에 유출되던 launch-scope 충돌은 전용
  `fake_sensor_publish_rate_hz` 인자로 차단돼 있다.
- **UI는 보이나 camera가 stale**: control-only JSON 또는 NullRHI인지 확인한다.
- **rendered server 시작 실패**: `nvidia-smi -L`과 `vulkaninfo --summary`를 먼저
  고친다. `nullrhi` 결과로 rendered 검증을 대체하지 않는다.
- **map에서 경로가 어긋남**: CARLA spawn pose, lanelet map과 SE(2) alignment를 한
  세트로 다시 검증한다.

## 12. 증거 범위

[virtual CARLA 증거 인덱스](evidence/virtual_carla/README.md)에 PNG/GIF, UI
screenshot과 live report가 정리되어 있다.

- 시나리오 PNG/GIF는 제어·자세 데이터를 이용해 생성한 **기술 시각화**다. 실제
  CARLA camera 영상으로 간주하지 않는다.
- `camrod_carla_ui_latest_develop_20260825.png`는 당시 실행한 production UI의 실제
  screenshot이다.
- live/full-test JSON은 당시 프로세스와 topic을 관찰한 실제 보고서다.
- 이 과거 증거는 새 checkout의 gate 또는 새 rendered full-sensor 시험을 자동으로
  승인하지 않는다. 현재 실행 결과는 새 gate와 새 runtime report로 별도 기록한다.
