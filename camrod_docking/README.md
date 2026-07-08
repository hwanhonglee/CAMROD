# CAMROD DOCKING

ROS 2 Humble 기반 CAMROD 로봇 자율 충전 도킹 시스템.  
Isaac ROS GPU 파이프라인(NITROS zero-copy)으로 AprilTag를 검출하고, EgoPolar 컨트롤러로 충전 스테이션 도킹을 수행한다.

> **현재 타겟 플랫폼:** Agile-X Ranger  
> 카메라 입력은 `camrod_sensing` 패키지가 담당하고, TF는 `camrod_sensor_kit` URDF가 관리한다.  
> CAMROD 풀스택 통합 시 `enable_odom_corrector:=false` 옵션을 사용한다.

---

## 도킹 시퀀스 (2-Phase)

도킹은 두 단계로 구성된다.

| Phase | 명칭 | 현황 |
|-------|------|------|
| Phase 1 | Nav2 자율 이동 → staging 위치 | **미구현** (설정 비활성 상태) |
| Phase 2 | AprilTag 시각 도킹 → 충전 스테이션 정밀 접촉 | **구현 완료** |

### Phase 1 — Navigate to Staging Pose

`opennav_docking`의 `navigate_to_staging_pose` 기능을 이용해  
Nav2 `/navigate_to_pose` action으로 로봇을 도킹 스테이션 앞 staging 위치까지 자율 이동한다.

- `docks.yaml`의 dock 좌표 기준 `staging_x_offset` 거리만큼 전방 위치 계산
- Nav2 `navigate_to_pose` → `SmacHybrid / NavFn` 플래너로 경로 생성
- staging 도달 판정: `dock_prestaging_tolerance` 이내 도달 시 Phase 2로 전환
- 실패 시 1회 재시도 후 `FailedToStage` 예외 발생 → ManualDock abort

### Phase 2 — AprilTag Visual Docking

후방 카메라로 AprilTag를 검출하고 EgoPolar 컨트롤러로 정밀 접근한다.

```
camrod_sensing (econ_rear 카메라)
  └─ /sensing/camera/econ_rear/image_raw
       └─ RectifyNode (isaac_ros_image_proc, GPU/NITROS)
            └─ AprilTagNode (isaac_ros_apriltag, CUDA)
                 └─ /docking/apriltag/detections_raw
                      └─ docking_apriltag_bridge
                           └─ /docking/detected_dock_pose  (camera optical frame)
                                └─ SimpleChargingDock.getRefinedPose()  (TF 변환 + EMA 필터)
                                     └─ opennav_docking EgoPolar Controller
                                          └─ /platform/cmd_vel  →  Agile-X Ranger
```

### Feedback 상태 흐름

```
ManualDock Action Feedback (phase_label)
  NAVIGATING_TO_STAGING  ← Phase 1: Nav2 이동 중
  INITIAL_PERCEPTION     ← Phase 2: AprilTag 첫 검출 대기
  DOCKING                ← Phase 2: EgoPolar 제어 중
  WAIT_FOR_CHARGE        ← 도킹 완료, 충전 확인 대기
  RETRY                  ← 실패 후 재시도
```

---

## 시스템 구성

### TF 트리

```
map ──(ESKF 위치추정)──  odom ──(odom_yaw_corrector 또는 ESKF)──  base_link
                                                                    ├─ camera_rear_link  (camrod_sensor_kit URDF)
                                                                    │   └─ camera_rear   (optical frame)
                                                                    └─ camera_front_link
                                                                        └─ camera_front
```

- **도킹 단독 실행**: `odom_yaw_corrector`가 `odom → base_link` TF 발행 (`enable_odom_corrector: true`)
- **풀스택 연동**: ESKF가 `map → odom → base_link` TF 전체 담당 (`enable_odom_corrector: false`)

### Phase 1+2 통합 아키텍처 (목표 상태)

```
[Phase 1]
camrod_planning (Nav2)
  └─ /navigate_to_pose action  →  SmacHybrid planner  →  /planning/cmd_vel_raw
                                                              └─ cmd_vel_gate  →  /planning/cmd_vel  →  platform

[Phase 2]
opennav_docking (EgoPolar)
  └─ /platform/cmd_vel  (cmd_vel_gate 우회, 저속 0.15 m/s)

[전환 조건]
staging 위치 도달 (dock_prestaging_tolerance: 0.3m)
  → Nav2 goal cancel
    → AprilTag INITIAL_PERCEPTION 시작
```

---

## 패키지 목록

| 패키지 | 설명 |
|--------|------|
| [`camrod_docking`](src/camrod_docking/) | 핵심 도킹 패키지. ChargingDock 플러그인, AprilTag 브릿지, 매뉴얼 도킹 서버, 런치 파일 포함 |
| [`avg_msgs`](src/avg_msgs/) | CAMROD 전용 ROS 2 인터페이스 패키지 (msg / srv / action 정의) |

---

## 의존 패키지 (외부 / 벤더)

| 패키지 | 설치 방법 | 비고 |
|--------|-----------|------|
| `opennav_docking` | apt (`setup_camrod.sh` 자동) | Nav2 기반 도킹 서버 프레임워크 |
| `isaac_ros_apriltag` | apt (`setup_camrod.sh` 자동) | CUDA AprilTag 검출 |
| `isaac_ros_image_proc` | apt (`setup_camrod.sh` 자동) | RectifyNode (카메라 왜곡 보정) |
| `isaac_ros_nitros` | apt (`setup_camrod.sh` 자동) | Zero-copy NITROS 파이프라인 |
| `image_pipeline` / `cv_bridge` | apt (`setup_camrod.sh` 자동) | ROS 2 이미지 처리 |
| `opencv4_vendor` / `yaml_cpp_vendor` | 로컬 벤더 (소스 빌드) | 커스텀 CMake 타겟 |
| `nav2_msgs`, `vision_msgs` 등 | `setup_camrod.sh` rosdep 자동 | 표준 ROS 2 apt 패키지 |

> **Isaac ROS apt 저장소** — `setup_camrod.sh` 최초 실행 시 NVIDIA apt 저장소를 자동으로 등록한다.

---

## 환경 요구사항

| 항목 | 버전 |
|------|------|
| ROS 2 | Humble |
| OS | Ubuntu 22.04 (Jetson / NVIDIA) |
| CUDA / Jetpack | JetPack 5.x (Isaac ROS NITROS 필요) |
| 로봇 플랫폼 | Agile-X Ranger |
| 카메라 | ECONSYSTEM 후방 카메라 (`camrod_sensing` 경유) |

---

## 빌드

### 1. 외부 의존성 설치 (최초 1회)

```bash
cd <workspace>/src
./setup_camrod.sh
```

| 옵션 | 동작 |
|------|------|
| `--update` | 이미 클론된 외부 패키지 최신화 (local 변경 덮어씀) |
| `--no-rosdep` | rosdep 설치 건너뜀 |

`setup_camrod.sh` 실행 순서:
1. 외부 저장소 클론 (ublox, robot_localization, Lanelet2, ugv_sdk, ranger_ros2 등)
2. Isaac ROS apt 저장소 등록 + `isaac_ros_*`, `opennav_docking` 패키지 설치
3. `rosdep install` — 표준 ROS 의존성 자동 설치

> `setup_deps.sh` (`camrod_docking/` 내부) 는 구버전이며 현재는 사용하지 않는다.

### 2. 전체 빌드 (최초 1회)

```bash
colcon build --symlink-install \
  --packages-up-to camrod_docking avg_msgs
source install/setup.bash
```

> **최초 빌드 시 약 6~10분 소요** — `opencv4_vendor`(OpenCV 소스 빌드), `yaml_cpp_vendor`, Isaac ROS 패키지 컴파일 포함.

### 3. 소스 수정 후 재빌드

```bash
colcon build --symlink-install \
  --packages-select avg_msgs camrod_docking
source install/setup.bash
```

---

## 실행

### 도킹 스택 단독 실행 (Phase 2만, 수동 staging)

로봇을 도킹 스테이션 앞 약 0.5~1.5m 위치에 수동 배치 후 실행한다.

```bash
# 기본 (매뉴얼 도킹만 활성, odom_yaw_corrector TF 발행)
ros2 launch camrod_docking docking.launch.py

# CAMROD 풀스택(ESKF) 연동 시
ros2 launch camrod_docking docking.launch.py enable_odom_corrector:=false
```

### 도킹 테스트 런치 (카메라 + IMU + 도킹, Phase 2)

카메라·IMU·도킹 모듈을 통합한 독립 테스트 환경. Nav2 미포함이므로 Phase 1 불가.

```bash
ros2 launch camrod_bringup docking_test.launch.py imu_model:=gq7
```

### CAMROD 풀스택 연동 (Phase 1+2, 목표 상태)

Nav2(`camrod_planning`)와 ESKF(`camrod_localization`)가 함께 실행되어야 Phase 1이 동작한다.
### CAMROD Full-Stack Integration (Localization TF)

HH_260617 - In the CAMROD full stack, the selected localization backend (EKF by default, ESKF optional) owns the `odom → base_link` TF, so `odom_yaw_corrector` TF publication must stay disabled.

```bash
# 풀스택 bringup (planning + localization + sensing + docking)
ros2 launch camrod_bringup bringup.launch.py

# 도킹 단독 — 풀스택에서 docking 모듈 단독 재시작 시
ros2 launch camrod_docking docking.launch.py enable_odom_corrector:=false
```

> Phase 1 활성화를 위해 `config/manual_dock_server.yaml`의 `navigate_to_staging_pose: true`로 변경 필요. ([Phase 1 TODO](#phase-1-연동-todo) 참조)

### 런치 인수 전체 목록

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `docking_ns` | `docking` | 도킹 노드 최상위 네임스페이스 |
| `enable_auto_docking` | `false` | 배터리 기반 자동 도킹 활성화 |
| `enable_manual_docking` | `true` | UI 명령 매뉴얼 도킹 활성화 |
| `enable_odom_corrector` | `true` | `odom_yaw_corrector` TF 발행 활성화. CAMROD 풀스택 localization TF 연동 시 `false` |

### 매뉴얼 도킹 트리거 (CLI)

```bash
# Phase 2만 (로봇이 이미 staging 위치에 있을 때)
ros2 action send_goal /docking/manual_dock \
  avg_msgs/action/ManualDock \
  "{dock_id: 'home_dock', max_staging_time: 0.0, dock_timeout_sec: 180.0}" --feedback

# Phase 1+2 (로봇이 임의 위치에 있을 때, navigate_to_staging_pose: true 필요)
ros2 action send_goal /docking/manual_dock \
  avg_msgs/action/ManualDock \
  "{dock_id: 'home_dock', max_staging_time: 120.0, dock_timeout_sec: 300.0}" --feedback
```

---

## 주요 설정 파일

| 파일 | 설명 |
|------|------|
| `config/apriltag.yaml` | RectifyNode 해상도 + AprilTag 검출기 설정 (size, tag_family, CUDA) |
| `config/docking_server.yaml` | opennav_docking 서버 파라미터 (`fixed_frame`, `dock_prestaging_tolerance` 등) |
| `config/controller.yaml` | EgoPolar 컨트롤러 게인 (`k_phi`, `k_delta`, `v_linear_max`) |
| `config/odom_yaw_corrector.yaml` | odom 보정기 설정 (Ranger: `use_firmware_yaw: true`) |
| `config/lifecycle_manager.yaml` | Nav2 라이프사이클 매니저 설정 |
| `config/docks.yaml` | 도킹 스테이션 위치 DB (Phase 1 staging 좌표 포함) |
| `config/bridge_params.yaml` | AprilTag 브릿지 파라미터 (`target_tag_id`, `filter_coef`) |
| `config/manual_dock_server.yaml` | 매뉴얼 도킹 서버 파라미터 (`navigate_to_staging_pose`) |
| `config/bt/dock_robot.xml` | DockRobot / UndockRobot BehaviorTree |

### 핵심 파라미터 참조

```yaml
# docking_server.yaml
controller_frequency: 40.0        # odom 40Hz와 일치 (50Hz → TF extrapolation 오류)
fixed_frame: "odom"               # Phase 1 통합 시 "map"으로 변경 권장
dock_prestaging_tolerance: 0.3    # Phase 1 완료 판정 거리 (m)
dock_backwards: true              # 후방 카메라 사용 → 후진 도킹

# SimpleChargingDock (apriltag_dock)
staging_x_offset: -1.5           # dock 기준 staging 위치 (뒤쪽 1.5m)
docking_threshold: 0.5           # 도킹 완료 판정 거리 (m)
filter_coef: 0.3                 # EMA 응답성 (이전 0.1 → 느린 수렴)
external_detection_rotation_pitch: 1.5708  # optical → nav 프레임 변환

# manual_dock_server.yaml
navigate_to_staging_pose: false  # Phase 1 활성화 스위치 (현재 비활성)
```

---

## 토픽 매핑

| 역할 | 토픽 |
|------|------|
| 후방 카메라 raw | `/sensing/camera/econ_rear/image_raw` |
| 후방 카메라 info | `/sensing/camera/econ_rear/camera_info` |
| 보정 이미지 (rectified) | `/sensing/camera/econ_rear/image_rect` |
| AprilTag 검출 raw | `/docking/apriltag/detections_raw` |
| AprilTag 검출 (진단용) | `/docking/apriltag/detections` |
| 도킹 Pose | `/docking/detected_dock_pose` |
| 매뉴얼 도킹 action | `/docking/manual_dock` (ManualDock) |
| opennav 도킹 action | `/docking/dock_robot` (DockRobot) |
| odom (입력) | `/odom` (Ranger CAN 출력) |
| cmd_vel Phase 1 | `/planning/cmd_vel_raw` → cmd_vel_gate → `/planning/cmd_vel` |
| cmd_vel Phase 2 | `/platform/cmd_vel` (docking_server 직접 발행) |

---

## Phase 1 연동 TODO

Phase 1 (Nav2 자율 이동 → staging) 활성화를 위해 필요한 작업 목록.

### A. 전제조건 — 좌표 및 TF 인프라

**A-1. 도킹 스테이션 staging 좌표 실측** _(즉시)_  
`config/docks.yaml`의 `pose: [1.0, 0.0, 0.0]` → 실측값으로 교체.  
staging 위치는 도크 기준 뒤쪽 ~1.5m, 후방 카메라로 태그 검출 가능한 지점.  
좌표 프레임은 ESKF 리셋 영향을 받지 않는 `map` 프레임 사용 권장.

```yaml
# config/docks.yaml
docks:
  home_dock:
    type: apriltag_dock
    frame: map         # odom → map으로 변경 권장
    pose: [x, y, yaw] # 실측값 입력
```

**A-2. fixed_frame → map 변경** _(높음)_  
`config/docking_server.yaml`의 `fixed_frame: "odom"` → `"map"`.  
이유: ESKF 리셋 시 odom 원점이 이동하므로 절대 좌표 기반 staging 위치가 틀어짐.  
전제: ESKF가 `map → odom` TF를 제공해야 함 (풀스택에서 이미 제공).

---

### B. 런치 아키텍처 통합

**B-1. Phase 1+2 통합 런치 작성** _(즉시)_  
현재 `docking_test.launch.py`는 Nav2를 포함하지 않아 Phase 1 실행 불가.  
아래 모듈을 포함하는 통합 런치 파일 작성 또는 `bringup.launch.py` 경로 확인.

```
camrod_sensing        (GNSS 포함)
camrod_localization   (ESKF: map→odom TF 제공)
camrod_planning       (Nav2: navigate_to_pose action 제공)
camrod_docking        (enable_odom_corrector:=false)
```

**B-2. enable_odom_corrector 강제 false** _(높음)_  
풀스택 실행 시 ESKF가 TF를 담당하므로 `odom_yaw_corrector` 중복 발행 방지.  
`docking_test.launch.py`의 `enable_odom_corrector='true'` → 통합 런치에서 `false`로 덮어씀.

---

### C. 설정 변경

**C-1. navigate_to_staging_pose 활성화** _(높음)_  
`config/manual_dock_server.yaml`

```yaml
navigate_to_staging_pose: true   # false → true
```

**C-2. navigator_bt_xml 경로 설정** _(중간)_  
`config/docking_server.yaml`에 파라미터 추가.  
빈 문자열이면 opennav_docking 기본 BT 사용 (네비게이션 실패 처리 없음).

```yaml
/docking/docking_server:
  ros__parameters:
    navigator_bt_xml: ""   # 추가: 도킹 전용 BT 또는 기본값 유지
```

**C-3. max_staging_time 기본값 설정** _(중간)_  
ManualDock goal의 `max_staging_time` 필드가 0이면 타임아웃 없음.  
CLI 호출 및 UI에서 실용적 값(예: 120.0초) 전달하도록 명문화.

---

### D. cmd_vel 안전 경로 정비

**D-1. Phase 1 중 cmd_vel_gate engage 보장** _(높음)_  
Phase 1(Nav2 이동) 구간에서 `/planning/engage` 발행이 없으면 cmd_vel_gate가 차단되어 로봇 미동작.  
도킹 시퀀스 진입 시 state_machine이 engage를 발행하는 경로 확인 또는 추가.

**D-2. Phase 2 cmd_vel_gate 바이패스 정책 결정** _(낮음)_  
Phase 2에서 docking_server가 `/platform/cmd_vel`에 직접 발행 → cost_stop / e-stop 안전 레이어 우회.  
현재: 저속(v_linear_max: 0.15 m/s)으로 허용 중. 문서화 또는 cmd_vel_gate 경유로 변경.

**D-3. Phase 전환 시 cmd_vel 경쟁 방지** _(중간)_  
Phase 1 완료 후 opennav_docking이 Nav2 goal을 cancel하고 Phase 2 EgoPolar 루프 진입하는 순서에서  
두 제어기가 동시에 cmd_vel을 발행하는 경쟁 조건 없는지 확인.

---

### E. 상태 머신 연동

**E-1. RETURN 씨나리오 완료 → 도킹 트리거 연결** _(중간)_  
`planning_state_machine_node.py`의 `require_docking_for_idle: false` → `true`로 변경 시  
`/docking/is_charging` (Bool) 토픽 구독으로 도킹 완료 후 IDLE 전환.

**E-2. /docking/is_charging 토픽 발행 구현** _(중간)_  
`use_battery_status: false`이므로 충전 감지 토픽이 없음.  
도킹 성공(ManualDock result.success=true) 후 Bool(True)를 발행하는 퍼블리셔 추가.

**E-3. 도킹 중 Nav2 goal 차단** _(중간)_  
Phase 1 실행 중 state_machine이 동시에 Nav2 goal 발행 시 충돌.  
도킹 시퀀스 진입 시 state_machine의 goal 발행을 중단하는 인터록 추가.

---

### F. 검증 항목

| 테스트 | 조건 | 기대 결과 |
|--------|------|-----------|
| Phase 1 단독 | 로봇 임의 위치, `navigate_to_staging_pose: true` | staging 위치까지 Nav2 자율 이동 |
| Phase 1→2 전환 | staging 도달 후 | AprilTag 검출 시작, cmd_vel 스파이크 없음 |
| Nav2 실패 리트라이 | staging 경로 장애물 | 1회 재시도 후 `FailedToStage`, ManualDock abort |
| Undock | Phase 1+2 완료 후 | staging 위치로 후진, state_machine RETURN 전환 |

---

## Auto Docking (UI 제어) TODO

UI 설정 탭의 **Auto Docking** 토글은 `camrod_ui`가 `/docking/auto_dock_enabled` (Bool) 토픽을 발행하도록 구현되어 있다.  
해당 토픽을 소비하는 ROS2 측 로직은 아직 미구현이며, 아래 작업이 필요하다.

### AA-1. planning_state_machine — 토픽 구독 추가 _(높음)_

`planning_state_machine_node.py`에서 `/docking/auto_dock_enabled` 구독을 추가하고  
플래그를 내부 상태(`self._auto_dock_enabled`)로 보관한다.

```python
# planning_state_machine_node.py 추가 예시
self._auto_dock_enabled = False
self.create_subscription(Bool, '/docking/auto_dock_enabled', self._on_auto_dock_enabled, 10)

def _on_auto_dock_enabled(self, msg: Bool) -> None:
    self._auto_dock_enabled = msg.data
```

### AA-2. 배터리 임계값 기반 자동 도킹 트리거 구현 _(높음)_

`/battery_percentage` (Int32) 값을 감시하다가 임계값 이하로 떨어지면  
`/docking/auto_dock_enabled` 플래그가 True일 때 자동으로 `/docking/manual_dock` 액션을 호출한다.

구현 위치: `planning_state_machine_node.py` 또는 별도 `auto_dock_monitor_node.py`

```python
# 파라미터 예시 (auto_dock_monitor.yaml)
auto_dock_battery_threshold: 20   # 배터리 20% 이하 시 자동 도킹 트리거
auto_dock_id: "home_dock"         # 대상 도크 ID
```

**설계 결정 필요:**  
- 배터리 모니터링을 state_machine에 통합할지, 독립 노드로 분리할지

### AA-3. 미션 중 자동 도킹 충돌 처리 정책 결정 _(높음)_

배송 미션(`RUNNING` 상태) 진행 중에 배터리 임계값에 도달하면 어떻게 처리할지 결정해야 한다.

| 옵션 | 동작 | 비고 |
|------|------|------|
| A. 미션 우선 | 배송 완료 후 도킹 시도 | 극저 배터리 시 미션 실패 위험 |
| B. 도킹 우선 | 미션 즉시 중단 → 도킹 | 미배송 상황 발생 |
| C. 임계값 2단계 | 경고 임계(30%) + 강제 임계(10%) 구분 | 권장 |

### AA-4. 도킹 완료 후 미션 복귀 로직 _(중간)_

자동 도킹 → 충전 완료 후 원래 미션(목표 사이트)으로 복귀할지 여부 결정.  
복귀 시 state_machine의 목표 사이트 정보를 도킹 진입 전에 보관해야 한다.

`/docking/is_charging` (Bool) 토픽 또는 ManualDock `result.success` 콜백으로 충전 완료를 감지한다.  
→ [알려진 설계 이슈 E-2](#medium-dockingis_charging-토픽-발행-미구현) 항목과 연계.

### AA-5. 검증 항목

| 테스트 | 조건 | 기대 결과 |
|--------|------|-----------|
| 토픽 연동 | UI 토글 ON → `ros2 topic echo /docking/auto_dock_enabled` | `data: true` 수신 |
| 배터리 트리거 | `auto_dock_enabled=true`, 배터리 모킹 → 임계값 이하 | ManualDock goal 자동 전송 |
| 미션 중 트리거 | RUNNING 상태에서 배터리 임계값 도달 | 결정된 정책대로 동작 |
| 토글 OFF | 자동 도킹 진행 중 UI에서 OFF | 다음 트리거 사이클부터 비활성 (진행 중 액션은 계속) |

---

## 알려진 설계 이슈

### [HIGH] Nav2 미션 중 매뉴얼 도킹 트리거 시 cmd_vel 경쟁

**현상:** Nav2 미션이 활성(`/planning/engage=True`) 상태에서 매뉴얼 도킹을 트리거하면,
Nav2 컨트롤러와 opennav_docking EgoPolar 컨트롤러가 동시에 `/platform/cmd_vel`에 발행하여
로봇 동작이 예측 불가능해진다.

**cmd_vel 경로 분석:**

```
경로 A: Nav2 (정상 주행)
  Nav2 → /planning/cmd_vel_raw
       → planning_cmd_vel_gate_node  ← engage + estop 체크
       → /planning/cmd_vel
       → (실 하드웨어: 직접 연결, platform gate 비활성)
       → /platform/cmd_vel → Ranger CAN

경로 B: opennav_docking
  EgoPolar Controller → /platform/cmd_vel (직접, 게이트 전혀 없음)
                      → Ranger CAN
```

**각 안전장치가 무력화되는 이유:**

| 안전장치 | 무력화 이유 |
|----------|------------|
| `/planning/engage` | planning gate(경로 A)만 제어. 경로 B는 이 게이트 자체를 우회 |
| `/platform/status/estop` | planning gate의 estop 입력으로 경로 A만 차단. 경로 B 미구독 |
| platform cmd_vel gate | `platform/cmd_vel_gate_enable: false`로 실 하드웨어에서 비활성 |
| `active_` 플래그 | 두 번째 도킹 goal reject 전용. Nav2와의 cmd_vel 경쟁과 무관 |

**재현 조건:**
```bash
# 1. bringup 풀스택 실행 (Nav2 미션 진행 중, engage=True)
# 2. UI에서 매뉴얼 도킹 버튼 클릭
# → Nav2(linear.x=+0.5)와 docking(linear.x=-0.2)이 교번 발행 → jitter
```

**개선 방향 (인터록 TODO):**

1. `manual_dock_server_node.cpp`의 goal execute 함수 초반에 `/planning/engage=False` 발행  
   → planning gate가 Nav2 cmd_vel을 즉시 차단 (가장 빠른 수정)

   ```cpp
   // manual_dock_server_node.cpp — execute() 시작 직후
   auto engage_msg = std_msgs::msg::Bool();
   engage_msg.data = false;
   engage_pub_->publish(engage_msg);
   // ... 도킹 완료/실패 후 원래 상태로 복귀
   ```

2. (중기) `planning_state_machine_node.py`에 `STATE_DOCKING` 추가:  
   - 진입 시: `nav2.cancel_goal()` + `engage=False`  
   - 완료 시: 이전 상태 복귀 + `engage=True`  
   → AA-3 TODO 미션 중 자동 도킹 충돌 처리 정책과 동일한 해법

> **현재 docking_test.launch.py 단독 테스트 환경에서는 Nav2 미션이 없으므로 발생하지 않음.**  
> Phase 1 통합(Nav2 + 도킹 풀스택) 전에 반드시 인터록 구현 후 진행할 것.

---

### [HIGH] manual_dock_server_node — lifecycle 미지원

**현상:** `opennav_docking` 서버가 기동되지 않았거나 크래시된 상태에서 ManualDock goal을 수신하면,
`dock_client_->wait_for_action_server(5s)` 타임아웃 후 abort로 종료된다.
이후 서버가 복구되어도 자동 재연결 로직이 없어 재시도하려면 goal을 다시 전송해야 한다.

**영향:** 운용 중 opennav_docking 프로세스가 재시작되는 상황(리소스 부족, SIGKILL)에서
수동 개입 없이 도킹 재시도 불가.

**재현 조건:**
```bash
# opennav_docking 없이 manual_dock_server_node만 기동 후 goal 전송
ros2 action send_goal /docking/manual_dock avg_msgs/action/ManualDock "{dock_id: 'home_dock'}"
# → 5초 후 "docking_server action not available" 메시지와 함께 abort
```

**개선 방향:**
- `manual_dock_server_node`에 `rclcpp_lifecycle::LifecycleNode` 도입
- `/docking/health` 토픽으로 `opennav_docking` 서버 가용 상태 발행
- UI "도킹" 버튼을 헬스 상태 기반으로 활성화/비활성화

---

### [HIGH] Phase 1 미구현 — navigate_to_staging_pose 비활성

**현상:** `manual_dock_server.yaml`의 `navigate_to_staging_pose: false`로 설정되어 있으며,
`docking_test.launch.py`에 Nav2가 포함되어 있지 않아 Phase 1 자율 이동이 동작하지 않는다.
현재는 로봇을 수동으로 staging 위치에 배치해야만 도킹을 시작할 수 있다.

**영향:** 완전 자율 도킹 시퀀스 미지원. 운영자가 매번 수동 개입 필요.

**개선 방향:** [Phase 1 연동 TODO](#phase-1-연동-todo) 항목 순서대로 진행.

---

### [MEDIUM] target_tag_id 단일 태그만 지원

**현상:** `docking_apriltag_bridge`의 `target_tag_id` 파라미터가 정수 1개로 고정되어 있어,
하나의 브릿지 인스턴스로 여러 도킹 스테이션(각기 다른 tag ID)을 구분할 수 없다.

**영향:** 다수 도킹 스테이션 환경에서 스테이션별로 별도 브릿지 인스턴스와 컨테이너를 기동해야 하며,
런타임에 목표 스테이션을 전환하는 것이 불가능하다.

**관련 파라미터:**
```yaml
# config/bridge_params.yaml
target_tag_id: 3   # tag36h11 family ID — 단일 값만 허용
```

**개선 방향:**
- `target_tag_id` 를 리스트(`int[]`)로 확장하거나
- `/docking/target_tag_id` 토픽 구독으로 런타임 전환 지원

---

### [MEDIUM] fixed_frame odom 사용 — ESKF 리셋 시 staging 좌표 틀어짐

**현상:** `docking_server.yaml`의 `fixed_frame: "odom"`과 `docks.yaml`의 `frame: odom` 조합에서
ESKF가 리셋되거나 초기화되면 odom 원점이 이동하여 staging 좌표가 실제 위치와 달라진다.

**영향:** Phase 1 활성화 시 재시작 후 첫 도킹에서 staging 위치 오류 가능성.

**개선 방향:** A-1, A-2 TODO 항목 참조 (`map` 프레임으로 전환).

---

### [MEDIUM] 프레임 ID 하드코딩

**현상:** 여러 소스 파일에서 `"odom"`, `"base_link"` 등 프레임 ID가 문자열 리터럴로 고정되어 있다.
로봇 플랫폼 변경 또는 멀티 로봇 네임스페이스 적용 시 소스 수정 없이는 대응 불가.

**해당 위치:**
- `odom_yaw_corrector.cpp` — child frame `"base_link"` 고정
- `docking_server.yaml` — `base_frame: "base_link"`, `fixed_frame: "odom"` (파라미터화는 되어 있으나 한 곳에서만 관리됨)

**개선 방향:**
- `camrod_sensor_kit`의 `robot_params.yaml`을 단일 소스로 지정하고, 런치 파일에서 주입

---

### [LOW] opennav_docking 헬스 모니터링 없음

**현상:** `opennav_docking` 서버의 가용 여부를 외부에서 확인할 수 있는 토픽이나 진단 항목이 없다.
`camrod_system` 진단 집계기에 도킹 서버 상태가 포함되지 않아, 운영자가 UI만으로는 도킹 준비 여부를 알 수 없다.

**개선 방향:**
- `camrod_system`에 `docking_server_checker` 진단 노드 추가
- UI 도킹 버튼에 서버 상태 인디케이터 표시

---

## 라이선스

이 저장소는 AVG(Avgenius) 내부 프로젝트입니다.  
외부 벤더 패키지(`opennav_docking`, `isaac_ros_*`)는 각 패키지의 라이선스를 따릅니다.

## 2026-06-17 Runtime Update

> HH_260617: Docking is no longer the only return-to-charger path.

`camrod_docking` remains the AprilTag/opennav docking path and is still Jetson/Isaac-ROS dependent. HH_260618: Full bringup selects this method only with `parking_method:=docking`; `parking_method:=rule_based` launches `camrod_parking` instead, and launch conditions prevent both final parking methods from commanding motion at the same time. On x86_64, `colcon_build.sh` skips `camrod_docking` because Isaac ROS/VPI dependencies are unavailable.

## 2026-07-02 Runtime Update

> HH_260702: The v1.16 field baseline uses rule-based parking unless docking is explicitly selected.

For outdoor campsite/drop-zone testing, `parking_method:=rule_based` is the supported path and `camrod_docking` should stay idle. Use `parking_method:=docking` only when the rear camera, Isaac ROS AprilTag pipeline, and opennav docking server are intentionally part of the test.

The Docker and selected-package build paths may skip this package on hosts without Isaac ROS/VPI. That skip is expected and does not affect lanelet driving, campsite crab motion, drop-zone reverse parking, or platform command gating.
