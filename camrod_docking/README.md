# CAMROD DOCKING

ROS 2 Humble 기반 CAMROD 로봇 자율 충전 도킹 시스템.  
Isaac ROS GPU 파이프라인(NITROS zero-copy)으로 AprilTag를 검출하고, EgoPolar 컨트롤러로 충전 스테이션 도킹을 수행한다.  
전진/후진 도킹 모두 지원하며, 로봇을 스테이션 앞에 배치한 뒤 명령으로 도킹하는 매뉴얼 모드도 제공한다.

> **현재 타겟 플랫폼:** Agile-X Ranger  
> 카메라 입력은 `camrod_sensing` 패키지가 담당하고, TF는 `camrod_sensor_kit` URDF가 관리한다.  
> CAMROD 풀스택 통합 시 `enable_odom_corrector:=false` 옵션을 사용한다.

---

## 시스템 구성

```
camrod_sensing (econ_rear 카메라)
  └─ /sensing/camera/econ_rear/image_raw  →  RectifyNode (isaac_ros_image_proc, GPU)
                                               └─ AprilTagNode (isaac_ros_apriltag, CUDA/NITROS)
                                                   └─ docking_apriltag_bridge
                                                       └─ /docking/detected_dock_pose  (odom frame)
                                                           └─ SimpleChargingDock.getRefinedPose()
                                                               └─ opennav_docking (EgoPolar)
                                                                   └─ /platform/cmd_vel  →  Agile-X Ranger
```

**TF 트리**

```
odom  ──(odom_yaw_corrector 또는 ESKF)──  base_link
                                            ├─ camera_rear_link  (camrod_sensor_kit URDF)
                                            │   └─ camera_rear   (optical frame)
                                            └─ camera_front_link
                                                └─ camera_front
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

### 도킹 스택 단독 실행 (Ranger 직결)

```bash
# 기본 (매뉴얼 도킹만 활성, odom_yaw_corrector TF 발행)
ros2 launch camrod_docking docking.launch.py

# 자동 도킹 활성화
ros2 launch camrod_docking docking.launch.py enable_auto_docking:=true

# 매뉴얼 도킹 비활성 (자동만)
ros2 launch camrod_docking docking.launch.py enable_manual_docking:=false
```

### CAMROD Full-Stack Integration (Localization TF)

HH_260617 - In the CAMROD full stack, the selected localization backend (EKF by default, ESKF optional) owns the `odom → base_link` TF, so `odom_yaw_corrector` TF publication must stay disabled.

```bash
ros2 launch camrod_docking docking.launch.py enable_odom_corrector:=false
```

### 런치 인수 전체 목록

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `docking_ns` | `docking` | 도킹 노드 최상위 네임스페이스 |
| `enable_auto_docking` | `false` | 배터리 기반 자동 도킹 활성화 |
| `enable_manual_docking` | `true` | UI 명령 매뉴얼 도킹 활성화 |
| `enable_odom_corrector` | `true` | `odom_yaw_corrector` TF 발행 활성화. CAMROD 풀스택 localization TF 연동 시 `false` |

### 매뉴얼 도킹 트리거 (CLI)

```bash
ros2 action send_goal /docking/manual_dock \
  avg_msgs/action/ManualDock "{dock_id: 'home_dock'}"
```

---

## 주요 설정 파일

| 파일 | 설명 |
|------|------|
| `camrod_docking/config/apriltag.yaml` | RectifyNode 해상도 + AprilTag 검출기 설정 (size, tag_family, CUDA) |
| `camrod_docking/config/docking_server.yaml` | opennav_docking 서버 파라미터 (`/platform/cmd_vel` 등) |
| `camrod_docking/config/controller.yaml` | EgoPolar 컨트롤러 게인 |
| `camrod_docking/config/odom_yaw_corrector.yaml` | odom 보정기 설정 (Ranger: `use_firmware_yaw: true`) |
| `camrod_docking/config/lifecycle_manager.yaml` | Nav2 라이프사이클 매니저 설정 |
| `camrod_docking/config/docks.yaml` | 도킹 스테이션 위치 DB |
| `camrod_docking/config/bridge_params.yaml` | AprilTag 브릿지 파라미터 |
| `camrod_docking/config/manual_dock_server.yaml` | 매뉴얼 도킹 서버 파라미터 |

---

## 토픽 매핑

| 역할 | 토픽 |
|------|------|
| 후방 카메라 raw | `/sensing/camera/econ_rear/image_raw` |
| 후방 카메라 info | `/sensing/camera/econ_rear/camera_info` |
| 보정 이미지 (rectified) | `/sensing/camera/econ_rear/image_rect` |
| AprilTag 검출 raw | `/docking/apriltag/detections_raw` |
| 도킹 Pose | `/docking/detected_dock_pose` |
| odom (입력) | `/odom` (Ranger CAN 출력) |
| cmd_vel (출력) | `/platform/cmd_vel` |

---

## 알려진 설계 이슈

코드 리뷰를 통해 파악된 이슈 목록. 우선순위 순으로 정렬.

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
