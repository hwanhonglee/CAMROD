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
| `opennav_docking` | `deps.repos` (소스 빌드) | Nav2 기반 도킹 서버 프레임워크 |
| `isaac_ros_apriltag` | apt (`setup_deps.sh` 자동) | CUDA AprilTag 검출 |
| `isaac_ros_image_proc` | apt (`setup_deps.sh` 자동) | RectifyNode (카메라 왜곡 보정) |
| `isaac_ros_nitros` | apt (`setup_deps.sh` 자동) | Zero-copy NITROS 파이프라인 |
| `image_pipeline` / `cv_bridge` | `deps.repos` (소스 빌드) | ROS 2 이미지 처리 |
| `opencv4_vendor` / `yaml_cpp_vendor` | 로컬 벤더 (소스 빌드) | 커스텀 CMake 타겟 |
| `nav2_msgs`, `vision_msgs` 등 | `setup_deps.sh` rosdep 자동 | 표준 ROS 2 apt 패키지 |

> **Isaac ROS apt 저장소** — `setup_deps.sh` 최초 실행 시 NVIDIA apt 저장소를 자동으로 등록한다.

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
./setup_deps.sh
```

| 옵션 | 동작 |
|------|------|
| `--full` | git history 포함 full 클론 (기본: shallow) |
| `--update` | 이미 클론된 패키지 업데이트 |
| `--skip-apt` | Isaac ROS apt 설치 건너뜀 |
| `--skip-rosdep` | rosdep 설치 건너뜀 |

`setup_deps.sh` 실행 순서:
1. `deps.repos` 기반 소스 의존성 클론 (vcs)
2. Isaac ROS apt 저장소 등록 + 패키지 설치
3. `rosdep install` — 표준 ROS 의존성 자동 설치

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

### CAMROD 풀스택 연동 (ESKF TF 사용)

ESKF가 `odom → base_link` TF를 담당하므로 `odom_yaw_corrector`의 TF 발행을 비활성화한다.

```bash
ros2 launch camrod_docking docking.launch.py enable_odom_corrector:=false
```

### 런치 인수 전체 목록

| 인수 | 기본값 | 설명 |
|------|--------|------|
| `docking_ns` | `docking` | 도킹 노드 최상위 네임스페이스 |
| `enable_auto_docking` | `false` | 배터리 기반 자동 도킹 활성화 |
| `enable_manual_docking` | `true` | UI 명령 매뉴얼 도킹 활성화 |
| `enable_odom_corrector` | `true` | `odom_yaw_corrector` TF 발행 활성화. CAMROD 풀스택(ESKF) 연동 시 `false` |

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

## 라이선스

이 저장소는 AVG(Avgenius) 내부 프로젝트입니다.  
외부 벤더 패키지(`opennav_docking`, `isaac_ros_*`)는 각 패키지의 라이선스를 따릅니다.
