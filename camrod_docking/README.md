# camrod_docking

ROS 2 Humble 기반 CAMROD 로봇 자율 도킹 패키지.  
AprilTag 검출 → Pose 추정 → EgoPolar 컨트롤러로 충전 스테이션 도킹을 수행한다.  
전진/후진 도킹 모두 지원하며, Nav2 없이 수동 배치 후 도킹하는 매뉴얼 모드도 제공한다.

---

## 패키지 구조

```
camrod_docking/
├── config/
│   ├── apriltag.yaml           # AprilTag 검출기 설정
│   ├── docking_server.yaml     # opennav_docking 서버 파라미터
│   ├── docks.yaml              # 도킹 스테이션 위치 DB
│   └── manual_dock_server.yaml # 매뉴얼 도킹 서버 파라미터
├── include/camrod_docking/
│   ├── camrod_docking_plugin.hpp
│   └── manual_dock_server_node.hpp
├── launch/
│   ├── docking.launch.py       # Main launch (AprilTag + DockingServer + ManualDockServer; camera TF is in camrod_sensor_kit)
│   └── parking.launch.py       # docking.launch.py 래퍼
└── src/
    ├── camrod_docking_plugin.cpp       # ChargingDock 플러그인
    ├── parking_apriltag_bridge.cpp     # AprilTag → PoseStamped 변환 노드
    ├── manual_dock_server_node.cpp     # 매뉴얼 도킹 액션 서버
    ├── odom_yaw_corrector.cpp          # RMP401 오도메트리 부호 보정
    └── tag_distance_node.cpp           # 태그 거리 발행 노드
```

---

## 도킹 파이프라인

```
econ_camera (rear or front)
  └─ image_proc/rectify_node
      └─ apriltag_node  (dock_tag TF 발행)
          └─ docking_apriltag_bridge  →  /docking/detected_dock_pose  (odom frame)
              └─ CamrodDockingPlugin.getRefinedPose()
                  └─ opennav_docking controller (EgoPolar)
                      └─ /rmp401/cmd_vel
```

---

## 노드 설명

### `docking_apriltag_bridge` (parking_apriltag_bridge.cpp)

AprilTag 검출 결과를 받아 TF로부터 `dock_tag` 위치를 odom 프레임 `PoseStamped`로 변환·발행한다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `target_tag_id` | 0 | 추적할 AprilTag ID |
| `tag_frame` | `dock_tag` | apriltag_ros가 발행하는 TF 프레임명 |
| `fixed_frame` | `odom` | 출력 PoseStamped 기준 프레임 |
| `ema_alpha` | 0.4 | EMA 필터 계수 (1.0 = 패스스루) |
| `detection_timeout` | 0.5s | 이 시간 이상 검출 없으면 발행 중단 |
| `publish_rate_hz` | 10.0 | 출력 발행 주기 |

| 구독 토픽 | 타입 |
|-----------|------|
| `input_detection_topic` | `apriltag_msgs/AprilTagDetectionArray` |

| 발행 토픽 | 타입 |
|-----------|------|
| `output_detected_dock_pose_topic` | `geometry_msgs/PoseStamped` |
| `output_avg_pose_topic` | `avg_msgs/AvgAprilTagPose` |
| `output_avg_detection_topic` | `avg_msgs/AvgAprilTagDetectionArray` |

---

### `CamrodDockingPlugin` (camrod_docking_plugin.cpp)

`opennav_docking_core::ChargingDock` 플러그인. DockingServer에서 로드되어 도킹 제어에 사용된다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `dock_backwards` | false | 후진 도킹 여부 (DockingServer 공유 파라미터) |
| `staging_x_offset` | -0.70m | 스테이징 위치 오프셋 (dock 기준) |
| `docking_threshold` | 0.05m | isDocked() 판정 거리 허용 오차 |
| `external_detection_timeout` | 1.0s | dock pose 수신 타임아웃 |
| `external_detection_translation_x` | -0.20m | tag → base_link 목표 오프셋 (아래 TODO 참조) |
| `external_detection_translation_y` | 0.0m | 좌우 오프셋 |
| `filter_coef` | 0.1 | 플러그인 내 EMA 계수 |
| `detected_dock_pose_topic` | `/docking/detected_dock_pose` | 입력 PoseStamped 토픽 |
| `base_frame` | `base_link` | 로봇 base 프레임 |

**동작 방식:**
- `getStagingPose()`: dock 데이터베이스 좌표 기반 스테이징 위치 계산. `dock_backwards=true`이면 +π 회전.
- `getRefinedPose()`: 첫 호출 시 로봇의 odom 헤딩을 `approach_yaw`로 고정 (PnP 카메라 yaw 드리프트 방지). EMA 필터 적용 후 `external_detection_translation_x` 만큼 이동한 위치를 목표 pose로 반환.
- `isDocked()`: `|base_link ↔ tag 거리 - |ext_translation_x||` < `docking_threshold`로 판정.
- `isCharging()`: 현재 하드웨어 미연결 → `isDocked()` 결과 반환. 실제 충전기 연동 시 `/battery_state` current 기반으로 교체.

---

### `manual_dock_server_node` (manual_dock_server_node.cpp)

UI 또는 CLI에서 도킹을 트리거하는 액션 서버. 내부적으로 DockingServer의 `DockRobot` 액션을 호출한다.

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `navigate_to_staging_pose` | false | false = 로봇이 이미 dock 앞에 있다고 가정, Nav2 생략 |
| `action_server_name` | `/docking/manual_dock` | 이 노드가 제공하는 액션 이름 |
| `dock_action_name` | `/docking/dock_robot` | DockingServer 액션 이름 |

**매뉴얼 도킹 실행:**
```bash
ros2 action send_goal /docking/manual_dock \
  avg_msgs/action/ManualDock "{dock_id: 'home_dock'}"
```

---

### `odom_yaw_corrector` (odom_yaw_corrector.cpp)

RMP401 `segwayrmp` 드라이버의 오도메트리 부호 버그를 보정한다.  
`/rmp401/odom` 구독 → `odom → base_link` TF 발행 (qz, position.y 부호 반전).

---

## Camera TF Structure

```
sensor_kit_base_link
 ├─ camera_front_link  [x=+0.40m, z=0.46m, yaw=0  — forward-facing]
 │   └─ camera_front   [RPY(-π/2, 0, -π/2) — REP-103 optical frame]
 └─ camera_rear_link   [x=+0.10m, z=0.46m, yaw=π  — backward-facing]
     └─ camera_rear    [RPY(-π/2, 0, -π/2) — REP-103 optical frame]
```

> **HH_260528:** Static TF is now published by `camrod_sensor_kit` (`robot_state_publisher` via `camrod_sensor_kit.xacro`). `docking.launch.py` no longer runs `static_transform_publisher` nodes for camera frames.

---

## 실행

```bash
# 전체 도킹 스택 (카메라 TF + AprilTag + DockingServer + ManualDockServer)
ros2 launch camrod_docking docking.launch.py

# 자동 도킹 활성화 (기본: 매뉴얼만 활성)
ros2 launch camrod_docking docking.launch.py enable_auto_docking:=true

# 매뉴얼 도킹 비활성 (자동만)
ros2 launch camrod_docking docking.launch.py enable_manual_docking:=false
```

---

## TODO

- [ ] **`external_detection_translation_x` 파라미터 가독성 개선**

  현재 `-0.565` 같은 숫자만 보아서는 의미를 파악하기 어렵다.  
  아래 두 방향 중 하나로 개선이 필요하다.

  **방향 A — 의미 있는 파라미터로 분리:**
  ```yaml
  # docking_server.yaml 또는 별도 robot_geometry.yaml
  robot_geometry:
    front_camera_to_base_link_x: 0.40    # base_link → 전면 카메라 (m)
    front_bumper_to_camera_x:    0.165   # 전면 카메라 → 전면 범퍼 (m)
    rear_camera_to_base_link_x:  0.10    # base_link → 후면 카메라 (m)
    rear_bumper_to_camera_x:     0.0     # 후면 카메라 → 후면 범퍼 (m) — 실측 필요
  # external_detection_translation_x = -(camera_to_base_link + bumper_to_camera)
  ```
  플러그인 configure()에서 두 값을 읽어 자동 계산:
  ```cpp
  ext_translation_x_ = -(camera_to_base_x + bumper_to_camera_x);
  ```

  **방향 B — URDF 모델에서 로드:**  
  `rmp401_description` 패키지에 차량 전체 물리 치수를 URDF로 정의하고,  
  플러그인이 TF 트리에서 `base_link → bumper_link` 변환을 읽어 자동 계산.

- [ ] **후면 카메라 → 후면 범퍼 거리 실측**  
  후진 도킹 재활성화 시 `rear_bumper_to_camera_x` 값을 측정해 적용.

- [ ] **`docks.yaml` 실제 스테이션 좌표 측정·적용**  
  현재 `pose: [1.0, 0.0, 0.0]` 은 placeholder.  
  `navigate_to_staging_pose=false`(매뉴얼) 모드에서는 retry 시에만 영향이 있으나,  
  자율 도킹(`navigate_to_staging_pose=true`) 전환 전에 반드시 실측값으로 교체 필요.

- [ ] **`isCharging()` 실제 충전 하드웨어 연동**  
  현재 `isDocked()` 반환값을 그대로 사용. 충전 하드웨어 연결 후  
  `/battery_state` current > `charging_current_threshold` 조건으로 교체.
