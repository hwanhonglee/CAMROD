# Robot Diagnostics

로봇 전체 시스템의 건강 상태를 실시간으로 감시하는 ROS 2 진단 패키지 모음입니다.

24개 체커 노드가 각 도메인의 토픽·서비스를 구독하여 `/diagnostics` 를 발행하고,
Aggregator 가 이를 계층 트리로 집계하여 `/diagnostics_agg` 로 출력합니다.

```
Checker Nodes (×24)
      │  /diagnostics
      ▼
  Aggregator
      │  /diagnostics_agg
      ▼
rqt_runtime_monitor / 외부 모니터링
```

**주요 특징**

- 체커 24개 · 7개 도메인 (Platform → HW → Sensing → Localization → Map → Perception → Planning)
- `robot_diagnostics_base` 의 `BaseChecker` 를 상속하여 모든 체커가 동일한 구조를 가짐
- `robot_diagnostics_system_bringup` 에서 전체 config 를 중앙 관리 — 로봇별 override 지원
- 각 체커마다 더미 퍼블리셔 제공 — 실제 로봇 없이 전체 파이프라인 테스트 가능

---

## 목차

1. [아키텍처](#1-아키텍처)
2. [디렉토리 구조](#2-디렉토리-구조)
3. [패키지 목록](#3-패키지-목록)
4. [빠른 시작](#4-빠른-시작)
5. [System Bringup 상세](#5-system-bringup-상세)
6. [카테고리별 단독 실행](#6-카테고리별-단독-실행)
7. [개별 체커 단독 실행](#7-개별-체커-단독-실행)
8. [의존성](#8-의존성)
9. [새 체커 추가 가이드](#9-새-체커-추가-가이드)

---

## 1. 아키텍처

### 1-1. 진단 계층 구조

```
┌─────────────────────────────────────────────────────────────┐
│  Platform   CAN 통신 / 차량 상태 / 배터리 / 액추에이터       │
├─────────────────────────────────────────────────────────────┤
│  Hardware   CPU / 메모리 / 디스크 / 온도 / GPU / 네트워크   │
├─────────────────────────────────────────────────────────────┤
│  Sensing    GNSS / IMU / LiDAR / Radar / Camera /           │
│             Wheel Odometry / Cost Grid / Velocity Converter  │
├─────────────────────────────────────────────────────────────┤
│  Localization  GNSS 입력 품질 / 모드·신뢰도 / 위치 추정 출력│
│                초기화 / 소스 전환 / Lanelet 범위             │
├─────────────────────────────────────────────────────────────┤
│  Map        Lanelet 레인 제약 격자                           │
├─────────────────────────────────────────────────────────────┤
│  Perception 장애물 검출 / 융합 출력                          │
├─────────────────────────────────────────────────────────────┤
│  Planning   Lifecycle / Costmap / Nav Status / Path 품질    │
└─────────────────────────────────────────────────────────────┘
                         │
                  /diagnostics
                         │
               ┌─────────▼──────────┐
               │    Aggregator       │
               │  (agg or official)  │
               └─────────┬──────────┘
                         │
                  /diagnostics_agg
```

### 1-2. Aggregator 이중 구조

두 가지 Aggregator 를 제공하며, 용도에 따라 선택합니다.

| 패키지 | 구현 | 특징 | 선택 기준 |
|---|---|---|---|
| `robot_diagnostics_agg` | C++ 커스텀 | YAML 명세 기반 항목별 타임아웃 관리, 그룹별 롤링 로그 | 항목·타임아웃을 세밀하게 제어할 때 |
| `robot_diagnostics_aggregator` | ROS2 official `diagnostic_aggregator` | `GenericAnalyzer` 계층 트리, rqt 완벽 호환 | 표준 rqt UI 와 완전 호환이 필요할 때 |

`system_bringup` 은 기본으로 `robot_diagnostics_agg` 를 사용합니다.

### 1-3. BaseChecker 패턴

모든 체커는 `robot_diagnostics_base::BaseChecker` 를 상속합니다.
Template Method 패턴으로 세 가지 훅을 오버라이드하면 체커가 완성됩니다.

```cpp
class MyChecker : public robot_diagnostics_base::BaseChecker
{
public:
  MyChecker() : BaseChecker("my_checker", "my_hw_id") {
    base_init();  // ← 생성자 마지막에 반드시 호출
  }

protected:
  void declare_parameters_() override { /* ROS2 파라미터 선언 */ }
  void load_parameters_()    override { /* 파라미터 읽기 */      }
  void setup_tasks_()        override {
    add_task("/domain/item", [this](auto & stat) { check(stat); });
  }
};
```

`add_task()` 에 등록된 콜백은 `publish_rate` Hz 주기로 자동 호출되어 `/diagnostics` 에 발행됩니다.

---

## 2. 디렉토리 구조

```
robot_diagnostics/
│
├── base/
│   └── robot_diagnostics_base/         # 모든 체커의 공통 기반 클래스 (헤더 전용)
│
├── agg/
│   ├── robot_diagnostics_agg/          # C++ 커스텀 Aggregator
│   └── robot_diagnostics_aggregator/   # ROS2 official diagnostic_aggregator 래퍼
│
├── checkers/
│   ├── hw/
│   │   ├── robot_hw_gpu_checker/       # CPU / GPU / 메모리 / 디스크 / 온도
│   │   ├── robot_network_checker/      # WiFi 연결 / 신호 / 패킷 품질
│   │   └── robot_hw_checkers_bringup/  # HW 카테고리 전용 bringup
│   │
│   ├── sensing/
│   │   ├── robot_gnss_checker/
│   │   ├── robot_imu_checker/
│   │   ├── robot_lidar_checker/
│   │   ├── robot_radar_checker/
│   │   ├── robot_camera_checker/
│   │   ├── robot_wheel_odometry_checker/
│   │   ├── robot_cost_grid_checker/
│   │   ├── robot_velocity_converter_checker/
│   │   └── robot_sensing_checkers_bringup/
│   │
│   ├── localization/
│   │   ├── robot_localization_gnss_checker/
│   │   ├── robot_localization_mode_checker/
│   │   ├── robot_localization_pose_checker/
│   │   ├── robot_localization_init_checker/
│   │   ├── robot_localization_source_checker/
│   │   ├── robot_localization_lanelet_checker/
│   │   └── robot_localization_checkers_bringup/
│   │
│   ├── map/
│   │   ├── robot_map_cost_grid_checker/
│   │   └── robot_map_checkers_bringup/
│   │
│   ├── perception/
│   │   ├── robot_perception_obstacle_checker/
│   │   └── robot_perception_checkers_bringup/
│   │
│   ├── planning/
│   │   ├── robot_planning_lifecycle_checker/
│   │   ├── robot_planning_costmap_checker/
│   │   ├── robot_planning_nav_status_checker/
│   │   ├── robot_planning_path_checker/
│   │   └── robot_planning_checkers_bringup/
│   │
│   └── platform/
│       ├── robot_ranger_platform_checker/
│       └── robot_platform_checkers_bringup/
│
└── robot_diagnostics_system_bringup/   # 시스템 통합 bringup (운영 배포용)
    ├── config/
    │   ├── default/                    # 전체 공통 기본 설정
    │   └── robots/                     # 로봇별 override yaml
    └── launch/
        ├── system_bringup.launch.py
        ├── system_dummy_test.launch.py
        └── components/                 # 카테고리별 include launch
```

---

## 3. 패키지 목록

### Base / Aggregator

| 패키지 | 역할 | 주요 실행파일 |
|---|---|---|
| `robot_diagnostics_base` | 모든 체커의 기반 클래스 (`BaseChecker`) | — (헤더 전용) |
| `robot_diagnostics_agg` | C++ 커스텀 Aggregator | `aggregator_node` |
| `robot_diagnostics_aggregator` | ROS2 official aggregator 래퍼 | — (launch 전용) |
| `robot_diagnostics_system_bringup` | 시스템 통합 bringup + config 중앙 관리 | — (launch 전용) |

### Hardware

| 패키지 | 구독 토픽 / 소스 | 진단 항목 | 실행파일 |
|---|---|---|---|
| `robot_hw_gpu_checker` | `/proc`, `/sys` (시스템 직접 읽기) | CPU 사용률 · 메모리 · 디스크 · CPU 온도 · GPU 사용률/VRAM/온도 | `hw_checker_node`<br>`gpu_checker_node` |
| `robot_network_checker` | 시스템 네트워크 인터페이스 | WiFi 연결 · 신호 강도 · 패킷 품질 | `network_checker_node` |

### Sensing

| 패키지 | 구독 토픽 | 진단 항목 | 실행파일 |
|---|---|---|---|
| `robot_gnss_checker` | `/sensing/gnss/navsatfix`<br>(`sensor_msgs/NavSatFix`) | 수신율(Hz) · Fix 상태 · 공분산 근사 여부 | `gnss_checker_node` |
| `robot_imu_checker` | `/sensing/imu/data`<br>(`sensor_msgs/Imu`) | 수신율(Hz) · 자이로/가속도 NaN · 가속도 이상 크기 | `imu_checker_node` |
| `robot_lidar_checker` | `/sensing/lidar/points`<br>(`sensor_msgs/PointCloud2`) | 수신율(Hz) · 포인트 수 부족 · NaN 비율 | `lidar_checker_node` |
| `robot_radar_checker` | `/sensor/radar/{REAR,LEFT1,LEFT2,RIGHT1,RIGHT2,FRONT}`<br>(`sensor_msgs/Range`) | 수신율(Hz) · 범위 이상 · 값 고착 | `radar_checker_node` |
| `robot_camera_checker` | `/sensing/camera/front/image_raw`<br>`/sensing/camera/front/camera_info` | FPS · 해상도 · 인코딩 | `camera_checker_node` |
| `robot_wheel_odometry_checker` | `/platform/wheel/odometry`<br>(`nav_msgs/Odometry`) | 수신율(Hz) · NaN 속도 · 과속 감지 | `wheel_odometry_checker_node` |
| `robot_cost_grid_checker` | `/sensing/lidar/near_cost_grid`<br>(`nav_msgs/OccupancyGrid`) | 수신율(Hz) · Unknown 비율 | `cost_grid_checker_node` |
| `robot_velocity_converter_checker` | `/sensing/platform_velocity_converter/twist_with_covariance`<br>(`geometry_msgs/TwistWithCovarianceStamped`) | Silent drop 감지 (주기적 무음 구간) | `velocity_converter_checker_node` |

### Localization

| 패키지 | 구독 토픽 | 진단 항목 | 실행파일 |
|---|---|---|---|
| `robot_localization_gnss_checker` | `/sensing/gnss/pose_with_covariance`<br>(`geometry_msgs/PoseWithCovarianceStamped`) | 수신율(Hz) · 위치 공분산 크기 · 위치 점프 | `localization_gnss_checker_node` |
| `robot_localization_mode_checker` | `/localization/status`<br>(`avg_msgs/AvgLocalizationStatus`) | 모드 (NORMAL/DEGRADED/DR\_ONLY/INVALID) · 신뢰도 · GNSS 유무 · Innovation | `localization_mode_checker_node` |
| `robot_localization_pose_checker` | `/localization/pose_with_covariance`<br>(`geometry_msgs/PoseWithCovarianceStamped`) | 수신율(Hz) · 위치 추정 공분산 · 위치 점프 | `localization_pose_checker_node` |
| `robot_localization_init_checker` | `/localization/initial_match_ok`<br>`/localization/initial_match_distance`<br>`/localization/initial_match_id` | Drop zone 매칭 성공 여부 · 초기화 거리 | `localization_init_checker_node` |
| `robot_localization_source_checker` | `/localization/pose_source`<br>(`std_msgs/String`) | ESKF 기본 / Kimera-VIO 폴백 전환 횟수 · Flapping 감지 | `localization_source_checker_node` |
| `robot_localization_lanelet_checker` | `/localization/lanelet_pose`<br>(`geometry_msgs/PoseWithCovarianceStamped`) | 수신율(Hz) · Lanelet 맵 범위 이탈 · 공분산 | `localization_lanelet_checker_node` |

### Map

| 패키지 | 구독 토픽 | 진단 항목 | 실행파일 |
|---|---|---|---|
| `robot_map_cost_grid_checker` | `/map/cost_grid/lanelet`<br>(`nav_msgs/OccupancyGrid`) | 수신율(Hz) · Unknown 셀 비율 · Silent failure 감지 | `map_cost_grid_checker_node` |

### Perception

| 패키지 | 구독 토픽 | 진단 항목 | 실행파일 |
|---|---|---|---|
| `robot_perception_obstacle_checker` | `/perception/camera/detections_2d`<br>`/perception/obstacles`<br>(`sensor_msgs/PointCloud2`) | 수신율(Hz) · 검출 수 부족 · 포인트 수 부족 | `perception_obstacle_checker_node` |

### Planning

| 패키지 | 구독 토픽 | 진단 항목 | 실행파일 |
|---|---|---|---|
| `robot_planning_lifecycle_checker` | Nav2 노드 lifecycle 서비스 | 각 Nav2 노드 상태 (ACTIVE/INACTIVE/UNCONFIGURED/FINALIZED) | `planning_lifecycle_checker_node` |
| `robot_planning_costmap_checker` | `/planning/global_costmap/costmap`<br>`/planning/local_costmap/costmap`<br>(`nav_msgs/OccupancyGrid`) | Global/Local Costmap 수신 신선도 | `planning_costmap_checker_node` |
| `robot_planning_nav_status_checker` | `/planning/navigate_to_pose/_action/status`<br>(`action_msgs/GoalStatusArray`) | Abort 발생 빈도 · 상태 수신 신선도 | `planning_nav_status_checker_node` |
| `robot_planning_path_checker` | `/planning/global_path`<br>`/planning/local_path`<br>(`nav_msgs/Path`) | 경로 수신 신선도 · 경로 포인트 수 부족 | `planning_path_checker_node` |

### Platform

| 패키지 | 구독 토픽 | 진단 항목 | 실행파일 |
|---|---|---|---|
| `robot_ranger_platform_checker` | `/system_state` (`ranger_msgs/SystemState`)<br>`/battery_state` (`sensor_msgs/BatteryState`)<br>`/actuator_state` (`ranger_msgs/ActuatorStateArray`)<br>`/odom` (`nav_msgs/Odometry`) | CAN 통신 상태 · 제어 모드 (RC/ESTOP) · 배터리 전압/SOC · 액추에이터 결함 · Odometry 수신율 | `ranger_platform_checker_node` |

---

## 4. 빠른 시작

### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-up-to robot_diagnostics_system_bringup
source install/setup.bash
```

### 실행 — 실제 로봇

```bash
# 전체 체커 + Aggregator 실행 (default config)
ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py

# 로봇별 config override 적용
ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py robot:=ranger_a
```

### 실행 — 더미 테스트 (로봇 없이)

```bash
# 전체 OK 시나리오
ros2 launch robot_diagnostics_system_bringup system_dummy_test.launch.py

# 특정 체커에 이상 시나리오 주입
ros2 launch robot_diagnostics_system_bringup system_dummy_test.launch.py \
    imu_scenario:=stale \
    lidar_scenario:=hz_warn \
    loc_mode_scenario:=degraded
```

### 모니터링

```bash
# rqt 런타임 모니터
ros2 run rqt_runtime_monitor rqt_runtime_monitor

# 터미널에서 직접 확인
ros2 topic echo /diagnostics_agg
```

---

## 5. System Bringup 상세

### 5-1. Launch 구조

```
system_bringup.launch.py
│  robot:=<name>  (기본값: default)
│
├── aggregator_node               (robot_diagnostics_agg)
│
└── components/
    ├── hw.launch.py              ← config_dir/hw/
    ├── sensing.launch.py         ← config_dir/sensing/
    ├── localization.launch.py    ← config_dir/localization/
    ├── map.launch.py             ← config_dir/map/
    ├── perception.launch.py      ← config_dir/perception/
    ├── planning.launch.py        ← config_dir/planning/
    └── platform.launch.py        ← config_dir/platform/
```

각 `components/*.launch.py` 는 `config_dir` 인자를 받아 해당 카테고리 yaml 로 체커 노드를 실행합니다.

### 5-2. Config 관리 구조

```
robot_diagnostics_system_bringup/config/
│
├── default/                      ← 공통 기본값 (모든 로봇에 적용)
│   ├── aggregator/
│   │   ├── diagnostics_config.yaml          # robot_diagnostics_agg 설정
│   │   └── robot_diagnostics_aggregator.yaml # ROS2 official aggregator 설정
│   ├── hw/
│   │   ├── hw_gpu_checker.yaml
│   │   └── network_checker.yaml
│   ├── sensing/
│   │   ├── gnss_checker.yaml
│   │   ├── imu_checker.yaml
│   │   ├── lidar_checker.yaml
│   │   ├── radar_checker.yaml
│   │   ├── camera_checker.yaml
│   │   ├── wheel_odometry_checker.yaml
│   │   ├── cost_grid_checker.yaml
│   │   └── velocity_converter_checker.yaml
│   ├── localization/
│   │   ├── localization_gnss_checker.yaml
│   │   ├── localization_mode_checker.yaml
│   │   ├── localization_pose_checker.yaml
│   │   ├── localization_init_checker.yaml
│   │   ├── localization_source_checker.yaml
│   │   └── localization_lanelet_checker.yaml
│   ├── map/
│   │   └── map_cost_grid_checker.yaml
│   ├── perception/
│   │   └── perception_obstacle_checker.yaml
│   ├── planning/
│   │   ├── planning_lifecycle_checker.yaml
│   │   ├── planning_costmap_checker.yaml
│   │   ├── planning_nav_status_checker.yaml
│   │   └── planning_path_checker.yaml
│   └── platform/
│       └── ranger_platform_checker.yaml
│
└── robots/                       ← 로봇별 override (기본값과 다른 yaml만 배치)
    ├── ranger_a/
    │   └── sensing/
    │       └── gnss_checker.yaml  # expected_hz 등 일부 파라미터만 다름
    └── ranger_b/
        └── platform/
            └── ranger_platform_checker.yaml
```

**Override 동작 원리**

`system_bringup.launch.py` 는 `robot:=<name>` 인수를 받아 다음 로직으로 config 디렉토리를 선택합니다.

```
config/robots/<name>/ 가 존재하면 → 해당 디렉토리 사용
존재하지 않으면                  → config/default/ 사용
```

**새 로봇 프로파일 추가 방법**

```bash
# 1. 로봇별 config 디렉토리 생성
mkdir -p config/robots/ranger_c/sensing

# 2. 기본값과 다른 yaml만 복사 후 수정
cp config/default/sensing/gnss_checker.yaml config/robots/ranger_c/sensing/
# gnss_checker.yaml 에서 expected_hz 등 변경

# 3. 실행 시 robot 인수 지정
ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py robot:=ranger_c
```

### 5-3. 더미 테스트 시나리오 인수 전체 목록

```bash
ros2 launch robot_diagnostics_system_bringup system_dummy_test.launch.py \
    [robot:=default]                \
    # ── Sensing ──────────────────
    [gnss_scenario:=ok]             \   # ok|hz_warn|hz_error|stale|no_fix|approx_cov
    [imu_scenario:=ok]              \   # ok|hz_warn|hz_error|stale|nan_gyro|nan_accel|high_accel
    [lidar_scenario:=ok]            \   # ok|hz_warn|hz_error|stale|few_points|high_nan
    [radar_scenario:=ok]            \   # ok|hz_warn|hz_error|stale|out_of_range|stuck_min|stuck_max
    [camera_scenario:=ok]           \   # ok|fps_warn|fps_error|stale|bad_res|bad_enc
    [wheel_scenario:=ok]            \   # ok|hz_warn|hz_error|stale|nan_velocity|high_speed
    [cost_grid_scenario:=ok]        \   # ok|hz_warn|hz_error|stale|high_unknown|full_unknown
    # velocity_converter 는 인수 없음 — 8s 주기 자동 전환
    # ── Perception ───────────────
    [perception_scenario:=ok]       \   # ok|hz_warn|hz_error|stale_det|stale_pc2|stale_both|few_points
    # ── Localization ─────────────
    [loc_gnss_scenario:=ok]         \   # ok|hz_warn|hz_error|stale|high_cov_warn|high_cov_error|jump
    [loc_mode_scenario:=ok]         \   # ok|degraded|dr_only|invalid|low_conf|no_gnss|high_innov|stale
    [loc_pose_scenario:=ok]         \   # ok|hz_warn|hz_error|stale|high_cov_warn|high_cov_error|jump
    [loc_init_scenario:=ok]         \   # ok|not_matched|far|stale
    [loc_source_scenario:=ok]       \   # ok|fallback|flapping|unknown|stale
    [loc_lanelet_scenario:=ok]      \   # ok|hz_warn|hz_error|stale|high_cov|no_snap
    # ── Planning ─────────────────
    [planning_path_scenario:=ok]    \   # ok|idle|stale_global|stale_local|stale_both|few_warn|few_error
    [planning_cm_scenario:=ok]      \   # ok|stale_global|stale_local|stale_both
    [planning_nav_scenario:=ok]     \   # ok|idle|stale|aborted_once|aborted_warn|aborted_error
    [planning_lc_scenario:=ok]      \   # ok|inactive|unconfigured|finalized|mixed
    # ── Map ──────────────────────
    [map_scenario:=ok]              \   # ok|hz_warn|hz_error|stale|high_unknown|full_unknown
    # ── Platform ─────────────────
    [platform_scenario:=ok]             # ok|estop|rc_mode|low_battery|error_code|stale|actuator_fault
```

---

## 6. 카테고리별 단독 실행

전체 시스템 대신 특정 도메인 체커만 올릴 때 사용합니다.

```bash
# ── Hardware ───────────────────────────────────────────────────────────
ros2 launch robot_hw_checkers_bringup hw_checkers_bringup.launch.py
ros2 launch robot_hw_checkers_bringup hw_checkers_dummy_test.launch.py

# ── Sensing ────────────────────────────────────────────────────────────
ros2 launch robot_sensing_checkers_bringup sensing_checkers_bringup.launch.py
ros2 launch robot_sensing_checkers_bringup sensing_checkers_dummy_test.launch.py \
    gnss_scenario:=no_fix imu_scenario:=stale

# ── Localization ───────────────────────────────────────────────────────
ros2 launch robot_localization_checkers_bringup localization_checkers_bringup.launch.py
ros2 launch robot_localization_checkers_bringup localization_checkers_dummy_test.launch.py \
    loc_mode_scenario:=degraded

# ── Map ────────────────────────────────────────────────────────────────
ros2 launch robot_map_checkers_bringup map_checkers_bringup.launch.py
ros2 launch robot_map_checkers_bringup map_checkers_dummy_test.launch.py \
    map_scenario:=high_unknown

# ── Perception ─────────────────────────────────────────────────────────
ros2 launch robot_perception_checkers_bringup perception_checkers_bringup.launch.py
ros2 launch robot_perception_checkers_bringup perception_checkers_dummy_test.launch.py \
    perception_scenario:=stale_both

# ── Planning ───────────────────────────────────────────────────────────
ros2 launch robot_planning_checkers_bringup planning_checkers_bringup.launch.py
ros2 launch robot_planning_checkers_bringup planning_checkers_dummy_test.launch.py \
    planning_lc_scenario:=inactive

# ── Platform ───────────────────────────────────────────────────────────
ros2 launch robot_platform_checkers_bringup platform_checkers_bringup.launch.py
ros2 launch robot_platform_checkers_bringup platform_checkers_dummy_test.launch.py \
    platform_scenario:=low_battery
```

---

## 7. 개별 체커 단독 실행

패키지 내부 launch 파일로 단일 체커만 실행합니다. config 는 패키지 내 `config/` 를 사용합니다.

```bash
# 형식: ros2 launch <package> <checker>.launch.py         (실제 토픽 필요)
#       ros2 launch <package> <checker>_test.launch.py    (더미 포함)

ros2 launch robot_gnss_checker gnss_checker.launch.py
ros2 launch robot_gnss_checker gnss_test.launch.py

ros2 launch robot_imu_checker  imu_test.launch.py
ros2 launch robot_lidar_checker lidar_test.launch.py

ros2 launch robot_localization_mode_checker localization_mode_test.launch.py \
    scenario:=degraded

ros2 launch robot_ranger_platform_checker ranger_platform_checker.launch.py
```

> **참고** — 개별 체커 패키지의 `config/` yaml 은 단위 테스트 및 단독 실행용으로 유지됩니다.
> 운영 배포 시에는 `robot_diagnostics_system_bringup/config/default/` 를 기준으로 하세요.

---

## 8. 의존성

### 빌드 / 런타임 공통

| 패키지 | 용도 |
|---|---|
| `rclcpp` | ROS 2 C++ 클라이언트 라이브러리 |
| `diagnostic_msgs` | `DiagnosticStatus` / `DiagnosticArray` 메시지 |
| `diagnostic_updater` | 체커 노드의 `/diagnostics` 발행 유틸리티 |
| `sensor_msgs` | NavSatFix, Imu, PointCloud2, Range, Image, BatteryState |
| `nav_msgs` | Odometry, OccupancyGrid, Path |
| `geometry_msgs` | PoseWithCovarianceStamped, TwistWithCovarianceStamped |
| `action_msgs` | GoalStatusArray (Planning Nav Status) |

### 선택적 의존성

| 패키지 | 필요 체커 | 비고 |
|---|---|---|
| `avg_msgs` | `robot_localization_mode_checker` | `AvgLocalizationStatus` 메시지 |
| `ranger_msgs` | `robot_ranger_platform_checker` | `SystemState`, `ActuatorStateArray` |
| `vision_msgs` | `robot_perception_obstacle_checker` | `Detection2DArray` |
| `diagnostic_aggregator` | `robot_diagnostics_aggregator` | ROS2 official aggregator |

### 설치

```bash
# ROS 2 Humble 기준
sudo apt install \
    ros-humble-diagnostic-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-diagnostic-aggregator

# avg_msgs (내부 패키지)
source ~/camload/install/setup.bash
```

---

## 9. 새 체커 추가 가이드

새로운 진단 항목이 생겼을 때 아래 절차를 따릅니다.

### Step 1 — 패키지 생성

```bash
cd ~/ros2_ws/src/robot_diagnostics/checkers/<카테고리>/
ros2 pkg create robot_<domain>_<name>_checker \
    --build-type ament_cmake \
    --dependencies rclcpp diagnostic_msgs diagnostic_updater robot_diagnostics_base
```

### Step 2 — BaseChecker 상속

```cpp
// src/my_checker_node.cpp
#include "robot_diagnostics_base/base_checker.hpp"

class MyChecker : public robot_diagnostics_base::BaseChecker
{
public:
  MyChecker() : BaseChecker("my_checker", "my_hw_id") {
    base_init();
  }

protected:
  void declare_parameters_() override {
    declare_parameter("warn_threshold", 80.0);
  }
  void load_parameters_() override {
    warn_ = get_parameter("warn_threshold").as_double();
    sub_ = create_subscription<SomeMsgType>(
      "/some/topic", 10,
      [this](const SomeMsgType::SharedPtr msg) { last_msg_ = msg; });
  }
  void setup_tasks_() override {
    add_task("/domain/item", [this](auto & stat) {
      if (!last_msg_) { stat.summary(S::STALE, "No data"); return; }
      // 판정 로직
      stat.summary(S::OK, "OK");
    });
  }

private:
  double warn_{80.0};
  rclcpp::Subscription<SomeMsgType>::SharedPtr sub_;
  SomeMsgType::SharedPtr last_msg_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyChecker>());
  rclcpp::shutdown();
}
```

### Step 3 — config yaml 작성

```yaml
# config/my_checker.yaml
my_checker:
  ros__parameters:
    publish_rate: 1.0
    warn_threshold: 80.0
```

### Step 4 — system_bringup 에 통합

```bash
# 1. system_bringup config/default/<카테고리>/ 에 yaml 복사
cp config/my_checker.yaml \
   ~/ros2_ws/src/robot_diagnostics/robot_diagnostics_system_bringup/config/default/<카테고리>/

# 2. components/<카테고리>.launch.py 에 Node 추가
#    (config_dir 인자로 yaml 경로를 받는 패턴 유지)

# 3. system_bringup/package.xml 에 exec_depend 추가
#    <exec_depend>robot_<domain>_<name>_checker</exec_depend>
```

### Step 5 — 더미 퍼블리셔 작성 (선택)

```cpp
// src/my_dummy_publisher.cpp
// scenario 파라미터를 받아 ok / warn / error / stale 상태를 발행하는 노드
```

`system_dummy_test.launch.py` 에 시나리오 인수와 함께 등록합니다.

### Step 6 — 빌드 및 확인

```bash
colcon build --packages-select robot_<domain>_<name>_checker
source install/setup.bash
ros2 launch robot_<domain>_<name>_checker my_checker_test.launch.py
ros2 topic echo /diagnostics
```
