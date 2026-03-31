# robot_diagnostics_system_bringup

Robot Diagnostics 전체 시스템을 단일 launch 파일로 실행하는 통합 Bringup 패키지.
모든 체커 노드 + Aggregator를 **중앙 집중형 Config** 기반으로 관리한다.

---

## 목차

1. [패키지 역할](#패키지-역할)
2. [디렉토리 구조](#디렉토리-구조)
3. [Config 관리 구조](#config-관리-구조)
4. [실행 방법](#실행-방법)
5. [로봇별 Config 오버라이드](#로봇별-config-오버라이드)
6. [Dummy Test (오프라인 검증)](#dummy-test-오프라인-검증)
7. [포함 노드 목록](#포함-노드-목록)
8. [Aggregator 설정](#aggregator-설정)

---

## 패키지 역할

```
각 체커 패키지 (25개)
        │
        ▼
robot_diagnostics_system_bringup
  ├─ config/default/      ← 공통 기본 파라미터 (중앙 관리)
  ├─ config/robots/       ← 로봇별 오버라이드 (선택)
  └─ launch/
       ├─ system_bringup.launch.py       ← 실제 로봇 실행
       ├─ system_dummy_test.launch.py    ← 오프라인 전체 검증
       └─ components/{category}.launch.py
```

- **단일 진입점**: `system_bringup.launch.py` 하나로 전체 24개 진단 노드 + Aggregator 기동
- **Config 중앙화**: 개별 패키지 config를 이 패키지의 `config/default/`에 모아 일괄 관리
- **로봇별 오버라이드**: `robot:=<이름>` 인수로 로봇별 파라미터를 런타임에 자동 선택
- **오프라인 검증**: Dummy Publisher 포함 `system_dummy_test.launch.py`로 실제 로봇 없이 파이프라인 전체 테스트 가능

---

## 디렉토리 구조

```
robot_diagnostics_system_bringup/
├── config/
│   ├── default/                        ← 기본 Config (모든 로봇 공통)
│   │   ├── aggregator/
│   │   │   ├── diagnostics_config.yaml     집계 토픽 목록 + timeout 설정
│   │   │   └── robot_diagnostics_aggregator.yaml
│   │   ├── hw/
│   │   │   ├── hw_gpu_checker.yaml
│   │   │   └── network_checker.yaml
│   │   ├── sensing/
│   │   │   ├── gnss_checker.yaml
│   │   │   ├── imu_checker.yaml
│   │   │   ├── lidar_checker.yaml
│   │   │   ├── radar_checker.yaml
│   │   │   ├── camera_checker.yaml
│   │   │   ├── wheel_odometry_checker.yaml
│   │   │   ├── cost_grid_checker.yaml
│   │   │   └── velocity_converter_checker.yaml
│   │   ├── localization/
│   │   │   ├── localization_gnss_checker.yaml
│   │   │   ├── localization_mode_checker.yaml
│   │   │   ├── localization_pose_checker.yaml
│   │   │   ├── localization_init_checker.yaml
│   │   │   ├── localization_source_checker.yaml
│   │   │   └── localization_lanelet_checker.yaml
│   │   ├── map/
│   │   │   └── map_cost_grid_checker.yaml
│   │   ├── perception/
│   │   │   └── perception_obstacle_checker.yaml
│   │   ├── planning/
│   │   │   ├── planning_lifecycle_checker.yaml
│   │   │   ├── planning_costmap_checker.yaml
│   │   │   ├── planning_nav_status_checker.yaml
│   │   │   └── planning_path_checker.yaml
│   │   └── platform/
│   │       └── ranger_platform_checker.yaml
│   └── robots/                         ← 로봇별 오버라이드 (필요한 yaml만 배치)
│       └── {robot_name}/               예) ranger_a/, ranger_b/
│           └── sensing/
│               └── lidar_checker.yaml  ← 이 파일만 해당 로봇 값으로 적용
├── launch/
│   ├── system_bringup.launch.py        ← 실제 로봇 실행용
│   ├── system_dummy_test.launch.py     ← 오프라인 검증용
│   └── components/                     ← 카테고리별 서브 launch
│       ├── hw.launch.py
│       ├── sensing.launch.py
│       ├── localization.launch.py
│       ├── map.launch.py
│       ├── perception.launch.py
│       ├── planning.launch.py
│       └── platform.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Config 관리 구조

### 기본 원칙

```
config/default/          ← 항상 존재. 모든 로봇의 기준값
config/robots/{name}/    ← 선택적. 해당 로봇에서 달라지는 yaml만 배치
```

launch 시 `robot:=<이름>` 인수가 주어지면:

```python
robot_config = config/robots/{name}/   # 존재하면 이 경로 사용
default_config = config/default/       # 없으면 fallback
config_dir = robot_config if os.path.isdir(robot_config) else default_config
```

> **주의**: 현재 구현은 config_dir 전체를 교체한다.
> 로봇별 오버라이드 디렉토리에는 **카테고리 하위 구조 전체**가 필요하다.

### Config 파일 수정 방법

```bash
# 설치된 경로 확인
ros2 pkg prefix robot_diagnostics_system_bringup

# config 파일 위치
$ROS_WS/install/robot_diagnostics_system_bringup/share/
  robot_diagnostics_system_bringup/config/default/
```

소스 수정 후 다시 빌드해야 install/share/에 반영된다:

```bash
colcon build --packages-select robot_diagnostics_system_bringup
```

---

## 실행 방법

### 기본 실행 (default config)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py
```

### 로봇 프로파일 지정

```bash
ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py robot:=ranger_a
```

`config/robots/ranger_a/` 디렉토리가 있으면 해당 config를 사용하고,
없으면 자동으로 `config/default/`로 fallback.

### 진단 결과 모니터링

```bash
# 원시 진단 메시지
ros2 topic echo /diagnostics

# Aggregator 집계 결과
ros2 topic echo /diagnostics_agg

# GUI 모니터
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 로봇별 Config 오버라이드

새 로봇 `ranger_b`의 LiDAR 주파수가 다를 경우:

**1. 오버라이드 디렉토리 생성**

```bash
mkdir -p src/robot_diagnostics/robot_diagnostics_system_bringup/config/robots/ranger_b/sensing
```

**2. 변경이 필요한 yaml만 복사 후 수정**

```bash
cp config/default/sensing/lidar_checker.yaml \
   config/robots/ranger_b/sensing/lidar_checker.yaml
# 값 수정
```

> 현재 구현에서는 config_dir 전체가 교체되므로, `ranger_b/` 하위에
> 모든 카테고리 yaml을 배치해야 한다 (default와 동일한 값이라도).

**3. 빌드 후 실행**

```bash
colcon build --packages-select robot_diagnostics_system_bringup
ros2 launch robot_diagnostics_system_bringup system_bringup.launch.py robot:=ranger_b
```

---

## Dummy Test (오프라인 검증)

실제 로봇 없이 전체 진단 파이프라인을 검증할 수 있다.
각 체커 패키지의 Dummy Publisher가 시나리오에 따라 토픽을 발행한다.

### 기본 실행 (전체 OK 시나리오)

```bash
ros2 launch robot_diagnostics_system_bringup system_dummy_test.launch.py
```

### 특정 시나리오 조합

```bash
ros2 launch robot_diagnostics_system_bringup system_dummy_test.launch.py \
    imu_scenario:=stale \
    lidar_scenario:=hz_warn \
    loc_mode_scenario:=degraded
```

### 시나리오 인수 목록

| 인수 | 가능한 값 |
|------|-----------|
| `gnss_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `no_fix` \| `approx_cov` |
| `imu_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `nan_gyro` \| `nan_accel` \| `high_accel` |
| `lidar_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `few_points` \| `high_nan` |
| `radar_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `out_of_range` \| `stuck_min` \| `stuck_max` |
| `camera_scenario` | `ok` \| `fps_warn` \| `fps_error` \| `stale` \| `bad_res` \| `bad_enc` |
| `wheel_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `nan_velocity` \| `high_speed` |
| `cost_grid_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `high_unknown` \| `full_unknown` |
| `perception_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale_det` \| `stale_pc2` \| `stale_both` \| `few_points` |
| `loc_gnss_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `high_cov_warn` \| `high_cov_error` \| `jump` |
| `loc_mode_scenario` | `ok` \| `degraded` \| `dr_only` \| `invalid` \| `low_conf` \| `no_gnss` \| `high_innov` \| `stale` |
| `loc_pose_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `high_cov_warn` \| `high_cov_error` \| `jump` |
| `loc_init_scenario` | `ok` \| `not_matched` \| `far` \| `stale` |
| `loc_source_scenario` | `ok` \| `fallback` \| `flapping` \| `unknown` \| `stale` |
| `loc_lanelet_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `high_cov` \| `no_snap` |
| `planning_path_scenario` | `ok` \| `idle` \| `stale_global` \| `stale_local` \| `stale_both` \| `few_warn` \| `few_error` |
| `planning_cm_scenario` | `ok` \| `stale_global` \| `stale_local` \| `stale_both` |
| `planning_nav_scenario` | `ok` \| `idle` \| `stale` \| `aborted_once` \| `aborted_warn` \| `aborted_error` |
| `planning_lc_scenario` | `ok` \| `inactive` \| `unconfigured` \| `finalized` \| `mixed` |
| `map_scenario` | `ok` \| `hz_warn` \| `hz_error` \| `stale` \| `high_unknown` \| `full_unknown` |
| `platform_scenario` | `ok` \| `estop` \| `rc_mode` \| `low_battery` \| `error_code` \| `stale` \| `actuator_fault` |

> `velocity_converter`는 시나리오 인수 없음 — 8초 주기로 자동 전환.

---

## 포함 노드 목록

### Hardware (3노드)

| 노드명 | 실행 파일 | 모니터링 대상 |
|--------|-----------|--------------|
| `hw_checker` | `hw_checker_node` | CPU / Memory / Disk / CPU 온도 |
| `gpu_checker` | `gpu_checker_node` | GPU 사용률 / 온도 |
| `network_checker` | `network_checker_node` | WiFi 연결 / 신호세기 / 패킷 품질 |

### Sensing (8노드)

| 노드명 | 모니터링 토픽 |
|--------|--------------|
| `gnss_checker` | `/sensing/gnss/navsatfix` |
| `imu_checker` | `/sensing/imu/data` |
| `lidar_checker` | `/sensing/lidar/points` |
| `radar_checker` | `/sensing/radar/*/range` (6채널) |
| `camera_checker` | `/sensing/camera/front/image_raw` |
| `wheel_odometry_checker` | `/platform/wheel/odometry` |
| `cost_grid_checker` | `/sensing/lidar/near_cost_grid` |
| `velocity_converter_checker` | `/sensing/platform_velocity_converter/...` |

### Localization (6노드)

| 노드명 | 모니터링 토픽 |
|--------|--------------|
| `localization_gnss_checker` | `/sensing/gnss/pose_with_covariance` |
| `localization_mode_checker` | `/localization/status` |
| `localization_pose_checker` | `/localization/pose_with_covariance` |
| `localization_init_checker` | `/localization/initial_match_*` |
| `localization_source_checker` | `/localization/pose_source` |
| `localization_lanelet_checker` | `/localization/lanelet_pose` |

### Map (1노드)

| 노드명 | 모니터링 토픽 |
|--------|--------------|
| `map_cost_grid_checker` | `/map/cost_grid/lanelet` |

### Perception (1노드)

| 노드명 | 모니터링 토픽 |
|--------|--------------|
| `perception_obstacle_checker` | `/perception/camera/detections_2d`, `/perception/obstacles` |

### Planning (4노드)

| 노드명 | 모니터링 대상 |
|--------|--------------|
| `planning_lifecycle_checker` | Nav2 lifecycle 노드 4개 상태 |
| `planning_costmap_checker` | `/planning/*/costmap` |
| `planning_nav_status_checker` | `/planning/navigate_to_pose/_action/status` |
| `planning_path_checker` | `/planning/global_path`, `/planning/local_path` |

### Platform (1노드)

| 노드명 | 모니터링 토픽 |
|--------|--------------|
| `ranger_platform_checker` | `/system_state`, `/battery_state`, `/actuator_state`, `/odom` |

### Aggregator (1노드)

| 노드명 | 역할 |
|--------|------|
| `diagnostics_agg` | `/diagnostics` → `/diagnostics_agg` 집계 (C++ custom) |

---

## Aggregator 설정

`config/default/aggregator/diagnostics_config.yaml`에서 집계 대상 토픽과 timeout을 관리한다.

```yaml
global:
  timeout_sec: 5.0    # 기본 STALE 판정 시간
  publish_rate: 1.0   # /diagnostics_agg 발행 주기 (Hz)

topics:
  - name: "gnss_checker: /sensor/gnss/main"
    group: sensing/gnss
    timeout_sec: 3.0
  # ...
```

새 체커를 추가할 때 이 파일에도 항목을 추가해야 `/diagnostics_agg`에 집계된다.
