# robot_diagnostics_agg

ROS 2 패키지로, `/diagnostics` 토픽을 구독해 **항목별 타임아웃 감지 및 그룹별 최악 레벨 집계**를
`/diagnostics_agg` 토픽으로 발행합니다.

YAML config 파일에 등록된 항목들이 정해진 시간 안에 데이터를 발행하는지,
각 도메인 그룹의 전체 상태는 어떤 레벨인지를 실시간으로 집계합니다.

> **커스텀 C++ Aggregator**: `diagnostic_aggregator` (ROS 2 공식 패키지) 와는 별개의 독립 노드입니다.
> 항목별 timeout / 그룹별 요약 로직을 직접 구현하여 설정 유연성과 로그 가독성을 높였습니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [config 파일 구조](#6-config-파일-구조)
7. [모니터링 항목 전체 목록](#7-모니터링-항목-전체-목록)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [항목 추가 방법](#10-항목-추가-방법)
11. [자주 묻는 질문 (FAQ)](#11-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

```
/diagnostics  ──►  aggregator_node  ──►  /diagnostics_agg
                        ▲
                        │
                  1초마다 집계 + 발행
```

노드는 `/diagnostics` 를 구독하고, `DiagnosticStatus.name` 기준으로 각 항목을 추적합니다.

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 항목 수신 여부 | `timeout_sec` 이내에 메시지 수신 | STALE (회색) |
| 항목 상태 레벨 | 수신된 레벨 그대로 반영 | WARN / ERROR |
| config에 없는 항목 | — | WARN 로그 + unknown 그룹 처리 |

그룹별로 **하위 항목 중 가장 나쁜 레벨**을 로그에 출력합니다.

```
[AGG] total=23 | hardware=OK | localization/gnss=WARN | sensing/lidar=OK | map/cost_grid=OK | ...
```

> **`name` 매칭**: `DiagnosticStatus.name` 은 `diagnostic_updater` 가 자동으로
> `"<노드이름>: <add_task() 경로>"` 형식으로 생성합니다.
> config의 `name` 값과 정확히 일치해야 추적됩니다.

> **STALE 처리**: 마지막 수신 후 `timeout_sec` 초가 지나면 해당 항목을 STALE 로 덮어씁니다.
> 원본 메시지를 변경하는 것이 아니라 발행 시점에 재계산합니다.

---

## 2. 패키지 구성

```
robot_diagnostics_agg/
├── config/
│   └── diagnostics_config.yaml          # 모니터링 항목 및 timeout 설정
├── launch/
│   ├── diagnostics_agg.launch.py        # aggregator 단독 실행
│   └── diagnostics_agg_test.launch.py   # aggregator + dummy_publisher 테스트
├── src/
│   ├── aggregator_node.cpp              # 메인 집계 노드
│   └── dummy_publisher.cpp              # 테스트용 더미 퍼블리셔
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `diagnostic_msgs` | `DiagnosticStatus` / `DiagnosticArray` 메시지 타입 |
| `yaml_cpp_vendor` | YAML config 파일 파싱 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_diagnostics_agg
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (aggregator만)
ros2 launch robot_diagnostics_agg diagnostics_agg.launch.py

# 테스트용 (aggregator + 더미 퍼블리셔)
ros2 launch robot_diagnostics_agg diagnostics_agg_test.launch.py
```

---

## 6. config 파일 구조

`config/diagnostics_config.yaml`

```yaml
global:
  timeout_sec: 5.0      # 전역 기본 timeout (항목별로 개별 설정 가능)
  publish_rate: 1.0     # /diagnostics_agg 발행 주기 (Hz)

topics:
  - name: "<node_name>: <task_path>"   # DiagnosticStatus.name 과 정확히 일치해야 함
    group: <domain/subdomain>           # 로그 집계용 그룹명
    timeout_sec: 3.0                    # 항목별 개별 timeout (생략 시 global 값 사용)
```

### `name` 형식 규칙

`diagnostic_updater` 가 자동 생성하는 형식을 그대로 사용합니다.

```
"<노드이름>: <add_task() 경로>"
예) "gnss_checker: /sensor/gnss/main"
    "radar_checker: /sensor/radar/REAR"
```

노드 이름은 체커 패키지 소스의 `BaseChecker` 생성자 인수에서 확인할 수 있습니다.

---

## 7. 모니터링 항목 전체 목록

### Hardware

| name | group | timeout |
|------|-------|---------|
| `hw_checker: /hardware/cpu` | hardware | 5.0s |
| `hw_checker: /hardware/memory` | hardware | 5.0s |
| `hw_checker: /hardware/disk` | hardware | 5.0s |
| `hw_checker: /hardware/cpu_temp` | hardware | 5.0s |
| `gpu_checker: /hardware/gpu0` | hardware | 5.0s |

### Sensing

| name | group | timeout |
|------|-------|---------|
| `gnss_checker: /sensor/gnss/main` | sensing/gnss | 3.0s |
| `imu_checker: /sensor/imu/main` | sensing/imu | 2.0s |
| `lidar_checker: /sensor/lidar/main` | sensing/lidar | 3.0s |
| `radar_checker: /sensor/radar/REAR` | sensing/radar | 3.0s |
| `radar_checker: /sensor/radar/LEFT1` | sensing/radar | 3.0s |
| `radar_checker: /sensor/radar/LEFT2` | sensing/radar | 3.0s |
| `radar_checker: /sensor/radar/RIGHT1` | sensing/radar | 3.0s |
| `radar_checker: /sensor/radar/RIGHT2` | sensing/radar | 3.0s |
| `radar_checker: /sensor/radar/FRONT` | sensing/radar | 3.0s |
| `camera_checker: /sensor/camera/front` | sensing/camera | 3.0s |
| `wheel_odometry_checker: /sensor/wheel/main` | sensing/wheel | 3.0s |
| `cost_grid_checker: /perception/lidar/cost_grid` | sensing/perception | 5.0s |
| `velocity_converter_checker: /sensing/velocity_converter` | sensing/velocity | 3.0s |

### Localization

| name | group | timeout |
|------|-------|---------|
| `localization_gnss_checker: /localization/gnss/main` | localization/gnss | 5.0s |
| `localization_mode_checker: /localization/mode` | localization/mode | 5.0s |
| `localization_pose_checker: /localization/pose` | localization/pose | 3.0s |
| `localization_init_checker: /localization/init` | localization/init | 10.0s |
| `localization_source_checker: /localization/source` | localization/source | 5.0s |
| `localization_lanelet_checker: /localization/lanelet` | localization/lanelet | 3.0s |

### Map

| name | group | timeout |
|------|-------|---------|
| `map_cost_grid_checker: /map/cost_grid` | map/cost_grid | 5.0s |

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 집계 결과 구독
ros2 topic echo /diagnostics_agg

# 그룹별 요약은 aggregator_node 로그에서 확인
ros2 launch robot_diagnostics_agg diagnostics_agg.launch.py
```

### 출력 예시 (정상)

```
[AGG] total=23 | hardware=OK | localization/gnss=OK | localization/mode=OK | map/cost_grid=OK | sensing/gnss=OK | ...
```

### 출력 예시 (항목 STALE)

```
name:    "lidar_checker: /sensor/lidar/main"
level:   3                        ← 3 = STALE
message: "STALE (last seen 6.2s ago)"
```

### 출력 예시 (config에 없는 항목 수신)

```
[WARN] config에 없는 토픽 수신: "new_checker: /some/path" → unknown 그룹으로 처리
```

### GUI로 확인 (rqt)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 9. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | 정상 동작 중 |
| WARN | 1 | 노랑 | 주의 필요 (동작하지만 품질 저하) |
| ERROR | 2 | 빨강 | 오류 발생 (즉각 대응 필요) |
| STALE | 3 | 회색 | 데이터 없음 (토픽 미수신 또는 타임아웃) |

---

## 10. 항목 추가 방법

새 체커 패키지를 추가했다면 `config/diagnostics_config.yaml`의 `topics:` 아래에 항목을 추가합니다.

```yaml
# 예: Planning 도메인 추가
- name: "planning_path_checker: /planning/path"
  group: planning/path
  timeout_sec: 3.0
```

`name` 값은 체커 노드의 생성자에서 지정한 **노드 이름** + `": "` + `add_task()` 에 넘긴 **경로**입니다.

```bash
# 실제 name 값 확인 방법
ros2 topic echo /diagnostics | grep "name:"
```

---

## 11. 자주 묻는 질문 (FAQ)

**Q. config에 등록했는데 해당 항목이 STALE로 나옵니다.**

체커 노드가 실행 중인지, `/diagnostics` 토픽을 발행하는지 확인하세요.

```bash
ros2 topic echo /diagnostics | grep "name:"
ros2 node list
```

---

**Q. "config에 없는 토픽 수신" 경고가 계속 나옵니다.**

새로 추가된 체커 패키지가 있으면 `diagnostics_config.yaml`에 항목을 추가하세요.
해당 항목의 `DiagnosticStatus.name` 을 먼저 확인합니다.

```bash
ros2 topic echo /diagnostics | grep "name:" | sort -u
```

---

**Q. timeout을 도메인별로 다르게 설정하고 싶습니다.**

각 항목에 `timeout_sec` 를 개별로 지정합니다. 지정하지 않으면 `global.timeout_sec` 가 사용됩니다.

```yaml
- name: "imu_checker: /sensor/imu/main"
  group: sensing/imu
  timeout_sec: 2.0   # IMU는 더 엄격하게
```

---

**Q. 발행 주기를 바꾸고 싶습니다.**

```yaml
global:
  publish_rate: 2.0   # 2Hz로 발행
```
