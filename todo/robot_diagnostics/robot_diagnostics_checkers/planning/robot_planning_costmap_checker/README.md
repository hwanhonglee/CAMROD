# robot_planning_costmap_checker

ROS 2 패키지로, Nav2 global / local costmap 토픽의 staleness 를
`/diagnostics` 토픽으로 발행합니다.

costmap 이 갱신되지 않으면 planner 와 controller 가 오래된 장애물 정보를 바탕으로
경로를 계획하게 되어 충돌이나 경로 실패의 원인이 됩니다.

> **전제 조건**: [`robot_planning_lifecycle_checker`](../robot_planning_lifecycle_checker/README.md) 가
> ERROR 를 발행 중이라면, 이 패키지의 진단 결과는 신뢰할 수 없습니다.
> Nav2 노드가 ACTIVE 상태인지 먼저 확인하세요.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [파라미터 설명](#6-파라미터-설명)
7. [진단 결과 확인하기](#7-진단-결과-확인하기)
8. [진단 상태 레벨이란?](#8-진단-상태-레벨이란)
9. [자주 묻는 질문 (FAQ)](#9-자주-묻는-질문-faq)
10. [고려했으나 제외된 항목](#10-고려했으나-제외된-항목)
11. [운용 가이드](#11-운용-가이드)

---

## 1. 어떻게 동작하나요?

```
/planning/global_costmap/costmap ──┐
  (nav_msgs/OccupancyGrid)         ├──► planning_costmap_checker_node ──► /diagnostics
/planning/local_costmap/costmap  ──┘                ▲
  (nav_msgs/OccupancyGrid)                          │
                                              1초마다 체크
```

두 costmap 토픽을 각각 독립적으로 모니터링합니다.
마지막 수신 시간을 기록하고 `stale_timeout` 초과 여부를 확인합니다.

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 수신됨 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | ERROR (빨강) |

### 구독 토픽 QoS

| 토픽 | 타입 | Reliability | Durability |
|------|------|-------------|------------|
| `global_costmap_topic` | `nav_msgs/OccupancyGrid` | `reliable` | `transient_local` |
| `local_costmap_topic` | `nav_msgs/OccupancyGrid` | `reliable` | `transient_local` |

`transient_local` 은 Nav2 costmap 의 기본 발행 QoS 와 일치합니다.
노드 기동 순서와 무관하게 마지막 메시지를 수신할 수 있습니다.
QoS 불일치 의심 시:

```bash
ros2 topic info /planning/global_costmap/costmap --verbose
```

---

## 2. 패키지 구성

```
robot_planning_costmap_checker/
├── src/
│   ├── planning_costmap_checker_node.cpp      # 메인 노드 코드
│   └── planning_costmap_dummy_publisher.cpp   # 테스트용 더미 퍼블리셔
├── config/
│   └── planning_costmap_checker.yaml          # 설정 파일
├── launch/
│   ├── planning_costmap_checker.launch.py     # 운영용 실행 파일
│   └── planning_costmap_test.launch.py        # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `nav_msgs` | `OccupancyGrid` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_planning_costmap_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_planning_costmap_checker planning_costmap_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_planning_costmap_checker planning_costmap_test.launch.py
ros2 launch robot_planning_costmap_checker planning_costmap_test.launch.py scenario:=stale_global
ros2 launch robot_planning_costmap_checker planning_costmap_test.launch.py scenario:=stale_local
ros2 launch robot_planning_costmap_checker planning_costmap_test.launch.py scenario:=stale_both
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | global + local 정상 발행 (1Hz) | 둘 다 OK |
| `stale_global` | global 발행 중단 | global ERROR, local OK |
| `stale_local` | local 발행 중단 | global OK, local ERROR |
| `stale_both` | 전부 발행 중단 | 둘 다 STALE |

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `global_costmap_topic` | `/planning/global_costmap/costmap` | global costmap 토픽 |
| `local_costmap_topic` | `/planning/local_costmap/costmap` | local costmap 토픽 |
| `stale_timeout` | `5.0` | 이 시간(초) 이상 미갱신이면 ERROR |

### stale_timeout 설정 가이드

Nav2 costmap 갱신 주기(`update_frequency`)와 여유배수를 곱해 설정합니다.

| Nav2 update_frequency | 권장 stale_timeout |
|-----------------------|-------------------|
| 5 Hz (기본값) | 2.0s (5배 여유) |
| 2 Hz | 5.0s (10배 여유) |
| 1 Hz | 10.0s |

### 런타임 파라미터 변경

파라미터는 **노드 기동 시 1회 로드**되며, `ros2 param set` 으로 변경해도 즉시 반영되지 않습니다.
임계값 변경 후에는 노드를 재시작해야 합니다.

```bash
# 현재 값 확인
ros2 param get /planning_costmap_checker stale_timeout
```

---

## 7. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 8 "planning/global_costmap\|planning/local_costmap"
```

### 출력 예시 (정상)

```
name:    "/planning/global_costmap"
level:   0                          ← OK
message: "OK (0.2s ago)"
values:
  - key: "last_msg_sec_ago" value: "0.20"

name:    "/planning/local_costmap"
level:   0                          ← OK
message: "OK (0.1s ago)"
values:
  - key: "last_msg_sec_ago" value: "0.10"
```

### 출력 예시 (ERROR — stale)

```
name:    "/planning/global_costmap"
level:   2                          ← ERROR
message: "6.3s 동안 costmap 미갱신 (timeout=5.0s)"
values:
  - key: "last_msg_sec_ago" value: "6.30"
```

### 출력 예시 (STALE — 토픽 미수신)

```
name:    "/planning/local_costmap"
level:   3                          ← STALE
message: "토픽 수신 없음: /planning/local_costmap/costmap"
values:
  - key: "topic" value: "/planning/local_costmap/costmap"
```

### GUI로 확인 (rqt)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 8. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | costmap 정상 갱신 중 |
| ERROR | 2 | 빨강 | costmap 갱신 중단 (`stale_timeout` 초과) |
| STALE | 3 | 회색 | 토픽 자체가 수신된 적 없음 (Nav2 미기동) |

> WARN 레벨은 사용하지 않습니다. costmap 은 "갱신 중 or 중단" 두 상태만 존재합니다.

---

## 9. 자주 묻는 질문 (FAQ)

**Q. Nav2 기동 직후 STALE 이 납니다.**

costmap 노드가 ACTIVE 상태로 전환되기 전에는 토픽이 발행되지 않습니다.
Nav2 lifecycle 상태는 [`robot_planning_lifecycle_checker`](../robot_planning_lifecycle_checker/README.md)로 확인하세요.

---

**Q. global costmap 은 자주 갱신되지 않는데 stale_timeout 을 어떻게 잡나요?**

Nav2 `global_costmap` 의 `update_frequency` 파라미터를 확인하세요.

```bash
ros2 param get /planning/global_costmap update_frequency
```

해당 주기의 5~10배를 `stale_timeout` 으로 설정하면 불필요한 알람 없이 실제 중단만 감지합니다.

---

**Q. local costmap 이 sensor 입력 없으면 갱신되지 않나요?**

네. LiDAR/카메라 등 센서 토픽이 끊기면 local costmap 도 갱신을 멈춥니다.
costmap ERROR 발생 시 센서 체커(`robot_camera_checker`, `robot_lidar_checker` 등)도 함께 확인하세요.

---

**Q. costmap QoS 설정이 맞지 않아서 토픽을 수신 못합니다.**

이 checker 는 `transient_local` + `reliable` QoS 로 구독합니다.
Nav2 costmap 의 기본 발행 QoS 도 동일합니다.
만약 QoS 불일치가 의심되면 아래로 확인하세요.

```bash
ros2 topic info /planning/global_costmap/costmap --verbose
```

---

## 10. 고려했으나 제외된 항목

초기 설계 시 아래 항목들도 검토했으나, 불필요하거나 오탐 가능성이 높아 제외했습니다.

| 항목 | 제외 이유 |
|------|-----------|
| **costmap size / resolution sanity 체크** | Nav2 파라미터(`width`, `height`, `resolution`)로 고정되며 런타임에 변하지 않음. 기동 후 값이 달라질 이유가 없으므로 진단할 실익 없음 |
| **cost_markers staleness** (`/planning/*/costmap/costmap_raw` 마커 토픽) | 시각화(RViz) 전용 토픽. 실제 planner/controller 동작과 무관하며, rviz 가 없는 운용 환경에서는 아예 발행되지 않음 |
| **최대 비용값(max cost) 임계값 체크** | 장애물이 많은 환경에서 costmap 의 최대 비용이 높은 것은 완전히 정상임. 환경에 따라 임계값이 크게 달라져 일반화 불가 |

---

## 11. 운용 가이드

### Planning Checker 의존관계

```
[robot_planning_lifecycle_checker]   ← 반드시 먼저 확인
         │
         ├──► [robot_planning_costmap_checker]  ← 이 패키지
         │         costmap 이 stale → planner / controller 오작동
         │
         └──► [robot_planning_global_path_checker]
              [robot_planning_local_path_checker]
              [robot_planning_nav_status_checker]
```

costmap ERROR 는 경로 계획 실패(nav_status ABORTED)의 원인이 될 수 있습니다.
nav_status ABORTED 빈도가 높다면 costmap checker 도 함께 확인하세요.

### diagnostic_aggregator 연동

`/diagnostics` 를 raw 로 소비하지 않고 `diagnostic_aggregator` 로 그룹핑하는 방법입니다.

```yaml
# config/diagnostic_aggregator.yaml
analyzers:
  planning:
    type: diagnostic_aggregator/GenericAnalyzer
    path: Planning
    contains:
      - '/planning/'
```

```bash
# aggregator 실행
ros2 run diagnostic_aggregator aggregator_node --ros-args \
  --params-file config/diagnostic_aggregator.yaml

# 그룹핑된 결과 확인 (/diagnostics_agg 토픽)
ros2 topic echo /diagnostics_agg | grep -A 5 "Planning"
```

### ERROR 발생 시 체크 순서

```
costmap ERROR 발생
  1. lifecycle_checker 확인 → global/local costmap 노드 ACTIVE?
  2. 센서 체커 확인 → LiDAR/카메라 토픽 정상?
     (costmap 은 센서 입력이 없으면 갱신 멈춤)
  3. ros2 topic hz /planning/global_costmap/costmap → 발행 주기 확인
  4. Nav2 costmap 노드 로그 확인
```
