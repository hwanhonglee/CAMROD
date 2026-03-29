# robot_planning_lifecycle_checker

ROS 2 패키지로, Nav2 lifecycle 노드들의 `/get_state` 서비스를 비동기 폴링하여
**각 노드가 ACTIVE 상태인지**를 `/diagnostics` 토픽으로 발행합니다.

planner_server, controller_server, bt_navigator, behavior_server 중 하나라도
ACTIVE 가 아니면 navigation 전체가 동작하지 않습니다.

> **다른 planning checker 의 전제 조건**: 이 checker 가 ERROR 이면
> [`robot_planning_global_path_checker`](../robot_planning_global_path_checker/README.md),
> [`robot_planning_local_path_checker`](../robot_planning_local_path_checker/README.md),
> [`robot_planning_nav_status_checker`](../robot_planning_nav_status_checker/README.md) 의
> 진단 결과는 의미가 없습니다.

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
/{node}/get_state (service) ──► planning_lifecycle_checker_node ──► /diagnostics
                                             ▲
                                             │
                               poll_rate(2Hz)마다 서비스 호출
                               1초마다 진단 발행
```

`diagnostic_updater` 타이머와 별도로 **`poll_rate` 타이머**가 동작합니다.
폴링은 비동기(`async_send_request`)로 수행되어 spin thread 를 블로킹하지 않습니다.
진단 콜백은 캐시된 최신 상태만 읽습니다.

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 서비스 존재 | `/get_state` 서비스 응답 | ERROR (노드 미기동) |
| 폴링 응답 | `stale_timeout` 이내 응답 | ERROR (응답 없음) |
| lifecycle state | ACTIVE (id=3) | INACTIVE → WARN / 그 외 → ERROR |

### 서비스 통신 방식 및 Executor 모델

lifecycle `/get_state` 는 ROS 2 서비스 기본 QoS(`reliable`)로 호출합니다.
폴링은 `async_send_request` + 별도 poll 타이머로 구현되어 있어
**`SingleThreadedExecutor` 로도 executor deadlock 없이 동작합니다.**
`MultiThreadedExecutor` 는 불필요하며, 혹시 사용 중이라면 콜백 스레드 안전성에 주의하세요.

```
[spin loop]
  ├── poll_timer 콜백 → async_send_request (non-blocking, 즉시 반환)
  │                        └── 응답 도착 시 람다 콜백으로 캐시 업데이트 (mutex 보호)
  └── diagnostic_timer 콜백 → 캐시 읽기 (mutex 보호)
```

### Lifecycle State ID 표

| id | 상태 | 진단 레벨 |
|----|------|-----------|
| 1 | UNCONFIGURED | ERROR |
| 2 | INACTIVE | WARN |
| 3 | ACTIVE | OK |
| 4 | FINALIZED | ERROR |

---

## 2. 패키지 구성

```
robot_planning_lifecycle_checker/
├── src/
│   ├── planning_lifecycle_checker_node.cpp       # 메인 노드 코드
│   └── planning_lifecycle_dummy_server.cpp       # 테스트용 더미 서비스 서버
├── config/
│   └── planning_lifecycle_checker.yaml           # 설정 파일
├── launch/
│   ├── planning_lifecycle_checker.launch.py      # 운영용 실행 파일
│   └── planning_lifecycle_test.launch.py         # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `lifecycle_msgs` | `GetState` 서비스 / `State` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_planning_lifecycle_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (Nav2 실행 후)
ros2 launch robot_planning_lifecycle_checker planning_lifecycle_checker.launch.py

# 테스트용 (checker + 더미 서비스 서버)
ros2 launch robot_planning_lifecycle_checker planning_lifecycle_test.launch.py
ros2 launch robot_planning_lifecycle_checker planning_lifecycle_test.launch.py scenario:=inactive
ros2 launch robot_planning_lifecycle_checker planning_lifecycle_test.launch.py scenario:=mixed

# 수동으로 특정 노드 서비스 직접 확인
ros2 service call /planning/planner_server/get_state lifecycle_msgs/srv/GetState
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 4개 노드 모두 ACTIVE | 모두 OK |
| `inactive` | 4개 노드 모두 INACTIVE | 모두 WARN |
| `unconfigured` | 4개 노드 모두 UNCONFIGURED | 모두 ERROR |
| `finalized` | 4개 노드 모두 FINALIZED | 모두 ERROR |
| `mixed` | planner:ACTIVE, controller:INACTIVE, bt_navigator:UNCONFIGURED, behavior:ACTIVE | OK / WARN / ERROR / OK 혼합 |
| (더미 없이 checker만) | `/get_state` 서비스 없음 | 모두 ERROR "서비스 없음" |

> **더미 서버 구조**: 토픽 퍼블리셔가 아니라 `/get_state` **서비스 서버**입니다.
> lifecycle checker 는 토픽이 아니라 서비스를 폴링하기 때문입니다.

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `poll_rate` | `2.0` | lifecycle `/get_state` 서비스 폴링 주기 (Hz) |
| `stale_timeout` | `5.0` | 마지막 폴링 응답 후 이 시간(초) 초과 시 ERROR |
| `node_names` | (아래 기본값 참조) | 모니터링할 lifecycle 노드 이름 목록 |

### 기본 node_names

```yaml
node_names:
  - "/planning/planner_server"
  - "/planning/controller_server"
  - "/planning/bt_navigator"
  - "/planning/behavior_server"
```

### 노드 추가 / 제거

`config/planning_lifecycle_checker.yaml`에서 직접 편집합니다.

```yaml
planning_lifecycle_checker:
  ros__parameters:
    node_names:
      - "/planning/planner_server"
      - "/planning/controller_server"
      - "/planning/bt_navigator"
      - "/planning/behavior_server"
      - "/planning/smoother_server"   # 추가 예시
```

### 런타임 파라미터 변경

파라미터는 **노드 기동 시 1회 로드**되며, `ros2 param set` 으로 변경해도 즉시 반영되지 않습니다.
특히 `node_names` 목록 변경은 반드시 노드 재시작이 필요합니다.

```bash
# 현재 모니터링 중인 노드 목록 확인
ros2 param get /planning_lifecycle_checker node_names

# 변경 후 재시작 필요
ros2 param set /planning_lifecycle_checker poll_rate 1.0
# → 이 명령은 값은 바뀌지만 실제 poll 주기는 재시작 전까지 변하지 않음
```

---

## 7. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 10 "planning/lifecycle"
```

### 출력 예시 (정상 — ACTIVE)

```
name:    "/planning/lifecycle//planning/planner_server"
level:   0                          ← OK
message: "/planning/planner_server ACTIVE"
values:
  - key: "node"          value: "/planning/planner_server"
  - key: "state"         value: "active"
  - key: "state_id"      value: "3"
  - key: "polled_sec_ago" value: "0.50"
```

### 출력 예시 (WARN — INACTIVE)

```
name:    "/planning/lifecycle//planning/controller_server"
level:   1                          ← WARN
message: "/planning/controller_server INACTIVE — 전환 중이거나 정지됨"
values:
  - key: "state"     value: "inactive"
  - key: "state_id"  value: "2"
```

### 출력 예시 (ERROR — 노드 미기동)

```
name:    "/planning/lifecycle//planning/bt_navigator"
level:   2                          ← ERROR
message: "/planning/bt_navigator — get_state 서비스 없음 (노드 미기동)"
```

### GUI로 확인 (rqt)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 8. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | ACTIVE — 정상 동작 중 |
| WARN | 1 | 노랑 | INACTIVE — 일시 정지 또는 전환 중 |
| ERROR | 2 | 빨강 | UNCONFIGURED / FINALIZED / 서비스 없음 |
| STALE | 3 | 회색 | 폴링 응답 대기 중 (초기 기동 직후) |

---

## 9. 자주 묻는 질문 (FAQ)

**Q. 노드가 켜져 있는데 ERROR 가 납니다.**

`/get_state` 서비스가 namespace 에 맞게 설정됐는지 확인하세요.

```bash
ros2 service list | grep get_state
```

예상 결과: `/planning/planner_server/get_state`

---

**Q. Nav2 기동 직후 잠깐 WARN 이 됩니다.**

lifecycle 노드가 configure → activate 전환 중에는 INACTIVE 상태입니다.
기동 완료 후 모든 노드가 ACTIVE 가 되면 OK 로 전환됩니다. 정상 동작입니다.

---

**Q. behavior_server 가 없는 구성입니다.**

`config/planning_lifecycle_checker.yaml`에서 해당 노드를 제거하세요.

```yaml
node_names:
  - "/planning/planner_server"
  - "/planning/controller_server"
  - "/planning/bt_navigator"
  # behavior_server 제거
```

---

**Q. `poll_rate`와 `publish_rate`의 차이가 뭔가요?**

- `poll_rate`: Nav2 노드에 서비스를 호출하는 주기. 너무 높으면 불필요한 서비스 트래픽.
- `publish_rate`: `/diagnostics` 토픽 발행 주기. 캐시된 상태를 그대로 읽어 발행.

일반적으로 `poll_rate=2.0`, `publish_rate=1.0` 이면 충분합니다.

---

## 10. 고려했으나 제외된 항목

초기 설계 시 아래 항목들도 검토했으나, 불필요하거나 오탐 가능성이 높아 제외했습니다.

| 항목 | 제외 이유 |
|------|-----------|
| **중간 전환 상태 세분화** (CONFIGURING, ACTIVATING 등을 별도 레벨로 처리) | Nav2 기동 시 필연적으로 거치는 상태이므로 진단 레벨을 높이면 오탐 다수 발생. INACTIVE(WARN) 하나로 묶는 것으로 충분 |
| **lifecycle transition event 구독** (`/lifecycle_node/transition_event`) | 전환이 완료됐는지 여부가 중요하지, 전환 이벤트 자체를 추적할 필요 없음. `/get_state` 폴링으로 충분 |
| **`spin_until_future_complete` 동기 폴링** | executor deadlock 을 유발함. `async_send_request` + 별도 poll 타이머로 대체 |

---

## 11. 운용 가이드

### Planning Checker 의존관계

이 패키지는 나머지 모든 planning checker 의 **전제 조건**입니다.

```
[robot_planning_lifecycle_checker]  ← 여기가 ERROR 이면
         │
         ▼
[robot_planning_nav_status_checker]  ┐
[robot_planning_global_path_checker] ├ 이 결과들은 신뢰 불가
[robot_planning_local_path_checker]  │
[robot_planning_costmap_checker]     ┘
```

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

aggregation 후 rqt_runtime_monitor 에서 `Planning` 그룹으로 묶여 표시됩니다.

### 노드 이름 확인

실제 배포 시 node name 이 다를 수 있습니다. 현재 실행 중인 노드 이름 확인:

```bash
ros2 node list | grep lifecycle
```
