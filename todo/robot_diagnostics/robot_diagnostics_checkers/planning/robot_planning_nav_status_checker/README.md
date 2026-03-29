# robot_planning_nav_status_checker

ROS 2 패키지로, `navigate_to_pose` action 의 `GoalStatusArray` 토픽을 구독해
**Nav2 navigation 실행 상태와 경로 계획 실패 빈도**를
`/diagnostics` 토픽으로 발행합니다.

navigation이 반복적으로 ABORTED 되고 있는지, action server로부터 status가
더 이상 오지 않는지를 실시간으로 모니터링합니다.

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
/planning/navigate_to_pose/_action/status ──► planning_nav_status_checker_node ──► /diagnostics
        (GoalStatusArray)                                    ▲
                                                             │
                                                       1초마다 체크
```

`GoalStatusArray`의 `status_list`에서 현재 가장 활성화된 상태를 추출하고,
60초 rolling window 안에 ABORTED 횟수가 임계값을 초과했는지 감시합니다.

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 현재 status | EXECUTING / ACCEPTED | WARN (단회 ABORTED) / OK (idle) |
| Abort 빈도 | `abort_warn` 이하 | WARN / ERROR (빨강) |

### Status 판정 우선순위

같은 `status_list` 안에 여러 goal 이 있을 경우 아래 순서로 dominant status 를 추출합니다.

```
EXECUTING > ACCEPTED > (ABORTED, CANCELED, SUCCEEDED 등)
```

### Abort 빈도 체크

```
60s rolling window 내 abort 횟수
 ├── abort_warn(2회) 초과  → WARN  "ABORTED 빈도 높음 (60s내 3회 > 2)"
 └── abort_error(5회) 초과 → ERROR "반복 ABORTED (60s내 6회 > 5)"
```

### 구독 토픽 QoS

| 토픽 | 타입 | Reliability | Durability |
|------|------|-------------|------------|
| `nav_status_topic` | `action_msgs/GoalStatusArray` | `reliable` | `volatile` |

Nav2 action server 의 기본 QoS 와 일치합니다. QoS 불일치 의심 시:

```bash
ros2 topic info /planning/navigate_to_pose/_action/status --verbose
```

---

## 2. 패키지 구성

```
robot_planning_nav_status_checker/
├── src/
│   ├── planning_nav_status_checker_node.cpp      # 메인 노드 코드
│   └── planning_nav_status_dummy_publisher.cpp   # 테스트용 더미 퍼블리셔
├── config/
│   └── planning_nav_status_checker.yaml          # 설정 파일
├── launch/
│   ├── planning_nav_status_checker.launch.py     # 운영용 실행 파일
│   └── planning_nav_status_test.launch.py        # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `action_msgs` | `GoalStatusArray` 메시지 타입 |
| `unique_identifier_msgs` | 더미 퍼블리셔 UUID 필드 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_planning_nav_status_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_planning_nav_status_checker planning_nav_status_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_planning_nav_status_checker planning_nav_status_test.launch.py
ros2 launch robot_planning_nav_status_checker planning_nav_status_test.launch.py scenario:=aborted_warn
ros2 launch robot_planning_nav_status_checker planning_nav_status_test.launch.py scenario:=aborted_error
ros2 launch robot_planning_nav_status_checker planning_nav_status_test.launch.py scenario:=stale
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | STATUS_EXECUTING 지속 발행 | OK |
| `idle` | STATUS_SUCCEEDED 발행 | OK (idle) |
| `stale` | 퍼블리시 중단 | STALE |
| `aborted_once` | ABORTED 1회 후 idle | WARN |
| `aborted_warn` | 5초마다 ABORTED (abort_warn 초과) | WARN |
| `aborted_error` | 2초마다 ABORTED (abort_error 초과) | ERROR |

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `nav_status_topic` | `/planning/navigate_to_pose/_action/status` | 구독할 GoalStatusArray 토픽 |
| `stale_timeout` | `5.0` | 이 시간(초) 이상 메시지 없으면 STALE |
| `abort_warn` | `2` | 60s 내 ABORTED 횟수 > 이 값 → WARN |
| `abort_error` | `5` | 60s 내 ABORTED 횟수 > 이 값 → ERROR |

### abort 임계값 설정 가이드

`abort_warn` / `abort_error`는 운용 환경의 경로 복잡도에 맞게 조정합니다.

| 환경 | 권장 abort_warn | 권장 abort_error |
|------|----------------|-----------------|
| 단순 창고 (개방 공간) | 2 | 5 |
| 좁은 통로 / 장애물 많음 | 4 | 8 |
| 동적 장애물 (사람 많음) | 5 | 10 |

### 런타임 파라미터 변경

파라미터는 **노드 기동 시 1회 로드**되며, `ros2 param set` 으로 변경해도 즉시 반영되지 않습니다.
임계값 변경 후에는 노드를 재시작해야 합니다.

```bash
# 현재 값 확인
ros2 param get /planning_nav_status_checker abort_warn

# 변경 후 재시작 필요
ros2 param set /planning_nav_status_checker abort_warn 4
ros2 lifecycle set /planning_nav_status_checker shutdown  # 또는 launch 재시작
```

---

## 7. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 15 "planning/nav_status"
```

### 출력 예시 (정상 — navigation 실행 중)

```
name:    "/planning/nav_status"
level:   0                          ← OK
message: "navigation 실행 중: EXECUTING"
values:
  - key: "current_status"    value: "EXECUTING"
  - key: "abort_count_60s"   value: "0"
  - key: "abort_warn/error"  value: "2 / 5"
  - key: "last_msg_sec_ago"  value: "0.10"
```

### 출력 예시 (WARN — abort 빈도 높음)

```
name:    "/planning/nav_status"
level:   1                          ← WARN
message: "ABORTED 빈도 높음 (60s내 3회 > 2)"
values:
  - key: "current_status"    value: "EXECUTING"
  - key: "abort_count_60s"   value: "3"
```

### 출력 예시 (ERROR — 반복 ABORTED)

```
name:    "/planning/nav_status"
level:   2                          ← ERROR
message: "반복 ABORTED (60s내 6회 > 5)"
values:
  - key: "abort_count_60s"   value: "6"
  - key: "abort_warn/error"  value: "2 / 5"
```

### 출력 예시 (STALE)

```
name:    "/planning/nav_status"
level:   3                          ← STALE
message: "6.0s 동안 nav status 없음 (timeout=5.0s)"
```

### GUI로 확인 (rqt)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 8. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | navigation 정상 실행 중 또는 idle |
| WARN | 1 | 노랑 | ABORTED 빈도 높음 (경로 재계획 반복) |
| ERROR | 2 | 빨강 | ABORTED 과다 (즉각 원인 파악 필요) |
| STALE | 3 | 회색 | action status 토픽 미수신 |

---

## 9. 자주 묻는 질문 (FAQ)

**Q. navigation이 잘 되고 있는데 abort count가 올라갑니다.**

동적 장애물 회피를 위해 controller가 경로를 잠시 ABORTED 처리하고 재계획하는 경우가 있습니다.
`abort_warn` / `abort_error` 값을 운용 환경에 맞게 높이세요.

---

**Q. idle 상태 (goal 없음) 일 때 어떻게 표시되나요?**

`status_list`가 비어 있거나 STATUS_SUCCEEDED / STATUS_CANCELED 이면 "idle" 메시지와 함께 OK 로 표시됩니다.

---

**Q. Nav2가 아직 뜨지 않아서 토픽 자체가 없습니다.**

`stale_timeout`(기본 5초) 경과 후 STALE로 전환됩니다.
Nav2 lifecycle 노드가 ACTIVE 상태인지는
[`robot_planning_lifecycle_checker`](../robot_planning_lifecycle_checker/README.md)로 확인하세요.

---

## 10. 고려했으나 제외된 항목

초기 설계 시 아래 항목들도 검토했으나, 불필요하거나 오탐 가능성이 높아 제외했습니다.

| 항목 | 제외 이유 |
|------|-----------|
| **goal_checker 전체 패키지** (`robot_planning_goal_checker`) | goal 전송은 one-shot event 이므로 staleness 진단이 무의미함. goal 이 없는 게 정상인 idle 상태와 구분 불가 |
| **stuck detection** (로봇이 멈췄는지 체크) | Nav2 의 `SimpleProgressChecker` / `StoppedProgressChecker` 플러그인이 이미 동일한 역할을 수행하며, ABORTED 로 리포트함. 중복 진단 |
| **단회 ABORTED를 ERROR로 처리** | 장애물 회피나 경로 재계획 과정에서 일시적 ABORTED는 정상 동작임. rolling window 빈도 기반 판정이 더 적절 |

---

## 11. 운용 가이드

### Planning Checker 의존관계

```
[robot_planning_lifecycle_checker]  ← 반드시 먼저 확인
         │ 모든 Nav2 노드 ACTIVE?
         ▼
[robot_planning_nav_status_checker] ← navigation 실행 상태 / abort 빈도
[robot_planning_global_path_checker]
[robot_planning_local_path_checker]
[robot_planning_costmap_checker]
```

lifecycle checker 가 ERROR 이면 나머지 checker 의 진단 결과는 의미가 없습니다.

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
