# robot_planning_path_checker

ROS 2 패키지로, navigation 실행 중 경로 토픽들(global/local)의 품질을
`/diagnostics` 토픽으로 발행합니다.

`path_names` 리스트로 모니터링할 경로 소스를 동적으로 추가할 수 있습니다.

> **전제 조건**: [`robot_planning_lifecycle_checker`](../robot_planning_lifecycle_checker/README.md) 가
> ERROR 를 발행 중이라면 이 패키지의 결과는 신뢰할 수 없습니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [경로 소스 추가하기](#6-경로-소스-추가하기)
7. [파라미터 설명](#7-파라미터-설명)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

```
/planning/global_path ──┐
  (nav_msgs/Path)        ├──► planning_path_checker_node ──► /diagnostics
/planning/local_path  ──┘              ▲
  (nav_msgs/Path)                1초마다 체크
                         (nav_active 상태 참조)
/planning/navigate_to_pose/_action/status ──► nav_active 판별
```

**navigation idle 상태에서는 체크하지 않습니다** — 경로가 없어도 OK 반환.

| 체크 항목 | navigation idle | navigation 중 |
|-----------|----------------|--------------|
| 토픽 미수신 | OK (idle) | ERROR |
| Staleness | OK (idle) | ERROR |
| Point count 부족 | OK (idle) | WARN / ERROR |

---

## 2. 패키지 구성

```
robot_planning_path_checker/
├── src/
│   ├── planning_path_checker_node.cpp    # 메인 노드 코드
│   └── planning_path_dummy_publisher.cpp # 테스트용 더미 퍼블리셔
├── config/
│   └── planning_path_checker.yaml        # 설정 파일
├── launch/
│   ├── planning_path_checker.launch.py   # 운영용 실행 파일
│   └── planning_path_test.launch.py      # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `nav_msgs` | `Path` 메시지 타입 |
| `action_msgs` | `GoalStatusArray` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_planning_path_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용
ros2 launch robot_planning_path_checker planning_path_checker.launch.py

# 테스트용
ros2 launch robot_planning_path_checker planning_path_test.launch.py
ros2 launch robot_planning_path_checker planning_path_test.launch.py scenario:=stale_global
ros2 launch robot_planning_path_checker planning_path_test.launch.py scenario:=stale_both
ros2 launch robot_planning_path_checker planning_path_test.launch.py scenario:=few_warn
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 두 경로 정상 + EXECUTING | 둘 다 OK |
| `idle` | 두 경로 정상 + nav inactive | 둘 다 idle OK |
| `stale_global` | global_path 발행 중단 | global_path ERROR |
| `stale_local` | local_path 발행 중단 | local_path ERROR |
| `stale_both` | 두 경로 발행 중단 | 둘 다 ERROR |
| `few_warn` | 두 경로 포인트 warn 기준 미만 | 둘 다 WARN |
| `few_error` | 두 경로 포인트 error 기준 미만 | 둘 다 ERROR |

---

## 6. 경로 소스 추가하기

`config/planning_path_checker.yaml` 의 `path_names` 에 이름을 추가하고
같은 이름의 섹션을 추가합니다.

```yaml
path_names:
  - "global_path"
  - "local_path"
  - "modified_path"    # 신규 추가 예시

modified_path:
  topic: "/planning/modified_path"
  stale_timeout: 2.0
  min_points_warn: 5
  min_points_error: 2
```

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `nav_status_topic` | `/planning/navigate_to_pose/_action/status` | navigation 상태 토픽 |
| `path_names` | `[]` | 모니터링할 경로 소스 이름 목록 |

### 소스별 파라미터 (`<소스이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | — | 구독할 `nav_msgs/Path` 토픽 |
| `stale_timeout` | `3.0` | navigation 중 이 시간(초) 이상 미수신이면 ERROR |
| `min_points_warn` | `5` | `poses.size()` 이 값 미만이면 WARN |
| `min_points_error` | `2` | `poses.size()` 이 값 미만이면 ERROR |

### min_points 판정 예시

`min_points_warn: 8`, `min_points_error: 3` 일 때:

```
point_count >= 8   → OK
3 <= count < 8     → WARN "경로 포인트 적음 (5 < 8)"
count < 3          → ERROR "경로 포인트 부족 (1 < 3)"
```

---

## 8. 진단 결과 확인하기

```bash
ros2 topic echo /diagnostics | grep -A 8 "planning/path"
```

### 출력 예시 (정상)

```
name:    "/planning/path/global_path"
level:   0                          ← OK
message: "OK (20 points, 0.2s ago)"
values:
  - key: "point_count"       value: "20"
  - key: "min_points_warn"   value: "5"
  - key: "min_points_error"  value: "2"
  - key: "last_path_sec_ago" value: "0.20"
  - key: "nav_active"        value: "true"

name:    "/planning/path/local_path"
level:   0                          ← OK
message: "OK (30 points, 0.1s ago)"
```

### 출력 예시 (idle)

```
name:    "/planning/path/global_path"
level:   0                          ← OK
message: "navigation idle"
values:
  - key: "nav_active"  value: "false"
  - key: "point_count" value: "20"
```

---

## 9. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | 정상 또는 navigation idle |
| WARN | 1 | 노랑 | 경로 포인트 수 warn 기준 미만 |
| ERROR | 2 | 빨강 | 경로 미수신, stale, 또는 포인트 수 error 기준 미만 |

> STALE 레벨은 사용하지 않습니다.
> navigation idle 시 토픽이 없어도 OK 처리하므로 STALE 발생 조건이 없습니다.

---

## 10. 자주 묻는 질문 (FAQ)

**Q. navigation을 시작했는데 바로 ERROR가 납니다.**

경로가 아직 계획되지 않았을 수 있습니다. `stale_timeout` 이후에도 ERROR면 Planner를 확인하세요.

```bash
ros2 topic hz /planning/global_path
ros2 node list | grep planner
```

**Q. global_path 포인트 수가 줄어들다가 WARN이 납니다.**

목적지에 가까워지면 남은 경로 포인트 수가 줄어드는 정상 현상입니다.
`min_points_warn` 을 낮추거나 목적지 도달 판정과 연동해 비활성화하는 것을 고려하세요.

**Q. local_path 가 global_path 보다 포인트 기준이 높은 이유는?**

`local_path` 는 `local_path_extractor` 가 lookahead 30m 창을 잘라서 만들며,
Controller 가 즉시 추종합니다. 포인트 수가 너무 적으면 제어 정확도가 떨어집니다.
