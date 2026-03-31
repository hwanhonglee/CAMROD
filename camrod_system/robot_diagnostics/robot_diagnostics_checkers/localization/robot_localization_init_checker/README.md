# robot_localization_init_checker

ROS 2 패키지로, `drop_zone_matcher_node`가 발행하는 초기화 상태 토픽들을 구독해
**로봇이 올바른 drop zone에서 시작했는지**를 `/diagnostics` 토픽으로 발행합니다.

> **로컬라이제이션 초기화 관제**: 시동 시 기준점이 올바른가?
>
> | 체커 | 역할 |
> |------|------|
> | `robot_localization_init_checker` | Drop zone **초기화** 상태 (기준점이 맞는가?) |
> | `robot_localization_pose_checker` | 로컬라이제이션 **출력** 품질 (Planning에 공급 가능한가?) |
> | `robot_localization_mode_checker` | 로컬라이제이션 **모드·신뢰도** (supervisor 자가진단) |

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

---

## 1. 어떻게 동작하나요?

이 노드는 `drop_zone_matcher_node`가 발행하는 3개 토픽을 구독하고, 주기적으로 아래 항목을 확인합니다.

```
/localization/initial_match_ok       ─┐
/localization/initial_match_distance  ├─► localization_init_checker_node ──► /diagnostics
/localization/initial_match_id       ─┘               ▲
                                                       │
                                                 1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| Match OK (grace 이내) | match_ok=true | WARN (아직 초기화 중) |
| Match OK (grace 초과) | match_ok=true | ERROR (drop zone 매칭 실패) |
| 거리 | `dist_warn_m` 미만 | WARN / ERROR |

### Grace Period 동작

시동 후 `grace_period_sec`(기본 30초) 동안은 drop zone 매칭 실패를 WARN으로 처리합니다.
이 시간이 지나도 매칭이 안 되면 ERROR로 격상됩니다.

```
시동
 │
 ├──── 0 ~ 30s ────────── match_ok=false → WARN  "초기화 중"
 │
 └──── 30s 이후 ────────── match_ok=false → ERROR "Drop zone 매칭 실패"
```

### 거리 기준

| 거리 | 의미 | 진단 레벨 |
|------|------|-----------|
| < `dist_warn_m` (3.0m) | zone 내부 | OK |
| `dist_warn_m` ~ `dist_error_m` | zone 경계 부근 | WARN |
| > `dist_error_m` (8.0m) | zone 밖 | ERROR |

---

## 2. 패키지 구성

```
robot_localization_init_checker/
├── src/
│   ├── localization_init_checker_node.cpp       # 메인 노드 코드
│   └── localization_init_dummy_publisher.cpp    # 테스트용 더미 퍼블리셔
├── config/
│   └── localization_init_checker.yaml           # 설정 파일
├── launch/
│   ├── localization_init_checker.launch.py      # 운영용 실행 파일
│   └── localization_init_test.launch.py         # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `std_msgs` | `Bool`, `Float32`, `String` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_localization_init_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_localization_init_checker localization_init_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_localization_init_checker localization_init_test.launch.py
ros2 launch robot_localization_init_checker localization_init_test.launch.py scenario:=not_matched
ros2 launch robot_localization_init_checker localization_init_test.launch.py scenario:=far
ros2 launch robot_localization_init_checker localization_init_test.launch.py scenario:=stale
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | match_ok=true, distance=0.5m | OK |
| `not_matched` | match_ok=false, distance=2.0m | WARN (grace 이내) → ERROR (grace 초과) |
| `far` | match_ok=false, distance=dist_warn_m×1.5 | WARN 또는 ERROR |
| `stale` | 퍼블리시 중단 | STALE |

> **tip**: `not_matched` 시나리오에서 grace_period_sec(기본 30초)가 지나면 WARN → ERROR로 전환됩니다.
> 빠르게 ERROR를 확인하려면 yaml에서 `grace_period_sec: 5.0`으로 낮추세요.

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `ok_topic` | `/localization/initial_match_ok` | match OK 토픽 (`std_msgs/Bool`) |
| `distance_topic` | `/localization/initial_match_distance` | 거리 토픽 (`std_msgs/Float32`) |
| `id_topic` | `/localization/initial_match_id` | zone ID 토픽 (`std_msgs/String`) |
| `stale_timeout` | `5.0` | 이 시간(초) 이상 메시지 없으면 STALE |
| `grace_period_sec` | `30.0` | 시동 후 미매칭 허용 시간 (초) |
| `dist_warn_m` | `3.0` | 거리 > 이 값 → WARN |
| `dist_error_m` | `8.0` | 거리 > 이 값 → ERROR |

---

## 7. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 15 "localization/init"
```

### 출력 예시 (정상 — 매칭 완료)

```
name:    "/localization/init"
level:   0                          ← OK
message: "Drop zone 매칭 완료 (id=zone_A)"
values:
  - key: "match_ok"               value: "true"
  - key: "match_id"               value: "zone_A"
  - key: "match_distance_m"       value: "0.500"
  - key: "dist_warn_m"            value: "3.0"
  - key: "dist_error_m"           value: "8.0"
  - key: "since_node_start_sec"   value: "12.3"
  - key: "grace_period_sec"       value: "30.0"
  - key: "last_msg_sec_ago"       value: "0.50"
```

### 출력 예시 (WARN — grace period 이내 미매칭)

```
name:    "/localization/init"
level:   1                          ← WARN
message: "초기화 중 (12s / grace=30s)"
values:
  - key: "match_ok"               value: "false"
  - key: "since_node_start_sec"   value: "12.0"
  - key: "grace_period_sec"       value: "30.0"
```

### 출력 예시 (ERROR — grace period 초과)

```
name:    "/localization/init"
level:   2                          ← ERROR
message: "Drop zone 매칭 실패 (grace period 초과)"
values:
  - key: "match_ok"               value: "false"
  - key: "since_node_start_sec"   value: "35.2"
  - key: "grace_period_sec"       value: "30.0"
```

### 출력 예시 (STALE)

```
name:    "/localization/init"
level:   3                          ← STALE
message: "6.1s 동안 메시지 없음 (timeout=5.0s)"
values:
  - key: "last_msg_sec_ago"       value: "6.1"
```

### GUI로 확인 (rqt)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 8. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | 정상 동작 중 |
| WARN | 1 | 노랑 | 주의 필요 (동작하지만 품질 저하) |
| ERROR | 2 | 빨강 | 오류 발생 (즉각 대응 필요) |
| STALE | 3 | 회색 | 데이터 없음 (토픽이 오지 않거나 너무 오래됨) |

---

## 9. 자주 묻는 질문 (FAQ)

**Q. grace_period_sec을 어떻게 설정해야 하나요?**

drop_zone_matcher_node가 시동 후 초기화를 완료하는 데 걸리는 시간보다 넉넉하게 설정하세요.
일반적으로 20~60초 사이를 권장합니다.

---

**Q. stale_timeout을 5초로 설정했는데 시동 직후 STALE이 됩니다.**

drop_zone_matcher_node가 늦게 시작하는 경우입니다.
`stale_timeout`을 늘리거나, launch 파일에서 checker보다 matcher를 먼저 시작하도록 순서를 조정하세요.

---

**Q. dist_error_m은 어떻게 결정하나요?**

drop zone의 반경(match_radius)보다 크게 설정하세요.
예: match_radius=5.0m이면 dist_warn_m=3.0, dist_error_m=8.0 정도가 적절합니다.
