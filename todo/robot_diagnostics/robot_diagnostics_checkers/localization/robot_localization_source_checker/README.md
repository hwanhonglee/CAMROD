# robot_localization_source_checker

ROS 2 패키지로, `localization_pose_selector_node`가 발행하는 소스 토픽을 구독해
**현재 로컬라이제이션이 정상 경로(ESKF)로 동작 중인지, 폴백(Kimera-VIO)으로 전환됐는지**를
`/diagnostics` 토픽으로 발행합니다.

> **폴백 활성화는 ESKF 이상 신호**: Kimera-VIO로 전환됐다면 GNSS/IMU 품질 저하 또는 ESKF 내부 오류가 발생한 것입니다.

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

```
/localization/pose_source  ──► localization_source_checker_node ──► /diagnostics
  (std_msgs/String)                          ▲
  "primary_eskf" or "kimera_vio"             │
                                       1초마다 체크
```

`localization_pose_selector_node`는 아래 두 값 중 하나를 발행합니다.

| 토픽 값 | 의미 | 진단 레벨 |
|---------|------|-----------|
| `"primary_eskf"` | ESKF 정상 동작 | OK |
| `"kimera_vio"` | Kimera-VIO 폴백 활성화 | WARN |
| 그 외 | 알 수 없는 소스 | ERROR |

### 폴백 지속 시간 체크

단순히 폴백 여부만 보는 것이 아니라, **얼마나 오래 폴백 상태인지**도 감시합니다.

```
폴백 전환 시작
 │
 ├── 0 ~ fallback_warn_sec(10s)  ── WARN "폴백(Kimera-VIO) 활성화"
 ├── fallback_warn_sec 초과       ── WARN "폴백 활성 (15s / warn=10s)"
 └── fallback_error_sec(60s) 초과 ── ERROR "폴백 장기 지속 (65s > 60s)"
```

### 소스 전환 횟수 체크 (불안정 감지)

최근 60초 내 전환 횟수가 임계값을 초과하면 시스템이 불안정하다고 판단합니다.

```
switch_warn(3회)  초과 → WARN  "소스 불안정 전환 (60s내 5회 > 3)"
switch_error(10회) 초과 → ERROR "소스 과도 전환 (60s내 12회 > 10)"
```

---

## 2. 패키지 구성

```
robot_localization_source_checker/
├── src/
│   ├── localization_source_checker_node.cpp       # 메인 노드 코드
│   └── localization_source_dummy_publisher.cpp    # 테스트용 더미 퍼블리셔
├── config/
│   └── localization_source_checker.yaml           # 설정 파일
├── launch/
│   ├── localization_source_checker.launch.py      # 운영용 실행 파일
│   └── localization_source_test.launch.py         # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `std_msgs` | `String` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_localization_source_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용
ros2 launch robot_localization_source_checker localization_source_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_localization_source_checker localization_source_test.launch.py
ros2 launch robot_localization_source_checker localization_source_test.launch.py scenario:=fallback
ros2 launch robot_localization_source_checker localization_source_test.launch.py scenario:=flapping
ros2 launch robot_localization_source_checker localization_source_test.launch.py scenario:=unknown
ros2 launch robot_localization_source_checker localization_source_test.launch.py scenario:=stale
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | `"primary_eskf"` 지속 발행 | OK |
| `fallback` | `"kimera_vio"` 지속 발행 | WARN → WARN(10s후) → ERROR(60s후) |
| `flapping` | primary/fallback 2초마다 교번 | WARN (switch_warn 초과 시) |
| `unknown` | `"unexpected_localization_source"` 발행 | ERROR |
| `stale` | 퍼블리시 중단 | STALE |

> **tip**: `fallback` 시나리오에서 ERROR를 빠르게 확인하려면 yaml에서 `fallback_error_sec: 5.0`으로 낮추세요.
> `flapping` 시나리오에서 WARN을 빠르게 확인하려면 `switch_warn: 1`로 낮추세요.

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `source_topic` | `/localization/pose_source` | 소스 토픽 (`std_msgs/String`, transient_local) |
| `primary_source` | `"primary_eskf"` | 정상 소스 문자열 |
| `fallback_source` | `"kimera_vio"` | 폴백 소스 문자열 |
| `stale_timeout` | `3.0` | 이 시간(초) 이상 메시지 없으면 STALE |
| `fallback_warn_sec` | `10.0` | 폴백 지속 > 이 값(초) → WARN |
| `fallback_error_sec` | `60.0` | 폴백 지속 > 이 값(초) → ERROR |
| `switch_warn` | `3` | 60s 내 전환 횟수 > 이 값 → WARN |
| `switch_error` | `10` | 60s 내 전환 횟수 > 이 값 → ERROR |

---

## 7. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 20 "localization/source"
```

### 출력 예시 (정상 — ESKF 사용 중)

```
name:    "/localization/source"
level:   0                          ← OK
message: "Primary ESKF 사용 중"
values:
  - key: "current_source"           value: "primary_eskf"
  - key: "primary_source"           value: "primary_eskf"
  - key: "fallback_source"          value: "kimera_vio"
  - key: "switch_count_60s"         value: "0"
  - key: "switch_warn/error"        value: "3 / 10"
  - key: "fallback_duration_sec"    value: "0.0"
  - key: "fallback_warn/error_sec"  value: "10 / 60"
  - key: "last_msg_sec_ago"         value: "0.50"
```

### 출력 예시 (WARN — 폴백 활성)

```
name:    "/localization/source"
level:   1                          ← WARN
message: "폴백(Kimera-VIO) 활성화 — ESKF 이상"
values:
  - key: "current_source"           value: "kimera_vio"
  - key: "fallback_duration_sec"    value: "5.2"
```

### 출력 예시 (ERROR — 폴백 장기 지속)

```
name:    "/localization/source"
level:   2                          ← ERROR
message: "폴백 장기 지속 (65s > 60s)"
values:
  - key: "current_source"           value: "kimera_vio"
  - key: "fallback_duration_sec"    value: "65.0"
```

### 출력 예시 (STALE)

```
name:    "/localization/source"
level:   3                          ← STALE
message: "4.2s 동안 메시지 없음 (timeout=3.0s)"
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
| WARN | 1 | 노랑 | 주의 필요 (폴백 활성 또는 소스 불안정) |
| ERROR | 2 | 빨강 | 오류 발생 (알 수 없는 소스 또는 폴백 장기 지속) |
| STALE | 3 | 회색 | 데이터 없음 (pose_selector가 발행하지 않는 상태) |

---

## 9. 자주 묻는 질문 (FAQ)

**Q. "primary_eskf", "kimera_vio" 이외의 소스 이름을 쓰려면?**

yaml에서 `primary_source`와 `fallback_source`를 변경하세요.
예: `primary_source: "eskf_output"`, `fallback_source: "vio_fallback"`

---

**Q. flapping 시나리오에서 언제 WARN이 발생하나요?**

더미 퍼블리셔가 2초마다 교번하므로 60초 내 약 30회 전환됩니다.
기본 `switch_warn=3`이면 약 6초 후 WARN이 됩니다.

---

**Q. pose_selector가 꺼져 있으면 STALE이 되나요?**

네. `stale_timeout`(기본 3초) 안에 메시지가 오지 않으면 STALE로 전환됩니다.
단, transient_local QoS이므로 연결 시 마지막 값이 즉시 전달됩니다.
