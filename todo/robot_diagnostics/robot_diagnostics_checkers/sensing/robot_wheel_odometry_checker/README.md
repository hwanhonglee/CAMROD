# robot_wheel_odometry_checker

ROS 2 패키지로, 휠 오도메트리 `nav_msgs/Odometry` 토픽을 구독해
**휠 오도메트리가 정상적으로 동작하고 있는지**를 `/diagnostics` 토픽으로 발행합니다.

수신 속도(Hz)가 제대로 나오는지, 속도 데이터에 NaN/Inf가 없는지,
속도 크기가 허용 범위 내에 있는지를 실시간으로 모니터링합니다.

> **센서 레벨 관제**: 휠 오도메트리 자체가 살아있는가?
> 휠 오도메트리는 GNSS가 없는 **DR_ONLY(Dead Reckoning)** 모드에서
> 유일한 속도 소스입니다. 끊기면 Dead Reckoning 자체가 불가능해집니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [휠 소스 추가하기](#6-휠-소스-추가하기)
7. [파라미터 설명](#7-파라미터-설명)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

이 노드는 휠 오도메트리 토픽을 **구독(subscribe)** 하고, 주기적으로 아래 항목을 확인합니다.

```
휠 오도메트리 토픽 ──► wheel_odometry_checker_node ──► /diagnostics
(Odometry)                      ▲
                                │
                          1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 수신 속도 (Hz) | `expected_hz`의 70% 이상 | WARN (노랑) / ERROR (빨강) |
| twist.linear NaN/Inf | 모든 성분이 유한값 | ERROR (빨강) |
| 속도 크기 (\|vx\|) | `max_speed_warn_ms` 미만 | WARN (노랑) / ERROR (빨강) |

> **Hz 계산 방식**: 최근 2초 내에 수신된 메시지 수로 계산합니다.

> **속도 체크**: 전진 속도 `twist.linear.x`의 절댓값을 기준으로 합니다.
> 로봇의 최대 허용 속도보다 여유있게 설정하세요.
> `0.0`으로 설정하면 해당 임계값 체크를 비활성화합니다.

---

## 2. 패키지 구성

```
robot_wheel_odometry_checker/
├── src/
│   ├── wheel_odometry_checker_node.cpp      # 메인 노드 코드
│   └── wheel_odometry_dummy_publisher.cpp   # 테스트용 더미 퍼블리셔
├── config/
│   └── wheel_odometry_checker.yaml          # 휠 설정 파일
├── launch/
│   ├── wheel_odometry_checker.launch.py     # 운영용 실행 파일
│   └── wheel_odometry_test.launch.py        # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `nav_msgs` | `Odometry` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_wheel_odometry_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_wheel_odometry_checker wheel_odometry_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_wheel_odometry_checker wheel_odometry_test.launch.py
ros2 launch robot_wheel_odometry_checker wheel_odometry_test.launch.py scenario:=nan_velocity
ros2 launch robot_wheel_odometry_checker wheel_odometry_test.launch.py scenario:=stale
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 정상 20Hz, 유효한 속도 데이터 | OK |
| `hz_warn` | Hz 저하 (warn 구간) | WARN |
| `hz_error` | Hz 심각 저하 (error 구간) | ERROR |
| `stale` | 퍼블리시 중단 | STALE |
| `nan_velocity` | `twist.linear` NaN 주입 | ERROR |
| `high_speed` | `max_speed_warn_ms` × 1.2 속도 주입 | WARN |

---

## 6. 휠 소스 추가하기

`config/wheel_odometry_checker.yaml`에 이름을 추가하고 해당 섹션을 작성합니다.

```yaml
wheel_odometry_checker:
  ros__parameters:
    wheel_names: ["main", "rear"]

    main:
      topic: "/platform/wheel/odometry"
      expected_hz: 20.0

    rear:
      topic: "/platform/wheel/rear/odometry"
      expected_hz: 20.0
```

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `wheel_names` | `[]` | 모니터링할 휠 이름 목록 |

### 휠별 파라미터 (`<휠이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | `/platform/wheel/odometry` | 구독할 Odometry 토픽 |
| `expected_hz` | `20.0` | 기대하는 수신 속도 (Hz) |
| `hz_warn_ratio` | `0.7` | 실제 Hz가 기대 Hz의 이 비율 미만이면 WARN |
| `hz_error_ratio` | `0.4` | 실제 Hz가 기대 Hz의 이 비율 미만이면 ERROR |
| `stale_timeout` | `1.0` | 이 시간(초) 이상 메시지가 없으면 STALE |
| `max_speed_warn_ms` | `3.0` | \|vx\| > 이 값 (m/s) → WARN (`0.0` = 비활성화) |
| `max_speed_error_ms` | `5.0` | \|vx\| > 이 값 (m/s) → ERROR (`0.0` = 비활성화) |

### Hz 판정 예시

`expected_hz: 20.0`, `hz_warn_ratio: 0.7`, `hz_error_ratio: 0.4` 일 때:

```
actual_hz >= 14.0  (20 × 0.7)  → OK
actual_hz >=  8.0  (20 × 0.4)  → WARN
actual_hz <   8.0              → ERROR
```

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# 휠 오도메트리만 필터링
ros2 topic echo /diagnostics | grep -A 12 "wheel/main"
```

### 출력 예시 (정상)

```
name:    "/sensor/wheel/main"
level:   0                        ← 0 = OK
message: "OK (20 Hz, vx=1.00 m/s)"
values:
  - key: "actual_hz"        value: "20.1"
  - key: "expected_hz"      value: "20"
  - key: "vx (m/s)"         value: "1.000"
  - key: "vy (m/s)"         value: "0.000"
  - key: "wz (rad/s)"       value: "0.0000"
  - key: "last_msg_sec_ago" value: "0.05"
```

### 출력 예시 (ERROR — NaN)

```
name:    "/sensor/wheel/main"
level:   2                        ← 2 = ERROR
message: "twist velocity에 NaN/Inf 포함"
```

### 출력 예시 (WARN — 속도 과대)

```
name:    "/sensor/wheel/main"
level:   1                        ← 1 = WARN
message: "속도 이상 (WARN 임계값 초과)"
values:
  - key: "vx (m/s)" value: "3.600"
```

### 출력 예시 (STALE — 수신 끊김)

```
name:    "/sensor/wheel/main"
level:   3                        ← 3 = STALE
message: "1.23s 동안 메시지 없음 (timeout=1.0s)"
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
| STALE | 3 | 회색 | 데이터 없음 (토픽이 오지 않거나 너무 오래됨) |

---

## 10. 자주 묻는 질문 (FAQ)

**Q. 휠 오도메트리가 STALE로 나옵니다.**

플랫폼 드라이버가 토픽을 발행하지 않고 있다는 뜻입니다.

```bash
ros2 topic list | grep odometry
ros2 topic hz /platform/wheel/odometry
```

---

**Q. 정지 상태인데 `vx`가 0이 아닙니다.**

인코더 노이즈나 드리프트입니다. 플랫폼 드라이버의 최소 속도 필터 설정을 확인하세요.
`max_speed_warn_ms`는 실제 로봇 최대 속도 기준으로 설정하며 정지 상태 노이즈와는 무관합니다.

---

**Q. 속도 WARN이 자주 발생합니다.**

로봇의 실제 최대 운용 속도에 맞게 `max_speed_warn_ms` 값을 조정하세요.
단순히 비활성화하려면 `0.0`으로 설정합니다.

```yaml
max_speed_warn_ms:  0.0   # 비활성화
max_speed_error_ms: 0.0   # 비활성화
```

---

**Q. GNSS 없을 때 이 체커가 왜 중요한가요?**

로컬라이제이션이 `DR_ONLY` 모드(Dead Reckoning)로 전환되면
IMU + 휠 오도메트리만으로 위치를 추정합니다.
이 상황에서 휠 오도메트리가 끊기면 위치 추정이 완전히 불가능해집니다.

---

**Q. `stale_timeout`을 조정하고 싶습니다.**

기본값 `1.0s`는 20Hz 기준으로 약 20프레임 유실 시 STALE입니다.
플랫폼 드라이버의 publish rate에 맞게 설정하세요.

```yaml
stale_timeout: 0.5   # 더 민감하게
stale_timeout: 2.0   # 더 여유있게
```
