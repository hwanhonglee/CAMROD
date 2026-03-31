# robot_imu_checker

ROS 2 패키지로, IMU `sensor_msgs/Imu` 토픽을 구독해 **IMU 센서가 정상적으로 동작하고 있는지**를
`/diagnostics` 토픽으로 발행합니다.

수신 속도(Hz)가 제대로 나오는지, gyro/accel 데이터에 NaN/Inf가 없는지,
가속도 크기가 물리적으로 합리적인지를 실시간으로 모니터링합니다.

> **센서 레벨 관제**: IMU 센서 자체가 살아있는가?
> IMU는 ESKF 예측 단계(100Hz+)의 유일한 고주파 소스입니다.
> 끊기거나 NaN이 발생하면 필터 전체가 즉시 정지됩니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [IMU 소스 추가하기](#6-imu-소스-추가하기)
7. [파라미터 설명](#7-파라미터-설명)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

이 노드는 IMU 토픽을 **구독(subscribe)** 하고, 주기적으로 아래 항목을 확인합니다.

```
IMU 토픽 ──► imu_checker_node ──► /diagnostics
(Imu)              ▲
                   │
             1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 수신 속도 (Hz) | `expected_hz`의 70% 이상 | WARN (노랑) / ERROR (빨강) |
| angular_velocity NaN/Inf | 모든 성분이 유한값 | ERROR (빨강) |
| linear_acceleration NaN/Inf | 모든 성분이 유한값 | ERROR (빨강) |
| 가속도 크기 (\|\|a\|\|) | `accel_magnitude_warn` 미만 | WARN (노랑) / ERROR (빨강) |

> **Hz 계산 방식**: 최근 2초 내에 수신된 메시지 수로 계산합니다.

> **가속도 크기 체크**: `||a|| = sqrt(ax² + ay² + az²)`.
> 정지 상태에서는 약 9.81 m/s² (중력)이 정상입니다.
> 이동 중에는 더 클 수 있으므로 warn/error 임계값을 여유있게 설정하세요.
> `0.0`으로 설정하면 해당 임계값 체크를 비활성화합니다.

---

## 2. 패키지 구성

```
robot_imu_checker/
├── src/
│   ├── imu_checker_node.cpp        # 메인 노드 코드
│   └── imu_dummy_publisher.cpp     # 테스트용 더미 퍼블리셔
├── config/
│   └── imu_checker.yaml            # IMU 설정 파일
├── launch/
│   ├── imu_checker.launch.py       # 운영용 실행 파일
│   └── imu_test.launch.py          # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `sensor_msgs` | `Imu` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_imu_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_imu_checker imu_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_imu_checker imu_test.launch.py
ros2 launch robot_imu_checker imu_test.launch.py scenario:=nan_gyro
ros2 launch robot_imu_checker imu_test.launch.py scenario:=stale
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 정상 100Hz, 유효한 데이터 | OK |
| `hz_warn` | Hz 저하 (warn 구간) | WARN |
| `hz_error` | Hz 심각 저하 (error 구간) | ERROR |
| `stale` | 퍼블리시 중단 | STALE |
| `nan_gyro` | `angular_velocity` NaN 주입 | ERROR |
| `nan_accel` | `linear_acceleration` NaN 주입 | ERROR |
| `high_accel` | `accel_magnitude_warn` × 1.2 주입 | WARN |

---

## 6. IMU 소스 추가하기

`config/imu_checker.yaml`에 이름을 추가하고 해당 섹션을 작성합니다.

```yaml
imu_checker:
  ros__parameters:
    imu_names: ["main", "redundant"]

    main:
      topic: "/sensing/imu/data"
      expected_hz: 100.0

    redundant:
      topic: "/sensing/imu/redundant/data"
      expected_hz: 50.0
```

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `imu_names` | `[]` | 모니터링할 IMU 이름 목록 |

### IMU별 파라미터 (`<imu이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | `/sensing/imu/data` | 구독할 Imu 토픽 |
| `expected_hz` | `100.0` | 기대하는 수신 속도 (Hz) |
| `hz_warn_ratio` | `0.7` | 실제 Hz가 기대 Hz의 이 비율 미만이면 WARN |
| `hz_error_ratio` | `0.4` | 실제 Hz가 기대 Hz의 이 비율 미만이면 ERROR |
| `stale_timeout` | `0.5` | 이 시간(초) 이상 메시지가 없으면 STALE |
| `accel_magnitude_warn` | `30.0` | \|\|a\|\| > 이 값 (m/s²) → WARN (`0.0` = 비활성화) |
| `accel_magnitude_error` | `50.0` | \|\|a\|\| > 이 값 (m/s²) → ERROR (`0.0` = 비활성화) |

### Hz 판정 예시

`expected_hz: 100.0`, `hz_warn_ratio: 0.7`, `hz_error_ratio: 0.4` 일 때:

```
actual_hz >= 70.0  (100 × 0.7)  → OK
actual_hz >= 40.0  (100 × 0.4)  → WARN
actual_hz <  40.0               → ERROR
```

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# IMU만 필터링
ros2 topic echo /diagnostics | grep -A 15 "imu/main"
```

### 출력 예시 (정상)

```
name:    "/sensor/imu/main"
level:   0                        ← 0 = OK
message: "OK (100 Hz)"
values:
  - key: "actual_hz"              value: "100.2"
  - key: "expected_hz"            value: "100"
  - key: "gyro_x (rad/s)"         value: "0.001"
  - key: "gyro_y (rad/s)"         value: "-0.002"
  - key: "gyro_z (rad/s)"         value: "0.000"
  - key: "accel_x (m/s²)"         value: "0.032"
  - key: "accel_y (m/s²)"         value: "-0.015"
  - key: "accel_z (m/s²)"         value: "9.810"
  - key: "accel_magnitude (m/s²)" value: "9.811"
  - key: "last_msg_sec_ago"       value: "0.01"
```

### 출력 예시 (ERROR — NaN)

```
name:    "/sensor/imu/main"
level:   2                        ← 2 = ERROR
message: "angular_velocity에 NaN/Inf 포함"
```

### 출력 예시 (STALE — 수신 끊김)

```
name:    "/sensor/imu/main"
level:   3                        ← 3 = STALE
message: "0.62s 동안 메시지 없음 (timeout=0.5s)"
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

**Q. IMU가 STALE로 나옵니다.**

IMU 드라이버가 토픽을 발행하지 않고 있다는 뜻입니다.

```bash
ros2 topic list | grep imu
ros2 topic hz /sensing/imu/data
```

---

**Q. Hz가 100Hz인데 WARN이 뜹니다.**

노드 시작 직후 약 2초간은 rolling window가 채워지지 않아 낮게 나올 수 있습니다.
2초 후에도 낮으면 IMU 드라이버의 publish rate 설정을 확인하세요.

---

**Q. 가속도 크기 WARN이 이동 중에 자주 발생합니다.**

이동 중에는 가속도 크기가 중력(9.81 m/s²)보다 클 수 있습니다.
`accel_magnitude_warn` 값을 로봇의 최대 예상 가속도에 맞게 높여주세요.
단순히 비활성화하려면 `0.0`으로 설정합니다.

```yaml
accel_magnitude_warn:  0.0   # 비활성화
accel_magnitude_error: 0.0   # 비활성화
```

---

**Q. NaN이 간헐적으로 발생합니다.**

IMU 드라이버 또는 하드웨어 연결 문제입니다. 드라이버 로그를 확인하고
USB/시리얼 케이블 상태를 점검하세요.

---

**Q. `stale_timeout`을 늘리고 싶습니다.**

기본값 `0.5s`는 100Hz IMU 기준으로 약 50프레임 유실 시 STALE입니다.
IMU Hz가 낮은 경우 적절히 늘려주세요.

```yaml
stale_timeout: 1.0   # 50Hz IMU의 경우
```
