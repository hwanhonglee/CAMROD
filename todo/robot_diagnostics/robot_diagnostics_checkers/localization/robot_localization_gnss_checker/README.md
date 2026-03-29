# robot_localization_gnss_checker

ROS 2 패키지로, GNSS PoseWithCovarianceStamped 토픽을 구독해
**GNSS 데이터가 로컬라이제이션 fusion에 사용 가능한 품질인지**를
`/diagnostics` 토픽으로 발행합니다.

위치 공분산이 너무 커서 ESKF가 신뢰할 수 없는지, 연속 수신 간 비정상적인
위치 점프가 발생했는지를 실시간으로 모니터링합니다.

> **로컬라이제이션 레벨 관제**: GNSS 데이터가 fusion에 쓸 수 있는 품질인가?
> GNSS 수신기 자체가 살아있는지(fix 여부, 수신율)는
> [`robot_gnss_checker`](../../sensing/robot_gnss_checker/README.md)가 담당합니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [GNSS 소스 추가하기](#6-gnss-소스-추가하기)
7. [파라미터 설명](#7-파라미터-설명)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

이 노드는 GNSS PoseWithCovarianceStamped 토픽을 **구독(subscribe)** 하고,
주기적으로 아래 항목을 확인합니다.

```
GNSS 포즈 토픽 ──► localization_gnss_checker_node ──► /diagnostics
(PoseWithCovarianceStamped)           ▲
                                      │
                                1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 수신 속도 (Hz) | `expected_hz`의 80% 이상 | WARN (노랑) / ERROR (빨강) |
| XY 공분산 trace | `cov_warn_threshold` 미만 | WARN (노랑) / ERROR (빨강) |
| 위치 점프 | `max_jump_m` 미만 | ERROR (빨강) |

> **XY 공분산 trace**: `covariance[0]`(var_x) + `covariance[7]`(var_y)의 합.
> 값이 클수록 GNSS 위치 불확실도가 높음.
> ESKF는 이 값이 크면 GNSS 측정을 자동으로 reject하거나 가중치를 낮춥니다.

> **위치 점프**: 연속으로 수신된 두 메시지 사이의 XY 이동 거리.
> 갑작스러운 큰 점프는 GNSS multipath, 위성 전환 등 이상 신호를 의미합니다.

---

## 2. 패키지 구성

```
robot_localization_gnss_checker/
├── src/
│   ├── localization_gnss_checker_node.cpp      # 메인 노드 코드
│   └── localization_gnss_dummy_publisher.cpp   # 테스트용 더미 퍼블리셔
├── config/
│   └── localization_gnss_checker.yaml          # 설정 파일
├── launch/
│   ├── localization_gnss_checker.launch.py     # 운영용 실행 파일
│   └── localization_gnss_test.launch.py        # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `geometry_msgs` | `PoseWithCovarianceStamped` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_localization_gnss_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_localization_gnss_checker localization_gnss_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_localization_gnss_checker localization_gnss_test.launch.py
ros2 launch robot_localization_gnss_checker localization_gnss_test.launch.py scenario:=high_cov_error
ros2 launch robot_localization_gnss_checker localization_gnss_test.launch.py scenario:=jump
```

---

## 6. GNSS 소스 추가하기

`config/localization_gnss_checker.yaml`에 이름을 추가하고 해당 섹션을 작성합니다.

```yaml
localization_gnss_checker:
  ros__parameters:
    gnss_names: ["main", "sub"]

    main:
      topic: "/sensing/gnss/pose_with_covariance"
      cov_warn_threshold:  4.0
      cov_error_threshold: 25.0
      max_jump_m:          5.0

    sub:
      topic: "/sensing/gnss/sub/pose_with_covariance"
      cov_warn_threshold:  4.0
      cov_error_threshold: 25.0
      max_jump_m:          5.0
```

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `gnss_names` | `[]` | 모니터링할 GNSS 소스 이름 목록 |

### GNSS별 파라미터 (`<gnss이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | `/sensing/gnss/pose_with_covariance` | 구독할 PoseWithCovarianceStamped 토픽 |
| `expected_hz` | `5.0` | 기대하는 수신 속도 (Hz) |
| `hz_warn_ratio` | `0.8` | 실제 Hz가 기대 Hz의 이 비율 미만이면 WARN |
| `hz_error_ratio` | `0.5` | 실제 Hz가 기대 Hz의 이 비율 미만이면 ERROR |
| `stale_timeout` | `2.0` | 이 시간(초) 이상 메시지가 없으면 STALE |
| `cov_warn_threshold` | `4.0` | XY 공분산 trace 상한 (WARN). σ ≈ 1.4m |
| `cov_error_threshold` | `25.0` | XY 공분산 trace 상한 (ERROR). σ ≈ 3.5m |
| `max_jump_m` | `5.0` | 연속 수신 간 최대 XY 이동 거리 (ERROR) |

### 공분산 임계값 설정 가이드

XY 공분산 trace = `var_x + var_y` = `σ_x² + σ_y²`.
각 축의 위치 표준편차(σ)를 기준으로 설정합니다.

| 상황 | 표준편차 (σ) | XY trace |
|------|-------------|---------|
| RTK fix | 0.02 m | 0.001 |
| 일반 GNSS fix | 1.0 m | 2.0 |
| WARN 권장 | 1.4 m | 4.0 |
| ERROR 권장 | 3.5 m | 25.0 |
| 실내 / 터널 | > 10 m | > 200 |

ESKF의 `gnss_r_pos` (GNSS 노이즈 행렬) 설정값과 일치시키는 것을 권장합니다.

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# 로컬라이제이션 GNSS만 필터링
ros2 topic echo /diagnostics | grep -A 15 "localization/gnss/main"
```

### 출력 예시 (정상)

```
name:    "/localization/gnss/main"
level:   0                        ← 0 = OK
message: "OK (5.0 Hz, cov_trace=1.0000)"
values:
  - key: "actual_hz"           value: "5.0"
  - key: "expected_hz"         value: "5.0"
  - key: "xy_cov_trace"        value: "1.0000"
  - key: "cov_warn_threshold"  value: "4.0000"
  - key: "cov_error_threshold" value: "25.0000"
  - key: "last_jump_m"         value: "0.010"
  - key: "last_msg_sec_ago"    value: "0.05"
```

### 출력 예시 (WARN — 공분산 높음)

```
name:    "/localization/gnss/main"
level:   1                        ← 1 = WARN
message: "GNSS 공분산 높음 (fusion 품질 저하)"
values:
  - key: "xy_cov_trace"        value: "14.5000"
  - key: "cov_warn_threshold"  value: "4.0000"
```

### 출력 예시 (ERROR — 위치 점프)

```
name:    "/localization/gnss/main"
level:   2                        ← 2 = ERROR
message: "위치 점프 감지 (7.50m > 5.00m)"
values:
  - key: "last_jump_m" value: "7.500"
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
| WARN | 1 | 노랑 | fusion 품질 저하 (동작은 하지만 정확도 감소) |
| ERROR | 2 | 빨강 | fusion 불가 수준 (즉각 대응 필요) |
| STALE | 3 | 회색 | 데이터 없음 (토픽이 오지 않거나 너무 오래됨) |

---

## 10. 자주 묻는 질문 (FAQ)

**Q. 공분산이 WARN/ERROR인데 로봇이 멀쩡히 달립니다.**

ESKF가 GNSS를 reject하고 IMU + 휠 오도메트리만으로 Dead Reckoning 중일 수 있습니다.
`/localization/mode` 토픽으로 현재 로컬라이제이션 모드를 확인하세요.

```bash
ros2 topic echo /localization/mode
```

---

**Q. `cov_warn_threshold`를 어떻게 설정해야 하나요?**

ESKF 설정 파일의 GNSS 노이즈 행렬(`gnss_r_pos`) 대각값과 맞추는 것을 권장합니다.
예를 들어 `gnss_r_pos: 2.0` 이면 `cov_warn_threshold: 4.0` (σ²=2.0 × 2축)이 적합합니다.

---

**Q. 위치 점프가 자주 발생합니다.**

GNSS multipath(건물 반사), 위성 배열 변경, RTK 기준국 전환 등이 원인일 수 있습니다.
`max_jump_m` 값을 실제 운용 속도와 수신 주기를 고려해 조정하세요.

```
max_jump_m = 최대속도(m/s) × 수신주기(s) × 여유배수(2~3)
예: 3m/s × 0.2s × 3 = 1.8m
```

---

**Q. GNSS 수신기가 정상인데 이 checker만 WARN입니다.**

GNSS 수신기 자체는 fix를 잡고 있지만 위치 불확실도가 높은 상태입니다.
이는 개방 공간이 아니거나 위성 수가 적은 환경에서 정상적으로 발생할 수 있습니다.
GNSS 수신기 상태는 [`robot_gnss_checker`](../../sensing/robot_gnss_checker/README.md)로
별도 확인하세요.

---

**Q. 발행 주기를 바꾸고 싶습니다.**

```yaml
publish_rate: 0.5   # 2초마다 발행
```
