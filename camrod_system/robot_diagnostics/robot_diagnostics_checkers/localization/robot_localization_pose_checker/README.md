# robot_localization_pose_checker

ROS 2 패키지로, 로컬라이제이션 최종 출력 토픽 `geometry_msgs/PoseWithCovarianceStamped`를 구독해
**ESKF/Fusion 이후 위치 추정 결과가 Planning/Control에 공급하기에 충분한 품질인지**를
`/diagnostics` 토픽으로 발행합니다.

> **로컬라이제이션 출력 레벨 관제**: 알고리즘이 최종 출력하는 위치가 신뢰할 수 있는가?
>
> | 체커 | 역할 |
> |------|------|
> | `robot_gnss_quality_checker` | GNSS **입력** 품질 (Fusion에 쓸 수 있는가?) |
> | `robot_localization_mode_checker` | 로컬라이제이션 **모드·신뢰도** (supervisor 자가진단) |
> | **`robot_localization_pose_checker`** | 로컬라이제이션 **출력** 품질 (Planning에 공급 가능한가?) |

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

이 노드는 로컬라이제이션 출력 토픽을 **구독(subscribe)** 하고, 주기적으로 아래 항목을 확인합니다.

```
/localization/pose_with_covariance ──► localization_pose_checker_node ──► /diagnostics
(PoseWithCovarianceStamped)                       ▲
                                                  │
                                            1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 출력 속도 (Hz) | `expected_hz`의 70% 이상 | WARN / ERROR |
| XY 공분산 trace | `cov_warn_threshold` 미만 | WARN / ERROR |
| 위치 점프 | `max_jump_m` 미만 | ERROR |

### XY 공분산 trace 기준

`trace = cov[0] + cov[7] = var_x + var_y`

| trace 값 | σ (위치 불확실도) | 의미 |
|----------|------------------|------|
| < 1.0 | < 0.7m | OK — 위치 신뢰 가능 |
| 1.0 ~ 9.0 | 0.7m ~ 2.1m | WARN — 위치 불확실 |
| > 9.0 | > 2.1m | ERROR — 위치 추정 신뢰불가 |

> **GNSS 입력 공분산과의 차이**: 입력 GNSS 공분산이 높아도 Fusion 후 출력 공분산은
> 낮을 수 있고, 반대로 DR_ONLY 모드에서는 GNSS 없이도 출력 공분산이 점점 커집니다.
> 이 체커는 Fusion 결과를 직접 감시합니다.

### 위치 점프 감지

연속 수신된 두 메시지 사이의 XY 거리가 `max_jump_m`을 초과하면 ERROR.
30Hz 기준으로 한 프레임 간격(≈33ms) 동안 2m 이상 이동은 물리적으로 불가능합니다.

---

## 2. 패키지 구성

```
robot_localization_pose_checker/
├── src/
│   ├── localization_pose_checker_node.cpp       # 메인 노드 코드
│   └── localization_pose_dummy_publisher.cpp    # 테스트용 더미 퍼블리셔
├── config/
│   └── localization_pose_checker.yaml           # 설정 파일
├── launch/
│   ├── localization_pose_checker.launch.py      # 운영용 실행 파일
│   └── localization_pose_test.launch.py         # 테스트용 실행 파일
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
colcon build --packages-select robot_localization_pose_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_localization_pose_checker localization_pose_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_localization_pose_checker localization_pose_test.launch.py
ros2 launch robot_localization_pose_checker localization_pose_test.launch.py scenario:=jump
ros2 launch robot_localization_pose_checker localization_pose_test.launch.py scenario:=high_cov_error
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 정상 30Hz, 낮은 공분산, 점프 없음 | OK |
| `hz_warn` | Hz 저하 (warn 구간) | WARN |
| `hz_error` | Hz 심각 저하 (error 구간) | ERROR |
| `stale` | 퍼블리시 중단 | STALE |
| `high_cov_warn` | XY 공분산 trace > cov_warn_threshold | WARN |
| `high_cov_error` | XY 공분산 trace > cov_error_threshold | ERROR |
| `jump` | 5초마다 위치 점프 주입 (max_jump_m × 1.5) | ERROR |

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `pose_topic` | `/localization/pose_with_covariance` | 구독할 토픽 |
| `expected_hz` | `30.0` | 기대하는 출력 속도 (Hz) — ESKF 기본 30Hz |
| `hz_warn_ratio` | `0.7` | 실제 Hz가 기대 Hz의 이 비율 미만이면 WARN |
| `hz_error_ratio` | `0.4` | 실제 Hz가 기대 Hz의 이 비율 미만이면 ERROR |
| `stale_timeout` | `1.0` | 이 시간(초) 이상 메시지가 없으면 STALE |
| `cov_warn_threshold` | `1.0` | XY trace 이 값 초과 → WARN |
| `cov_error_threshold` | `9.0` | XY trace 이 값 초과 → ERROR |
| `max_jump_m` | `2.0` | 연속 프레임 간 최대 XY 점프 (m) → ERROR |

---

## 7. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 15 "localization/pose"
```

### 출력 예시 (정상)

```
name:    "/localization/pose"
level:   0                          ← OK
message: "OK (30 Hz, cov_trace=0.0200)"
values:
  - key: "actual_hz"            value: "30.1"
  - key: "expected_hz"          value: "30"
  - key: "xy_cov_trace"         value: "0.0200"
  - key: "cov_warn_threshold"   value: "1.0000"
  - key: "cov_error_threshold"  value: "9.0000"
  - key: "last_jump_m"          value: "0.003"
  - key: "max_jump_m"           value: "2.00"
  - key: "pos_x (m)"            value: "10.100"
  - key: "pos_y (m)"            value: "5.010"
  - key: "last_msg_sec_ago"     value: "0.03"
```

### 출력 예시 (ERROR — 위치 점프)

```
name:    "/localization/pose"
level:   2                          ← ERROR
message: "위치 점프 감지 (3.00m > 2.00m)"
values:
  - key: "last_jump_m" value: "3.000"
  - key: "max_jump_m"  value: "2.00"
```

### 출력 예시 (ERROR — 공분산 심각)

```
name:    "/localization/pose"
level:   2                          ← ERROR
message: "출력 공분산 심각 (위치 추정 불가)"
values:
  - key: "xy_cov_trace"        value: "10.8000"
  - key: "cov_error_threshold" value: "9.0000"
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

**Q. cov_error_threshold를 어떻게 설정해야 하나요?**

로봇 사양과 운용 환경에 따라 다릅니다. 아래를 참고하세요.

```
trace = σ²_x + σ²_y

정밀 제어 (실내): cov_warn=0.25 (σ≈0.35m), cov_error=1.0 (σ≈0.7m)
일반 운용 (실외): cov_warn=1.0  (σ≈0.7m),  cov_error=9.0 (σ≈2.1m)
```

---

**Q. DR_ONLY 모드에서 공분산이 계속 커집니다.**

정상 동작입니다. GNSS 없이 Dead Reckoning만으로 이동하면
Kalman Filter의 공분산이 시간에 따라 증가합니다.
`cov_error_threshold`를 조정하거나 `robot_localization_mode_checker`와 연계하여
DR_ONLY 상태임을 함께 확인하세요.

---

**Q. 위치 점프가 간헐적으로 발생합니다.**

ESKF가 GNSS를 재초기화할 때 발생할 수 있습니다.
`max_jump_m`을 높이거나, 로컬라이제이션 파라미터의 `reinit_distance_threshold_`
설정을 확인하세요.
