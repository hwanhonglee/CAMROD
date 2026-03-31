# robot_localization_mode_checker

ROS 2 패키지로, `avg_msgs/AvgLocalizationStatus` 토픽을 구독해
**로컬라이제이션 전체 상태(모드·신뢰도·센서 상태·Innovation)**를
`/diagnostics` 토픽으로 발행합니다.

`localization_supervisor_node`가 이미 계산한 자가진단 결과를
Diagnostic 시스템으로 노출하여, 운영자가 `rqt_robot_monitor` 등에서
단일 항목으로 로컬라이제이션 건강 상태를 확인할 수 있습니다.

> **로컬라이제이션 레벨 관제**: 로컬라이제이션 알고리즘 전체가 정상인가?
> 개별 센서 품질은 `robot_gnss_quality_checker`, `robot_imu_checker`,
> `robot_wheel_odometry_checker` 가 담당합니다.

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

이 노드는 supervisor가 발행하는 Status 토픽을 구독하여, 주기적으로 아래 항목을 확인합니다.

```
/localization/status ──► localization_mode_checker_node ──► /diagnostics
(AvgLocalizationStatus)            ▲
                                   │
/localization/health ──────────────┘
(std_msgs/Bool)               1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | status가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 로컬라이제이션 모드 | NORMAL | WARN / ERROR |
| 신뢰도 (confidence) | `conf_warn` 이상 | WARN / ERROR |
| 센서 플래그 | gnss_ok, imu_ok, wheel_ok | WARN |
| Innovation norm | `innov_warn` 미만 | WARN / ERROR |

### 모드별 Diagnostic 레벨

| 모드 | 값 | 레벨 | 설명 |
|------|-----|------|------|
| `NORMAL` | 0 | OK | IMU + GNSS + Wheel 모두 정상 |
| `DEGRADED` | 1 | WARN | 일부 센서 부족, 성능 저하 |
| `DR_ONLY` | 2 | WARN | GNSS 없음, Dead Reckoning만 동작 |
| `INVALID` | 3 | ERROR | IMU 없음 또는 모든 센서 실패 |

### Confidence 판정 기준

`conf_warn: 0.6`, `conf_error: 0.3` 일 때:

```
confidence >= 0.6  → OK
confidence >= 0.3  → WARN
confidence <  0.3  → ERROR
```

Confidence는 아래 조건에 따라 감소합니다 (supervisor 계산):

| 조건 | 감소량 |
|------|--------|
| GNSS 불량 | -0.35 |
| IMU 없음 | -0.40 |
| Wheel 없음 | -0.15 |
| Innovation 높음 | -0.20 |
| GNSS 공분산 불량 | -0.15 |
| GNSS 점프 감지 | -0.15 |
| GNSS 수신율 저하 | -0.10 |

---

## 2. 패키지 구성

```
robot_localization_mode_checker/
├── src/
│   ├── localization_mode_checker_node.cpp      # 메인 노드 코드
│   └── localization_mode_dummy_publisher.cpp   # 테스트용 더미 퍼블리셔
├── config/
│   └── localization_mode_checker.yaml          # 설정 파일
├── launch/
│   ├── localization_mode_checker.launch.py     # 운영용 실행 파일
│   └── localization_mode_test.launch.py        # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `std_msgs` | `Bool` 메시지 타입 |
| `avg_msgs` | `AvgLocalizationStatus`, `AvgLocalizationMode` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

> **중요**: `avg_msgs`는 `camload` 워크스페이스에 있습니다.
> 빌드 전에 camload 워크스페이스를 먼저 source 해야 합니다.
> ```bash
> source ~/camload/install/setup.bash
> ```

---

## 4. 빌드 방법

```bash
# camload 워크스페이스 먼저 source
source ~/camload/install/setup.bash

cd ~/ros2_ws
colcon build --packages-select robot_localization_mode_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_localization_mode_checker localization_mode_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_localization_mode_checker localization_mode_test.launch.py
ros2 launch robot_localization_mode_checker localization_mode_test.launch.py scenario:=invalid
ros2 launch robot_localization_mode_checker localization_mode_test.launch.py scenario:=dr_only
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | NORMAL 모드, confidence=1.0 | OK |
| `degraded` | DEGRADED 모드, confidence=0.65, wheel_ok=false | WARN |
| `dr_only` | DR_ONLY 모드, gnss_ok=false | WARN |
| `invalid` | INVALID 모드, 모든 센서 실패 | ERROR |
| `low_conf` | NORMAL 모드, confidence=0.25 | ERROR |
| `no_gnss` | DEGRADED 모드, gnss_ok=false | WARN |
| `high_innov` | NORMAL 모드, gnss_innovation_norm=5.0 | WARN |
| `stale` | 퍼블리시 중단 | STALE |

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `status_topic` | `/localization/status` | 구독할 AvgLocalizationStatus 토픽 |
| `health_topic` | `/localization/health` | 구독할 Bool 헬스 플래그 토픽 |
| `stale_timeout` | `2.0` | 이 시간(초) 이상 status 없으면 STALE |
| `conf_warn` | `0.6` | confidence 이 값 미만 → WARN |
| `conf_error` | `0.3` | confidence 이 값 미만 → ERROR |
| `innov_warn` | `3.0` | innovation norm 이 값 초과 → WARN |
| `innov_error` | `6.0` | innovation norm 이 값 초과 → ERROR |

---

## 7. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 20 "localization/mode"
```

### 출력 예시 (정상)

```
name:    "/localization/mode"
level:   0                             ← OK
message: "NORMAL — 정상 로컬라이제이션"
values:
  - key: "mode"                    value: "NORMAL"
  - key: "mode_value"              value: "0"
  - key: "confidence"              value: "0.950"
  - key: "gnss_ok"                 value: "true"
  - key: "imu_ok"                  value: "true"
  - key: "wheel_ok"                value: "true"
  - key: "health"                  value: "true"
  - key: "gnss_innovation_norm"    value: "0.520"
  - key: "wheel_innovation_norm"   value: "0.310"
  - key: "last_msg_sec_ago"        value: "0.15"
```

### 출력 예시 (ERROR — INVALID 모드)

```
name:    "/localization/mode"
level:   2                             ← ERROR
message: "INVALID — 로컬라이제이션 불가 (IMU 없음 또는 전체 센서 실패)"
values:
  - key: "mode"       value: "INVALID"
  - key: "confidence" value: "0.000"
  - key: "imu_ok"     value: "false"
```

### 출력 예시 (WARN — DR_ONLY 모드)

```
name:    "/localization/mode"
level:   1                             ← WARN
message: "DR_ONLY — Dead Reckoning만 동작 중 (GNSS 없음)"
values:
  - key: "mode"       value: "DR_ONLY"
  - key: "gnss_ok"    value: "false"
  - key: "confidence" value: "0.450"
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
| WARN | 1 | 노랑 | 주의 필요 (동작하지만 성능·품질 저하) |
| ERROR | 2 | 빨강 | 오류 발생 (즉각 대응 필요) |
| STALE | 3 | 회색 | 데이터 없음 (토픽이 오지 않거나 너무 오래됨) |

---

## 9. 자주 묻는 질문 (FAQ)

**Q. `/localization/status` 토픽이 없습니다.**

`localization_supervisor_node`가 실행되지 않고 있습니다.
localization launch 파일을 확인하세요.

```bash
ros2 topic list | grep localization
ros2 node list | grep supervisor
```

---

**Q. avg_msgs를 찾을 수 없다는 빌드 오류가 납니다.**

camload 워크스페이스가 source되지 않은 상태입니다.

```bash
source ~/camload/install/setup.bash
colcon build --packages-select robot_localization_mode_checker
```

---

**Q. DEGRADED와 DR_ONLY 모두 WARN인데 구분이 필요합니다.**

`values` 섹션의 `mode` 필드로 구분할 수 있습니다.
`diagnostic_aggregator`에서 규칙을 추가하거나 `rqt_robot_monitor`에서
상세 값을 확인하세요.

---

**Q. confidence 임계값을 조정하고 싶습니다.**

```yaml
conf_warn:  0.5   # 더 민감하게
conf_error: 0.2   # 더 여유있게
```

supervisor 내부의 confidence 계산식과 함께 조정하는 것을 권장합니다.
