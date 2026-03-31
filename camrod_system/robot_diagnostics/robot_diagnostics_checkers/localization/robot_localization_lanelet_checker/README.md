# robot_localization_lanelet_checker

ROS 2 패키지로, `centerline_snapper_node`가 발행하는 `/localization/lanelet_pose`를 구독해
**로봇이 lanelet 맵 범위 안에 있는지**를 `/diagnostics` 토픽으로 발행합니다.

> **핵심 감시 포인트**: `centerline_snapper_node`는 로봇이 `max_search_radius` 밖에 있으면
> 토픽 발행을 **멈춥니다(silent failure)**. 이 체커는 그 침묵을 STALE로 드러냅니다.

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
/localization/pose  ──► centerline_snapper_node
                              │
                    맵 내부: 발행 ──► /localization/lanelet_pose ──► localization_lanelet_checker_node ──► /diagnostics
                    맵 외부: 발행 중단 (silent failure)                           ▲
                                                                            1초마다 체크
```

### 체크 항목

| 항목 | 판정 기준 | 진단 레벨 |
|------|-----------|-----------|
| 토픽 미수신 | `stale_timeout`(1.0s) 초과 | STALE (맵 밖 이탈 의심) |
| 발행 속도 | `< expected_hz × hz_warn_ratio` | WARN |
| 발행 속도 | `< expected_hz × hz_error_ratio` | ERROR |
| XY 공분산 trace | `> cov_warn_threshold`(1.0) | WARN |
| XY 공분산 trace | `> cov_error_threshold`(9.0) | ERROR |

> **covariance[14]** = 9999.0 (z 미관측 고정값). XY만 체크합니다.

---

## 2. 패키지 구성

```
robot_localization_lanelet_checker/
├── src/
│   ├── localization_lanelet_checker_node.cpp       # 메인 노드 코드
│   └── localization_lanelet_dummy_publisher.cpp    # 테스트용 더미 퍼블리셔
├── config/
│   └── localization_lanelet_checker.yaml           # 설정 파일
├── launch/
│   ├── localization_lanelet_checker.launch.py      # 운영용
│   └── localization_lanelet_test.launch.py         # 테스트용
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
colcon build --packages-select robot_localization_lanelet_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용
ros2 launch robot_localization_lanelet_checker localization_lanelet_checker.launch.py

# 테스트용
ros2 launch robot_localization_lanelet_checker localization_lanelet_test.launch.py
ros2 launch robot_localization_lanelet_checker localization_lanelet_test.launch.py scenario:=hz_warn
ros2 launch robot_localization_lanelet_checker localization_lanelet_test.launch.py scenario:=hz_error
ros2 launch robot_localization_lanelet_checker localization_lanelet_test.launch.py scenario:=stale
ros2 launch robot_localization_lanelet_checker localization_lanelet_test.launch.py scenario:=high_cov
ros2 launch robot_localization_lanelet_checker localization_lanelet_test.launch.py scenario:=no_snap
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 20 Hz, cov=0.1 발행 | OK |
| `hz_warn` | 속도 저하 (expected × 0.55) | WARN |
| `hz_error` | 속도 심각 저하 (expected × 0.2) | ERROR |
| `stale` | 퍼블리시 중단 | STALE |
| `high_cov` | cov_warn_threshold × 1.5 | WARN |
| `no_snap` | 퍼블리시 완전 중단 (centerline 없음) | STALE |

> `stale`과 `no_snap`은 모두 STALE이지만 의미가 다릅니다:
> - `stale`: 잠시 발행하다 중단 (일시적 맵 이탈)
> - `no_snap`: 처음부터 발행 안 함 (맵 로드 실패 또는 맵 범위 완전 이탈)

---

## 6. 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `lanelet_topic` | `/localization/lanelet_pose` | 감시할 토픽 |
| `expected_hz` | `20.0` | 기대 발행 주파수 (입력 포즈와 동일) |
| `hz_warn_ratio` | `0.7` | actual/expected < 이 값 → WARN |
| `hz_error_ratio` | `0.4` | actual/expected < 이 값 → ERROR |
| `stale_timeout` | `1.0` | 이 시간(초) 이상 메시지 없으면 STALE |
| `cov_warn_threshold` | `1.0` | XY cov trace > 이 값 → WARN |
| `cov_error_threshold` | `9.0` | XY cov trace > 이 값 → ERROR |

---

## 7. 진단 결과 확인하기

```bash
ros2 topic echo /diagnostics | grep -A 15 "localization/lanelet"
```

### 출력 예시 (정상)

```
name:    "/localization/lanelet"
level:   0                          ← OK
message: "OK (20 Hz, cov_trace=0.1000)"
values:
  - key: "actual_hz"            value: "20"
  - key: "expected_hz"          value: "20"
  - key: "xy_cov_trace"         value: "0.1000"
  - key: "pos_x (m)"            value: "123.456"
  - key: "pos_y (m)"            value: "78.901"
  - key: "last_msg_sec_ago"     value: "0.05"
```

### 출력 예시 (STALE — 맵 밖 이탈)

```
name:    "/localization/lanelet"
level:   3                          ← STALE
message: "2.3s 동안 메시지 없음 — 맵 범위 이탈 의심 (timeout=1.0s)"
values:
  - key: "last_msg_sec_ago"     value: "2.3"
```

---

## 8. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | lanelet 맵 내부, snapping 정상 |
| WARN | 1 | 노랑 | snapping 간헐 실패 또는 불확실 |
| ERROR | 2 | 빨강 | snapping 빈번 실패 또는 공분산 심각 |
| STALE | 3 | 회색 | 맵 범위 이탈 (토픽 발행 중단됨) |

---

## 9. 자주 묻는 질문 (FAQ)

**Q. 운영 중에 STALE이 됐습니다. 무엇을 확인해야 하나요?**

1. `centerline_snapper_node`가 살아있는지 확인: `ros2 node list | grep snapper`
2. 로봇 위치가 lanelet 맵 경계 근처인지 확인
3. `centerline_snapper_node`의 `max_search_radius` 파라미터 확인
4. 로그에서 `No centerline within search radius` 메시지 확인

---

**Q. stale_timeout을 얼마로 설정해야 하나요?**

입력 포즈(`/localization/pose`) 발행 주기의 5~10배를 권장합니다.
기본 20 Hz 입력이라면 `stale_timeout: 0.5 ~ 1.0`이 적절합니다.
