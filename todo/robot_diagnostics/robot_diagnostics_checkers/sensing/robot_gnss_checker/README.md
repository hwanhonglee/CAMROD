# robot_gnss_checker

ROS 2 패키지로, GNSS NavSatFix 토픽을 구독해 **GNSS 수신기가 정상적으로 동작하고 있는지**를
`/diagnostics` 토픽으로 발행합니다.

GNSS fix가 잡혀 있는지, 수신 속도(Hz)가 제대로 나오는지, 공분산 타입이 신뢰할 수 있는지를
실시간으로 모니터링합니다.

> **센서 레벨 관제**: GNSS 수신기 자체가 살아있는가?
> GNSS 데이터가 로컬라이제이션 fusion에 쓸 수 있는 품질인지는
> [`robot_localization_gnss_checker`](../../localization/robot_localization_gnss_checker/README.md)가 담당합니다.

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

이 노드는 GNSS NavSatFix 토픽을 **구독(subscribe)** 하고, 주기적으로 아래 항목을 확인합니다.

```
GNSS 토픽 ──► gnss_checker_node ──► /diagnostics
(NavSatFix)        ▲
                   │
             1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 수신 속도 (Hz) | `expected_hz`의 80% 이상 | WARN (노랑) / ERROR (빨강) |
| Fix 상태 | `STATUS_FIX` 이상 | ERROR (빨강) |
| 공분산 타입 | `DIAGONAL_KNOWN` 또는 `KNOWN` | WARN (노랑) |

> **Hz 계산 방식**: 최근 2초 내에 수신된 메시지 수로 계산합니다.

> **Fix 상태 값**:
> - `-1` (`STATUS_NO_FIX`) — 위성 신호 없음 → **ERROR**
> - `0` (`STATUS_FIX`) — 일반 GPS fix → **OK**
> - `1` (`STATUS_SBAS_FIX`) — SBAS 보정 → **OK**
> - `2` (`STATUS_GBAS_FIX`) — GBAS(RTK) 보정 → **OK**

> **공분산 타입 값**:
> - `0` (`UNKNOWN`) — 공분산 불명 → **WARN**
> - `1` (`APPROXIMATED`) — 근사값 → **WARN**
> - `2` (`DIAGONAL_KNOWN`) — 대각 공분산 → **OK**
> - `3` (`KNOWN`) — 완전 공분산 → **OK**

---

## 2. 패키지 구성

```
robot_gnss_checker/
├── src/
│   ├── gnss_checker_node.cpp       # 메인 노드 코드
│   └── gnss_dummy_publisher.cpp    # 테스트용 더미 퍼블리셔
├── config/
│   └── gnss_checker.yaml           # GNSS 설정 파일
├── launch/
│   ├── gnss_checker.launch.py      # 운영용 실행 파일
│   └── gnss_test.launch.py         # 테스트용 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `sensor_msgs` | `NavSatFix` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_gnss_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_gnss_checker gnss_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_gnss_checker gnss_test.launch.py
ros2 launch robot_gnss_checker gnss_test.launch.py scenario:=no_fix
ros2 launch robot_gnss_checker gnss_test.launch.py scenario:=stale
```

---

## 6. GNSS 소스 추가하기

`config/gnss_checker.yaml`에 이름을 추가하고 해당 섹션을 작성합니다.

```yaml
gnss_checker:
  ros__parameters:
    gnss_names: ["main", "sub"]

    main:
      topic: "/sensing/gnss/navsatfix"
      expected_hz: 5.0

    sub:
      topic: "/sensing/gnss/sub/navsatfix"
      expected_hz: 5.0
```

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `gnss_names` | `[]` | 모니터링할 GNSS 이름 목록 |

### GNSS별 파라미터 (`<gnss이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | `/sensing/gnss/navsatfix` | 구독할 NavSatFix 토픽 |
| `expected_hz` | `5.0` | 기대하는 수신 속도 (Hz) |
| `hz_warn_ratio` | `0.8` | 실제 Hz가 기대 Hz의 이 비율 미만이면 WARN |
| `hz_error_ratio` | `0.5` | 실제 Hz가 기대 Hz의 이 비율 미만이면 ERROR |
| `stale_timeout` | `2.0` | 이 시간(초) 이상 메시지가 없으면 STALE |

### Hz 판정 예시

`expected_hz: 5.0`, `hz_warn_ratio: 0.8`, `hz_error_ratio: 0.5` 일 때:

```
actual_hz >= 4.0  (5 × 0.8)  → OK
actual_hz >= 2.5  (5 × 0.5)  → WARN
actual_hz <  2.5             → ERROR
```

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# GNSS만 필터링
ros2 topic echo /diagnostics | grep -A 10 "gnss/main"
```

### 출력 예시 (정상)

```
name:    "/sensor/gnss/main"
level:   0                        ← 0 = OK
message: "OK (5.0 Hz, fix=0)"
values:
  - key: "actual_hz"        value: "5.0"
  - key: "expected_hz"      value: "5.0"
  - key: "fix_status"       value: "0"
  - key: "covariance_type"  value: "2"
  - key: "last_msg_sec_ago" value: "0.05"
```

### 출력 예시 (ERROR — fix 없음)

```
name:    "/sensor/gnss/main"
level:   2                        ← 2 = ERROR
message: "GNSS fix 없음 (NO_FIX)"
values:
  - key: "fix_status" value: "-1"
```

### 출력 예시 (STALE — 수신 끊김)

```
name:    "/sensor/gnss/main"
level:   3                        ← 3 = STALE
message: "3.2s 동안 메시지 없음 (timeout=2.0s)"
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

**Q. GNSS가 STALE로 나옵니다.**

GNSS 드라이버가 토픽을 발행하지 않고 있다는 뜻입니다.

```bash
ros2 topic list | grep gnss
ros2 topic hz /sensing/gnss/navsatfix
```

---

**Q. fix_status가 계속 -1(NO_FIX)입니다.**

실외에서 위성 신호를 확인하세요. 실내나 터널에서는 fix가 잡히지 않습니다.
안테나 연결 상태와 GNSS 드라이버 설정을 점검하세요.

---

**Q. 공분산 타입이 WARN(APPROXIMATED)으로 나옵니다.**

GNSS 드라이버가 정확한 공분산을 계산하지 못하고 있는 상태입니다.
이 상태에서도 위치 데이터는 발행되지만 신뢰도가 낮습니다.
로컬라이제이션 fusion 품질은
[`robot_localization_gnss_checker`](../../localization/robot_localization_gnss_checker/README.md)로
추가 확인하세요.

---

**Q. Hz가 기대보다 낮게 나옵니다.**

노드 시작 직후 약 2초간은 rolling window가 채워지지 않아 낮게 나올 수 있습니다.
2초 후에도 낮으면 GNSS 드라이버 설정(publish rate)을 확인하세요.

---

**Q. 발행 주기를 바꾸고 싶습니다.**

```yaml
publish_rate: 0.5   # 2초마다 발행
```
