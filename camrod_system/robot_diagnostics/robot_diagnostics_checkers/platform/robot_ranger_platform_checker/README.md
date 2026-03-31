# robot_ranger_platform_checker

ROS 2 패키지로, `ranger_ros2`가 발행하는 **Ranger 섀시 토픽**들을 구독해
플랫폼 상태를 `/diagnostics` 토픽으로 발행합니다.

CAN 통신 연결, 차량 상태(비상 정지·예외), 제어 모드(CAN/RC), 오류 코드 비트,
배터리, 오도메트리, 액추에이터 드라이버·모터 상태를 실시간으로 모니터링합니다.

> **플랫폼 레벨 관제**: ranger_ros2 노드가 실행 중이고 섀시가 정상인가?
> 이 체커는 `ranger_base_node` 위에서 동작하며, ugv_sdk → CAN → 로봇 펌웨어까지의
> 통신 체인이 살아있는지 확인합니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [파라미터 설명](#6-파라미터-설명)
7. [진단 항목 상세](#7-진단-항목-상세)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [알려진 버그 및 주의사항](#9-알려진-버그-및-주의사항)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

이 노드는 `ranger_ros2`의 4개 토픽을 **구독(subscribe)** 하고, 주기적으로 7개 항목을 확인합니다.

```
/system_state    ──┐
/battery_state   ──┤
/actuator_state  ──┤──► ranger_platform_checker_node ──► /diagnostics
/odom            ──┘              ▲
                                  │
                            1초마다 체크
```

| 진단 태스크 경로 | 체크 항목 | 소스 토픽 |
|----------------|----------|----------|
| `/platform/ranger/connection` | CAN 통신 연결 (staleness) | `/system_state` |
| `/platform/ranger/vehicle` | 차량 상태 (NORMAL/ESTOP/EXCEPTION) | `/system_state` |
| `/platform/ranger/control_mode` | 제어 모드 (CAN/RC/STANDBY) | `/system_state` |
| `/platform/ranger/error_code` | CAN 오류 코드 비트 10개 분석 | `/system_state` |
| `/platform/ranger/battery` | 배터리 전압·SOC·온도 | `/battery_state` |
| `/platform/ranger/odom` | 오도메트리 freshness & Hz | `/odom` |
| `/platform/ranger/actuator` | 액추에이터 드라이버·모터 상태 | `/actuator_state` |

---

## 2. 패키지 구성

```
robot_ranger_platform_checker/
├── src/
│   ├── ranger_platform_checker_node.cpp     # 메인 노드 코드
│   └── ranger_platform_dummy_publisher.cpp  # 테스트용 더미 퍼블리셔
├── config/
│   └── ranger_platform_checker.yaml         # 파라미터 설정 파일
├── launch/
│   └── ranger_platform_checker.launch.py    # 단독 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `ranger_msgs` | `SystemState`, `ActuatorStateArray` 메시지 타입 |
| `nav_msgs` | `Odometry` 메시지 타입 |
| `sensor_msgs` | `BatteryState` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | `BaseChecker` 공통 클래스 |

> `ranger_msgs`는 `ranger_ros2` 저장소에 포함되어 있습니다.
> `ranger_base_node`가 실행 중이어야 체커가 의미 있는 데이터를 수신합니다.

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_ranger_platform_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 단독 실행 (실제 ranger_base_node 필요)
ros2 launch robot_ranger_platform_checker ranger_platform_checker.launch.py

# 더미 퍼블리셔로 테스트 (시나리오별)
ros2 run robot_ranger_platform_checker ranger_platform_dummy_publisher \
  --ros-args -p scenario:=ok

ros2 run robot_ranger_platform_checker ranger_platform_dummy_publisher \
  --ros-args -p scenario:=estop

ros2 run robot_ranger_platform_checker ranger_platform_dummy_publisher \
  --ros-args -p scenario:=rc_mode
```

### 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 정상 상태 (CAN 제어, NORMAL, 배터리 정상) | 전 항목 OK |
| `estop` | 비상 정지 활성화 (vehicle_state=0x01) | `/platform/ranger/vehicle` → ERROR |
| `rc_mode` | RC 리모컨 제어 중 (control_mode=0x03) | `/platform/ranger/control_mode` → WARN |
| `low_battery` | 배터리 38V, SOC 8% | `/platform/ranger/battery` → ERROR |
| `error_code` | 모터 1·2 통신 오류 비트 활성 (0x0018) | `/platform/ranger/error_code` → ERROR |
| `stale` | 토픽 발행 중단 | 전 항목 → STALE |
| `actuator_fault` | 액추에이터[2] 드라이버 결함 비트 | `/platform/ranger/actuator` → ERROR |

---

## 6. 파라미터 설명

### 공통

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `system_state_topic` | `/system_state` | 구독 토픽 |
| `battery_state_topic` | `/battery_state` | 구독 토픽 |
| `actuator_state_topic` | `/actuator_state` | 구독 토픽 |
| `odom_topic` | `/odom` | 구독 토픽 |
| `stale_timeout` | `1.0` | system_state·actuator_state STALE 판정 타임아웃 (초) |

### 오도메트리

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `odom_expected_hz` | `50.0` | ranger_ros2 기본 update_rate |
| `odom_hz_warn_ratio` | `0.7` | 실제 Hz / 기대 Hz 비율이 이 값 미만이면 WARN |
| `odom_hz_error_ratio` | `0.4` | 실제 Hz / 기대 Hz 비율이 이 값 미만이면 ERROR |
| `odom_stale_timeout` | `1.0` | STALE 판정 타임아웃 (초) |

### 배터리 (Ranger 공칭 전압: 46~50V)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `battery.voltage_warn` | `43.0` | V 이하 → WARN |
| `battery.voltage_error` | `41.0` | V 이하 → ERROR |
| `battery.soc_warn` | `20.0` | % 이하 → WARN |
| `battery.soc_error` | `10.0` | % 이하 → ERROR |
| `battery.temp_warn` | `45.0` | °C 이상 → WARN |
| `battery.temp_error` | `55.0` | °C 이상 → ERROR |

### 액추에이터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `actuator.driver_temp_warn` | `60.0` | 드라이버 온도 °C 이상 → WARN |
| `actuator.driver_temp_error` | `75.0` | 드라이버 온도 °C 이상 → ERROR |
| `actuator.motor_temp_warn` | `70.0` | 모터 온도 °C 이상 → WARN |
| `actuator.motor_temp_error` | `85.0` | 모터 온도 °C 이상 → ERROR |
| `actuator.driver_voltage_warn` | `40.0` | 드라이버 전압 V 이하 → WARN |
| `actuator.driver_voltage_error` | `38.0` | 드라이버 전압 V 이하 → ERROR |

---

## 7. 진단 항목 상세

### `/platform/ranger/connection` — 섀시 CAN 통신 연결

`/system_state` 토픽의 수신 여부와 수신 주기를 감시합니다.

| 상태 | 조건 |
|------|------|
| OK | `stale_timeout` 이내에 메시지 수신 중 |
| STALE | 토픽 수신 없음 또는 타임아웃 초과 |

```
name:    "/platform/ranger/connection"
level:   0
message: "OK (0.02s ago)"
values:
  - key: "last_msg_sec_ago"  value: "0.02"
  - key: "motion_mode"       value: "DUAL_ACKERMAN"
```

---

### `/platform/ranger/vehicle` — 차량 상태

펌웨어에서 보고하는 차량 상태 코드를 감시합니다.

| 상태 | 조건 |
|------|------|
| OK | `vehicle_state = 0x00` (NORMAL) |
| ERROR | `vehicle_state = 0x01` (E-STOP 활성화) |
| ERROR | `vehicle_state = 0x02` (시스템 예외) |

---

### `/platform/ranger/control_mode` — 제어 모드

현재 어떤 방식으로 섀시를 제어하고 있는지 확인합니다.

| 상태 | 조건 |
|------|------|
| OK | `control_mode = 0x01` (CAN 명령 제어) |
| WARN | `control_mode = 0x03` (RC 리모컨 — CAN 명령 무시됨) |
| WARN | `control_mode = 0x00` (대기 — CAN 제어 비활성) |
| WARN | `control_mode = 0x02` (UART 제어) |

> RC 리모컨이 켜져 있으면 CAN 명령이 무시됩니다.
> 자율주행 전 반드시 CAN 모드(0x01)인지 확인해야 합니다.

---

### `/platform/ranger/error_code` — CAN 오류 코드

`/system_state.error_code` 의 16비트 비트마스크를 개별 항목으로 분석합니다.

| 비트 | 마스크 | 항목 | 레벨 |
|------|--------|------|------|
| 0 | `0x0001` | `BATTERY_FAULT` | ERROR |
| 1 | `0x0002` | `BATTERY_WARN` | WARN |
| 2 | `0x0004` | `RC_SIGNAL_LOSS` | WARN |
| 3 | `0x0008` | `MOTOR1_COMM` | ERROR |
| 4 | `0x0010` | `MOTOR2_COMM` | ERROR |
| 5 | `0x0020` | `MOTOR3_COMM` | ERROR |
| 6 | `0x0040` | `MOTOR4_COMM` | ERROR |
| 7 | `0x0080` | `STEER_ENCODER` | ERROR |
| 8 | `0x0100` | `MOTOR_DRIVER` | ERROR |
| 9 | `0x0200` | `HL_COMM` | ERROR |

```
name:    "/platform/ranger/error_code"
level:   2
message: "오류 코드 활성: 0x0018"
values:
  - key: "error_code_hex"  value: "0x0018"
  - key: "MOTOR1_COMM"     value: "active"
  - key: "MOTOR2_COMM"     value: "active"
  - key: "BATTERY_FAULT"   value: "-"
  ...
```

---

### `/platform/ranger/battery` — 배터리

`/battery_state` 토픽의 전압·전류·SOC·온도를 감시합니다.
`/battery_state`가 없으면 `/system_state.battery_voltage`로 대체합니다.

| 레벨 | 조건 예시 |
|------|---------|
| OK | 47.5V, SOC 78%, 28°C |
| WARN | 전압 42V 또는 SOC 15% 또는 온도 47°C |
| ERROR | 전압 40V 또는 SOC 8% 또는 온도 56°C |

---

### `/platform/ranger/odom` — 오도메트리

`/odom` 토픽의 수신율과 freshness를 감시합니다.
Hz 계산은 최근 2초 rolling window 방식을 사용합니다.

```
name:    "/platform/ranger/odom"
level:   0
message: "OK (50 Hz)"
values:
  - key: "actual_hz"        value: "50.2"
  - key: "expected_hz"      value: "50.0"
  - key: "last_msg_sec_ago" value: "0.02"
```

---

### `/platform/ranger/actuator` — 액추에이터 상태

8개 액추에이터(구동 4개, 조향 4개)의 드라이버·모터 상태를 감시합니다.

**드라이버 상태 비트:**

| 비트 | 마스크 | 항목 | 레벨 |
|------|--------|------|------|
| 0 | `0x01` | 입력 전압 낮음 | WARN |
| 1 | `0x02` | 모터 과열 | WARN |
| 2 | `0x04` | 드라이버 과부하 | WARN |
| 3 | `0x08` | 드라이버 과열 | WARN |
| 5 | `0x20` | 센서 결함 | ERROR |
| 6 | `0x40` | 드라이버 결함 | ERROR |

```
name:    "/platform/ranger/actuator"
level:   0
message: "OK (8 액추에이터)"
values:
  - key: "act[0].driver_voltage_V"  value: "47.20"
  - key: "act[0].driver_temp_C"     value: "35.0"
  - key: "act[0].motor_temp_C"      value: "40.0"
  - key: "act[0].rpm"               value: "120"
  ...
```

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# Ranger 플랫폼 항목만 필터링
ros2 topic echo /diagnostics | grep -A 10 "/platform/ranger"

# aggregator 집계 결과 (Platform/Ranger 트리)
ros2 topic echo /diagnostics_agg | grep -A 20 "Platform"
```

### GUI로 확인 (rqt)

```bash
# 개별 상태 트리 뷰
ros2 run rqt_runtime_monitor rqt_runtime_monitor

# 집계된 로봇 상태 뷰
ros2 run rqt_robot_monitor rqt_robot_monitor
```

aggregator 트리에서 `Robot → Platform → Ranger` 아래 7개 항목을 확인합니다.

---

## 9. 알려진 버그 및 주의사항

### ranger_msgs::msg::SystemState 상수 오류

```
// SystemState.msg 에 잘못 정의된 상수:
uint8 CONTROL_MODE_RC = 0   ← 실제 펌웨어 값은 0x03

// 이 체커는 ugv_sdk agilex_types.h 기준 값을 직접 사용합니다:
CONTROL_MODE_STANDBY = 0x00
CONTROL_MODE_CAN     = 0x01
CONTROL_MODE_UART    = 0x02
CONTROL_MODE_RC      = 0x03   ← 올바른 값
```

### actuator_state 인덱스 버그 (ranger_messenger.cpp)

`ranger_ros2`의 `ranger_messenger.cpp`에서 8개 액추에이터를 모두 인덱스 0의
복사본으로 발행하는 버그가 있습니다. 따라서 `/platform/ranger/actuator` 의
`act[1]~act[7]` 값은 `act[0]`과 동일하게 표시됩니다.
이 체커는 수신된 데이터를 그대로 감시하므로, 버그 수정 후 자동으로 올바른 값이 표시됩니다.

---

## 10. 자주 묻는 질문 (FAQ)

**Q. 모든 항목이 STALE로 표시됩니다.**

`ranger_base_node`가 실행 중인지 확인하세요.

```bash
ros2 node list | grep ranger
ros2 topic list | grep system_state
ros2 topic hz /system_state
```

---

**Q. `control_mode`가 WARN(RC)으로 표시됩니다.**

RC 리모컨이 켜져 있으면 CAN 명령이 무시됩니다.
리모컨을 끄거나 조이스틱 전원을 차단한 뒤 CAN 모드(0x01)가 되는지 확인하세요.

---

**Q. `connection`은 OK인데 `vehicle`이 ERROR(E-STOP)입니다.**

섀시 비상 정지 버튼이 눌려있거나 안전 장치가 활성화된 상태입니다.
물리적인 E-STOP 버튼을 확인하세요.

---

**Q. `battery`가 system_state 기준으로 표시됩니다.**

`/battery_state` 토픽이 수신되지 않을 때 `/system_state.battery_voltage`로 자동
대체됩니다. BMS(배터리 관리 시스템) 통신을 확인하세요.

```bash
ros2 topic hz /battery_state
```

---

**Q. `odom` Hz가 노드 시작 직후 낮게 나옵니다.**

rolling window(2초)가 채워지기 전까지 낮게 계산될 수 있습니다. 2초 후에도
기대값의 70% 미만이면 `ranger_base_node`의 `update_rate` 파라미터를 확인하세요.

```bash
ros2 param get /ranger_base_node update_rate
```
