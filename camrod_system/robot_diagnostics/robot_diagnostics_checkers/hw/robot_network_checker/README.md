# robot_network_checker

ROS 2 패키지로, 로봇의 **WiFi 네트워크 인터페이스 상태**를 실시간으로 모니터링하여
`/diagnostics` 토픽으로 발행합니다.

WiFi 연결이 살아있는지, 신호 세기(RSSI)는 충분한지, 패킷 손실은 없는지를
`/proc`, `/sys` 파일 시스템을 직접 읽어서 확인합니다.

> **하드웨어 레벨 관제**: 통신 인프라(WiFi) 자체가 살아있는가?

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
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

이 노드는 토픽을 구독하지 않습니다. 대신 OS 파일 시스템에서 직접 네트워크 상태를 읽습니다.

```
/sys/class/net/<iface>/operstate   ─┐
/sys/class/net/<iface>/carrier     ─┼─► network_checker_node ──► /diagnostics
/proc/net/wireless                 ─┤        ▲
/proc/net/dev                      ─┘        │
                                        1초마다 체크
```

| 체크 항목 | 데이터 소스 | 정상 | 문제 시 |
|---|---|---|---|
| 연결 상태 | `/sys/class/net/<iface>/operstate`<br>`/sys/class/net/<iface>/carrier` | operstate=up, carrier=1 | WARN / ERROR |
| 신호 세기 (RSSI) | `/proc/net/wireless` | RSSI > -70 dBm | WARN / ERROR |
| 패킷 손실률 | `/proc/net/dev` | drop rate < 1% | WARN / ERROR |

---

## 2. 패키지 구성

```
robot_network_checker/
├── src/
│   └── network_checker_node.cpp    # 메인 노드 코드
├── config/
│   └── network_checker.yaml        # 설정 파일
├── launch/
│   └── network_checker.launch.py   # 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

> **참고** — HW 체커는 실제 시스템 값을 읽으므로 더미 퍼블리셔가 없습니다.
> 더미 테스트가 필요한 경우 실제 네트워크 환경을 직접 변경하여 테스트하세요.

---

## 3. 의존성

| 패키지 | 용도 |
|---|---|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

**런타임 의존 (OS)**

| 파일 | 용도 |
|---|---|
| `/sys/class/net/<iface>/operstate` | 링크 동작 상태 (up/down/unknown) |
| `/sys/class/net/<iface>/carrier` | 캐리어 감지 여부 (1=연결, 0=끊김) |
| `/proc/net/wireless` | RSSI(dBm), 링크 품질, 노이즈 |
| `/proc/net/dev` | rx/tx 패킷·에러·드롭 카운터 |

> **컨테이너 환경** — Docker 등에서 실행할 경우 `/proc`, `/sys` 를 호스트에서 마운트해야 합니다.
> `robot_hw_gpu_checker` 의 `Dockerfile` 참고.

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_network_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_network_checker network_checker.launch.py

# 파라미터 오버라이드
ros2 launch robot_network_checker network_checker.launch.py  # config yaml 수정 후 재빌드
```

인터페이스 이름을 확인하는 방법:

```bash
ip link show
# 또는
iw dev
```

---

## 6. 파라미터 설명

```yaml
network_checker:
  ros__parameters:

    # 모니터링할 WiFi 인터페이스 이름 (ip link show 로 확인)
    interface: "wlan0"

    connection:
      enabled: true               # 연결 상태 체크 on/off

    signal:
      enabled: true               # 신호 세기 체크 on/off
      rssi_warn:  -70.0           # dBm — 이 값보다 낮으면 WARN
      rssi_error: -85.0           # dBm — 이 값보다 낮으면 ERROR

    quality:
      enabled: true               # 패킷 품질 체크 on/off
      drop_rate_warn:  1.0        # % — 손실률이 이 값 초과이면 WARN
      drop_rate_error: 5.0        # % — 손실률이 이 값 초과이면 ERROR
```

### 파라미터 표

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz) |
| `interface` | `"wlan0"` | 모니터링할 네트워크 인터페이스 이름 |
| `connection.enabled` | `true` | 연결 상태 진단 항목 활성화 |
| `signal.enabled` | `true` | 신호 세기 진단 항목 활성화 |
| `signal.rssi_warn` | `-70.0` | RSSI WARN 임계값 (dBm, 이 값 미만) |
| `signal.rssi_error` | `-85.0` | RSSI ERROR 임계값 (dBm, 이 값 미만) |
| `quality.enabled` | `true` | 패킷 품질 진단 항목 활성화 |
| `quality.drop_rate_warn` | `1.0` | 패킷 손실률 WARN 임계값 (%) |
| `quality.drop_rate_error` | `5.0` | 패킷 손실률 ERROR 임계값 (%) |

### RSSI 판정 기준

RSSI 는 음수이며 값이 클수록(0에 가까울수록) 신호가 강합니다.

```
RSSI > -70 dBm              → OK    (양호)
-85 dBm < RSSI ≤ -70 dBm   → WARN  (약함)
RSSI ≤ -85 dBm              → ERROR (매우 약함)
```

### 패킷 손실률 판정 기준

직전 발행 주기 대비 delta 패킷 수로 계산됩니다.

```
drop_rate < 1%   → OK
drop_rate < 5%   → WARN
drop_rate ≥ 5%   → ERROR
```

---

## 7. 진단 항목 상세

### `/hardware/network/connection`

WiFi 인터페이스의 링크 상태를 확인합니다.

| 조건 | 레벨 | 메시지 예시 |
|---|---|---|
| operstate=up, carrier=1 | OK | `WiFi 연결됨 (wlan0)` |
| operstate=unknown 또는 carrier=0 | WARN | `연결 불안정 (operstate=unknown)` |
| operstate=down | ERROR | `인터페이스 down: wlan0` |
| 인터페이스 자체 없음 | ERROR | `인터페이스 없음: wlan0` |
| `/proc/net/wireless` 없음 | STALE | `/proc/net/wireless 에 wlan0 없음 (미연결?)` |

**Key-Value 출력**

| Key | 설명 |
|---|---|
| `interface` | 인터페이스 이름 |
| `operstate` | up / down / unknown |
| `carrier` | connected / no carrier |

---

### `/hardware/network/signal`

`/proc/net/wireless` 에서 WiFi 신호 품질을 읽습니다.

| 조건 | 레벨 | 메시지 예시 |
|---|---|---|
| RSSI > rssi_warn | OK | `RSSI -55 dBm  품질 81%` |
| rssi_error < RSSI ≤ rssi_warn | WARN | `RSSI -75 dBm  품질 50% (Weak)` |
| RSSI ≤ rssi_error | ERROR | `RSSI -90 dBm  품질 21% (Very Weak)` |
| `/proc/net/wireless` 에 항목 없음 | STALE | `/proc/net/wireless 에 wlan0 없음 (미연결?)` |

**Key-Value 출력**

| Key | 설명 |
|---|---|
| `rssi_dBm` | 신호 세기 (dBm, 음수) |
| `link_quality_%` | 링크 품질 (0~100%) |
| `noise_dBm` | 노이즈 레벨 (dBm) |
| `interface` | 인터페이스 이름 |

> **링크 품질 정규화** — 드라이버마다 최댓값이 다릅니다 (보통 70 또는 100).
> 이 노드는 값이 70 이하이면 max=70, 초과이면 max=100 으로 자동 판단하여 % 로 변환합니다.

---

### `/hardware/network/quality`

`/proc/net/dev` 의 rx/tx 패킷 카운터를 직전 주기와 비교하여 손실률을 계산합니다.

| 조건 | 레벨 | 메시지 예시 |
|---|---|---|
| worst_drop < drop_rate_warn | OK | `RX drop 0.00%  TX drop 0.00%` |
| drop_rate_warn ≤ worst_drop < drop_rate_error | WARN | `RX drop 1.50%  TX drop 0.10% (High drop)` |
| worst_drop ≥ drop_rate_error | ERROR | `RX drop 6.20%  TX drop 0.30% (Critical drop)` |
| `/proc/net/dev` 에 항목 없음 | STALE | `/proc/net/dev 에 wlan0 없음` |

**Key-Value 출력**

| Key | 설명 |
|---|---|
| `rx_drop_%` | RX 패킷 손실률 (%) |
| `tx_drop_%` | TX 패킷 손실률 (%) |
| `rx_err_%` | RX 에러율 (%) |
| `tx_err_%` | TX 에러율 (%) |
| `rx_packets_total` | 누적 RX 패킷 수 |
| `tx_packets_total` | 누적 TX 패킷 수 |

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# network만 필터링
ros2 topic echo /diagnostics | grep -A 10 "network"
```

### 출력 예시 — 정상 (connection)

```
name:    "/hardware/network/connection"
level:   0                          ← 0 = OK
message: "WiFi 연결됨 (wlan0)"
values:
  - key: "interface"   value: "wlan0"
  - key: "operstate"   value: "up"
  - key: "carrier"     value: "connected"
```

### 출력 예시 — 정상 (signal)

```
name:    "/hardware/network/signal"
level:   0
message: "RSSI -58 dBm  품질 77%"
values:
  - key: "rssi_dBm"          value: "-58"
  - key: "link_quality_%"    value: "77.1"
  - key: "noise_dBm"         value: "-95"
  - key: "interface"         value: "wlan0"
```

### 출력 예시 — WARN (신호 약함)

```
name:    "/hardware/network/signal"
level:   1                          ← 1 = WARN
message: "RSSI -75 dBm  품질 50% (Weak)"
```

### 출력 예시 — ERROR (인터페이스 없음)

```
name:    "/hardware/network/connection"
level:   2                          ← 2 = ERROR
message: "인터페이스 없음: wlan0"
values:
  - key: "interface"  value: "wlan0"
```

### GUI로 확인 (rqt)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 9. 진단 상태 레벨이란?

| 레벨 | 숫자 | 색상 | 의미 |
|---|---|---|---|
| OK | 0 | 초록 | 정상 동작 중 |
| WARN | 1 | 노랑 | 주의 필요 (동작하지만 품질 저하) |
| ERROR | 2 | 빨강 | 오류 발생 (즉각 대응 필요) |
| STALE | 3 | 회색 | 데이터 없음 (파일을 읽을 수 없거나 인터페이스 없음) |

---

## 10. 자주 묻는 질문 (FAQ)

**Q. 인터페이스 이름을 모릅니다.**

```bash
ip link show
# 또는
iw dev
# wlan0, wlp2s0, wlp3s0 등 환경마다 다릅니다
```

확인 후 `config/network_checker.yaml` 의 `interface` 를 수정하세요.

---

**Q. `/proc/net/wireless` 에 항목이 없다고 나옵니다.**

WiFi 가 연결되지 않은 상태이거나 유선(Ethernet) 인터페이스를 지정한 경우입니다.
`/proc/net/wireless` 는 WiFi 전용 파일로, 유선 인터페이스는 나타나지 않습니다.

---

**Q. 컨테이너 내에서 실행 시 모든 항목이 STALE 입니다.**

`/proc`, `/sys` 를 호스트에서 마운트해야 합니다.

```yaml
# docker-compose.yml 예시
volumes:
  - /proc:/proc:ro
  - /sys:/sys:ro
```

---

**Q. 패킷 손실률이 노드 시작 직후 이상한 값이 나옵니다.**

노드가 처음 실행될 때는 직전 카운터 스냅샷이 없어서 첫 번째 주기의 delta 계산이 0으로 처리됩니다.
두 번째 발행 주기부터 정상적인 값이 나옵니다.

---

**Q. Ethernet 인터페이스도 모니터링할 수 있나요?**

`connection` 과 `quality` 항목은 `/sys/class/net` 과 `/proc/net/dev` 기반이므로
Ethernet 인터페이스에도 동작합니다. 단, `signal` 항목은 `/proc/net/wireless` 를 사용하므로
WiFi 전용입니다. Ethernet 사용 시 `signal.enabled: false` 로 비활성화하세요.

```yaml
network_checker:
  ros__parameters:
    interface: "eth0"
    signal:
      enabled: false    # Ethernet 은 신호 세기 없음
```
