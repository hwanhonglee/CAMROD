# robot_lidar_checker

ROS 2 패키지로, LiDAR 포인트클라우드 토픽을 구독해 **LiDAR 센서가 정상적으로 동작하고 있는지**를
`/diagnostics` 토픽으로 발행합니다.

LiDAR가 살아있는지, 스캔 속도(Hz)가 제대로 나오는지, 포인트 수나 NaN 비율이 정상인지를
실시간으로 모니터링합니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [LiDAR 추가하기](#6-lidar-추가하기)
7. [파라미터 설명](#7-파라미터-설명)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

이 노드는 LiDAR 포인트클라우드 토픽을 **구독(subscribe)** 하고, 주기적으로 아래 항목을 확인합니다.

```
LiDAR 토픽 ──► lidar_checker_node ──► /diagnostics
(PointCloud2)         ▲
                      │
                1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 스캔 속도 (Hz) | `expected_hz`의 80% 이상 | WARN (노랑) / ERROR (빨강) |
| 포인트 수 | `min_point_count` 이상 | ERROR (빨강) |
| NaN 비율 | `max_nan_ratio` 이하 | ERROR (빨강) |

> **Hz 계산 방식**: 최근 2초 내에 수신된 메시지 수로 계산합니다.

> **NaN 포인트란?**: `x`, `y`, `z` 값이 `NaN` 또는 `Inf`인 포인트입니다.
> 센서 불량, 반사율 문제 등으로 발생하며 비율이 높으면 데이터를 신뢰하기 어렵습니다.

---

## 2. 패키지 구성

```
robot_lidar_checker/
├── src/
│   └── lidar_checker_node.cpp    # 메인 노드 코드
├── config/
│   └── lidar_checker.yaml        # LiDAR 설정 파일
├── launch/
│   └── lidar_checker.launch.py   # 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `sensor_msgs` | `PointCloud2` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
# 워크스페이스 루트로 이동
cd ~/ros2_ws

# 빌드
colcon build --packages-select robot_lidar_checker

# 빌드 결과 적용
source install/setup.bash
```

> 처음 빌드하거나 의존 패키지도 함께 빌드하려면:
> ```bash
> colcon build
> source install/setup.bash
> ```

---

## 5. 실행 방법

```bash
# launch 파일로 실행 (권장)
ros2 launch robot_lidar_checker lidar_checker.launch.py

# 직접 실행
ros2 run robot_lidar_checker lidar_checker_node \
  --ros-args --params-file <설정파일 경로>
```

---

## 6. LiDAR 추가하기

`config/lidar_checker.yaml` 파일을 수정하면 됩니다.

### LiDAR가 1대일 때

```yaml
lidar_checker:
  ros__parameters:
    lidar_names: ["main"]

    main:
      topic: "/sensing/lidar/points"
      expected_hz: 10.0
      stale_timeout: 2.0
```

### LiDAR가 여러 대일 때 (원본 + 전처리 토픽 동시 감시 등)

`lidar_names` 목록에 이름을 추가하고, 같은 이름의 섹션을 추가합니다.

```yaml
lidar_checker:
  ros__parameters:
    lidar_names: ["main", "filtered"]

    main:
      topic: "/sensing/lidar/points"
      expected_hz: 10.0
      hz_warn_ratio: 0.8
      hz_error_ratio: 0.5
      stale_timeout: 2.0
      min_point_count: 100
      max_nan_ratio: 0.1

    filtered:
      topic: "/sensing/lidar/points_filtered"
      expected_hz: 10.0
      hz_warn_ratio: 0.8
      hz_error_ratio: 0.5
      stale_timeout: 2.0
      min_point_count: 50
      max_nan_ratio: 0.0    # 전처리 후에는 NaN이 없어야 함
```

노드 하나가 모든 LiDAR를 함께 모니터링합니다.

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz). 1.0 = 1초마다 |
| `lidar_names` | `[]` | 모니터링할 LiDAR 이름 목록 |

### LiDAR별 파라미터 (`<lidar이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | `/sensing/lidar/points` | 구독할 PointCloud2 토픽 |
| `expected_hz` | `10.0` | 기대하는 스캔 속도 (Hz) |
| `hz_warn_ratio` | `0.8` | 실제 Hz가 기대 Hz의 이 비율 미만이면 WARN |
| `hz_error_ratio` | `0.5` | 실제 Hz가 기대 Hz의 이 비율 미만이면 ERROR |
| `stale_timeout` | `2.0` | 이 시간(초) 이상 메시지가 없으면 STALE |
| `min_point_count` | `100` | 스캔 1회당 최소 포인트 수. `0` = 체크 안 함 |
| `max_point_count` | `0` | 스캔 1회당 최대 포인트 수. `0` = 체크 안 함 |
| `max_nan_ratio` | `0.1` | NaN·Inf 포인트 비율 상한 (0.0~1.0). `0.0` = 체크 안 함 |

### Hz 판정 예시

`expected_hz: 10.0`, `hz_warn_ratio: 0.8`, `hz_error_ratio: 0.5` 일 때:

```
actual_hz >= 8.0  (10 × 0.8)  → OK
actual_hz >= 5.0  (10 × 0.5)  → WARN
actual_hz <  5.0              → ERROR
```

### NaN 비율 판정 예시

`max_nan_ratio: 0.1` 일 때:

```
nan_ratio <= 0.1  (포인트의 10% 이하가 NaN)  → OK
nan_ratio >  0.1                             → ERROR
```

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# 특정 LiDAR만 필터링
ros2 topic echo /diagnostics | grep -A 10 "lidar/main"
```

### 출력 예시 (정상)

```
name:    "/sensor/lidar/main"
level:   0                        ← 0 = OK
message: "OK (10.1 Hz, 45320 pts)"
values:
  - key: "actual_hz"        value: "10.1"
  - key: "expected_hz"      value: "10.0"
  - key: "point_count"      value: "45320"
  - key: "nan_ratio"        value: "0.002"
  - key: "last_msg_sec_ago" value: "0.05"
```

### 출력 예시 (STALE — LiDAR 연결 끊김)

```
name:    "/sensor/lidar/main"
level:   3                        ← 3 = STALE
message: "3.2s 동안 메시지 없음 (timeout=2.0s)"
```

### 출력 예시 (WARN — 스캔 속도 저하)

```
name:    "/sensor/lidar/main"
level:   1                        ← 1 = WARN
message: "Hz 저하: 7.2 Hz (기대: 10.0, warn ratio: 0.8)"
values:
  - key: "actual_hz"   value: "7.2"
  - key: "expected_hz" value: "10.0"
```

### GUI로 확인 (rqt)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 9. 진단 상태 레벨이란?

ROS 2 diagnostics 는 4가지 상태 레벨을 사용합니다.

| 레벨 | 숫자 | 색상 | 의미 |
|------|------|------|------|
| OK | 0 | 초록 | 정상 동작 중 |
| WARN | 1 | 노랑 | 주의 필요 (동작은 하지만 성능 저하 등) |
| ERROR | 2 | 빨강 | 오류 발생 (즉각 대응 필요) |
| STALE | 3 | 회색 | 데이터 없음 (토픽이 오지 않거나 너무 오래됨) |

---

## 10. 자주 묻는 질문 (FAQ)

**Q. 노드를 실행했는데 아무 출력이 없어요.**

정상입니다. 진단 결과는 `/diagnostics` 토픽으로 발행되므로
`ros2 topic echo /diagnostics` 로 확인하세요.

---

**Q. LiDAR가 STALE로 나옵니다.**

LiDAR 토픽이 발행되지 않고 있다는 뜻입니다. 아래를 확인하세요.

```bash
# 토픽이 실제로 발행되고 있는지 확인
ros2 topic list | grep lidar
ros2 topic hz /sensing/lidar/points
```

`topic` 파라미터의 토픽 이름이 실제 LiDAR 드라이버가 발행하는 토픽과 일치하는지 확인하세요.

---

**Q. Hz가 기대보다 낮게 나옵니다.**

노드 시작 직후 약 2초간은 Hz 계산 window가 채워지지 않아 낮게 나올 수 있습니다.
2초 후에도 계속 낮으면 LiDAR 드라이버나 시스템 부하를 확인하세요.

---

**Q. NaN 비율이 높게 나옵니다.**

LiDAR 원본 토픽(`/sensing/lidar/points`)은 센서 특성상 NaN 포인트가 포함될 수 있습니다.
전처리 노드(`lidar_preprocessor`)를 거친 `/sensing/lidar/points_filtered` 토픽은
NaN이 제거되어 있으므로 해당 토픽을 감시할 때는 `max_nan_ratio: 0.0` 으로 설정하세요.

---

**Q. 포인트 수 체크를 끄고 싶습니다.**

`min_point_count: 0`, `max_point_count: 0` 으로 설정하면 체크하지 않습니다.

---

**Q. NaN 비율 체크를 끄고 싶습니다.**

`max_nan_ratio: 0.0` 으로 설정하면 체크하지 않습니다.

---

**Q. 발행 주기를 바꾸고 싶습니다.**

`publish_rate` 파라미터(단위: Hz)를 변경하세요.

```yaml
publish_rate: 0.5   # 2초마다 발행
```
