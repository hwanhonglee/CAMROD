# robot_perception_obstacle_checker

ROS 2 패키지로, 장애물 감지 파이프라인의 출력 토픽들을 통합 모니터링합니다.

| 소스 | 토픽 | 타입 |
|------|------|------|
| `camera_detections` | `/perception/camera/detections_2d` | `vision_msgs/Detection2DArray` |
| `fused_obstacles` | `/perception/obstacles` | `sensor_msgs/PointCloud2` |

각 소스별로 **발행 속도(Hz)**, **메시지 지연(staleness)**, **출력 수(count)** 를 독립적으로
`/diagnostics` 토픽으로 발행합니다.

> **전제 조건**: 상위 입력 토픽이 먼저 정상이어야 합니다.
> - LiDAR 입력: [`robot_lidar_checker`](../../sensing/robot_lidar_checker/README.md) (`filtered` 항목)
> - 카메라 입력: [`robot_camera_checker`](../../sensing/robot_camera_checker/README.md)

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [소스 추가하기](#6-소스-추가하기)
7. [파라미터 설명](#7-파라미터-설명)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)
11. [고려했으나 제외된 항목](#11-고려했으나-제외된-항목)
12. [운용 가이드](#12-운용-가이드)

---

## 1. 어떻게 동작하나요?

```
/perception/camera/detections_2d ──┐
  (vision_msgs/Detection2DArray)   ├──► perception_obstacle_checker_node ──► /diagnostics
/perception/obstacles ─────────────┘                   ▲
  (sensor_msgs/PointCloud2)                       1초마다 체크
```

소스마다 독립적으로 아래 항목을 확인합니다.

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| 발행 속도 (Hz) | `expected_hz`의 80% 이상 | WARN (노랑) / ERROR (빨강) |
| 출력 수 하한 | `min_count` 이상 | ERROR (빨강) |
| 출력 수 상한 | `max_count` 이하 | WARN (노랑) |

> **Hz 계산 방식**: 최근 2초 내에 수신된 메시지 수로 계산합니다.

---

## 2. 패키지 구성

```
robot_perception_obstacle_checker/
├── src/
│   ├── perception_obstacle_checker_node.cpp    # 메인 노드 코드
│   └── perception_obstacle_dummy_publisher.cpp # 테스트용 더미 퍼블리셔
├── config/
│   └── perception_obstacle_checker.yaml        # 설정 파일
├── launch/
│   ├── perception_obstacle_checker.launch.py   # 운영용 실행 파일
│   └── perception_obstacle_test.launch.py      # 테스트용 실행 파일
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
| `vision_msgs` | `Detection2DArray` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_perception_obstacle_checker
source install/setup.bash
```

---

## 5. 실행 방법

```bash
# 운영용 (checker만)
ros2 launch robot_perception_obstacle_checker perception_obstacle_checker.launch.py

# 테스트용 (checker + 더미 퍼블리셔)
ros2 launch robot_perception_obstacle_checker perception_obstacle_test.launch.py
ros2 launch robot_perception_obstacle_checker perception_obstacle_test.launch.py scenario:=stale_det
ros2 launch robot_perception_obstacle_checker perception_obstacle_test.launch.py scenario:=stale_pc2
ros2 launch robot_perception_obstacle_checker perception_obstacle_test.launch.py scenario:=stale_both
ros2 launch robot_perception_obstacle_checker perception_obstacle_test.launch.py scenario:=few_points
```

### 전체 시나리오 목록

| 시나리오 | 동작 | 예상 결과 |
|----------|------|-----------|
| `ok` | 두 토픽 모두 정상 발행 | 둘 다 OK |
| `hz_warn` | 두 토픽 모두 Hz 저하 | 둘 다 WARN |
| `hz_error` | 두 토픽 모두 Hz 심각 저하 | 둘 다 ERROR |
| `stale_det` | Detection2DArray 발행 중단 | camera_detections STALE, fused_obstacles OK |
| `stale_pc2` | PointCloud2 발행 중단 | camera_detections OK, fused_obstacles STALE |
| `stale_both` | 둘 다 발행 중단 | 둘 다 STALE |
| `few_points` | PointCloud2 포인트 수 부족 | fused_obstacles ERROR |

---

## 6. 소스 추가하기

`config/perception_obstacle_checker.yaml` 의 `obstacle_names` 목록에 이름을 추가하고,
같은 이름의 섹션을 추가합니다.

```yaml
obstacle_names:
  - "camera_detections"
  - "fused_obstacles"
  - "radar_obstacles"       # 신규 추가 예시

radar_obstacles:
  topic: "/perception/radar/obstacles"
  type: "PointCloud2"
  expected_hz: 20.0
  stale_timeout: 2.0
  min_count: 0
  max_count: 0
```

지원 타입: `"PointCloud2"`, `"Detection2DArray"`

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 발행 주기 (Hz) |
| `obstacle_names` | `[]` | 모니터링할 소스 이름 목록 |

### 소스별 파라미터 (`<소스이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `topic` | — | 구독할 토픽 이름 |
| `type` | `"PointCloud2"` | 메시지 타입: `"PointCloud2"` 또는 `"Detection2DArray"` |
| `expected_hz` | `10.0` | 기대 발행 속도 (Hz) |
| `hz_warn_ratio` | `0.8` | 실제 Hz가 기대 Hz의 이 비율 미만이면 WARN |
| `hz_error_ratio` | `0.5` | 실제 Hz가 기대 Hz의 이 비율 미만이면 ERROR |
| `stale_timeout` | `2.0` | 이 시간(초) 이상 메시지가 없으면 STALE |
| `min_count` | `0` | 최소 포인트 수 또는 검출 수. `0` = 체크 안 함 |
| `max_count` | `0` | 최대 포인트 수 또는 검출 수. `0` = 체크 안 함 |

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
ros2 topic echo /diagnostics | grep -A 8 "perception/obstacles"
```

### 출력 예시 (정상)

```
name:    "/perception/obstacles/camera_detections"
level:   0                         ← OK
message: "OK (10.1 Hz, 3 det)"
values:
  - key: "actual_hz"        value: "10.1"
  - key: "expected_hz"      value: "10.0"
  - key: "count"            value: "3"
  - key: "last_msg_sec_ago" value: "0.05"

name:    "/perception/obstacles/fused_obstacles"
level:   0                         ← OK
message: "OK (10.0 Hz, 250 pts)"
values:
  - key: "actual_hz"        value: "10.0"
  - key: "expected_hz"      value: "10.0"
  - key: "count"            value: "250"
  - key: "last_msg_sec_ago" value: "0.03"
```

### 출력 예시 (STALE — detection 노드 다운)

```
name:    "/perception/obstacles/camera_detections"
level:   3                         ← STALE
message: "3.1s 동안 메시지 없음 (timeout=2.0s)"
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
| WARN | 1 | 노랑 | 발행 속도 저하 또는 출력 수 초과 |
| ERROR | 2 | 빨강 | 발행 속도 심각 저하 또는 출력 수 부족 |
| STALE | 3 | 회색 | 토픽 미수신 또는 timeout 초과 |

---

## 10. 자주 묻는 질문 (FAQ)

**Q. camera_detections 는 STALE 인데 fused_obstacles 는 OK 입니다.**

`obstacle_fusion_node` 가 카메라 검출 없이도 LiDAR 만으로 동작하는 경우입니다.
카메라 검출 노드를 별도로 확인하세요.

---

**Q. fused_obstacles 포인트 수가 가끔 0 입니다.**

장애물이 없는 환경에서는 빈 PointCloud2 가 발행됩니다.
`min_count` 체크가 필요 없다면 `0` 으로 유지하세요.

---

**Q. 새 소스를 추가했는데 STALE 이 납니다.**

`obstacle_names` 에 이름을 추가하고 같은 이름의 섹션을 작성했는지,
`topic` 이름이 실제 발행 토픽과 일치하는지 확인하세요.

```bash
ros2 topic list | grep perception
```

---

## 11. 고려했으나 제외된 항목

| 항목 | 제외 이유 |
|------|-----------|
| **NaN 비율 체크 (fused_obstacles)** | `obstacle_fusion_node` 는 LiDAR filtered 포인트(NaN 제거됨) 기반으로 출력하므로 NaN 없음 |
| **bounding box 크기 유효성 체크** | 환경·객체에 따라 정상 범위가 크게 달라 일반화 불가 |
| **검출 클래스 분포 체크** | 운용 환경에 따라 기대 클래스가 다르며 동적 변화가 큼 |

---

## 12. 운용 가이드

### Perception Checker 의존관계

```
[robot_lidar_checker (filtered)]   ← LiDAR 전처리 출력 확인
[robot_camera_checker]             ← 카메라 센서 원본 확인
  │
  └──► [robot_perception_obstacle_checker]  ← 이 패키지
            camera_detections stale → fusion 카메라 입력 없음
            fused_obstacles stale   → planning 장애물 입력 없음
```

### diagnostic_aggregator 연동

```yaml
# config/diagnostic_aggregator.yaml
analyzers:
  perception:
    type: diagnostic_aggregator/GenericAnalyzer
    path: Perception
    contains:
      - '/perception/obstacles/'
```

### ERROR 발생 시 체크 순서

```
camera_detections ERROR/STALE
  1. camera_checker 확인 → 카메라 토픽 정상?
  2. ros2 topic hz /perception/camera/detections_2d
  3. 검출 모델 노드 확인 → ros2 node list | grep detection

fused_obstacles ERROR/STALE
  1. lidar_checker (filtered) 확인 → LiDAR 전처리 정상?
  2. camera_detections 확인 → 검출 정상?
  3. ros2 node list | grep obstacle_fusion → 노드 살아있음?
  4. obstacle_fusion 노드 로그 확인 → TF 문제 없음?
```
