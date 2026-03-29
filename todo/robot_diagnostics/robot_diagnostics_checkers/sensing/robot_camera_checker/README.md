# robot_camera_checker

ROS 2 패키지로, 카메라 토픽을 구독해 **카메라가 정상적으로 동작하고 있는지**를
`/diagnostics` 토픽으로 발행합니다.

카메라가 살아있는지, FPS가 제대로 나오는지, 해상도나 인코딩이 맞는지를
실시간으로 모니터링합니다.

---

## 목차

1. [어떻게 동작하나요?](#1-어떻게-동작하나요)
2. [패키지 구성](#2-패키지-구성)
3. [의존성](#3-의존성)
4. [빌드 방법](#4-빌드-방법)
5. [실행 방법](#5-실행-방법)
6. [카메라 추가하기](#6-카메라-추가하기)
7. [파라미터 설명](#7-파라미터-설명)
8. [진단 결과 확인하기](#8-진단-결과-확인하기)
9. [진단 상태 레벨이란?](#9-진단-상태-레벨이란)
10. [자주 묻는 질문 (FAQ)](#10-자주-묻는-질문-faq)

---

## 1. 어떻게 동작하나요?

이 노드는 카메라 토픽을 **구독(subscribe)** 하고, 주기적으로 아래 항목을 확인합니다.

```
카메라 토픽 ──► camera_checker_node ──► /diagnostics
   (image_raw)         ▲
   (camera_info)       │
                 1초마다 체크
```

| 체크 항목 | 정상 | 문제 시 |
|-----------|------|---------|
| 토픽 수신 여부 | 메시지가 오고 있음 | STALE (회색) |
| 메시지 지연 | `stale_timeout` 이내 | STALE (회색) |
| FPS | `expected_fps`의 80% 이상 | WARN (노랑) / ERROR (빨강) |
| 해상도 | 설정값과 일치 | WARN (노랑) |
| 인코딩 | 설정값과 일치 | WARN (노랑) |

> **FPS 계산 방식**: 최근 2초 내에 수신된 메시지 수로 계산합니다.

---

## 2. 패키지 구성

```
robot_camera_checker/
├── src/
│   └── camera_checker_node.cpp   # 메인 노드 코드
├── config/
│   └── camera_checker.yaml       # 카메라 설정 파일
├── launch/
│   └── camera_checker.launch.py  # 실행 파일
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 3. 의존성

| 패키지 | 용도 |
|--------|------|
| `rclcpp` | ROS 2 C++ 기본 라이브러리 |
| `sensor_msgs` | `Image`, `CameraInfo` 메시지 타입 |
| `diagnostic_msgs` | 진단 메시지 타입 |
| `diagnostic_updater` | 진단 발행 유틸리티 |
| `robot_diagnostics_base` | 베이스 체커 클래스 |

---

## 4. 빌드 방법

```bash
# 워크스페이스 루트로 이동
cd ~/ros2_ws

# 빌드
colcon build --packages-select robot_camera_checker

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
ros2 launch robot_camera_checker camera_checker.launch.py

# 직접 실행
ros2 run robot_camera_checker camera_checker_node \
  --ros-args --params-file <설정파일 경로>
```

---

## 6. 카메라 추가하기

`config/camera_checker.yaml` 파일을 수정하면 됩니다.

### 카메라가 1대일 때

```yaml
camera_checker:
  ros__parameters:
    camera_names: ["front"]

    front:
      image_topic: "/sensing/camera/front/image_raw"
      camera_info_topic: "/sensing/camera/front/camera_info"
      expected_fps: 30.0
      stale_timeout: 2.0
```

### 카메라가 여러 대일 때

`camera_names` 목록에 이름을 추가하고, 같은 이름의 섹션을 추가합니다.

```yaml
camera_checker:
  ros__parameters:
    camera_names: ["front", "rear", "left"]

    front:
      image_topic: "/sensing/camera/front/image_raw"
      camera_info_topic: "/sensing/camera/front/camera_info"
      expected_fps: 30.0
      stale_timeout: 2.0

    rear:
      image_topic: "/sensing/camera/rear/image_raw"
      camera_info_topic: "/sensing/camera/rear/camera_info"
      expected_fps: 15.0
      stale_timeout: 2.0

    left:
      image_topic: "/sensing/camera/left/image_raw"
      camera_info_topic: "/sensing/camera/left/camera_info"
      expected_fps: 15.0
      stale_timeout: 2.0
```

노드 하나가 모든 카메라를 함께 모니터링합니다.

---

## 7. 파라미터 설명

### 공통 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `publish_rate` | `1.0` | 진단 결과 발행 주기 (Hz). 1.0 = 1초마다 |
| `camera_names` | `[]` | 모니터링할 카메라 이름 목록 |

### 카메라별 파라미터 (`<카메라이름>.<파라미터>`)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `image_topic` | `/sensing/camera/<name>/image_raw` | 구독할 이미지 토픽 |
| `camera_info_topic` | `/sensing/camera/<name>/camera_info` | 구독할 카메라 정보 토픽 |
| `expected_fps` | `30.0` | 기대하는 FPS |
| `fps_warn_ratio` | `0.8` | 실제 FPS가 기대 FPS의 이 비율 미만이면 WARN |
| `fps_error_ratio` | `0.5` | 실제 FPS가 기대 FPS의 이 비율 미만이면 ERROR |
| `stale_timeout` | `2.0` | 이 시간(초) 이상 메시지가 없으면 STALE |
| `expected_width` | `0` | 기대 이미지 가로 해상도. `0` = 체크 안 함 |
| `expected_height` | `0` | 기대 이미지 세로 해상도. `0` = 체크 안 함 |
| `expected_encoding` | `""` | 기대 인코딩 (예: `"rgb8"`, `"bgr8"`). `""` = 체크 안 함 |

### FPS 판정 예시

`expected_fps: 30.0`, `fps_warn_ratio: 0.8`, `fps_error_ratio: 0.5` 일 때:

```
actual_fps >= 24.0 (30 × 0.8)  → OK
actual_fps >= 15.0 (30 × 0.5)  → WARN
actual_fps <  15.0              → ERROR
```

---

## 8. 진단 결과 확인하기

### 터미널에서 확인

```bash
# 전체 진단 출력
ros2 topic echo /diagnostics

# 특정 카메라만 필터링
ros2 topic echo /diagnostics | grep -A 10 "camera/front"
```

### 출력 예시 (정상)

```
name:    "/sensor/camera/front"
level:   0                        ← 0 = OK
message: "OK (29.8 fps)"
values:
  - key: "actual_fps"    value: "29.8"
  - key: "expected_fps"  value: "30.0"
  - key: "width"         value: "1920"
  - key: "height"        value: "1080"
  - key: "encoding"      value: "rgb8"
  - key: "camera_info"   value: "OK"
  - key: "last_msg_sec_ago" value: "0.03"
```

### 출력 예시 (STALE — 카메라 연결 끊김)

```
name:    "/sensor/camera/front"
level:   3                        ← 3 = STALE
message: "3.2s 동안 메시지 없음 (timeout=2.0s)"
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

**Q. 카메라가 STALE로 나옵니다.**

카메라 토픽이 발행되지 않고 있다는 뜻입니다. 아래를 확인하세요.

```bash
# 토픽이 실제로 발행되고 있는지 확인
ros2 topic list | grep camera
ros2 topic hz /sensing/camera/front/image_raw
```

`image_topic` 파라미터의 토픽 이름이 실제 카메라가 발행하는 토픽과 일치하는지 확인하세요.

---

**Q. FPS가 기대보다 낮게 나옵니다.**

노드 시작 직후 약 2초간은 FPS 계산 window가 채워지지 않아 낮게 나올 수 있습니다.
2초 후에도 계속 낮으면 카메라나 네트워크 문제를 확인하세요.

---

**Q. camera_info가 "없음"으로 나옵니다.**

`camera_info_topic`으로 메시지가 오지 않고 있다는 뜻입니다.
카메라 드라이버가 `camera_info`를 발행하는지 확인하세요.
camera_info가 불필요하다면 무시해도 됩니다 (진단 레벨에 영향을 주지 않습니다).

---

**Q. 해상도/인코딩 체크를 끄고 싶습니다.**

`expected_width: 0`, `expected_height: 0`, `expected_encoding: ""`으로 설정하면
해당 항목은 체크하지 않습니다 (기본값이 이미 이렇게 설정되어 있습니다).
