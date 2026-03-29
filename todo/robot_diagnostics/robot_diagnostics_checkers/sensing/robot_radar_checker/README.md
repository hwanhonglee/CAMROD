# robot_radar_checker

SEN0592 레이다 센서의 ROS 2 토픽을 감시하고, 이상이 감지되면 `/diagnostics` 토픽으로 경보를 발행하는 진단 패키지입니다.

---

## 이 패키지가 하는 일

레이다 센서 노드(`sen0592_radar_node`)가 발행하는 `sensor_msgs/Range` 토픽을 구독하여 아래 항목을 주기적으로 검사합니다.

| 검사 항목 | 정상 | 경보(WARN) | 오류(ERROR) | 데이터 없음(STALE) |
|---|---|---|---|---|
| **토픽 수신 여부** | 메시지 수신 중 | — | — | 메시지가 한 번도 안 옴 |
| **타임아웃** | 1초 이내 수신 | — | — | 1초 이상 메시지 없음 |
| **발행 속도** | ~16 Hz 유지 | 11 Hz 미만 | 6.4 Hz 미만 | — |
| **쓰레기값** | — | — | NaN / Inf / 음수 | — |
| **측정 범위 이탈** | 0.02 ~ 4.5 m | 범위 벗어남 | — | — |
| **최솟값 고착** | 정상 범위 | ≤ stuck_min_warn_m | ≤ stuck_min_error_m | — |
| **최댓값 고착** | 정상 범위 | ≥ stuck_max_warn_m | ≥ stuck_max_error_m | — |

> **최솟값 고착**: range 가 센서 최솟값 근처에 고착 → 센서 전면이 물리적으로 막혔거나 충격으로 인한 고장 의심
> **최댓값 고착**: range 가 센서 최댓값 근처에 고착 → 센서가 아무것도 감지하지 못하는 무감지 상태, 케이블 불량 의심
> **쓰레기값**: NaN / Inf / 음수 → 드라이버 버그 또는 통신 오류. 파라미터 없이 항상 ERROR

검사 결과는 `/diagnostics` 토픽에 아래 이름으로 발행됩니다.

```
radar_checker: /sensor/radar/REAR
radar_checker: /sensor/radar/LEFT2
radar_checker: /sensor/radar/LEFT1
radar_checker: /sensor/radar/RIGHT2
radar_checker: /sensor/radar/RIGHT1
radar_checker: /sensor/radar/FRONT
```

---

## 패키지 구조

```
robot_radar_checker/
├── src/
│   ├── radar_checker_node.cpp      # 진단 체커 메인 코드
│   └── radar_dummy_publisher.cpp   # 테스트용 가짜 레이다 발행자
├── config/
│   └── radar_checker.yaml          # 파라미터 설정 파일
├── launch/
│   ├── radar_checker.launch.py     # 운영 환경 실행 파일
│   └── radar_test.launch.py        # 테스트 실행 파일 (하드웨어 불필요)
├── CMakeLists.txt
└── package.xml
```

---

## 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_radar_checker
source install/setup.bash
```

> **처음 빌드할 때** `robot_diagnostics_base` 도 함께 빌드해야 합니다.
> ```bash
> colcon build --packages-select robot_diagnostics_base robot_radar_checker
> source install/setup.bash
> ```

---

## 실행 방법

### 실제 레이다 하드웨어가 연결된 경우

`sen0592_radar_node` 가 먼저 실행 중이어야 합니다.

```bash
ros2 launch robot_radar_checker radar_checker.launch.py
```

### 하드웨어 없이 테스트하는 경우

더미 발행자가 가짜 레이다 데이터를 만들어 줍니다.

```bash
ros2 launch robot_radar_checker radar_test.launch.py
```

---

## 진단 결과 확인 방법

### 터미널에서 원시 데이터 보기

```bash
ros2 topic echo /diagnostics
```

출력 예시 (정상):
```
status:
  - level: 0          # 0=OK, 1=WARN, 2=ERROR, 3=STALE
    name: "radar_checker: /sensor/radar/FRONT"
    message: "OK (16.2 Hz, 1.250 m)"
    values:
      - key: actual_hz        value: "16.2"
      - key: expected_hz      value: "16.0"
      - key: range_m          value: "1.250"
      - key: range_valid      value: "true"
      - key: last_msg_sec_ago value: "0.06"
```

출력 예시 (전방 차폐 의심):
```
status:
  - level: 1          # WARN
    name: "radar_checker: /sensor/radar/REAR"
    message: "센서 전방 차폐 의심 (WARN)"
    values:
      - key: range_m      value: "0.041"
      - key: range_valid  value: "true"
```

출력 예시 (쓰레기값):
```
status:
  - level: 2          # ERROR
    name: "radar_checker: /sensor/radar/LEFT1"
    message: "range 쓰레기값 (NaN/Inf/음수)"
    values:
      - key: range_valid  value: "false(NaN/Inf)"
```

### rqt_runtime_monitor (GUI)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

모든 센서 상태를 신호등(초록/노랑/빨강) 형태로 한눈에 볼 수 있습니다.

---

## 설정 파일 수정 방법

`config/radar_checker.yaml` 을 수정하면 각 센서의 임계값을 바꿀 수 있습니다.

```yaml
radar_checker:
  ros__parameters:
    publish_rate: 1.0              # 진단 결과 발행 주기 (Hz)
    radar_names: ["REAR", "FRONT"] # 모니터링할 센서 이름 목록

    FRONT:
      topic: "/sensing/radar/front/range"  # 구독할 ROS 토픽
      expected_hz:        16.0   # 정상 발행 속도 (Hz)
      hz_warn_ratio:      0.7    # 이 비율 미만이면 WARN  (16 × 0.7 = 11.2 Hz)
      hz_error_ratio:     0.4    # 이 비율 미만이면 ERROR (16 × 0.4 =  6.4 Hz)
      stale_timeout:      1.0    # 이 시간(초) 이상 무응답이면 STALE
      min_range_m:        0.02   # 센서 측정 하한 (m)
      max_range_m:        4.5    # 센서 측정 상한 (m)
      stuck_min_warn_m:   0.05   # 0.0 = 비활성화 / range ≤ 이 값이면 WARN  (전방 차폐 의심)
      stuck_min_error_m:  0.03   # 0.0 = 비활성화 / range ≤ 이 값이면 ERROR (전방 차폐 확실)
      stuck_max_warn_m:   4.4    # 0.0 = 비활성화 / range ≥ 이 값이면 WARN  (무감지 의심)
      stuck_max_error_m:  0.0    # 0.0 = 비활성화 / range ≥ 이 값이면 ERROR (무감지 확실)
```

> **센서 추가 방법**: `radar_names` 목록에 이름을 추가하고, 동일한 형식으로 해당 이름의 파라미터 블록을 추가합니다.
> 토픽 이름 기본값은 이름을 소문자로 변환하여 자동 생성됩니다.
> 예) `"SIDE_LEFT"` → `/sensing/radar/side_left/range`

---

## 자주 묻는 질문

**Q. 빌드 후 실행해도 아무 진단도 안 나옵니다.**

`source install/setup.bash` 를 실행했는지 확인하세요. 빌드 후 새 터미널을 열었다면 반드시 다시 실행해야 합니다.

**Q. STALE 상태가 계속 나옵니다.**

레이다 센서 노드(`sen0592_radar_node`)가 실행 중인지, 토픽 이름이 `radar_checker.yaml` 의 `topic` 값과 일치하는지 확인하세요.

```bash
ros2 topic list | grep radar   # 실제 발행 중인 토픽 확인
```

**Q. stuck_min / stuck_max 체크를 끄고 싶습니다.**

해당 파라미터를 `0.0` 으로 설정하면 해당 체크가 비활성화됩니다.

**Q. 발행 속도 경보가 너무 자주 뜹니다.**

`hz_warn_ratio` 와 `hz_error_ratio` 값을 낮추거나, `expected_hz` 를 실제 측정 속도에 맞게 조정하세요.

---

## 관련 패키지

| 패키지 | 역할 |
|---|---|
| `robot_diagnostics_base` | 모든 체커가 상속하는 베이스 클래스 |
| `robot_diagnostics_agg` | 각 체커의 진단 결과를 수집·집계 |
| `robot_lidar_checker` | LiDAR PointCloud2 진단 체커 (구조 참고용) |
