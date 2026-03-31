# robot_cost_grid_checker

`LidarCostGridNode` 가 발행하는 `nav_msgs/OccupancyGrid` 를 감시하고, 이상이 감지되면 `/diagnostics` 토픽으로 경보를 발행하는 진단 패키지입니다.

> **이 패키지는 센서 자체가 아닌 처리 파이프라인 출력을 감시합니다.**
> 원시 LiDAR 센서 감시는 `robot_lidar_checker` 를 사용하세요.

---

## 이 패키지가 하는 일

| 검사 항목 | 정상 | 경보(WARN) | 오류(ERROR) | 데이터 없음(STALE) |
|---|---|---|---|---|
| **토픽 수신 여부** | 메시지 수신 중 | — | — | 메시지가 한 번도 안 옴 |
| **타임아웃** | 2초 이내 수신 | — | — | 2초 이상 메시지 없음 |
| **발행 속도** | ~10 Hz | 7 Hz 미만 | 4 Hz 미만 | — |
| **unknown 비율** | 낮음 | ≥ 90% | 100% | — |

> **unknown 비율이란?**
> OccupancyGrid 의 각 셀은 `free(0)`, `occupied(1~100)`, `unknown(-1)` 중 하나입니다.
> `LidarCostGridNode` 는 입력 PointCloud2 가 없거나 TF 변환에 실패하면 모든 셀을 `unknown(-1)` 으로 초기화된 상태로 발행합니다.
> → unknown 비율이 높다면 **파이프라인 입력 문제** 를 의심해야 합니다.

검사 결과는 `/diagnostics` 토픽에 아래 이름으로 발행됩니다.
```
cost_grid_checker: /perception/lidar/cost_grid
```

---

## 패키지 구조

```
robot_cost_grid_checker/
├── src/
│   ├── cost_grid_checker_node.cpp      # 진단 체커 메인 코드
│   └── cost_grid_dummy_publisher.cpp   # 테스트용 가짜 그리드 발행자
├── config/
│   └── cost_grid_checker.yaml          # 파라미터 설정 파일
├── launch/
│   ├── cost_grid_checker.launch.py     # 운영 환경 실행 파일
│   └── cost_grid_test.launch.py        # 테스트 실행 파일 (하드웨어 불필요)
├── CMakeLists.txt
└── package.xml
```

---

## 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_cost_grid_checker
source install/setup.bash
```

> **처음 빌드할 때** `robot_diagnostics_base` 도 함께 빌드해야 합니다.
> ```bash
> colcon build --packages-select robot_diagnostics_base robot_cost_grid_checker
> source install/setup.bash
> ```

---

## 실행 방법

### 실제 LidarCostGridNode 가 실행 중인 경우

```bash
ros2 launch robot_cost_grid_checker cost_grid_checker.launch.py
```

### 하드웨어 없이 테스트하는 경우

더미 발행자가 10초 주기로 두 가지 상황을 번갈아 시뮬레이션합니다.

```bash
ros2 launch robot_cost_grid_checker cost_grid_test.launch.py
```

| 더미 Phase | 발행 내용 | 예상 진단 결과 |
|---|---|---|
| Phase 0 (0~10s) | 장애물 포함 정상 그리드 | OK |
| Phase 1 (10~20s) | 전체 unknown(-1) 그리드 | ERROR |

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
    name: "cost_grid_checker: /perception/lidar/cost_grid"
    message: "OK (10.1 Hz, unknown=2%)"
    values:
      - key: actual_hz          value: "10.1"
      - key: expected_hz        value: "10.0"
      - key: unknown_ratio_pct  value: "2.0"
      - key: last_msg_sec_ago   value: "0.10"
```

출력 예시 (입력 없음):
```
status:
  - level: 2          # ERROR
    name: "cost_grid_checker: /perception/lidar/cost_grid"
    message: "그리드 전체 unknown (입력 없음 또는 TF 실패)"
    values:
      - key: unknown_ratio_pct  value: "100.0"
```

### rqt_runtime_monitor (GUI)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 설정 파일 수정 방법

`config/cost_grid_checker.yaml` 을 수정합니다.

```yaml
cost_grid_checker:
  ros__parameters:
    publish_rate: 1.0              # 진단 발행 주기 (Hz)
    output_topic: "/sensing/lidar/near_cost_grid"  # 감시할 토픽
    expected_hz:         10.0      # 정상 발행 속도 (Hz)
    hz_warn_ratio:       0.7       # 이 비율 미만이면 WARN (10 × 0.7 = 7 Hz)
    hz_error_ratio:      0.4       # 이 비율 미만이면 ERROR (10 × 0.4 = 4 Hz)
    stale_timeout:       2.0       # 이 시간(초) 이상 무응답이면 STALE
    unknown_ratio_warn:  0.9       # unknown 셀 비율 90% 이상이면 WARN
    unknown_ratio_error: 1.0       # unknown 셀 비율 100% 이면 ERROR
```

---

## 자주 묻는 질문

**Q. STALE 이 계속 나옵니다.**

`LidarCostGridNode` 가 실행 중인지, 토픽 이름이 yaml 의 `output_topic` 과 일치하는지 확인하세요.
```bash
ros2 topic list | grep cost_grid
```

**Q. unknown 비율 경보가 너무 자주 뜹니다.**

LiDAR 입력 토픽(`/perception/obstacles`) 이 오고 있는지, TF 가 정상인지 먼저 확인하세요.
```bash
ros2 topic hz /perception/obstacles
ros2 run tf2_tools view_frames
```

**Q. unknown_ratio_error 를 비활성화하고 싶습니다.**

`unknown_ratio_error: 1.01` 처럼 1.0 을 초과하는 값으로 설정하면 사실상 비활성화됩니다.

---

## 관련 패키지

| 패키지 | 역할 |
|---|---|
| `robot_diagnostics_base` | 모든 체커가 상속하는 베이스 클래스 |
| `robot_diagnostics_agg` | 각 체커의 진단 결과를 수집·집계 |
| `robot_lidar_checker` | LiDAR 원시 센서 진단 체커 |
