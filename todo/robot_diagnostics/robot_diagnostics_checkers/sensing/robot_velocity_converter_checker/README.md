# robot_velocity_converter_checker

`PlatformVelocityConverterNode` 의 입·출력 토픽을 감시하고, 이상이 감지되면 `/diagnostics` 토픽으로 경보를 발행하는 진단 패키지입니다.

> **핵심 기능: Silent Drop 감지**
> `PlatformVelocityConverterNode` 는 `require_imu=true`(기본값) 상태에서 IMU 가 오지 않으면
> velocity 데이터를 `DEBUG` 레벨 로그만 남기고 **조용히 출력을 차단**합니다.
> 이 패키지는 그 상황을 `ERROR` 로 외부에 노출합니다.

---

## 이 패키지가 하는 일

3개의 토픽을 동시에 감시합니다.

```
[/platform/status/velocity]  ─┐
                               ├─→ PlatformVelocityConverterNode ─→ [출력 토픽]
[/sensing/imu/data]           ─┘
         ↑
    IMU 없으면 velocity 드롭 (Silent Drop)
```

| 검사 항목 | 정상 | 경보(WARN) | 오류(ERROR) | 데이터 없음(STALE) |
|---|---|---|---|---|
| **velocity 입력** | 수신 중 | — | timeout / 미수신 | — |
| **IMU 입력** | 수신 중 | timeout / 미수신 | — | — |
| **Silent Drop** | — | — | velocity OK + 출력 없음 | — |
| **출력 속도** | ~10 Hz | 7 Hz 미만 | 4 Hz 미만 | — |
| **출력 타임아웃** | 1초 이내 | — | — | 미수신 / 1초 초과 |

검사 결과는 `/diagnostics` 토픽에 아래 이름으로 발행됩니다.
```
velocity_converter_checker: /sensing/velocity_converter
```

---

## 패키지 구조

```
robot_velocity_converter_checker/
├── src/
│   ├── velocity_converter_checker_node.cpp      # 진단 체커 메인 코드
│   └── velocity_converter_dummy_publisher.cpp   # 테스트용 더미 발행자
├── config/
│   └── velocity_converter_checker.yaml          # 파라미터 설정 파일
├── launch/
│   ├── velocity_converter_checker.launch.py     # 운영 환경 실행 파일
│   └── velocity_converter_test.launch.py        # 테스트 실행 파일 (하드웨어 불필요)
├── CMakeLists.txt
└── package.xml
```

---

## 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_velocity_converter_checker
source install/setup.bash
```

> **처음 빌드할 때** `robot_diagnostics_base` 도 함께 빌드해야 합니다.
> ```bash
> colcon build --packages-select robot_diagnostics_base robot_velocity_converter_checker
> source install/setup.bash
> ```

---

## 실행 방법

### 실제 PlatformVelocityConverterNode 가 실행 중인 경우

```bash
ros2 launch robot_velocity_converter_checker velocity_converter_checker.launch.py
```

### 하드웨어 없이 테스트하는 경우

더미 발행자가 8초 주기로 4가지 상황을 순서대로 시뮬레이션합니다.

```bash
ros2 launch robot_velocity_converter_checker velocity_converter_test.launch.py
```

| 더미 Phase | 발행 토픽 | 예상 진단 결과 |
|---|---|---|
| Phase 0 (0~8s) | velocity + IMU + output | OK |
| Phase 1 (8~16s) | velocity + output (IMU 없음) | WARN |
| Phase 2 (16~24s) | velocity 만 (output 없음) | **ERROR** (Silent Drop) |
| Phase 3 (24~32s) | 없음 | STALE |

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
    name: "velocity_converter_checker: /sensing/velocity_converter"
    message: "OK (10.2 Hz)"
    values:
      - key: velocity_input  value: "OK"
      - key: imu_input       value: "OK"
      - key: output          value: "OK"
      - key: output_hz       value: "10.2"
      - key: vel_age_sec     value: "0.08"
      - key: imu_age_sec     value: "0.01"
      - key: out_age_sec     value: "0.08"
```

출력 예시 (Silent Drop 감지):
```
status:
  - level: 2          # ERROR
    name: "velocity_converter_checker: /sensing/velocity_converter"
    message: "velocity 수신 중이나 출력 차단 (IMU 대기 드롭 의심)"
    values:
      - key: velocity_input  value: "OK"
      - key: imu_input       value: "STALE"
      - key: output          value: "STALE"
```

출력 예시 (IMU timeout):
```
status:
  - level: 1          # WARN
    name: "velocity_converter_checker: /sensing/velocity_converter"
    message: "IMU 입력 timeout (angular velocity 부정확)"
    values:
      - key: imu_input   value: "STALE"
      - key: imu_age_sec value: "3.21"
```

### rqt_runtime_monitor (GUI)

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

## 설정 파일 수정 방법

`config/velocity_converter_checker.yaml` 을 수정합니다.

```yaml
velocity_converter_checker:
  ros__parameters:
    publish_rate: 1.0

    # 감시할 토픽 이름 (PlatformVelocityConverterNode 파라미터와 일치해야 함)
    velocity_topic: "/platform/status/velocity"
    imu_topic:      "/sensing/imu/data"
    output_topic:   "/sensing/platform_velocity_converter/twist_with_covariance"

    velocity_stale_timeout:  1.0   # velocity 입력 timeout (초)
    imu_stale_timeout:       1.0   # IMU 입력 timeout (초)
    output_stale_timeout:    1.0   # 출력 timeout (초) — Silent Drop 감지 기준

    expected_output_hz:  10.0      # 정상 출력 속도 (Hz)
    hz_warn_ratio:       0.7       # 이 비율 미만이면 WARN (10 × 0.7 = 7 Hz)
    hz_error_ratio:      0.4       # 이 비율 미만이면 ERROR (10 × 0.4 = 4 Hz)
```

---

## 자주 묻는 질문

**Q. "velocity 수신 중이나 출력 차단" 이 계속 뜹니다.**

`PlatformVelocityConverterNode` 의 `require_imu` 파라미터가 `true` 인데 IMU 토픽이 오지 않는 상태입니다.
```bash
ros2 topic hz /sensing/imu/data      # IMU 발행 여부 확인
ros2 param get /platform_velocity_converter require_imu  # 파라미터 확인
```

**Q. IMU WARN 이 자주 뜨는데 출력은 정상입니다.**

`PlatformVelocityConverterNode` 가 `require_imu=false` 로 설정되어 IMU 없이도 동작 중입니다.
이 경우 IMU stale timeout 을 늘리거나, 필요 없다면 `imu_stale_timeout` 을 크게 설정해 사실상 비활성화하세요.

**Q. STALE 이 계속 나옵니다.**

`PlatformVelocityConverterNode` 가 실행 중인지 확인하세요.
```bash
ros2 node list | grep platform
ros2 topic list | grep twist_with_covariance
```

---

## 관련 패키지

| 패키지 | 역할 |
|---|---|
| `robot_diagnostics_base` | 모든 체커가 상속하는 베이스 클래스 |
| `robot_diagnostics_agg` | 각 체커의 진단 결과를 수집·집계 |
| `robot_radar_checker` | 레이다 센서 진단 체커 (구조 참고용) |
