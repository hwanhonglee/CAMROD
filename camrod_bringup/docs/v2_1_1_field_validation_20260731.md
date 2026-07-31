# CAMROD v2.1.1 실차 정지 검증 — 2026-07-31

<!-- HH_260731 - Preserve the first post-v2.1.1 physical, no-motion evidence
     without promoting incomplete driving/radar-clear tests to FIELD-PASS. -->

## 1. 시험 경계

- 기준: `develop`, HEAD `09a7e1ca`, dirty working tree
- 진입점: `camrod_bringup/bringup.launch.py`, `sim:=false`
- 차량: 실제 센서/CAN이 연결된 실차
- 안전: motion engage와 goal 주행을 수행하지 않았고 물리 E-stop 상태를
  유지했다.
- 위치: lanelet 밖 정지 위치
- 사용자 환경 조건: REAR와 LEFT2 가까이에 실제 물체가 있었다.

이 문서의 FIELD-PASS는 수행한 개별 항목에만 적용한다. goal/path/cmd_vel
실주행, lanelet 접촉 복구, lateral/crab 수렴은 수행하지 않았다.

## 2. 로봇 경계

| 경계 | `robot_base_link` 기준 X | Y | 전체 크기 | 소비자 |
|---|---:|---:|---:|---|
| 실제 차체 | -0.29023 ~ +1.20137 m | -0.53495 ~ +0.53505 m | 1.49160 × 1.07000 m | URDF collision, RViz 청록색 footprint |
| planning/safety | -0.39023 ~ +1.30137 m | -0.63495 ~ +0.63505 m | 1.69160 × 1.27000 m | Nav2 footprint, safety gate, 노란 boundary |

planning/safety 경계는 실제 차체 사방에 0.10 m를 더한 값이다.
`/platform/robot/planning_boundary`가 5 Hz transient-local polygon으로
발행되며, cmd_vel safety gate는 `robot_base_link` 한 점이 아니라 polygon의
꼭짓점·변·내부 cell 전체를 raw lanelet grid와 비교한다.

실행 중 다음 값이 모두 같음을 확인했다.

- robot visualization body extents와 `planning_boundary_margin=0.1`
- cmd_vel safety gate fallback extents
- Nav2 local/global costmap footprint
- source, bringup, install의 boundary 관련 YAML

현재 정지 위치에서는 base cell, planning boundary 4개 꼭짓점, 외곽선
54개 표본과 내부 33개 cell이 모두 lanelet cost 100이었다. 따라서 현재
위치를 off-lane으로 차단하는 것은 정상이다. 정상 corridor 내부에서의
주행 허용과 한쪽 경계 접촉 시 지속 정지는 별도 현장 시험이 필요하다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_144047/boundary_live_params.txt`
- `/home/nvidia/camrod_field_logs/20260731_144047/boundary_current_offlane_probe.txt`

## 3. FIELD-PASS

### 3.1 Radar 전체 비활성화

`enable_radar:=false` 실제 bringup을 600.063초 연속 계측했다.

| 항목 | 결과 |
|---|---:|
| `sen0592_radar_node` | 없음 |
| `radar_dummy_publisher` | 있음 |
| 각 채널 range | 5,986개 |
| FRONT1/FRONT2 | 전부 1.501 m |
| LEFT1/LEFT2/RIGHT1/RIGHT2 | 전부 0.801 m |
| REAR | 전부 0.501 m |
| channel/global dummy | 전부 true |
| obstacle evidence | clear 6,000 / active 0 |
| radar grid | 5,976개, max cost 0, cost>=85 cell 0 |
| gate radar/cost status | 0 |
| `/rosout` radar cost stop | 0 |

TODO 1의 10분 합격 조건을 만족했다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_144047/radar_off/radar_off_600s_summary_retry.txt`
- `/home/nvidia/camrod_field_logs/20260731_144047/bringup.log`

### 3.2 전방 카메라와 YOLO

물리 `/dev/video0` 경로를 300초 연속 검증했다.

- 물리 compressed publisher 1, front dummy publisher 0
- 2,750 frame, 9.167 Hz
- JPEG payload 504,095~566,900 byte
- decode 성공 2,750/2,750
- empty/decode failure/shape mismatch 0
- CameraInfo 2,682개
- detection 약 3.275 Hz
- OpenCV 4.5.4와 4.8.0이 함께 로드된 상태에서 camera/YOLO
  SIGABRT·restart 0

TODO 2의 5분 합격 조건을 만족했다.
현장 helper가 설치한 payload probe를 실행할 수 있도록
`camera_payload_probe.py` mode를 100755로 바로잡았다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_140639/camera_yolo_300s_retry.log`

## 4. FIELD-FAIL 또는 부분 확인

### 4.1 후방 카메라

측정 subscriber 부하를 줄인 120.827초 재측정에서도 목표를 만족하지 못했다.

| 스트림 | frame | rate | max gap |
|---|---:|---:|---:|
| raw | 439 | 3.633 Hz | 0.717 s |
| compressed | 188 | 1.556 Hz | 1.263 s |
| camera_info | 438 | 3.625 Hz | 0.721 s |

raw는 1920×1080 `bgr8`, compressed payload는 284,923~291,016 byte였다.
빈 frame이나 encoding 오진이 아니라 CPU 포화 또는 camera pipeline
throughput 문제다. 진단은 `FPS low`/`FPS critically low`로 올바르게
표시됐다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_140639/rear_camera_low_overhead/summary.txt`

### 4.2 Radar ON

시험 당시 bringup 설정은
`sensor_enabled=[true,true,true,false,true,false,true]`였다.
LEFT2와 RIGHT2는 DUMMY이므로 7채널 물리 시험이 아니다.

- 물리 REAR는 가까운 실제 물체를 0.096~0.107 m로 봤지만 기존 REAR
  fixed-return exclusion 0.090~0.190 m 안에 전부 들어가 cost에서 제거됐다.
- LEFT2는 비활성이라 주변 물체를 측정할 수 없었다.
- 312.9초 DB에서 FRONT1 0.132 m 3 frame과 RIGHT1 0.795~0.800 m
  11 frame이 active evidence였다.
- evidence 3,127개 중 active는 14개였다.

REAR 값은 clear-area self echo로 확정할 수 없다. 주변 물체를 제거한 반복
측정과 같은 거리의 실제 장애물이 exclusion에 가려지지 않는지 확인해야 한다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_140639/radar_on/radar_on_bag/radar_on_bag_0.db3`
- `/home/nvidia/camrod_field_logs/20260731_140639/bringup.log`

### 4.3 GNSS와 IMU

물리 포트와 owner는 다음과 같았다.

| 역할 | 포트 | owner |
|---|---|---|
| u-blox rover | `/dev/ttyACM0` | `ublox_gps_node` 1개 |
| MicroStrain CV7 | by-id → `/dev/ttyACM1` | IMU driver 1개 |
| moving-base RTCM | FTDI DN03DF8V by-id → `/dev/ttyUSB0` | writer 1개 |

120.807초 GNSS 품질 측정:

- NAV-PVT 121개, 1.002 Hz, flags 67만 121회
- NAV-RELPOSNED 121개, flags 303 111회, flags 11 10회
- flags 131/311 Fixed 0회
- hAcc 14~19 mm, RTCM 6.490 Hz

센티미터 단위 hAcc만으로 Fixed라고 판정할 수 없다. 재기동 뒤
dual-GNSS heading/RTK Fixed 유지 시험은 실패했다.

IMU는 `set_baud=false`, 115200, publisher 1개, 약 99.9 Hz로 정상
발행했다. 다만 startup에 `Failed to set baudrate for port 0X13 /
Error(3): Invalid Parameter`가 한 번 발생해 영향 규명이 남아 있다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_144047/gnss/quality_120s.txt`
- `/home/nvidia/camrod_field_logs/20260731_144047/bringup.log`

### 4.4 Localization 정지 측정

60초 동안 GNSS/GNSS pose는 1.00 Hz, IMU는 95.37 Hz,
platform/wheel odometry는 약 49.25 Hz, EKF/adapter/final은 각각
14.99 Hz였다. final pose rate 자체는 목표 15 Hz를 만족했다.

- selected pose header age p50/p95/max: 35.6/352.5/747.6 ms
- GNSS→final XY p50/p95/max: 0.023/0.041/0.056 m
- GNSS→final yaw p50/p95/max: 0.26/0.74/1.15 deg
- 정지 GNSS yaw span: 2.532 deg

CPU 포화 정지 측정이므로 주행 지연, crab yaw, lateral overshoot 또는
GNSS antenna lever arm 합격을 의미하지 않는다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_140639/pose_latency_stationary.json`

### 4.5 Voice/state startup

radar-off 재기동 전부터 240초 동안 capture했다.

- planning state: `WAIT_DZ -> ERROR_STOP -> WARN_RECOVERY -> WAIT_DZ`
- voice request: 36.052초의 `system.startup` 1회
- `system.ready`, drop-zone 복귀, driving 음성의 조기 발행: 0

READY 조건에 도달하지 않았고 실제 스피커 청취와 manual/UI mission은
수행하지 않았으므로 전체 voice/state TODO는 유지한다.

근거:

- `/home/nvidia/camrod_field_logs/20260731_radar_off/voice_state_startup_retry.txt`

## 5. CPU profile

RViz+WebKit 정지 상태 5분 profile:

- 8개 CPU core 평균 99.26%
- GPU 평균 36.95%
- RAM 평균 10.66/15.66 GB, swap 15 MB
- CPU 평균 온도 60.6℃

병목은 GPU·RAM·열이 아니라 CPU다. 8-core 전체 사용률 기준 주요 그룹:

| 그룹 | 전체 CPU |
|---|---:|
| Planning/Nav2 | 13.96% |
| 독립 system diagnostics | 13.56% |
| GNSS/IMU/LiDAR/Radar | 13.15% |
| RViz + WebKit + UI backend | 5.67% |
| Localization | 4.98% |
| Front camera + YOLO | 4.00% |
| Rear camera | 3.51% |

외부 Brave는 2.44%, VS Code/Codex/Claude는 5.08%를 추가 사용했다.
profile의 15개 `ros2 topic hz`와 수집 프로세스도 약 7.73%를 추가했다.
계측 자체만의 문제는 아니며, 원래 bringup log의 CPU 경고 737개 평균도
98.91%였다.

동일 과부하 구간에서 front camera 6.75 Hz, rear raw 2.92 Hz,
LiDAR filtered 4.52 Hz로 떨어졌고 `controller_server`가
`pure virtual method called`, exit -6 후 respawn했다.

우선순위:

1. 주행 때 Brave/VS Code/Codex/Claude를 종료하고 WebKit 또는 RViz 하나만 사용
2. 28개 독립 diagnostics process 통합 또는 필수 항목만 기동
3. 240 m × 240 m global costmap 크기/갱신 정책 재검토
4. rear 1920×1080 raw copy와 CPU JPEG subscriber gating 또는 HW encode
5. ground segmentation/LiDAR frame decimation
6. front YOLO 추론·debug image rate와 copy 감소

근거:

- `/home/nvidia/camrod_field_logs/20260731_142251`
- `/home/nvidia/camrod_field_logs/20260731_140639/bringup.log`

## 6. 설정 동기화 상태

첫 `field_test_tool.sh config`에서 한 건의 DIFF를 발견했다.

- `camrod_sensing/config/radar/sen0592_radar.yaml`: 7채널 모두 true
- `camrod_bringup/config/sensing/radar/sen0592_radar.yaml`:
  LEFT2/RIGHT2 false

실제 bringup에서 사용하고 이번에 검증한 더 안전한 격리 상태를 정본으로
삼아 package와 bringup 모두 LEFT2/RIGHT2 false로 맞추고, 주석에 격리
목적과 재활성 조건을 기록했다. `camrod_sensing`과 `camrod_bringup`을
재빌드한 뒤 package↔bringup↔install 전체 config 검사는 `config sync OK`로
통과했다.

<!-- HH_260731 - Record the post-sync regression result separately from older,
     unrelated workspace test artifacts left under build/. -->
LEFT2/RIGHT2 격리 전의 `7채널 모두 true`를 기대하던 radar 기본값 계약
테스트도 현재 운용 배열로 갱신했다.

- `camrod_bringup`: 81 tests, 0 failures
- `camrod_sensing`: 89 tests, 0 failures
- 합계: 170 tests, 0 failures

근거:

- `/home/nvidia/camrod_field_logs/20260731_config_sync_after.txt`

## 7. 종료 상태

모든 bringup과 계측 process를 정상 종료했다. ROS discovery cache에 잠시
남은 node 이름과 달리 OS process는 남지 않았다.
