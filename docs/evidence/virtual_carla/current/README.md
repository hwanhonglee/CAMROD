# 2026-09-01 develop-parity rendered smoke

이 디렉터리는 `test/virtual-carla-parity-20260901`에서 검증한 뒤 최종
`virtual/carla`에 통합하는 **현재 develop-parity smoke**의 선별 증거다. 과거
Woraksan-tuned B1/B12 결과와 섞지 않는다.

## 실행 식별자

| 항목 | 값 |
|---|---|
| core 비교 기준 | `origin/develop` `9756edf2d7aebf59cf8b635b5d8f5982bf6211aa` |
| 실행 전 CARLA branch 기준 | `origin/virtual/carla` `c3646a89824fb075da58727d216a56ae0124b890` |
| 실행 프로필 | `./scripts/virtual_carla/run.sh camrod` (`develop-parity`) |
| CARLA / UE | CARLA 0.9.15 source, UE 4.26, onscreen Woraksan custom map |
| actor | id `2`, `vehicle.ranger.default`, role `ego_vehicle` |
| baseline gate | `5db8309dd257c29f7606f01d2033c9ca39c8147268cc74d36659c3785437fc26` |
| physical 4WS gate | `672695cfe8de4edfdf666df53584f7d107ec93af0dbeb839c610a3c2ccc5ad39` |
| CAMROD launch log | `/home/hong/.ros/log/2026-09-01-15-51-24-257975-htop-1544085` (실행 host-local) |

CARLA server, standard ROS bridge, 20 Hz step pacer, Ranger spawn, CAMROD를
각각 분리된 프로세스로 시작했다. 종료는 CAMROD, spawn, bridge, pacer, server
순서로 수행했고 actor 2~17 및 pseudo actor를 모두 destroy한 뒤 port 2000/8010을
확인했다.

## 실제 sensor와 perception 결과

`./scripts/virtual_carla/run.sh audit-sensors` 결과는 다음과 같다.

- actual stream `36/36` PASS, CARLA sensor actor `13/13` PASS
- front/rear Image와 CameraInfo, 전방 LiDAR, radar 7개, GNSS 2개, IMU 1개가
  모두 CARLA actor에서 시작함
- fake/dummy node 0개, `/bringup/fake_sensor_publisher` topic 없음
- `/perception/camera/detections_2d`의 유일 publisher는
  `/perception/yolov9mit`
- `/perception/obstacles`의 유일 publisher는 `/perception/obstacle_fusion`
- `/sensing/lidar/lidar_cost_grid`는 semantic `/perception/obstacles`를 구독하고
  raw LiDAR cost는 `off`
- `/perception/lidar/bboxes`는 classless Euclidean cluster의 fusion 보조/시각화
  경계이며 CARLA UI는 이를 직접 구독하지 않음

| 실측 stream | rate |
|---|---:|
| `/clock` | 약 19.2 Hz |
| front compressed camera | 약 4.74 Hz |
| YOLO `Detection2DArray` | 약 4.71 Hz |
| YOLO annotated image | 약 4.76 Hz |
| semantic `/perception/obstacles` | 약 4.75 Hz |
| filtered LiDAR | 약 9.48 Hz |

UI 카메라 화면에서 800×600 CARLA front/rear 영상이 각각 약 4.8/4.7 Hz로
표시됐다. 지도·인지 화면에서는 `Semantic fusion cost`와
`Semantic obstacle cloud`만 안전 계층으로 표시됐다. 당시 장면에는 YOLO class
장애물이 없어 semantic cost `0 cells`, obstacle sample `0`이었지만 raw CARLA
LiDAR는 약 2,685 points와 9.5 Hz로 살아 있었다. 즉 0은 fake sensor 누락이 아니라
맑은 장면의 의미론 결과다.

## 수동 명령과 physical 4WS smoke

production UI와 같은 `/ws/manual-drive` single-owner/deadman protocol을 사용했다.
두 번째 client 연결은 `manual_drive_busy`로 거부되는 것도 먼저 확인했다.

직진은 `ARM -> 0.2 m/s 요청 3초 -> ZERO` 순서였다. develop-parity safety gate의
기존 `0.5` speed scale 뒤 physical command는 보수적으로 제한된다. CARLA odometry는
`(-20.672642, 33.952450)`에서 `(-20.365639, 33.989552)`로 약 `0.309 m` 이동했다.
ZERO 후 의도적으로 0.5초를 둔 첫 실행에서는 0.25초 deadman이 먼저 zero/disengage를
수행했다. 종료 후 `/control/cmd_vel_ros`는 정확히 0이고 safety gate는 `STANDBY`였다.

그 다음 모드 전환은
`ZERO_TURN -> exact ZERO -> CRAB -> exact ZERO -> DISARM`으로 실행했다. 마지막
backend 응답은 `client_disarm`이었다. phase-qualified ROS probe 결과는 다음과 같다.

| 모드 | extended command | physical wheel target | drive sign | wheel torque |
|---|---|---|---|---|
| ZERO_TURN | mode 2, yaw `0.05 rad/s` | FL `+1.01564`, FR `-1.01564`, RL `-1.01564`, RR `+1.01564 rad` | `+ - + -` | `-0.0272, +0.0272, -0.0272, +0.0272 Nm` |
| CRAB | mode 1, speed `0.05 m/s`, crab `+1.53589 rad` | 네 바퀴 모두 `-1.53589 rad` (`-88°`) | `+ + + +` | 네 바퀴 모두 약 `+0.00885 Nm` |

두 모드 모두 `PHYSX_FOUR_WHEEL_STEERING`, physical gate accepted,
`independent_wheel_torque_active=true`였다. 표의 값은 controller가 physical bridge에
보낸 wheel별 적용 명령이다. 현재 ROS status가 네 wheel의 실제 angle/torque를
연속 계측해 발행하는 것은 아니므로, startup의 physical telemetry gate와 실제 영상은
함께 보되 이 값을 continuous actual-angle telemetry라고 부르지 않는다.

## 선별 시각 증거

| 파일 | 내용 | SHA-256 |
|---|---|---|
| `ranger_actor_actual_carla_20260901.png` | 실제 onscreen CARLA의 Ranger actor와 Woraksan 주차장 | `347c316a3247e574252aacc66cf247f116be750523cda2ca93fb77bf469fe34a` |
| `ui_camera_actual_carla_20260901.png` | production UI의 실제 CARLA front/rear camera와 live rate | `2347b720f81442e536dd6db5a7c9f0b8ff75087cfe2a5b264ddf1dabfaf880ba` |
| `ui_semantic_perception_actual_carla_20260901.png` | semantic fusion 전용 지도·인지 화면 | `43047ebe4ff288f66286243e8d4cabef440c557558dd0991bffccc540a87513b` |
| `manual_straight_4ws_live_20260901.gif` | bounded 직진과 spectator follow | `36292300ee3901bd16973f74afa2a457def00b4145962653164f6205fe716005` |
| `manual_zero_turn_crab_4ws_live_20260901.gif` | ZERO_TURN, mode-cut ZERO, CRAB 실제 영상 | `4b9acacdce1d132c3242e6fd8c865fc1931d5f07f3ea325b3ef632ec59383dfa` |

PNG/GIF는 AI 생성 또는 fake sensor 화면이 아니다. X11의 실제 CARLA window와
production UI를 캡처했다. 원본 MP4와 중간 capture는 선별 파일 생성 후 `/tmp`에서
복구 가능한 휴지통으로 옮겼다.

### 원시 수치 증빙의 한계

위 rate, point count, odometry 이동량과 wheel command probe의 당시 console 출력은
별도 JSON/manifest로 이 디렉터리에 커밋하지 않았다. 따라서 5개 시각 파일과 gate
해시는 독립적으로 대조할 수 있지만, 표의 수치는 이 문서만으로 기계 재계산할 수 없다.
동일 수치를 독립 검증하려면 같은 commit/profile로 rendered stack을 다시 실행하고
`audit-sensors`, `ros2 topic hz`, odometry 및 physical command probe 원본을 새 run
디렉터리에 저장해야 한다. 이 한계를 이유로 B1~B13 mission PASS를 주장하지 않는다.

## 판정 경계

이번 실행이 닫은 범위는 rendered server/bridge/spawn, production CAMROD/UI,
actual sensor provenance, YOLO/fusion, bounded manual straight/zero-turn/crab와
physical wheel command다. 목적지 dispatch, B1~B13 도착, Return, Drop Zone parking과
charging은 이번 smoke에서 실행하지 않았다. 따라서 현재 develop-parity B1~B13은
계속 `PENDING`이며 과거 tuned B1/B12 결과로 대체하지 않는다.
