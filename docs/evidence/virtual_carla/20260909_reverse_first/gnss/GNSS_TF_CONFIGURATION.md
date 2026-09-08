> 휴대용 읽기용 뷰: 원본 문서는 [별도 보존](GNSS_TF_CONFIGURATION.original.md)했습니다. 본문의 source 링크만 이 저장소 상대경로로 바꿨으며, 파일 내용의 실제 검증 runtime은 f02입니다.

# 현재 GNSS 중심 보정·TF 설정 — 2026-09-09

## 결론과 감사 범위

**GNSS 센서를 강제로 로봇 중심으로 옮긴 것이 아니다.** 실제 센서는 왼쪽 0.45m에 남아 있고, localization 입력 코드가 측정한 안테나 좌표에서 회전된 장착 오프셋을 빼서 **로봇 중심의 위치를 계산**한다. TF는 센서가 실제 어디에 붙어 있는지를 계속 나타낸다.

2026-09-09 03:38 KST 기준 배포 소스 `f02c2fb512a25565c676b5103d780635ae0892f9`, actor 98의 소스·실행 노드 파라미터·실제 `/tf`, `/tf_static`, CARLA 센서 상대 위치를 읽기 전용으로 대조했다. ROS는 기존 환경을 source한 `ROS_DOMAIN_ID=5`, `ROS_LOCALHOST_ONLY=0`이다. 제어 명령·파라미터 변경·서비스 상태 변경·90°/180° 주행은 하지 않았다. CARLA 조회는 서버가 이미 발생시킨 `world.wait_for_tick(5.0)`만 기다렸고 `world.tick()`는 호출하지 않았다.

이번 감사에서 제품 소스·브랜치·원격을 변경하지 않았다. 이후 90°/180° 명령 시험은 별도 실행이며 이 문서만으로 성공했다고 볼 수 없다.

## 1. 실제로 무엇을 비교해야 하는가

| 자료 | 현재 토픽 | 뜻 |
| --- | --- | --- |
| 원시 GNSS | `/carla/ego_vehicle/gnss` | 실제 CARLA 왼쪽 GNSS 센서의 위도·경도·고도. 차체가 제자리 회전해도 센서 자체는 원호를 그린다 |
| 보정 직후 중심 | `/sensing/gnss/pose_ros` | input_adapter가 안테나 lever arm을 한 번 뺀 로봇 중심 위치, `map` 좌표 |
| 최종 추정 중심 | `/localization/pose` | GNSS·IMU·휠 속도를 융합한 EKF 결과를 pose_selector가 선택한 중심 위치 |
| 비교용 실제 차체 중심 | `/camrod_carla/metric_pose` | 실제 CARLA 차체 위치를 CAMROD map으로 정렬한 값. 현재 GNSS/EKF 대신 주입하는 입력이 아니라 비교용 truth |
| 방향 | `/sensing/gnss/navheading` | 현재 시뮬레이션에서는 실제 CARLA IMU에서 변환한 heading. 실물 dual-GNSS receiver의 측정 메시지와 출처가 다르다 |

따라서 raw GNSS가 움직였다는 이유만으로 보정 실패가 아니다. `보정 중심−실제 중심` 및 `최종 추정 중심−실제 중심`의 **오차벡터 변화**, 그리고 실제 차체 자체의 미끄럼을 분리해야 한다. raw 위·경도는 local ENU 등 미터 좌표로 변환해야 하며 원시 ENU와 map 좌표를 정렬 없이 직접 빼면 안 된다.

## 2. 실제 장착 위치: 소스·ROS TF·CARLA가 일치

ROS body 축은 +X 전방, +Y 왼쪽, +Z 위쪽이다.

| 확인 대상 | 왼쪽 GNSS | 오른쪽 GNSS |
| --- | --- | --- |
| spawn 설정 | `(0,+0.45,0)m` | `(0,−0.45,0)m` |
| 실제 CARLA ROS TF | `ego_vehicle→ego_vehicle/gnss = (0,+0.45,0)` | `ego_vehicle→ego_vehicle/gnss_right = (0,−0.45,0)` |
| CARLA frame **1028102**, actor 98의 역변환으로 직접 측정한 ROS body 좌표 | actor **111**, `(-0.000002315,+0.450000300,-0.000000052)m` | actor **112**, `(-0.000002335,−0.449999996,-0.000000018)m` |

미세한 차이는 floating-point 변환 수준이며 **현재 센서는 전방 장착이 아니다.** 첫 CARLA client 연결 직후 frame 0·빈 actor 목록은 유효 측정으로 쓰지 않았고, 기존 서버 tick을 기다린 뒤 위 실제 frame에서 다시 측정했다.

설정 근거는 [Ranger 센서 spawn JSON](../../../../../camrod_carla_adapter/config/ranger_spawn_camrod_full_sensors.json#L221), [sensor kit 장착 YAML](../../../../../camrod_sensor_kit/config/robot_params.yaml#L70)이다. 왼쪽 GNSS가 localization 주 입력이며 오른쪽은 호환 센서 정보에 사용된다. spawn의 GNSS tick은 0.1s, noise/bias는 0이다. 이는 **노이즈 없는 시뮬레이션 센서 설정**이지 실제 RTK 정확도·위성 수·전파환경 검증이 아니다.

## 3. 안테나 측정부터 중심 위치까지의 코드 경로

```text
CARLA GNSS (왼쪽 안테나의 위·경도)
  → carla_ros_bridge: /carla/ego_vehicle/gnss
  → carla_feedback_bridge: /sensing/gnss/ublox_gps_node/fix
  → localization/input_adapter:
       WGS84 위·경도 → ECEF → 기준점 ENU → map XY
       중심 XY = 안테나 XY − R(현재 heading) × 장착 offset
  → /sensing/gnss/pose_with_covariance_ros (이미 중심 기준, frame=map)
  → EKF: GNSS 위치·heading + IMU 각속도 + 휠 속도
  → /localization/primary/odometry_ros
  → input_adapter의 출력 형식 변환 → pose_selector
  → /localization/pose (robot_center_link의 map 위치)
```

실제 input_adapter 파라미터를 GetParameters로 확인했다:

| 파라미터 | 실행 값 |
| --- | --- |
| `navsat_topic` | `/sensing/gnss/ublox_gps_node/fix` |
| `utm_pose_topic` | 빈 문자열 — metric truth 입력 없음 |
| `map_frame_id` | `map` |
| `enable_gnss_lever_arm_correction` | `true` |
| `gnss_antenna_offset_x_m`, `gnss_antenna_offset_y_m` | `0.0`, `0.45` |
| `gnss_heading_yaw_offset_deg` | **−92.0°** |
| `gnss_heading_timeout_s` | `1.0s` |
| `gnss_lever_arm_require_fresh_heading` | `true` |
| `enable_gnss_lever_arm_ekf_heading_fallback` | `true`, 유효 GNSS anchor 최대 `0.5s` |
| `offset_lat`, `offset_lon`, `offset_alt` | `36.8435737`, `128.0925646`, `0.0` |
| `yaw_offset_deg`, `rotate_latlon_xy_by_yaw_offset` | `0.0°`, `true` |
| `gnss_covariance_floor_xy` | `0.1m²` |
| `use_sim_time` | `false` |

핵심 구현은 [onNavSatFix](../../../../../camrod_localization/src/localization_input_adapter_node.cpp#L549)와 [antennaPositionToRobotCenter](../../../../../camrod_localization/include/camrod_localization/gnss_lever_arm.hpp#L17)이다. 안테나 body offset을 `(a,b)`, 방향을 `ψ`라 하면:

```text
center_x = antenna_x − (cosψ × a − sinψ × b)
center_y = antenna_y − (sinψ × a + cosψ × b)
```

현재 `(a,b)=(0,0.45)`이므로 `center_x=antenna_x+0.45sinψ`, `center_y=antenna_y−0.45cosψ`이다. 고정 map 방향으로 0.45m를 빼는 것이 아니라 **매 heading에 맞춰 회전한 벡터를 뺀다.** 출력의 frame은 `gnss_link`가 아니라 `map`이며 내용은 이미 중심 위치다. 지금 구현은 planar XY 보정으로, 3차원 안테나 높이/roll/pitch lever-arm 보정까지 수행하는 것은 아니다.

heading이 없으면 과거 유효 GNSS anchor와 시간 정렬된 EKF yaw 차분을 짧게 사용한다. anchor가 오래되면 보정을 승인하지 않으며, fallback을 실제 유효 GNSS heading으로 둔갑시키지 않는다. 이는 오프셋을 두 번 빼는 동작이 아니라 한 번의 XY 보정에 필요한 회전각을 선택하는 제한된 fallback이다.

## 4. +90°와 −92°는 서로 다른 처리다

1. 표준 CARLA ROS bridge가 Unreal의 좌수계 좌표를 ROS 우수계로 바꾼다: 위치 `(x,y,z)→(x,−y,z)`, yaw 부호 반전. 실제 bridge 변환 코드 (원래 외부 bridge 설치 경로: `/home/hong/Downloads/ranger-carla-4ws-pipeline/.work/ros-bridge-ws/src/ros-bridge/carla_common/src/carla_common/transforms.py:81`; 저장소에 포함되지 않음).
2. feedback bridge는 CARLA ROS 위치·자세를 CAMROD map과 정렬한다. 실행 값은 translation `(6.9521841297,9.4901857126)m`, yaw `8.14281112e−7rad`이다. 이 map 정렬은 센서 0.45m 보정과 별개다. raw GNSS 위·경도에는 이 평면 translation을 직접 더하지 않는다. [정렬 설정](../../../../../camrod_carla_adapter/config/woraksan_lane_anchor_alignment.yaml#L16), [실제 IMU 변환](../../../../../camrod_carla_adapter/src/camrod_carla_adapter/feedback_bridge_node.py#L405).
3. 시뮬레이션에 hardware dual-GNSS heading 메시지가 없으므로 feedback bridge는 실제 IMU orientation에 **+90°**를 더해 기존 `/sensing/gnss/navheading` 인터페이스를 만든다. 실행 파라미터 `publish_gnss_heading_from_imu=true`, `gnss_heading_yaw_bias_rad=π/2`를 확인했다.
4. production input_adapter가 그 heading에 배포 trim **−92°**를 적용한다. [onGnssHeading](../../../../../camrod_localization/src/localization_input_adapter_node.cpp#L488), [입력 설정](../../../../../camrod_localization/config/source/input_adapter.yaml#L65).

현재 순효과는 map 정렬된 CARLA IMU yaw 대비 **−2°**다. [feedback YAML](../../../../../camrod_carla_adapter/config/feedback_bridge.yaml#L38) 및 bridge 주석에 남은 “unchanged −90°”는 현재 −92° 배포값과 맞지 않는 과거 설명이다. **이번 감사에서 −92를 −90으로 바꾸지 않았다.** 이 trim의 실차 교정 근거까지 새로 검증한 것은 아니다.

참고로 길이 0.45m lever arm에 순수 2° heading 오차만 있다면 오프셋 벡터 차이 크기는 약 `2×0.45×sin(1°)=0.015707m`이다. 이는 이상화한 계산이며 관측된 모든 GNSS/EKF 오차의 원인이 −2°라고 단정하는 근거가 아니다.

## 5. TF의 실제 구조와 중복 보정 여부

```text
map
 └─ odom                         고정 identity: ekf_map_to_odom_static_tf
     └─ robot_center_link        동적: localization/ekf_filter
         ├─ robot_base_link      고정 (-0.443,0,0): 기존 후륜축 호환 frame
         └─ sensor_kit_base_link  고정 (0,0,0)
             └─ gnss_link        고정 (0,+0.45,0)
```

위 정적 값과 동적 `odom→robot_center_link`를 실제 TF 수신으로 확인했다. `robot_base_link`는 로봇 중심과 같은 이름만 바꾼 frame이 아니라 **뒤 차축 기준의 호환 frame**이다. 중심↔뒤 차축 0.443m는 GNSS 0.45m와 다른 보정이다.

sensor kit의 고정 TF는 [sensor_kit.launch.py](../../../../../camrod_sensor_kit/launch/sensor_kit.launch.py#L158)가 YAML을 xacro 인자로 넘겨 `/sensor_kit/robot_state_publisher`에서 발행한다. [xacro 중심/뒤축 연결](../../../../../camrod_sensor_kit/urdf/camrod_sensor_kit.xacro#L209), [GNSS joint](../../../../../camrod_sensor_kit/urdf/camrod_sensor_kit.xacro#L240), [map→odom 고정 publisher](../../../../../camrod_localization/launch/localization.launch.py#L116).

**TF를 변경하는 것만으로 NavSatFix의 위·경도가 중심 위치로 바뀌지는 않는다.** 위 input_adapter의 수식이 실제 데이터 보정을 한다. EKF에는 이미 중심 기준 `map` pose를 넣으므로 다시 `gnss_link→center` 이동을 적용하지 않는다.

실제 EKF는 20Hz, `two_d_mode=true`, `world_frame=odom`, `base_link_frame=robot_center_link`, `publish_tf=true`다. 같은 GNSS pose를 `pose0`에서는 XYZ, `pose1`에서는 yaw만 선택해 사용하며 같은 XYZ를 이중 융합하지 않는다. IMU에서는 roll/pitch·각속도를, wheel 입력에서는 vx/vy/yaw-rate를 사용한다. [EKF 설정](../../../../../camrod_localization/config/filter/ekf.yaml#L16).

feedback bridge의 실제 `publish_ground_truth_localization=false`, `publish_ground_truth_tf=false`, pose_selector의 `publish_selected_tf=false`를 확인했다. 따라서 현재 구성에서 truth bridge와 selector가 EKF의 중심 TF를 또 덮어쓰도록 설정돼 있지 않다. `/tf`에는 CARLA 센서 frame·AprilTag 등 여러 publisher가 있으므로 publisher 개수만 보고 중심 TF 중복이라고 판단하면 안 된다. 이 감사는 모든 가능한 동적 TF 충돌을 장시간 전수 검사한 것은 아니다.

예전 [metric-truth 전용 입력 YAML](../../../../../camrod_carla_adapter/config/camrod_input_adapter_carla.yaml#L5)은 GNSS lever arm이 꺼져 있지만 **현재 활성 설정이 아니다.** 현재 full launch는 production 입력 YAML을 사용하며, 실제 `utm_pose_topic=''`와 NavSatFix 입력·보정 ON으로 재확인했다. [launch 기본값](../../../../../camrod_carla_adapter/launch/camrod_carla_full.launch.py#L96), [bringup 전달](../../../../../camrod_carla_adapter/launch/camrod_carla_full.launch.py#L1107).

## 6. 실제 토픽 출처·QoS·시간 기준

| 토픽 | 실제 publisher | QoS: reliability / durability / depth |
| --- | --- | --- |
| `/carla/ego_vehicle/gnss` | `/carla_ros_bridge` | RELIABLE / VOLATILE / 10 |
| `/sensing/gnss/ublox_gps_node/fix` | `/carla_feedback_bridge` | BEST_EFFORT / VOLATILE / 5 |
| `/sensing/gnss/navheading` | `/carla_feedback_bridge` | BEST_EFFORT / VOLATILE / 5 |
| `/sensing/gnss/pose_ros` | `/localization/input_adapter` | RELIABLE / VOLATILE / 10 |
| `/localization/pose` | `/localization/pose_selector` | RELIABLE / TRANSIENT_LOCAL / 1 |
| `/camrod_carla/metric_pose` | `/carla_feedback_bridge` | RELIABLE / VOLATILE / 20 |

input_adapter는 NavSatFix와 heading을 SensorDataQoS로 구독한다. 실제 bridge의 `stamp_with_reception_time=true`, 위 localization 노드들의 `use_sim_time=false`다. raw CARLA sensor/odometry 시뮬레이션 stamp와 다시 찍은 CAMROD wall-clock stamp를 같은 clock처럼 직접 비교하면 안 된다. 회전 오차 비교에서는 동일 clock의 stamp 또는 명시된 수신 monotonic과 매칭 시간차를 사용하고 그 허용차·수신 지연을 기록해야 한다.

## 7. 실차 전방 안테나로 옮길 때 바꿔야 하는 곳

실차의 **앞/뒤 차축 중점인 robot_center_link에서 안테나까지** 실제 `(X,Y,Z)`를 측정해야 한다. “차량 앞”이라는 말만으로 수치를 추정하지 않는다.

- TF 장착 위치: [camrod_sensor_kit/config/robot_params.yaml의 gnss](../../../../../camrod_sensor_kit/config/robot_params.yaml#L70). 실제 운영 launch에서 이 파일 또는 별도 profile을 선택하는지 확인한다.
- planar 데이터 보정: 실제 선택된 [localization 입력 YAML](../../../../../camrod_localization/config/source/input_adapter.yaml#L23)의 `gnss_antenna_offset_x_m/y_m`. [bringup 쪽 대응 설정](../../../../../camrod_bringup/config/localization/source/input_adapter.yaml#L23)도 존재하므로 파일 한쪽만 보고 운영값을 단정하지 말고 **실제 노드 파라미터**와 일치시킨다.
- GNSS heading의 장착방향/수신기 기준: 같은 입력 YAML의 `gnss_heading_yaw_offset_deg`. 위치 offset과 yaw trim은 별개이며 실차 dual-antenna baseline 방향에 맞게 확인한다. 실차에 CARLA용 +90° bridge를 추가하는 것이 아니다.
- 시뮬레이션도 같은 전방 장착으로 비교하려는 경우에만 Ranger spawn JSON의 GNSS 센서 위치를 그 실측값에 맞춘다. TF만 옮기고 실제 CARLA sensor는 왼쪽에 남겨두면 잘못된 시험이다.

이 문서에서는 위 값들을 변경하지 않았다. 실차 전방 offset은 현재 미확인이다.

## 8. 90°/180°에서 기대할 것과 현재 확보한 결과

차체 중심이 완전히 고정되고 안테나가 중심에서 0.45m 떨어져 있다면 raw 안테나의 시작→끝 변위는 90°에서 약 **0.6364m**, 180°에서 **0.9m**다. 올바르게 보정된 중심은 이 원호 운동을 따라가지 않아야 한다. 다만 실제 4륜 차체가 돌면서 미끄러지면 실제 중심 자체도 움직이며, 그 이동까지 억지로 0으로 고정하면 오히려 실제 위치를 숨긴다.

이미 확보한 [−90.416908° 부분구간 분석](first90/README.md)은 실제 −178.176376° 연속 회전의 첫 부분이다. raw 안테나 이동 약 62.44cm, 실제 중심 이동 4.18cm, 보정 직후 중심 이동 3.19cm, 보정−truth 오차벡터 변화 1.35cm를 구분해 기록했다. **90° 명령을 내리고 정지한 결과가 아니며**, 180° 명령 후 정착 오차까지 새로 검증한 보고서도 아니다.

후속 독립 90°/180° 시험에는 실제 시작·종료 yaw, 정지 유지 구간, raw/보정/EKF/truth 궤적, 실제 중심 이동, 오차벡터 변화·최대 절대 오차, 시각 매칭 품질을 함께 남겨야 한다. 허용 오차를 미리 정하지 않은 측정을 임의의 GNSS PASS로 바꾸지 않는다.
