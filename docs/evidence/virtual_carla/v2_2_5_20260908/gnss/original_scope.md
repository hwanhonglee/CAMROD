# GNSS 안테나 → 로봇 중심 및 90° 주차 회전 검증 범위

작성 기준: 2026-09-08, PURE `5e1e967f017b35ef4301a7c4b2b9ee5bb81371c0` 대 develop `f3a177038567f8f1b568e2f49d4c127bc298d9a5`. VIRTUAL 후보는 `9f7cd2bacda17a4581bd44ed66aaba857e89f1d4`. 이번 점검은 소스 읽기와 격리 native 검사만 했고, offset·지도·runtime·차량은 변경하지 않았다.

## 결론

**중심 보정 수식은 이미 develop에 있다. 다만 설정은 전방 안테나가 아니라 body X=0.0m, Y=+0.45m(좌측)이고, 회전 완료 뒤 XY를 다시 맞추는 단계는 현재 없다.** 실차 안테나가 전방에 있다면 실제 장착점에서 로봇 중심까지의 X/Y 치수 및 축 정의를 대조해야 한다. TF 이름을 중심으로 바꾸거나 XY 허용 오차만 조절하는 것으로 장착점 보정이 대신되지는 않는다. 이번 점검에서 임의의 실차 치수를 적용하지 않았다.

## develop 대비 실제 변경

| 항목 | develop f3 → PURE 5e | 소스 근거 |
| --- | --- | --- |
| GNSS 위치 오프셋·중심 frame | **변경 없음**: lever-arm=true, X=0.0m, Y=+0.45m; EKF 기준 robot_center_link | [input_adapter.yaml:22](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_localization/config/source/input_adapter.yaml:22), [ekf.yaml:24](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_localization/config/filter/ekf.yaml:24) |
| 수신기 heading trim | canonical config만 −90°→−92°로 동기화. bringup mirror는 **f3부터 −92°**였으므로 새로 바꾼 것이 아님 | [canonical:59](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_localization/config/source/input_adapter.yaml:59), [mirror:59](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_bringup/config/localization/source/input_adapter.yaml:59) |
| Nav2 / 주차 전 XY 허용값 | 실제 값 **0.10m / 0.20m 모두 유지**. 오래된 0.05m 설명 주석만 배포값과 맞춤 | [nav2_base.yaml:212](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_planning/config/nav2_base.yaml:212), [control.yaml:214](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_control/config/control.yaml:214) |
| 주차 전 XY 접근·yaw 전환 | controller/helper/기존 회귀 **변경 없음**. XY 정착 다음 yaw 정렬, 이후 주차기 START | [onTimer:919](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_control/src/drop_zone_maneuver_controller_node.cpp:919) |
| 이번 별도 PURE 주차 개선 | reverse 축 정지 범위에 들어와도 실제 station XY 원 밖이면, 도달 가능한 제한 범위에서 최종 접근 유지. **GNSS 중심 보정이나 회전 후 XY 재정렬 수정이 아님** | [reverse controller:476](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_control/src/reverse_parking_controller_node.cpp:476), commit `ddd599672` |
| 이번 별도 PURE 경로 개선 | local path 종료 0.25→0.05m 및 종점 기존 점 보존. Nav2 0.10m 유지. **GNSS calibration이 아님** | [TERMINAL_PATH_FIX.md](TERMINAL_PATH_FIX.md), commit `5e1e967f` |

## 중심 보정과 실제 주차 순서

[gnss_lever_arm.hpp:14](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_localization/include/camrod_localization/gnss_lever_arm.hpp:14)의 수식은 `p_center = p_antenna − R(yaw) × [offset_x, offset_y]`이다. ROS body축은 +X 전방, +Y 좌측이다. 안테나가 중심에서 거리 L만큼 떨어졌다면 제자리 90° 회전으로 안테나 자체는 약 `sqrt(2) × L` 움직이지만, 정확한 offset과 같은 시점의 yaw를 쓰면 보정한 중심은 고정된다. L=0.45m의 안테나 이동량 약 0.636m는 기하학 예시이지 이번 실차 측정값이 아니다.

[input adapter:580](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_localization/src/localization_input_adapter_node.cpp:580)는 fresh heading 또는 제한된 GNSS-anchor/EKF yaw-delta fallback을 선택한 다음 위치에서 회전한 오프셋을 뺀다. 필요한 heading이 없으면 위치 발행을 보류한다. 현재 YAML의 fallback anchor 제한은 **0.5초**, sample 매칭 허용은 **0.2초**이며, 이 fallback이 GNSS heading 자체를 유효하게 바꾸지는 않는다.

주차 전 동작은 `POSITION_PARKING_POINT → ALIGN_PARKING_YAW → selected parking START`이다. [target 선택:440](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_control/src/drop_zone_maneuver_controller_node.cpp:440)에서 fresh pose, frame 및 snapped-goal/station stamp 일치 등을 확인하고 목표 XY를 고정한다. [접근 helper:65](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_control/include/camrod_control/drop_zone_parking_approach.hpp:65)는 현재 yaw로 map 위치 오차를 body 전후/좌우 속도로 변환한다. 배포값은 XY 0.20m, 접근 한계 0.75m, 정착 0.5초, 접근 timeout 12초이다. [yaw 정렬:780](/home/hong/camrod_ws/pure_camrod_v224_verification/camrod_control/src/drop_zone_maneuver_controller_node.cpp:780)은 yaw/각속도 정착을 확인하며, 배포값은 yaw 5°, hold 1초, rate 3°/s이다. **이 단계가 끝나면 XY 오차를 재검사하여 접근 단계로 되돌리지 않고 주차기로 넘긴다.** 따라서 회전 후 중심 추정 오차가 저절로 다시 보정된다고 설명하면 부정확하다.

## 현재 CARLA가 검증하는 것과 하지 않는 것

현재 `camrod-site-geometry`는 [full launch:96](/home/hong/camrod_ws/src/camrod_carla_adapter/launch/camrod_carla_full.launch.py:96)의 production input_adapter YAML을 [1107행](/home/hong/camrod_ws/src/camrod_carla_adapter/launch/camrod_carla_full.launch.py:1107)으로 연결한다. 즉 **현재 프로필은 raw GNSS 입력과 lever-arm 보정 경로를 사용하도록 구성되어 있다.** 과거 metric-pose 전용 [camrod_input_adapter_carla.yaml:5](/home/hong/camrod_ws/src/camrod_carla_adapter/config/camrod_input_adapter_carla.yaml:5)는 별도 선택용 파일이며 lever-arm=false이다. 이 파일이 존재한다는 이유만으로 현재 프로필도 GNSS를 우회한다고 판정하지 않는다. 이번 추가 점검은 launch/config 대조이며 runtime 파라미터를 새로 조회한 것은 아니다.

CARLA 센서는 [spawn JSON:221](/home/hong/camrod_ws/src/camrod_carla_adapter/config/ranger_spawn_camrod_full_sensors.json:221)의 X=0/Y=±0.45m 좌우 GNSS, 10Hz, 위치 noise/bias=0 설정이다. **전방 X 오프셋 실차 형상을 재현한 구성이 아니다.** GNSS 위치는 실제 CARLA sensor actor에서 오지만, CARLA에 실제 이중안테나 수신기 heading이 없어 [feedback bridge:446](/home/hong/camrod_ws/src/camrod_carla_adapter/src/camrod_carla_adapter/feedback_bridge_node.py:446)는 실제 CARLA IMU 자세에 +90°를 더해 heading topic을 만든다. 현재 production trim −92°와 합치면 mapped IMU yaw 대비 순효과는 −2°이다. 이를 실수신기의 heading/위치 동기화·RTK 오차·장착 calibration 검증으로 볼 수 없다.

일정한 2° yaw 오차가 실제로 존재한다는 가정 아래 L=0.45m의 잔여 중심 오차는 `2L sin(1°) ≈ 0.0157m`이다. 이는 수식으로 계산한 영향 크기이며, 실차 오차를 측정했거나 이번 CARLA 오차 전부의 원인을 확정한 것이 아니다. 오프셋이나 heading을 다시 임의 변경하지 않았다.

## 확인한 회귀와 아직 필요한 검증

PURE 원본의 기존 native **15/15 PASS**: GNSS lever-arm 5개(90°/회전 중심 보존/전방+좌측 조합/heading 시간 불일치), bounded heading fallback 7개, 주차 접근 벡터/허용값/범위 제한 3개. [실행 XML](logs/gnss_center_scope_native.xml). 새 source 회귀를 추가한 것이 아니라 기존 테스트를 격리 실행했다. GNSS fallback 테스트의 예시 anchor는 3초 설정이므로 배포 YAML의 0.5초 제한을 실시간 검증한 결과로 해석하지 않는다. 주차 helper의 5cm 테스트도 generic helper 예시이며 배포값 20cm를 바꿨다는 의미가 아니다.

```bash
cd /home/hong/camrod_ws/pure_camrod_v224_verification
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  -Icamrod_localization/include -Icamrod_control/include \
  camrod_localization/test/test_gnss_lever_arm.cpp \
  camrod_localization/test/test_gnss_heading_fallback.cpp \
  camrod_control/test/test_drop_zone_parking_approach.cpp \
  -lgtest -lgtest_main -pthread \
  -o /tmp/camrod-gnss-center-native.JrISPU/test_gnss_center_scope
/tmp/camrod-gnss-center-native.JrISPU/test_gnss_center_scope --gtest_brief=1
```

실차에서 남은 검증은 장착 X/Y 실측, GNSS fix와 heading timestamp 정렬, 제자리 ±90°/180°에서 보정 전 안테나 궤적과 보정 후 중심 분산이다. 현재 CARLA 복귀에서도 `/localization/pose`, `/planning/goal_pose_snapped`, `/control/drop_zone_maneuver_controller/parking_approach_path_ros`와 maneuver/parking 상태를 같은 시각에 기록해 **회전 시작·종료의 실제 XY/yaw**를 비교해야 한다. yaw 단계의 `parking_approach_error_m`는 마지막 접근 계산값일 수 있으므로 그 cached 상태만으로 회전 후 오차를 판단하면 안 된다. 성공 PNG/GIF만으로 실차 전방 안테나 보정 검증을 주장하지 않는다.

## 별도 VIRTUAL 후보 검토 메모

`9f7cd2ba`의 작은 body-yaw 명령 하한은 precompensate/restore 전용이며 이 문서의 GNSS·drop-zone 주차 보정과 별개다. 설정 전달·범위 제한에서 새 실행 오류는 발견하지 못했지만, 실제 수렴은 미검증이다. `camrod_carla_develop_site_geometry.launch.py:251`의 “measured angular deadband” 주석은 추후 정리 대상이다. 관측된 것은 작은 토크에서 흔들림/미수렴이며 deadband를 직접 측정한 것은 아니다. 동결된 source는 이 점검에서 수정하지 않았다.
