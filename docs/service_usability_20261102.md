<!-- HH_261102 - Keep the requested scope, measured evidence and unverified hardware behavior separate. -->
# 2026-11-02 수정·시험 브리핑

<!-- HH_261103 - Preserve dated test context instead of presenting old publication status as the current release state. -->
> 이 문서는 **2026-11-02 당시의 작업·시험 기록**이다. 본문의 미커밋·미push, CARLA 미실행, 경로·해시·시험 수는 당시 상태이며 이후 발행 상태나 별도 CARLA 시험을 뜻하지 않는다. 현재 순수 릴리스 범위·기본값·한계는 [unit2-v1.0.1](releases/unit2-v1.0.1.md)을 참고한다.

## 요청 범위

원래 기능 요청은 세 가지이고, Ponytail 코드 점검은 추가 작업이다.

1. 비이용객의 화면 조작 대책을 직원·이용객의 추가 업무 없이 재검토한다.
2. **충전 전용 UI**에 충전 위치에서 앞으로 나와 **drop_zone 안의 기존 대기·주차 위치에 정지**하는 버튼을 둔다.
3. 기존 radar 제외 설정 / radar 개선만 / LiDAR 보조만 / 둘 함께를 같은 조건으로 비교하고 PNG·GIF를 남긴다.

기존 35% 출발 기준은 새 기능이 아니다. 별도 ‘짐 작업 완료 → 강제 재도킹’ 버튼은 요청에 없어 제거했다.
일반 복귀 후에는 기존 후진 주차 및 배터리 기준 자동 도킹 정책을 그대로 사용한다.

작업은 `origin/unit2/main`의 `c7d2f9c88`에서 분기한
`test/unit2-service-usability-20261102`에서 진행했다.
위치는 `/home/hong/camrod_service_test_ws/src`이며 기존 `camrod_ws` 소스·운영 서버와 원격 브랜치는 변경하지 않았다.
커밋·push 또는 실제 로봇 배포 완료를 뜻하지 않는다.

## 1. 화면 조작 대책 재검토

현재 Guest 접속 제한은 단일 연결/IP 재접속 유예일 뿐 이용객 인증이 아니다.
`guest/robot/operator`도 사람의 신원이 아니라 명령이 들어온 경로다.
따라서 ‘휴대폰을 연결하면 비이용객을 차단한다’고 설명할 수 없다.

현재 코드는 활성 미션의 사이트·세대·요청 경로와 단계가 맞지 않으면 새 목적지나 완료 요청을 거절한다.
이동 중에는 그 제한을 유지하고, 명시적인 출발·운행 정지 확인은 기존 흐름을 재사용한다.
새 직원 승인 버튼, 고객 비밀번호, 로그인, 매번 휴대폰 조작을 요구하는 잠금은 추가하지 않았다.

완전한 이용 자격 제한이 필요하면 **기존 예약·체크인 안내에 자동 발급된 이용 링크를 포함하는 방식**을 검토할 수 있다.
이미 제공하는 안내를 활용하면 직원의 매회 승인이나 별도 로그인은 피할 수 있지만,
현재 저장소에는 신뢰할 예약 인증 연동이 없다. 외부 서비스가 있다고 가정해 임의 구현하지 않았다.
길게 누르기·사이트명 입력·확인창만으로 타인을 식별할 수는 없다.
현 단계의 미션·단계 제한은 오조작 방지이며 완전한 무단 이용 차단은 아니다.

## 2. 충전 화면과 drop_zone 내부 정차

### 화면 위치와 동작

일반 서비스 선택 화면에 이동 버튼을 추가한 것이 아니라 **별도 충전 화면**에 배치했다.

1. 실제 차량 충전 신호, 배터리, 현재 상태를 표시한다.
2. ‘짐을 꺼내기 위해 대기 위치로 이동’을 누르고 주변 확인에 동의한다.
3. 기존 정지 대기·음성·제어 소유권 정리 후, 지도에 이미 있는 **일반 후진 주차 위치**로 이동한다.
4. drop_zone 안에서 정차한다. 도로 진출용 90도 회전·캠핑사이트 미션·주차 시작은 실행하지 않는다.
5. 그 뒤 기존 배송·호출 서비스 선택을 사용한다. 새 완료·재도킹 절차는 없다.

실제 충전 중이 아니면 이 이동 API를 거절한다. 페이지 접속·새로고침만으로 움직이지 않는다.
정차 표시는 명령 수락만으로 하지 않고 최신 차량 상태, 실제 저속/정지 측정과 충전 해제 피드백을 확인한다.
연결이 끊기면 이동 버튼은 비활성화하지만 기존 운행 정지 확인 기능은 유지한다.

### 제어 수정

기존 일반 `EXIT`는 `/planning/lanelet_pose`까지 나간 뒤 도로 방향으로 회전한다.
이를 충전 화면 버튼에 그대로 적용하는 것은 요청과 달라 수정했다.
정확한 충전 화면 요청만 기존 지도 station 좌표를 목표로 고정하고, 도로 목표와 분리한다.
차체 경계는 기존 `/platform/robot/planning_boundary`를 재사용한다.
전체 차체와 기존 여유 영역이 같은 drop_zone 안에 머무르는 이동만 허용하며,
알 수 없는 경계·지도, 뒤쪽/과도한 측면 목표, 위치·방향 오류는 거절한다.
목표가 부적합하다고 임의의 다른 좌표나 고정 전진 거리로 대체하지 않는다.

새 정차 상태는 `CARGO_ACCESS_READY`이고 도로 진출 완료인 `ROAD_HANDOFF_READY`와 구분한다.
일반 배송·호출의 기존 EXIT는 변경하지 않는다.
이전 이동의 늦은 상태 메시지나 완료 신호가 정차 후 미션을 다시 열지 못하도록 처리했다.

기존 자동 주차 정책은 일반 복귀 시 후진 주차를 먼저 하고,
실제로 주차가 완료된 뒤 SOC가 35% 미만이거나 유효 SOC를 알 수 없으면 도킹을 선택한다.
35% 이상은 명시적인 도킹 요청이 없다면 일반 주차다. 기존 정책을 새 완료 버튼으로 중복 구현하지 않는다.

[충전 화면 PNG·GIF와 재현 설명](evidence/service_usability_20261102/charging_ui/README.md).
이 화면 자료는 실제 production UI에 시험 응답을 공급한 **UI fixture**이며 실제 로봇 이동 영상이 아니다.

주의: 현재 플랫폼에 충전기 전원/접촉기 차단·케이블 잠금 해제 API는 없다.
버튼은 기존 이동 제어 요청이지 전기적 분리 보증이 아니다.
보수적인 차체 경계 검사가 현재 충전 위치를 거절할 수도 있다.
현장 충전 접점·케이블·지도·장착 위치를 확인하기 전에는 실제 로봇 출차 완료로 판단하지 않는다.

## 3. radar·LiDAR 네 조건 비교

### 무엇을 바꿨나

- 2호기는 차체 근접 반사가 없다는 운영자 설명에 맞춰 기존 고정 반사 제외 필터를 해제했다.
  전방 후보 최대 0.30 m, 측면 0.10 m와 후방 비활성 설정은 임의로 바꾸지 않았다.
- float32의 정확한 300 mm가 경계 비교에서 빠지던 문제를 수정했다.
- 필수 radar의 입력 소실·오래된 데이터·형식 오류·좌표 변환 실패가 최신 ‘장애물 없음’으로 바뀌지 않게 했다.
  최종 제어기는 필수 센서의 최신성을 별도로 확인한다.
- 아주 느린 비영점 전진·후진·크랩·회전도 정지 검사에서 빠지지 않게 했다. 진짜 영점 명령과는 구분한다.
- **기존 LiDAR 비용 노드를 재사용하는 별도 전방 보조 모드**를 추가했다.
  `/sensing/lidar/points_filtered`의 지면 제거된 원시 형상으로 차체 전면부터 0.30 m 영역을 확인한다.
  YOLO 분류·클래스 유무·기존 인지의 1 m 전방 제한에 의존하지 않는다.
- 차체·노면·측면 범위 밖 포인트를 제외하고, 입력의 원래 시각·TF·형식을 확인한다.
  정상적으로 비어 있는 최신 스캔과 입력 소실은 구분한다.
- 지면 분리 노드가 입력 시각을 `now()`로 덮어쓰던 한 곳도 수정해 원래 취득 시각을 보존한다.
- 전방 보조로 멈췄어도 후진으로 빠져나오는 것을 가로막지 않는다. 후방·측면 전체 LiDAR 보호를 주장하지 않는다.

LiDAR 보조는 기본 활성화하지 않은 시험용 opt-in이다. `enable_lidar_front_guard`로 감지 노드와 제어기를 함께 켠다.
기존 넓은 영역의 `raw_lidar_cost_enabled`를 통째로 켠 것이 아니다.

### 실제 기록에서 나온 차이

아래는 차체 전면 기준 결과다. ‘정지’는 **최종 속도 명령이 영점이 됨**을 뜻한다.

| 조건 | 차체 앞 10 cm | 20 cm | 30 cm | 40 cm |
| --- | --- | --- | --- | --- |
| 기존 radar 제외 설정 | 이동 명령 유지 | 정지 | 이동 명령 유지 | 이동 명령 유지 |
| radar 개선만 | 정지 | 정지 | 이동 명령 유지 | 이동 명령 유지 |
| LiDAR 보조만 | 정지 | 정지 | 정지 | 이동 명령 유지 |
| radar + LiDAR | 정지 | 정지 | 정지 | 이동 명령 유지 |

**병용 이유:** 기준 TF에서 radar 센서면 10 cm 지점은 차체 전면 약 1.95 cm 앞이지만 LiDAR 전방 시야 밖이다.
그 조건에서는 LiDAR 보조만으로 감지하지 못하고, 개선 radar와 병용 조건이 정지했다.
반대로 차체 앞 30 cm에서는 radar 센서면 기준 30 cm를 넘으므로 LiDAR 보조가 차이를 만들었다.

- [차체 전면 기준 PNG](evidence/service_usability_20261102/nearfield_comparison.png)
- [센서면 기준·사각 구간 PNG](evidence/service_usability_20261102/nearfield_sensor_face_comparison.png)
- [거리별 비교 GIF](evidence/service_usability_20261102/nearfield_comparison.gif)
- [원시 JSON·설정·소스/바이너리 해시](evidence/service_usability_20261102/nearfield_comparison.json), [CSV](evidence/service_usability_20261102/nearfield_comparison.csv)

23개 시나리오 × 네 조건 = **92개 기대 출력 검사 통과**다.
노면·차체 반환, 미분류 물체, 센서 소실·복구, 오래된 스캔, 정상 빈 스캔,
좌표 변환 실패·복구, 장애물 제거 후 출력 재개와 후진 이탈도 포함한다.
92/92는 구성한 논리 조건의 일치 수이지 실제 센서 정확도·장애물 탐지율이 아니다.

### 시험의 정확한 범위

실제 수정한 radar/LiDAR 노드와 최종 속도 제어기 네 개를 격리 ROS 환경에서 실행했다.
같은 합성 원시 포인트와 이상화한 radar 거리 입력을 공급했다.
기준 조건은 **현재 재빌드한 코드에 과거 제외 설정을 적용한 것**이지 과거 전체 바이너리가 아니다.
비교 입력 속도 0.2 m/s와 짧은 해제 대기는 시험 전용이며 운영·수동 속도 기본값을 바꾼 것이 아니다.
센서 논리를 분리하기 위해 이 fixture는 CAN·lanelet 등 다른 조건을 제외한다. 전체 override는 JSON에 기록했다.

PNG는 실제 기록으로 만든 비교표이고 GIF는 그 정적 시험 위치를 강조한 표다.
CARLA·실제 로봇의 동영상, 실제 제동 거리, 사람/물체의 실제 센서 탐지 성능은 측정하지 않았다.
기준 TF는 radar가 차체 앞보다 8.05 cm 뒤에 있는 기존 설정이며 새 2호기 장착 좌표를 추정해 넣지 않았다.
합성 포인트가 실제 센서의 수직 시야·최소 거리에서 수신될 것이라는 보장도 없다.
확인한 ‘재개’는 속도 명령 출력 재개이지 장시간 Nav2 미션 유지까지 검증한 결과는 아니다.

## 4. Ponytail 전체 자체 코드 점검

전체 파일 목록·중복 후보를 조사한 범위는 자체 코드·설정·시험 **826개 파일**이다.
외부 의존성·생성 파일·모델/음성 자산을 무작정 정리하지 않았다.

- 호출이 없는 기존 JSON 목적지 파서·MB 단위 디스크 래퍼·시뮬레이션 이벤트 래퍼와 미사용 import를 제거했다.
- Robot/Guest 배터리 변환을 기존 `battery_policy.py` 한 곳으로 모았다.
  실제 CAN `0.35F`를 Guest가 34%로 표시하던 불일치도 고쳤다.
  바로 아래 float32 값은 여전히 34%로 처리해 출발 기준을 완화하지 않는다.
- 104개 동일 파일 그룹은 대부분 단독 실행/배포용 설정 사본이라 보존했다.
- 기존 호환 callback·안전 검증·현장 보정·이전 `HH_...` 주석은 보존했다.
- 전체 코드를 일괄 재작성하거나 감소율 목표를 맞춰 기능을 삭제하지 않았다.
  새 감지 기능·시험이 추가되므로 저장소 전체 줄 수가 줄었다고 주장하지 않는다.

[전체 점검 보고서](evidence/service_usability_20261102/global_code_audit.md)와
[826개 파일 목록](evidence/service_usability_20261102/owned_code_inventory.tsv).
이는 전체 경로 목록/정적 후보 조사와 관련 코드의 집중 검토이며, 20만 줄 전체의 의미·모든 실행 경로 검증은 아니다.

## 변경 이력과 재현

수정한 주요 로직에는 `HH_261102 - English explanation` 형식으로 의도를 남겼다.
주요 파일은 아래와 같다.

| 영역 | 코드·설정 |
| --- | --- |
| 충전 화면 | `camrod_ui/camrod_ui_robot/assets/frontend/src/ChargingAccessScreen.js`, `App.js` |
| 충전 요청·상태 | `camrod_ui/runtime/python/camrod_ui/charging_access_api.py`, `ui_backend_node.py` |
| 내부 대기 위치·차체 범위 | `camrod_control/src/drop_zone_maneuver_controller_node.cpp`, `include/camrod_control/cargo_access_target.hpp` |
| radar | `camrod_sensing/src/radar_cost_grid_node.cpp`, `config/radar/cost_grid.yaml`와 bringup 사본 |
| LiDAR 보조 | `camrod_sensing/src/lidar_cost_grid_node.cpp`, `config/lidar/front_guard.yaml`와 bringup 사본 |
| 최종 정지 | `camrod_control/src/motion_cost_stop.cpp`, `cmd_vel_safety_gate_node.cpp` |
| 취득 시각 | `camrod_sensing/external/ground_segmentation_ros2/src/ground_segmentation_ros2_node.cpp`의 통합 패치 |
| 배터리 중복 제거 | `camrod_ui/runtime/python/camrod_ui/battery_policy.py`, `ui_guest_node.py`, `ui_backend_node.py` |

radar 거리 조절은 두 `cost_grid.yaml`의 `stop_candidate_max_ranges_m`에서 한다.
순서는 FRONT1, FRONT2, LEFT1, LEFT2, RIGHT1, RIGHT2, REAR이고 단위는 m다.
센서면 기준이므로 범퍼 여유 거리와 혼동하면 안 된다. 변경 후 설치 반영·노드 재시작이 필요하다.
LiDAR 보조 거리는 `front_guard.yaml`의 `front_guard_distance_m`이며 차체 전면 기준이다.

실행 비교 재현은 `camrod_sensing/test/test_nearfield_comparison_ros.py`,
시각자료 재생성은 증빙 폴더의 `render_nearfield_comparison.py`를 사용한다.
시험은 기존 ROS/interface 설치본을 의존성으로 사용한 별도 빌드이며 전체 작업 공간 클린 빌드나 ARM/Orin 시험은 아니다.
실제 CARLA 주행·충전 접점·물리 제동·현장 장착 위치 검증은 남아 있다.

## 최종 검사 결과와 발견한 기존 불일치

| 검사 | 결과 | 기록 |
| --- | --- | --- |
| UI·bringup·시각화 전체 Python 회귀 | 1,326 통과 / 기존 지도 검사 4개 실패 | [전체 보고서](evidence/service_usability_20261102/python_regression.xml) |
| React 전체 | 문구 간소화 후 20개 묶음, 338개 통과 | [UI 기록](evidence/service_usability_20261102/charging_ui/README.md) |
| 네이티브 control 전체 CTest | 13/13 묶음 통과 | [제어 보고서](evidence/service_usability_20261102/control_ctest.xml) |
| Radar·LiDAR·취득 시각 격리 시험 | 9/9 묶음 통과 | [센서 보고서](evidence/service_usability_20261102/front_guard_ctest.xml) |
| 충전 위치 이동 신규 검사 | 순수 정책 7개 + 실제 ROS 제어기 7개 통과 | [경로 수치·해시·범위](evidence/service_usability_20261102/cargo_access_native_verification.json) |
| UI 자산 | GLB·도색 포함 소스→빌드→설치 58개 일치 | `verify_frontend_assets.py --installed` |

충전 위치 이동의 합성 위치 피드백 시험에서는 기존 지도 목표까지 오차 0.169344 m로 정차했다.
허용 오차는 기존 0.20 m이며, 전체 이동에서 회전 명령은 0, 자동 주차 START는 0이었다.
이는 실제 차량이 아니라 실제 제어기에 공급한 합성 위치 피드백 시험이다.

전체 Python 회귀에서 발견한 네 실패는 아래의 **기존 지도/과거 자료 검사 불일치**다.
현재 `lanelet2_maps.osm`은 v27이고, 아래 검사 일부는 과거 v22 해시 또는 v26 사본을 기대한다.
해당 지도·사본·검사 파일은 `HEAD`와 변경 사항이 없으며 현재 지도 SHA-256도 기준 커밋과 동일하다.
이번 시험을 통과시키려고 지도 또는 기대 해시를 임의 변경하지 않았다.

- `test_lanelet_map_revision_sync.py::test_active_park_map_matches_the_current_user_revision`
- `test_module_readme_assets.py::test_map_v15_release_recovery_visuals_are_hash_guarded`
- `test_park_operating_points_assets.py::test_historical_operating_point_report_retains_map_v22_identity`
- `test_park_operating_points_assets.py::test_active_map_changes_only_approved_metadata_and_retired_zone_relation`

현재 지도 SHA-256: `6a2efc4d0393d923afcb0b0691572f4780763268dcdd92664fa4470c266d8cea`.
따라서 ‘전체 회귀 검사도 모두 통과’ 또는 ‘실제 로봇 사용 검증 완료’라고 보고하지 않는다.

## 7. 서비스 화면 문구 간소화

HH_261102 - Keep customer instructions brief while preserving safety warnings and existing confirmations.

충전·배달·호출·휴대폰 화면을 현재 상태, 한 문장 안내, 다음 행동 중심으로 정리했다.
긴 정책·내부 오류는 접힌 상세 안내로 옮기고, 35% 기준·사이트 확인·두 단계 호출 복귀·정지 확인은 유지했다.
이번 표시 변경 후 Python UI 1,045개와 React 338개가 통과했으며 UI 패키지도 다시 빌드·설치했다.
수정 내용, 변경 전후 PNG, 재현 방법과 확인 범위는
[서비스 화면 문구 정리](evidence/service_usability_20261102/ui_copy_simplification/README.md)에 모았다.
