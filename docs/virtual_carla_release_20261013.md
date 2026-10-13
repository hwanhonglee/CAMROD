# virtual/carla — 2026-10-13 정리 및 실행 안내

<!-- HH_261013 - Publish tested simulator adaptations, exact dependency boundaries and qualified evidence without changing native defaults or version tags. -->

이번 반영 대상은 **CAMROD 저장소의 `virtual/carla` 하나**다. 기존 v2.3.1 연동 커밋
`be530cfc4f1123294ba2f38d5f27f627bc3b7e07` 이후의 로컬 수정·주석·시험 자료를 반영한다.
`develop`, 과거 태그, 원격의 기존 커밋 이력은 변경하지 않는다.

## 실제로 확인한 범위

[B1–B13 자료 목록](evidence/virtual_carla/releases/20261013/README.md)에 각 사이트의
실제 화면 PNG/GIF, 시간·거리, 분석, 원시 trial 기록, supervisor 및 해시가 있다.
원시 `trial.json.gz`는 원본 JSON을 손실 없이 압축한 것이며, 압축 해제한 바이트의
SHA-256을 원본과 대조했다. 원본 시험 파일은 수정하거나 삭제하지 않았다.

- 선택한 완료 사례 13개: V5 9개, V6 2개, V7 1개, V9 restart1 1개.
- 확인한 흐름: 배송 출발 → 해당 사이트 진입·서비스 → 도로 **전진 복귀** → **후진 주차**.
- 최종 상태: reverse controller/dispatcher `PARKED`, command gate `STANDBY`.
- 각 사례 collision 기록 0, trial/capture 종료 코드 0/0.
- 최종 station 오차 0.206369652–0.233476055 m, 기존 0.25 m 이내.
- 선택 사례 시간 합계 4827.259922 s, 물리 XY 표본 거리 합계 2788.748913 m.

**최신 한 설정으로 13곳을 다시 일괄 실행했다는 뜻은 아니다.** 연속 10 km 실증,
recall 전체 구역, 충전 도킹, 실차 성능·안전 인증 또는 무한 반복 성공을 이 자료만으로
완료 처리하지 않는다. 과거 실패와 GPU/인프라 중단은 상세 보고서에 구분해 보존했다.

## 수정 내용과 핵심 코드

| 목적 | 반영 내용 | 핵심 파일 |
| --- | --- | --- |
| 도로 후진 복귀 방지 | B1–B10은 사이트 내부에서 출발 방향을 복원한 뒤 전진 loop로 복귀. B11–B13의 roadside 전진 복귀와 마지막 후진 주차는 분리 | [site catalog](../camrod_carla_adapter/config/camping_sites_carla_forward_return.yaml), [maneuver controller](../camrod_control/src/camping_site_maneuver_controller_node.cpp) |
| 회전 중 중심 이탈 대응 | 회전을 멈추고 정해진 중심으로 재정렬한 뒤 같은 목표 yaw를 재개. B9/B10만 center 0.015 m·trigger 0.035 m 선택, B10만 dwell 0.4 s·recovery cap 20 선택 | 위 controller/catalog, [C++ regression tests](../camrod_control/test/test_camping_site_recall_return.cpp) |
| 수동 명령 전달·상태 표시 | CARLA site profile의 전후·측면 최대 2.0 m/s를 UI·adapter·backend에 일관되게 전달. gate의 실제 거절 이유 표시, 중단 미션은 명시적 재개 | [site launch](../camrod_carla_adapter/launch/camrod_carla_develop_site_geometry.launch.py), [command mapping](../camrod_carla_adapter/src/camrod_carla_adapter/command_mapping.py), [manual panel](../camrod_ui/camrod_ui_robot/assets/frontend/src/ManualDrivePanel.js) |
| 서비스 구역 수동 복구 | 명시적으로 허용한 site/access polygon에서만 제한된 정적 지도 복구 판단. 전역 boundary 해제 아님 | [manual service geometry](../camrod_carla_adapter/config/manual_service_access.yaml), [safety gate](../camrod_control/src/cmd_vel_safety_gate_node.cpp), [cost policy](../camrod_control/src/motion_cost_stop.cpp) |
| 도로 속도 | CARLA site의 clear-road 목표 4 km/h. 회전·장애물·접근 감속은 유지하며 최저 속도 보장은 하지 않음 | [Nav2 overlay](../camrod_carla_adapter/config/nav2_carla_reverse_return.yaml) |
| 조경 객체 유지 우회 | 원본 OSM 대신 별도 감사한 shoulder-detour 지도 명시 선택. 실제 CARLA 객체 삭제나 원본 map 전체 확장 아님 | [map and generation record](../camrod_carla_adapter/config/object_preserving_detour/README.md) |
| 4륜 물리 controller 연동 | simulator-only crab traction/yaw correction·bounded zero-turn yaw integral를 명시적으로 활성화 | site launch 및 아래 외부 controller companion |
| UI 브랜드·자산 | CAM-Geni / 국립공원 / AVGenius 표시, 기존 Ranger navigation 모델·도색 자산 유지 | [branding](../camrod_ui/camrod_ui_robot/assets/frontend/src/serviceBranding.js), [tracked model](../camrod_ui/camrod_ui_robot/assets/frontend/public/models/ranger-navigation.glb) |
| GPU profiling crash 대응 | 선택한 시험에서만 `CAMROD_CARLA_DISABLE_GPU_PROFILER=true`로 timestamp profiling 비활성화. 렌더링·물리 끄기 아님, GPU 드라이버 문제 해결 보장 아님 | [run.sh](../scripts/virtual_carla/run.sh) |
| 주석·테스트 정합성 | 자체 코드의 역사적 날짜를 유지한 English `HH_YYMMDD - ...` 주석 정리. 센서 mount·station pose·map lineage·runtime signature의 낡은 테스트 기준 수정 | adapter/control/UI tests 및 자체 코드·설정 |

새 hook들은 기본 비활성 또는 CARLA 전용 wrapper에서 선택한다. 공용 controller에
선택적 hook이 들어갔다는 사실과 기본 설정을 구분해야 한다. 이 브랜치를 native
`develop` 변경으로 통째로 이식하면 안 된다. EStop, 신선도, 장애물 확인, rotation
hard guard 0.05 m, episode 90 s, 최종 주차 tolerance 0.25 m를 성공을 위해 풀지 않았다.

B1-only 실험 YAML에 잘못 섞인 B5 복귀 설정은 제거했다. 실제 선택하는 aggregate
forward catalog에는 필요한 B5 설정이 그대로 있다. 외부 라이브러리와 무관한
생성 결과·임시 로그는 이번 변경에 추가하지 않는다.

## 외부 의존성: CAMROD clone만으로 모두 준비되는 것은 아니다

실제 시험에 사용한 외부 Ranger controller의 일부 변경은 별도 저장소에 아직 push되지
않았다. 이번에는 그 저장소를 임의로 변경하지 않고 **정확한 적용 패치**를 함께 제공한다:
[companion README](../camrod_carla_adapter/companion/ranger_controller_20261013/README.md).

companion은 Ranger base `10af3fcaf8137c7552486a7ba7e2000c203a0fe3`의 지정한 19개 파일에
대해서만 before/after SHA와 mode를 확인한다. 기본 명령은 read-only check,
명시적 `--apply`만 파일을 바꾼다. 이미 적용된 이 PC는 `ALREADY_APPLIED`가 정상이다.
임의 reset/clean/외부 commit/push는 수행하지 않는다.

CARLA 0.9.15 / UE 4.26 환경, physical 4-wheel RPC를 포함한 빌드, Ranger blueprint,
표준 ROS bridge, **시험에 사용한 준비된 Woraksan world·collision asset**도 필요하다.
이번 CAMROD commit은 UE 바이너리나 수정된 지형 `.uasset` 전체를 포장한 배포본이 아니다.
따라서 다른 PC의 원본 world에서 곧바로 동일한 모든 구역 통과를 보장하지 않는다.
공개된 historical receipts는 경과 증거이며 새 host/server/actor의 runtime gate가 아니다.

## 실행 순서

이미 환경이 준비된 이 PC에서는 각 터미널에서 다음 설정을 사용한다. 로컬 설정 파일은
저장소 바깥의 host-specific 파일이며 다른 PC에 그대로 복사할 경로가 아니다.

```bash
cd /home/hong/camrod_ws/src
source /home/hong/camrod_ws/config/virtual_carla.env
```

다른 PC는 실제 경로의 `RANGER_CARLA_ROOT`, `CARLA_ROOT`, `UE_ROOT`, ROS overlay와
prepared world를 먼저 선택한다. 기존 [setup](../scripts/virtual_carla/setup.sh)과
[build](../scripts/virtual_carla/build.sh)는 독립 Ranger overlay → CAMROD/UI 순서로 준비한다.
controller companion 적용 후 rebuild·offline test 및 새 source-bound gate가 필요하다.
Ranger의 `bootstrap/76_create_physical_gate.sh plan`이 해당 환경의 acceptance 순서를
출력한다. 과거 자료를 복사하거나 해시를 덮어써서 gate를 통과시키지 않는다.

아래는 준비·gate 검증이 끝난 환경에서 사용하는 명령이다. **각 장기 실행 명령은
서로 다른 터미널에서**, 동일한 설정을 source한 후 순서대로 실행한다.

```bash
# 실제 host 경로 및 prerequisite 점검
./scripts/virtual_carla/run.sh doctor

# 준비된 시험 world와 별도 lanelet map을 선택해야 한다.
# 이 PC의 virtual_carla.env는 해당 world와 아래 detour를 이미 선택한다.
export CAMROD_LANELET_MAP="$PWD/camrod_carla_adapter/config/object_preserving_detour/lanelet2_maps_carla_shoulder_2285.osm"
export CAMROD_CARLA_CAMPING_SITES_YAML="$PWD/camrod_carla_adapter/config/camping_sites_carla_forward_return.yaml"

# 터미널 1 → 2 → 3 → 4 → 5
./scripts/virtual_carla/run.sh server
./scripts/virtual_carla/run.sh bridge
./scripts/virtual_carla/run.sh pacer
./scripts/virtual_carla/run.sh spawn
./scripts/virtual_carla/run.sh camrod-site-geometry

# 선택: UI 창 / 실제 Ranger 따라가는 시점
./scripts/virtual_carla/run.sh operator-ui
./scripts/virtual_carla/run.sh spectator
```

엄격한 native/develop parity 확인은 `run.sh camrod`이며, 현재 개선된 시뮬레이션 시험은
`camrod-site-geometry`다. 두 profile을 혼동하면 기본 수동 cap과 torque/yaw opt-in이 달라진다.
수동으로 이동시킨 뒤 명령을 놓는 것만으로 자동주행을 붙이지 않으며 UI의 재개를 명시한다.

## 커밋 전 확인

- React: 19 suites / 317 tests 통과; production build `Compiled successfully`.
- UI public assets 58개 확인: navigation GLB와 front/side/rear livery 포함.
- C++ control: CTest 14개 test target 모두 통과.
- 수정한 runtime signature/matrix/recorder fixture: 231 tests 통과.
- catalog/sensor/publisher 집중 검사: 163 tests 통과 (환경의 Matplotlib Axes3D 경고 1개).
- launch/runtime/map/manual/UI/companion 집중 검사: 314 tests 통과.
- companion 안전 적용 검사: 51 tests 통과; 외부 Ranger HEAD/index/worktree 불변 확인.
- map-v27 fixture 검사는 Git 이력이 없는 경우도 10 tests 통과.

위 Python 묶음에는 중복 대상이 있으므로 숫자를 합쳐 유일한 전체 test 수라고 하지 않는다.
초기 넓은 검사에서 36개 실패·1122개 통과를 확인했고, 낡은 테스트 계약과 B1 실험 catalog의
scope 오류를 수정한 뒤 관련 대상을 다시 통과시켰다. 전체 넓은 묶음을 같은 명령으로 다시
완주한 결과라고 표기하지 않는다. 당일 새 CARLA driving trial은 실행하지 않았으며,
공개 PNG/GIF는 10월 12일 실제 주행 capture다.

주석 정리 때문에 source-bound SHA는 과거 receipt와 달라질 수 있다. archival receipt를
수정하지 않았으며, 다음 runtime 적용은 새 빌드·source 검증·gate를 기준으로 한다.
