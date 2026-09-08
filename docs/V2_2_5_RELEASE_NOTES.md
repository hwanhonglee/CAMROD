# CAMROD v2.2.5 — 순수 주차·경로·UI 개선

기준 develop: `f3a177038567f8f1b568e2f49d4c127bc298d9a5` (v2.2.4).
이번 제품 코드 검증 기준: `91158175102a2bbf86624f5c5b726e5e272f8b90`.
후속 릴리스 커밋은 문서·검증 결과를 정리하며 패키지별 독립 버전을 일괄 변경하지 않는다.

**소프트웨어 개선 릴리스이며, 전체 사이트·충전 도킹·실차 기능 검증 완료 선언이 아니다.**
외부 시뮬레이터의 어댑터·센서 프로필·맵·모델·물리 튜닝·시험 구동기는 포함하지 않는다.

## 1. 기존 develop에서 그대로 이어받은 기능

- 충전/대기 장소 안에서 요청을 받아 출차하는 지도 기반 위치 판단과 출발 절차.
- 현재 사이트 출구 위치에서 복귀 경로 생성. 이전 진입 XY로 되돌아가는 것을 필수로 하지 않는다.
- B1–B10 Recall의 첫 확인 → 진입·180° 회전 → 최종 적재 확인 절차와 미션 소유권 검사.
- `auto` 주차 선택: 신선한 SOC 35% 이상은 비충전 후진 주차, 미만/미확인은 충전 도킹.
  사용자가 명시적으로 Dock를 누르면 SOC와 별개로 도킹을 요청한다.
- 해당 상태에 맞는 음성 정책 및 새 요청/복귀 UI. 이번 릴리스에서 모든 음성을 새로 만든 것이 아니다.

## 2. 이번에 실제 수정한 코드

| 패키지·파일 | 변경 내용과 이유 | 유지한 조건 |
| --- | --- | --- |
| `camrod_control/launch/parking.launch.py` | detector include를 `GroupAction(scoped=True)`로 격리. detector의 `parameter_file`이 부모 주차 YAML을 덮어쓰던 문제 수정 | 주차 방법·충전 정책 및 controller YAML의 설정값 |
| `camrod_control/src/reverse_parking_controller_node.cpp`, `include/camrod_control/reverse_parking_completion.hpp` | 축 방향 정지 범위에 먼저 들어와도 XY 완료 원 밖이면 도달 가능한 제한된 최종 접근을 허용 | XY 0.25m, 횡방향 실패·목표 평면 통과·최대 거리·timeout·충전 시 정지 |
| `camrod_planning/src/local_path_extractor_node.cpp`, package/bringup `local_path_extractor.yaml` | 경로 종료 반경 0.25→0.05m. 종점에서 기존 연속 경로 점을 보존해 너무 이른 빈 local path를 방지 | 새 경로 점을 만들어내지 않음. 유한값/큰 점프 검사와 lanelet·장애물 안전 조건 유지 |
| `camrod_localization/config/source/input_adapter.yaml` | package heading trim −90°→−92°로 기존 bringup 배포값과 동기화 | 안테나 X=0/Y=+0.45m 및 중심 보정 수식은 변경 없음 |
| `camrod_bringup/config/sim/fake_sensors.yaml` | 기존 내장 시험 센서의 heading bias +90°→+92°로 위 trim과 맞춤 | 실차 센서를 대체하지 않으며 외부 시뮬레이터 연동 코드가 아님 |
| `camrod_ui/camrod_ui_guest/assets/guest_frontend/index.html` | 새 WebSocket 연결에서 revision 기준을 다시 시작하고, 이전 socket의 늦은 callback/timer를 차단 | 같은 연결의 오래된 메시지 거부, 실제 서버 미션 상태·소유권 |
| `camrod_ui/camrod_ui_robot/assets/frontend/src/App.js` | Guest 호출 안내 4종을 접수 단계의 비차단 header 안내로 변경. 중지·최종 완료 버튼을 가리던 전면 알림 제거 | 정확한 site/owner/intent/generation 및 두 단계 확인 권한 |
| 같은 `App.js` | 미수신 튜닝값 0.50 대신 `—`. 편집값과 ACK 적용값 분리, 미가용/처리 중 비활성화, 늦은 응답·중복 요청 방지 | 기존 실제 GET/POST와 0.05–2.0rad/s 유효 범위; 제어기 gain 변경 없음 |
| `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py` | 완료한 station 목표의 표시 수명만 종료해 주차 후 `목표 수신` 잔류 방지 | 실제 미션/목표를 임의 삭제하지 않음. 새 요청·수동 목표·안전 오류 우선 |
| Robot UI 및 회귀 | 현재 안전/오류 문구가 일반 이동 안내보다 우선하도록 정리. 배터리 heartbeat가 적재 완료 화면을 지우지 않는지 확인 | 안전 경고를 정상으로 숨기지 않음 |
| bringup/control/planning 회귀·문서 | 현행 지도 fixture, package/bringup 설정 및 실제 departure 정책과 검사 일치 | 지도 좌표·지형·전체 경계 해제 변경 없음 |

Nav2 XY 0.10m, 주차 전 local XY 0.20m, 최종 reverse XY 0.25m는 이번에 넓힌 값이 아니다.
local-path 0.05m는 경로 유지 조건이며 Nav2 도착 승인 신호를 대신하지 않는다.

## 3. 검증 기록

새 검사 원본은 [검증 목차](evidence/v2_2_5_20260908/README.md)에 보관한다.
UI 364개와 지정 계약 119개가 통과했다. 별도 집중 검사는 새 C++ 회귀 35개와
소스 동일성을 확인한 경로 runtime 재실행 17개가 통과했다. 모두 failure/error/skip 0이며,
중복 범위가 있으므로 고유 535개 기능 성공으로 합산하지 않는다.
빌드 재사용·기존 ROS 의존성·격리 도메인·원본 JUnit은 위 목차와 JSON 요약을 따른다.
이전 [v2.2.4 검증 기록](V2_2_4_VERIFICATION_20260908.md)의 `final ddd59967` 및
당시 52개 결과는 그 시점의 기록이며, 최신 UI/경로 변경 전체 검증으로 재해석하지 않는다.

제품 코드 회귀, 실제 UI 동작, 주행 결과는 서로 다른 증거다.
최신 통합 시험에서 확인된 것은 한 차례 B1 Guest 요청 → Robot 최종 확인 → 복귀·비충전 후진 주차다.
전체 B1–B13 일반 배송·Robot Recall·Guest Recall과 모든 UI/음성 상태를 통과했다는 뜻은 아니다.

## 4. 남은 검증·알려진 제한

- 선택 도킹은 요청·시작 이후 접근 중 태그 시야 이탈로 최신 통합 시험을 완료하지 못했다.
  정렬 제어와 센서/태그 배치 중 원인을 더 분리해야 한다. 도킹 완료를 보장하지 않는다.
- 비충전 PARKED에서 새 요청으로 출발, 주차/도킹 진행 중 새 요청으로 전환 및 모든 사이트 왕복은 추가 검증 대상이다.
- 한 B1 복귀의 성공은 과거 문제 lanelet 경계 조건을 재현·해결했다는 증거가 아니다.
  경계 및 실측 장애물 안전 검사를 일괄 해제하지 않았다.
- GNSS 중심 보정은 `p_center = p_antenna - R(yaw) * offset`이다.
  현재 설정은 전방 장착이 아닌 X=0/Y=+0.45m다. 실제 전방 안테나 치수·축·heading 시간 정합은 별도로 확인해야 한다.
- 90° 주차 yaw 정렬 이후 XY를 다시 보정하는 단계는 추가하지 않았다.
  허용 오차 조정이나 한 번의 후진 주차 성공으로 실차 중심 보정을 인증하지 않는다.
- 전체 센서 갱신률/정확도, 실제 하드웨어 튜닝 POST, 모든 음성·오류 화면, 충전기 및 ARM64 실차 검증은 남아 있다.

## 5. 빌드와 적용

대상 플랫폼의 기존 ROS 2 Humble 의존성을 준비한 뒤 저장소의 `./colcon_build.sh`로 빌드한다.
특히 launch scope 및 설정 수정은 설치·프로세스 재시작 후 적용된다. 실행 중인 노드는 자동으로 바뀌지 않는다.
실차에서는 GNSS 장착 치수와 heading trim을 먼저 확인하고 제한된 공간에서 출발·복귀·정지부터 검증한다.
기존 데이터·지도·패키지 버전을 임의 초기화할 필요는 없다.
