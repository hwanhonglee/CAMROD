# 서비스 선택 UI와 후진 주차 우선 정책 — 2026-09-09

## 변경 범위

기준은 develop `bfc7047597c1dc490f8cffbfd87bca438b2a38f7`이다.
기존 `v2.2.5` 태그를 이동하거나 새 태그를 만들지 않는 후속 개선이다.

- `worak-test`의 새 서비스 선택 화면과 배송·호출·도킹 메뉴를 선택 반영한다.
- Guest 호출 후 Robot 최종 확인에서 복귀 요청 세대도 함께 기록해 이후 복귀·주차 상태를 같은 임무에 연결한다.
- 기본 `parking_method=auto`에서는 배터리와 관계없이 후진 주차를 먼저 완료한다.
- 신선한 실제 후진 주차 완료 상태를 확인한 뒤, 충전이 필요하거나 명시적 Dock 요청이 있으면 AprilTag 제어기로 인계한다.
- 후진 주차 목표 반경 안에서 새 요청을 받은 경우 불필요하게 더 후진하지 않고 정지·완료 처리한다. 기존 허용오차를 넓히지 않는다.
- 배터리 float32 분율의 백분율 변환을 제어기와 일치시킨다. 정확한 35%와 바로 아래 float32 값을 구분한다.
- Dock 응답의 `parking_requested_final_method`는 요청 의도이며 실제 제어기 소유권은 주차 상태 토픽으로 결정한다.

## 유지한 계약

1. 배터리 정보 미가용은 이전 값을 유지하지 않고 unknown(-1)으로 갱신한다.
2. 25% 미만 긴급 복귀와 25~35% 임무 후 사용자 복귀 확인 정책은 변경하지 않는다.
3. 후진 주차 뒤 35% 이상이며 Dock 요청이 없으면 비충전 주차 상태로 대기한다.
4. 제어기 인계는 CANCEL/START 응답, 요청 세대, 상태 신선도 검사를 유지한다.
5. 비상정지·수동 소유권·충전·플랫폼 입력 검사를 우회하지 않는다.
6. 명시적인 `reverse` / `apriltag` 단독 디버그 모드는 유지한다.
7. GNSS 장착값, TF, 지도, 주행 속도·토크와 하드웨어 배포 설정은 이 업데이트에서 변경하지 않는다.

## 코드 위치

- `camrod_control/include/camrod_control/parking_selection_policy.hpp`: 초기/최종 주차 방법 선택 분리.
- `camrod_control/src/parking_dispatcher_node.cpp`: 후진 완료 검증과 최종 도킹 인계.
- `camrod_control/src/reverse_parking_controller_node.cpp`: 이동 명령 전 기존 XY 완료 조건 확인.
- `camrod_ui/runtime/python/camrod_ui/ui_backend_node.py`: 복귀 요청 세대, SOC 수치 처리, Dock 응답 의미.
- `camrod_ui/camrod_ui_robot/assets/frontend/src/`: 새 서비스 메뉴와 상태 처리.

## 검증 범위

이 순수 조합에서 아래 검증을 수행했다.

| 검사 | 결과 |
| --- | --- |
| reverse completion 4 + reverse controller 21 + selection policy 13 + dispatcher 17 | native 55 PASS, 실패 0 |
| 전체 `camrod_ui/test` | 392 PASS, 실패·skip 0 |
| Robot UI production build | 성공, `main.6a489fbd.js` / `main.f40e0eb6.css` |
| 선택 변경의 whitespace 검사 | `git diff --check` 통과 |

빌드한 제어기 실행파일은 `reverse_parking_controller_node`, `parking_dispatcher_node`이다.
UI source build에서 postbuild/install 동기화는 실행하지 않았다.
native 시험은 기존 격리 ROS domain 188/189와 localhost 모드로 실행했으며 운영 노드에 연결하지 않았다.
코드 회귀/빌드 성공은 모든 사이트·모든 UI 화면·실물 충전기의 현장 검증 완료를 뜻하지 않는다.
별도의 도킹 초기 이격/재시도 확장이나 시험용 런타임은 이번 순수 업데이트에 포함하지 않는다.
