# v2.3.0 준비: 변경 분리 및 실제 시험 기록

검증일: 2026-10-02. 이 문서의 단위 테스트와 실제 CARLA 주행 결과는 서로 다른 검증이다.
**B9 배송 왕복과 호출 왕복은 완료했다. B1–B13 전체 통과나 거리 정확도를 의미하지 않는다. 원격 push는 하지 않는다.**

## 변경을 어디에 넣는가

| 대상 | 포함 내용 | 제외 내용 |
| --- | --- | --- |
| `develop` | 네비게이션 화면, 상시 기본 지도, 10cm 초록 경계선, 수신 객체의 측정 크기 표시, 취소 확인/Guest 재호출, 미션·CAN 기록, 충전 완료 표시, 음성 방향 정책, 안전한 UI 빌드 게시 | CARLA 브리지·맵·제어기, 시뮬레이터 수동 속도 상향, 가상 환경의 운행시간 제한 해제 |
| `virtual/carla` | 위 공통 기능과 기존 시뮬레이터 연동, 독립 데이터 저장, CARLA 전용 제어 주기/준비상태 검사, 시나리오 도구·증빙 | 실제 로봇 데이터베이스 변경, 안전 ACK 우회, 토크 제한 해제 |
| `v2.3.0` | 최종 검증된 `develop` 커밋에 붙일 예정 | 미완료 시나리오를 완료했다고 표시하지 않음 |

두 작업트리는 별개다. 실제 시험은 `/home/hong/camrod_ws/src`의 `virtual/carla`에서 수행하며,
순수 코드 이식·검증은 별도 `develop` 작업트리에서 한다. 기존 `v2.2.9` 태그를 이동하지 않는다.

## 화면 변경 및 원리

- `/map/markers`의 기본 지도는 경로가 없을 때도 유지한다. 미션 경로만 파란색으로 별도 표시한다.
- 왼쪽/오른쪽 경계만 폭 **0.10m**의 초록색 면으로 그린다. 작은 지도는 **3px**이다.
  이는 UI 도색 폭이며 Lanelet 좌표, 통과 판정, 충돌·정지 정책은 바꾸지 않는다.
- 기본 카메라에서 로봇을 중앙에 배치하고 외관 보기에서만 비스듬히 본다.
- 배경 도로 폭과 나무는 읽기 쉬운 **예시 배경**이다. 실제 지도 경계·수신 경로와 구별한다.
- 객체는 수신한 LiDAR 관측 범위가 있을 때 그 크기로 표시한다. 측정 크기가 없으면
  미확인 표식으로 남긴다. 렌더링용 관측 크기를 기존 제어/회피 출력에 주입하지 않는다.
- 프런트엔드는 임시 폴더에서 빌드하고, 모든 이미지/번들을 게시한 뒤 index를 교체한다.
  열려 있는 브라우저를 위해 이전 해시 번들은 보존한다.
- 신규 변경 이유·제약은 해당 소스에 `HH_261002 - English explanation` 형식으로 기록한다.
  10월 1일에 작성된 기능의 `HH_261001` 등 기존 실제 날짜는 유지한다.

## 실제 CARLA B9 시험: 첫 시도

1. 월악산 v224 맵에 Ranger를 다시 생성하고 실제 Robot UI 버튼으로 B9 배송을 요청했다.
2. 드랍존 출발/방향 정렬 및 B9 경로 생성이 관측됐다. 생성 경로는 약 49.27m, 250점이었다.
3. 일부 이동 후 정지했다. 요청 속도는 약 0.556m/s였지만 제어 상태는
   `awaiting physical steering acknowledgement`, `brake=1`, `hand_brake=true`였다.
4. 정지한 상태를 도착으로 처리하지 않고 실제 중지 요청으로 미션을 취소했다.
   배송 완료·복귀·호출 왕복을 성공으로 기록하지 않았다.

### 증빙

- [실제 UI 경로 화면](evidence/virtual_carla/v230_b9_20261002/delivery_navigation.png)
- [실제 UI 일부 출발 구간 GIF](evidence/virtual_carla/v230_b9_20261002/delivery_departure_partial.gif)
  — 12초 원본 구간, UI 부분만 잘라낸 영상. 완주 영상이 아니다.
- [정지 시 UI와 CARLA 동시 화면](evidence/virtual_carla/v230_b9_20261002/physical_ack_stall_visible.png)
- [물리 제어기 정지 상태 JSON](evidence/virtual_carla/v230_b9_20261002/physical_control_stall.json)
- [UI 버튼·WebSocket 요청 기록](evidence/virtual_carla/v230_b9_20261002/delivery_dispatch.json)

원본 MP4와 고빈도 JSONL은 같은 로컬 폴더에 보존하지만 Git에는 요약·선별 자료만 넣는다.
`delivery_stall_dual.png`, `delivery_stall_carla_ui.png`는 CARLA가 다른 창 뒤에 가려진
초기 캡처이므로 동시 화면 증빙으로 사용하지 않는다.

## CARLA 실행 문제와 재시험 조건

| 항목 | 확인 내용 | 변경 범위 |
| --- | --- | --- |
| 제어 주기 | 상위 제어기 기본 0.02초, CARLA fixed delta 0.05초 | CARLA launch에서만 0.05초 지정; ACK/watchdog/토크 상한 유지 |
| 화면 부하 | 실제 스텝 약 4Hz; 알고리즘 중지 후 CARLA 창을 1280×720으로 줄인 측정은 약 15Hz | 뷰포트 크기만 변경; 센서 해상도·물리 시간 간격 변경 없음. 부하 조건이 달라 단독 인과 비교는 아님 |
| 재시작 준비 검사 | 정상적인 진행 중 STEP ACK 상태를 기존 정지 상태 검사에서 거부 | 별도 `/startup_ready` 추가; 최근 완료된 정확한 프레임 ACK 필요. 기존 엄격한 정지 검사·watchdog 보존, 실제 시작 통과 |

재시작 중 남았던 시험용 actor 18과 그 센서 19–33은 ID·부모·role을 확인한 후
명시적으로 제거하고 다시 생성했다. 서버의 맵 지형/FBX/Blueprint를 수정하지 않았다.

## 재시험: 배송 도착·복귀·후진 주차 완료

미션 세대값 `1790922474608001`, 저널 `2026-10-02 #006 B9 delivery`.

- 15:28:45.146 KST에 기록 시작, B9 진입/180도 회전 후 `WAITING_FOR_RETURN_REQUEST / ARRIVED` 확인.
- 실제 UI 복귀 버튼과 같은 세대값의 `usage_complete`를 확인했다.
- 복귀 중 map(7.81,45.00) 근처에서 약 40초 정체했으나 자력으로 재출발했다.
- 15:40:48.958 KST에 후진 주차 후 `DROP_ZONE_WAIT / READY`, 미션 비활성 및 저널 `completed` 확인.
- 기록상 전체 벽시계 시간 **723.81초(약 12분 4초)**. 이용/버튼 확인 대기와 정체를 포함하므로
  순수 이동 시간이나 일반적인 서비스 소요시간으로 해석하지 않는다.
- 배터리 80%, `charging_required=false`: 도킹은 이번 시험에 포함되지 않았다.

| 화면/데이터 | 자료 |
| --- | --- |
| UI·실제 CARLA 주행 동시 화면 | [PNG](evidence/virtual_carla/v230_b9_20261002/retry_delivery_driving.png), [GIF](evidence/virtual_carla/v230_b9_20261002/retry_live_navigation.gif) |
| B9 도착 | [PNG](evidence/virtual_carla/v230_b9_20261002/retry_delivery_arrived.png), [상태 JSON](evidence/virtual_carla/v230_b9_20261002/retry_delivery_arrived.json) |
| 복귀 주행 | [GIF](evidence/virtual_carla/v230_b9_20261002/delivery_return_live.gif) |
| 후진 주차 완료·기록 | [PNG](evidence/virtual_carla/v230_b9_20261002/retry_delivery_return_completed.png), [상태·저널 JSON](evidence/virtual_carla/v230_b9_20261002/retry_delivery_return_completed.json) |
| 새 호출 주행 | [PNG](evidence/virtual_carla/v230_b9_20261002/recall_navigation.png), [GIF](evidence/virtual_carla/v230_b9_20261002/recall_live_navigation.gif) |
| 호출 도착/적재 대기 | [PNG](evidence/virtual_carla/v230_b9_20261002/recall_arrived.png), [상태·저널 JSON](evidence/virtual_carla/v230_b9_20261002/recall_arrived.json) |

호출 미션은 새 세대값 `1790922474608002`, `GUEST_LOADING_WAIT / ARRIVED`와 실제
UI 적재 완료 요청을 확인했다.

### 최종 호출 복귀 판정

- 저널 `2026-10-02 #007 B9 recall`: 15:42:07.682 KST 시작,
  15:53:41.369 KST 완료. 전체 벽시계 시간 **693.69초(약 11분 34초)**.
- 중간 `RECALL_RETURN_WAIT`은 상태 전환 과정에서 관측됐으며, 그 상태만 보고 완료로 판단하지 않았다.
- 실제 복귀 후 `DROP_ZONE_PARKING` → `DROP_ZONE_WAIT / READY`, `PARKED`, `active=false` 확인.
  최종 위치 map(-8.48,40.63), 배터리 80%, 도킹 요청 없음.
- [호출 복귀 GIF](evidence/virtual_carla/v230_b9_20261002/recall_return_live.gif),
  [후진 주차 완료 PNG](evidence/virtual_carla/v230_b9_20261002/recall_return_completed.png),
  [완료 상태·저널 JSON](evidence/virtual_carla/v230_b9_20261002/recall_return_completed.json).
- 최종 저널에서 배송·호출 각각 `result=completed`, 기록기 `READY`, 수집 큐 유실 0을 확인했다.
  로드된 프런트엔드는 마지막 정지 보호 보강을 포함한 `main.2c03cd74.js`였다.
- 호출 기록의 135.71m도 아래 시간축 문제의 영향을 받으므로 정확한 이동거리라고 사용하지 않는다.

두 미션은 각각 별도 미션으로 생성·종료됐으며, 강제 도착 처리나 좌표 순간이동으로 완료하지 않았다.
모든 소요시간에는 이용 확인 대기, 회전·사이트 기동·주차 및 일시 정체가 포함된다.

마지막에는 미션이 없는 상태에서도 기본 지도·도로·0.10m 초록 경계가 남고 로봇이 중앙에
표시되는 것을 확인했다. 네 면 도색 로딩, 경로 정점 0, 중앙 투영 오차 약 0,
표시 전용 확인 중 변경 HTTP 요청 0을 기록했다.
[대기 기본 지도 PNG](evidence/virtual_carla/v230_b9_20261002/final_idle_map/idle_filled_roads_surroundings.png),
[외관 전환 GIF — 주행 아님](evidence/virtual_carla/v230_b9_20261002/final_idle_map/idle_filled_roads_camera_views.gif),
[렌더링 검증 JSON](evidence/virtual_carla/v230_b9_20261002/final_idle_map/idle_filled_roads_capture.json).

## 기록 확인에서 발견한 제한

기록기는 `READY`, `environment=simulation`, `raw_can_status=disabled`였고 배송/복귀 구간과
플랫폼 표본·상태 이벤트가 저장됐다. 실제 CAN 원본을 수집한 것이 아니다. 이번 시뮬레이터
플랫폼 표본의 `motor_rpm`, `motor_speed`, `motor_angle` 배열은 비어 있으므로 해당 저널로
휠별 실측 CAN 값까지 검증했다고 주장하지 않는다.

**거리 정확도는 실패/미검증이다.** 완료 미션에 저장된 137.70m는 시간축 문제의 영향을 받으므로
실측 이동거리로 사용하지 않는다. 첫 210초의 속도 적분은 68.255m, UI 위치 누적은 53.973m였다.
[원인과 수정 범위](release_v230_simulation_distance_clock_issue_20261002.md)에 상세히 기록했다.
기존 실증 통계와 새 저널 모두 이 CARLA 입력 시간축의 영향을 받는다. 원본은 변경하지 않았다.

## 코드 검증과 아직 남은 검증

- 현재 `virtual/carla` UI·음성 Python: **903 passed**.
- 현재 `virtual/carla` 프런트엔드: **194 passed**, production build `main.2c03cd74.js` 성공.
- 별도 순수 `develop`: Python **789 passed**, 프런트엔드 **194 passed**, build `main.11090c1b.js` 성공.
- CARLA 준비상태·ROS 경계·launch/script 집중 테스트 **133 passed**.
- 관측 범위 C++ **8 passed** 및 fusion 문법 검사 통과; 전체 패키지 링크 성공을 뜻하지 않는다.
- 정지 확인창을 연 뒤 미션 세대값이 바뀌면 창을 닫고 이전 확인으로 새 미션을 정지시키지 않도록
  두 브랜치에서 보강했다. 취소/정상 확인/세대값 누락 경계도 프런트엔드 회귀 테스트에 포함했다.
- 이 수치는 소스 단위·계약 테스트이며 실주행, 실제 CAN, 스피커 출력을 증명하지 않는다.
- 이 PC의 `SDL2_mixer` 개발 패키지가 없어 음성 패키지 전체 재빌드/실제 스피커 확인은 미완료다.
- 실제 휴대폰 Guest UI, 모든 구역, 충전 접촉, 저전압 자동 복귀는 이번 실주행 검증 범위가 아니다.

## 커밋·태그·push 판정

- `develop`: 기능 `11034aa6c`, 모델 출처 문서 보강 `e2069007a`.
- `virtual/carla`: 소스 통합 `c8035a892`; 증빙 문서는 후속 커밋으로 분리.
- 두 브랜치 모두 상세 영어 커밋 메시지와 소스 내 날짜 주석을 남겼다.
- **v2.3.0 생성 및 실제 push 보류.** 거리 시간축 문제와 기존 pre-push 훅의 이력 정책 충돌이 남았다.
  [push 사전 점검 결과와 준비 명령](release_v230_push_preflight_20261002.md)을 참고한다.
- 이번 기능 밖 기존 로컬 주석 정리·외부 소스·자료는 삭제하거나 함께 stage하지 않았다.

## 배포 주의

실제 로봇의 기록 DB, 과거 태그, 원격 브랜치는 이 검증으로 삭제하거나 덮어쓰지 않는다.
실제 CAN 원본 수집은 확인된 CAN 인터페이스에서만 선택 활성화한다. 시뮬레이터 표본을
실제 CAN 프레임이라고 표기하지 않는다. 브랜치별 커밋과 최종 결과 확인 후 push한다.
