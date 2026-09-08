# 2026-09-08 요청별 기능·실제 자료·릴리스 정리

## 결론과 버전

**전체 기능 검증은 미완료다.** 순수 CAMROD 개선은 `develop` 및 `v2.2.5`로
공개했으며, 이 브랜치는 그 순수 개선 위에 CARLA 연동·시험 도구를 유지한다.

- 순수 릴리스: `bfc7047597c1dc490f8cffbfd87bca438b2a38f7`, 태그 `v2.2.5`.
- 실제 시험 runtime/driver: `067568ecfe411a5cc31844fa84696220da879249`.
  당시 순수 코드 `91158175102a2bbf86624f5c5b726e5e272f8b90`, actor98,
  Robot UI `main.60f92f35.js`.
- 릴리스 문서 병합·증거 export를 위해 생긴 후속 커밋을 이미 실행한 runtime으로 재표기하지 않는다.
- 최신 통합 시험의 승인된 주행은 **B1 Guest recall → Robot 최종 확인 → 복귀·비충전 후진 주차 1건**이다.
  선택 도킹은 실패했다. 나머지 38개 주행은 미승인이다.

## 오늘 요청한 기능만 정리

| 요청 | 구현/수정 | 실제 검증 및 남은 일 |
| --- | --- | --- |
| 최신 develop 기능 이식 | v2.2.4 기반 순수 변경을 통합했고 v2.2.5 순수 릴리스도 병합 | 코드·설정·회귀 확인. 모든 기능 E2E 통과는 아님 |
| 충전/drop zone에서 새 호출로 출차 | 기존 지도 기반 출발 조건과 소유권 유지 | B1 Guest recall 출차 PASS. 최신 일반 배송·Robot recall·비충전 PARKED 출발 및 여러 초기 위치는 재검증 필요 |
| 복귀 후 일반 후진 주차 | 주차 launch YAML scope 격리, 축 방향 정지와 XY 완료 조건 일치 | B1 복귀 reverse PARKED/DROP_ZONE_WAIT/비충전 PASS. XY 오차 0.232110449m |
| 후진 주차 뒤 필요할 때 Dock | 실제 UI 버튼·확인창·기존 API·AprilTag controller 사용 | 요청과 접근은 동작. 태그 검출 소실 후 ERROR → STOP. 충전 완료 FAIL |
| lanelet 경계에서 주차/종점 정지 | local path 종료 0.25→0.05m 및 기존 종점 점 보존, 제한된 reverse 최종 접근 | B1 완주. 과거 문제와 동일한 경계 접촉 조건의 재현 검증은 없음. 안전 경계 일괄 해제는 하지 않음 |
| 90° 회전/전방 GNSS 중심 보정 | 기존 lever-arm 수식 확인, heading package/mirror 동기화 | 현재 X=0/Y=+0.45m. 실차 전방 X/Y 실측·시간 정합은 미확인. 회전 후 XY 재정렬 단계는 없음 |
| Guest/Robot UI·문구·음성 | Guest 재연결, 비차단 알림, 완료 표시 수명, 응답받은 튜닝값만 표시 | 실제 B1 첫 Guest/최종 Robot 확인, 재연결·설정 미가용 화면 확인. 전체 음성/오류 화면/하드웨어 설정 적용은 미완료 |
| 자료·시간·거리 및 전체 시나리오 | 실제 PNG/GIF·명령 ACK·native/strict 결과·휠/시간거리 기록 | 최신 성공 1건과 실패 1건 자료 확보. B2–B13 등의 성공 자료는 만들거나 과거 것으로 대체하지 않음 |

이 표의 ‘출차’는 출발 상태에서 요청을 처리하는 경우다. **이미 주차/도킹 제어 중인 상태를 새 요청으로
중단·전환하는 모든 경우까지 검증했다는 의미가 아니다.**

## 바로 확인할 수 있는 실제 자료

원본을 변경하지 않은 [휴대 가능한 증거 목차](evidence/virtual_carla/v2_2_5_20260908/README.md)와
[파일별 출처·해시](evidence/virtual_carla/v2_2_5_20260908/manifest.json)를 제공한다.
원본 JSON 안의 절대 경로는 provenance 보존을 위해 그대로 둔다. 파일을 열 때는 위 목차의 상대 링크를 사용한다.

| 자료 | 링크 / 해석 |
| --- | --- |
| B1 성공 주행 PNG·GIF | [PNG](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/visual/representative_contact_sheet.png), [GIF](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/visual/representative_motion.gif) |
| Guest 첫 확인·Robot 최종 확인 | [Guest](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/ui/first_guest.png), [Robot](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/ui/final_robot.png) — 실제 버튼과 새 미션-bound ROS 요청을 원본 native가 기록 |
| 완료 후 홈·비충전·Dock 버튼 | [실제 Robot 화면](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/ui/terminal_robot.png) — CPU 98.7% Critical 표시도 원본에 보존 |
| 성공 판정 및 시간·거리 | [strict](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/strict.json), [native](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/native.json), [metrics](evidence/virtual_carla/v2_2_5_20260908/b1_guest_recall/metrics.json) |
| 도킹 실패 PNG·GIF | [PNG](evidence/virtual_carla/v2_2_5_20260908/optional_docking/desktop/representative_contact_sheet.png), [GIF](evidence/virtual_carla/v2_2_5_20260908/optional_docking/desktop/representative_motion.gif), [FAIL 결과](evidence/virtual_carla/v2_2_5_20260908/optional_docking/result.json) |
| 태그 시야 조사 | [실제 후방 PNG](evidence/virtual_carla/v2_2_5_20260908/optional_docking/investigation/actual_rear.png), [ROS 출처](evidence/virtual_carla/v2_2_5_20260908/optional_docking/investigation/actual_rear.json) — STOP 이후 별도 촬영이며 실패 순간의 동일 프레임은 아님 |
| Guest 재연결 | [수정 전](evidence/virtual_carla/v2_2_5_20260908/ui/guest_before/actual.png), [새로고침 없는 서버 재연결 후](evidence/virtual_carla/v2_2_5_20260908/ui/guest_reconnected/actual.png) |
| 설정 미가용·진단 8탭 | [설정 표시](evidence/virtual_carla/v2_2_5_20260908/ui/tuning/actual.png), [탭별 실제 관측 JSON](evidence/virtual_carla/v2_2_5_20260908/ui/diagnostics/observations.json), 개별 PNG는 증거 목차 참조 |

B1 수치: 총 **868.907초 / 173.935410m**, 편도 247.615초/81.125159m,
첫 확인부터 진입·회전·최종 확인·복귀·주차까지 621.292초/92.810251m.
충돌 이벤트 0, 최종 속도 0.003376m/s. 단계 wall 1056.452초는 준비·영상 변환을 포함하므로 주행 시간과 구분한다.
대표 GIF는 약 7초의 실제 구간 요약이며 14분 29초 전체 영상이 아니다.

Dock는 마지막 검출 거리 0.427m에서 검출을 잃고 대기 후 오류로 끝났다(설정 stop 0.400m).
실패 뒤 새 후방 영상에서 태그 왼쪽 모서리가 프레임 밖으로 잘리는 현상을 확인했다.
정렬 제어·카메라/태그 배치 중 무엇이 시야 이탈을 유발했는지는 추가 원인 분리가 필요하다.
실패 cleanup의 STOP 요청·실제 정지 관측은 성공했지만 이를 Dock PASS로 바꾸지 않는다.
충전 입력은 위치/속도/접촉 유지시간 에뮬레이션이며 실제 충전기 인증이 아니다.

## 검사와 자료의 한계

- 원본 감사는 24개 PNG/GIF 경로의 decode·치수·GIF 프레임·해시를 확인했다.
  기존 해시 주장이 있는 20개는 일치, Dock checkpoint 4개는 현재 해시만 산출했다.
  tuning/system PNG는 동일 내용이므로 원본 24경로가 서로 다른 장면 24개라는 뜻은 아니다.
- 이 Git용 subset은 원본 일부와 helper 사본을 별도로 export한다. export의 헤더·해시 검사는 새 주행 승인 검사가 아니다.
- strict 보고서는 runner의 `VALIDATING` 시점 run manifest를 해시한다. 검증 성공 후 runner가
  최종 `PASS`와 갱신 시각을 써서 현재 run manifest 해시는 그 중간 해시와 다르다.
  이 수명 차이는 archive provenance에 양쪽 값으로 기록한다. 과거 strict를 다시 쓰지 않으며,
  site/native/metrics/미디어의 원본 해시 일치 검사는 별도로 유지한다.
- 진단 8탭 중 카메라 실제 갱신은 약 4.75Hz. 목표 10Hz 통과나 나머지 모든 센서의 정확도 인증은 아니다.
- 음성은 과거 실행에서 출력된 자산의 파형 감사가 있으나 최신 모든 상황의 자동 음성 성공은 아니다.
- [순수 회귀](evidence/v2_2_5_20260908/README.md)는 UI364, 계약119,
  새 C++35 및 경로 재실행17이다. 중복 범위를 합산하지 않는다.
- TEST-only Dock helper 회귀74, continuation 회귀27 등은 제품 코드/실주행 완료 건수에 합산하지 않는다.
- 주행 원본 MP4는 당시 설정에 따라 파생 PNG/GIF·출처 해시를 남긴 뒤 삭제됐다.
  전체 녹화 파일이 남아 있다고 주장하지 않는다. 대용량 raw 휠/음성·빌드·임시 파일은 이 Git subset에 넣지 않았다.

## GNSS 관련 기존 문서 정정

현재 site-geometry 프로필은 production input adapter의 raw GNSS/lever-arm 경로를 사용한다.
별도로 존재하는 metric-pose 프로필의 `lever_arm=false`를 현재 설정으로 오해하면 안 된다.
현재 CARLA GNSS는 X=0/Y=±0.45m이고, IMU 기반 heading+90°와 production trim−92°를 사용한다.
실제 전방 안테나 형상·이중 GNSS 수신기의 timestamp/RTK 특성을 검증한 구성이 아니다.
자세한 원본 범위는 증거 목차의 GNSS 문서를 참고한다.

## 실행 상태와 다음 작업

suite13은 **2026-09-08 09:06:44 UTC FAIL 종료**, 이후 차량 STOP을 확인했다.
이번 릴리스 정리/push는 서버나 주행을 재시작하지 않았다.
다음 검증은 태그 지속 가시성 및 도킹 정렬 원인 분리, 남은 초기 상태/38주행/음성·센서 검증이다.
과거 `docs/evidence/virtual_carla/current` 자료는 이전 버전 자료이며 이번 완료 수에 넣지 않는다.
