# 서비스 화면 문구 간소화

<!-- HH_261103 - Retain the original capture and no-push statements as dated evidence, not current publication status. -->
이 문서는 **2026-11-02 당시의 변경·UI fixture 기록**이다. 아래 경로·번들·시험 수와 미커밋·미push·실물/CARLA 미검증 설명은 당시 범위다. 현재 발행 범위·기본값·이후 별도 시험 구분은 [unit2-v1.0.1](../../../releases/unit2-v1.0.1.md)을 참고한다. 원본 manifest와 결과 파일은 변경하지 않았다.

HH_261102 - Simplify public service copy without changing mission commands, authority, battery thresholds or safety conditions.

적용 위치: `/home/hong/camrod_service_test_ws/src`, `test/unit2-service-usability-20261102`.
기존 스타일·서비스 흐름을 유지하고, **현재 상태 → 짧은 설명 → 다음 행동**이 먼저 보이도록 정리했다.
관리자 기능이나 예약 인증 기능을 새로 추가한 작업은 아니다.

## 바뀐 화면

| 위치 | 이전 | 변경 후 |
| --- | --- | --- |
| 충전 화면 | 충전·접점·배터리·목표 위치·정책 설명이 동시에 표시됨 | `충전 중`, 현재 배터리, 필요한 행동만 먼저 표시 |
| 충전 위치 이동 버튼 | 짐을 꺼내기 위해 대기 위치로 이동 | **앞으로 이동** |
| 서비스 메뉴 | 긴 서비스 설명과 반복 확인 문장 | **배달 서비스 / 호출 서비스**, 카드마다 한 문장 |
| 휴대폰 목적지 선택 | 긴 도로 측 대기점·서비스 정책 설명 | **어디로 부를까요?**, **B2로 호출** |
| 호출 후 첫 확인 | 정리 완료 · 사이트 재진입 | **정리 완료 · 회전 시작** |
| 호출 후 최종 확인 | 사이트별로 복귀 버튼 표현이 다름 | **짐 싣기 완료 · 복귀** |
| 긴 오류 | `phase=ERROR` 등 내부 원인이 이용객 화면에 그대로 노출 | 짧은 문의 안내 + 접힌 **상세 원인 / 상세 안내** |

충전의 긴 차단 사유·목표 위치·배터리 정책은 **자세한 안내**에 보존했다.
휴대폰의 배터리·주차 정책도 접힌 상세 영역에서 확인할 수 있다.
센서 신호가 최신이어도 배터리 값이 없으면 “충전이 더 필요”라고 단정하지 않고 **배터리 확인 중**으로 안내한다.

## 유지한 기능·안전 안내

- 이동 요청 API, 버튼 활성화 조건, 35% 기준, 미션 소유권·세대 검증은 그대로다.
- 호출의 첫 확인 → 사이트 진입·회전 → 최종 적재 확인 → 복귀라는 두 단계 절차도 그대로다.
- 사이트명 입력 확인과 일반 운행 정지·취소 확인을 제거하지 않았다.
- 충전 접점에 손대지 말라는 경고, 정차 전 접근 금지, 회전 중 주변 비우기는 계속 표시한다.
- “출발하면 정차할 수 없습니다”라는 잘못된 문구만 제거했다. 기존 운행 정지 버튼은 유지한다.
- 충전 출차 후 **정차 상태에서 기다리며**, 별도 완료·강제 재도킹 절차를 추가하지 않았다.

## 화면 자료

모두 실제 HTML/React를 Chrome에서 렌더링한 **루프백 UI 시험 화면**이다. 실제 로봇/CARLA 영상이 아니다.

- 충전: [이전](before_charging.png) → [수정 후](../charging_ui/02_soc35_eligible.png)
- 서비스 선택: [이전](before_service_menu.png) → [수정 후](../charging_ui/06_existing_service_menu.png)
- [짧아진 배달 확인](../charging_ui/07_delivery_confirmation.png)
- [휴대폰 사이트 선택](../charging_ui/10_guest_phone.png), [휴대폰 호출 확인](../charging_ui/11_guest_confirmation.png)
- [충전 화면 흐름 GIF](../charging_ui/charging_access_flow.gif) — 정지 화면 몽타주이며 실제 주행 영상이 아님.

이전 PNG는 이 작업 직전 `main.b01cdf47.js` 화면을 보존한 것이다.
수정 후 11개 PNG는 `main.361f76de.js` / `main.f4da5088.css`로 다시 캡처했다.
충전 API·Guest WebSocket 입력, 소스 해시, 번들 목록은 [manifest](../charging_ui/manifest.json)에 남겼다.
재현 스크립트는 [capture_fixture.cjs](../charging_ui/capture_fixture.cjs)이며 실제 서비스에 접속하지 않는다.

## 확인 결과

- Python UI 전체 **1,045개 통과**: [JUnit](python_ui_regression.xml).
- React 전체 **20개 묶음, 338개 통과**. 상태별 문구, 접힌 상세 오류, 확인 전 미전송, 두 단계 호출 복귀도 확인했다.
- 프런트엔드 배포 빌드 성공, 테스트 작업 공간의 `camrod_ui` 패키지 빌드·설치 성공.
- 소스 → 빌드 → 설치의 GLB·도색 등 공개 자산 **58개 일치**, 설치된 Guest HTML도 소스와 일치.
- `git diff --check` 통과.

빌드의 기존 Browserslist 데이터 갱신 안내와 테스트의 기존 가상 키보드 중복 React key 경고는 남아 있다.
이번 문구 작업에서는 키보드 구조나 의존성을 변경하지 않았다.
실제 어르신 대상 사용성 평가·실물 주행은 수행하지 않았고, 기존 로봇 서비스 재시작·커밋·push도 하지 않았다.
