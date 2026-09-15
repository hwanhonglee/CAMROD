# v2.2.8 미션 기록·UI 변경 검증 (HH_260915)

## 배포 범위

기준 develop: `71fb54db6ce476c1ad4eabc7f2c5a07a12ece80b`.
기존 `v2.2.7`을 이동하지 않고 새 `v2.2.8`로 구분한다.

이번 변경은 `camrod_ui` 안의 수동적 기록 노드, 승인 이벤트 관측 훅,
읽기 전용 API, 기존 실증 현황에 추가한 미션 상세 패널, 테스트/설명이다.
CARLA 패키지·의존성, 지도, 주차/주행 제어기, 안전 게이트 정책을 변경하지 않는다.
작업 트리에 이미 있던 다른 지도/GNSS/설정 변경은 이번 커밋에 포함하지 않는다.

구현별 설명은 [기록기 운영 설명](mission_recording.md),
CAN/state JSONL 예시는 [데이터 형식](mission_recording_data_format.md)을 따른다.

- `mission_journal.py`: 일반/recall+복귀 왕복, CAN1/RC0/미확인 거리,
  STOP 이후 수동 복구·복귀 연결, 정지 사유, 저장/재시작/중복 방지.
- `mission_recorder_node.py`: 독립 ROS 구독 노드, 저장 worker, bounded queue,
  stale/overflow 진단, 읽기 전용 snapshot. 주행 명령 발행 없음.
- `raw_can_capture.py`: 명시적으로 켰을 때만 수신하는 원시 CAN 기록; 송신 없음.
- `mission_recording_bridge.py` 및 백엔드: 승인 이후 기록 이벤트와 조회 API.
- `MissionRecords.js/.css`: 날짜/사이트/유형별 미션, 거리 구성, 수동 개입,
  정지 이유·상태 전이 타임라인과 파일 설명.
- 릴리스 검토 중 경과 시간/진행 거리/배터리 수치만 변해도 같은 정지 이유가
  반복 기록되던 문제를 수정하고 17개 회귀 검사를 추가했다. 실제 사유 변화와
  최신 원문 증거는 유지한다. 안전 판단 로직은 수정하지 않았다.

## 검증 결과와 한계

다른 로컬 수정이 섞이지 않도록 기준 커밋의 별도 소스 트리에 이번 변경만 적용했다.
커밋 대상 파일과 검증한 파일은 바이트 단위로 비교했다.

| 검사 | 결과 |
| --- | --- |
| React production build | 성공. 실제 생성 bundle의 ROS 설치 포함도 확인 |
| React 전체 테스트 | 61 통과 |
| UI Python 전체 테스트 | 880 통과 |
| 소유 패키지 Python 검사 | 1,330개 중 1,326 통과, 기존 기대값 불일치 4 실패 |
| 1차 패키지 빌드 | 아래 14개 모두 성공 |
| 등록된 기능 CTest | 78 타깃 중 74 통과, 아래 4 타깃 실패 |
| assert 기반 system 검사 보완 | `-UNDEBUG`로 별도 컴파일한 4개 모두 통과 |
| 실제 ROS 기록 노드 + 합성 입력 | B7/B8/B9 × 일반/recall = 6건 완료 |
| 현재 전체 UI 렌더링 | 기존/새 패널 동시 존재, 값·표·그래프 확인, 브라우저 예외 0 |

Python 검사와 CTest에는 같은 검사도 있으므로 두 개수를 합해 고유 테스트 수라고
표현하지 않는다. pytest는 기존 일부 테스트의 전역 launch 모듈 stub 간섭을 피하려고
파일별 독립 프로세스로 실행했다. 린트 전체를 통과했다는 의미도 아니다.

빌드 패키지: `avg_msgs`, `camrod_runtime`, `camrod_sensor_kit`, `camrod_map`,
`camrod_control`, `camrod_planning`, `camrod_localization`, `camrod_platform`,
`camrod_system`, `camrod_sensing`, `camrod_perception`, `camrod_bringup`,
`camrod_ui`, `camrod_voice`.

Ubuntu x86 / ROS Humble / Release / BUILD_TESTING=ON으로 기존 설치 의존성을 사용했다.
음성 빌드는 로컬에 추출된 SDL2_mixer의 include/library 경로를 검증 셸에만 추가했다.
이를 CAMROD 코드의 로컬 경로 의존성으로 넣지 않았다.
Boost/PCL/CMake 경고와 기존 OpenCV 4.5/4.5d 링크 경고는 남아 있다.
Jetson 전용 카메라 노드는 x86에서 생략되며, 외부 드라이버 전체를 다시 빌드한 것은 아니다.
실차 CAN 수신·음성 출력·카메라 하드웨어·실차/CARLA 전 구역 주행을 인증하지 않는다.

### 통과하지 않은 기존 검사

다음의 소스/설정/테스트는 기준 `71fb54db`와 동일하다. UI 변경으로 생긴 실패로
분류하지 않지만, **전체 기능 검사가 모두 정상이라고 보고하지 않는다.**

1. `test_camping_site_recall_return`: 22개 C++ case 중 10개 실패.
   controller의 음성 안내 완료 대기는 절대 시각을 사용하지만 fixture는 단계 시작
   시각만 앞당긴다. 따라서 clearance 대기가 해제되지 않아 다음 단계/offset 기대가
   연쇄 실패한다. 이번에 수정한 UI 코드에 의존하지 않는 별도 C++ 실행 파일이다.
   별도 복제 fixture에서 이 음성 게이트만 끈 원인 분리 실험은 22/22 통과했다.
   이는 원인 확인용이며 배포 소스에 반영하지 않았고, 원래 실패나 실제 음성 연동
   검증을 대체하지 않는다. 재현 자료는
   `release_check/assertion_checks/clearance_fixture/REPORT.md`에 남겼다.
2. `test_planning_runtime_coalescing`: 1개 Python case 실패.
   기존 설정 `xy_goal_tolerance=0.20`과 테스트가 요구하는 `0.10`의 불일치.
3. `test_module_readme_assets`: 1개 Python case 실패.
   활성 map v27을 과거 map v15 증빙으로 사용하는 것을 정상 거부하지만,
   테스트는 거부 문구에 map v22만 허용한다.
4. `test_park_operating_points_assets`: 2개 Python case 실패.
   과거 지도 해시/형상을 활성 map v27에 그대로 기대하는 불일치.

이를 통과시키려고 주행 허용 오차나 지도, 음성 대기 정책을 바꾸지 않았다.
실패 원본과 보완 실험은 아래 검증 폴더에 남겼다.

## 기존 내용 보존 및 현재 화면 확인

기존 `ServiceEvidence.js` 변경은 import와 `<MissionRecords />` 삽입 3줄뿐이다.
기존 평균 거리·시간 SVG, 사이트 표, 일별/최근 이력과 누적 계산은 그대로다.
기존 `service_metrics.py`와 `ServiceEvidence.css`는 변경하지 않았다.

운영 `service_metrics.sqlite3`는 이번 기능에서 수정하지 않으며,
검증 전후 SHA-256은 다음과 같다.

```text
43e292886f786f0c359675b88b8219af150ac24d8a4a171c205db48db5c46261
```

새 전체 페이지 캡처에서는 실제 현재 컴포넌트에 다음 두 **독립 검증 입력**을 넣었다.

- 기존 누적 API fixture: 64 m / 완료 3건, B1–B13 및 DROP_ZONE 표 14행,
  실제 SVG 점·거리/시간 막대, 일별 1행/최근 3행.
- 새 ROS 기록 결과: 6건 완료 / 13.3 m, 자율 10.3 m·수동 2.8 m·미확인 0.2 m,
  선택 미션의 주요 이벤트 17건. 표시 값은 반올림한다.

이 둘을 합산하지 않으며 실차 운행 값이라고 표시하지 않는다.
CLOSED 표시는 시험 노드를 종료한 뒤의 저장 snapshot 상태 그대로다.
제품 전체 실증 현황 컴포넌트를 캡처했지만 상위 로봇 메뉴나 실차 연결을 켠 것은 아니다.

## 로컬 자료 위치

공통 루트: `/home/hong/camrod_ws/_maintenance/20260915_mission_recorder/`

| 자료 | 공통 루트 아래 경로 |
| --- | --- |
| 기존 그래프 + 새 기록 한 화면 | `evidence/combined_current/06_simultaneous_overview.png` |
| 정량표·이력까지 전체 PNG | `evidence/combined_current/00_full_current_page_new_and_existing.png` |
| 현재 페이지 탐색 GIF | `evidence/combined_current/current_full_page_navigation.gif` |
| 촬영 소스 SHA/DOM/값/이미지 해시 | `evidence/combined_current/{README.md,validation.json,manifest.json}` |
| 최신 실제 ROS 노드 시험 | `ros_smoke_release_v228/result.json`, 같은 폴더의 `records/` |
| React/Python 결과 | `release_check/react_all.json`, `release_check/python_isolated/` |
| C++/기능 검사 로그 | `release_check/ctest/`, `release_check/build/*/test_results/` |
| assert 유효성 보완 | `release_check/assertion_checks/REPORT.md` |
| 전체 빌드 로그 | `release_check/log/latest_build/` |

과거 PNG/GIF와 정량 자료도 아래 폴더에 그대로 남겼다.

- `/home/hong/camrod_ws/_maintenance/20260914_service_metrics_fix/capture_final/`
- `/home/hong/camrod_ws/_maintenance/20260914_historical_metrics_preservation/preview_ui/capture/`

이 로컬 원본 DB/로그/PNG/GIF와 빌드 출력은 Git에 포함하지 않는다.
Git에는 순수 CAMROD 소스·테스트·설명만 포함하며, 기존 실제 운행 DB를 배포 데이터로
덮어쓰지 않는다. 다른 PC에서 같은 그림 파일을 보려면 별도로 이 증빙 폴더를 복사한다.
