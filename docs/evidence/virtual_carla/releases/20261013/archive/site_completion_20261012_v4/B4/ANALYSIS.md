# B4 V4 — 위치 guard 유지, 4회 재중심 recovery 한도 소진

<!-- HH_261012 - Preserve the observed recovery-count failure; do not classify it as a position breach, timeout, server crash, or completed return. -->

## 최종 결과

trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v4/B4/trial.json`): **FAILED**, `roundtrip_verified=false`, `formal_acceptance=false`. 새 서버의 actor **12**, 2026-10-12 **03:23:08~03:26:18 UTC**, trial wall **190.271초**, **824개** 표본, 위치 표본 누적 거리 **72.514m**. supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v4/B4/supervisor.json`)는 probe exit **1**, capture exit **0**으로 종료했다.

진입 yaw·CRAB_IN·첫 180° 회전·UNLOAD_WAIT·WAIT_RETURN을 통과했고, 사이트 내 복귀 방향 회전 중 네 번 재중심을 수행했다. 다섯 번째 재중심이 필요해진 순간 다음 오류로 정지했다.

```text
forward_loop recenter recovery limit reached: 4/4; immutable turn guard retained
```

이는 **recovery 횟수 한도 실패**다. 0.05m hard position guard 초과, 15초 centering timeout, 90초 전체 timeout, 서버 crash가 아니다. 최종 gate는 **ENABLED**, 명령은 **0**이며 `return_published=False`, `return_ack=False`다. 도로 복귀와 최종 주차는 도달하지 못했다. 충돌 publisher/subscriber가 발견된 상태에서 기록된 충돌 이벤트 **0건**이며, 모든 접촉 부재를 보장하지 않는다.

## 실제 재중심 sequence

고정 B4 중심은 ROS **(16.3537, −1.41168)m**다. V4는 0.025m를 넘으면 회전을 멈추고 0.01m까지 재중심하며, 회전·진행 중 재중심의 0.05m guard와 최대 네 번 recovery를 유지한다.

| 사건 | 실제 노드 UNIX 시각 s | 노드가 기록한 중심 거리 m |
| --- | ---: | ---: |
| return heading 준비 시작 | 1791775553.925601 | — |
| recovery 1/4 시작 | 1791775560.839300 | 0.025 |
| recovery 2/4 시작 | 1791775565.539587 | 0.026 |
| recovery 3/4 시작 | 1791775570.040558 | 0.027 |
| recovery 4/4 시작 | 1791775574.339729 | 0.027 |
| 추가 recovery 차단·ERROR | 1791775578.639625 | — |

실제 사이트 노드 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012_v4/camping_site_maneuver_controller_node_3302820_1791775331821.log`)가 위 사건을 기록한다. phase 시작부터 종료까지 **24.714초**이며 recovery 때 전체 phase deadline을 새로 시작하지 않았다.

초기 및 네 번 recovery의 중심 완료 detail은 각각 **0.007, 0.007, 0.010, 0.009, 0.010m**였다. 첫 yaw 명령이 있는 trial 표본은 wall **170.071초**, 중심 거리 **0.014916m**다. 완료 판단 tick·명령 수신·pose·저주기 상태 문자열의 표본 시각이 다르므로, 완료 detail과 첫 명령 표본을 같은 순간의 위치로 취급하지 않는다.

첫 yaw 명령 이후 모든 기록 표본의 중심 거리 최대는 **0.030064641217m**로, **0.05m 이내**였다. 마지막 정상 표본은 **0.026640544384m**, 최종 ERROR pose **(16.3433610285, −1.3861395967)m**는 **0.027553702707m**다. 최종 yaw **8.84°**, 목표 **−67.43°**, 남은 오차 **−76.27°**로 아직 회전이 완료되지 않았다. ERROR 전 마지막 상태 count는 **4**였으며, ERROR 상태 전환에서 mission-local count가 0으로 초기화되므로 최종 문자열의 count=0을 “recovery 없음”으로 해석하지 않는다.

## 이전 시험과 구분

- [V2](../../site_completion_20261012_v2/B4/ANALYSIS.md): 느슨한 출발 중심에서 두 번째 회전을 시작해 0.05m guard 초과.
- [V3](../../site_completion_20261012_v3/B4_attempt02/ANALYSIS.md): 0.01m 재중심에는 도달했지만 연속 회전 drift로 0.05m guard 초과.
- **V4**: 기록된 회전·재중심 위치는 guard 안에 유지됐으나 네 번 recovery만으로 heading 목표를 달성하지 못함. 성공·반복 신뢰성·하드웨어 개선으로 해석하지 않는다.

기존 작은 entry-yaw 응답 문제는 이번에도 통과했다. nominal 오차 **7.14→3.72°**(후자는 phase elapsed **2.1초** 표본), 후속 body-yaw 정렬 후 CRAB_IN으로 진행했다. 이번 full mission 실패와 entry yaw 성공을 분리한다.

## 증거 및 한계

- 새 서버 source-bound gate SHA-256: `605e51da63ca0836615a24b97f60c4cda6eced9c01fff457b394d44907885e8b`.
- 사이트 노드 로그 SHA-256: `5f8d6ebcc735fde3c9f5a03de8d82cdea61026c150fe68535b0bd4020ccc0c93`.
- [frozen inputs](../frozen_inputs.json), contact sheet (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v4/B4/visual/representative_contact_sheet.png`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v4/B4/visual/capture_manifest.json`).
- [사전 서버 crash](../SERVER_RESTART.md)는 이 trial 전의 별도 infrastructure 사건이다. 이번 실제 B4 실패를 그 crash에 귀속하지 않는다.

실제 contact sheet를 열어 outbound·사이트 진입·사이트 내부 종료를 확인했다. 마지막 `Operator Stopped`는 controller 오류 뒤 cleanup이고 최초 원인이 아니다. 성긴 contact sheet는 recovery 네 번 전체의 연속 증거가 아니므로 회수·위치·시간 분석은 trial·노드 로그에 근거한다. 영상 manifest PASS는 capture 검증이지 mission PASS가 아니다. 본 작업은 읽기 전용 관측과 최종 문서 작성만 수행했으며 frozen code·runtime·차량 API·안전 설정은 변경하지 않았다.
