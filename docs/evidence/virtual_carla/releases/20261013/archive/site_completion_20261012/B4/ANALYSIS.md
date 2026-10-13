# B4 고정 공통 프로필 재시험 — yaw 통과, 복귀 도로 안전 정지

<!-- HH_261012 - Separate successful yaw tracking from the failed full mission; do not turn prior PASS into current acceptance. -->

## 최종 결과

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012/B4/trial.json`): **`FAILED`, `roundtrip_verified=false`, `formal_acceptance=false`**. B4 진입 yaw 정렬과 사이트 서비스·이탈은 완료했지만, 복귀 도로의 안전 정지가 지속되어 supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012/B4/supervisor.json`)가 `persistent_safety_hold_90s`로 시험을 종료했다. trial의 `error=SIGTERM`은 이 진단 종료 신호이며, 이전의 진입 yaw timeout 재발을 뜻하지 않는다.

- actor **28**, 2026-10-12 02:27:57~02:32:43 UTC.
- trial wall **286.171초**, 위치 표본 누적 거리 **82.329m**.
- 사이트 단계: `ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`.
- 마지막 서비스는 `RETURN_WITH_CARGO`; Drop Zone 도착·주차는 완료하지 못했다.
- 충돌 publisher/subscriber가 발견된 상태에서 기록된 충돌 이벤트 **0건**. 접촉 부재를 보장하는 수치는 아니다.

## 실제 yaw 적분 응답과 이전 시험 비교

이번 시험은 [profile.sh](../profile.sh)의 전 사이트 공통 PI=true 설정이며 B4만 선택하는 스위치를 사용하지 않는다. 설정뿐 아니라 해당 세션의 실제 controller 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012/python3_3230081_1791772061661.log`)에서 적분 항의 증가를 확인했다.

| 항목 | 10/08 고정 survey | 10/08 PI 후보 | 10/12 공통 PI 프로필 |
| --- | --- | --- | --- |
| 시작 nominal yaw 오차 | 6.31° | 6.95° | 5.17° |
| nominal 정렬 | 15.1초 timeout, 오차 5.06° | 2.1초 표본 오차 3.91° | 2.1초 표본 오차 3.82° |
| 후속 body-yaw 정렬 | 시작 못함 | 3.2초 표본 오차 0.99° | 5.2초 표본 오차 1.21° |
| 전체 결과 | yaw 정렬 실패 | 왕복 완료 관측 | yaw 통과 후 복귀 안전 정지 |

비교 근거: survey 분석 (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_regression_20261008/B4/ANALYSIS.md`), 이전 PI 후보 분석 (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_fixes_20261008/B4/ANALYSIS.md`), 현재 trial. 각 도착 yaw·위치·접촉 조건은 동일하지 않으므로 통제된 반복 A/B 인과 증명은 아니다.

현재 `ALIGN_ENTRY_YAW` 표본은 **36개**, trial wall **134.168~142.300초**다. CARLA yaw는 **71.600→69.645°**로 변화했고, 관측 외부 wheel torque 절댓값 최대는 **0.786Nm**였다. ROS yaw 절댓값과 CARLA yaw 절댓값은 좌표계가 달라 직접 비교하지 않는다. nominal/body 정렬의 각 phase elapsed는 위 trial wall 시간과도 구분한다.

실제 노드 진단의 대표 값은 다음과 같다. 단위는 target/feedback rad/s, 토크 Nm이며 CARLA 부호계다.

| UNIX 로그 시각 s | target | feedback | 적분 항 | P 항 | 합계 |
| --- | --- | --- | --- | --- | --- |
| 1791772212.028508 | −0.10832 | +0.00015 | 0.00000 | −0.54236 | −0.54236 |
| 1791772213.609955 | −0.08424 | −0.05874 | −0.27389 | −0.12748 | −0.40138 |
| 1791772215.130858 | −0.08000 | +0.00014 | 0.00000 | −0.40070 | −0.40070 |
| 1791772218.826512 | −0.08000 | −0.00883 | −0.27648 | −0.35585 | −0.63234 |

진단은 여전히 `experimental_unqualified`로 표시된다. P=5·일반 회전 2Nm 제한·정렬 허용 오차·timeout을 완화해 통과시킨 결과가 아니다. 로그와 물리 표본은 주기가 달라 표본 최대값을 tick 전체의 최대값으로 해석하지 않는다.

## 이번 실패 위치와 원인 범위

복귀 `SAFETY_HOLD`는 wall **193.628~284.656초**, **393개 표본**에서 관측됐다. 해당 표본의 `cmd_vel` 세 성분은 모두 0이었다. 마지막 ROS 위치는 약 **(16.396, 6.512)m**, 사이트 상태는 이미 `DONE`이고 `return_published=True`, `return_ack=True`였다.

실제 안전 게이트 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012/cmd_vel_safety_gate_node_3230827_1791772065393.log`)에 다음 트리거가 반복된다.

```text
dynamic_front_path:fusion
cost_stop_latched:dynamic_front_path:fusion:trigger_evidence_not_fresh
```

따라서 이번 직접 미완료 원인은 **진입 yaw 응답이 아니라 복귀 경로의 fusion 기반 안전 정지 지속**이다. 위 문자열만으로 특정 실제 물체, 잘못된 분류, 경로와 물체의 정확한 간격 중 무엇이 근본 원인인지 확정할 수 없다. 안전 정지 해제·분류 무시·임계값 완화로 통과 처리해서는 안 된다.

실제 contact sheet (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012/B4/visual/representative_contact_sheet.png`)를 열어 사이트 진입 후 복귀 도로 정지와 UI 안전 정지를 확인했다. 복귀 장면에는 `fire hydrant` 약 2.4m 표시가 보이지만, 그 표시만으로 정지 트리거의 fused box와 동일하다고 단정하지 않는다. 마지막 운행 정지 확인창은 장시간 안전 정지 뒤의 시험 종료 과정이며 최초 원인이 아니다. manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012/B4/visual/capture_manifest.json`)의 영상 파생물 `PASS`는 왕복 성공 판정이 아니다.

## 증거 및 결론

- controller 로그 SHA-256: `da3fe7c41ee17f17acba9adc12f69247cc2b4483516c18eeb3ef51202b0713e7`.
- trial physical gate SHA-256: `bb83233dd82ad336aea6b526f62167572fed24beabb0a69d3797f83aa479a169`.
- 공통 프로필·소스 기준: [frozen_inputs.json](../frozen_inputs.json).

작은 yaw PI의 이전 실패 지점 통과는 재관측됐지만, **현재 B4 전체 임무는 실패**다. 10/08 후보의 왕복 성공을 이번 결과로 이월하지 않는다. 본 작업은 저장된 trial·실제 노드 로그·영상의 읽기 전용 확인과 이 문서 작성만 수행했으며, 런타임 API·소스·안전 설정은 변경하지 않았다.
