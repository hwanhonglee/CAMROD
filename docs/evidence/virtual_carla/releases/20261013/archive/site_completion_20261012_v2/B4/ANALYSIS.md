# B4 V2 — 진입 yaw 통과, 사이트 내 두 번째 회전의 위치 한계 정지

<!-- HH_261012 - Preserve the failed physical result and distinguish entry PI from the forward-loop second-pivot position guard. -->

## 결과

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v2/B4/trial.json`): **FAILED**, `roundtrip_verified=false`, `formal_acceptance=false`. actor **86**, 2026-10-12 **02:51:14~02:54:04 UTC**, trial wall **169.929초**, 표본 **739개**, 위치 표본 누적 거리 **71.870m**. supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v2/B4/supervisor.json`)는 probe exit **1**, capture exit **0**으로 종료했다. 최종 주차에는 도달하지 않았다.

`ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN`은 통과했다. 새 `forward_loop` 복귀 요청 후 `ALIGN_RETRACE_YAW`에서 다음 오류로 정지했다.

```text
forward_loop in-site heading preparation exceeded site position bound or steady turn timeout
```

최종 gate는 **ENABLED**, controller 명령은 0, `return_published=False`, `return_ack=False`였다. 여기서 controller의 route handoff ack와 probe가 별도로 관측한 UI return-request acknowledgement를 구분한다. 충돌 publisher/subscriber가 발견된 상태에서 기록된 충돌 이벤트는 **0건**이며, 모든 접촉 부재를 보장하지 않는다.

## 작은 yaw PI: 이전 실패 지점은 다시 통과

[실제 live profile](../live_profile.json)은 PI=true, Ki=2.0이며 [frozen inputs](../frozen_inputs.json)는 시험 시작 시 3,662개 파일의 변경 없음을 확인했다. 실제 controller 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012_v2/python3_3255132_1791773317824.log`)에서도 적분 증가가 관측됐다.

- nominal 진입 yaw 오차 **7.22° → 3.96°**: 후자는 phase elapsed **2.0초** 표본이다.
- 후속 body-yaw 정렬도 완료되어 wall **134.579초**에 `CRAB_IN`이 관측됐다. 실제 노드의 nominal→body 전환은 UNIX 1791773604.537293, body 정렬 완료는 1791773608.737321이다.
- 전체 `ALIGN_ENTRY_YAW` 표본 **30개**, wall **127.641~134.336초**; CARLA yaw **73.646→69.813°**.
- 이 구간 외부 wheel torque 표본 절댓값 최대 **0.831Nm**, PI 로그 적분 항 절댓값 최대 **0.35909Nm**. 로그/telemetry 주기가 달라 tick 전체의 최대값은 아니다.

10/08 baseline (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_regression_20261008/B4/ANALYSIS.md`)의 15.1초 entry-yaw timeout은 재발하지 않았다. 10/08 PI 후보 (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_fixes_20261008/B4/ANALYSIS.md`)는 왕복 완료를 관측했고, [10/12 첫 프로필](../../site_completion_20261012/B4/ANALYSIS.md)은 yaw 통과 후 복귀 도로 fusion 안전 정지였다. **이번 V2는 그 두 실패와 다른 사이트 내 두 번째 회전 위치 한계 실패**다. 각 도착 조건이 달라 이 비교를 통제된 반복 A/B 인과 증명으로 해석하지 않는다.

## 실제로 초과한 한계: 90초 timeout이 아니라 0.05m 위치 한계

고정 [forward-return catalog](../../../../../../../../camrod_carla_adapter/config/camping_sites_carla_forward_return.yaml)의 B4 중심은 ROS **(16.3537, −1.41168)m**다. 아래 거리는 반올림된 로그 좌표가 아니라 catalog의 정확한 중심과 trial pose로 계산했다.

| 시점 | trial wall s | 중심 거리 m | 관측 |
| --- | ---: | ---: | --- |
| return 재중심 시작 | 166.354466 | 0.057748093213 | lateral 보정 명령 |
| 재중심 후 첫 회전 명령 표본 | 168.903291 | 0.042587675070 | w=−0.45rad/s |
| 마지막 정상 표본 | 169.837411 | 0.048586796636 | w=−0.45rad/s |
| 최종 ERROR pose | 169.928989 (result) | **0.050207320456** | 명령 0 |

최종 pose는 **(16.3226045992, −1.3722610823)m**, 중심 한계 초과량은 약 **0.207mm**다. trial의 마지막 정상 표본과 ERROR tick은 동일 tick이 아니므로 표본만으로 최초 초과 순간을 재구성하지 않는다.

사이트 노드 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012_v2/camping_site_maneuver_controller_node_3255903_1791773321624.log`)에서 `ALIGN_RETRACE_YAW` 시작 UNIX **1791773640.401975**, ERROR **1791773644.037791**: 약 **3.636초**로 설정된 90초보다 훨씬 짧다. 재중심 완료 후에는 centering flag=false였다. 따라서 `siteForwardReturnTurnPositionValid()`의 **0.05m 위치 guard**가 정지를 결정한 것으로 확인된다. 최종 yaw는 **103.54°**, 목표 **−67.43°**, 오차 **−170.96°**로 회전 초기에 종료했다.

해당 회전 명령 절댓값 **0.45rad/s**는 작은 yaw PI의 적용 범위 **≤0.2rad/s** 밖이다. 이번 정지를 PI 저응답이나 entry-yaw timeout으로 분류하지 않는다. 현재 centering은 0.05m 안에 들어오면 출발하여 회전 drift 여유가 작았다. 더 엄격한 *출발 전* centering은 검증할 후보이지만, 이 증거만으로 개선 성공을 주장하거나 0.05m guard를 넓혀서는 안 된다.

## 증거 보존

- physical gate SHA-256: `879d0d8231759c58aaca362c9ba3b8c69a3137669fd94850e5c84ba32b844435`.
- 이 actor의 controller 로그 SHA-256: `1776b3d9381a1ad80c7122f1c1821a0d5804671866546ed46ee558239a062193`.
- 실제 contact sheet (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v2/B4/visual/representative_contact_sheet.png`)를 열어 outbound와 사이트 내 종료를 확인했다. 마지막 `Operator Stopped`는 controller 오류 뒤 cleanup이다. 이 성긴 contact sheet 자체는 yaw 정렬 전 과정을 보여주지 않으며, 수치 분석은 trial·노드 로그에 근거한다.
- 영상 manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v2/B4/visual/capture_manifest.json`)의 PASS는 캡처 검증일 뿐 임무 PASS가 아니다.

읽기 전용 모니터링과 최종 문서 작성만 수행했다. 런타임·차량 API·소스·안전 임계값은 변경하지 않았다.
