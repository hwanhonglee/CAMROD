# B4 V3 attempt02 — 정밀 재중심 성공, 두 번째 회전 중 0.05m 한계 초과

<!-- HH_261012 - Separate achieved precentering from failed physical completion; retain the unchanged guard and failed result. -->

## 최종 결과

trial (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v3/B4_attempt02/trial.json`): **FAILED**, `roundtrip_verified=false`, `formal_acceptance=false`. actor **128**, 2026-10-12 **03:05:19~03:08:18 UTC**, wall **178.682초**, **773개** 표본, 위치 표본 누적 거리 **72.116m**. supervisor (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v3/B4_attempt02/supervisor.json`)는 probe exit **1**, capture exit **0**으로 종료했다. 첫 B4 setup failure (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v3/B4/supervisor.json`)는 실제 mission이 시작되지 않은 별도 결과이며 이 시험과 합치지 않는다. 실제 시험 전 [live-profile readback](../live_profile_ready.json)은 PASS였다.

진입 yaw·CRAB_IN·첫 180° 회전·UNLOAD_WAIT·WAIT_RETURN은 통과했다. 복귀를 위한 정밀 재중심도 성공했지만, **두 번째 회전 중 기존 0.05m 위치 guard를 초과**했다. 도로 복귀 handoff와 최종 주차는 시작하지 못했다. 최종 gate **ENABLED**, controller 명령 **0**, `return_published=False`, `return_ack=False`; 기록된 충돌 이벤트 **0건**이며 모든 접촉 부재를 뜻하지 않는다.

## 0.01m 정밀 재중심은 실제로 도달했다

고정 B4 중심은 ROS **(16.3537, −1.41168)m**다. V3는 출발 전 중심 허용 오차만 **0.01m**, 해당 episode의 lateral 전환 오차만 **0.005m**로 엄격하게 했다. 회전 중 위치 guard **0.05m**, centering **15초**, 전체 heading 준비 **90초**는 그대로다.

| 관측 | trial wall s | 중심 거리 m |
| --- | ---: | ---: |
| return 재중심 첫 표본 | 166.118023 | 약 0.05126 |
| 첫 실제 yaw 명령 | 170.720293 | **0.005868492403** |
| 회전 중 최소 표본 | — | 0.003765744477 |
| 마지막 회전 표본 | 178.592249 | **0.050339601583** |
| 최종 ERROR pose | 178.682271 (result) | **0.050520922640** |

첫 yaw 명령까지 약 **4.602초**다. 사이트 진단은 `precise site center restored; error_m=0.008 tolerance_m=0.010`을 기록했다. 진단 문자열의 centering flag는 실제 명령보다 늦게 갱신되므로 물리 구간은 **cmd_vel의 실제 yaw 명령**으로 나누었다.

첫 yaw 명령 pose **(16.3540333293, −1.4175390182)m**에서 마지막 회전 표본 pose **(16.3051222261, −1.3984786603)m**까지 ROS 위치가 **0.052494m** 이동했다. 같은 구간 CARLA actor world XY도 **0.055103m** 이동하여 단순한 localization 표시 문제만은 아니다. 회전 표본 **35개**, CARLA yaw 변화 **+151.109°**, 최고 평면 속도 약 **0.019945m/s**, 외부 wheel torque 절댓값 최대 **2Nm**였다. ROS/CARLA yaw 좌표계 부호는 구분한다.

최종 pose **(16.3049077374, −1.3985773768)m**, 중심 거리 **0.050520922640m**로 guard를 약 **0.521mm** 넘었다. 최종 yaw **−41.08°**, 목표 **−67.43°**, 남은 오차 **−26.35°**다. 마지막 정상 표본과 ERROR pose는 동일 tick이 아니므로 표본 시간으로 최초 초과 순간까지 확정하지 않는다.

실제 사이트 노드 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012_v3/camping_site_maneuver_controller_node_3276192_1791774159443.log`)의 phase 시작 UNIX **1791774485.358752**, ERROR **1791774497.974003**: **12.615초**다. 재중심 완료 후 회전 중에 실패했고 90초에 못 미치므로 **centering timeout이나 전체 turn timeout이 아니라 위치 guard 실패**다.

## 이전 결과와 작은 yaw PI

[V2](../../site_completion_20261012_v2/B4/ANALYSIS.md)는 0.042588m에서 회전을 시작해 약 3.636초 만에 위치 guard에 걸렸고 yaw가 약 171° 남았다. V3는 0.005868m에서 시작해 약 26°가 남을 때까지 진행했다. **정밀 재중심은 실행됐지만 전체 회전 성공에는 충분하지 않았다.** 이 한 차례 비교로 반복 신뢰성·정확한 tire/steering 근본 원인·하드웨어 개선을 주장하지 않는다.

기존 작은 yaw 문제는 이번에도 재발하지 않았다. nominal 진입 오차 **5.82→3.60°**(후자는 phase elapsed 2.2초), 후속 body-yaw 오차 **1.29°**(4.3초 표본) 이후 CRAB_IN으로 진행했다. entry 정렬 **33개 표본**, wall **127.548~135.012초**, 외부 wheel torque 표본 절댓값 최대 **0.719Nm**, 실제 PI 로그 적분 항 절댓값 최대 **0.30909Nm**였다. 실패한 두 번째 pivot의 명령 절댓값 **0.45rad/s**는 작은 yaw PI의 **≤0.2rad/s** 적용 범위 밖이다.

## 증거와 범위

- gate SHA-256: `a3d97a8f3ad7219596ee73e40ecc23bae777f0a3aaf6dd3f034c934518342200`.
- controller 로그 (LOCAL_ARCHIVE: `/home/hong/camrod_ws/log/virtual_carla/site_completion_20261012_v3/python3_3275278_1791774155558.log`) SHA-256: `ec56a0964ab8ab1c213a2f4152645f839d53e9ee95661c5e9a06c10dfb2d5bce`.
- [frozen inputs](../frozen_inputs.json), 실제 contact sheet (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v3/B4_attempt02/visual/representative_contact_sheet.png`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v3/B4_attempt02/visual/capture_manifest.json`).

contact sheet를 열어 실제 outbound·사이트 진입·사이트 내 복귀 준비를 확인했다. 성긴 정지 영상만으로 전 회전이나 정확한 guard 시점을 입증하지 않으며 수치 분석은 trial·노드 로그에 근거한다. 영상 manifest PASS는 capture 검증이지 mission PASS가 아니다. 본 분석은 읽기 전용 관측과 최종 문서 작성만 수행했고, 런타임·차량 API·소스·안전 임계값은 바꾸지 않았다.
