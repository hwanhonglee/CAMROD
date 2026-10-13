# B3 V5 attempt02 실패: 두 번째 회전 전 정밀 중심 맞춤 시간 초과

<!-- HH_261012 - Preserve the finalized failure; do not turn capture PASS into trial PASS. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/trial.json`)은 **`FAILED`**, `roundtrip_verified=false`, `formal_acceptance=false`이다. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/supervisor.json`)은 `finished`, trial 종료 코드 **1**, capture 종료 코드 **0**; capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/visual/capture_manifest.json`)의 `PASS`는 **캡처 성공만** 뜻한다. 동일 frozen V5 profile에서 실제 controller 오류로 끝난 실행이며, 앞선 cohort의 원인 미확인 SIGTERM 중단과 구분한다.

- 실제 주행: **2026-10-12 13:36:41.058–13:39:40.966 KST**; 캡처 후처리 완료 **13:40:10.155 KST**.
- Actor **156**, mission generation **1791779793037001**, delivery / **turnaround**.
- 총 **179.946 s / 73.598071 m**, outbound **163.611 s / 72.824564 m**, 780개 물리 샘플. `return_leg=null`: 완료된 복귀는 없다. Outbound 완료 이후 실패까지의 차이는 **16.335 s / 0.773507 m**이며, 완료 복귀 지표가 아니다.
- 최종 localization XY **(22.526547382198068, 0.15695559384570398)**. 고정 station **(-8.47366,40.391)**까지 **50.791644854401824 m** — 변경하지 않은 native **0.25 m** 주차 기준을 달성하지 않았다.
- 최종 site `ERROR`, 최종 명령 `(0,0,0)`. Collision publisher 1개 발견, 기록 이벤트 **0**, 누락/오래된 큐 이벤트 **0**. 충돌이 없다는 기록과 미완료 결과는 별개다.

## 첫 실패 지점과 직접 원인

`IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → ERROR`. 첫 서비스 회전 및 도착/Return 승인은 통과했다. **두 번째 회전 시작 전 최초 precenter**에서 멈췄으며, pivot recenter count는 **0/10**, 남은 yaw 약 **177°**였다. `CRAB_OUT`, `DONE`, `roadside_forward` handoff 또는 복귀 도로 계획 완료 증거는 없다.

`camrod-sites-20261012-v5-camrod.service`의 **13:39:20–13:39:43 KST** 범위 journal:

| 시각 KST | 관측 |
| --- | --- |
| 13:39:25.873 | 초기 precenter 시작; current `(22.422,0.363)`, target `(22.506,0.151)`, radial 약 `0.229 m`, 최대 속도 `0.120 m/s`, timeout **15 s** |
| 13:39:27.185 | lateral → `steering_settle` |
| 13:39:28.385 | `longitudinal` 단계 진입 |
| 13:39:40.885 | `ERROR`: `forward_loop in-site heading preparation exceeded site position bound or steady centering/turn timeout` |

시작과 ERROR의 embedded ROS log 시각 차이는 **15.011662395 s**이다. 초기 precenter의 기존 admission envelope는 `0.65 m`이고 최종 center error는 **0.021116159744915246 m**였다. 따라서 이 실행의 직접 실패는 **15 s centering deadline**, 두 번째 pivot의 `0.05 m` guard 초과 또는 전체 90 s 회전 deadline이 아니다.

실패를 일으킨 수렴 불가 상태는 다음과 같다. 현재 centering sequencer는 lateral 완료를 latch한 다음 longitudinal만 보정한다. 최초 longitudinal 샘플(13:39:28.505)에서 body-lateral 잔차는 약 **0.004381 m**였지만 마지막 샘플(13:39:40.762)에는 **0.020621 m**로 커졌다. 마지막 body-longitudinal 잔차는 약 **0.001549 m**, radial은 **0.020679 m**로, longitudinal만 0으로 만들어도 정밀 radial 목표 **0.01 m**에 들어갈 수 없었다. 샘플 전체의 최소 radial도 **0.013687884 m**였다. Body 잔차는 기록된 localization XY 및 소수 둘째 자리 yaw로 재계산했으므로 근사값이다.

이는 단순한 마지막 저속 명령이나 torque 부족으로 단정할 수 없다. Longitudinal 실제 body 속도는 대략 **-0.116–+0.103 m/s**로 왕복했고, 기록된 FL external torque는 **-3.0–+1.065 Nm**였다. 마지막 샘플의 명령은 `vx=-0.000823 m/s`, 실제 body `vx≈-0.005038 m/s`, FL torque `-1.055 Nm`, 바퀴 조향 `0°`였다. 최초 longitudinal 샘플부터 마지막까지 CARLA actor XY를 최초 body 축에 투영하면 longitudinal 이동 **-0.234658 m**, lateral 이동 **+0.011255 m**로, 물리 차체의 횡방향 drift도 실제로 관측된다.

## 후속 판단 범위

고정 profile의 다른 사이트 분류를 계속하며 이 실패는 그대로 보존한다. 후보 수정은 optional `forward_loop` 정밀 centering 중에만 횡방향 잔차를 재확인하고 bounded lateral 단계로 다시 보내는 것이다. **아직 적용하지 않았고 성공을 주장하지 않는다.** 단순 시간 연장, torque 확대, 장애물 삭제 또는 `0.05 m` guard/최종 `0.25 m` 주차 기준 완화는 이 증거가 뒷받침하는 해결책이 아니다. 기존 fresh-pose 검사, 15 s centering/90 s 전체 deadline을 유지하는 테스트 및 실제 재검증이 필요하다.

## 실제 시각 증거

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/visual/representative_contact_sheet.png`)를 직접 확인했다. 출발·사이트 진입·최종 사이트 정지와 `Operator Stopped` 화면을 포함한다. 이 마지막 화면은 실패 후 상태이며, 위 controller timeout의 원인을 operator stop으로 바꾸지 않는다. 대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v5/B3_attempt02/visual/capture_manifest.json`)에 파일과 해시가 보존되어 있다. 캡처 길이 **186.0 s**; 원본 MP4는 정상 파생 후 제거되었다.
