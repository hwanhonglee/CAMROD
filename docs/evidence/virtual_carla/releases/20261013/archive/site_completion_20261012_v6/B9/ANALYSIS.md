# B9 V6 실패: 실제 lateral realign 실행 후 10회 recenter 한도 도달

<!-- HH_261012 - Finalized physical result and actual branch logs; do not infer execution from source presence. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/trial.json`)은 **`FAILED`**, `roundtrip_verified=false`, `formal_acceptance=false`다. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/supervisor.json`)은 `finished`, trial/capture 종료 코드 **1/0**. Capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/visual/capture_manifest.json`)의 `PASS`는 캡처 성공만 뜻한다. [V5 B9의 최초 centering 15 s timeout](../../site_completion_20261012_v5/B9_attempt02/ANALYSIS.md)과 달리, V6는 최초 centering을 통과하고 **recenter recovery 10/10 한도**에서 정지했다.

- 실제 주행 **2026-10-12 14:34:05.036–14:37:45.655 KST**; 캡처 후처리 완료 **14:38:16.525 KST**.
- Actor **268**, mission generation **1791783236935001**, delivery / **turnaround**.
- 총 **220.645382 s / 55.093284 m**, outbound **138.905003 s / 53.868711 m**, 물리 actor body-XY **956개** 기록 샘플. `return_leg=null`. Total − outbound **81.740380 s / 1.224573 m**는 미완료 구간이지 완료 복귀 지표가 아니다. UI 속도 적분 거리와 구분한다([정의](../../../B4/ANALYSIS.md)).
- 최종 localization XY **(15.231008411801694, 17.298689308914216)**에서 station **(-8.47366,40.391)**까지 **33.09329414801591 m**. 최종 parking에 도달하지 않았으며 native **0.25 m** 기준 미달이다.
- 최종 site `ERROR`, parking `IDLE`, gate `ENABLED`, 명령 `(0,0,0)`. Collision publisher **1개**, 기록·누락·오래된 큐 이벤트 각각 **0**.

## 새 분기가 실제로 실행된 증거

`camrod-sites-20261012-v6-camrod.service`의 **14:36:25–14:37:47 KST** journal에서 `forward_loop precise centering lateral realign`을 **정확히 11회** 확인했다. 소스에 분기가 존재한다는 추론이나 오프라인 replay count가 아니다. 각 로그의 target은 **0.010000 m**, 아래 잔차 단위는 m이다.

| 실제 시각 KST | 해당 centering episode | longitudinal | lateral |
| --- | --- | ---: | ---: |
| 14:36:31.377 | 최초 precenter | -0.009911 | +0.018258 |
| 14:36:44.078 | recovery 2 | -0.005134 | +0.010857 |
| 14:36:46.678 | recovery 2 | -0.008933 | +0.010199 |
| 14:36:49.578 | recovery 2 | -0.006786 | +0.010743 |
| 14:36:52.478 | recovery 2 | -0.006124 | +0.010461 |
| 14:37:23.779 | recovery 9 | +0.004326 | +0.010139 |
| 14:37:26.479 | recovery 9 | +0.002689 | +0.010220 |
| 14:37:33.278 | recovery 10 | +0.009881 | +0.013389 |
| 14:37:35.878 | recovery 10 | +0.009060 | +0.010451 |
| 14:37:38.578 | recovery 10 | +0.006719 | +0.010567 |
| 14:37:41.779 | recovery 10 | +0.002557 | +0.010691 |

최초 return-heading phase는 **14:36:25.162**에 시작했고, 첫 realign은 약 **6.215 s** 뒤에 발생했다. 첫 실제 yaw 명령 샘플은 **14:36:34.875051**, `w=+0.45 rad/s`, radial **0.007165011937173028 m**였다. 따라서 원래 실패한 최초 정밀 centering 단계는 이번 실행에서 통과했다. 다만 이 부분 개선을 전체 왕복 성공으로 표현하지 않는다.

## 실제 종료 조건과 유지된 경계

Bounded recenter는 **14:36:36.778의 1/10 → 14:37:30.179의 10/10**으로 진행했다. **14:37:45.578931** embedded log에서 `forward_loop recenter recovery limit reached: 10/10; immutable turn guard retained`로 ERROR에 진입했다. 전체 heading phase는 **80.416927 s**이며 **90 s timeout이 아니다**. Final yaw **-117.15°**, 고정 target **-68.22°**, 남은 yaw **48.93°**였다.

첫 yaw 명령 이후 마지막 물리 샘플까지 localization radial 최대는 **0.0325273079672023 m**이고 최종 radial은 **0.026948928033532642 m**로, 기록 샘플에서는 active **0.05 m guard**를 초과하지 않았다. 각 centering **15 s**, 전체 **90 s**, 최대 **10회** 제한을 늘리지 않았다. Repeated lateral realign과 회전 drift 보정에 시간이 소요됐지만, 직접 종료 원인은 명시적인 **recovery 횟수 제한**이다. ERROR 진입 후 count가 clear되어 최종 상태에 `0`이 남는 것을 실제 recovery 0회로 해석하지 않는다.

`CRAB_OUT`/`DONE`이나 도로 복귀 handoff는 없고 `return_published=false`, `return_ack=false`다. V6는 새 분기의 실제 작동과 최초 centering 통과를 입증했으나 B9 왕복 완료는 입증하지 못했다. 이 결과로 안전 제한을 확대하거나 성공으로 재분류하지 않는다.

## 실제 실패 시각 증거

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/visual/representative_contact_sheet.png`)를 직접 확인했다. 출발·B9 진입·사이트 내 회전/정지·최종 `Operator Stopped` 화면을 원본 그대로 보존했다. 시작 화면의 이전 stop 상태와 종료 화면만으로 이번 실패 원인을 판정하지 않으며, 위 controller journal이 직접 근거다. 대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B9/visual/capture_manifest.json`)에 파생 파일·해시가 있다. 캡처 **227.0 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되었다.
