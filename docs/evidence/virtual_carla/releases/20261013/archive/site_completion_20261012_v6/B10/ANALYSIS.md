# B10 V6 실패: lateral realign 4회 후 recovery 3의 15 s timeout

<!-- HH_261012 - Actual branch executions and finalized failure; preserve the immutable 0.05 m guard. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/trial.json`)은 **`FAILED`**, `roundtrip_verified=false`, `formal_acceptance=false`다. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/supervisor.json`)은 `finished`, trial/capture 종료 코드 **1/0**. Capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/visual/capture_manifest.json`)의 `PASS`는 캡처 성공만 뜻한다. [V5 B10](../../site_completion_20261012_v5/B10_attempt02/ANALYSIS.md)은 recovery 2 중 **0.05 m 위치 guard**를 초과했지만, V6의 직접 종료 조건은 **recovery 3의 15 s centering timeout**이다.

- 실제 주행 **2026-10-12 14:39:10.072–14:42:03.349 KST**; 캡처 후처리 완료 **14:42:28.705 KST**.
- Actor **284**, mission generation **1791783541951001**, delivery / **turnaround**.
- 총 **173.304091 s / 53.719343 m**, outbound **138.393625 s / 52.902127 m**, 물리 actor body-XY **753개** 기록 샘플. `return_leg=null`. Total − outbound **34.910466 s / 0.817215 m**는 미완료 구간이지 완료 복귀 지표가 아니다. UI 속도 적분 거리와 구분한다([정의](../../../B4/ANALYSIS.md)).
- 최종 localization XY **(9.01589538627895,16.504984394416194)**에서 station **(-8.47366,40.391)**까지 **29.604497785301344 m**. 최종 parking 미도달이며 native **0.25 m** 기준 미달이다.
- 최종 site `ERROR`, parking `IDLE`, gate `ENABLED`, 명령 `(0,0,0)`. Collision publisher **1개**, 기록·누락·오래된 큐 이벤트 각각 **0**.

## 실제 새 분기 실행

`camrod-sites-20261012-v6-camrod.service`의 **14:41:29–14:42:04 KST** journal에서 `forward_loop precise centering lateral realign`을 **정확히 4회** 확인했다. 전부 recovery 3 안에서 실행됐으며 소스 존재나 오프라인 조건 평가를 실행 증거로 대체하지 않았다. 각 로그의 target은 **0.010000 m**다.

| 실제 시각 KST | longitudinal error (m) | lateral error (m) |
| --- | ---: | ---: |
| 14:41:51.371744 | -0.008793 | -0.010859 |
| 14:41:54.471793 | -0.007603 | -0.011973 |
| 14:41:57.671838 | -0.008247 | -0.012238 |
| 14:42:00.872150 | -0.006447 | -0.010639 |

Return-heading phase는 **14:41:29.754601** 시작했다. 첫 yaw 명령 샘플 **14:41:37.749249**의 radial은 **0.005377968125511651 m**로 최초 정밀 precenter는 통과했다. Recovery 1/2는 각각 **14:41:38.871830 / 14:41:43.772325**에 시작해 완료했고 recovery 3은 **14:41:48.272000**에 시작했다.

## 유지된 guard와 실제 종료 조건

첫 yaw 명령부터 마지막 물리 샘플까지 localization radial 최대는 **0.039613862019693205 m**(14:41:40.553150), 최종 radial은 **0.019982195088124887 m**였다. 기록 샘플은 변경하지 않은 active **0.05 m** guard 안에 있었다. 따라서 V5의 위치 guard 초과를 이번 V6 실패 원인으로 재사용하지 않는다.

ERROR embedded 시각은 **14:42:03.272104**로 recovery 3 시작 후 **15.000104 s**, 전체 heading phase **33.517504 s**다. Repeated lateral realign이 centering deadline을 초기화하지 않았다는 실제 시간 근거이며, 전체 **90 s**나 recovery **10회** 한도 초과가 아니다. 최종 yaw **73.22°**, target **-69.18°**, 남은 yaw **-142.40°**. ERROR에서 count가 clear되므로 최종 `0`을 실제 recovery 0회로 해석하지 않는다.

새 분기는 lateral 잔차가 radial 목표보다 큰 상태를 실제로 다시 보정했지만, 이 episode는 **0.01 m** 정밀 목표 안에서 완료되지 못했다. `CRAB_OUT`/`DONE` 또는 도로 handoff 없이 `return_published=false`, `return_ack=false`로 정지했다. **위치 guard 보존과 새 분기 실행은 확인했지만 왕복 성공은 아니다.** V6의 B3 성공과 합쳐 B10 성공으로 집계하지 않으며, 시간·위치·횟수 제한을 늘려 통과 처리하지 않는다.

## 실제 실패 시각 증거

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/visual/representative_contact_sheet.png`)를 직접 확인했다. 배달 확인, 출발·B10 진입·사이트 내 회전과 최종 controller ERROR 화면이 보이며 번역 팝업도 원본 그대로다. 대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v6/B10/visual/capture_manifest.json`)에 파생 파일·해시가 있다. 캡처 **178.8 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되었다. 캡처 성공은 임무 성공으로 집계하지 않는다.
