# B10 V7 실패: 실제 0.015/0.035 m profile에서 recovery 4의 15 s timeout

<!-- HH_261012 - Preserve finalized failure, actual profile selection and dynamic branch evidence. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v7/B10/trial.json`)은 **`FAILED`**, `roundtrip_verified=false`, `formal_acceptance=false`다. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v7/B10/supervisor.json`)은 `finished`, trial/capture 종료 코드 **1/0**. Capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v7/B10/visual/capture_manifest.json`)의 `PASS`는 캡처 성공만 뜻한다. [V5 위치 guard 실패](../../site_completion_20261012_v5/B10_attempt02/ANALYSIS.md), [V6 recovery 3 centering timeout](../../site_completion_20261012_v6/B10/ANALYSIS.md)에 이어 V7에서도 왕복을 완료하지 못했다.

- 실제 주행 **2026-10-12 15:13:55.537–15:16:50.782 KST**; 캡처 후처리 완료 **15:17:16.234 KST**.
- Actor **332**, mission generation **1791785628461001**, delivery / **turnaround**.
- 총 **175.275585 s / 53.457734 m**; outbound **137.606854 s / 52.681574 m**; 물리 actor body-XY **760개** 기록 샘플. `return_leg=null`. Total − outbound **37.668730 s / 0.776160 m**는 미완료 구간이며 완료 복귀 지표가 아니다. UI 속도 적분 거리와 구분한다([정의](../../../B4/ANALYSIS.md)).
- 최종 localization XY **(9.01281058273523,16.50586225159954)**에서 station **(-8.47366,40.391)**까지 **29.601967142420264 m**. 최종 parking에 도달하지 않았고 native **0.25 m** 기준 미달이다.
- 최종 site `ERROR`, parking `IDLE`, gate `ENABLED`, 명령 `(0,0,0)`. Collision publisher **1개**, 기록·누락·오래된 큐 이벤트 각각 **0**.

## 실제 활성 profile과 새 분기 실행

`camrod-sites-20261012-v7-camrod.service`의 **15:13:03–15:17:17 KST** journal에서 **15:16:14.369124** `ALIGN_RETRACE_YAW`는 `centering_tolerance_m=0.015 recenter_trigger_m=0.035`를 보고했다. 네 번의 recenter 로그에도 `target_m=0.015 trigger_m=0.035`가 기록됐고, ERROR 전 실제 `site.message`에도 같은 활성 값이 있다. **최종 ERROR는 profile과 count를 clear하여 0.010/0.025 m와 count 0을 표시한다. 이는 실행 중 profile이 잘못 선택됐다는 증거가 아니다.**

`forward_loop precise centering lateral realign`은 **정확히 4회**, 모두 recovery 4 안에서 실제 실행됐다. 아래는 embedded log 시각이며 각 target은 **0.015000 m**다.

| 실제 시각 KST | longitudinal error (m) | lateral error (m) |
| --- | ---: | ---: |
| 15:16:38.931767 | -0.011639 | -0.016693 |
| 15:16:41.732904 | -0.014922 | -0.016779 |
| 15:16:44.731887 | -0.014777 | -0.017062 |
| 15:16:47.732228 | -0.013735 | -0.015661 |

이는 소스 존재나 replay 조건 추론이 아닌 실제 분기 실행 증거다. 반면 [V7 B9 완료](../../../B9/ANALYSIS.md)의 realign marker는 0회였다. 두 결과를 혼합하지 않는다.

## 직접 실패 조건과 유지된 hard guard

- Return-heading 시작 **15:16:14.369124**. 첫 nonzero yaw 명령 샘플 **15:16:20.568520**에서 radial **0.00733066047119115 m**로 최초 precenter를 통과했다.
- Recovery 1/2/3 시작은 각각 **15:16:22.131973 / 15:16:26.731861 / 15:16:31.232289**이며 완료 후 회전이 재개됐다.
- Recovery **4/10**은 **15:16:35.732267** 시작, ERROR는 **15:16:50.732331**. 정확한 차이는 **15.000064 s**, 전체 heading phase는 **36.363207 s**다. 따라서 직접 조건은 **각 centering 15 s deadline**이며 전체 **90 s**나 **10회 cap** 초과가 아니다. Realign 네 번이 deadline을 초기화하지 않았음도 실제 시간으로 확인된다.
- 첫 yaw 명령 이후 마지막 물리 샘플까지 localization radial 최대는 **0.04241382455012275 m**(15:16:36.400288), 최종 radial **0.021485201346074873 m**. 기록 샘플은 변경하지 않은 active **0.05 m hard guard** 이내다. 최대 5 Hz 비동기 샘플이므로 연속시간 최대치라고 주장하지 않는다.

최종 yaw **52.69°**, 고정 target **-69.18°**, 남은 yaw **-121.87°**다. 정밀 목표를 `0.015 m`로 선택해도 lateral/longitudinal 잔차를 deadline 안에 수렴시키지 못했다. `CRAB_OUT`/`DONE`, 도로 handoff, `return_published`/`return_ack`는 없다.

V7은 B9 왕복을 완료했지만 **B10은 미해결**이다. 이 실패를 위치·시간·횟수 제한 확대나 캡처 `PASS`로 성공 처리하지 않는다. 본 분석 중 source, config, gate, runtime 또는 vehicle API 변경은 없었다.

## 보존된 실제 실패 화면

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v7/B10/visual/representative_contact_sheet.png`)를 직접 확인했다. 배달 확인, 출발·B10 진입·사이트 내 회전 및 최종 controller ERROR 화면을 보존했다. 번역 팝업도 원본 그대로다. 대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v7/B10/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v7/B10/visual/capture_manifest.json`)에 파생 파일·해시가 있다. 캡처 **180.8 s / 5 fps**; 원본 MP4는 정상 파생 후 제거되었다. 실패 이미지와 이전 실행 기록을 유지한다.
