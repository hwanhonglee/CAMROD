# B10 V8 restart1 실패: 0.4 s dwell 적용 후 recenter 10회 한도

<!-- HH_261012 - Preserve the finalized failure and distinguish measured progress from extrapolated reachability. -->

trial.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/trial.json`)은 **`FAILED`**, `roundtrip_verified=false`, `formal_acceptance=false`다. supervisor.json (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/supervisor.json`)은 `finished`, trial/capture 종료 코드 **1/0**이며, capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/capture_manifest.json`)의 `PASS`는 캡처 성공만 뜻한다. [V7 B10](../../site_completion_20261012_v7/B10/ANALYSIS.md)의 recovery 4 centering timeout과 달리 이번 직접 종료 조건은 **`recenter recovery limit reached: 10/10`**이다. 왕복·최종 주차는 완료하지 못했다.

- 실제 주행 **2026-10-12 15:51:09.284–15:54:19.503 KST**; 캡처 후처리 완료 **15:54:45.774758 KST**.
- Actor **12**, mission generation **1791787794944001**, delivery / **turnaround**.
- 총 **190.248137 s / 54.221423 m**, outbound **137.777972 s / 52.906908 m**, 물리 actor body-XY 기록 **826개**. `return_leg=null`; total−outbound **52.470165 s / 1.314515 m**는 미완료 구간이지 완료 복귀 지표가 아니다. UI 속도 적분 거리와 구분한다([측정 정의](../../../B4/ANALYSIS.md)).
- 최종 localization XY **(9.01109907571328,16.51955759935794)**에서 station **(-8.47366,40.391)**까지 **29.589906425 m**. Native 최종 주차 **0.25 m** 기준을 충족하지 않았다.
- 최종 site `ERROR`, parking `IDLE`, gate `ENABLED`, 명령 **(0,0,0)**. Collision publisher **1개**, event/sample/omitted/stale-queued 이벤트 모두 **0**.

앞선 V8 bootstrap의 GPU/Vulkan server crash는 **이 actor의 dispatch 전 infrastructure 실패**다. 이번 restart1의 실제 controller 실패와 합치지 않는다. 새 server에서 같은 map을 사용한 실행의 [상태 기록](../STATUS.md), [고정 입력](../frozen_inputs.json), [후보 근거](../../site_completion_20261012_v8/CANDIDATE.md)를 함께 보존한다.

## 실제 0.4 s 적용과 native dwell 보존

`camrod-sites-20261012-v8-r1-camrod.service`의 **15:51:00–15:54:25 KST** journal에서 **15:53:28.205592** `ALIGN_RETRACE_YAW`와 열 번의 recovery 로그는 모두 **target 0.015 m / trigger 0.035 m / steering settle 0.400 s**를 보고한다. 마지막 active sample도 이 profile과 count **10**을 표시한다. ERROR 이후에는 mission-local profile/count가 clear되어 **0.010/0.025/1.200, count 0**을 표시한다. 이를 실제 실행 중 default profile을 사용했다는 증거로 해석하지 않는다.

Return-heading에서 실제 `steering_settle → longitudinal` 로그 쌍은 **15개**, 간격은 **0.400001–0.500480 s**였다. 설정값만이 아니라 짧아진 실제 stage 전환을 확인했다. Timer tick/관측 차이 때문에 모든 실제 간격을 정확히 0.4 s라고 부르지 않는다. 같은 실행의 첫 service-turn entry에서는 **15:53:03.216686 → 15:53:04.416721**, **1.200036 s**로 native dwell이 유지됐다.

실제 lateral-realign marker는 **4회**였다. Recovery 5에서 1회, recovery 6에서 3회 발생했고 이후 yaw 회전이 재개됐다.

| 실제 시각 KST | longitudinal error (m) | lateral error (m) |
| --- | ---: | ---: |
| 15:53:54.217605 | -0.010609 | -0.015435 |
| 15:53:59.417536 | -0.014695 | -0.015760 |
| 15:54:01.417898 | -0.011883 | -0.015453 |
| 15:54:03.617645 | -0.006759 | -0.015619 |

이는 소스 존재나 replay 조건이 아닌 실제 새 분기 실행 증거다. V7에서는 recovery 4의 반복 보정이 15 s에 종료됐지만, 이번에는 모든 recovery가 다음 yaw 구간으로 진행했다. 서로 다른 실제 실행을 비교한 관측이며 dwell 하나의 인과 효과를 인증한 것은 아니다.

## 직접 종료 조건과 유지된 위치·시간 제한

- Return-heading 시작 **15:53:28.205592**, ERROR **15:54:19.418161**: 전체 phase **51.212569 s**, 기존 **90 s** 이내다.
- Recovery 10 시작 **15:54:16.218470**, 그로부터 ERROR까지 **3.199691 s**. ERROR 직전 active detail은 `precise site center restored; error_m=0.009 ... recenter_recoveries=10/10`이고 yaw 명령이 재개된 상태였다. 다음 trigger에서 11번째 recovery가 거부됐다. **각 centering 15 s timeout이나 0.05 m breach가 직접 종료 조건이 아니다.**
- 최초 nonzero yaw 명령 sample **15:53:35.200176**의 radial은 **0.003190443 m**. 이 이후 ERROR까지의 physical-sample에 연결된 localization radial 최대는 **0.045088185 m**(**15:53:37.978554**), 변경하지 않은 active hard guard **0.05 m**보다 **0.004911815 m** 작다. 최종 observation의 site-center radial은 **0.035188332 m**다. 최대 5 Hz 비동기 표본이므로 연속시간 최대치나 안전 여유 보증은 아니다.
- 최초 heading→recovery 1 전체 구간은 **8.212149 s**. Recovery 시작→다음 recovery/ERROR 전체 구간의 최대는 recovery 6의 **9.300792 s**다. 이 구간들은 centering과 뒤따르는 yaw를 함께 포함하므로 해당 centering 시간의 상한이며, 모두 **15 s**보다 짧다. 각 reset이 전체 **90 s** deadline을 갱신하지 않았음도 실제 phase 시간과 함께 확인된다.

최종 yaw **-3.87°**, 고정 target **-69.18°**, 남은 yaw **-65.32°**. `CRAB_OUT`/`DONE`, 도로 handoff, `return_published`/`return_ack`, 최종 station parking은 없다.

## 회전 진척과 아직 증명되지 않은 완주 가능성

아래는 atomic physical body yaw를 unwrap하고 journal recovery 경계 시각에 선형 보간한 값이다. 각 구간은 **centering + 이어진 yaw 회전**이며, command-only yaw나 정확한 controller tick 변화량이 아니다.

| Recovery 시작→다음 recovery/ERROR | 구간 시간 (s) | 물리 net yaw 진척 (°) |
| --- | ---: | ---: |
| 1 | 4.299904 | 15.803 |
| 2 | 3.500036 | 11.461 |
| 3 | 3.502388 | 9.497 |
| 4 | 3.799871 | 10.983 |
| 5 | 5.699200 | 7.328 |
| 6 | 9.300792 | 8.852 |
| 7 | 3.498368 | 6.603 |
| 8 | 3.000338 | 8.238 |
| 9 | 3.199833 | 12.450 |
| 10 | 3.199691 | 9.209 |

열 구간 모두 같은 방향의 net 진척이며 최소 **6.603°**, 평균 **10.042°**다. 관측 평균을 그대로 연장하면 남은 65.32°에 약 **6.50회 / 27.97 s**, 전체 약 **79.18 s**가 필요하다는 산술 추정이 나온다. 그러나 가장 느린 실제 구간의 속도가 반복되거나 아직 지나지 않은 body heading에서 drift가 커지면 **90 s 안의 완주는 보장되지 않는다**. 이 계산은 후속 후보 검토 자료일 뿐, 이번 실패를 성공으로 바꾸거나 횟수/시간/위치 제한 변경을 승인하는 근거가 아니다.

## 보존된 실제 실패 화면

대표 PNG (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/representative_contact_sheet.png`)를 직접 확인했다. 실제 CARLA/운영 UI의 출발, B10 접근·진입, 도착 후 복귀 확인과 마지막 **Operator Stopped** 화면이 있다. 마지막 화면은 trial 오류 후 기록된 production stop cleanup과 양립하며 왕복 완료 화면이 아니다. 번역 popup을 포함해 원본 그대로 유지했다.

대표 GIF (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/representative_motion.gif`), capture manifest (LOCAL_ARCHIVE: `docs/evidence/virtual_carla/site_completion_20261012_v8_restart1/B10/visual/capture_manifest.json`)에 실제 파생 파일과 해시가 있다. 캡처는 **195.8 s / 5 fps**이고 정상 파생 후 원본 MP4 제거 정책이 적용됐다. 본 분석은 finalized 파일·journal 읽기와 이 Markdown 추가만 수행했으며 source/config/runtime/gate/vehicle API 변경은 없었다.
