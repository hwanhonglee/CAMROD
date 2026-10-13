# B10 V9 restart1 관측 왕복 완료: 13/20 recovery로 두 번째 회전 후 정상 복귀·주차

<!-- HH_261012 - Finalized diagnostic roundtrip, not formal/hardware acceptance or an all-sites same-binary claim. -->

[trial.json](trial.json.gz)의 결과는 **`COMPLETED_OBSERVED`**, `roundtrip_verified=true`, **`formal_acceptance=false`**다. [supervisor.json](supervisor.json)은 `finished`, trial/capture 종료 코드 **0/0**이며 [capture manifest](visual/capture_manifest.json)는 `PASS`다. 실제 B10 배달·사이트 회전·도로 복귀·native reverse parking을 완료했다. 이전 [V7 centering timeout](../archive/site_completion_20261012_v7/B10/ANALYSIS.md)과 [V8 10회 한도 실패](../archive/site_completion_20261012_v8_restart1/B10/ANALYSIS.md)는 그대로 보존한다.

- 실제 주행 **2026-10-12 16:54:41.956–17:01:31.747 KST**; capture/supervisor 완료 **17:02:32.049438 KST**.
- Actor **12**, mission generation **1791791602473001**, delivery / **turnaround**.
- 총 **409.820287 s / 217.426499 m**, outbound **137.411399 s / 52.741827 m**, return **272.260041 s / 164.684671 m**. 물리 actor body-XY **1770개** 샘플 기반이며 UI 속도 적분 거리와 구분한다([측정 정의](../B4/ANALYSIS.md)). 각 시간 구간은 별도 기록되어 합계와 작은 관측 간격 차이가 있다.
- 최종 localization XY **(-8.644358455324104,40.531853136613265)**와 station **(-8.47366,40.391)** 사이 **0.221308763 m**로, 변경하지 않은 native **0.25 m** 기준을 **0.028691237 m** 여유로 만족했다. 최종 drop-zone `parking_point_error_m=0.042`는 별도 approach 잔차이며 station XY 오차와 혼용하지 않는다.
- Reverse controller와 dispatcher 모두 **`PARKED`**, site **`DONE`**, gate **`STANDBY`**, 최종 명령 **(0,0,0)**. Collision publisher **1개**, event/sample/omitted/stale-queued 이벤트 모두 **0**.

첫 V9 bootstrap의 GPU/Vulkan crash는 dispatch 전 infrastructure 사건으로 [별도 기록](../archive/site_completion_20261012_v9/SERVER_RESTART.md)한다. 이번 결과는 restart1의 실제 주행이다. [고정 입력](../archive/site_completion_20261012_v9_restart1/frozen_inputs.json), [live profile](../archive/site_completion_20261012_v9_restart1/live_profile.json), [현재 시도 상태](../archive/site_completion_20261012_v9_restart1/STATUS.md)와 연결된다. 이 디렉터리에서 실제 시험한 사이트는 B10 하나이며, 다른 버전의 이전 사이트 성공을 이번 동일 binary의 13-site 재실행으로 표현하지 않는다.

## 실제 선택된 profile과 10회를 넘은 실행

`camrod-sites-20261012-v9-r1-camrod.service`의 **16:54:35–17:01:40 KST** journal에서 **16:57:00.608418** `ALIGN_RETRACE_YAW`는 **centering target 0.015 m / recenter trigger 0.035 m / steering settle 0.400 s / limit 20**을 보고했다. Recovery 로그와 마지막 active/final `site.message`도 같은 값을 유지한다.

실제 recovery **11/20, 12/20, 13/20**은 각각 **16:57:46.048541 / 16:57:50.149153 / 16:57:54.348696**에 시작했다. **16:57:59.548606**에는 `CRAB_OUT: forward_loop outbound yaw confirmed inside site; crab exit`가 기록됐다. 따라서 단순히 설정만 20으로 표시한 것이 아니라, V8의 10회 cap 이후 구간을 실제로 사용해 두 번째 heading change를 완료했다.

실제 lateral-realign marker는 **2회**, 모두 recovery 4에서 발생했다.

| 시각 KST | longitudinal error (m) | lateral error (m) | target (m) |
| --- | ---: | ---: | ---: |
| 16:57:20.447849 | -0.008809 | -0.016394 | 0.015000 |
| 16:57:22.948290 | -0.002094 | -0.016493 | 0.015000 |

Return-heading의 `steering_settle → longitudinal` 실제 로그 쌍은 **16개**, 간격 **0.400004–0.500001 s**였다. 같은 실행의 첫 service-turn entry에서는 **16:56:36.247239 → 16:56:37.547252**, **1.300013 s**로 짧은 departure override와 구분된다. Timer tick 간격을 포함하므로 실제 전환을 설정값과 정확히 같은 길이라고 주장하지 않는다.

## 변경하지 않은 위치·시간 guard의 실제 증거

- 전체 return-heading은 **58.940187 s**로 기존 **90 s** 이내이며, 13번의 recovery가 전체 deadline을 갱신하지 않았다.
- Heading 시작→첫 recovery는 **6.740220 s**. Recovery 시작→다음 recovery 또는 `CRAB_OUT`까지의 최대 구간은 recovery 4의 **7.901026 s**다. 이 구간들은 centering과 뒤따르는 yaw를 함께 포함하므로 centering 시간의 상한이며, 모두 기존 **15 s**보다 짧다.
- 최초 nonzero yaw 명령 sample은 **16:57:05.924388**, localization radial **0.008278727 m**. 이후 `CRAB_OUT`까지 **230개** 물리 sample에 연결된 localization radial 최대는 **0.046313096 m**(**16:57:39.418599**)로 active hard guard **0.05 m**보다 **0.003686904 m** 작다. 마지막 heading sample의 radial은 **0.033149620 m**다.
- 위 최대는 최대 5 Hz의 비동기 기록 표본 범위이며, 연속시간 최대치나 hardware 안전 여유 인증이 아니다. 실제 guard는 유지된 채 실행됐고, 이번에는 breach/centering timeout/전체 timeout/cap 오류가 기록되지 않았다.

## 도로 handoff와 최종 주차

**16:58:07.549625** site `DONE`은 live lanelet handoff **lateral error 0.11 m**, original-anchor error **0.27 m**를 보고했다. 이어 **16:58:07.553335** planner가 `forward_one_way` 복귀를 승인했고, **23 lanelets / 799 points / 158.52 m**의 `mode=one_way` 경로가 기록됐다. 최종 `return_published=True`, `return_ack=True`다.

**17:01:31.355308** reverse controller가 `PARKED: station XY goal reached`를 기록했다. 실제 final observation의 station 거리 **0.221308763 m**와 함께 최종 주차 완료를 확인했다. Site `DONE`의 과거 turn target/yaw field를 최종 station 주차 방향 오차로 해석하지 않는다.

## 실제 화면·파생 파일 검증

[대표 PNG](visual/representative_contact_sheet.png)를 직접 확인했다. 실제 CARLA/운영 UI에서 B10 접근·사이트 회전, 도로 복귀, 최종 station의 차량과 서비스 선택 화면이 보인다. UI의 약 **228 m** 표시는 속도 적분 지표로, 본문의 sampled body-XY **217.426499 m**와 같은 거리 정의라고 주장하지 않는다. 번역 popup 등 화면은 원본 그대로다.

[대표 GIF](visual/representative_motion.gif)와 PNG의 실제 SHA-256이 [manifest](visual/capture_manifest.json)와 각각 일치함을 확인했다. 캡처 **415.4 s / 5 fps**, 정상 파생 후 원본 MP4 제거 정책이 적용됐다.

- PNG SHA-256: `4018e9dc2c4f9fc296ca8316c7d0ce614c7b4d199ba2616e2f237a874511f62a`
- GIF SHA-256: `448a28bbfdc2b2bbff0849e806df2c1c856daf408ad15e3cc7ea3f6927f42c3c`

이 결과는 **해당 CARLA delivery 진단 시나리오의 관측 완료**다. Hardware/guest/recall/배포 formal acceptance 또는 모든 사이트의 동일 최종 binary 재검증을 주장하지 않는다. 분석 중 source/config/runtime/gate/vehicle API 변경 없이 finalized 파일·journal 읽기, 실제 이미지 확인과 이 Markdown 추가만 수행했다.
