# B3 V6 완료 관측 — 새 재정렬 분기 직접 실행과 구분

<!-- HH_261012 - Finalized physical trial and capture only; retain the V5 failure. -->

[trial.json](trial.json.gz)은 **`COMPLETED_OBSERVED`**, `roundtrip_verified=true`, `formal_acceptance=false`이다. [supervisor.json](supervisor.json)은 `finished`, trial/capture 종료 코드 모두 **0**, [capture manifest](visual/capture_manifest.json)는 **PASS**이다. 이는 V6 B3 한 실행의 CARLA 완료 관측이며 동일 V6 profile 전체 13개 성공이나 실제 로봇 인증이 아니다. [V5 B3 실패](../archive/site_completion_20261012_v5/B3_attempt02/ANALYSIS.md)는 그대로 보존한다.

- 실제 주행: **2026-10-12 14:24:44.397–14:31:22.164 KST**; 캡처 후처리 종료 **14:32:24.053 KST**.
- Actor **252**, mission generation **1791782600687001**, delivery / 기존 **turnaround** 유지.
- Trial에 기록된 physical gate SHA256: `07d38ccc60e4920340107021b0a2c97a849e6b4919c4c7d03ec7e28fcfe10c16`.
- 총 **397.796 s / 215.957646 m**, outbound **163.111 s / 72.771374 m**, return **234.483 s / 143.186272 m**. 거리는 1,711개 기록 샘플에 기반한 물리 actor XY 거리이며 UI 누적 거리나 계획 경로 길이와 구분한다.
- 최종 localization XY **(-8.662177041317175, 40.517694629722726)**. 고정 station **(-8.47366,40.391)**과의 거리 **0.2271347707145703 m**, 변경하지 않은 native 최종 주차 기준 **0.25 m** 이내.
- 최종 reverse parking `PARKED`, site `DONE`, gate `STANDBY`. Collision publisher 1개 발견, 기록 이벤트 **0**, 누락/오래된 큐 이벤트도 **0**. 미관측 접촉까지 부정하는 주장은 아니다.

## 실제 서비스·회전·복귀 증거

기록 순서는 `IDLE → ALIGN_ENTRY_YAW → CRAB_IN → ROTATE_180 → UNLOAD_WAIT → WAIT_RETURN → ALIGN_RETRACE_YAW → CRAB_OUT → DONE`이다. 기존 첫 서비스 회전과 도착 대기를 유지한 채 사용자 Return 이후 두 번째 사이트 내 회전, crab exit, 합법적 정방향 도로 복귀와 최종 reverse parking을 마쳤다.

`camrod-sites-20261012-v6-camrod.service`의 **14:24:44–14:31:24 KST** 범위 journal을 확인했다.

| 시각 KST | 관측 |
| --- | --- |
| 14:27:28.725 | `ALIGN_RETRACE_YAW`, `forward_loop`, precise-center target `0.010 m` |
| 14:27:37.058–14:28:11.859 | Pivot pause/recenter **1/10 → 9/10**; 남은 target yaw와 전체 phase deadline 보존 |
| 14:28:16.258 | 사이트 안에서 outbound yaw 확인 후 `CRAB_OUT` |
| 14:28:24.061 | `DONE`: live handoff `(19.85,-0.99)`, projection `(19.70,-1.06)`, lateral error `0.17 m` |
| 14:28:24.063 | `forward_one_way`, source `camping_site_maneuver_controller:done_live_lanelet:site_policy_roadside_forward` 인증 |
| 14:28:24.186 | `start_ll=2193`, `goal_ll=751`, `one_way`, `reverse_source=roadside_forward`, heading error **3.9°**, **20 lanelets / 138.07 m** 계획 |

두 번째 회전 단계는 embedded log 시각 기준 **47.535092 s**, 전체 90 s 한도 이내였으며 최종 상태에 **9/10** recenter가 남았다. 최초 precenter 샘플 center error는 **0.235418209 m**였다. 최초 precenter를 제외한 회전/중간 recenter 샘플에서 target `(22.5064,0.150633)`과 localization XY의 최대 거리는 **0.031239503 m < 0.05 m**였다. 이산 샘플 최대치이며 연속시간 최대치라는 주장은 아니다.

## V6 수정 분기 검증의 한계

이 B3 실행의 bounded journal에서 새 marker **`forward_loop precise centering lateral realign` 호출은 0회**였다. 이번 초기 centering은 radial 목표에 자연스럽게 도달했고, 이후 pivot의 9회 recenter도 **기존 segmented-pivot 회복**이다. 따라서 **이번 완료를 새 lateral-realignment 분기가 물리적으로 실행되어 V5 실패를 해결한 직접 인과 증거로 해석하지 않는다.** 새 binary의 B3 왕복 완료와 그 분기의 직접 실행 증거는 별개다. B9/B10 및 B2 회귀 결과를 따로 확인해야 한다.

실제 focused gtest XML (LOCAL_ARCHIVE: `/home/hong/camrod_ws/build/camrod_control/test_results/camrod_control/test_camping_site_recall_return.gtest.xml`)을 읽어 **2026-10-12 14:20:37, 54 tests / 0 failures / 0 errors**를 확인했다. 측정 잔차의 재정렬, reachable lateral 유지, radial 성공 우선, deadline/position/latch fault 보존, non-finite no-reset, native 불변 테스트가 포함되어 있다. 이 단위 검사를 위에서 관측되지 않은 물리 분기 실행으로 대체하지 않는다.

## 실제 시각 증거

[대표 PNG](visual/representative_contact_sheet.png)를 직접 확인했다. 출발, 사이트 서비스, 실제 도로 복귀 및 최종 주차/서비스 화면을 포함한다. [대표 GIF](visual/representative_motion.gif), [capture manifest](visual/capture_manifest.json)에 파생 파일 및 해시가 보존되어 있다. 캡처 길이는 **403.4 s**, 원본 MP4는 정상 파생 후 제거되었다.
