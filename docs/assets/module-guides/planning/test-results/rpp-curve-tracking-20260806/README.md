# RPP Curve Tracking And Right-Oversteer Check - 2026-08-06

<!-- HH_260806 - Record the reproduced stop-turn-forward defect and the
selected continuous-curve controller contract. -->

이 기록은 곡선에서 차체가 오른쪽으로 과도하게 돌아가고 전진과 정지를
반복한 현상을 같은 AMD64 full-bringup simulation 경로에서 비교한 결과다.

![RPP curve-tracking comparison](rpp-curve-tracking-comparison.png)

## Root Cause

기존 `RPP.use_rotate_to_heading=true`와 `2 deg` 임계값은 시작 자세만
정렬한 것이 아니다. RPP가 매 주기 선택하는 경로 carrot의 방위가 2도만
달라도 선속도를 0으로 만들고 제자리 회전을 다시 요청했다. B7 기록에서
이 동작은 Nav2 원시 명령의 `rotation <-> translation` 전환을 403회
만들었다. 화면에서는 오른쪽 오버스티어와 `멈춤 -> 회전 -> 전진` 반복으로
보였다.

## Selected Control Contract

| Responsibility | Selected value |
|---|---:|
| Ordinary RPP curve tracking | `use_rotate_to_heading=false` |
| RPP lookahead floor | `1.1 m` |
| Gross route-start alignment | gate enter `75 deg`, exit `5 deg` |
| Manual RotationShim engagement | enter `45 deg`, release `5 deg` |
| Gross alignment linear velocity | `0.0 m/s` |

큰 시작 오차는 최종 safety gate가 한 번만 정렬한다. 그 뒤 RPP는 곡선에서
선속도와 각속도를 동시에 출력하므로 작은 경로 각도마다 제자리 회전으로
재진입하지 않는다.

## Same-Map Comparison

| Case | Raw R/T switches | Pose path | Route result | Boundary result |
|---|---:|---:|---|---|
| Before: RPP `2 deg` rotate mode, `1.1 m` | `403` | `27.15 m` before real hold | Not reached | Stopped at a real planning-margin contact |
| Selected: continuous RPP, `1.1 m` | `0` | `59.931 m` | `GOAL_REACHED` in `321.400 s` | One margin contact recovered, then completed |
| Rejected: continuous RPP, `1.2 m` | `0` | `26.033 m` | Not reached | Released once, recontacted in `0.999 s`, retry latched |

The selected `1.1 m` run emitted one `19.2 s` pure-rotation interval at the
start because the simulated initial heading differed from the route by
approximately 180 degrees. Raw RPP emitted no pure-rotation sample during the
route. The final gate saw one recoverable 5 cm planning-margin contact near
`(11.031, 43.572)` and returned control to Nav2; the route then reached the
goal near `(14.791, 10.690)`.

Increasing lookahead to `1.2 m` did not fix the right-side excursion. It
contacted the planning margin near `(11.002, 43.751)`, released, and contacted
the opposite/continued margin near `(11.007, 43.552)`. The existing rapid
recontact policy correctly latched output instead of repeatedly challenging
the same corridor.

## Scope

- This is AMD64 kinematic simulation evidence, not physical steering calibration.
- The map used in all final comparisons has SHA-256
  `559bdad88e2733c854024013e02c2f29e4ad4aaef5bc6678e3c593d6c4e49516`.
- The B8 validation runner reached the lanelet navigation goal through its
  manual-goal path. It does not prove the subsequent campsite maneuver,
  unload, return, parking, or charging phases.
- A persistent one-sided bias on the real robot after this correction must be
  checked against steering-center offsets, front/rear steering ratio, wheel
  speed asymmetry, and localization yaw latency; those effects are not modeled
  by this simulation.

Machine-readable values and raw bag hashes are in
[`result.json`](result.json). The bags remain under `/tmp` and are not committed.

