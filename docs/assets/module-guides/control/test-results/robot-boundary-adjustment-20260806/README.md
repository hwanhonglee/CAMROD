# Provisional Robot Boundary Adjustment - 2026-08-06

<!-- HH_260806 - Preserve the prior robot boundary and document the provisional reduction. -->

이 기록은 기존 robot boundary 실측이 잘못되었을 가능성을 확인하기 위해
앞·뒤·좌·우 경계를 각각 `0.10 m` 축소한 **임시 재측정 후보**를 보존한다.
실차 외곽 재측정 완료 기록이 아니며, 실제 로봇 FIELD-PASS로 사용하면 안
된다.

## Old and New Values

| Side | Previous body | Reduction | Provisional body | Planning margin | New planning extent |
|---|---:|---:|---:|---:|---:|
| Front | `0.75837 m` | `0.10 m` | `0.65837 m` | `0.05 m` | `0.70837 m` |
| Rear | `0.73323 m` | `0.10 m` | `0.63323 m` | `0.05 m` | `0.68323 m` |
| Left | `0.53505 m` | `0.10 m` | `0.43505 m` | `0.05 m` | `0.48505 m` |
| Right | `0.53495 m` | `0.10 m` | `0.43495 m` | `0.05 m` | `0.48495 m` |

| Recorded envelope | Previous | Provisional | Difference |
|---|---:|---:|---:|
| Physical/body boundary | `1.49160 x 1.07000 m` | `1.29160 x 0.87000 m` | length `-0.20 m`, width `-0.20 m` |
| Planning boundary | `1.59160 x 1.17000 m` | `1.39160 x 0.97000 m` | length `-0.20 m`, width `-0.20 m` |

`robot_center_link`, wheelbase, sensor TF, height, and the four-sided `0.05 m`
planning margin did not change. `track_width=1.07 m` remains an independent
provisional kinematic value until wheel-center spacing is measured.

## Same-Bag Replay

The original rectangular replay image is no longer published because it does
not represent the active rounded boundary. Its recorded counts remain below
and in `result.json`; the current contour is shown in the policy figure in the
next section.

The old and new rectangles were evaluated against the same lanelet grid and
the same 1,582 localization samples from the 2026-08-06 body-only diagnostic.
Edge samples use `0.125 m` spacing and lanelet cost `100` as contact.

| Boundary | Cost-100 pose samples | First contact | Last contact |
|---|---:|---|---|
| Previous physical body | `262 / 1582` | `Y=33.6676 m` | `Y=29.6184 m` |
| Previous planning boundary | `1574 / 1582` | first sample | last sample |
| Provisional physical body | `0 / 1582` | none | none |
| Provisional planning boundary | `0 / 1582` | none | none |

This demonstrates that the smaller calculated rectangle clears the recorded
grid trajectory. That replay alone does **not** prove that the real robot
clears the route because the same old pose trace is being reclassified.

## Fresh Simulation Validation

<!-- HH_260806 - Separate the body hard-stop from the recoverable planning
margin and bind the result to fresh full-bringup simulation records. -->

![Fresh body and planning-margin validation](02-runtime-boundary-policy.png)

The first physical-contact run reproduced a policy defect: the gate sampled
only the planning polygon, so it issued a `0.05 m/s` recovery command and moved
`0.0652 m` while the physical body was already on cost 100. The fix adds an
independent physical-body sample before the planning-margin check. A physical
contact now reports `lanelet_physical_body_cost` and rejects every automatic
recovery candidate.

| Fresh map-v15 scenario | Measured result | Verdict |
|---|---|---|
| Normal route | `10.0403 m`, goal error `0.2932 m`, no route hold, final zero | SIM PASS |
| Margin contact, ordinary command | body max `70`, planning max `100`, challenged `0.10 m/s`, final `0.0 m/s` | SIM PASS |
| Margin contact, recovery owner | `CRAB_RIGHT`, `0.133 m`, max `0.05 m/s`; both polygons clear at release | SIM PASS |
| Physical-body contact | body/planning max `100`; owner motion false; recovery output `0.0 m/s`; `0.0023 m` pose variation | SIM PASS |

The lateral scan found margin-only windows at `-0.36..-0.32 m` and
`+0.17..+0.21 m`. The tests selected `+0.19 m` for margin-only contact and
`+0.27 m` for physical contact. These are controlled simulation placements,
not recommended field offsets.

Raw scenario records are kept beside this file:

- [`runtime-physical-body-hard-stop-before-fix.json`](runtime-physical-body-hard-stop-before-fix.json)
- [`runtime-route-clear.json`](runtime-route-clear.json)
- [`runtime-margin-contact-stop.json`](runtime-margin-contact-stop.json)
- [`runtime-margin-recovery.json`](runtime-margin-recovery.json)
- [`runtime-physical-body-hard-stop.json`](runtime-physical-body-hard-stop.json)

## Build And Regression Test

The final source/configuration state was installed through the repository
wrapper, not a standalone ad-hoc build:

```bash
./src/colcon_build.sh --packages-select camrod_control camrod_bringup \
  camrod_planning camrod_sensor_kit camrod_platform \
  --event-handlers console_direct+
```

The five packages built successfully. Fresh `colcon test` runs passed all
`32/32` registered CTest targets across control (`2`), planning (`10`),
bringup (`18`), and platform (`2`). Sensor-kit has no registered test target.
The aggregate xUnit reader reported `334` records, zero errors/failures, and
eight skipped cppcheck records because the installed 2.7 checker is disabled
by the ROS ament wrapper for its known performance issue.

## Configured Hardware Protrusion Check

The source-derived sensor side view exposes a safety conflict with the reduced
rectangle:

| Reference point | Configured X/Y | Relative to provisional boundary |
|---|---:|---|
| Front LiDAR | `X=+0.76336 m` | `+0.10499 m` beyond body front; `+0.05499 m` beyond planning front |
| Front camera | `X=+0.76337 m` | `+0.10500 m` beyond body front; `+0.05500 m` beyond planning front |
| IMU | `X=+0.68800 m` | `+0.02963 m` beyond body front; inside planning front by `0.02037 m` |
| Rear camera | `X=-0.61933 m` | only `0.01390 m` inside body rear before housing size |
| Side radar reference | `Y=+/-0.41005 m` | only about `0.025 m` inside body side before housing size |

These are mount reference points, not complete housing bounds, so the real
protrusion may be larger. The configured `track_width=1.07 m` is also wider
than the new `0.97 m` planning rectangle if that value represents actual wheel
center spacing. This conflict is why the candidate must not be treated as a
real-robot collision envelope before measurement.

## Synchronized Runtime Contract

The provisional values are synchronized across:

- `camrod_sensor_kit/config/robot_params.yaml` and its bringup mirror;
- Nav2 local/global costmap footprints and their bringup mirror;
- safety-gate source defaults, YAML fallback, and bringup launch defaults;
- RobotParams source defaults and geometry contract tests.

The live `/platform/robot/planning_boundary` remains authoritative after
startup. Its publisher derives the polygon from the provisional body plus the
unchanged `0.05 m` margin. The safety gate also carries the synchronized body
extents as an independent hard-stop envelope; it does not infer the body by
subtracting a margin at runtime.

## Required Physical Validation

Before real-robot acceptance, measure the maximum outer envelope from
`robot_center_link`, including:

- wheel/tire outer edges and steering sweep;
- chassis corners, bumpers, and covers;
- LiDAR/camera/radar housings, brackets, cables, and antenna hardware;
- the normal payload or cargo envelope.

If any hardware lies outside the provisional rectangle, the boundary must be
expanded to contain that hardware before driving. The controlled route,
one-sided margin contact/recovery, and physical-body hard stop now pass on
AMD64 simulation. Their physical-robot repetitions plus crab and zero-turn
swept-clearance checks remain P0 work in `TODOLIST.txt`.

Machine-readable values and source bag hashes are in [`result.json`](result.json).
