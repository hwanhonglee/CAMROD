# Operator telemetry AMD64 profile (2026-08-10)

This result measures only the `ui_backend` process while the complete simulated
CAMROD graph is running. The browser renewed one telemetry-tab lease at 2 Hz;
six process samples were collected over about three seconds for each view.

| View | Dynamic ROS subscriptions | CPU mean [% of one logical CPU] | PSS mean [MiB] | API payload [KiB] |
|---|---:|---:|---:|---:|
| Idle | 0 | 7.581 | 79.321 | 2.533 |
| GNSS / IMU | 6 | 9.558 | 79.341 | 3.191 |
| Radar / LiDAR | 11 | 11.855 | 79.201 | 5.616 |
| Camera, no sim publishers | 4 | 7.911 | 78.769 | 2.519 |
| Trajectory | 7 | 10.875 | 79.026 | 7.733 |
| Map / perception | 9 | 14.446 | 80.001 | 36.141 |
| Safety / control | 7 | 10.156 | 81.269 | 3.354 |
| Idle after all tabs | 0 | 7.581 | 81.294 | 2.517 |

The previous implementation held 32 dynamic subscriptions regardless of the
selected view. On-demand view leases now reduce that graph fan-in by 65.6% to
87.5%, depending on the selected tab. These are subscription-count reductions,
not claimed CPU reductions. Payload pruning also prevents the safety tab from
retransmitting the map/perception arrays: its current payload is 90.7% smaller
than the current perception payload.

The end PSS is about 2 MiB above the first idle sample. A short AMD64 sample
cannot distinguish Python allocator retention from a leak, so a 30-minute ARM64
soak remains required. This run also had no camera publishers, and therefore
does not qualify JPEG encoding cost or live camera frame pacing.

The 1280-pixel-wide browser check exercised all six telemetry tabs. It found no
document, modal, workspace, or text overflow and no HTTP/console errors. The
expected absence of the Ranger tuning service in simulation is represented as
`HTTP 200` with `available:false`; write attempts still fail explicitly.

This evidence is an AMD64 development comparison only. It is not an acceptance
result for the target ARM64 computer with 8 CPU cores and 16 GiB RAM.
