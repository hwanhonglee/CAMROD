# v2.3.0 blocker: simulated distance clock mismatch

Audit: 2026-10-02, read-only. **Simulation distance accuracy is NOT verified.**
Withhold the v2.3.0 tag and push approval until the measurement-clock issue is
resolved and tested. Successful record creation, phase transitions or screenshots
do not establish accurate metres. No source, database or historical record was
modified, deleted or rescaled by this audit.

## Measured discrepancy

Mission: `2026-10-02 #006 B9 delivery`, ID
`244f076bc40e4702ac845c1471dccd95:mission:1790922474608001`.
Recorder storage: `/home/hong/camrod_ws/state/virtual_carla/mission_records/`.
Mission directory: `2026-10-02/006_B9_delivery_47776db6/`.

Passive evidence: `docs/evidence/virtual_carla/v230_b9_20261002/retry/`
`passive_objects_20261002T062837Z.jsonl`, with the adjacent `_summary.json`.
Collector duration was 210.027 wall seconds, approximately **15:28:37.843 to
15:32:07.699 KST** (06:28:37.843 to 06:32:07.699 UTC).

| Independently recomputed quantity | Result |
| --- | ---: |
| Platform samples | 1,050 |
| Platform header interval | 1790922517.830536 to 1790922727.6908855 seconds |
| Platform header elapsed time | 209.860350 seconds |
| Trapezoidal planar speed integral, 0.03 m/s minimum | **68.254967 m** |
| Matching mission recorder telemetry samples | 1,013 |
| Recorder telemetry interval | 1790922525.2921188 to 1790922727.6908855 seconds |
| Same integral from recorder telemetry | **68.254967 m** |
| UI localization poses / sampled path length | 416 / **53.973005 m** |
| Largest adjacent UI pose displacement | 0.344865 m |

The earlier passive samples precede mission creation and are stationary, hence
both speed integrals agree. The UI pose window spans elapsed 0.121–209.870 s.
Speed integration exceeds the sampled UI pose path by **14.281962 m (26.46%)**.
UI localization is not independent CARLA ground truth: this comparison exposes
a discrepancy, not a certified true-distance correction factor. Variable tick
rate and localization/filter effects prohibit rescaling history by this ratio.

At 15:36:50 KST `/api/mission-records?limit=3` reported recorder
`environment=simulation`, `use_sim_time=false`, status `READY`, and current
mission `phase=return`, `total_m=108.646891`. That larger number covers a later
window and must not be compared directly with the first-210-second quantities.

## Exact cause and affected consumers

Line references are from the audited source checkout:

1. [Feedback configuration](../camrod_carla_adapter/config/feedback_bridge.yaml)
   line 42 enables `stamp_with_reception_time`. [Adapter launch](../camrod_carla_adapter/launch/adapter.launch.py)
   line 193 sets this feedback node's `use_sim_time` to `False`.
2. [Feedback bridge](../camrod_carla_adapter/src/camrod_carla_adapter/feedback_bridge_node.py)
   lines 307–310 replace the source CARLA timestamp with the node clock. Lines
   345–358 copy the odometry, replace its header and preserve its velocity values.
   Therefore the speed remains metres per **simulation second**, but its output
   sample timestamps advance in **wall seconds**.
3. [Platform bridge](../camrod_platform/src/ranger_platform_bridge_node.cpp)
   lines 507–513 preserve that odometry header in aggregate platform velocity.
4. [Recorder normalization](../camrod_ui/runtime/python/camrod_ui/mission_recorder_node.py)
   lines 67–81 selects `velocity.header.stamp` before any receipt-time fallback.
   [Journal integration](../camrod_ui/runtime/python/camrod_ui/mission_journal.py)
   lines 481–522 uses those sample deltas and trapezoidal planar speed. It does
   **not** unconditionally integrate receipt wall time. Archived telemetry
   confirms `timestamp_basis=velocity.header.stamp` with epoch wall timestamps.
5. Existing service statistics are affected by the same simulator input:
   [UI backend](../camrod_ui/runtime/python/camrod_ui/ui_backend_node.py)
   lines 5674–5685 passes the same velocity timestamp to
   [ServiceMetricsTracker](../camrod_ui/runtime/python/camrod_ui/service_metrics.py)
   lines 463–535. Its distance is also speed × source-header elapsed time; it is
   not accumulated from robot XY positions. Mission dates/durations use wall
   time separately, which is not itself a distance error.

When simulation advances slower than wall time, unchanged simulation velocity
multiplied by wall-time deltas inflates distance. This is a CARLA boundary
clock-domain mismatch, not evidence that the physical-robot recorder integration
formula or physical-platform defaults should change. Do not change normal
platform/localization timestamps, scale robot speeds, or relax freshness checks
to conceal it.

## Next bounded fix proposal — not implemented

Add a CARLA-only, read-only measurement stream containing the original CARLA
odometry timestamp and velocity, with source-labelled, fresh platform metadata.
Do not replace `/platform/status` or any control/localization input.

The recorder already accepts `platform_status_topic`. However
`camrod_ui_robot/launch/ui.launch.py:124–142` shares the ordinary UI
`platform_status_topic`; there is no independent recorder input/clock forwarding
through bringup. A virtual-only wrapper could disable that embedded recorder and
launch exactly one existing `mission_recorder_node` with the same isolated
storage, `environment=simulation`, `use_sim_time=true` and a dedicated measurement
topic. This requires explicit lifecycle/duplicate-writer tests.

**That recorder-only change would not fix existing service statistics.** Their
velocity accumulator also needs a dedicated measurement-only input or timestamp
contract, without replacing the UI's safety/platform status subscription. Design
and review that small interface separately; do not declare the release fixed
after changing only the journal. Preserve date/event wall timestamps and record
the distance clock basis explicitly.

Required validation:

- Identical known simulated travel at 1×, 0.5× and varying real-time factors
  produces the same distance in both journal and existing service statistics.
- Compare against original CARLA odometry/actor pose, not only UI localization;
  state numerical tolerances and preserve raw source timestamps in evidence.
- Paused world, duplicate stamps, rewinds/reset, stale metadata, missing source
  samples and node restart add no invented distance and expose incomplete data.
- Exactly one recorder owns the existing store; recorded events, mission/leg
  classification, manual/auto separation and UI graph/history remain intact.
- Normal platform/localization/control topics and real-platform defaults remain
  unchanged; existing simulator records are retained with a known-issue label.

No global correction factor or retrospective overwrite of real or simulated
records is authorized. Keep the original measurements available for audit.
