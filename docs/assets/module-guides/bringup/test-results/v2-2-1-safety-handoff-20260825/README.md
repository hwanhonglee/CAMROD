# v2.2.1 Campsite, Charging, And Radar Safety Result

<!-- HH_260825 - Keep measured AMD64 simulation evidence separate from field acceptance. -->

![v2.2.1 measured simulation summary](v2-2-1-safety-handoff-summary.png)

This record covers three changes exercised on the full AMD64 ROS graph:

| Check | Result | Measured observation |
|---|---|---|
| B8 campsite exit | PASS | Live lanelet projection was `0.140 m` away while the historical entry anchor remained `0.231 m` away; after a `1.20 s` stopped hold, `done_live_lanelet` started a current-pose `reverse_shortest` route |
| Charging campsite recall | PASS | Destination B2 held all three authorization gates closed for `6.996 s`, then emitted one parking cancel, one drop-zone `EXIT`, and `DEPARTING_CHARGER`; the site goal remained deferred until station exit completion |
| FRONT1 near-field radar | PASS | A fresh `0.300 m` `AvgRange` stream produced active obstacle evidence with cost `95`; route-outside handling retained the cost fail-open |

The first automated charging-recall run reached `CHARGING` and internal
`PARKED`, but its old validator waited for a presentation-specific
`state=CHARGING` command-gate string. A direct production UI request then
completed the 7-second dwell and station exit. The validator was corrected to
use public `CHARGING` and `DEPARTING_CHARGER` service states.

The raw values and claim boundary are in [`result.json`](result.json). This is
logic and integration evidence only. Physical radar multipath, real stopping
distance, charger clearance, and ARM64 resource use remain field work.
