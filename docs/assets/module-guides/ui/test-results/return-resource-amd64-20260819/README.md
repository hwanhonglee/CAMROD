# Return and telemetry resource A/B

<!-- HH_260819 - Record measured AMD64 impact without promoting it to Jetson acceptance. -->

![AMD64 Return and telemetry resource profile](return-resource-profile.png)

## Result

| Metric | Before | After | Change |
|---|---:|---:|---:|
| Whole graph CPU, one-core basis | 81.88% | 80.78% | -1.3% |
| UI backend CPU, one-core basis | 6.93% | 6.53% | -5.8% |
| Whole graph summed RSS | 1955.6 MiB | 1938.7 MiB | -16.9 MiB |
| Whole graph CPU, 12-core capacity | 6.823% | 6.732% | -0.091 percentage points |

The A/B used the same 45-process full-simulation topology for 30 seconds at
`CHARGING` idle. RViz, the guest UI, and the browser window were disabled; the
API UI backend remained enabled. Linux per-process CPU is summed on the basis
that 100% is one fully occupied logical core. Summed RSS may count shared pages
more than once.

The change removes permanent 10 Hz telemetry-lease polling. An HTTP/WebSocket
request now triggers a ROS `GuardCondition` immediately, while a 1 Hz timer is
retained only to expire abandoned leases. Visible telemetry stays at 10 Hz and
sensor subscriptions remain lazy by selected tab.

These measurements are AMD64 comparison evidence, not ARM64 acceptance. The
8-core, 16-GiB Jetson still requires the 30-minute camera/tab-cycle and frame
pacing test listed in `TODOLIST.txt`.

## Regenerate

```bash
python3 camrod_bringup/scripts/visualization/render_return_resource_profile.py \
  --input docs/assets/module-guides/ui/test-results/return-resource-amd64-20260819/return-resource-profile.json \
  --output docs/assets/module-guides/ui/test-results/return-resource-amd64-20260819/return-resource-profile.png
```
