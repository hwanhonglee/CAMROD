# Manual Return preemption

<!-- HH_260819 - Preserve the measured outbound-stop ordering and its limits. -->

![Measured outbound Return timing](manual-return-preemption.png)

## Result

The isolated AMD64 full graph dispatched B6 and waited until the simulated
platform had moved `2.019 m`. Two immediate Return requests were coalesced by
the shared UI authority. The command output became zero after `5.01 ms`, stayed
zero throughout the measured `0.10-0.45 s` barrier, and emitted one planning
recall after `0.508 s`. The new reverse-return path contained 12 poses over
`2.133 m`.

| Assertion | Measured | Result |
|---|---:|---|
| First two requests enter preemption | `return_preempting` | PASS |
| First zero command | `5.01 ms` | PASS, limit `200 ms` |
| Hold-window maximum command | `0.000` | PASS |
| Fresh planning recalls | `1` | PASS |
| Recall delay | `0.508 s` | PASS, window `0.45-0.75 s` |
| Service state | `RETURNING_TO_DROP_ZONE` | PASS |
| Fresh reverse path | `12 poses / 2.133 m` | PASS |

The frontend contract test separately verifies that both visible Return
buttons call this same `POST /ui/manual_return` endpoint. This run is AMD64
simulation evidence only. Physical stopping distance, scheduler latency, and
resource acceptance on the 8-core/16-GiB ARM64 target remain field tasks.

## Reproduce

Start the full simulation graph with the API UI enabled, then run:

```bash
ros2 run camrod_bringup manual_return_preemption_probe.py \
  --base-url http://127.0.0.1:18101 \
  --site B6 \
  --output /tmp/manual-return-preemption.json
```

Regenerate the timeline together with the resource profile:

```bash
python3 camrod_bringup/scripts/visualization/render_return_resource_profile.py \
  --input docs/assets/module-guides/ui/test-results/return-resource-amd64-20260819/return-resource-profile.json \
  --output docs/assets/module-guides/ui/test-results/return-resource-amd64-20260819/return-resource-profile.png \
  --preemption-input docs/assets/module-guides/ui/test-results/manual-return-preemption-amd64-20260819/manual-return-preemption.json \
  --preemption-output docs/assets/module-guides/ui/test-results/manual-return-preemption-amd64-20260819/manual-return-preemption.png
```
