# Campsite Full Return Validation - 2026-08-19

<!-- HH_260819 - Record the current-map B1-B13 round trips and the constrained
roadside forward-return decision separately from historical map-v16 evidence. -->

![Campsite policy validation](campsite-policy-validation.png)

![Campsite phase sequence](campsite-phase-sequence.gif)

## Result

| Sites | Runtime sequence | Return route | Result |
|---|---|---|---|
| B1-B10 | `CRAB_IN -> ROTATE_180 -> UNLOAD_WAIT -> WAIT_RETURN -> ALIGN_RETRACE_YAW -> CRAB_OUT -> DONE` | Explicit Return may retrace the reverse shortest route after the vehicle turns on site. | PASS 10/10 |
| B11-B13 | `CRAB_IN -> UNLOAD_WAIT -> WAIT_RETURN -> CRAB_OUT -> DONE` | No zero-turn in the narrow lane; `done_roadside_forward` forces the legal forward one-way loop. | PASS 3/3 |
| B11 full service | UI Return, `155.73 m` forward loop, drop-zone yaw alignment, reverse parking, charger feedback | `WAITING_FOR_CHARGING -> CHARGING`, parking controller `PARKED` | PASS |

B11-B13 use a `0.30 m` lateral cap and a final gated command maximum of
`0.10 m/s`. The raw campsite controller crab command remained pure lateral at
`0.20 m/s`; no mixed longitudinal/lateral sample was observed. Both operator
Return controls use the same `/ui/manual_return` contract.

## Identity

| Input | SHA-256 |
|---|---|
| User-maintained `lanelet2_maps.osm` | `8fa13157b8e956559ad29b1bf49b4357ec6d252b0259debfb40a946b29f24e59` |
| Active campsite policy YAML | `2380ce4f336b9b3257551c79380298caa4159df607a6cc1c4285fc0a6e04fc48` |

The package and bringup campsite YAML files were byte-identical. The map was
read for this test and was not modified.

## Safety Scope

- Nav2 is cancelled and the drive gate is held closed for `0.50 s` before an ordinary-travel manual Return publishes a new route.
- Repeated Return presses are coalesced; an active return is not cancelled and restarted.
- B11-B13 complete `CRAB_OUT` before route planning and never rotate in the narrow lane.
- Static road-lanelet checks are bypassed only in configured campsite phases because the service area lies outside the road polygon. Live obstacle stops remain active; ordinary physical-body and planning-footprint lanelet checks resume at `DONE` before route travel.
- The result is AMD64 ROS 2 simulation evidence, not ARM64 Jetson timing or physical-road acceptance.

## Files

- `camping_site_1.json` through `camping_site_13.json`: focused controller and UI Return checks.
- `camping_site_11_full_cycle.json`: complete campsite-to-charger service cycle.
- `campsite-policy-summary.json`: machine-readable aggregate consumed by the contract test.
- `campsite-policy-validation.png` and `campsite-phase-sequence.gif`: generated visual summaries.

## Reproduction

Run the simulation on an isolated ROS domain and invoke the validation runner
with `run_camping:=true`, `camping_return_via_ui:=true`, and the desired
`camping_site_key`. Regenerate the visual summary with:

```bash
python3 camrod_bringup/scripts/visualization/render_camping_site_sequence_results.py \
  --result-dir docs/assets/module-guides/bringup/test-results/camping-site-full-return-20260819 \
  --camping-sites camrod_planning/config/camping_sites.yaml
```
