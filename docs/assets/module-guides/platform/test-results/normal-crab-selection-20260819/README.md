# Normal and crab motion selection

## Scope

- Input: canonical `camrod_platform/config/ranger_driver.yaml`.
- Normal Nav2 commands with `linear.y` inside the configured `0.02 m/s`
  deadband remain Dual-Ackermann.
- Explicit campsite and route-recovery lateral commands remain parallel-wheel
  crab motion.

## Files

- `normal-vs-crab-mode-selection.png`: static contract summary.
- `normal-vs-crab-mode-selection.gif`: normal route followed by explicit crab.
- `SHA256SUMS`: artifact integrity hashes.

The paired full-graph run moved `3.73 m` on a normal Nav2 route with measured
maximum final-command `|linear.y| = 0.000 m/s`, then completed B8 explicit crab
entry/return. Its machine-readable report is stored under the bringup test
result for the same date. Jetson and physical steering acceptance remain field
work.
