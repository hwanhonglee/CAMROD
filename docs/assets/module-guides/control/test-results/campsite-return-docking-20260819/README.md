# Campsite return and parking control

## Scope

- B8 campsite sequence executed under amd64 ROS 2 simulation.
- Entry and exit use one lanelet snap anchor.
- The controller published both IN/OUT paths; measured CRAB_IN/OUT commands had
  no simultaneous longitudinal/lateral component.
- The 180-degree turn settles before translation resumes.
- Restart direction is derived from current heading and site geometry.
- Reverse parking slows over the final `0.30 m`; AprilTag docking slows over
  the final `0.60 m`; charging feedback commands an immediate zero.

## Files

- `b8-same-anchor-return.png`: same-anchor geometry and measured sim phases.
- `b8-entry-return-sequence.gif`: controller phase animation.
- `parking-slowdown-profile.png`: speed curves generated from canonical YAML.
- `SHA256SUMS`: artifact integrity hashes.

The B8 report is in the bringup test-result directory. Parking speed profiles
are deterministic source/config evidence; charger contact and tag accuracy
still require the physical robot.
