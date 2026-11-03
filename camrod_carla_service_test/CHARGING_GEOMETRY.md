# Test-only charging geometry — HH_261103

The native normal/cargo station remains ROS `(-8.47366, 40.391)`. The charging
body anchor is 0.60 m behind it along body yaw `+91.7873°`, at
`(-8.454946473479461, 39.79129190106755)`.

Only the existing CARLA contact emulator loads `config/charging_station.yaml`.
Its unchanged 0.35 m radius, stopped-speed <=0.05 m/s, fresh measured pose and
odometry, fresh selected-AprilTag terminal state and one-second dwell still
apply. This is simulated contact evidence, not a claim of hardware charger/CAN
or mechanical disconnect proof. No contact is manufactured at the normal
station, which is 0.60 m from this separate charger anchor.

The 0.60 m choice accounts for native cargo's 0.20 m stop tolerance: even a
nominal early stop is >=0.40 m from the charger anchor, beyond its inclusive
0.35 m radius. A 0.55 m offset would leave zero such margin. Native cargo's
minimum 0.30 m initial displacement and strictly-forward check remain exact.

`test/check_charging_geometry.cpp` executes the existing native cargo policy
and canonical tapered/rounded planning-boundary generator with calibrated
0.10 m margins. Its 30 float32 points give radius `0.94656048008187943 m`,
exactly matching the captured live `/platform/robot/planning_boundary`.
Endpoint disks prove the whole straight sweep inside the original convex
drop zone. Minimum remaining disk clearance is 0.3699225 m at the charging
anchor and 0.4131194 m at the normal station. Runtime native geometry checks
remain required; this nominal proof does not authorize an arbitrary pose.

The historical scene operation reused the exact-46 translation-only clone helper:

```
source: Woraksan_v27_service_tag_20261102 (frozen)
target: Woraksan_v27_charge_anchor_20261103 (new namespace)
```

Only the 46 tag locations translate 0.60 m backward with the charging body;
facing, absolute Z, 10° tilt, materials, ID3/0.16 m black border, all non-tag
actor/component contracts and terrain stay unchanged. Original V224, the Nov2
scene and native station/robot files are hash protected. PLAN and APPLY
receipts are under the pipeline's
`.work/evidence/service_usability_tag_alignment_20261103/`; schema is
`camrod.service_charging_geometry.stage.v1`. The archived stage source under
`historical/` is disabled as an executable entrypoint. New hosts must provision
assets and generate a local invariant-checked receipt with their official
pipeline tools. Active overlay tools and charging YAML have no home paths.

Actual rear images, successful AprilTag docking, contact and forward cargo
release must still be demonstrated on this new scene. No docking claim is
inferred merely from staging or unit-test success.
