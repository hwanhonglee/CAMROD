# CAMROD CARLA site-evidence collection validation

- Result: `PASS`
- Sites: 5 (B1, B2, B3, B4, B5)
- Authority / mission: `operator-browser` / `recall`
- Ranger identity: `vehicle.ranger.default` / `ego_vehicle`
- Physical backend: `PHYSX_FOUR_WHEEL_STEERING`
- Total elapsed: 3444.339 s
- Total odometry distance: 787.229579 m
- Collision events: 0; every final speed <= 0.05 m/s; every wheel grounded

| Site | Result | Elapsed s | Out s | Return s | Total m | Out m | Return m | Final m/s | Actor | Collision pub/events | Wheels/samples | Parked | Charging |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| B1 | PASS | 731.434 | 242.503 | 488.931 | 167.871 | 83.570 | 84.300 | 0.000000 | 50 | 1/0 | 4/7820 | true | true |
| B2 | PASS | 724.621 | 244.201 | 480.420 | 168.199 | 83.849 | 84.351 | 0.000000 | 50 | 1/0 | 4/7727 | true | true |
| B3 | PASS | 684.006 | 226.199 | 457.807 | 155.234 | 77.336 | 77.898 | 0.000000 | 50 | 1/0 | 4/7321 | true | true |
| B4 | PASS | 678.599 | 222.114 | 456.485 | 152.679 | 76.131 | 76.548 | 0.000000 | 50 | 1/0 | 4/7273 | true | true |
| B5 | PASS | 625.679 | 211.205 | 414.474 | 143.247 | 71.399 | 71.847 | 0.000000 | 50 | 1/0 | 4/6746 | true | true |

Every row is cross-checked against its site manifest, native matrix report, physical-wheel stream summary, visual capture manifest, and SHA-256 bindings.
