# CAMROD CARLA site-evidence collection validation

- Result: `PASS`
- Sites: 13 (B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13)
- Authority / mission: `operator-browser` / `delivery`
- Ranger identity: `vehicle.ranger.default` / `ego_vehicle`
- Physical backend: `PHYSX_FOUR_WHEEL_STEERING`
- Total elapsed: 7840.705 s
- Total odometry distance: 1786.452243 m
- Collision events: 0; every final speed <= 0.05 m/s; every wheel grounded

| Site | Result | Elapsed s | Out s | Return s | Total m | Out m | Return m | Final m/s | Actor | Collision pub/events | Wheels/samples | Parked | Charging |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| B1 | PASS | 706.654 | 238.005 | 468.648 | 171.066 | 83.110 | 87.956 | 0.000000 | 50 | 1/0 | 4/7549 | true | true |
| B2 | PASS | 751.600 | 266.096 | 485.503 | 174.216 | 86.968 | 87.248 | 0.000000 | 50 | 1/0 | 4/7989 | true | true |
| B3 | PASS | 701.878 | 254.392 | 447.485 | 162.942 | 81.598 | 81.345 | 0.000000 | 50 | 1/0 | 4/7503 | true | true |
| B4 | PASS | 683.079 | 244.311 | 438.768 | 159.194 | 79.420 | 79.774 | 0.000000 | 50 | 1/0 | 4/7309 | true | true |
| B5 | PASS | 649.466 | 234.608 | 414.858 | 150.607 | 75.116 | 75.491 | 0.000000 | 50 | 1/0 | 4/6960 | true | true |
| B6 | PASS | 645.552 | 233.406 | 412.147 | 145.947 | 72.794 | 73.153 | 0.000000 | 50 | 1/0 | 4/6909 | true | true |
| B7 | PASS | 598.849 | 222.215 | 376.633 | 138.814 | 69.375 | 69.439 | 0.000000 | 50 | 1/0 | 4/6443 | true | true |
| B8 | PASS | 576.626 | 213.710 | 362.916 | 132.731 | 66.215 | 66.516 | 0.000000 | 50 | 1/0 | 4/6250 | true | true |
| B9 | PASS | 555.299 | 206.290 | 349.009 | 126.424 | 63.147 | 63.277 | 0.000000 | 50 | 1/0 | 4/6029 | true | true |
| B10 | PASS | 540.787 | 200.490 | 340.297 | 120.013 | 60.140 | 59.872 | 0.000000 | 50 | 1/0 | 4/5867 | true | true |
| B11 | PASS | 490.672 | 165.785 | 324.887 | 107.998 | 53.744 | 54.253 | 0.000000 | 50 | 1/0 | 4/5367 | true | true |
| B12 | PASS | 484.272 | 160.099 | 324.173 | 102.476 | 51.104 | 51.372 | 0.000000 | 50 | 1/0 | 4/5317 | true | true |
| B13 | PASS | 455.971 | 150.206 | 305.765 | 94.024 | 46.842 | 47.182 | 0.000000 | 50 | 1/0 | 4/5019 | true | true |

Every row is cross-checked against its site manifest, native matrix report, physical-wheel stream summary, visual capture manifest, and SHA-256 bindings.
