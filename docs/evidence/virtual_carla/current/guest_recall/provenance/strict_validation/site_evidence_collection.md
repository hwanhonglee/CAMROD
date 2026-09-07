# CAMROD CARLA site-evidence collection validation

- Result: `PASS`
- Sites: 13 (B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13)
- Authority / mission: `guest` / `recall`
- Ranger identity: `vehicle.ranger.default` / `ego_vehicle`
- Physical backend: `PHYSX_FOUR_WHEEL_STEERING`
- Total elapsed: 7803.845 s
- Total odometry distance: 1722.789852 m
- Collision events: 0; every final speed <= 0.05 m/s; every wheel grounded

| Site | Result | Elapsed s | Out s | Return s | Total m | Out m | Return m | Final m/s | Actor | Collision pub/events | Wheels/samples | Parked | Charging |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| B1 | PASS | 753.340 | 256.474 | 496.866 | 167.556 | 83.566 | 83.990 | 0.000000 | 66 | 1/0 | 4/8000 | true | true |
| B2 | PASS | 728.370 | 243.707 | 484.663 | 168.202 | 83.834 | 84.367 | 0.000000 | 66 | 1/0 | 4/7804 | true | true |
| B3 | PASS | 701.556 | 230.306 | 471.250 | 155.066 | 77.388 | 77.678 | 0.000000 | 66 | 1/0 | 4/7505 | true | true |
| B4 | PASS | 681.406 | 225.470 | 455.936 | 152.920 | 76.124 | 76.796 | 0.000000 | 66 | 1/0 | 4/7301 | true | true |
| B5 | PASS | 634.300 | 210.982 | 423.318 | 143.355 | 71.426 | 71.929 | 0.000000 | 66 | 1/0 | 4/6837 | true | true |
| B6 | PASS | 640.017 | 206.105 | 433.912 | 140.204 | 69.767 | 70.437 | 0.000000 | 66 | 1/0 | 4/6899 | true | true |
| B7 | PASS | 579.192 | 196.802 | 382.390 | 131.381 | 65.435 | 65.947 | 0.000000 | 66 | 1/0 | 4/6274 | true | true |
| B8 | PASS | 568.099 | 190.614 | 377.485 | 126.715 | 63.130 | 63.584 | 0.000000 | 66 | 1/0 | 4/6219 | true | true |
| B9 | PASS | 533.085 | 178.998 | 354.087 | 118.417 | 58.938 | 59.479 | 0.000000 | 66 | 1/0 | 4/5851 | true | true |
| B10 | PASS | 517.525 | 174.369 | 343.156 | 114.257 | 56.841 | 57.416 | 0.000000 | 66 | 1/0 | 4/5670 | true | true |
| B11 | PASS | 507.260 | 166.808 | 340.452 | 108.103 | 53.844 | 54.259 | 0.000000 | 66 | 1/0 | 4/5573 | true | true |
| B12 | PASS | 477.254 | 160.111 | 317.143 | 102.553 | 51.085 | 51.468 | 0.000000 | 66 | 1/0 | 4/5280 | true | true |
| B13 | PASS | 482.441 | 150.306 | 332.135 | 94.061 | 46.812 | 47.249 | 0.000000 | 66 | 1/0 | 4/5309 | true | true |

Every row is cross-checked against its site manifest, native matrix report, physical-wheel stream summary, visual capture manifest, and SHA-256 bindings.
