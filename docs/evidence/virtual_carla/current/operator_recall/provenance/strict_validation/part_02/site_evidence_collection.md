# CAMROD CARLA site-evidence collection validation

- Result: `PASS`
- Sites: 8 (B6, B7, B8, B9, B10, B11, B12, B13)
- Authority / mission: `operator-browser` / `recall`
- Ranger identity: `vehicle.ranger.default` / `ego_vehicle`
- Physical backend: `PHYSX_FOUR_WHEEL_STEERING`
- Total elapsed: 4225.487 s
- Total odometry distance: 935.827745 m
- Collision events: 0; every final speed <= 0.05 m/s; every wheel grounded

| Site | Result | Elapsed s | Out s | Return s | Total m | Out m | Return m | Final m/s | Actor | Collision pub/events | Wheels/samples | Parked | Charging |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| B6 | PASS | 610.957 | 206.844 | 404.113 | 140.144 | 69.753 | 70.391 | 0.000000 | 50 | 1/0 | 4/6601 | true | true |
| B7 | PASS | 569.534 | 196.841 | 372.693 | 131.417 | 65.422 | 65.995 | 0.000000 | 50 | 1/0 | 4/6175 | true | true |
| B8 | PASS | 552.539 | 190.549 | 361.991 | 126.901 | 63.102 | 63.799 | 0.000000 | 50 | 1/0 | 4/5989 | true | true |
| B9 | PASS | 537.618 | 180.040 | 357.578 | 118.550 | 58.993 | 59.558 | 0.000000 | 50 | 1/0 | 4/5852 | true | true |
| B10 | PASS | 504.918 | 174.143 | 330.775 | 113.981 | 56.783 | 57.198 | 0.000000 | 50 | 1/0 | 4/5514 | true | true |
| B11 | PASS | 495.861 | 167.507 | 328.355 | 108.144 | 53.917 | 54.228 | 0.000000 | 50 | 1/0 | 4/5467 | true | true |
| B12 | PASS | 489.989 | 161.240 | 328.749 | 102.752 | 51.271 | 51.481 | 0.000000 | 50 | 1/0 | 4/5379 | true | true |
| B13 | PASS | 464.071 | 150.540 | 313.531 | 93.938 | 46.825 | 47.114 | 0.000000 | 50 | 1/0 | 4/5116 | true | true |

Every row is cross-checked against its site manifest, native matrix report, physical-wheel stream summary, visual capture manifest, and SHA-256 bindings.
