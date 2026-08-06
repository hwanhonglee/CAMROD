# GNSS Left-Antenna Lever-Arm Validation

<!-- HH_260806 - Preserve the focused AMD64 ROS A/B used to verify the
left-antenna transform without claiming real-robot accuracy. -->

## Scope

This test isolates `fake_sensor_publisher` and
`localization_input_adapter_node` on ROS domain 223. The fake source publishes
the raw left-antenna NavSatFix and dual-GNSS heading. Two adapters consume the
same messages: production correction ON and an A/B probe with correction OFF.
The timestamp-matched reference is simulated center odometry.

## Result

| Metric | Result |
|---|---:|
| Matched samples | `30` |
| Corrected versus uncorrected displacement | `0.450000 m` |
| Corrected center residual, mean/max | `0.000071 / 0.000071 m` |
| Uncorrected center residual, mean | `0.449995 m` |
| GNSS heading / corrected pose rate | `10 / 10 Hz` |
| Projection regression | `2/2 PASS`, ENU round trip `< 1 mm` |
| Process shutdown | clean exit |

The first A/B run exposed an independent fake-sensor projection defect:
correcting the lever arm reduced center error from `0.581708 m` to
`0.134478 m`, but did not remove it. `_xy_to_latlon()` used the WGS84 equatorial
radius for northing. After using the meridional and prime-vertical curvature
radii, the same test produced the final `0.071 mm` residual above.
`test_fake_gnss_projection.py` now locks the corrected inverse projection in
the ordinary bringup CTest suite; the full suite passed `19/19`.

## Boundaries

- This is AMD64 simulation evidence, not GNSS field accuracy.
- Rover X/Z, moving-base XYZ, baseline direction, receiver reference,
  multipath, timing, and moving residuals remain field-pending.
- The Lanelet map and active robot boundary were not modified.

Structured result: [`result.json`](result.json).
