# camrod_control

`camrod_control` owns vehicle-motion maneuvers, final parking controllers, and
the supervised velocity command path. All active runtime nodes are native C++.

<!-- HH_260720 - Document the new package boundary and canonical velocity topics. -->

Runtime responsibilities:

- `cmd_vel_safety_gate`: engage, platform status, timeout, localization recovery, and cost-grid command gating
- `camping_site_maneuver_controller`: campsite crab entry/exit and 180-degree rotation
- `drop_zone_maneuver_controller`: drop-zone departure and yaw alignment before reverse parking
- `reverse_parking_controller`: final yaw-aware reverse motion only
- `apriltag_parking_controller`: final AprilTag-based parking controller

<!-- HH_260720 - Clarify charging departure and method-independent parking handoff. -->

`cmd_vel_safety_gate` normally blocks commands while
the `is_charging` field of `/platform/status` is true. A new `camping_site_*` mission key opens
a bounded `DEPARTING_CHARGER` window so `drop_zone_maneuver_controller` can leave the
station. The gate returns to normal `ENABLED` after charging feedback clears,
or closes again when the departure window expires.

`drop_zone_maneuver_controller` reads semantic drop-zone yaw as the reverse travel axis,
aligns the robot body 180 degrees away from that axis, and starts the selected
parking method only after yaw convergence.

<!-- HH_260720 - Keep launch topology separate from safety policy tuning. -->

`launch/cmd_vel_safety_gate.launch.py` only selects topics, namespace, and the parameter file.
All field-tuned gate policy is grouped in `config/cmd_vel_safety_gate.yaml`; full bringup may
still override those values from `launch_defaults.yaml` without duplicating them in the launch file.
Authorization, charging departure, and directional cost logic are split into
testable policy classes instead of being embedded in the ROS callback surface.

Canonical command path:

```text
/control/cmd_vel_raw -> cmd_vel_safety_gate -> /control/cmd_vel
Nav2 /control/nav2_cmd_vel_ros -----------^             |
                                                       +-> /control/cmd_vel_ros -> Ranger
```

`/control/cmd_vel` is the generated CAMROD command contract. The gate publishes
`/control/cmd_vel_ros` only as the explicit `geometry_msgs/Twist` boundary required by Ranger.

<!-- HH_260720 - Flatten runtime sources and remove the redundant Python package wrapper. -->

```text
camrod_control/
  include/camrod_control/
    charging_departure_policy.hpp
    cmd_vel_gate_policy.hpp
    directional_cost_guard.hpp
    parking_geometry.hpp
    control_support.hpp
  src/
    cmd_vel_safety_gate_node.cpp
    directional_cost_guard.cpp
    camping_site_maneuver_controller_node.cpp
    drop_zone_maneuver_controller_node.cpp
    reverse_parking_controller_node.cpp
    apriltag_parking_controller_node.cpp
  test/
    test_control_policies.cpp
    test_parking_geometry.cpp
```

<!-- HH_260721 - Record the charging-complete parking contract and directional guard behavior. -->

`reverse_parking_controller` enters `WAIT_FOR_CHARGING` after its configured
reverse distance and reports `PARKED` only after `/platform/status.is_charging`
becomes true. `complete_without_charging` is `false` and the default wait timeout
is 20 seconds.

The directional guard checks live LiDAR/radar costs for forward, reverse, crab,
and rotation commands. Static lanelet checks are direction-configurable for
site maneuvers. LiDAR/radar cost nodes first remove costs outside the active
route lanelets plus `route_lanelet_margin_m` (0.35 m); live obstacle checks stay
active during configured static-cost maneuver exceptions.
