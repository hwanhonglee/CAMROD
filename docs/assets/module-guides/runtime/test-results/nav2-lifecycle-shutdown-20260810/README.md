# Nav2 Lifecycle And Full-Graph Shutdown, 2026-08-10

<!-- HH_260810 - Preserve the post-fix AMD64 run without promoting it to ARM64
or physical-robot acceptance. -->

This record validates the lifecycle-manager shutdown correction against the
installed Release workspace. No navigation goal was published. The graph
reached `[SYSTEM] OK`; `/ui/state` reported `ready=true`,
`mission_phase=READY`, and zero diagnostic errors.

The launch parent alone received SIGINT. All 44 launched processes reported a
clean finish, including the standalone Nav2 lifecycle manager and six scoped
component containers. No nonzero/signal exit, forced kill, or descendant
remained. The isolated vendored lifecycle suite also passed all 10 CTest
targets, including lifecycle and bond launch tests.

```bash
ROS_DOMAIN_ID=79 ros2 launch camrod_bringup bringup.launch.py \
  sim:=true rviz:=false clean_before_launch:=false clean_on_shutdown:=false \
  enable_operator_ui_window:=false enable_guest_ui:=false
```

| File | Purpose |
|---|---|
| [`result.json`](result.json) | Command, source/map identity, assertions, counts, and limitations |
| [`full-graph-controlled-stop.log`](full-graph-controlled-stop.log) | Unfiltered startup, SYSTEM OK, parent-only SIGINT, and process-exit log |

This is AMD64 functional/shutdown evidence only. ARM64 8-core/16GB resource,
thermal, sensor-rate, CAN, and physical-motion acceptance remains in
`TODOLIST.txt`.
