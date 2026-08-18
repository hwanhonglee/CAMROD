# Operator docking workspace

## Scope

The screenshot is from the production frontend served by the ROS 2 UI backend
and captured at `1600x1000` with headless Google Chrome. The selected docking
view was active at `10 Hz` with telemetry schema v3 and seven lazy ROS
subscriptions.

The view exposes:

- manual Return and parking start/cancel commands;
- AprilTag debug image and exact tag pose/distance;
- charging boolean and battery state;
- AprilTag/reverse controller status and maneuver paths.

`operator-docking-workspace.png` intentionally shows pending sensor values
because no physical tag or charger was connected during the UI-only capture.
