# camrod_parking

## Role
`camrod_parking` provides docking-related runtime flow based on AprilTag detection and docking server orchestration.

## Package Diagram
```mermaid
graph TD
  TAG[Apriltag Node] --> RAW[Raw Tag Detections]
  BRIDGE[Parking Apriltag Bridge] <-- RAW
  BRIDGE --> DET[Avg Tag Detections]
  BRIDGE --> POSE[Avg Tag Pose]
  BRIDGE --> DOCK[Detected Dock Pose]
  OPN[Open Nav Docking] --> ACT[Docking Actions]
  LIFE[Lifecycle Manager] --> OPN
```

## Node Data Flow
| Node | Main Inputs | Main Outputs |
|---|---|---|
| `apriltag_ros/apriltag_node` | `/camera/color/image_rect`, `/camera/color/camera_info` | `/parking/docking/apriltag/detections_raw` |
| `parking_apriltag_bridge` | `/parking/docking/apriltag/detections_raw` | `/parking/docking/apriltag/detections`, `/parking/docking/apriltag/pose`, `/parking/docking/detected_dock_pose` |
| `opennav_docking` | docking target/config | docking control interfaces |
| `nav2_lifecycle_manager` | lifecycle config | activates `docking_server` |

## Inter-Package Connections
```mermaid
graph LR
  CAM[Camera Stream] --> PARK[Camrod Parking]
  PARK --> NAV[Nav Docking Flow]
  BR[Camrod Bringup] --> PARK
```

## Topic Summary
| Direction | Topic | Purpose |
|---|---|---|
| In | `/camera/color/image_rect`, `/camera/color/camera_info` | AprilTag detection input |
| In | `/parking/docking/apriltag/detections_raw` | bridge conversion input |
| Out | `/parking/docking/apriltag/detections` | avg_msgs-style detection output |
| Out | `/parking/docking/apriltag/pose` | tag pose output |
| Out | `/parking/docking/detected_dock_pose` | docking target pose output |

## Practical Usage
```bash
ros2 launch camrod_parking parking.launch.py
```

Or docking-only:
```bash
ros2 launch camrod_parking docking.launch.py
```

## Config Files
- `config/apriltag.yaml`
- `config/docking_server.yaml`
- `config/docks.yaml`
