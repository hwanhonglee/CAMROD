# Documentation Changelog

<!-- HH_260724 - Track manual engage and operator-stop visibility after v2.0.7 tagging. -->
## [manual-engage-operator-stop-docs] - 2026-07-24 (HH_260724)

### Changed

| Doc | What changed |
|-----|--------------|
| `camrod_ui/README.md` | Documented Manual driving display and `/service/state=OPERATOR_STOPPED` cancel/stop behavior |
| `camrod_system/README.md` | Documented suppression of expected Nav2 ABORTED diagnostics during campsite maneuver ownership |
| `camrod_bringup/README.md` | Documented live terminal watch visibility for manual/mission engage and platform drive-enable |

---

<!-- HH_260724 - Track the low-battery campsite mission policy and v2.0.7 release docs. -->
## [v2.0.7-battery-policy-docs] - 2026-07-24 (HH_260724)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.7 and added the sim validation flag for finish-current-mission low-battery return |
| `camrod_bringup/README.md` | Documented the low-battery user-return simulation and 34% blocked recall probe |
| `camrod_control/README.md` | Documented the 20% hard stop, 35% charger departure gate, and parked-versus-charging state semantics |
| `camrod_ui/README.md` | Documented the 35% campsite dispatch gate and user-confirmed low-battery return latch parameters |
| `docs/V2_0_7_RELEASE_NOTES.md` | Recorded the battery policy scope, state semantics, synchronized config values, and validation evidence |

---

<!-- HH_260723 - Track the v2.0.6 localization, routing, perception, and occupancy field release. -->
## [v2.0.6-field-docs] - 2026-07-23 (HH_260723)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.6 and summarized the explicit reverse-route, camera/YOLO, campsite-occupancy, and perception-only cost profile |
| package READMEs | Synchronized localization diagnostics, semantic detections, occupied-site blocking, and raw-LiDAR cost-switch contracts |
| `docs/V2_0_6_RELEASE_NOTES.md` | Recorded root causes, runtime interfaces, verification evidence, and remaining field limits |

---

<!-- HH_260722 - Track the hardware-verified dual-GNSS v2.0.6 production default. -->
## [v2.0.6-dual-gnss-docs] - 2026-07-22 (HH_260722)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Promoted v2.0.6 and added the two logical GNSS port roles used by default hardware bringup |
| `camrod_sensing/README.md` | Replaced the direct-rover diagram and stale mountpoint/topic guidance with the corrected moving-base cascade, A/B evidence, and no-argument default launch |
| `camrod_bringup/README.md` | Documented the five synchronized GNSS defaults, config-mirror contract, and live acceptance commands |
| `camrod_bringup/docs/field_test_runbook.md` | Added dual-port preflight, snapshot evidence, RTK/heading acceptance flags, and one-shot recovery guidance |
| `docs/V2_0_6_DUAL_GNSS_RELEASE_NOTES.md` | Recorded implementation scope, hardware measurements, verification evidence, and the two-logical-port limit |

---

<!-- HH_260721 - Track active coordinate export and constrained roadside campsite behavior. -->
## [v2.0.5-roadside-docs] - 2026-07-21 (HH_260721)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Split normal campsite turnaround from B12/B13 roadside return behavior |
| package READMEs | Documented operational service poses across map, planning, control, UI, and bringup |
| `docs/V2_0_5_RELEASE_NOTES.md` | Added active coordinate values, full simulation evidence, and physical roadside validation limit |
| `camrod_sensing/README.md` | Documented the single ROS 2 ground-segmentation package and stale-tree build guard |

---

<!-- HH_260721 - Track the campsite retrace and parked departure validation release. -->
## [v2.0.5-docs] - 2026-07-21 (HH_260721)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.5 as the validated baseline and clarified same-lane campsite retrace |
| package READMEs | Updated control phase names, planning handoff guards, simulation charging feedback, UI departure ordering, and diagnostics behavior |
| `docs/V2_0_5_RELEASE_NOTES.md` | Recorded the complete reverse-parking simulation, build/test evidence, synchronized configuration, and hardware-only limits |

---

<!-- HH_260721 - Record the corrected non-hardware release-validation invocation. -->
## [2.0.4-validation] - 2026-07-21 (HH_260721)

### Changed

| Doc | What changed |
|-----|--------------|
| root `README.md` | Paired the simulated BMS publisher with `sim_platform_status_enable:=true` in bringup |
| `camrod_bringup/README.md` | Added the exact reverse-parking charging-recall launch and final report paths |
| `docs/V2_0_4_RELEASE_NOTES.md` | Updated final charging, obstacle, perception, and forced-cppcheck evidence |
| `camrod_platform/README.md` | Distinguished ordinary-PC simulation from real Ranger CAN startup |

Tracks changes to CAMROD documentation (READMEs, style guides, templates).
For code changes see git log and release tags.

---

## [v2.0.4-docs] - 2026-07-21 (HH_260721)

<!-- HH_260721 - Track the native control, EKF-only, and charging-recall release evidence. -->

### Changed - native control and full reverse-parking validation

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.4 as the validated baseline and updated charging/build commands |
| `camrod_control/README.md` | Replaced the stale Python tree with native C++ nodes and policy helpers |
| `camrod_bringup/README.md` | Added charging recall, directional gate, replan, and config-mirror commands |
| `camrod_localization/README.md` | Documented robot_localization EKF as the only runtime backend |
| `docs/V2_0_4_RELEASE_NOTES.md` | Recorded implementation scope, exact simulation evidence, package tests, and known limits |

---

## [v2.0.3-docs] - 2026-07-21 (HH_260721)

<!-- HH_260721 - Track the control consolidation and reverse-only release evidence. -->

### Changed - control ownership and reverse-parking validation baseline

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.3 as the current validated baseline and linked its release notes |
| `camrod_control/README.md` | Documented gate, maneuver, parking, charging-departure, and command-topic ownership |
| `camrod_bringup/README.md` | Documented the four byte-identical control configuration mirrors and reverse-only validation command |
| package READMEs | Replaced legacy docking, parking, platform-gate, and message-alias descriptions with current interfaces |
| `docs/V2_0_3_RELEASE_NOTES.md` | Added migration inventory, build/package evidence, complete reverse-only simulation results, and known limits |

---

## [v2.0.2-docs] — 2026-07-16 (HH_260716)

### Changed — field map, sensing, localization, and validation baseline

| Doc | What changed |
|-----|--------------|
| root `README.md` | Set v2.0.2 current release; summarized moved Park map, radar thresholds, JECH NTRIP, camera/YOLO gating, GNSS health gates, and full config/install sync |
| `camrod_bringup/README.md` | Updated active map path/profile and added the reusable camera/YOLO probe |
| `camrod_bringup/docs/field_test_runbook.md` | Added all-package install checks, radar minimum-range validation, GNSS covariance triage, YOLO subscriber-gating notes, and duplicate debug-process load guidance |
| `camrod_map/README.md` | Documented `copy_park_moved`, map version 13, and synchronized projection metadata |
| `camrod_sensing/README.md` | Updated JECH mountpoint, single/dual GNSS rates, 15-degree radar profile, and measured self-echo dead zones |
| `camrod_localization/README.md` | Updated monitor thresholds and clarified why live GNSS may still result in `DR_ONLY` |
| `camrod_perception/README.md` | Documented the component-container path, continuous detection health topic, and subscriber-gated YOLO image behavior |
| `docs/V2_0_2_RELEASE_NOTES.md` | Added operator-facing change inventory, validation checklist, and known runtime observations |

## [lights-docs] — 2026-07-08 (260708)

### Added — exterior light feature docs

| Doc | What changed |
|-----|--------------|
| `camrod_platform/docs/lights-design-doc.html` | New: implementation design for headlight relay + WS2815 turn indicators (decisions, architecture, serial protocol, failsafe, TODO) |
| `camrod_platform/docs/turn-signal-explainer.html` | New: visual comparison of curvature vs lanelet `turn_direction` direction sources |
| `camrod_platform/README.md` | Added light_controller/mcu_serial_bridge summary item, `lights.launch.py` row, lights topics in interface contract, launch args, `config/lights.yaml` row |
| root `README.md` | Added `/planning/route_turn_segments`, `/platform/headlight/command`, `/platform/lights/*` to the key-topics table |

---

## [field-test-docs] — 2026-07-08 (HH_260708)

### Added — outdoor test memory and evidence capture

| Doc | What changed |
|-----|--------------|
| `camrod_bringup/docs/field_test_runbook.md` | New: outdoor workflow for config sync, bringup logging, snapshots, topic Hz probes, software gates, radar/LiDAR/perception-cost checks, camping-site flow, and drop-zone return |
| `camrod_bringup/README.md` | Added `field_test_tool.sh` quick-start commands and field runbook reference |
| root `README.md` | Added field helper and runbook rows to runtime/operator reference |

---

## [1.11-docs] — 2026-05-28 (HH_260528)

### Changed — package READMEs

| Package | What changed |
|---------|-------------|
| `camrod_sensing` | Dual econ camera architecture (front: GPU/VPI, rear: CPU/GStreamer); `imu_mode` → `imu_model` rename; updated topic list and launch args |
| `camrod_docking` | Full Korean → English translation; Camera TF ownership moved to `camrod_sensor_kit`; new frame names `camera_front_link`/`camera_rear_link` |
| `camrod_sensor_kit` | TF diagram updated with `camera_front_link` and `camera_rear_link` frames |
| `camrod_platform` | Added `platform_type`, `ranger_bridge_enable`, `sensor_kit_bridge_enable` launch args |
| `camrod_bringup` | Updated sensing config filename; added per-camera enable flags; platform args |

### Changed — docs/

- `docs/archive/TODO_BRINGUP_SUMMARY.md` — updated topic paths and node names to match HH_260528 state
- `docs/archive/TODO_MODULE_RUNTIME_FLOW.md` — sensing section updated for dual camera and unified IMU

### Added — code (reflected in docs)

- `camrod_sensing/config/camera/camera_rear_calibration.yaml` — rear camera intrinsics (plumb_bob, 1920×1080)
- `camrod_bringup/config/sensing/camera/camera_params.yaml` — bringup-level deployment override for both cameras

### Deleted — code (reflected in docs)

- `camrod_sensing/launch/imu_cv7.launch.py` — replaced by unified `imu.launch.py`
- `camrod_sensing/launch/imu_gq7_ntrip.launch.py` — replaced by unified `imu.launch.py`
- `camrod_sensing/src/camera_publisher_node.cpp` — renamed to `camera_front_publisher_node.cpp`
- All `*copy_org*`, `*copy_750*`, `*copy_722*` backup files across sensing and bringup

---

## [1.10-docs] — 2026-05-21

### Added
- `docs/templates/README_STYLE_GUIDE.md` — canonical 12-section README structure, Mermaid legend, Interface Contract column rules, Key Behavior block format
- `docs/templates/PACKAGE_README_TEMPLATE.md` — copy-paste skeleton for new package READMEs
- `docs/DOCS_CHANGELOG.md` — this file
- `camrod_parking/README.md` → renamed to `camrod_docking/README.md` with architecture update
- Root `README.md` — Documentation Map table, Hardware & Software Requirements, Docker section, First Run Guide, Glossary, versioning table
- All package READMEs — stateDiagram-v2 and sequenceDiagram blocks for key behaviors

### Changed
- All package READMEs now follow the fixed 12-section structure from `README_STYLE_GUIDE.md`
- Topic tables expanded with Required / Rate / Meaning columns (Interface Contract format)
- Architecture diagrams split into Context (graph LR) + Runtime (graph TD) where applicable
- `PARAMETER_NAMING_STANDARD.md` — added §6 Quick Reference Cheatsheet, §7 Writing New Configs, §8 Per-Package Canonical Keys Index, §9 Related Docs

---

## [1.9-docs] — 2026-05-13

### Added
- Initial `PARAMETER_NAMING_STANDARD.md` with canonical naming rules and migration policy

### Changed
- `camrod_system` diagnostic checker params migrated to canonical `*_hz` / `*_s` suffixes

---

## [1.0-docs] — 2026-04-28

### Added
- Initial `README.md` at workspace root with architecture diagram and build/run instructions
- Package-level `README.md` files for all initial packages
