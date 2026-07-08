# Documentation Changelog

Tracks changes to CAMROD documentation (READMEs, style guides, templates).
For code changes see git log and release tags.

---

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
