# Documentation Changelog

Tracks changes to CAMROD documentation (READMEs, style guides, templates).  
For code changes see git log and release tags.

---

## [1.10-docs] — 2026-05-21

### Added
- `docs/templates/README_STYLE_GUIDE.md` — canonical 12-section README structure, Mermaid legend, Interface Contract column rules, Key Behavior block format
- `docs/templates/PACKAGE_README_TEMPLATE.md` — copy-paste skeleton for new package READMEs
- `docs/DOCS_CHANGELOG.md` — this file
- `camrod_parking/README.md` — new package documentation for AprilTag dock detection + opennav_docking
- Root `README.md` — Documentation Map table, Hardware & Software Requirements, Docker section, First Run Guide, Glossary, versioning table
- All package READMEs — stateDiagram-v2 and sequenceDiagram blocks for key behaviors

### Changed
- All package READMEs now follow the fixed 12-section structure from `README_STYLE_GUIDE.md`
- Topic tables expanded with Required / Rate / Meaning columns (Interface Contract format)
- Architecture diagrams split into Context (graph LR) + Runtime (graph TD) where applicable
- `PARAMETER_NAMING_STANDARD.md` — added §6 Quick Reference Cheatsheet, §7 Writing New Configs, §8 Per-Package Canonical Keys Index, §9 Related Docs

### Notes
- Legacy `*_sec` param aliases remain accepted at runtime (see `PARAMETER_NAMING_STANDARD.md §5`)
- `<!-- TODO: verify -->` comments mark values that need field-confirmation
- `camrod_parking` diagnostics checker not yet implemented (tracked with TODO in `camrod_system/README.md`)

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
