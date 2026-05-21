# CAMROD README Style Guide (v1.10)

Reference for anyone writing or reviewing package-level documentation in this repo.

---

## 1. Fixed Section Order

Every package README must contain these sections **in this order**:

| # | Section | Purpose |
|---|---------|---------|
| 1 | Title | Package name + one-line role |
| 2 | Summary | At-a-glance block (4–6 lines) + Non-goals |
| 3 | Quick Start | Single most-common launch command first |
| 4 | System Position | Where this package sits (graph LR) |
| 5 | Runtime Architecture | Internal nodes and topic flow (graph TD) |
| 6 | Interface Contract | Inputs / Outputs / TF / Services tables |
| 7 | Key Behaviors | State diagrams / sequence diagrams for important logic |
| 8 | Launch | Arguments table |
| 9 | Config | Which file to edit for which scenario |
| 10 | Validation | Commands to verify the package is healthy |
| 11 | Troubleshooting | Symptom → likely cause → what to check |
| 12 | Related Docs | Relative links to root README + related packages |

**Exceptions:** Interface-only packages (e.g. `avg_msgs`) have no Launch or Validation sections; adapt with a note.

---

## 2. Mermaid Legend

Use this legend **consistently across all diagrams**:

| Syntax | Meaning |
|--------|---------|
| `[node_name]` | ROS 2 node (executable) |
| `((topic_name))` | ROS 2 topic |
| `{{hardware_or_file}}` | External hardware device or file on disk |
| `[(config_file)]` | ROS parameter / config file |
| `[[external_stack]]` | External package or sub-launch |
| Dashed arrow `-.->` | Non-runtime dependency (build-time, interface, optional) |
| Solid arrow `-->` | Runtime data flow |

---

## 3. Diagram Types — When to Use Which

| Diagram | Mermaid type | When to use |
|---------|-------------|-------------|
| **Context** | `graph LR` | Show upstream/downstream packages (System Position) |
| **Runtime** | `graph TD` | Show internal nodes, topics, data flow (Runtime Architecture) |
| **State** | `stateDiagram-v2` | Show mode transitions (localization mode, mission state, docking lifecycle) |
| **Sequence** | `sequenceDiagram` | Show cross-package interaction order (recall flow, init flow, docking mission) |
| **Flowchart** | `graph TD` | Decision trees, sentinel logic, gate logic |

Prefer **2–4 small purpose-built diagrams** over one large diagram.

---

## 4. Interface Contract Table Columns

### Inputs

| Topic | Type | Required | Producer | Rate / Trigger | Meaning |
|-------|------|----------|----------|----------------|---------|

### Outputs

| Topic | Type | Consumer | Rate / Trigger | Meaning |
|-------|------|----------|----------------|---------|

Fill every column. Use `—` only if genuinely not applicable.  
Use `<!-- TODO: verify -->` when the value is uncertain.

---

## 5. Key Behavior Block Format

Every significant runtime behavior must use this format:

```
**Behavior name**
- **Trigger:** what condition activates this behavior
- **Internal logic:** what the node does internally
- **Output effect:** what topic/service changes
- **Operator-visible symptom:** what the operator sees in UI or RViz
- **Related params:** `param_name_s`, `param_name_hz`
- **Related topics:** `/topic/name`
```

---

## 6. Canonical Parameter Naming (v1.10)

See [/PARAMETER_NAMING_STANDARD.md](../../PARAMETER_NAMING_STANDARD.md) for the full standard.

Quick reference:

| Concept | Suffix | Example |
|---------|--------|---------|
| Duration in seconds | `_s` | `cost_stop_hold_s` |
| Frequency in Hz | `_hz` | `publish_rate_hz` |
| Topic name | `_topic` | `cmd_vel_in_topic` |
| Topic list | `_topics` | `subscribed_topics` |
| Frame ID | `_frame_id` | `map_frame_id` |

**Rules:**
- Always use canonical keys in documentation examples.
- If documenting backward-compatible legacy keys, add an explicit note: _"Legacy `*_sec` alias still accepted; see PARAMETER_NAMING_STANDARD.md §5."_
- Never silently use `*_sec` in new documentation.

---

## 7. Relative Link Conventions

Always use **repo-relative paths** from the file's own location:

```markdown
<!-- From inside camrod_planning/README.md -->
[Root README](../README.md)
[camrod_localization](../camrod_localization/README.md)
[PARAMETER_NAMING_STANDARD](../PARAMETER_NAMING_STANDARD.md)
[avg_msgs](../camrod_common/avg_msgs/README.md)
```

Link to source files where helpful:
```markdown
[planning.launch.py](launch/planning.launch.py)
[nav2_base.yaml](config/nav2_base.yaml)
```

---

## 8. General Writing Rules

- Write in clear English. Keep ROS/robotics terms as-is (do not translate).
- Each major section starts with 1–2 sentences explaining why it exists.
- Do **not** delete existing factual content — move it into the new structure.
- Mark uncertain values with `<!-- TODO: verify -->` rather than inventing them.
- No emoji in headings or tables.
- Code blocks use ` ```bash ` for shell commands, ` ```yaml ` for config, ` ```cpp ` for C++.
