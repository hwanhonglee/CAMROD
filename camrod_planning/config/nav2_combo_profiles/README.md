# Nav2 Combo Profiles

- Source dir: `camrod_planning/config/nav2_combo_profiles`
- Purpose: Planner/Controller pair-specific override profiles loaded via `nav2_combo_param_file`.
- Baseline sweep date: `2026-05-21` (sim, 20s timeout per combo).

## Usage

```bash
ros2 launch camrod_bringup bringup.launch.py sim:=true \
  nav2_combo_param_file:=/home/hong/camrod_ws/src/camrod_planning/config/nav2_combo_profiles/navfn_rpp.yaml
```

Use `disabled.yaml` to apply no combo-specific overrides.

## Auto Selector Behavior

- HH_260528: `nav2_combo_param_file` now also auto-selects planner/controller IDs.
- Mapping is inferred from combo filename tokens:
  - planner: `navfn`, `smac2d`, `thetastar`, `smachybrid`, `smaclattice`
  - controller: `rpp`, `dwb`, `mppi`, `graceful`, `rotationshim`
- Override explicitly only when needed:
  - `nav2_selected_planner:=NavFn|Smac2D|ThetaStar|SmacHybrid|SmacLattice`
  - `nav2_selected_controller:=RPP|DWB|MPPI|Graceful|RotationShim`

## Common `use_*` Toggles

- `use_astar`:
  - `True`: A* search (heuristic-guided)
  - `False`: Dijkstra-style expansion
- `use_rotate_to_heading` (RPP):
  - `True`: rotate-in-place heading alignment before path tracking
  - `False`: track with linear+angular together
- `use_velocity_scaled_lookahead_dist` (RPP):
  - `True`: lookahead scales with speed
  - `False`: fixed lookahead distance only
- `use_collision_detection` (RPP):
  - `True`: enable controller-level collision check
  - `False`: skip internal check (costmap/layers still apply)
- `use_regulated_linear_velocity_scaling` (RPP):
  - `True`: curvature-based speed regulation
  - `False`: disabled
- `use_cost_regulated_linear_velocity_scaling` (RPP):
  - `True`: costmap-based speed regulation
  - `False`: disabled

## Sweep Matrix (P/C/M)

- `P`: global path observed
- `C`: local/controller path observed
- `M`: displacement >= 0.5m within 20s

| Planner | Controller | Result | Displacement(m) | Profile |
|---|---|---|---:|---|
| NavFn | RPP | `P=Y/C=Y/M=Y` | 7.196 | `navfn_rpp.yaml` |
| NavFn | DWB | `P=Y/C=Y/M=Y` | 10.975 | `navfn_dwb.yaml` |
| NavFn | MPPI | `P=N/C=Y/M=Y` | 3.727 | `navfn_mppi.yaml` |
| NavFn | Graceful | `P=Y/C=Y/M=N` | 0.000 | `navfn_graceful.yaml` |
| NavFn | RotationShim | `P=Y/C=Y/M=N` | 0.000 | `navfn_rotationshim.yaml` |
| Smac2D | RPP | `P=Y/C=Y/M=Y` | 10.917 | `smac2d_rpp.yaml` |
| Smac2D | DWB | `P=Y/C=Y/M=N` | 0.000 | `smac2d_dwb.yaml` |
| Smac2D | MPPI | `P=Y/C=Y/M=N` | 0.006 | `smac2d_mppi.yaml` |
| Smac2D | Graceful | `P=Y/C=Y/M=N` | 0.000 | `smac2d_graceful.yaml` |
| Smac2D | RotationShim | `P=Y/C=Y/M=N` | 0.000 | `smac2d_rotationshim.yaml` |
| ThetaStar | RPP | `P=Y/C=Y/M=Y` | 11.698 | `thetastar_rpp.yaml` |
| ThetaStar | DWB | `P=Y/C=Y/M=N` | 0.008 | `thetastar_dwb.yaml` |
| ThetaStar | MPPI | `P=Y/C=Y/M=N` | 0.053 | `thetastar_mppi.yaml` |
| ThetaStar | Graceful | `P=Y/C=Y/M=N` | 0.003 | `thetastar_graceful.yaml` |
| ThetaStar | RotationShim | `P=Y/C=Y/M=N` | 0.001 | `thetastar_rotationshim.yaml` |
| SmacHybrid | RPP | `P=Y/C=Y/M=Y` | 10.818 | `smachybrid_rpp.yaml` |
| SmacHybrid | DWB | `P=Y/C=Y/M=N` | 0.009 | `smachybrid_dwb.yaml` |
| SmacHybrid | MPPI | `P=Y/C=Y/M=N` | 0.000 | `smachybrid_mppi.yaml` |
| SmacHybrid | Graceful | `P=Y/C=Y/M=N` | 0.000 | `smachybrid_graceful.yaml` |
| SmacHybrid | RotationShim | `P=Y/C=Y/M=N` | 0.000 | `smachybrid_rotationshim.yaml` |
| SmacLattice | RPP | `P=Y/C=Y/M=Y` | 6.766 | `smaclattice_rpp.yaml` |
| SmacLattice | DWB | `P=Y/C=Y/M=Y` | 10.397 | `smaclattice_dwb.yaml` |
| SmacLattice | MPPI | `P=Y/C=Y/M=Y` | 4.794 | `smaclattice_mppi.yaml` |
| SmacLattice | Graceful | `P=Y/C=Y/M=Y` | 1.022 | `smaclattice_graceful.yaml` |
| SmacLattice | RotationShim | `P=Y/C=Y/M=Y` | 0.846 | `smaclattice_rotationshim.yaml` |
