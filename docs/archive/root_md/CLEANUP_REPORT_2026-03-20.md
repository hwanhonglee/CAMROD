# Cleanup Report (2026-03-20)

## Scope
Workspace: `/home/camrod_ws/src`

## Removed (hard delete)
- `.vscode/`
- `log/`
- `todo/` (including nested `vanjee_ws/build`, `install`, `log` artifacts)
- all `__pycache__/` directories under source packages

## Moved to review folder (non-destructive)
Moved to: `/_cleanup_review_20260320/root_backups/`

- `Dockerfile.camrod.amd64`
- `Dockerfile.camrod.arm64`
- `lanelet2_map_(c_track_test).osm`
- `lanelet2_maps (national_park_v0.1).osm`
- `lanelet2_maps (national_park_v0.2_add_drop_zone).osm`
- `lanelet2_maps (national_park_v0.3).osm`
- `lanelet2_maps (national_park_v0.4_no_transform).osm`
- `lanelet2_maps (national_park_v0.5_transform).osm`
- `lanelet2_maps (national_park_v0.6_add_camping_site).osm`
- `map_projector_info (copy_org).yaml`
- `map_projector_info (copy_park).yaml`

## Kept intentionally
- `lanelet2_maps.osm` (active map)
- `map_projector_info.yaml` (active projector config)
- `docker/` (active Docker build scripts and Dockerfiles)
- `util/` (manual conversion/utility scripts; not auto-removed)

## Finalize (if you approve permanent deletion)
```bash
cd /home/camrod_ws/src
rm -rf _cleanup_review_20260320
```

