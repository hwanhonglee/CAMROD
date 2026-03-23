# CAMROD Build Command (Including External Packages)

Run from `/home/camrod_ws`:

```bash
./src/bootstrap_module_externals.sh
./build_with_external.sh
```

Equivalent raw command:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src $(find src -mindepth 3 -maxdepth 3 -type d -path '*/external/*')
```

Example (partial build):

```bash
./build_with_external.sh --packages-up-to camrod_bringup
```
