# Unpublished Ranger controller companion

<!-- HH_261013 - Explicit dependency closure, not an upstream release or renewed motion authorization. -->

The selected October 12 CARLA observations used local controller changes in the
external [Ranger repository](https://github.com/hwanhonglee/ranger-carla-4ws-pipeline).
Those changes were **not committed or pushed upstream** when this companion was
captured. CAMROD alone is therefore not a complete runnable reproduction.

This [patch](controller.patch) contains exactly 13 modified tracked files and six
new controller/helper tests or modules. [MANIFEST.json](MANIFEST.json) records the
exact base `10af3fcaf8137c7552486a7ba7e2000c203a0fe3`, patch SHA-256 and every
before/after SHA-256, size and executable mode. It excludes maps, terrain history,
`.work`, raw logs and authorization receipts. A separately prepared compatible
CARLA/Unreal world and assets are still required; this patch does not prepare them.

## Check, then explicitly apply

Use a separate checkout at the exact base commit. Replace the example path with
that checkout. Running without `--apply` never changes its worktree or index:

```bash
python3 camrod_carla_adapter/companion/ranger_controller_20261013/bundle.py \
  --repo /path/to/ranger-carla-4ws-pipeline
```

`READY` means the patch checks against exact original file bytes and modes.
`ALREADY_APPLIED` requires exact patched bytes plus `git apply --reverse --check`;
it does not reapply or undo anything. Wrong base, partial application, conflicting
targeted files, staged targeted changes, corrupt patch or symlink paths fail closed.
Unrelated dirty files are neither cleared nor staged. Only after reviewing the
patch and explicitly deciding to modify that checkout:

```bash
python3 camrod_carla_adapter/companion/ranger_controller_20261013/bundle.py \
  --repo /path/to/ranger-carla-4ws-pipeline --apply
```

No reset, clean, commit, push, build, simulator startup, vehicle control or automatic
rollback is performed. Keep local work backed up; after any unexpected failure,
inspect the checkout before retrying. The hash manifest is integrity/provenance,
not a cryptographic signature or permission to run motion.

## Rebuild and obtain fresh evidence

Rebuild the external ROS controller and CAMROD against the actual environment,
run their offline tests, and use the documented physical acceptance/bootstrap
workflow to obtain fresh source-bound build/test receipts and a fresh gate bound
to the actual simulator server and actor. Historical receipts in the public
evidence bundle are not reusable runtime authorization. Existing safety limits,
obstacle retention, prepared-world requirements and operator control remain in
force. The selected 13 completions are mixed historical profiles, not a fresh
same-profile replay, real-robot, recall or charging-docking acceptance.

## Regenerate only from the exact recorded source state

`bundle.py --repo /path/to/source --bundle /new/output --generate` constructs the
explicit allowlisted patch using `git diff` plus new-file diffs. It verifies it
against the base in a disposable index and reverse-checks the source worktree;
the actual source index/worktree/ref are not written. Existing patch/manifest
outputs are never overwritten. No automatic external apply is part of generation.

Focused offline checks: `python3 -m pytest -q
camrod_carla_adapter/companion/ranger_controller_20261013/test_bundle.py`.
