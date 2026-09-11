# Shared UI and voice restoration — 2026-09-11

<!-- HH_260911 - Restore retained user changes without rewriting calibration, audio or published tags. -->

## Basis and scope
The pre-restoration refs were develop/worak-test `276e7ee68` and virtual/carla
`25fc22f28`. The original Korean UI was present on CARLA (`de3e5187e`, retained
in `44ad5bdba`) but not in the merged worak PR baseline `914c3ed1d`.
This follow-up restores shared presentation, not the CARLA physical/controller
implementation. Earlier uncommitted map/calibration work remains outside it.

## Changes
- Restore Korean mission/health/manual/parking/camera/docking labels, explicit
  stop-button wording, safe departure instructions and departure-retry feedback.
- Keep existing arrival/charging-complete and service-available notices intact.
- Translate known docking acknowledgement/error codes without changing request
  endpoints, success flags, authority checks or unknown diagnostic messages.
- Add 13 missing site-name PCM WAV clips for navigation.site_B1 through B13.
  These clips are NEW synthesis, not recovered user recordings. Exact text,
  tool/voice settings, lengths and SHA256 are in site_announcements.json.
- Preserve all 24 preexisting audio/BGM assets byte-for-byte, including the
  user's current 5.856-second recall cue and the archived 6.552-second cue.
- Restore the old transcript separately as recall_clear_site(before).md;
  do not falsely describe the current recording with the old transcript.
- Correct frontend postbuild installation: sibling develop output must not
  overwrite the CARLA install prefix. Use colcon_build.sh --print-paths.
- Export the voice component library so a fresh sourced ROS environment can
  locate the dlopen-loaded component, without manual library-path injection.

## Verification
- Actual develop UI + voice test directories: 612 passed, no failures.
- Actual virtual/carla UI + voice test directories: 727 passed, no failures.
- Both actual frontend source trees completed npm production builds. Fresh
  Chrome rendered waiting/service-selection screens without JavaScript errors.
  These browser checks had NO robot backend and did not demonstrate driving.
- Actual voice_announcer executable on isolated domain 189/localhost:
  all 13 new site clips, the preserved recall clip and the generic departure
  clip reached PLAYING then IDLE; no missing-file or decoder warnings.
  SDL dummy output was used: physical speaker/listening quality is NOT verified.
- Voice was built/installed from each worktree; develop outputs were isolated.
  Develop UI was also installed into its checkout-specific prefix. CARLA's
  installed frontend was synchronized to its own new hashed bundle.
- All 24 preexisting audio/BGM files and earlier dirty calibration/map files
  match the saved before hashes. Backend, C++ control and voice source code
  are unchanged in this restoration; the voice CMake export is intentional.

## Limits and future edits
This is restoration verification, not a new B1-B13 CARLA-driving acceptance or
resolution of the separately paused map/calibration audit. Existing recordings
may intentionally be replaced later; update the provenance/hash manifest under
review rather than silently substituting older audio. Technical ROS identifiers
remain stable; user-facing restoration is not a blanket translation of all
third-party/debug terminology. No git push or published v2.2.6 tag move is part
of this change. Map/calibration edits are preserved uncommitted and not bundled
with these commits.

## Local evidence
`/home/hong/camrod_ws/_sync_backups/ui_voice_restore_20260911T124133/`
contains source backups, full test/build/playback logs, rendered screenshots,
asset provenance and preservation checks. Compact machine-readable results
are committed under `docs/evidence/ui_voice_restore_20260911/`.
