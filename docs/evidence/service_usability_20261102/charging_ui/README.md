# Charging UI browser fixtures

<!-- HH_261103 - Keep original fixture hashes and counts historical, not a claim about later release acceptance. -->
Historical capture record as of **2026-11-02**. Bundle hashes, test counts and references to current source/install state below describe that capture, not later publication or CARLA validation. See [unit2-v1.0.1 release scope and limits](../../../releases/unit2-v1.0.1.md); the raw manifest is unchanged.

HH_261102 — These images render the production React bundle in headless Chrome at 1280×800 against a loopback-only fixture server. The visible badge marks every image **UI FIXTURE · NO ROBOT / NO CARLA**. They are UI evidence, not electrical isolation, physical charger separation, live robot movement, or safety certification.

`01` and `02` show the existing 34%/35% service threshold. `03`–`05` show explicit contact-clearance confirmation, movement inside drop_zone, and the stopped staging state. `06` shows that ready can use the existing delivery/recall menu without a cargo-complete step. `07` shows the concise delivery confirmation. `08` shows stale vehicle feedback without a fresh charging claim or enabled movement; `09` shows the ordinary mission Stop confirmation still available in that state. The public stop is checked inside the viewport whenever the charging screen is shown. `10` and `11` render the real Guest HTML at 430×932 with a separate idle WebSocket fixture and its existing call confirmation.

`manifest.json` records exact fixture snapshots, the single captured POST (`leave`, `confirm=true`), source hashes, and the captured build's asset manifest. Ready makes no automatic request and adds no completion or return workflow. The frontend Jest suite separately checks a one-hour wait without any automatic movement.

To reproduce after installing the frontend's existing dependencies and building it, run from the repository root:

```sh
node docs/evidence/service_usability_20261102/charging_ui/capture_fixture.cjs
```

The script starts and closes its own isolated loopback HTTP fixture and headless Chrome; it does not connect to ROS or restart any robot service. PNGs and `manifest.json` are regenerated in this folder. `charging_access_flow.gif` is a montage of `01`–`06`, not a live motion recording.

After the public-copy simplification on 2026-11-02, full frontend Jest passed **338 tests in 20 suites**, and the complete Python UI suite passed **1,045 tests**. The two updated Robot source-contract files passed **51/51**. See [copy-change details and before/after images](../ui_copy_simplification/README.md).

The final `CI=true npm run build` compiled `main.361f76de.js` / `main.f4da5088.css` and verified **58 public assets**, also checked against the test workspace's installed package. The existing outdated Browserslist-data notice was non-blocking. Eleven fixture PNGs and their manifest match this production bundle and current source hashes; the only recorded mock movement request is `leave&confirm=true`. These are software/UI checks, not live robot or charger safety acceptance.
