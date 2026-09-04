# v2.2.3 Radar And Service-Metrics UI Evidence

<!-- HH_260904 - Preserve deterministic desktop/mobile UI proof separately
from physical field evidence so fixture values cannot be mistaken for robot logs. -->

## Scope

These captures were produced from the `v2.2.3` production React frontend on
AMD64 with deterministic API fixtures. They verify rendering, terminology,
responsive layout, and the frontend/backend data contract. Distances, times,
counts, and radar ranges shown here are test inputs, not physical-robot
measurements.

| Artifact | Verified UI behavior |
|---|---|
| `service-evidence-b1-b13-desktop.png` | B1-B13 average distance/time bars, active B7 run, and aggregate KPIs fit the desktop view |
| `service-evidence-b1-b13-mobile.png` | The same evidence remains readable without horizontal page overflow on a narrow viewport |
| `service-evidence-responsive.gif` | Desktop and mobile evidence layouts use the same data contract |
| `radar-echo-cost-telemetry.png` | Finite `0.39-0.55 m` raw returns remain `ECHO`; only the fixture's authoritative RIGHT2 `0.09 m` evidence is labeled `COST` |

## Verification

- React production build completed successfully: JS `76.71 kB`, CSS
  `13.66 kB` after gzip.
- Frontend/backend and configuration contracts passed in the `166`-test
  focused Python run.
- Physical radar stop behavior and ARM64 performance remain field acceptance
  items in `TODOLIST.txt`.

