# CAMROD v2.0.6 Dual-GNSS Baseline Notes

<!-- HH_260722 - Record the hardware-verified corrected moving-base RTK and heading release. -->

Release date: 2026-07-22
Target branch: `develop`
Target remotes: `hwanhonglee/CAMROD`, `tele-genius/CAMROD`

## Scope

- Made the corrected moving-base cascade the default dual-GNSS topology.
- Routed CORS RTCM from `ntrip_client` through
  `moving_base_rtcm_writer` to `/dev/ttyUSB0` at 115200 baud.
- Kept `/dev/ttyACM0` dedicated to heading-rover NAV-PVT and
  NAV-RELPOSNED output, with rover USB RTCM input disabled by default.
- Matched the dual-GNSS navigation rate to the moving base at 1 Hz.
- Retained direct rover NTRIP and warm start as explicit diagnostic and
  one-shot recovery controls instead of production defaults.
- Synchronized the five GNSS route defaults through sensing launch, full
  bringup, deployment YAML, and package-owned YAML.
- Added regression tests for correction ownership, launch argument forwarding,
  safe recovery behavior, and byte-identical bringup configuration mirrors.

## Default Hardware Route

<!-- HH_260722 - Define the two logical serial roles validated on the field harness. -->

```text
NTRIP CORS
    -> /sensing/gnss/ntrip_client/rtcm
    -> moving_base_rtcm_writer
    -> /dev/ttyUSB0 at 115200 baud (POWER+XBEE / Lite UART1)
    -> corrected moving-base RTCM over the board UART/XBee path
    -> heading rover UART2
    -> /dev/ttyACM0 (NAV-PVT and NAV-RELPOSNED output)
```

The synchronized launch defaults are:

| Argument | Default |
|---|---|
| `ublox_dual_antenna` | `true` |
| `ublox_dual_forward_ntrip_to_rover` | `false` |
| `ublox_dual_warm_start_on_startup` | `false` |
| `ublox_dual_base_rtcm_device` | `/dev/ttyUSB0` |
| `ublox_dual_base_rtcm_baud` | `115200` |

## Correction Ownership

<!-- HH_260722 - Explain why message filtering could not make two reference streams coexist. -->

The previous dual mode mixed moving-base RTCM on rover UART2 with CORS RTCM on
rover USB. Both live streams used station ID `0`: CORS carried `1006` and MSM5,
while the moving base carried `4072.0`, MSM4, and `1230`. Filtering only `4072`
left the other independent reference and observation messages mixed at the
rover, so absolute position and moving-baseline heading did not remain fixed
together.

The production route now assigns the external correction stream to exactly one
owner. `moving_base_rtcm_writer` preserves RTCM bytes, retries serial opening
after a disconnect, and feeds only the moving base. The heading rover receives
only the corrected moving-base stream on UART2. The direct-rover switch remains
available for diagnosis, but enabling it suppresses the base writer.

## Hardware A/B Validation

<!-- HH_260722 - Preserve the measured fixed-position and fixed-heading acceptance evidence. -->

The direct-rover and corrected-base routes were tested on the same field
hardware:

- Direct CORS to rover USB, with the moving-base writer disabled, did not
  recover in about 226 seconds. NAV-PVT fixed was `0/8` in the sampled window,
  median `hAcc` was `462 mm`, RELPOSNED valid/fixed was `0/8`, and flags stayed
  at `3` with zero baseline and heading.
- CORS to the moving base only produced NAV-PVT fixed `18/18`, median
  `hAcc=27.5 mm`, RELPOSNED valid/fixed `18/18`, and approximately `0.417 deg`
  heading accuracy.
- An independent poll of the corrected moving base reported carrier fixed,
  `hAcc=36 mm`, and `vAcc=54 mm`; a later rover sample reached `hAcc=15 mm`
  and `vAcc=22 mm`.
- After negative one-port tests, one warm-start recovery followed by a normal
  launch restored NAV-PVT `flags=131` with `hAcc=37..43 mm` and RELPOSNED
  `flags=311` in every sampled epoch. The observed baseline was about `1.178 m`
  and heading accuracy was about `0.489 deg`.

A later default-route acceptance sample also confirmed:

- the isolated NTRIP topic had one publisher and one subscriber, owned by
  `moving_base_rtcm_writer`;
- the heading rover reported NAV-PVT `fix_type=3`, `flags=131`,
  `h_acc=41 mm`, and `v_acc=63 mm`;
- NAV-PVT carrier status was fixed because `(flags & 0xC0) == 0x80`;
- NAV-RELPOSNED reported decimal flags `311` (`0x137`), a 117 cm physical
  baseline, and approximately 0.49168 degrees of heading accuracy.

These measurements verify that absolute RTK and moving-baseline heading remain
valid together when CORS corrections are applied to the moving base first.

Do not hard-code the historical 1.178 m baseline. Compare the live
`rel_pos_length` with the currently measured antenna spacing and require it to
remain stable.

## Rejected Alternatives

<!-- HH_260722 - Record the station-ID experiments that must not return to production. -->

- Rewriting CORS station ID `0` to `42` preserved absolute fixed
  (`hAcc=14 mm`) but produced RELPOSNED flags `279` and an approximately 11 km
  vector to the CORS station instead of the antenna baseline.
- Keeping CORS at `0` and changing moving-base `CFG-RTCM-DF003_OUT` to station
  `1` in RAM repeated the failure (`hAcc=17 mm`, flags `279`, approximately
  11 km).
- The temporary station-ID rewrite executable was removed. The moving-base RAM
  station ID was restored to `0` and verified with VALGET.

Changing station IDs separated message labels but did not create two
independent RTK engines on the tested receiver and firmware.

## Configuration And Acceptance

<!-- HH_260722 - Record the active caster profile and RTK acceptance contract. -->

- `CNJU-RTCM32` remains the active mountpoint. The package-owned GNSS YAML files
  under `camrod_sensing` and their `camrod_bringup` deployment mirrors are
  byte-identical.
- `fix_type=3` or NavSatFix status alone does not prove RTK Fixed. NAV-PVT must
  satisfy `(flags & 0xC0) == 0x80`.
- Heading acceptance requires moving-baseline, relative-position-valid,
  heading-valid, and fixed-carrier RELPOSNED bits. The validated receiver
  normally reported decimal flags `311` (`0x137`).
- The isolated NTRIP topic must show one publisher and one subscriber, the
  moving-base writer. A direct-rover diagnostic must not run at the same time.
- If a diagnostic leaves the rover tracking the wrong reference, enable warm
  start for exactly one recovery launch, then restore the default `false`.

## Verification

<!-- HH_260722 - Record reproducible build, test, syntax, and configuration checks. -->

- `colcon build --packages-select camrod_sensing camrod_bringup
  --symlink-install --cmake-force-configure` completed successfully.
- `colcon test --packages-select camrod_sensing camrod_bringup` reported
  39 tests, 0 errors, 0 failures, and 0 skipped.
- The focused GNSS routing and bringup synchronization suites reported
  12 passed tests.
- Python byte-compilation, shell syntax, YAML parsing, configuration-mirror
  comparison, launch-argument inspection, and `git diff --check` passed.
- `field_test_tool.sh config` reported `config sync OK`.

## Known Limits

<!-- HH_260722 - Keep unverified single-logical-port alternatives outside the production claim. -->

- The validated wiring requires two logical serial devices: `/dev/ttyUSB0` for
  corrections into the moving base and `/dev/ttyACM0` for rover output.
- A USB hub may reduce host cabling, but it still exposes two logical ports; a
  single logical serial stream was not validated for simultaneous correction
  input and rover navigation output in this hardware layout.
- The PC can use only `/dev/ttyACM0` when an independent Wi-Fi, LTE, or radio
  device supplies NTRIP directly to the Lite moving base.
- RTK fix quality still depends on caster connectivity, correction age,
  antenna visibility, baseline geometry, and the stored receiver port settings.
