# Replacement dual-GNSS 10 Hz validation — 2026-08-19

<!-- HH_260819 - Preserve the physical A/B evidence and replacement checklist.
ROS cadence, host-side NTRIP forwarding, rover RTCM reception, and vehicle
heading are separate acceptance gates. -->

## Deployment under test

- Heading rover: `/dev/ttyACM0`
- CORS-to-Lite FTDI: `DN05Y9E7`, resolved to `/dev/ttyUSB0`
- Rover measurement rate: `10 Hz`, `nav_rate: 1`
- CORS-to-Lite writer baud: `460800`
- Normal route: NTRIP -> FTDI -> Lite -> board link -> rover UART2

The canonical GNSS YAML owns `rate` and `nav_rate`. The dual launch overlay must
not redefine them. The driver writes the YAML-derived 100 ms measurement period
to rover RAM, while u-center remains the owner of Lite processing/output and the
Lite-to-rover physical serial configuration.

## Test A — normal moving-base cascade

The production route ran for 120.1 seconds after confirming the effective ROS
parameters and the writer's open file descriptor.

| Observation | Result |
|---|---:|
| NAV-PVT | 1201 messages, `10.002 Hz` |
| NavSatFix | 1199 messages, `10.000 Hz` |
| NAV-RELPOSNED | 1199 messages, `10.000 Hz` |
| NAV-PVT iTOW delta | median `100 ms`, range `99-101 ms` |
| Incoming NTRIP | 783 frames, 202503 bytes, zero malformed frames |
| FTDI writer | `DN05Y9E7` open at `460800` |
| Rover RXM-RTCM | **0 messages** |
| NAV-PVT carrier | none for 1201/1201 samples |
| NAV-RELPOSNED | flags `3`, baseline `0`, relative/heading/moving invalid |

The incoming CORS stream contained the expected station, MSM, and 1230 traffic.
This passes the ROS 10 Hz, caster, NTRIP client, writer callback, FTDI identity,
and host serial-open gates. It fails after the host writer: no RTCM reached the
rover decoder through the production moving-base route.

A one-shot GNSS-engine warm start and production-route reapplication did not
change the result. The failure is not a stale rover tracking state.

## Test B — direct CORS-to-rover USB isolation

For diagnosis only, the same NTRIP stream bypassed the Lite and entered the
rover over USB for 120.1 seconds.

| Observation | Result |
|---|---:|
| Incoming NTRIP / rover RXM-RTCM | 778 / 776 frames |
| RTCM CRC failures | `0` |
| Message-type delivery | Nearly 1:1 for 1006, 1008, 1013, 1033, 1046, 1075, 1085, 1095, 1115, 1125, 1230 |
| NAV-PVT | 1201 messages, `10.000 Hz`, iTOW `100 ms` |
| NAV-PVT solution | RTK Float for 1201/1201 samples |
| Horizontal accuracy | min/median/max `22/29/46 mm` |
| NAV-RELPOSNED | 1200 messages, `9.999 Hz`, flags `271` (`0x10f`) |
| Reported baseline | median `9793.5968 m` |

The direct test proves that the rover USB RTCM input, RTCM parser, receiver
correction engine, and GNSS antenna reception are functional. RTK Fixed was not
reached during this two-minute window, but RTK Float with centimetre-scale
reported accuracy was reached consistently.

The direct-mode RELPOS heading is **not vehicle heading**. Flags `0x10f` include
relative-position and heading-valid bits but not `is_moving`; the roughly 9.8 km
baseline is the CORS-reference-to-rover vector. Never feed this heading into
localization or accept it as a dual-antenna result.

## Test C — Lite VALGET and baud repair

The next bench check queried the Lite itself with UBX-CFG-VALGET. Both Lite
serial ports were still configured for `115200`, despite the two external
endpoints already using `460800`.

| Link | Transmitting side | Receiving side | Finding |
|---|---:|---:|---|
| Host FTDI writer -> Lite UART1 | `460800` | `115200` | Baud mismatch |
| Lite UART2 -> heading-rover UART2 | `115200` | `460800` | Baud mismatch |

This was a double mismatch, not a single failed link. In particular, seeing the
FTDI file descriptor open at `460800` in Test A proved only the host-side serial
setting; it did not prove that Lite UART1 was configured to receive at that
rate.

Lite UART1 and UART2 were both changed to `460800` in the RAM and FLASH
configuration layers, and the receiver acknowledged the writes. Rover
RXM-RTCM reception recovered immediately after the two link speeds matched,
without replacing either GNSS receiver or changing the inter-board wiring.
This directly identifies the baud mismatch as the cause of the Test A failure.

## Test D — 30-second NTRIP-off isolation after repair

With the repaired moving-base cascade already operating, host NTRIP forwarding
was disabled for 30 seconds. NAV-RELPOSNED continued at 10 Hz and all `300/300`
samples reported flags `311` (`0x137`). This short isolation verifies that the
Lite-to-rover moving-base stream and vehicle-heading solution continue to work
after the CORS input is removed. It is not a long-duration CORS-outage or
absolute-position retention claim.

## Test E — uninterrupted wired-NAT64 full bringup

The complete system was then run uninterrupted for 120 seconds with NTRIP over
the wired NAT64 path. No GNSS-only restart or direct-USB RTCM bypass was used.
The original NTRIP client forced an IPv4 socket and failed with
`ENETUNREACH` because this wired route exposes the caster through DNS64/NAT64.
The client now uses the address-family-neutral connection helper, allowing the
resolver to select IPv4 or IPv6; a focused regression test locks this behavior.

| Observation | Result |
|---|---:|
| Expected 10 Hz epochs | approximately `1200` |
| NAV-PVT | 1197 messages, `9.994 Hz`; max callback gap `382.8 ms`; max iTOW gap `400 ms` |
| NavSatFix | 1194 messages, `9.974 Hz`; max callback gap `382.3 ms` |
| NAV-RELPOSNED | 1194 messages, `9.973 Hz`; max callback gap `386.5 ms` |
| RXM-RTCM continuity | max callback gap `192.7 ms` |
| NAV-PVT RTK Fixed | `895/1197` (`74.770%`) |
| NAV-RELPOSNED flags `311` | `892/1194` (`74.707%`) |

The full stack therefore sustained approximately 10 Hz PVT, fix, RELPOSNED,
and rover RTCM reception, and it produced valid moving-baseline vehicle heading.
The counts and maximum gaps also show that this run was not completely
lossless: PVT was three epochs below 1200, fix and RELPOSNED were six epochs
below 1200, and isolated callback/iTOW gaps reached roughly 0.2--0.4 seconds.
The aggregate Fixed and flags-311 ratios include the acquisition/convergence
portion of the run and are not sufficient by themselves to pass strict Fixed
retention.

### Post-convergence exact-iTOW join

A separate, continuous 60.1-second stabilized window was evaluated by exact
iTOW rather than callback arrival order.

| Observation | Result |
|---|---:|
| Unique NAV-PVT epochs | `600`, all RTK Fixed |
| NAV-RELPOSNED epochs | `599` |
| Exact-iTOW joined epochs | `599` |
| PVT Fixed and RELPOSNED flags `311` together | `598/599` (`99.833%`) |
| Same joint condition, final 30 seconds | `298/299` (`99.666%`) |
| Exceptional joined epoch | flags `19`, once |
| Valid flags-311 baseline, n=598 | min/median/max/last `0.8690/0.9021/0.9286/0.9029 m` |

After convergence, Fixed plus valid vehicle heading was therefore nearly
continuous and the measured baseline remained around 0.9 m. It was still not
perfectly lossless or 100%: one PVT iTOW had no RELPOSNED match within the
sampled window, and one joined RELPOSNED epoch reported flags `19` instead of
`311`.

## Conclusion

No receiver or inter-board hardware failure was observed in these tests. The
observed outage was caused by two independent baud mismatches, and matching
Lite UART1 and UART2 to the external `460800` endpoints restored rover
RTCM reception immediately.

The replacement configuration passes the functional **10 Hz ROS output** and
**dual-antenna vehicle-heading** gates. It also demonstrates RTK Fixed
acquisition and nearly continuous Fixed-plus-heading operation after
convergence. **Final 10-minute Fixed-retention acceptance remains pending**:
the uninterrupted 120-second window was Fixed for only `895/1197` PVT samples,
and the stabilized 60-second window still contained an unmatched PVT iTOW and
one flags-`19` exception. Do not describe this result as completely lossless or
100% retained.

## Configuration ownership and future-change checklist

1. Rediscover `/dev/serial/by-id` and update both mirrored
   `zed_f9p_rover.yaml` files. Never depend on `/dev/ttyUSB*` numbering.
2. Keep the receiver epoch in the canonical YAML only. Verify effective
   `rate=10.0`, NAV-PVT/fix near 10 Hz, and 100 ms iTOW increments.
3. Keep the host writer on FTDI serial `DN05Y9E7` at `460800`. This setting owns
   only the host-to-Lite endpoint; it does not configure the Lite receiver.
4. Keep Lite UART1 and UART2 at `460800`. When a Lite board is replaced or its
   configuration is reset, use VALGET to verify the effective values and the
   persistent layer; write both RAM and FLASH and require ACKs before testing.
5. Keep rover UART2 at `460800` with RTCM3 input enabled. ROS deliberately
   disables rover UART1 protocols, so the board link must terminate on UART2.
6. Treat host writer -> Lite UART1 and Lite UART2 -> rover UART2 as independent
   links. If any baud is changed, update and verify both endpoints of the
   affected link rather than assuming one `460800` setting covers the cascade.
7. Require non-empty rover RXM-RTCM before evaluating RTK or heading. Final
   vehicle-heading acceptance requires the documented PVT Fixed and RELPOSNED
   moving-heading Fixed bitmasks (normally decimal `131` and `311`) and a
   baseline consistent with the measured antenna separation.
8. For a strict Fixed-retention pass, evaluate an agreed drive-duration window
   by exact iTOW and set explicit limits for missing epochs, non-Fixed epochs,
   non-mask-valid heading epochs, and maximum gaps. The present evidence passes
   function but not a 100%-retention criterion.
