# Mission, CAN and distance records

The service-metrics database and mission journal are independent. The former
keeps aggregate service distance and delivery/Recall/return/unknown breakdowns;
the latter keeps each admitted mission's event timeline, stop reasons, decoded
platform/CAN telemetry and, when explicitly enabled, receive-only raw CAN
frames. Neither recorder grants motion authority.

## Files and source identity

- Service totals: `${XDG_STATE_HOME:-~/.local/state}/camrod/service_metrics.sqlite3`.
- Mission index: `${XDG_STATE_HOME:-~/.local/state}/camrod/mission_records/mission_journal.sqlite3`.
- Per-mission event/telemetry JSONL files: dated subdirectories below the mission
  records root. The `MissionRecords` panel reads the bounded
  `/api/mission-records` snapshot, rather than browsing arbitrary files.
- Real Ranger's `/platform/status` contains CAN-decoded platform values such as
  control mode, vehicle/motor speed, steering and BMS data. The mission recorder
  subscribes independently of the browser, so closing the UI does not stop it.
- Raw CAN ID and byte payload files require an explicit SocketCAN interface.
  The default is disabled; this does **not** disable decoded platform telemetry.
- CARLA records carry `environment=simulation` and are not physical CAN logs.
  The adapter passes an isolated `mission_records_root` and forces raw CAN off.

The UI shows a stale/unavailable recorder as an error rather than a plausible
zero. The default JSONL quota is 256 MiB; reaching it preserves existing files
and reports degraded capture. Set `mission_recorder_quota_bytes` after checking
available disk space and expected mission duration.

## Real robot launch

`ui.launch.py` enables the passive mission recorder by default. To include
receive-only physical frames, first verify the actual interface and that a
short capture is arriving, then launch with
`mission_recorder_raw_can_interface:=can0`. The deployed Ranger driver currently
uses `can0`, but the robot's interface and access rights must be verified on
that robot. Never assign `can0` to a CARLA run.

Mission records are indexed by date, sequence, site and mission intent. The
timeline includes accepted mission start, Recall and final Return boundaries,
service phases, control mode transitions, stop reasons and platform samples.
If the platform topic or disk becomes unavailable, the record is marked
incomplete rather than backfilled from fake measurements.

## Protect older field data

Schema v2 adds delivery/Recall/return/unknown segments to service metrics.
Historical rows whose leg cannot be proven remain `unknown`; the lifetime total
retains their original distance. Before running a new deployment against a
field robot's database, stop the old writer and make a copy-only verified
migration to a **new** directory:

```bash
python3 -m camrod_ui.service_metrics_migration \
  --source /path/to/robot/service_metrics.sqlite3 \
  --output-dir /path/to/new/private/migration-20261002
```

Review the generated report and typed-row hashes before switching the service
to the migrated copy. Do not point CARLA at the robot's database or infer old
Recall distances from site names. This workstation's existing service history
was not modified during the restoration.
