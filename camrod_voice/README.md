# camrod_voice

<!-- HH_260804 - Replace package-tree and release-history prose with the
event pipeline, priorities, readiness contract, and measurable limits. -->

Runtime-event adapter and priority-queued playback of pre-recorded Korean WAV
announcements.

![Voice event and priority](../docs/assets/module-guides/voice/guide/voice-events-and-priority.png)

## Actual Simulation Runtime

![Live voice event adapter](../docs/assets/module-guides/voice/evidence/runtime-capture-20260804/runtime-event-terminal-20260804.png)

`SIM RUNTIME CAPTURE`: a live adapter emitted `system.startup` as an
`avg_msgs/AudioRequest`, and ROS graph inspection shows its real subscriptions
and output topic. Speaker playback and outdoor audibility remain field-pending.

## At A Glance

| Uses | Function | Main outputs |
|---|---|---|
| Planning, platform, localization, gate, system, TF, and Nav2 action state | Maps state edges to stable audio keys | `avg_msgs/AudioRequest` |
| C++ priority queue + SDL2_mixer | Selects, interrupts, and plays WAV assets | `avg_msgs/VoiceState` and speaker audio |
| `resource/audio/ko-KR/` | Resolves `category.file_name` keys | Deterministic packaged prompts |

Voice is operator feedback only. Disabling it does not remove any safety or
motion condition.

## Priority Policy

| Priority | Meaning | Example |
|---:|---|---|
| `0` | Info | Repeated travel reminders under the music bed |
| `1` | Notice | Startup, ready, mission, arrival, resume, charging, low battery |
| `2` | Warning | Obstacle hold |
| `3` | Critical | Estop/release; may interrupt when requested |

## Event Mapping

| Runtime event | Audio key | Priority |
|---|---|---:|
| Startup delay expires | `system.startup` | 1 |
| First complete readiness | `system.ready` | 1 |
| Announcer node shuts down | `system.shutdown` | — |
| Engaged departure to site | `navigation.to_campsite` | 1 |
| Engaged departure to drop zone | `navigation.to_dropzone` | 1 |
| Recall return enters `RECALL_CLEARANCE_WAIT` (B1–B10) | `navigation.recall_clear_site` | 2 |
| Trip to site under way, every period | `system.announce1` + `system.announce2` | 0 |
| Trip to drop zone under way, every period | `navigation.return_to_dropzone` | 0 |
| Site/manual goal reached | `navigation.arrived_campsite` | 1 |
| Engaged cost/route hold | `safety.obstacle` | 2 |
| Hold still blocking, every period | `navigation.please_step_aside` | 1 |
| Announced hold clears | `safety.thankyou` | 1 |
| Selected AprilTag docking starts | `docking.started` | 1 |
| Current AprilTag docking attempt reports `PARKED` | `docking.succeeded` | 1 |
| Current AprilTag docking attempt reports `ERROR` | `docking.failed` | 2 |
| Ordinary reverse parking starts/completes/fails | No docking cue; no verified ordinary-parking asset is packaged | — |
| Estop asserted / released | `safety.estop` / `safety.estop_released` | 3 |
| Battery `<= 20%` | `battery.low` | 1 |
| Charging starts | `battery.charging` | 1 |
| Charging and `>= 99%`, every period | `battery.full` | 1 |

`WAIT_DZ` intentionally has no navigation announcement. Generic planning
recovery does not trigger obstacle speech; the final command gate's actual
cost/route hold does.

<!-- HH_260907 - PARKED is shared by reverse parking and charging docking; do
not describe a reverse-distance-limit completion as successful docking. -->
Parking voice follows the selected `parking_method` and `attempt` in the
authoritative `/parking/status` message, not the common phase name alone.
Only `apriltag` emits docking cues. `reverse`, an unknown method, or a terminal
status without a matching started attempt cannot announce docking success.
Repeated status messages and older attempts do not replay cues; dispatcher
cancellation clears the run. A verified `PARKED` after a docking `ERROR` may
announce recovery once. In legacy modes the controller topic identifies the
method (AprilTag also has an explicit module name); once dispatcher status is
received, legacy status cannot overwrite that authoritative selection.

There is currently no verified ordinary-parking completion WAV/transcript.
Reverse parking therefore stays silent for these three docking announcements;
no differently worded clip is reused and no missing `parking.*` key is emitted.
Battery/charging, safety, navigation, and recall announcements remain separate.

The first roadside confirmation at B1–B10 triggers a stationary clearance phase
before the robot re-enters the site to turn around. The controller's authoritative
`ModuleState.operating_state` edge, rather than a change of the shared service
ID, emits `navigation.recall_clear_site` once: “해당 캠핑 사이트를 잠시 비워 주세요.
로봇이 들어가 방향을 바꾼 뒤 복귀합니다.” The packaged clip is 6.552 seconds,
within the controller's default 8-second clearance pause. Repeated controller
status messages do not replay it. Ordinary departure, arrival, travel reminders,
and BGM stay suppressed through the post-turn `RECALL_RETURN_WAIT`: the robot
remains stopped for loading and requires the second explicit
`recall_final_return` confirmation before exiting. The clearance recording
does not authorize that departure. These travel cues resume only after the
turnaround/exit reaches `DONE`; cancellation (`IDLE`) or
`ERROR` resets the clearance cue for a deliberate retry. B11–B13 do not enter
this phase and keep their existing opposite-direction exit announcements.

This is a fixed pre-motion pause, not an acknowledgement that physical speaker
playback finished. The controller's pose, obstacle, and site-clearance checks
remain responsible for authorizing movement. The cue's source text, voice, and
generation settings are recorded beside the audio asset.

A trip is one latched identity — travel context, mission key, and goal source —
held from the departure cue until arrival, a mission change, or disengage. It
deliberately survives a `WARN_RECOVERY` tick, because a single low-rate sensor
sample flips that state while the robot keeps driving the same route: without
the latch every blip replayed the departure cue, restarted the bed, and reset
the reminder schedule before it could come due.

## Background Music

`system.bgm` is a looping bed on the SDL2_mixer music stream while speech plays
on a reserved chunk channel, so the two sound together. The adapter latches
`voice_announcer/bgm` for the whole trip; the announcer starts the bed only
once the queue is idle, which puts it after the departure cue, and ducks it for
every cue that follows. A safety hold keeps the bed running — arrival, ending
the trip, or disengaging releases it.

`system.shutdown` is not an adapter event. The announcer plays it from its own
destructor after `SIGINT`, bounded by `shutdown_timeout_s` so it stays inside
the launch `SIGTERM` window.

## Repeated Cues

Three cues repeat instead of firing once, all polled at `1 Hz` and all ended by
the condition itself rather than a counter:

| Cue | Runs while | Ends on |
|---|---|---|
| `system.announce1` + `announce2` / `return_to_dropzone` | Trip under way | Arrival or disengage |
| `navigation.please_step_aside` | Announced hold still blocking | Hold clears (then `safety.thankyou`) |
| `battery.full` | Charging and `>= battery_full_threshold` | Charger removed or level drops |

## Active Values

| Item | Value |
|---|---:|
| Startup delay | `3.0 s` |
| Readiness check | `0.5 s` |
| Required readiness modules | 7 (`map`, `sensing`, `localization`, `planning`, `control`, `platform`, `system`) |
| Required frame | `map -> robot_center_link` |
| Maximum ready localization mode | `NORMAL (0)` |
| Low battery cue | `20%` |
| First travel reminder / repeat period | `30 s` / `90 s` |
| Blocked-route explanation period | `20 s` |
| Charge-complete cue period | `60 s` |
| Bed level, ducked level | `0.55` / `0.12` |
| Shutdown cue budget | `4.5 s` |

`system.ready` is announced once after required modules are non-error, Nav2 is
available, planning is idle, localization is normal, TF exists, the gate is
standby/charging, platform estop is released, and engage is false.

That strict idle snapshot controls only the one-shot `system.ready` cue. After
`system.startup`, mission, obstacle, and docking cues follow their direct
planning, engage, command-gate, and parking-phase evidence, so beginning a
service while the gate is leaving an idle hold cannot mute voice for the rest of
the process. Every emitted `AudioRequest` is logged at INFO; the announcer's
`Playing [...]` line is the separate downstream playback-stage evidence.

## Topics

| Direction | Topic | Purpose |
|---|---|---|
| Publish | `/voice/voice_announcer/say` | Audio request queue input |
| Publish | `/voice/voice_announcer/bgm` | Latched music-bed request |
| Publish | `/voice/voice_announcer/state` | Playback state |
| Subscribe | `/platform/status` | Estop, SOC, charging |
| Subscribe | `/system/status` | Module readiness/health |
| Subscribe | `/localization/mode` | Localization admission |
| Subscribe | `/control/cmd_vel_safety_gate/status` | Actual obstacle/safety state |
| Subscribe | `/control/planning_engaged` | Manual-or-mission engage |
| Subscribe | `/control/camping_site_maneuver_controller/status` | Recall clearance and turnaround phase |
| Subscribe | `/parking/status`, legacy `/parking/*_parking_controller/status` | Selected method, attempt, and phase for actual docking cues |
| Action check | `/planning/navigate_to_pose` | Nav2 readiness |

## Build And Run

<!-- HH_260825 - Route package builds through the workspace wrapper so the
source checkout never accumulates local build/install/log directories. -->

```bash
cd ~/camrod_ws/src
./colcon_build.sh --packages-select camrod_voice
cd ~/camrod_ws
source install/setup.bash

ros2 launch camrod_voice voice.launch.py
ros2 launch camrod_voice voice.launch.py enable_voice_adapter:=false
```

System dependencies are `libsdl2-dev` and `libsdl2-mixer-dev`.

## Bluetooth Provisioning

`setup_bt_audio.sh` installs an opt-in reconnect service for an already paired
amplifier. It is not run by ROS bringup.

```bash
bluetoothctl pair AA:BB:CC:DD:EE:FF
bluetoothctl trust AA:BB:CC:DD:EE:FF
sudo ./setup_bt_audio.sh AA:BB:CC:DD:EE:FF "CamrodAmp"
```

The helper validates the target and refuses to overwrite conflicting existing
service files. Speaker loudness, Bluetooth delay, and outdoor audibility remain
physical acceptance items; no acoustic latency is claimed from simulation.
