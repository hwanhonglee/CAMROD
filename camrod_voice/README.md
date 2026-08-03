# camrod_voice

CAMROD voice announcement module. It plays pre-recorded WAV files through a priority queue and converts CAMROD runtime events into `avg_msgs/AudioRequest` messages.

> HH_260617: This README reflects the implemented runtime files. The old TODO-only description was removed because `voice.launch.py`, `voice_event_adapter.yaml`, and `voice_event_adapter_node.py` now exist.

## Package Structure

```text
camrod_voice/
├── CMakeLists.txt
├── package.xml
├── setup_bt_audio.sh
├── config/
│   └── voice_event_adapter.yaml
├── include/voice_announcer/
│   ├── audio_player.hpp
│   ├── priority_queue.hpp
│   └── voice_announcer_node.hpp
├── launch/
│   └── voice.launch.py
├── resource/audio/ko-KR/
│   ├── battery/
│   ├── navigation/
│   ├── safety/
│   └── system/
└── src/
    ├── audio_player.cpp
    ├── main.cpp
    ├── priority_queue.cpp
    ├── voice_announcer_node.cpp
    └── voice_event_adapter_node.py
```

## Nodes

| Node | Type | Purpose |
|------|------|---------|
| `/voice/voice_announcer` | C++ | Plays `AudioRequest.key` WAV files with priority handling |
| `/voice/voice_event_adapter` | Python | Converts planning/platform/battery/charging events to audio requests |

## Topics

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribe | `/voice/voice_announcer/say` | `avg_msgs/AudioRequest` | Audio request input |
| Publish | `/voice/voice_announcer/state` | `avg_msgs/VoiceState` | Playback state |
| Subscribe | `/planning/state_machine/state` | `avg_msgs/PlanningState` | Navigation state changes |
| Subscribe | `/platform/status` | `avg_msgs/AvgPlatformStatus` | E-stop, normalized battery, and charging edges |
| Subscribe | `/system/status` | `avg_msgs/SystemStatus` | Required module health and startup state |
| Subscribe | `/localization/mode` | `avg_msgs/AvgLocalizationMode` | Localization admission state |
| Subscribe | `/control/cmd_vel_safety_gate/status` | `avg_msgs/ModuleState` | Final command-gate health and operating state |
| Subscribe | `/control/planning_engaged` | `avg_msgs/AvgBool` | Unified manual-or-mission engage state |
| Subscribe | `/tf`, `/tf_static` | TF | `map` to `robot_center_link` availability |
| Action check | `/planning/navigate_to_pose` | `nav2_msgs/NavigateToPose` | Goal action-server availability |

## Audio Keys

`AudioRequest.key` format is `category.file_name`, resolved as:

```text
resource/audio/{locale}/{category}/{file_name}.wav
```

Example: `navigation.arrived_campsite` resolves to `resource/audio/ko-KR/navigation/arrived_campsite.wav`.

Priority convention: `0=info`, `1=notice`, `2=warning`, `3=critical`. Critical requests may interrupt the current queue when `interrupt=true`.

## Event Mapping

| Event | Audio key | Priority |
|-------|-----------|----------|
| Startup timer | `system.startup` | 1 |
| First complete readiness transition | `system.ready` | 1 |
| Engaged, gate-enabled `RUNNING` site/manual route | `navigation.to_campsite` | 1 |
| Engaged, gate-enabled released drop-zone route | `navigation.return_to_dropzone` | 1 |
| `GOAL_REACHED` after an announced site/manual route | `navigation.arrived_campsite` | 1 |
| Engaged control gate enters cost/route-safety hold | `safety.obstacle` | 2 |
| E-stop `false -> true` | `safety.estop` | 3 |
| E-stop `true -> false` | `safety.estop_released` | 3 |
| Battery `<= 0.20` | `battery.low` | 1 |
| Battery `<= 0.10` | `battery.critical` | 2 |
| Charging `false -> true` | `battery.charging` | 1 |

`WAIT_DZ` is a stopped drop-zone wait state and intentionally has no navigation
audio. A generic planning `WARN_RECOVERY` also has no obstacle audio because it
can represent localization, graph, or lifecycle health; obstacle speech is tied
to the final control gate's cost/route hold instead.

## Readiness contract

`system.ready` is announced once per adapter process, after `system.startup`,
only when all of these conditions are simultaneously true:

1. `/system/status` contains every configured required module (`map`, `sensing`,
   `localization`, `planning`, `control`, `platform`, and `system` by default),
   no module is health level `ERROR`, and none reports a startup/fault/inactive
   state. An explicit `WARN`, including intentional `DUMMY DATA`, remains
   degraded-but-operational and is still shown by system diagnostics. Required
   graph gaps report `STARTING` during startup grace and `FAULT/ERROR`
   afterward, so missing planner, controller, behavior, or navigator servers
   block readiness. The
   `/planning/navigate_to_pose` action server must also be discoverable.
2. Planning has published an idle semantic state: `READY` or `WAIT_DZ`.
3. `/localization/mode` is `NORMAL` and TF `map -> robot_center_link` is available.
4. The final command gate reports no `ERROR` and is `STANDBY` or `CHARGING`.
5. `/platform/status` has been received with e-stop released and
   `error_code == 0`.
6. `/control/planning_engaged` has been received and is false.

<!-- HH_260804 - Link the shared readiness frame and UI state evidence. -->
Voice and both UIs now use the same `map -> robot_center_link` readiness target;
the coordinate migration is documented in
[the frame ledger](../docs/V2_1_3_ROBOT_CENTER_MIGRATION.md), while service-state
and safety-overlay behavior is captured in
[the UI/recovery report](../docs/V2_1_3_BOUNDARY_RECOVERY_VALIDATION.md).

During a mission, movement audio additionally waits for unified engage to be
true and for the final command gate to report `ENABLED` (or
`DEPARTING_CHARGER`). Losing and reacquiring localization does not repeat the
one-shot ready announcement. An explicit disengage/re-engage may repeat the
movement announcement once, after the gate enables again.

## Launch

```bash
ros2 launch camrod_voice voice.launch.py
ros2 launch camrod_voice voice.launch.py enable_voice_adapter:=false
ros2 launch camrod_voice voice.launch.py voice_namespace:=voice locale:=ko-KR
```

## Build

```bash
./colcon_build.sh --packages-select camrod_voice
```

System dependencies: `libsdl2-dev`, `libsdl2-mixer-dev`. `setup_camrod.sh` installs them when apt is available. On machines without `SDL2_mixer`, `colcon_build.sh` skips `camrod_voice` so other packages can still build.

## Bluetooth amplifier provisioning

<!-- HH_260728 - Document the opt-in system service helper and its safety boundary. -->
`setup_bt_audio.sh` registers one already-paired Bluetooth audio amplifier as a
systemd reconnect service. It is an explicit administrator action and is not
started by ROS bringup:

```bash
# Pair and trust the exact target first.
bluetoothctl pair AA:BB:CC:DD:EE:FF
bluetoothctl trust AA:BB:CC:DD:EE:FF
sudo ./setup_bt_audio.sh AA:BB:CC:DD:EE:FF "CamrodAmp"
```

<!-- HH_260728 - Keep provisioning prerequisites and no-overwrite behavior
     synchronized with the guarded helper. -->
The MAC address, device name, backend commands, and `Paired: yes` state are
validated before any system file is written. PipeWire requires both `pw-cli`
and the actually used routing command `wpctl`; PulseAudio requires `pactl`, and
BlueALSA requires `bluealsa-aplay`. An identical rerun is idempotent. If the
per-MAC connection script or systemd unit already exists with different
content, the helper stops without overwriting either target; review and back up
the existing file before explicitly removing it for reprovisioning.

The installed copy is available under
`share/camrod_voice/scripts/setup_bt_audio.sh`.

## 2026-06-17 Runtime Update

> HH_260617: Voice event adaptation consumes semantic planning and charging states.

<!-- HH_260720 - Voice consumes generated planning and unified platform contracts only. -->
`voice_event_adapter_node.py` listens to `avg_msgs/PlanningState` and
`avg_msgs/AvgPlatformStatus`. `setup_camrod.sh` installs `libsdl2-mixer-dev`; if
that package is missing, `colcon_build.sh` skips `camrod_voice` on development
PCs instead of blocking the motion stack build.

## 2026-07-02 Runtime Update

> HH_260702: Voice is operator feedback only and must not be treated as safety authority.

The voice adapter follows `/planning/state_machine/state` and the e-stop,
battery, and charging fields on `/platform/status`. It announces events after
the control gate has made the motion decision; disabling voice does not remove
any control condition.

```bash
ros2 launch camrod_voice voice.launch.py enable_voice_adapter:=false
```
