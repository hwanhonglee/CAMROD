# camrod_voice

<!-- HH_260804 - Replace package-tree and release-history prose with the
event pipeline, priorities, readiness contract, and measurable limits. -->

Runtime-event adapter and priority-queued playback of pre-recorded Korean WAV
announcements.

![Voice event and priority](../docs/assets/module-guides/voice/voice-events-and-priority.png)

## Actual Simulation Runtime

![Live voice event adapter](../docs/assets/module-guides/voice/runtime-event-terminal-20260804.png)

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
| `0` | Info | Ordinary informational cue |
| `1` | Notice | Startup, ready, mission, arrival, charging, low battery |
| `2` | Warning | Obstacle hold, critical battery |
| `3` | Critical | Estop/release; may interrupt when requested |

## Event Mapping

| Runtime event | Audio key | Priority |
|---|---|---:|
| Startup delay expires | `system.startup` | 1 |
| First complete readiness | `system.ready` | 1 |
| Engaged route to site | `navigation.to_campsite` | 1 |
| Engaged return route | `navigation.return_to_dropzone` | 1 |
| Site/manual goal reached | `navigation.arrived_campsite` | 1 |
| Engaged cost/route hold | `safety.obstacle` | 2 |
| Estop asserted / released | `safety.estop` / `safety.estop_released` | 3 |
| Battery `<= 20%` | `battery.low` | 1 |
| Battery `<= 10%` | `battery.critical` | 2 |
| Charging starts | `battery.charging` | 1 |

`WAIT_DZ` intentionally has no navigation announcement. Generic planning
recovery does not trigger obstacle speech; the final command gate's actual
cost/route hold does.

## Active Values

| Item | Value |
|---|---:|
| Startup delay | `3.0 s` |
| Readiness check | `0.5 s` |
| Required readiness modules | 7 (`map`, `sensing`, `localization`, `planning`, `control`, `platform`, `system`) |
| Required frame | `map -> robot_center_link` |
| Maximum ready localization mode | `NORMAL (0)` |
| Low / critical battery cue | `20 / 10%` |

`system.ready` is announced once after required modules are non-error, Nav2 is
available, planning is idle, localization is normal, TF exists, the gate is
standby/charging, platform estop is released, and engage is false.

## Topics

| Direction | Topic | Purpose |
|---|---|---|
| Publish | `/voice/voice_announcer/say` | Audio request queue input |
| Publish | `/voice/voice_announcer/state` | Playback state |
| Subscribe | `/platform/status` | Estop, SOC, charging |
| Subscribe | `/system/status` | Module readiness/health |
| Subscribe | `/localization/mode` | Localization admission |
| Subscribe | `/control/cmd_vel_safety_gate/status` | Actual obstacle/safety state |
| Subscribe | `/control/planning_engaged` | Manual-or-mission engage |
| Action check | `/planning/navigate_to_pose` | Nav2 readiness |

## Build And Run

```bash
cd ~/camrod_ws
colcon build --packages-select camrod_voice --symlink-install
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
