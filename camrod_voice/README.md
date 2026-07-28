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
| Startup timer | `system.startup` | 0 |
| State `READY` | `system.ready` | 1 |
| State `GOAL_REACHED` | `navigation.arrived_campsite` | 1 |
| State `RETURNING` or `WAIT_DZ` | `navigation.return_to_dropzone` | 1 |
| State `WARN_RECOVERY` | `safety.obstacle` | 2 |
| E-stop `false -> true` | `safety.estop` | 3 |
| E-stop `true -> false` | `safety.estop_released` | 3 |
| Battery `<= 0.20` | `battery.low` | 1 |
| Battery `<= 0.10` | `battery.critical` | 2 |
| Charging `false -> true` | `battery.charging` | 1 |

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
sudo ./setup_bt_audio.sh AA:BB:CC:DD:EE:FF "CamrodAmp"
```

The MAC address and device name are validated before any system file is
written. The installed copy is available under
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
