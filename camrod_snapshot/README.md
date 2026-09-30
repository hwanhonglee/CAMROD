# camrod_snapshot

Solution for [this rosbag2 issue](https://github.com/ros2/rosbag2/issues/663) which acts similarly to [`rosbag_snapshot`](https://github.com/ros/rosbag_snapshot). It is added as a new package here rather than patching `rosbag2` because that's how they did it in ROS 1 ;).

It subscribes to topics and maintains a buffer of recent messages like a dash cam. This is useful in live testing where unexpected events can occur which would be useful to have data on but the opportunity is missed if `rosbag record` was not running (disk space limits make always running `rosbag record` impracticable). Instead, users may run snapshot in the background and save data from the recent past to disk as needed.


## Usage

`camrod_snapshot` can be configured through ROS params for more granular control. The `snapshotter` command will run the buffer server and the `snapshotter_client` command can be used as a client to request that the server write data to disk or freeze the buffer to preserve interesting data until a user can decide what to write.

### Server

```bash
$ ros2 run camrod_snapshot snapshotter
```

Buffer recent messages until triggered to write or trigger an already running instance.

Full bringup owns the production topic list at
`camrod_bringup/config/snapshot/camrod_topics.params.yaml`. With the workspace's
`--symlink-install`, edits to that source YAML are visible from `install/`
without rebuilding. Restart the snapshotter to reload YAML parameters; topics
added from Robot UI are applied immediately for the current process.

### Example param file
```yaml
/**:
  ros__parameters:
    default_duration_limit: 10.0           # [Optional, default=-1] Maximum time difference between newest and oldest message in seconds
    bagfile_split_duration_s: 60           # [Optional, default=60] Maximum duration of each DB file in one bag
    default_memory_limit: 64.0             # [Optional, default=-1] Maximum memory used by messages in each topic's buffer, in MB
    default_min_interval: 0.2              # [Optional, default=0] Minimum seconds between stored messages per topic (throttling). 0 stores every message.
    topics: ["/topic1", "/topic2"]         # [Optional] List of topics to buffer. If empty, buffer all topics.
    topic_details:
      /topic1:
        type: "sensor_msgs/msg/NavSatFix"  # [Required if topic is specified] Topic type
      /topic2:
        type: "sensor_msgs/msg/Odometry"
        duration: 15.0                     # [Optional] Override duration limit, inherit memory limit
      /topic3:
        type: "sensor_msgs/msg/Image"
        duration: 2.0                      # [Optional] Override both limits
        memory: -1                         # Negative value means no limit
        interval: 0.0                      # [Optional] Override min interval; 0 disables throttling for this topic
```

### Client

###### Write all buffered data to `<datetime>.bag`
`ros2 run camrod_snapshot snapshotter_client --ros-params -p action_type:=trigger_write`

###### Write buffered data from selected topics to `new_lighting<datetime>.bag`
`ros2 run camrod_snapshot snapshotter_client --ros-params -p filename:=new_lighting -p topics:=["/camera/image_raw", "/camera/camera_info"]`

###### Write all buffered data to `/home/user/crashed_into_wall.bag`
`ros2 run camrod_snapshot snapshotter_client --ros-params -p filename:="/home/user/crashed_into_wall.bag"`

###### Pause buffering of new data, holding current buffer in memory until resumed or write is triggered
`ros2 run camrod_snapshot snapshotter_client --ros-params -p action_type:=pause`

###### Resume buffering new data
`ros2 run camrod_snapshot snapshotter_client --ros-params -p action_type:=resume`

###### Call trigger service manually

```
$ ros2 service call /trigger_snapshot avg_msgs/srv/TriggerSnapshot "{filename: '', topics: [], start_time: {sec: 0, nanosec: 0}, stop_time: {sec: 0, nanosec: 0}}"
requester: making request: avg_msgs.srv.TriggerSnapshot_Request(filename='', topics=[], start_time=builtin_interfaces.msg.Time(sec=0, nanosec=0), stop_time=builtin_interfaces.msg.Time(sec=0, nanosec=0))

response:
avg_msgs.srv.TriggerSnapshot_Response(success=True, message='')
```

###### Call pause/resume service manually

```
$ ros2 service call /enable_snapshot std_srvs/srv/SetBool "{data: false}"
requester: making request: std_srvs.srv.SetBool_Request(data=False)

response:
std_srvs.srv.SetBool_Response(success=True, message='')
```

###### Add or remove a runtime topic

The topic must currently exist in the ROS graph so the snapshotter can discover
its message type. YAML-owned base topics cannot be removed at runtime.

```bash
ros2 service call /configure_snapshot_topics \
  avg_msgs/srv/ConfigureSnapshotTopics \
  "{add_topics: ['/sensing/radar/right1/range'], remove_topics: []}"
```

### Automatic capture

The snapshotter can write a bag by itself when a watched status topic reports
trouble, so evidence survives a fault that nobody was at the UI for. The node
stays domain-agnostic: a rule names a topic and the typed `operating_state` /
`level` values worth a bag, and knows nothing about what produced them. Rules
never parse the free-text `message` field, whose format may change.

Automatic and manual snapshots share one write path, so they share the same
pause/estimate/disk-reserve behaviour and cannot overlap. The write runs on its
own thread, so `trigger_snapshot` and `estimate_snapshot` stay answerable while
a capture is in progress.

A snapshot that is **rejected before the bag is opened** — no buffered data, a
byte budget too small for even the newest message, or an unavailable filesystem
reserve — leaves the buffer untouched. Those checks take as long as a stat of
the buffer and the disk, so the history is still continuous, and discarding it
would cost the operator exactly what the snapshot was meant to preserve. Only a
write that actually opened the bag clears the buffer afterwards, because the
pause then lasted long enough to drop messages and leave a real gap.

Guards keep a capture rare, because **writing a snapshot pauses recording and
clears every buffer afterwards** — an unthrottled trigger would spend the
history on the first event of a cascade and record nothing for the rest:

- `require_healthy_first` — the rule stays disarmed until its topic reports
  healthy for `require_healthy_s`. Boot ordering reaches ERROR *before* it
  reaches OK, while a real fault can only follow an OK, so this excludes
  startup by the shape of the transition instead of by guessing boot duration.
- `hold_s` — the condition must persist, so one dropped heartbeat costs nothing.
- `min_occurrences` + `scope_*` — fire on the Nth rising edge, counted only
  while the scope topic says the rule is live, and bounded by an episode.
- `cooldown_s` — at most one capture per window, across all rules. `0.0`
  inherits `default_duration_limit`, i.e. one full buffer length.

A rule re-arms only on a falling edge, so one continuous fault writes one bag
rather than one per cooldown window.

Only one capture is taken per cooldown window, so **the order of `rules` is
priority**: the first rule in the list that has tripped wins the window, and the
others are dropped with a log line saying so. Put the most severe rule first.

**Pick the debounce that matches the shape of the event.** A condition that
persists until someone intervenes is waited out with `hold_s`. A condition that
its own recovery clears in under a second — a margin-boundary contact, where the
robot crabs back toward the lane centre — never persists, so `hold_s` would
never be reached; count it instead. Counting needs a scope, supplied by a second
topic:

- `scope_active_states` — count only while the scope topic reports one of
  these. Empty means always. This is what keeps a rule from counting events
  that are expected in some phases: a robot inside a campsite or the charger
  bay is deliberately outside the road lanelets, and `motion_cost_stop` already
  grants those maneuver phases an explicit lanelet bypass, so a boundary
  contact there is intended behaviour rather than evidence.
- `scope_reset_states` — clear the count on every *entry* into one of these.
  Required when `min_occurrences > 1`, or the total would accumulate across
  unrelated missions until it tripped on events that had nothing to do with
  each other.

Together they express "twice while driving the road legs of one delivery"
without this package knowing what a delivery is. Rising edges are counted in
the subscription callback rather than on the evaluation tick, so a contact
shorter than the tick period is still counted.

```yaml
/**:
  ros__parameters:
    auto_trigger:
      enabled: true
      output_directory: "/home/nvidia/storage/camrod"  # [Required when enabled]
      filename_prefix: "autosnapshot"                # <prefix>_<rule>_<datetime>.bag
      cooldown_s: 0.0                                # 0 = default_duration_limit
      startup_grace_s: 90.0                          # secondary startup guard
      lookback_s: 0.0                                # 0 = whole buffer
      minimum_free_space_mb: 5120                    # mirrors the operator UI policy
      minimum_free_space_ratio: 0.10
      size_safety_factor: 1.30
      rules: ["system_error", "gate_fault_hold", "route_boundary_repeat_contact"]
      rule:
        # Sustained condition -> wait it out.
        system_error:
          topic: "/system/status"
          kind: "system_status"            # module_state | system_status | service_state
          on_system_not_ok: true           # system_status only
          min_level: 2                     # ERROR; <0 disables
          # modules: ["localization"]      # system_status only, optional
          hold_s: 10.0
          require_healthy_first: true
          require_healthy_s: 15.0

        # Hardware fault -> one is enough, and it is never scoped away.
        gate_fault_hold:
          topic: "/control/cmd_vel_safety_gate/status"
          kind: "module_state"
          operating_states: ["FAULT_HOLD"]
          hold_s: 2.0
          require_healthy_first: true
          require_healthy_s: 5.0

        # Self-clearing condition -> count it, only on the road legs, and
        # only within one delivery.
        route_boundary_repeat_contact:
          topic: "/control/cmd_vel_safety_gate/status"
          kind: "module_state"
          operating_states: ["ROUTE_SAFETY_HOLD"]
          hold_s: 0.0
          min_occurrences: 2
          scope_topic: "/service/state"
          scope_kind: "service_state"
          scope_active_states: ["MOVING_TO_SITE", "RETURNING_TO_DROP_ZONE"]
          scope_reset_states: ["MOVING_TO_SITE"]
          require_healthy_first: true
          require_healthy_s: 5.0
```

`kind: service_state` matches `AvgServiceState.state_name`; the other two match
`ModuleState.operating_state` / `level`.

A malformed rule throws at startup rather than silently never firing. Captures
are announced on `/rosout` before recording pauses, so the reason is inside the
bag that gets written.


### Offloading finished bags

A finished bag can be moved to shared storage automatically. Both the trigger
service and an automatic capture end in the same write path, so an operator
snapshot from the UI and an auto-trigger snapshot are transferred alike.

The transfer runs on its own thread. The service response returns as soon as
the bag is on local disk — the operator is not made to wait for the network —
and a slow link never delays the next capture. **A bag is removed locally only
after a verified transfer**, so a failed one always leaves the only copy in
place; failures are retried and then logged, never silently dropped.

```yaml
/**:
  ros__parameters:
    offload:
      enabled: true
      host: "220.90.18.50"
      port: 8008
      user: "admin"
      remote_directory: "/volume1/home/admin/camrod/storage"
      identity_file: ""                   # empty = agent or default key
      remove_local_after_transfer: true   # false turns the move into a copy
      connect_timeout_s: 10
      transfer_timeout_s: 1800
      retries: 2
      retry_delay_s: 30
```

**Key-based SSH is required.** The transfer runs with `BatchMode=yes` and never
prompts, so an unattended robot fails loudly and keeps its bag instead of
blocking forever on a password. Install the key once, as the user the
snapshotter runs as:

```bash
ssh-copy-id -p 8008 admin@220.90.18.50
```

A rosbag2 bag is a directory of metadata plus database files, so the transfer
uses `rsync -a --partial` over `ssh` and the remote directory is created with
`mkdir -p` first. Commands are executed directly through `execvp` rather than a
shell, so a bag path can never be re-parsed as a command.

Note that a moved bag no longer exists at the path the UI reported when it was
written; the transfer is announced on `/rosout` with both the local path and the
remote destination.
