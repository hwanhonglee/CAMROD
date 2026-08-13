#!/usr/bin/env python3
"""CAMROD 시스템 이벤트 → AudioRequest 변환 노드.

구독하는 시스템 이벤트를 음성 키로 변환하여 voice_announcer/say 토픽에 발행한다.
모든 전환은 엣지-트리거(edge-triggered) 방식으로 동작한다.
"""

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from avg_msgs.msg import (
    AudioRequest,
    AvgBool,
    AvgLocalizationMode,
    AvgPlatformStatus,
    ModuleState,
    PlanningState,
    SystemStatus,
)

from voice_event_policy import VoiceEvent, VoiceEventPolicy


class VoiceEventAdapterNode(Node):

    # Travel speech is one feature: the departure cue, the reminders repeated
    # over the music bed, and the obstacle/resume pair all follow enable_nav_audio.
    _NAV_EVENT_PREFIXES = ('navigation.', 'system.announce')
    _NAV_EVENT_KEYS = frozenset({'safety.obstacle', 'safety.thankyou'})
    # Repeat scheduling is polled; every period itself is a parameter.
    _REPEAT_TICK_S = 1.0

    def __init__(self):
        super().__init__('voice_event_adapter')

        self.declare_parameter('startup_delay_s',          3.0)
        self.declare_parameter('enable_nav_audio',         True)
        self.declare_parameter('enable_estop_audio',       True)
        self.declare_parameter('enable_battery_audio',     True)
        # HH_260813 - Drop-zone docking start and outcome.
        self.declare_parameter('enable_docking_audio',     True)
        # HH_260720 - Announce the canonical platform charging state directly.
        self.declare_parameter('enable_charging_audio',    True)
        # HH_260812 - Music bed and repeated travel reminders.
        self.declare_parameter('enable_bgm',               True)
        self.declare_parameter('enable_travel_announce',   True)
        self.declare_parameter('travel_announce_initial_delay_s', 30.0)
        self.declare_parameter('travel_announce_period_s', 90.0)
        # HH_260812 - Standing explanation while a hold keeps blocking the route.
        self.declare_parameter('obstacle_repeat_period_s', 20.0)
        # AvgPlatformStatus battery percentage is normalized to 0.0..1.0.
        self.declare_parameter('battery_low_threshold',    0.20)
        # HH_260812 - Repeat the charge-complete cue until the charger is removed.
        self.declare_parameter('battery_full_threshold',   0.99)
        self.declare_parameter('battery_full_repeat_period_s', 60.0)
        self.declare_parameter('platform_status_topic', '/platform/status')
        self.declare_parameter('system_status_topic', '/system/status')
        self.declare_parameter('localization_mode_topic', '/localization/mode')
        self.declare_parameter(
            'control_gate_status_topic',
            '/control/cmd_vel_safety_gate/status')
        self.declare_parameter(
            'planning_engaged_topic',
            '/control/planning_engaged')
        self.declare_parameter(
            'navigate_to_pose_action',
            '/planning/navigate_to_pose')
        # HH_260813 - Only the selected parking controller publishes, so both
        # candidates can be subscribed unconditionally.
        self.declare_parameter(
            'parking_status_topics',
            ['/parking/reverse_parking_controller/status',
             '/parking/apriltag_parking_controller/status'])
        self.declare_parameter(
            'readiness_required_modules',
            ['map', 'sensing', 'localization', 'planning',
             'control', 'platform', 'system'])
        self.declare_parameter('readiness_map_frame', 'map')
        # HH_260803 - Voice readiness follows the axle-midpoint navigation frame.
        self.declare_parameter('readiness_base_frame', 'robot_center_link')
        self.declare_parameter('readiness_check_period_s', 0.5)
        self.declare_parameter('max_ready_localization_mode', 0)
        self.declare_parameter('return_mission_key', 'drop_zone')

        p = self.get_parameter
        self._en_nav = p('enable_nav_audio').value
        self._en_estop = p('enable_estop_audio').value
        self._en_battery = p('enable_battery_audio').value
        self._en_docking = bool(p('enable_docking_audio').value)
        self._en_charging = p('enable_charging_audio').value
        self._en_bgm = bool(p('enable_bgm').value)
        self._en_travel_announce = bool(p('enable_travel_announce').value)
        self._announce_initial_delay = max(
            0.0, float(p('travel_announce_initial_delay_s').value))
        self._announce_period = max(
            5.0, float(p('travel_announce_period_s').value))
        self._obstacle_repeat_period = max(
            5.0, float(p('obstacle_repeat_period_s').value))
        self._bat_low = p('battery_low_threshold').value
        self._bat_full = float(p('battery_full_threshold').value)
        self._bat_full_period = max(
            5.0, float(p('battery_full_repeat_period_s').value))
        startup_delay = p('startup_delay_s').value
        platform_status_topic = str(p('platform_status_topic').value)
        system_status_topic = str(p('system_status_topic').value)
        localization_mode_topic = str(p('localization_mode_topic').value)
        control_gate_status_topic = str(p('control_gate_status_topic').value)
        planning_engaged_topic = str(p('planning_engaged_topic').value)
        navigate_to_pose_action = str(p('navigate_to_pose_action').value)
        parking_status_topics = [
            str(topic).strip()
            for topic in (p('parking_status_topics').value or [])
            if str(topic).strip()
        ]
        self._readiness_map_frame = str(p('readiness_map_frame').value)
        self._readiness_base_frame = str(p('readiness_base_frame').value)
        readiness_check_period_s = max(
            0.1, float(p('readiness_check_period_s').value))
        required_modules = [
            str(name) for name in p('readiness_required_modules').value
        ]

        # 이전 상태 추적
        self._prev_estop = None   # None = 초기 미수신
        self._prev_charging = None
        self._bat_low_fired = False
        self._last_readiness_reasons = None
        self._travel_active = False
        self._announce_elapsed = 0.0
        self._announce_due_after = self._announce_initial_delay
        self._obstacle_repeat_elapsed = 0.0
        self._charging = False
        self._battery_pct = None
        self._bat_full_fired = False
        self._bat_full_elapsed = 0.0

        self._policy = VoiceEventPolicy(
            required_modules,
            return_mission_key=str(p('return_mission_key').value),
            max_ready_localization_mode=int(
                p('max_ready_localization_mode').value),
        )

        # 발행: voice_announcer/say → /voice/voice_announcer/say
        self._say_pub = self.create_publisher(
            AudioRequest, 'voice_announcer/say', 10)
        # Latched so a restarted announcer picks up an in-progress trip.
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._bgm_pub = self.create_publisher(
            AvgBool, 'voice_announcer/bgm', latched_qos)
        self._publish_bgm(False)

        # 구독
        # Planning state is avg_msgs/PlanningState, not a raw string.
        self.create_subscription(
            PlanningState, '/planning/state_machine/state',
            self._on_state, 10)
        # Consume safety and battery data from the canonical platform status.
        self.create_subscription(
            AvgPlatformStatus, platform_status_topic,
            self._on_platform_status, 10)
        self.create_subscription(
            SystemStatus, system_status_topic,
            self._on_system_status, 10)
        self.create_subscription(
            AvgLocalizationMode, localization_mode_topic,
            self._on_localization_mode, 10)
        self.create_subscription(
            ModuleState, control_gate_status_topic,
            self._on_control_gate_status, latched_qos)
        self.create_subscription(
            AvgBool, planning_engaged_topic,
            self._on_planning_engaged, latched_qos)
        for topic in parking_status_topics:
            self.create_subscription(
                ModuleState, topic, self._on_parking_status, 10)

        # 시작 음성 (딜레이 후 1회)
        self._startup_timer = self.create_timer(
            max(0.1, float(startup_delay)), self._on_startup_timer)
        self._repeat_timer = self.create_timer(
            self._REPEAT_TICK_S, self._on_repeat_timer)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self._navigate_action_client = ActionClient(
            self, NavigateToPose, navigate_to_pose_action)
        self._readiness_timer = self.create_timer(
            readiness_check_period_s, self._on_readiness_timer)

        self.get_logger().info(
            f'VoiceEventAdapter started '
            f'(nav={self._en_nav}, estop={self._en_estop}, '
            f'battery={self._en_battery}, charging={self._en_charging}, '
            f'docking={self._en_docking}, bgm={self._en_bgm}, '
            f'announce={self._en_travel_announce}@{self._announce_period:.0f}s, '
            f'readiness_modules={",".join(required_modules)}, '
            f'tf={self._readiness_map_frame}<-{self._readiness_base_frame})')

    # ── 타이머 콜백 ──────────────────────────────────────────────────────────

    def _on_startup_timer(self):
        self._startup_timer.cancel()
        self._emit_policy_events(self._policy.announce_startup())

    def _on_readiness_timer(self):
        try:
            tf_ready = self._tf_buffer.can_transform(
                self._readiness_map_frame,
                self._readiness_base_frame,
                Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f'readiness TF check failed: {exc}')
            tf_ready = False
        self._emit_policy_events(self._policy.update_tf(bool(tf_ready)))
        self._emit_policy_events(
            self._policy.update_action_server(
                self._navigate_action_client.server_is_ready()))

    def _on_repeat_timer(self):
        self._tick_travel_announce()
        self._tick_obstacle_repeat()
        self._tick_battery_full()

    def _tick_travel_announce(self):
        if not self._en_travel_announce or not self._policy.travel_active:
            return
        if self._policy.obstacle_hold_announced:
            # Standing still behind an obstacle is not "on the way" — hold the
            # schedule so the blocked-route explanation is the only reminder.
            return
        self._announce_elapsed += self._REPEAT_TICK_S
        if self._announce_elapsed < self._announce_due_after:
            return
        self._announce_elapsed = 0.0
        self._announce_due_after = self._announce_period
        self._emit_policy_events(self._policy.travel_announce_events())

    def _tick_obstacle_repeat(self):
        # safety.obstacle already fired on the hold edge; this is the standing
        # explanation for whoever is waiting while the route stays blocked.
        if not self._policy.obstacle_hold_announced:
            self._obstacle_repeat_elapsed = 0.0
            return
        self._obstacle_repeat_elapsed += self._REPEAT_TICK_S
        if self._obstacle_repeat_elapsed < self._obstacle_repeat_period:
            return
        self._obstacle_repeat_elapsed = 0.0
        self._emit_policy_events(self._policy.obstacle_repeat_events())

    def _tick_battery_full(self):
        full = (
            self._en_battery
            and self._charging
            and self._battery_pct is not None
            and self._battery_pct >= self._bat_full
        )
        if not full:
            # Unplugging (or draining below full) ends the reminder.
            self._bat_full_fired = False
            self._bat_full_elapsed = 0.0
            return
        if not self._bat_full_fired:
            self._bat_full_fired = True
            self._bat_full_elapsed = 0.0
            self._say('battery.full', priority=1)
            return
        self._bat_full_elapsed += self._REPEAT_TICK_S
        if self._bat_full_elapsed < self._bat_full_period:
            return
        self._bat_full_elapsed = 0.0
        self._say('battery.full', priority=1)

    # ── 구독 콜백 ────────────────────────────────────────────────────────────

    def _on_state(self, msg: PlanningState):
        self._emit_policy_events(
            self._policy.update_planning(
                state=msg.label,
                scenario=msg.scenario_label,
                active_mission_key=msg.active_mission_key,
                active_goal_source=msg.active_goal_source,
                return_requested=msg.return_requested,
            )
        )

    def _on_system_status(self, msg: SystemStatus):
        modules = {
            str(module.module_name): (
                int(module.level),
                str(module.operating_state),
            )
            for module in msg.modules
        }
        self._emit_policy_events(self._policy.update_system(modules))

    def _on_localization_mode(self, msg: AvgLocalizationMode):
        self._emit_policy_events(
            self._policy.update_localization(int(msg.value)))

    def _on_control_gate_status(self, msg: ModuleState):
        self._emit_policy_events(
            self._policy.update_gate(
                level=int(msg.level),
                operating_state=msg.operating_state,
                message=msg.message,
            )
        )

    def _on_planning_engaged(self, msg: AvgBool):
        self._emit_policy_events(self._policy.update_engaged(bool(msg.data)))

    def _on_parking_status(self, msg: ModuleState):
        if not self._en_docking:
            return
        self._emit_policy_events(
            self._policy.update_docking(msg.operating_state))

    def _on_platform_status(self, msg: AvgPlatformStatus):
        self._on_estop(bool(msg.estop))
        # Charge level and charger state feed the repeated full cue regardless
        # of whether the one-shot edges below are enabled.
        self._charging = bool(msg.is_charging)
        self._battery_pct = (
            float(msg.battery_percentage) if msg.battery_state_available
            else None
        )
        if msg.battery_state_available:
            self._on_battery(float(msg.battery_percentage))
        self._on_charging(bool(msg.is_charging))
        self._emit_policy_events(
            self._policy.update_platform(
                estop=bool(msg.estop),
                error_code=int(msg.error_code),
            )
        )

    def _on_estop(self, engaged: bool):
        if not self._en_estop:
            return
        if self._prev_estop is None:
            self._prev_estop = engaged
            return
        if engaged and not self._prev_estop:
            self._say('safety.estop', priority=3, interrupt=True)
        elif not engaged and self._prev_estop:
            self._say('safety.estop_released', priority=3)
        self._prev_estop = engaged

    def _on_battery(self, pct: float):
        if not self._en_battery:
            return
        # HH_260812 - One low-battery notice only; the critical cue is retired
        # because it interrupted mission speech without changing any action.
        if not self._bat_low_fired and pct <= self._bat_low:
            self._bat_low_fired = True
            self._say('battery.low', priority=1)
        # 충전 시 플래그 리셋
        if pct > self._bat_low:
            self._bat_low_fired = False

    def _on_charging(self, charging: bool):
        if not self._en_charging:
            return
        if self._prev_charging is None:
            self._prev_charging = charging
            return
        if charging and not self._prev_charging:
            # Charging is reported directly by the platform/BMS state.
            self._say('battery.charging', priority=1)
        self._prev_charging = charging

    # ── 발행 헬퍼 ────────────────────────────────────────────────────────────

    def _emit_policy_events(self, events):
        reasons = self._policy.readiness_reasons()
        if reasons != self._last_readiness_reasons:
            self._last_readiness_reasons = reasons
            if reasons:
                self.get_logger().debug(
                    f'voice readiness pending: {", ".join(reasons)}')
            else:
                self.get_logger().info(
                    'voice readiness complete: system/modules, planning/Nav2, '
                    'localization/TF, control gate, and platform are ready')
        for event in events:
            if not isinstance(event, VoiceEvent):
                continue
            if not self._en_nav and self._is_travel_event(event.key):
                continue
            self._say(
                event.key,
                priority=event.priority,
                interrupt=event.interrupt)
        self._sync_travel_state()

    def _is_travel_event(self, key: str) -> bool:
        return (
            key.startswith(self._NAV_EVENT_PREFIXES)
            or key in self._NAV_EVENT_KEYS
        )

    def _sync_travel_state(self):
        """Publish music-bed edges and restart the reminder schedule."""

        active = bool(
            self._en_nav and self._en_bgm and self._policy.travel_active)
        if active == self._travel_active:
            return
        self._travel_active = active
        self._announce_elapsed = 0.0
        self._announce_due_after = self._announce_initial_delay
        self._publish_bgm(active)
        self.get_logger().info(
            f'travel bgm {"start" if active else "stop"} '
            f'(context={self._policy.travel_context() or "none"})')

    def _publish_bgm(self, active: bool):
        msg = AvgBool()
        msg.data = bool(active)
        self._bgm_pub.publish(msg)

    def _say(self, key: str, *, priority: int = 1, interrupt: bool = False):
        req = AudioRequest()
        req.key = key
        req.priority = priority
        req.interrupt = interrupt
        req.locale = ''          # node default 사용
        self._say_pub.publish(req)
        self.get_logger().debug(f'say → {key} (pri={priority})')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceEventAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
