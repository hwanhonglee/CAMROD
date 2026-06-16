#!/usr/bin/env python3
"""CAMROD 시스템 이벤트 → AudioRequest 변환 노드.

구독하는 시스템 이벤트를 음성 키로 변환하여 voice_announcer/say 토픽에 발행한다.
모든 전환은 엣지-트리거(edge-triggered) 방식으로 동작한다.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from avg_msgs.msg import AudioRequest


# 상태머신 상태 → 음성 키 매핑 (엣지 트리거)
_STATE_AUDIO = {
    'READY':        ('system.ready',                   1),
    'GOAL_REACHED': ('navigation.arrived_campsite',    1),
    'RETURNING':    ('navigation.return_to_dropzone',  1),
    'WAIT_DZ':      ('navigation.return_to_dropzone',  1),
    'WARN_RECOVERY':('safety.obstacle',                2),
}

# WAIT_DZ → RUNNING 전용 매핑 (undocking 감지)
_UNDOCKING_FROM = 'WAIT_DZ'
_UNDOCKING_KEY  = ('undocking.started', 1)


class VoiceEventAdapterNode(Node):

    def __init__(self):
        super().__init__('voice_event_adapter')

        self.declare_parameter('startup_delay_s',          3.0)
        self.declare_parameter('enable_nav_audio',         True)
        self.declare_parameter('enable_estop_audio',       True)
        self.declare_parameter('enable_battery_audio',     True)
        self.declare_parameter('enable_docking_audio',     True)
        self.declare_parameter('battery_low_threshold',    20)
        self.declare_parameter('battery_critical_threshold', 10)

        p = self.get_parameter
        self._en_nav      = p('enable_nav_audio').value
        self._en_estop    = p('enable_estop_audio').value
        self._en_battery  = p('enable_battery_audio').value
        self._en_docking  = p('enable_docking_audio').value
        self._bat_low     = p('battery_low_threshold').value
        self._bat_crit    = p('battery_critical_threshold').value
        startup_delay     = p('startup_delay_s').value

        # 이전 상태 추적
        self._prev_state    = ''
        self._prev_estop    = None   # None = 초기 미수신
        self._prev_charging = None
        self._bat_low_fired = False
        self._bat_crit_fired = False

        # 발행: voice_announcer/say → /voice/voice_announcer/say
        self._say_pub = self.create_publisher(AudioRequest, 'voice_announcer/say', 10)

        # 구독
        self.create_subscription(
            String, '/planning/state_machine/state',
            self._on_state, 10)
        self.create_subscription(
            Bool, '/platform/status/estop',
            self._on_estop, 10)
        self.create_subscription(
            Int32, '/battery_percentage',
            self._on_battery, 10)
        self.create_subscription(
            Bool, '/docking/is_charging',
            self._on_charging, 10)

        # 시작 음성 (딜레이 후 1회)
        self._startup_timer = self.create_timer(
            startup_delay, self._on_startup_timer)

        self.get_logger().info(
            f'VoiceEventAdapter started '
            f'(nav={self._en_nav}, estop={self._en_estop}, '
            f'battery={self._en_battery}, docking={self._en_docking})')

    # ── 타이머 콜백 ──────────────────────────────────────────────────────────

    def _on_startup_timer(self):
        self._startup_timer.cancel()
        self._say('system.startup', priority=0)

    # ── 구독 콜백 ────────────────────────────────────────────────────────────

    def _on_state(self, msg: String):
        if not self._en_nav:
            return
        state = msg.data
        prev  = self._prev_state
        self._prev_state = state

        if state == prev:
            return

        # WAIT_DZ → RUNNING: undocking 감지
        if state == 'RUNNING' and prev == _UNDOCKING_FROM:
            key, pri = _UNDOCKING_KEY
            self._say(key, priority=pri)
            return

        if state in _STATE_AUDIO:
            key, pri = _STATE_AUDIO[state]
            self._say(key, priority=pri)

    def _on_estop(self, msg: Bool):
        if not self._en_estop:
            return
        engaged = msg.data
        if self._prev_estop is None:
            self._prev_estop = engaged
            return
        if engaged and not self._prev_estop:
            self._say('safety.estop', priority=3, interrupt=True)
        elif not engaged and self._prev_estop:
            self._say('safety.estop_released', priority=3)
        self._prev_estop = engaged

    def _on_battery(self, msg: Int32):
        if not self._en_battery:
            return
        pct = msg.data
        if not self._bat_crit_fired and pct <= self._bat_crit:
            self._bat_crit_fired = True
            self._say('battery.critical', priority=2)
        elif not self._bat_low_fired and pct <= self._bat_low:
            self._bat_low_fired = True
            self._say('battery.low', priority=1)
        # 충전 시 플래그 리셋
        if pct > self._bat_low:
            self._bat_low_fired  = False
            self._bat_crit_fired = False

    def _on_charging(self, msg: Bool):
        if not self._en_docking:
            return
        charging = msg.data
        if self._prev_charging is None:
            self._prev_charging = charging
            return
        if charging and not self._prev_charging:
            self._say('docking.succeeded', priority=1)
        self._prev_charging = charging

    # ── 발행 헬퍼 ────────────────────────────────────────────────────────────

    def _say(self, key: str, *, priority: int = 1, interrupt: bool = False):
        req = AudioRequest()
        req.key       = key
        req.priority  = priority
        req.interrupt = interrupt
        req.locale    = ''          # node default 사용
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
