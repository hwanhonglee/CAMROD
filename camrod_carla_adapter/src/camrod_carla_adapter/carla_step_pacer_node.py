"""Fail-closed wall-time pacing for the synchronous CARLA ROS bridge.

The upstream bridge owns ``world.tick()`` and its Ranger lifecycle lock.  This
node therefore paces the bridge only through its public ``/carla/control``
contract: acquire PAUSE, request exactly one STEP_ONCE, wait for the matching
``/carla/status`` frame and PAUSE acknowledgement, then wait one wall-clock
period before requesting the next frame.

The state machine is deliberately ROS-independent.  It uses monotonic wall
time because ROS time stops while CARLA is paused and because scheduling from
the previous acknowledgement prevents catch-up bursts after a slow frame.
"""

from dataclasses import dataclass
from enum import Enum
import fcntl
import math
import os
from pathlib import Path
from threading import RLock
import time


UINT64_MAX = (1 << 64) - 1


class PacerCommand(Enum):
    """Bridge commands emitted by :class:`StepPacerStateMachine`."""

    PLAY = 'play'
    PAUSE = 'pause'
    STEP_ONCE = 'step_once'


class PacerPhase(Enum):
    """Externally reportable lifecycle phases for one pacer owner."""

    ACQUIRING_PAUSE = 'acquiring_pause'
    WAITING_FOR_DEADLINE = 'waiting_for_deadline'
    WAITING_FOR_STEP_ACK = 'waiting_for_step_ack'
    FAULTED = 'faulted'
    RELEASED = 'released'


@dataclass(frozen=True)
class StepPacerConfig:
    """Fail-closed timing and CARLA settings contract."""

    step_period_s: float = 0.05
    expected_fixed_delta_seconds: float = 0.05
    fixed_delta_tolerance_s: float = 0.0001
    pause_retry_s: float = 0.10
    startup_timeout_s: float = 10.0
    status_timeout_s: float = 0.50
    # Managed Ranger authorization takes roughly two seconds.  On the first
    # full-sensor frame the upstream bridge then waits up to one second for
    # each of 13 newly registered CARLA sensors, serially.  This longer bound
    # applies only while one explicit STEP_ONCE is outstanding; ordinary
    # paused-status freshness remains the stricter status_timeout_s contract.
    step_ack_timeout_s: float = 20.0


@dataclass(frozen=True)
class PacerHealth:
    """One active health-service decision."""

    success: bool
    message: str


def _finite_positive(value, name):
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f'{name} must be finite and positive') from error
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return numeric


def validate_step_pacer_config(config):
    """Normalize a pacing contract and reject non-real-time combinations."""
    checked = StepPacerConfig(
        step_period_s=_finite_positive(
            config.step_period_s, 'step_period_s'),
        expected_fixed_delta_seconds=_finite_positive(
            config.expected_fixed_delta_seconds,
            'expected_fixed_delta_seconds',
        ),
        fixed_delta_tolerance_s=_finite_positive(
            config.fixed_delta_tolerance_s,
            'fixed_delta_tolerance_s',
        ),
        pause_retry_s=_finite_positive(
            config.pause_retry_s, 'pause_retry_s'),
        startup_timeout_s=_finite_positive(
            config.startup_timeout_s, 'startup_timeout_s'),
        status_timeout_s=_finite_positive(
            config.status_timeout_s, 'status_timeout_s'),
        step_ack_timeout_s=_finite_positive(
            config.step_ack_timeout_s, 'step_ack_timeout_s'),
    )
    if checked.fixed_delta_tolerance_s >= (
        checked.expected_fixed_delta_seconds
    ):
        raise ValueError(
            'fixed_delta_tolerance_s must be smaller than '
            'expected_fixed_delta_seconds'
        )
    if abs(
        checked.step_period_s - checked.expected_fixed_delta_seconds
    ) > checked.fixed_delta_tolerance_s:
        raise ValueError(
            'step_period_s must match expected_fixed_delta_seconds within '
            'fixed_delta_tolerance_s for real-time pacing'
        )
    if checked.pause_retry_s >= checked.startup_timeout_s:
        raise ValueError('pause_retry_s must be smaller than startup_timeout_s')
    if checked.status_timeout_s <= checked.step_period_s:
        raise ValueError('status_timeout_s must be greater than step_period_s')
    return checked


def _valid_monotonic(value):
    try:
        numeric = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(numeric) or numeric < 0.0:
        return None
    return numeric


def _valid_frame(value):
    if isinstance(value, bool):
        return None
    try:
        numeric = int(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if numeric < 0 or numeric > UINT64_MAX:
        return None
    try:
        if float(value) != numeric:
            return None
    except (TypeError, ValueError, OverflowError):
        return None
    return numeric


class StepPacerStateMachine:
    """Verify and pace one synchronous bridge without calling CARLA APIs."""

    def __init__(self, config=StepPacerConfig(), now_s=0.0):
        self.config = validate_step_pacer_config(config)
        started = _valid_monotonic(now_s)
        if started is None:
            raise ValueError('now_s must be finite and non-negative')
        self.phase = PacerPhase.ACQUIRING_PAUSE
        self.started_s = started
        self.last_status_s = None
        self.last_pause_command_s = None
        self.step_command_s = None
        self.next_step_deadline_s = None
        self.frame = None
        self.expected_frame = None
        self.last_synchronous_mode = None
        self.last_synchronous_mode_running = None
        self.last_fixed_delta_seconds = None
        self.fault_reason = ''

    @property
    def target_rate_hz(self):
        """Return the configured wall-time frame rate."""
        return 1.0 / self.config.step_period_s

    @property
    def ready(self):
        """Report healthy pacing ownership after PAUSE acquisition."""
        return self.phase in {
            PacerPhase.WAITING_FOR_DEADLINE,
            PacerPhase.WAITING_FOR_STEP_ACK,
        }

    def _pause_command(self, now_s, force=False):
        if (
            force
            or self.last_pause_command_s is None
            or now_s - self.last_pause_command_s >= self.config.pause_retry_s
        ):
            self.last_pause_command_s = now_s
            return (PacerCommand.PAUSE,)
        return ()

    def _fault(self, reason, now_s):
        if self.phase != PacerPhase.RELEASED:
            self.phase = PacerPhase.FAULTED
            if not self.fault_reason:
                self.fault_reason = str(reason)
            self.next_step_deadline_s = None
            self.step_command_s = None
            self.expected_frame = None
            return self._pause_command(now_s, force=True)
        return ()

    def observe_status(
        self,
        *,
        frame,
        synchronous_mode,
        synchronous_mode_running,
        fixed_delta_seconds,
        now_s,
    ):
        """Consume one ordered ``CarlaStatus`` and return control commands."""
        now = _valid_monotonic(now_s)
        if now is None:
            fallback = self.last_status_s or self.started_s
            return self._fault('invalid monotonic status timestamp', fallback)
        if self.phase == PacerPhase.RELEASED:
            return ()

        parsed_frame = _valid_frame(frame)
        try:
            delta = float(fixed_delta_seconds)
        except (TypeError, ValueError, OverflowError):
            delta = math.nan
        self.last_status_s = now
        self.last_synchronous_mode = bool(synchronous_mode)
        self.last_synchronous_mode_running = bool(
            synchronous_mode_running)
        self.last_fixed_delta_seconds = delta

        if parsed_frame is None:
            return self._fault('CARLA status frame is invalid', now)
        if not bool(synchronous_mode):
            return self._fault('CARLA bridge is not in synchronous mode', now)
        if (
            not math.isfinite(delta)
            or abs(delta - self.config.expected_fixed_delta_seconds)
            > self.config.fixed_delta_tolerance_s
        ):
            return self._fault(
                'CARLA fixed_delta_seconds mismatch: '
                f'expected={self.config.expected_fixed_delta_seconds:.6f} '
                f'actual={delta!r}',
                now,
            )

        if self.phase == PacerPhase.FAULTED:
            return self._pause_command(now)

        running = bool(synchronous_mode_running)
        if self.phase == PacerPhase.ACQUIRING_PAUSE:
            if not running:
                if parsed_frame >= UINT64_MAX:
                    return self._fault(
                        'CARLA frame counter cannot advance safely', now)
                self.frame = parsed_frame
                self.phase = PacerPhase.WAITING_FOR_DEADLINE
                self.next_step_deadline_s = now + self.config.step_period_s
            return ()

        if self.frame is None:
            return self._fault('pacer has no acknowledged baseline frame', now)

        if self.phase == PacerPhase.WAITING_FOR_DEADLINE:
            if parsed_frame != self.frame:
                return self._fault(
                    'CARLA frame changed without an outstanding STEP_ONCE: '
                    f'expected={self.frame} actual={parsed_frame}',
                    now,
                )
            if running:
                return self._fault(
                    'CARLA resumed PLAY without pacer ownership', now)
            return ()

        if self.phase != PacerPhase.WAITING_FOR_STEP_ACK:
            return self._fault(f'unexpected pacer phase {self.phase.value}', now)

        if parsed_frame == self.frame:
            # The bridge publishes RUNNING before performing the requested
            # tick; duplicate pre-step PAUSED samples are also harmless.
            return ()
        if parsed_frame != self.expected_frame:
            return self._fault(
                'CARLA STEP_ONCE frame progression was not exactly +1: '
                f'expected={self.expected_frame} actual={parsed_frame}',
                now,
            )
        if running:
            # The matching frame is not complete until the bridge confirms
            # the PAUSE that STEP_ONCE queues for its next loop iteration.
            return ()

        if self.step_command_s is None:
            return self._fault(
                'CARLA STEP_ONCE acknowledgement has no command timestamp',
                now,
            )
        completed_step_command_s = self.step_command_s
        self.frame = parsed_frame
        self.expected_frame = None
        self.step_command_s = None
        self.phase = PacerPhase.WAITING_FOR_DEADLINE
        # Account for the wall time already spent completing this one frame.
        # A late acknowledgement may make the next single step immediately
        # eligible, but no historical deadlines are accumulated and there is
        # still at most one outstanding STEP_ONCE command.
        self.next_step_deadline_s = max(
            now,
            completed_step_command_s + self.config.step_period_s,
        )
        return ()

    def poll(self, now_s):
        """Advance wall-time checks and request at most one bridge command."""
        now = _valid_monotonic(now_s)
        if now is None:
            fallback = self.last_status_s or self.started_s
            return self._fault('invalid monotonic poll timestamp', fallback)
        if self.phase == PacerPhase.RELEASED:
            return ()
        if self.phase == PacerPhase.FAULTED:
            return self._pause_command(now)

        if self.last_status_s is None:
            if now - self.started_s >= self.config.startup_timeout_s:
                return self._fault(
                    'timed out waiting for the first CARLA status', now)
        elif (
            self.phase != PacerPhase.WAITING_FOR_STEP_ACK
            and now - self.last_status_s > self.config.status_timeout_s
        ):
            return self._fault(
                'CARLA status timed out: '
                f'age={now - self.last_status_s:.3f}s',
                now,
            )

        if self.phase == PacerPhase.ACQUIRING_PAUSE:
            if now - self.started_s >= self.config.startup_timeout_s:
                return self._fault(
                    'timed out waiting for CARLA PAUSED acknowledgement', now)
            return self._pause_command(now)

        if self.phase == PacerPhase.WAITING_FOR_STEP_ACK:
            if (
                self.step_command_s is not None
                and now - self.step_command_s
                > self.config.step_ack_timeout_s
            ):
                return self._fault(
                    'CARLA STEP_ONCE acknowledgement timed out: '
                    f'frame={self.frame} age='
                    f'{now - self.step_command_s:.3f}s',
                    now,
                )
            return ()

        if self.phase != PacerPhase.WAITING_FOR_DEADLINE:
            return self._fault(f'unexpected pacer phase {self.phase.value}', now)
        if (
            self.next_step_deadline_s is None
            or now < self.next_step_deadline_s
        ):
            return ()
        if self.frame is None or self.frame >= UINT64_MAX:
            return self._fault('CARLA frame counter cannot advance safely', now)

        self.phase = PacerPhase.WAITING_FOR_STEP_ACK
        self.step_command_s = now
        self.expected_frame = self.frame + 1
        self.next_step_deadline_s = None
        return (PacerCommand.STEP_ONCE,)

    def health(self, now_s):
        """Return a fresh, currently-paused preflight decision."""
        now = _valid_monotonic(now_s)
        frame_text = 'unknown' if self.frame is None else str(self.frame)
        age = math.inf
        if now is not None and self.last_status_s is not None:
            age = max(0.0, now - self.last_status_s)
        common = (
            f'state={self.phase.value} frame={frame_text} '
            f'rate={self.target_rate_hz:.3f}Hz status_age={age:.3f}s'
        )
        if now is None:
            return PacerHealth(False, common + ' reason=invalid_health_time')
        if self.phase == PacerPhase.FAULTED:
            return PacerHealth(
                False, common + f' reason={self.fault_reason}')
        if not self.ready:
            return PacerHealth(False, common + ' reason=pacer_not_ready')
        if age > self.config.status_timeout_s:
            return PacerHealth(False, common + ' reason=status_stale')
        if self.phase != PacerPhase.WAITING_FOR_DEADLINE:
            return PacerHealth(False, common + ' reason=step_ack_pending')
        if self.last_synchronous_mode_running is not False:
            return PacerHealth(False, common + ' reason=not_paused')
        return PacerHealth(True, common + ' paused_ack=true')

    def release(self, now_s):
        """Stop pacing and return PLAY so the bridge can shut down cleanly."""
        now = _valid_monotonic(now_s)
        if now is None:
            raise ValueError('now_s must be finite and non-negative')
        self.phase = PacerPhase.RELEASED
        self.next_step_deadline_s = None
        self.step_command_s = None
        self.expected_frame = None
        return (PacerCommand.PLAY,)


class ExclusiveProcessLock:
    """Prevent two local pacers from owning one CARLA control topic."""

    def __init__(self, path):
        lock_path = Path(str(path)).expanduser()
        if not str(path).strip():
            raise ValueError('lock_file must not be empty')
        if not lock_path.parent.is_dir():
            raise ValueError(
                f'lock_file parent is not a directory: {lock_path.parent}')
        self.path = lock_path
        self._handle = None

    def acquire(self):
        """Acquire the non-blocking host lock or reject the second owner."""
        if self._handle is not None:
            return
        handle = self.path.open('a+', encoding='utf-8')
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            handle.seek(0)
            handle.truncate()
            handle.write(f'pid={os.getpid()}\n')
            handle.flush()
        except (BlockingIOError, OSError):
            handle.close()
            raise RuntimeError(
                f'another CARLA step pacer owns lock {self.path}'
            ) from None
        self._handle = handle

    def release(self):
        """Release ownership; the harmless lock inode may remain on disk."""
        if self._handle is None:
            return
        try:
            fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        finally:
            self._handle.close()
            self._handle = None


_ROS_IMPORT_ERROR = None
try:
    import rclpy
    from carla_msgs.msg import CarlaControl, CarlaStatus
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
except ImportError as error:  # Pure state machine remains unit-testable.
    _ROS_IMPORT_ERROR = error
    rclpy = None
    CarlaControl = None
    CarlaStatus = None
    ExternalShutdownException = Exception
    Node = object
    QoSDurabilityPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    SignalHandlerOptions = None
    Bool = None
    Trigger = None


class CarlaStepPacerNode(Node):
    """Own the public CARLA bridge run-state control at real-time 20 Hz."""

    def __init__(self, **node_kwargs):
        if rclpy is None:
            raise RuntimeError('ROS 2 is required to run CARLA step pacer') from (
                _ROS_IMPORT_ERROR)
        super().__init__('carla_step_pacer', **node_kwargs)
        self._callback_lock = RLock()
        self._last_published_ready = None
        self._last_logged_phase = None
        self._last_logged_fault = ''
        self._ready_announced = False

        self.control_topic = self._topic_parameter(
            'control_topic', '/carla/control')
        self.status_topic = self._topic_parameter(
            'status_topic', '/carla/status')
        self.ready_topic = self._topic_parameter(
            'ready_topic', '/virtual_carla/step_pacer/ready')
        self.release_service_name = self._topic_parameter(
            'release_service', '/virtual_carla/step_pacer/release')
        self.health_service_name = self._topic_parameter(
            'health_service', '/virtual_carla/step_pacer/health')

        config = validate_step_pacer_config(StepPacerConfig(
            step_period_s=self.declare_parameter(
                'step_period_s', 0.05).value,
            expected_fixed_delta_seconds=self.declare_parameter(
                'expected_fixed_delta_seconds', 0.05).value,
            fixed_delta_tolerance_s=self.declare_parameter(
                'fixed_delta_tolerance_s', 0.0001).value,
            pause_retry_s=self.declare_parameter(
                'pause_retry_s', 0.10).value,
            startup_timeout_s=self.declare_parameter(
                'startup_timeout_s', 10.0).value,
            status_timeout_s=self.declare_parameter(
                'status_timeout_s', 0.50).value,
            step_ack_timeout_s=self.declare_parameter(
                'step_ack_timeout_s', 20.0).value,
        ))
        poll_period_s = _finite_positive(
            self.declare_parameter('poll_period_s', 0.005).value,
            'poll_period_s',
        )
        if poll_period_s > config.step_period_s / 2.0:
            raise ValueError(
                'poll_period_s must be no more than half step_period_s')

        lock_file = str(self.declare_parameter(
            'lock_file', '/tmp/camrod_carla_step_pacer.lock').value)
        self._exclusive_lock = ExclusiveProcessLock(lock_file)
        self.state = StepPacerStateMachine(config, time.monotonic())

        bridge_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        ready_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.control_publisher = self.create_publisher(
            CarlaControl, self.control_topic, bridge_qos)
        self.ready_publisher = self.create_publisher(
            Bool, self.ready_topic, ready_qos)
        self.status_subscription = self.create_subscription(
            CarlaStatus, self.status_topic, self._on_status, bridge_qos)
        self.release_service = self.create_service(
            Trigger, self.release_service_name, self._on_release)
        self.health_service = self.create_service(
            Trigger, self.health_service_name, self._on_health)
        self.timer = self.create_timer(poll_period_s, self._on_timer)

        # No control command is published until exclusive ownership succeeds.
        # A losing duplicate may construct ROS endpoints briefly, but cannot
        # PAUSE, STEP, or PLAY the bridge.
        self._exclusive_lock.acquire()
        self._publish_readiness(force=True)
        self._dispatch(self.state.poll(time.monotonic()))
        self.get_logger().info(
            'CARLA step pacer acquiring PAUSE: target=%.3fHz delta=%.3fs '
            'status=%s control=%s'
            % (
                self.state.target_rate_hz,
                self.state.config.expected_fixed_delta_seconds,
                self.status_topic,
                self.control_topic,
            )
        )

    def _topic_parameter(self, name, default):
        value = str(self.declare_parameter(name, default).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    @staticmethod
    def _command_value(command):
        return {
            PacerCommand.PLAY: CarlaControl.PLAY,
            PacerCommand.PAUSE: CarlaControl.PAUSE,
            PacerCommand.STEP_ONCE: CarlaControl.STEP_ONCE,
        }[command]

    def _dispatch(self, commands):
        for command in commands:
            message = CarlaControl()
            message.command = self._command_value(command)
            self.control_publisher.publish(message)

    def _publish_readiness(self, force=False):
        ready = self.state.ready
        if force or ready != self._last_published_ready:
            self.ready_publisher.publish(Bool(data=ready))
            self._last_published_ready = ready

    def _report_transition(self):
        if self.state.phase != self._last_logged_phase:
            if (
                self.state.phase == PacerPhase.WAITING_FOR_DEADLINE
                and not self._ready_announced
            ):
                self.get_logger().info(
                    'CARLA real-time pacing ready: paused frame=%s '
                    'target=%.3fHz'
                    % (self.state.frame, self.state.target_rate_hz)
                )
                self._ready_announced = True
            elif self.state.phase == PacerPhase.RELEASED:
                self.get_logger().info(
                    'CARLA step pacing released; PLAY sent for clean shutdown')
            self._last_logged_phase = self.state.phase
        if (
            self.state.phase == PacerPhase.FAULTED
            and self.state.fault_reason != self._last_logged_fault
        ):
            self.get_logger().error(
                'CARLA step pacer faulted closed: %s; PAUSE asserted'
                % self.state.fault_reason
            )
            self._last_logged_fault = self.state.fault_reason

    def _complete_transition(self, commands):
        self._dispatch(commands)
        self._publish_readiness()
        self._report_transition()

    def _on_status(self, message):
        with self._callback_lock:
            commands = self.state.observe_status(
                frame=message.frame,
                synchronous_mode=message.synchronous_mode,
                synchronous_mode_running=message.synchronous_mode_running,
                fixed_delta_seconds=message.fixed_delta_seconds,
                now_s=time.monotonic(),
            )
            self._complete_transition(commands)

    def _on_timer(self):
        with self._callback_lock:
            self._complete_transition(self.state.poll(time.monotonic()))

    def _on_health(self, _request, response):
        with self._callback_lock:
            health = self.state.health(time.monotonic())
            response.success = health.success
            response.message = health.message
        return response

    def _on_release(self, _request, response):
        with self._callback_lock:
            self._complete_transition(self.state.release(time.monotonic()))
            response.success = True
            response.message = (
                'PLAY published; pacing released and bridge shutdown may proceed'
            )
        return response

    def release_for_shutdown(self):
        """Best-effort local counterpart of the public release service."""
        with self._callback_lock:
            if self.state.phase != PacerPhase.RELEASED:
                self._complete_transition(
                    self.state.release(time.monotonic()))

    def destroy_node(self):
        """Release only local ownership; unexpected exit remains PAUSED."""
        try:
            return super().destroy_node()
        finally:
            self._exclusive_lock.release()


def main(args=None):
    """Run the wall-time CARLA step pacer with clean SIGINT release."""
    if rclpy is None:
        raise RuntimeError('ROS 2 is required to run CARLA step pacer') from (
            _ROS_IMPORT_ERROR)
    # Keeping SIGINT as KeyboardInterrupt lets this node publish PLAY before
    # its DDS publisher is destroyed.  The release service is still the
    # deterministic shutdown path for an external process supervisor.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CarlaStepPacerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.release_for_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
