"""Unit tests for fail-closed public-control CARLA wall pacing."""

from dataclasses import replace
import math
from pathlib import Path

from camrod_carla_adapter.carla_step_pacer_node import (
    ExclusiveProcessLock,
    PacerCommand,
    PacerPhase,
    StepPacerConfig,
    StepPacerStateMachine,
    validate_step_pacer_config,
)
import pytest


def _paused(machine, frame=100, now_s=0.01, delta=0.05):
    return machine.observe_status(
        frame=frame,
        synchronous_mode=True,
        synchronous_mode_running=False,
        fixed_delta_seconds=delta,
        now_s=now_s,
    )


def _status(machine, frame, running, now_s, delta=0.05, sync=True):
    return machine.observe_status(
        frame=frame,
        synchronous_mode=sync,
        synchronous_mode_running=running,
        fixed_delta_seconds=delta,
        now_s=now_s,
    )


def test_startup_retries_pause_until_exact_paused_ack():
    machine = StepPacerStateMachine(now_s=0.0)

    assert machine.ready is False
    assert machine.poll(0.0) == (PacerCommand.PAUSE,)
    assert machine.poll(0.09) == ()
    assert machine.poll(0.10) == (PacerCommand.PAUSE,)

    assert _status(machine, 98, True, 0.11) == ()
    assert machine.phase == PacerPhase.ACQUIRING_PAUSE
    assert machine.ready is False

    assert _paused(machine, frame=100, now_s=0.12) == ()
    assert machine.phase == PacerPhase.WAITING_FOR_DEADLINE
    assert machine.ready is True
    assert machine.frame == 100
    assert machine.poll(0.169) == ()
    assert machine.poll(0.170) == (PacerCommand.STEP_ONCE,)


def test_one_outstanding_step_requires_frame_plus_one_and_paused_ack():
    machine = StepPacerStateMachine(now_s=0.0)
    _paused(machine, frame=700, now_s=0.01)
    assert machine.poll(0.061) == (PacerCommand.STEP_ONCE,)
    assert machine.phase == PacerPhase.WAITING_FOR_STEP_ACK
    assert machine.expected_frame == 701

    # The bridge first reports RUNNING at the old frame, then at the new
    # frame, and finally reports PAUSED at that same new frame.
    assert _status(machine, 700, True, 0.061) == ()
    assert machine.poll(0.062) == ()
    assert _status(machine, 701, True, 0.063) == ()
    assert machine.phase == PacerPhase.WAITING_FOR_STEP_ACK
    assert machine.poll(0.064) == ()
    assert _status(machine, 701, False, 0.065) == ()

    assert machine.phase == PacerPhase.WAITING_FOR_DEADLINE
    assert machine.frame == 701
    assert machine.expected_frame is None
    # The 4 ms spent completing the frame counts toward the 50 ms period.
    assert machine.poll(0.110) == ()
    assert machine.poll(0.111) == (PacerCommand.STEP_ONCE,)
    # A second poll cannot issue a second outstanding step.
    assert machine.poll(0.116) == ()


def test_slow_ack_is_immediately_eligible_but_never_bursts_catch_up_steps():
    config = replace(
        StepPacerConfig(), status_timeout_s=2.0, step_ack_timeout_s=2.0)
    machine = StepPacerStateMachine(config, now_s=0.0)
    _paused(machine, frame=10, now_s=0.01)
    assert machine.poll(0.061) == (PacerCommand.STEP_ONCE,)

    assert _status(machine, 11, False, 0.80) == ()
    assert machine.next_step_deadline_s == pytest.approx(0.80)
    # The late frame can start one new step immediately. Historical 50 ms
    # deadlines are not queued, so a second poll cannot emit a burst.
    assert machine.poll(0.80) == (PacerCommand.STEP_ONCE,)
    assert machine.poll(1.20) == ()


@pytest.mark.parametrize(
    ('frame', 'running', 'reason'),
    [
        (52, False, 'exactly +1'),
        (49, False, 'exactly +1'),
    ],
)
def test_frame_jump_or_rewind_faults_closed(frame, running, reason):
    machine = StepPacerStateMachine(now_s=0.0)
    _paused(machine, frame=50, now_s=0.01)
    machine.poll(0.061)

    assert _status(machine, frame, running, 0.07) == (PacerCommand.PAUSE,)
    assert machine.phase == PacerPhase.FAULTED
    assert machine.ready is False
    assert reason in machine.fault_reason


def test_external_play_or_uncommanded_frame_faults_closed():
    played = StepPacerStateMachine(now_s=0.0)
    _paused(played, frame=20, now_s=0.01)
    assert _status(played, 20, True, 0.02) == (PacerCommand.PAUSE,)
    assert 'resumed PLAY' in played.fault_reason

    advanced = StepPacerStateMachine(now_s=0.0)
    _paused(advanced, frame=20, now_s=0.01)
    assert _status(advanced, 21, False, 0.02) == (
        PacerCommand.PAUSE,)
    assert 'without an outstanding' in advanced.fault_reason


@pytest.mark.parametrize(
    ('sync', 'delta', 'reason'),
    [
        (False, 0.05, 'not in synchronous mode'),
        (True, 0.04, 'fixed_delta_seconds mismatch'),
        (True, math.nan, 'fixed_delta_seconds mismatch'),
    ],
)
def test_bridge_settings_are_continuously_enforced(sync, delta, reason):
    machine = StepPacerStateMachine(now_s=0.0)

    commands = _status(
        machine, 1, False, 0.01, delta=delta, sync=sync)

    assert commands == (PacerCommand.PAUSE,)
    assert machine.phase == PacerPhase.FAULTED
    assert reason in machine.fault_reason


def test_status_and_step_timeouts_latch_fault_and_retry_pause():
    stale = StepPacerStateMachine(now_s=0.0)
    _paused(stale, frame=1, now_s=0.01)
    assert stale.poll(0.511) == (PacerCommand.PAUSE,)
    assert 'status timed out' in stale.fault_reason
    assert stale.poll(0.60) == ()
    assert stale.poll(0.612) == (PacerCommand.PAUSE,)

    step = StepPacerStateMachine(now_s=0.0)
    _paused(step, frame=1, now_s=0.01)
    assert step.poll(0.061) == (PacerCommand.STEP_ONCE,)
    # A managed Ranger spawn may legitimately occupy one bridge-owned step for
    # more than the ordinary 0.5 s paused-status freshness bound.
    _status(step, 1, True, 0.50)
    assert step.poll(0.562) == ()
    assert step.phase == PacerPhase.WAITING_FOR_STEP_ACK
    assert step.poll(19.999) == ()
    assert step.poll(20.062) == (PacerCommand.PAUSE,)
    assert 'acknowledgement timed out' in step.fault_reason


def test_startup_timeout_and_invalid_frame_fail_closed():
    startup = StepPacerStateMachine(now_s=0.0)
    startup.poll(0.0)
    assert startup.poll(10.0) == (PacerCommand.PAUSE,)
    assert startup.phase == PacerPhase.FAULTED
    assert 'first CARLA status' in startup.fault_reason

    malformed = StepPacerStateMachine(now_s=0.0)
    commands = _status(malformed, -1, False, 0.01)
    assert commands == (PacerCommand.PAUSE,)
    assert 'frame is invalid' in malformed.fault_reason


def test_health_requires_fresh_current_paused_ack():
    machine = StepPacerStateMachine(now_s=0.0)
    initial = machine.health(0.0)
    assert initial.success is False
    assert 'state=acquiring_pause' in initial.message
    assert 'rate=20.000Hz' in initial.message

    _paused(machine, frame=123, now_s=0.01)
    healthy = machine.health(0.02)
    assert healthy.success is True
    assert 'frame=123' in healthy.message
    assert 'paused_ack=true' in healthy.message

    machine.poll(0.061)
    pending = machine.health(0.061)
    assert pending.success is False
    assert 'state=waiting_for_step_ack' in pending.message
    assert 'reason=step_ack_pending' in pending.message


def test_release_is_explicit_idempotent_play_and_disables_readiness():
    machine = StepPacerStateMachine(now_s=0.0)
    _paused(machine, frame=5, now_s=0.01)

    assert machine.release(0.02) == (PacerCommand.PLAY,)
    assert machine.release(0.03) == (PacerCommand.PLAY,)
    assert machine.phase == PacerPhase.RELEASED
    assert machine.ready is False
    assert machine.poll(1.0) == ()
    assert _status(machine, 99, True, 1.0) == ()


def test_startup_ready_accepts_continuous_slow_steps_but_health_stays_strict():
    # HH_261002 - Slow frames have no idle period to sample; a completed exact
    # PAUSED ACK proves pacing without bypassing the next outstanding ACK.
    machine = StepPacerStateMachine(now_s=0.0)
    assert machine.startup_ready(0.0).success is False
    _paused(machine, frame=100, now_s=0.01)
    assert machine.startup_ready(0.02).success is True
    machine.poll(0.061)
    first = machine.startup_ready(0.062)
    assert first.success is False
    assert 'no_completed_step_ack' in first.message
    _status(machine, 101, True, 0.13)
    assert machine.startup_ready(0.13).success is False
    _paused(machine, frame=101, now_s=0.14)
    assert machine.poll(0.14) == (PacerCommand.STEP_ONCE,)
    for frame, now in [(102, 0.21), (103, 0.28), (104, 0.35)]:
        _status(machine, frame - 1, True, now - 0.01)
        ready = machine.startup_ready(now - 0.005)
        assert ready.success is True
        assert 'completed_step_ack=true' in ready.message
        assert 'step_ack_pending=true' in ready.message
        assert machine.health(now - 0.005).success is False
        _paused(machine, frame=frame, now_s=now)
        assert machine.poll(now) == (PacerCommand.STEP_ONCE,)
        assert machine.poll(now) == ()


def test_startup_ready_rejects_stale_completed_ack_even_with_fresh_running_status():
    machine = StepPacerStateMachine(now_s=0.0)
    _paused(machine, now_s=0.01)
    machine.poll(0.061)
    _paused(machine, frame=101, now_s=0.14)
    machine.poll(0.14)
    _status(machine, 101, True, 0.65)
    readiness = machine.startup_ready(0.65)
    assert readiness.success is False
    assert 'completed_step_ack_stale' in readiness.message
    assert machine.last_completed_step_ack_s == 0.14


@pytest.mark.parametrize('failure', ['stale_status', 'expired_step', 'fault', 'released', 'invalid_time'])
def test_startup_ready_preserves_watchdogs_faults_and_release(failure):
    config = replace(StepPacerConfig(), step_ack_timeout_s=0.10)
    machine = StepPacerStateMachine(config, now_s=0.0)
    _paused(machine, now_s=0.01)
    machine.poll(0.061)
    _paused(machine, frame=101, now_s=0.14)
    machine.poll(0.14)
    now = 0.15
    if failure == 'stale_status':
        now = 0.65
    elif failure == 'expired_step':
        now = 0.25
        _status(machine, 101, True, now)
    elif failure == 'fault':
        _paused(machine, frame=103, now_s=now)
    elif failure == 'released':
        machine.release(now)
    else:
        now = math.nan
    assert machine.startup_ready(now).success is False


@pytest.mark.parametrize(
    'config',
    [
        replace(StepPacerConfig(), step_period_s=0.04),
        replace(StepPacerConfig(), fixed_delta_tolerance_s=0.05),
        replace(StepPacerConfig(), pause_retry_s=10.0),
        replace(StepPacerConfig(), status_timeout_s=0.05),
        replace(StepPacerConfig(), step_ack_timeout_s=math.inf),
    ],
)
def test_invalid_timing_contract_is_rejected(config):
    with pytest.raises(ValueError):
        validate_step_pacer_config(config)


def test_exclusive_process_lock_rejects_second_local_owner(tmp_path):
    path = tmp_path / 'step-pacer.lock'
    first = ExclusiveProcessLock(path)
    second = ExclusiveProcessLock(path)

    first.acquire()
    try:
        with pytest.raises(RuntimeError, match='another CARLA step pacer'):
            second.acquire()
    finally:
        first.release()

    second.acquire()
    second.release()


def test_package_registers_node_and_runtime_message_dependencies():
    package_root = Path(__file__).resolve().parents[1]
    setup_text = (package_root / 'setup.py').read_text(encoding='utf-8')
    package_xml = (package_root / 'package.xml').read_text(encoding='utf-8')

    assert 'carla_step_pacer_node:main' in setup_text
    assert '<exec_depend>carla_msgs</exec_depend>' in package_xml
    assert '<exec_depend>std_srvs</exec_depend>' in package_xml
