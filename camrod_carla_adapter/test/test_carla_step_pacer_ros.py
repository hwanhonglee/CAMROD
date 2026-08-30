"""ROS-boundary tests for CARLA step-pacer messages and services."""

import os

from camrod_carla_adapter.carla_step_pacer_node import (
    CarlaStepPacerNode,
    PacerPhase,
)
from carla_msgs.msg import CarlaControl, CarlaStatus
import rclpy
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


class _CapturePublisher:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _status(frame, running):
    message = CarlaStatus()
    message.frame = frame
    message.synchronous_mode = True
    message.synchronous_mode_running = running
    message.fixed_delta_seconds = 0.05
    return message


def test_ros_boundary_maps_control_readiness_health_and_release(tmp_path):
    suffix = str(os.getpid())
    rclpy.init()
    node = CarlaStepPacerNode(parameter_overrides=[
        Parameter('control_topic', value='/carla/test/control_' + suffix),
        Parameter('status_topic', value='/carla/test/status_' + suffix),
        Parameter(
            'ready_topic', value='/virtual_carla/test/ready_' + suffix),
        Parameter(
            'release_service', value='/virtual_carla/test/release_' + suffix),
        Parameter(
            'health_service', value='/virtual_carla/test/health_' + suffix),
        Parameter('lock_file', value=str(tmp_path / 'step-pacer.lock')),
    ])
    controls = _CapturePublisher()
    readiness = _CapturePublisher()
    node.control_publisher = controls
    node.ready_publisher = readiness

    try:
        node._on_status(_status(42, running=False))
        assert node.state.phase == PacerPhase.WAITING_FOR_DEADLINE
        assert readiness.messages[-1].data is True

        health = node._on_health(Trigger.Request(), Trigger.Response())
        assert health.success is True
        assert 'state=waiting_for_deadline' in health.message
        assert 'frame=42' in health.message
        assert 'rate=20.000Hz' in health.message

        node.state.next_step_deadline_s = 0.0
        node._on_timer()
        assert controls.messages[-1].command == CarlaControl.STEP_ONCE
        assert node.state.phase == PacerPhase.WAITING_FOR_STEP_ACK

        pending = node._on_health(Trigger.Request(), Trigger.Response())
        assert pending.success is False
        assert 'reason=step_ack_pending' in pending.message

        node._on_status(_status(42, running=True))
        node._on_status(_status(43, running=True))
        node._on_status(_status(43, running=False))
        assert node.state.frame == 43
        assert node.state.phase == PacerPhase.WAITING_FOR_DEADLINE

        released = node._on_release(Trigger.Request(), Trigger.Response())
        assert released.success is True
        assert controls.messages[-1].command == CarlaControl.PLAY
        assert readiness.messages[-1].data is False
        assert node.state.phase == PacerPhase.RELEASED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
