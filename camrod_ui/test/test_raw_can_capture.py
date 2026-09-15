"""HH_260915 - Receive-only tests use packed frames and mocked sockets only."""
from pathlib import Path
import socket
import struct
import sys
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "runtime" / "python"))
from camrod_ui.raw_can_capture import (  # noqa: E402
    RawCanCapture, decode_can_frame, kernel_timestamp_from_ancillary,
    CAN_EFF_FLAG, CAN_RTR_FLAG, CAN_ERR_FLAG,
)


def classic(can_id=0x211, data=b"\x00\x01\x02"):
    return struct.pack("=IBBBB8s", can_id, len(data), 0, 0, 0, data)


def decode(packet):
    return decode_can_frame(packet, channel="fixture0", received_unix_ns=100,
                            received_monotonic_ns=200, kernel_timestamp_ns=90)


def test_classic_preserves_observed_bytes_and_separate_clocks():
    frame = decode(classic())
    assert frame["can_id"] == 0x211
    assert frame["length"] == 3
    assert frame["data_hex"] == "000102"
    assert frame["direction"] == "bus_observed" and frame["direction_known"] is False
    assert frame["received_unix_ns"] == 100 and frame["received_monotonic_ns"] == 200
    assert frame["kernel_timestamp_ns"] == 90
    assert frame["hardware_bus_timestamp_available"] is False


@pytest.mark.parametrize("flags,extended,remote,error", [
    (CAN_EFF_FLAG, True, False, False), (CAN_RTR_FLAG, False, True, False),
    (CAN_ERR_FLAG, False, False, True),
])
def test_can_flags_are_not_interpreted_as_vehicle_status(flags, extended, remote, error):
    frame = decode(classic(flags | 0x211))
    assert (frame["extended"], frame["remote"], frame["error"]) == (extended, remote, error)
    assert frame["data_hex"] == ("" if remote else "000102")


def test_fd_payload_and_flags_preserved():
    packet = struct.pack("=IBBBB64s", CAN_EFF_FLAG | 0x1ABCDEF, 64, 3, 0, 0, bytes(range(64)))
    frame = decode(packet)
    assert frame["format"] == "fd" and frame["fd_flags"] == 3
    assert frame["can_id"] == 0x1ABCDEF and frame["length"] == 64
    assert frame["data_hex"] == bytes(range(64)).hex()


@pytest.mark.parametrize("packet", [b"", b"\0" * 15, b"\0" * 17,
    struct.pack("=IBBBB8s", 1, 9, 0, 0, 0, b""),
    struct.pack("=IBBBB64s", 1, 65, 0, 0, 0, b""),
    struct.pack("=IBBBB64s", CAN_RTR_FLAG | 1, 1, 0, 0, 0, b"")])
def test_invalid_or_unsupported_frame_not_invented(packet):
    with pytest.raises(ValueError):
        decode(packet)


def test_timestamp_absent_invalid_or_unrelated_is_unknown():
    assert kernel_timestamp_from_ancillary([], 35) is None
    assert kernel_timestamp_from_ancillary([(socket.SOL_SOCKET, 35, struct.pack("@ll", 2, 42))], 35) == 2000000042
    assert kernel_timestamp_from_ancillary([(socket.SOL_SOCKET, 35, struct.pack("@ll", 2, -1))], 35) is None
    assert kernel_timestamp_from_ancillary([(socket.SOL_SOCKET, 34, b"short")], 35) is None


class FakeSocket:
    def __init__(self, packet=None, *, bind_error=None, flags=0, ancillary=None):
        self.packet, self.bind_error, self.flags = packet, bind_error, flags
        self.ancillary = ancillary or []
        self.options, self.bound, self.closed = [], None, False

    def settimeout(self, timeout):
        self.timeout = timeout

    def setsockopt(self, *arguments):
        self.options.append(arguments)

    def bind(self, address):
        if self.bind_error:
            raise OSError(self.bind_error)
        self.bound = address

    def recvmsg(self, size, ancillary_size):
        assert size == 72
        if self.packet is not None:
            packet, self.packet = self.packet, None
            return packet, self.ancillary, self.flags, ("fixture0",)
        time.sleep(0.01)
        raise socket.timeout()

    def close(self):
        self.closed = True

    def send(self, *args):
        raise AssertionError("CAN send is forbidden")

    def sendto(self, *args):
        raise AssertionError("CAN sendto is forbidden")


def test_disabled_default_never_opens_a_socket():
    statuses = []
    def forbidden(*args):
        pytest.fail("disabled raw observer opened a socket")
    observer = RawCanCapture("", lambda frame: None, statuses.append, socket_factory=forbidden)
    assert observer.start() is False
    observer.stop()
    assert statuses[-1]["state"] == "disabled"


def test_missing_interface_or_permission_is_explicit_not_available():
    fake, statuses = FakeSocket(bind_error="fixture no device"), []
    observer = RawCanCapture("fixture0", lambda frame: None, statuses.append,
                             socket_factory=lambda *args: fake)
    assert observer.start() is False
    assert fake.closed
    assert statuses[-1]["state"] == "not_available"
    assert "no device" in statuses[-1]["error"]


def test_listener_receives_without_transmit_and_stops():
    fake, frames, statuses, received = FakeSocket(classic()), [], [], threading.Event()
    def capture(frame):
        frames.append(frame)
        received.set()
    observer = RawCanCapture("fixture0", capture, statuses.append,
                             socket_factory=lambda *args: fake,
                             now_ns=lambda: 11, monotonic_ns=lambda: 22)
    assert observer.start() is True
    assert received.wait(1)
    observer.stop()
    assert fake.bound == ("fixture0",) and fake.closed
    assert frames[0]["kernel_timestamp_ns"] is None
    assert frames[0]["received_unix_ns"] == 11
    assert [status["state"] for status in statuses] == ["listening", "stopped"]


def test_truncated_receive_fails_explicitly_and_does_not_record_frame():
    fake, frames, statuses, failed = FakeSocket(classic(), flags=socket.MSG_TRUNC), [], [], threading.Event()
    def status(value):
        statuses.append(value)
        if value["state"] == "error":
            failed.set()
    observer = RawCanCapture("fixture0", frames.append, status, socket_factory=lambda *args: fake)
    observer.start()
    assert failed.wait(1)
    observer.stop()
    assert not frames and statuses[-1]["state"] == "error"


@pytest.mark.parametrize("interface", ["../can0", "can0;echo", "a" * 16, "can 0"])
def test_interface_input_does_not_allow_paths_or_shell_text(interface):
    with pytest.raises(ValueError):
        RawCanCapture(interface, lambda frame: None, lambda status: None)


def test_kernel_queue_loss_is_reported_without_claiming_complete_raw_capture():
    fake = FakeSocket(classic(), ancillary=[(socket.SOL_SOCKET, 40, struct.pack("=I", 7))])
    statuses, frames, received = [], [], threading.Event()
    def capture(frame):
        frames.append(frame)
        received.set()
    observer = RawCanCapture("fixture0", capture, statuses.append, socket_factory=lambda *args: fake)
    observer.start()
    assert received.wait(1)
    observer.stop()
    assert frames[0]["kernel_dropped_frames_total"] == 7
    assert any("kernel_receive_overflow" in (entry["error"] or "") for entry in statuses)
