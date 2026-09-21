"""HH_260915 - Optional receive-only SocketCAN observer, with no CAN writes.

The independent listener cannot determine whether a bus frame was sent by
CAMROD or another controller. Every frame is labelled ``bus_observed``. Frame
storage/rotation belongs to MissionJournal; this module never edits an interface
or creates a log file.
"""

from __future__ import annotations

import re
import socket
import struct
import sys
import threading
import time


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF


def decode_can_frame(packet: bytes, *, channel: str, received_unix_ns: int,
                     received_monotonic_ns: int, kernel_timestamp_ns=None) -> dict:
    """Decode Linux classic CAN/CAN-FD structures without guessing payloads."""
    if len(packet) == 16:
        raw_id, length, _pad, _reserved, raw_dlc, data = struct.unpack("=IBBBB8s", packet)
        if length > 8:
            raise ValueError("invalid classic CAN payload length")
        frame_format, fd_flags = "classic", None
    elif len(packet) == 72:
        raw_id, length, fd_flags, _reserved, _reserved2, data = struct.unpack("=IBBBB64s", packet)
        if length > 64 or raw_id & CAN_RTR_FLAG:
            raise ValueError("invalid CAN FD frame")
        frame_format, raw_dlc = "fd", None
    else:
        raise ValueError(f"unsupported CAN frame size: {len(packet)}")
    extended = bool(raw_id & CAN_EFF_FLAG)
    remote = bool(raw_id & CAN_RTR_FLAG)
    error = bool(raw_id & CAN_ERR_FLAG)
    return {
        "format": frame_format, "channel": channel, "direction": "bus_observed",
        "direction_known": False, "can_id": raw_id & (CAN_EFF_MASK if extended or error else CAN_SFF_MASK),
        "raw_can_id": raw_id, "length": length, "data_hex": "" if remote else data[:length].hex(),
        "extended": extended, "remote": remote, "error": error,
        "fd_flags": fd_flags, "classic_len8_dlc": raw_dlc if length == 8 and raw_dlc and raw_dlc > 8 else None,
        "kernel_timestamp_ns": kernel_timestamp_ns,
        "kernel_timestamp_clock": "realtime" if kernel_timestamp_ns is not None else None,
        "received_unix_ns": received_unix_ns,
        "received_monotonic_ns": received_monotonic_ns,
        "hardware_bus_timestamp_available": False,
    }


def kernel_timestamp_from_ancillary(ancillary, option) -> int | None:
    for level, kind, payload in ancillary:
        if level == socket.SOL_SOCKET and kind == option and len(payload) >= struct.calcsize("@ll"):
            seconds, nanos = struct.unpack("@ll", payload[:struct.calcsize("@ll")])
            if seconds >= 0 and 0 <= nanos < 1_000_000_000:
                return seconds * 1_000_000_000 + nanos
    return None


class RawCanCapture:
    """Start an optional daemon receiver; start/stop never transmit CAN data."""

    def __init__(self, interface, on_frame, on_status, *, socket_factory=socket.socket,
                 now_ns=time.time_ns, monotonic_ns=time.monotonic_ns):
        self.interface = str(interface).strip()
        if self.interface and not re.fullmatch(r"[A-Za-z0-9_.:-]{1,15}", self.interface):
            raise ValueError("raw_can_interface must be one explicit Linux interface name")
        self._on_frame, self._on_status = on_frame, on_status
        self._socket_factory = socket_factory
        self._now_ns, self._monotonic_ns = now_ns, monotonic_ns
        self._stop = threading.Event()
        self._socket = None
        self._thread = None
        self._timestamp_option = None
        self._fd_enabled = False
        self._error_frames_enabled = False
        self._state = "disabled"
        self._overflow_option = None
        self._kernel_dropped = 0

    def _status(self, state, error=None):
        self._state = state
        self._on_status({
            "state": state, "configured": bool(self.interface), "interface": self.interface,
            "direction": "bus_observed", "direction_known": False,
            "fd_receive_enabled": self._fd_enabled,
            "error_frames_enabled": self._error_frames_enabled,
            "kernel_timestamp_enabled": self._timestamp_option is not None,
            "kernel_drop_monitor_enabled": self._overflow_option is not None,
            "kernel_dropped_frames_total": self._kernel_dropped if self._overflow_option is not None else None,
            "error": error,
        })

    def start(self):
        if self._thread is not None:
            raise RuntimeError("raw CAN observer is already started")
        if not self.interface:
            self._status("disabled")
            return False
        if sys.platform != "linux" or not hasattr(socket, "AF_CAN"):
            self._status("not_available", "Linux SocketCAN is unavailable")
            return False
        try:
            receiver = self._socket_factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self._socket = receiver
            receiver.settimeout(0.2)
            try:
                # Linux SO_RXQ_OVFL: receive-queue loss counter, not a bus write.
                receiver.setsockopt(socket.SOL_SOCKET, 40, 1)
                self._overflow_option = 40
            except OSError:
                pass
            # HH_260915 - These options affect this listener only; no link setup,
            # bitrate changes, CAN send, or controller command is performed.
            try:
                receiver.setsockopt(socket.SOL_CAN_RAW, 5, 1)  # CAN_RAW_FD_FRAMES
                self._fd_enabled = True
            except OSError:
                pass
            try:
                receiver.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack("=I", CAN_EFF_MASK))
                self._error_frames_enabled = True  # CAN_RAW_ERR_FILTER
            except OSError:
                pass
            timestamp_option = getattr(socket, "SO_TIMESTAMPNS", None)
            # Linux 64-bit UAPI SO_TIMESTAMPNS_OLD; native timespec is two longs.
            if timestamp_option is None and struct.calcsize("@l") == 8:
                timestamp_option = 35
            if timestamp_option is not None:
                try:
                    receiver.setsockopt(socket.SOL_SOCKET, timestamp_option, 1)
                    self._timestamp_option = timestamp_option
                except OSError:
                    pass
            receiver.bind((self.interface,))
        except OSError as exc:
            if self._socket is not None:
                self._socket.close()
                self._socket = None
            self._status("not_available", str(exc))
            return False
        self._status("listening")
        self._thread = threading.Thread(target=self._receive, name="camrod-raw-can-observer", daemon=True)
        self._thread.start()
        return True

    def _receive(self):
        while not self._stop.is_set():
            try:
                packet, ancillary, flags, _address = self._socket.recvmsg(72, socket.CMSG_SPACE(32))
                received_unix = self._now_ns()
                received_monotonic = self._monotonic_ns()
                if flags & getattr(socket, "MSG_TRUNC", 0):
                    raise ValueError("truncated CAN frame")
                kernel_ns = kernel_timestamp_from_ancillary(ancillary, self._timestamp_option)
                frame = decode_can_frame(packet, channel=self.interface,
                                         received_unix_ns=received_unix,
                                         received_monotonic_ns=received_monotonic,
                                         kernel_timestamp_ns=kernel_ns)
                frame["ancillary_truncated"] = bool(flags & getattr(socket, "MSG_CTRUNC", 0))
                for level, kind, payload in ancillary:
                    if level == socket.SOL_SOCKET and kind == self._overflow_option and len(payload) >= 4:
                        dropped = struct.unpack("=I", payload[:4])[0]
                        if dropped != self._kernel_dropped:
                            self._kernel_dropped = dropped
                            self._status("listening", f"kernel_receive_overflow: {dropped}")
                frame["kernel_dropped_frames_total"] = self._kernel_dropped if self._overflow_option is not None else None
                self._on_frame(frame)
            except socket.timeout:
                continue
            except (OSError, ValueError) as exc:
                if not self._stop.is_set():
                    self._status("error", str(exc))
                break
            except Exception as exc:
                self._status("error", f"frame recorder rejected input: {exc}")
                break

    def stop(self):
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._state == "listening":
            self._status("stopped")
