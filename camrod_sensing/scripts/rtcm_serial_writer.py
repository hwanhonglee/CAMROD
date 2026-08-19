#!/usr/bin/env python3
"""Forward NTRIP RTCM to the ArduSimple Lite moving-base serial port.

HH_260722 - Field topology and correction ownership:

* Input: ``/gnss/ntrip_client/rtcm`` from the CORS caster.
* Output: the FTDI DN05Y9E7 stable by-id path at 460800 baud. This device is
  currently ``/dev/ttyUSB0`` and reaches the Lite moving base on UART1 through
  the board's POWER+XBEE connector.
* The moving base applies CORS, becomes absolute RTK fixed, and independently
  emits its moving-baseline 4072.0/MSM stream to the heading rover.

This node deliberately does not write CORS to ``/dev/ttyACM0``. That device is
the heading rover's native USB and is owned by ``ublox_gps_node`` for NAV-PVT
and NAV-RELPOSNED output. Mixing CORS and moving-base reference streams on that
receiver broke heading during the field A/B test.
"""

import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from rtcm_msgs.msg import Message as RtcmMessage
import serial


def _coerce_bytes(data: Iterable[int]) -> bytes:
    return bytes(int(value) & 0xFF for value in data)


class RtcmSerialWriter(Node):
    """Write rtcm_msgs/Message payloads to a configured serial device."""

    def __init__(self) -> None:
        super().__init__("rtcm_serial_writer")
        # HH_260722 - Keep the node-level device default empty so an accidental
        # standalone start cannot write to an arbitrary serial receiver.
        self.declare_parameter("device", "")
        # HH_260819 - Canonical launches override this from the GNSS YAML. Keep
        # the standalone fallback aligned with the replacement Lite UART1 link.
        self.declare_parameter("baud", 460800)
        self.declare_parameter("rtcm_topic", "ntrip_client/rtcm")
        self.declare_parameter("write_timeout_sec", 0.2)
        self.declare_parameter("reopen_delay_sec", 1.0)

        self._device = str(self.get_parameter("device").value)
        self._baud = int(self.get_parameter("baud").value)
        self._write_timeout_sec = float(self.get_parameter("write_timeout_sec").value)
        self._reopen_delay_sec = float(self.get_parameter("reopen_delay_sec").value)
        self._serial = None
        self._last_open_attempt = 0.0

        topic = str(self.get_parameter("rtcm_topic").value)
        if not self._device:
            raise ValueError("rtcm_serial_writer requires a non-empty 'device' parameter")

        self.create_subscription(RtcmMessage, topic, self._on_rtcm, 100)
        self.get_logger().info(
            f"Forwarding RTCM topic '{topic}' to {self._device} @ {self._baud} baud"
        )

    def destroy_node(self) -> bool:
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        return super().destroy_node()

    def _open_serial(self):
        now = time.monotonic()
        if self._serial is not None:
            return self._serial
        if now - self._last_open_attempt < self._reopen_delay_sec:
            return None
        self._last_open_attempt = now
        try:
            self._serial = serial.Serial(
                port=self._device,
                baudrate=self._baud,
                timeout=0.0,
                write_timeout=self._write_timeout_sec,
            )
        except serial.SerialException as exc:
            self.get_logger().warn(f"Could not open RTCM serial device {self._device}: {exc}")
            return None
        self.get_logger().info(f"Opened RTCM serial device {self._device}")
        return self._serial

    def _on_rtcm(self, msg: RtcmMessage) -> None:
        # HH_260722 - Preserve every RTCM frame byte-for-byte. The F9P performs
        # protocol decoding; this bridge changes only the physical destination.
        payload = _coerce_bytes(msg.message)
        if not payload:
            return
        port = self._open_serial()
        if port is None:
            return
        try:
            written = port.write(payload)
            if written != len(payload):
                self.get_logger().warn(
                    f"Short RTCM serial write to {self._device}: {written}/{len(payload)} bytes"
                )
        except serial.SerialException as exc:
            self.get_logger().warn(f"RTCM serial write failed on {self._device}: {exc}")
            port.close()
            self._serial = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtcmSerialWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # HH_260722 - SIGINT may already have shut the default context down.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
