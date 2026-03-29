#!/usr/bin/env python3
# HH_260329: Shared helpers/constants for CAMROD plugin API nodes.

from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticStatus


def to_diag_level_int(value: object) -> int:
    """Normalize ROS diagnostic level field into an integer."""
    if isinstance(value, (bytes, bytearray)):
        if not value:
            return int(DiagnosticStatus.OK)
        return int(value[0])
    return int(value)


def to_diag_level_bytes(value: object) -> bytes:
    """Normalize diagnostic level field into single-byte wire format."""
    if isinstance(value, (bytes, bytearray)):
        if len(value) == 1:
            return bytes(value)
        if len(value) > 1:
            return bytes([value[0]])
        return b"\x00"
    return bytes([int(value) & 0xFF])
