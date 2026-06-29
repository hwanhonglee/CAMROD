#!/usr/bin/env bash
set -euo pipefail

iface="${1:-can0}"
bitrate="${2:-500000}"
restart_ms="${3:-100}"

ip link set "${iface}" down || true
ip link set "${iface}" type can bitrate "${bitrate}" restart-ms "${restart_ms}"
ip link set "${iface}" up
ip -details link show "${iface}"
