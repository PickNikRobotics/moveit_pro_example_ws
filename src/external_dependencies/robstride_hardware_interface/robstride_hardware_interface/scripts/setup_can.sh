#!/usr/bin/env bash
# Brings up a SocketCAN interface for RobStride actuators (e.g. a canable
# adapter) with the bitrate and queue tuning called out in the
# robstride_hardware_interface design notes:
#   - 1 Mbps bitrate (fixed by the RobStride CAN protocol)
#   - restart-ms so the interface self-recovers from bus-off
#   - a longer txqueuelen so a batched sendmmsg() burst of N motor commands
#     doesn't overflow the queue and get silently dropped
#
# Usage: setup_can.sh [interface] [bitrate] [txqueuelen]
#   setup_can.sh can0
#   setup_can.sh can0 1000000 1000

set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-1000000}"
TXQUEUELEN="${3:-1000}"

echo "Configuring ${IFACE}: bitrate=${BITRATE} txqueuelen=${TXQUEUELEN}"

sudo ip link set "${IFACE}" down 2>/dev/null || true
if ! sudo ip link set "${IFACE}" type can bitrate "${BITRATE}" restart-ms 100; then
  echo "restart-ms not supported by this controller; configuring without it" >&2
  sudo ip link set "${IFACE}" type can bitrate "${BITRATE}"
fi
sudo ip link set "${IFACE}" txqueuelen "${TXQUEUELEN}"
sudo ip link set "${IFACE}" up

ip -details link show "${IFACE}"
