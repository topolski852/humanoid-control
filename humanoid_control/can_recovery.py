"""CAN-bus diagnosis and recovery for the fault-clear path.

A firmware fault that CLEAR_ERROR cannot clear is almost never a firmware problem. It is a bus
the host cannot transmit on, and the two ways that happened on this robot look the same from
the web app (every clear times out with "no SDO ack"):

  1. A JAMMED TX QUEUE. The CANable (gs_usb) has a handful of TX-echo slots. When they are all
     outstanding and never complete, the driver stops the queue: every write() is ENOBUFS, the
     qdisc backlog sits at a few hundred frames, and sysfs tx_packets stops advancing, while
     RX keeps working so every joint still looks online. No watchdog feed goes out, so every
     joint on that bus faults ERROR_WATCHDOG_TIMEOUT (0x0040). A link down/up flushes the
     queue and resets the slots; the daemon's bound socket survives it.

  2. AN EMCY FLOOD. A node with a fault it cannot leave broadcasts EMCY (func 1, arb id
     0x080|node) back to back. That is the highest-priority traffic on the bus, so it takes the
     whole 1 Mbit and nothing the host sends ever wins arbitration. That also jams the queue,
     so (1) comes back seconds after a bounce. Observed 2026-09-25: right_knee_pitch (node 8)
     at ~9,400 EMCY/s with ENCODER_FAULT, 0x2040. Nothing on the host clears it: the node
     acks NMT and then drops straight back into DAMPING with the fault set. It takes a power
     cycle of that leg, and a look at the encoder if it comes back.

This module only reads the bus (a raw socket that receives a copy of the traffic, never sends)
and bounces links through the passwordless `sudo ip link` rule in /etc/sudoers.d/humanoid-can.
"""
from __future__ import annotations

import json
import logging
import re
import socket
import struct
import subprocess
import time
from pathlib import Path

_log = logging.getLogger(__name__)

FUNC_SYNC_EMCY = 0x1
FUNC_ID_SHIFT = 7
DEVICE_ID_MASK = 0x7F

# Mirrors ErrorCode in daemon/src/motor/recoil_protocol.hpp.
ERROR_BITS = {
    0x0001: "GENERAL", 0x0002: "ESTOP", 0x0004: "INITIALIZATION", 0x0008: "CALIBRATION",
    0x0010: "POWERSTAGE", 0x0020: "INVALID_MODE", 0x0040: "WATCHDOG_TIMEOUT",
    0x0080: "OVER_VOLTAGE", 0x0100: "OVER_CURRENT", 0x0200: "OVER_TEMPERATURE",
    0x0400: "CAN_RX_FAULT", 0x0800: "CAN_TX_FAULT", 0x1000: "I2C_FAULT",
    0x2000: "ENCODER_FAULT",
}

# A healthy node sends EMCY once when it faults, not continuously. Anything sustained above
# this rate is a node shouting over the bus.
EMCY_FLOOD_HZ = 200.0


def decode_error(err: int) -> str:
    names = [n for bit, n in ERROR_BITS.items() if err & bit]
    return "|".join(names) or f"0x{err:04x}"


def _tx_packets(ifname: str) -> int | None:
    try:
        return int(Path(f"/sys/class/net/{ifname}/statistics/tx_packets").read_text())
    except (OSError, ValueError):
        return None


def _qdisc_backlog(ifname: str) -> int:
    """Frames waiting in the kernel queue for this interface (0 if unknown)."""
    try:
        out = subprocess.run(["tc", "-s", "qdisc", "show", "dev", ifname],
                             capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return 0
    m = re.search(r"backlog\s+\S+\s+(\d+)p", out)
    return int(m.group(1)) if m else 0


def tx_stalled(ifname: str, window: float = 0.3) -> bool:
    """True if frames are queued for this interface but none are leaving it."""
    before = _tx_packets(ifname)
    if before is None:
        return False
    time.sleep(window)
    after = _tx_packets(ifname)
    return after == before and _qdisc_backlog(ifname) > 0


def _bitrate(ifname: str) -> int:
    try:
        out = subprocess.run(["ip", "-j", "-d", "link", "show", ifname],
                             capture_output=True, text=True, timeout=2).stdout
        return int(json.loads(out)[0]["linkinfo"]["info_data"]["bittiming"]["bitrate"])
    except Exception:
        return 1_000_000


def bounce(ifname: str) -> bool:
    """Link down/up at the current bitrate: flushes a jammed TX queue. True on success."""
    rate = _bitrate(ifname)
    for args in (["link", "set", ifname, "down"],
                 ["link", "set", ifname, "up", "type", "can", "bitrate", str(rate)]):
        r = subprocess.run(["sudo", "-n", "ip", *args], capture_output=True, text=True, timeout=5)
        if r.returncode != 0:
            _log.warning("bounce %s: `ip %s` failed: %s", ifname, " ".join(args),
                         r.stderr.strip())
            return False
    _log.info("bounced %s (bitrate %d) to flush a jammed TX queue", ifname, rate)
    return True


def emcy_flooders(ifname: str, window: float = 0.3) -> dict[int, tuple[float, int]]:
    """Sniff the bus and return {node_id: (emcy_per_second, error_code)} for every node
    sustaining EMCY above EMCY_FLOOD_HZ. Receive-only; empty if the bus cannot be opened."""
    counts: dict[int, int] = {}
    last_err: dict[int, int] = {}
    try:
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    except (OSError, AttributeError):
        return {}
    try:
        s.bind((ifname,))
        s.settimeout(0.05)
        end = time.monotonic() + window
        while time.monotonic() < end:
            try:
                frame = s.recv(16)
            except socket.timeout:
                continue
            can_id, dlc = struct.unpack_from("<IB", frame)
            can_id &= socket.CAN_SFF_MASK
            if (can_id >> FUNC_ID_SHIFT) & 0xF != FUNC_SYNC_EMCY:
                continue
            node = can_id & DEVICE_ID_MASK
            counts[node] = counts.get(node, 0) + 1
            if dlc >= 4:
                last_err[node] = struct.unpack_from("<I", frame, 8)[0]
    except OSError as exc:
        _log.warning("emcy sniff on %s failed: %s", ifname, exc)
        return {}
    finally:
        s.close()
    return {n: (c / window, last_err.get(n, 0))
            for n, c in counts.items() if c / window >= EMCY_FLOOD_HZ}
