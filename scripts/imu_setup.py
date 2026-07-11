#!/usr/bin/env python3
"""Configure the WitMotion IMU and SAVE it to the sensor's flash so it survives power loss.

The daemon's reader is PASSIVE — it needs the IMU already streaming quaternion (0x59) +
angular velocity (0x52) at the target baud. WitMotion sensors revert to 9600 baud on every
power loss unless the config is written to their onboard flash. This tool finds the IMU's
current baud, sets the target baud + output rate, and issues the SAVE command, so the IMU
comes up streaming correctly on its own after any power cycle.

*** Needs EXCLUSIVE access to the serial port — stop the daemon first: ***

    sudo systemctl stop humanoid-daemon
    python scripts/imu_setup.py
    sudo systemctl start humanoid-daemon

Options:
    --device /dev/humanoid_imu   serial device (default)
    --baud 921600                target baud to set + save (default)
    --rate 100                   output rate Hz (default 100)
    --no-save                    configure for this session only (reverts on power cycle)

No pyserial dependency: baud is set via `stty`, raw bytes via os.read/os.write.
"""
from __future__ import annotations

import argparse
import collections
import os
import select
import subprocess
import sys
import time

# WitMotion standard register write: FF AA <reg> <dataL> <dataH>
_UNLOCK = [0xFF, 0xAA, 0x69, 0x88, 0xB5]     # KEY register unlock
_SAVE   = [0xFF, 0xAA, 0x00, 0x00, 0x00]     # reg 0x00 = 0x0000 → save config to flash
_REG_RATE = 0x03
_REG_BAUD = 0x04

# numeric → WIT register value
_BAUD_CODE = {4800: 1, 9600: 2, 19200: 3, 38400: 4, 57600: 5,
              115200: 6, 230400: 7, 460800: 8, 921600: 9}
_RATE_CODE = {0.2: 1, 0.5: 2, 1: 3, 2: 4, 5: 5, 10: 6, 20: 7, 50: 8,
              100: 9, 125: 10, 200: 11}
_PROBE_BAUDS = [921600, 9600, 115200, 230400, 460800, 57600, 38400, 19200]


def _stty(dev: str, baud: int) -> None:
    subprocess.run(["stty", "-F", dev, str(baud), "raw", "-echo"],
                   check=True, stderr=subprocess.DEVNULL)


def _read_frames(dev: str, dur: float = 0.8) -> tuple[int, int, dict]:
    """Read for `dur` s at the current stty baud; return (bytes, valid_frames, {type: count})."""
    fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
    buf = bytearray(); total = 0; frames = 0; types: collections.Counter = collections.Counter()
    end = time.monotonic() + dur
    try:
        while time.monotonic() < end:
            r, _, _ = select.select([fd], [], [], 0.1)
            if not r:
                continue
            try:
                c = os.read(fd, 512)
            except BlockingIOError:
                continue
            total += len(c); buf.extend(c)
            while len(buf) >= 11:
                if buf[0] != 0x55:
                    del buf[0]; continue
                p = buf[:11]
                if (sum(p[:10]) & 0xFF) == p[10]:
                    frames += 1; types[hex(p[1])] += 1; del buf[:11]
                else:
                    del buf[0]
    finally:
        os.close(fd)
    return total, frames, dict(types)


def _write(dev: str, *cmds: list) -> None:
    fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
    try:
        for cmd in cmds:
            os.write(fd, bytes(cmd))
            time.sleep(0.2)
    finally:
        os.close(fd)


def _find_baud(dev: str) -> int | None:
    for b in _PROBE_BAUDS:
        _stty(dev, b)
        _total, frames, types = _read_frames(dev, 0.6)
        if frames >= 3:
            print(f"  found IMU @ {b} baud (types={types})", file=sys.stderr)
            return b
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/humanoid_imu")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rate", type=float, default=100)
    ap.add_argument("--no-save", action="store_true")
    args = ap.parse_args()

    if args.baud not in _BAUD_CODE:
        raise SystemExit(f"unsupported target baud {args.baud} (use one of {sorted(_BAUD_CODE)})")
    if args.rate not in _RATE_CODE:
        raise SystemExit(f"unsupported rate {args.rate} (use one of {sorted(_RATE_CODE)})")
    dev = args.device
    if not os.path.exists(dev):
        print(f"✗ device {dev} not present — is the IMU plugged in / udev rule installed?",
              file=sys.stderr)
        return 1

    print(f"IMU setup: {dev} → {args.baud} baud @ {args.rate:g} Hz, "
          f"{'SAVE to flash' if not args.no_save else 'session only'}", file=sys.stderr)

    cur = _find_baud(dev)
    if cur is None:
        print("✗ IMU not found at any baud — is it powered / on the right device?", file=sys.stderr)
        return 1

    rate_cmd = [0xFF, 0xAA, _REG_RATE, _RATE_CODE[args.rate], 0x00]
    baud_cmd = [0xFF, 0xAA, _REG_BAUD, _BAUD_CODE[args.baud], 0x00]

    # Configure at the CURRENT baud: unlock, set rate, set output baud (switches immediately).
    _stty(dev, cur)
    print(f"  configuring at {cur} baud: rate + target baud…", file=sys.stderr)
    _write(dev, _UNLOCK, rate_cmd, _UNLOCK, baud_cmd)
    time.sleep(0.3)

    # Verify at the new baud.
    _stty(dev, args.baud)
    _total, frames, types = _read_frames(dev, 1.0)
    if frames < 3 or "0x59" not in types:
        print(f"✗ after baud switch: frames={frames} types={types} — quaternion missing.",
              file=sys.stderr)
        print("  Re-probing to locate the IMU…", file=sys.stderr)
        _find_baud(dev)
        return 1
    print(f"  ✓ streaming at {args.baud}: {frames} frames/s-ish, types={types}", file=sys.stderr)

    if args.no_save:
        print("  --no-save: not persisting (reverts to 9600 on next power loss).", file=sys.stderr)
        return 0

    # Persist to the sensor's flash so it survives power loss.
    _write(dev, _UNLOCK, _SAVE)
    time.sleep(0.4)
    _stty(dev, args.baud)
    _total, frames, types = _read_frames(dev, 1.0)
    ok = frames >= 3 and "0x59" in types
    print(f"  {'✓ saved' if ok else '✗ post-save check'}: frames={frames} types={types}",
          file=sys.stderr)
    print("DONE — power-cycle the IMU to confirm it comes back at the target baud."
          if ok else "CHECK — save may not have held.", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
