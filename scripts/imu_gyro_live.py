#!/usr/bin/env python3
"""Live gyro/accel monitor — the decisive test for a gyro that reads exactly zero.

The IMU's config registers are known-good (CALSW=0 normal, RSW enables gyro output, 100 Hz,
921600 baud) and gyro frames DO arrive on schedule — but every axis payload is 0x0000 while
accel and magnetometer dither normally. Two explanations remain, and only motion separates
them:

  * **rest auto-zero** — the sensor suppresses gyro output while stationary. Then rotating it
    produces obvious nonzero rates, and the gyro is fine for walking (though M5 gyro-bias is
    unmeasurable, because the sensor refuses to report its own bias).
  * **dead gyro** — the rates stay 0.00 no matter how hard you move it. Then `base_ang_vel`
    is three permanently-zero policy inputs and the IMU needs replacing.

Needs the port, so the daemon must be stopped:

    sudo systemctl stop humanoid-daemon
    python scripts/imu_gyro_live.py            # ROTATE / TILT the robot while this runs
    sudo systemctl start humanoid-daemon

No pyserial dependency (stty + os.read), same as scripts/imu_setup.py.
"""
from __future__ import annotations

import argparse
import os
import select
import struct
import subprocess
import sys
import time

DEV = "/dev/humanoid_imu"
BAUD = 921600
GYRO_SCALE = 2000.0 / 32768.0    # -> deg/s
ACC_SCALE = 16.0 / 32768.0       # -> g


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default=DEV)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    if subprocess.run(["pgrep", "-f", "humanoid_daemon"],
                      capture_output=True, text=True).stdout.strip():
        print("REFUSING: the daemon holds the IMU port. Stop it first:\n"
              "  sudo systemctl stop humanoid-daemon", file=sys.stderr)
        return 2

    subprocess.run(["stty", "-F", args.device, str(args.baud), "raw", "-echo"], check=True)
    fd = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK)
    buf = bytearray()
    gyro = (0.0, 0.0, 0.0)
    acc = (0.0, 0.0, 0.0)
    peak = 0.0
    nonzero_frames = 0
    gyro_frames = 0
    last_print = 0.0
    end = time.time() + args.seconds

    print(">>> ROTATE OR TILT THE ROBOT NOW — watching for any nonzero gyro rate <<<\n")
    try:
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            buf.extend(os.read(fd, 4096))
            while len(buf) >= 11:
                if buf[0] != 0x55:
                    del buf[0]
                    continue
                f = bytes(buf[:11])
                if (sum(f[:10]) & 0xFF) != f[10]:
                    del buf[0]
                    continue
                t = f[1]
                d = struct.unpack("<hhhh", f[2:10])
                if t == 0x52:
                    gyro_frames += 1
                    gyro = tuple(v * GYRO_SCALE for v in d[:3])
                    mag = max(abs(x) for x in gyro)
                    peak = max(peak, mag)
                    if mag > 0:
                        nonzero_frames += 1
                elif t == 0x51:
                    acc = tuple(v * ACC_SCALE for v in d[:3])
                del buf[:11]

            now = time.time()
            if now - last_print >= 0.2:
                last_print = now
                pct = 100.0 * nonzero_frames / gyro_frames if gyro_frames else 0.0
                sys.stdout.write(
                    f"\rgyro deg/s [{gyro[0]:8.2f} {gyro[1]:8.2f} {gyro[2]:8.2f}]  "
                    f"peak {peak:7.2f}  nonzero {pct:5.1f}%  |  "
                    f"accel g [{acc[0]:6.2f} {acc[1]:6.2f} {acc[2]:6.2f}]   ")
                sys.stdout.flush()
    finally:
        os.close(fd)

    print("\n")
    pct = 100.0 * nonzero_frames / gyro_frames if gyro_frames else 0.0
    print(f"gyro frames      : {gyro_frames}")
    print(f"nonzero frames   : {nonzero_frames} ({pct:.2f}%)")
    print(f"peak |rate|      : {peak:.3f} deg/s")
    print()
    if gyro_frames == 0:
        print("VERDICT: no gyro frames at all — check RSW output-content register.")
    elif peak == 0.0:
        print("VERDICT: DEAD GYRO. Frames arrive on schedule but every sample is exactly zero,")
        print("         even under motion. base_ang_vel is 3 permanently-zero policy inputs.")
        print("         The IMU needs replacing; do not trust any angular-rate observation.")
    elif peak < 1.0:
        print("VERDICT: INCONCLUSIVE — peak rate under 1 deg/s. Was the robot actually moved?")
        print("         Re-run and rotate it firmly through 45+ degrees.")
    else:
        print("VERDICT: GYRO IS ALIVE. It reads zero at rest (internal auto-zero) but responds")
        print("         to motion. Walking observations are fine; M5 gyro-BIAS is unmeasurable,")
        print("         because the sensor zeroes the very bias we wanted to characterise.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
