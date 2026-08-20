#!/usr/bin/env python3
"""
Live WitMotion IMU monitor — verify the sensor and CALIBRATE the mounting convention.

Runs read-only against the sensor (no flash writes). Use it to confirm data is fresh and
to pin down the IMU→base frame before trusting the balance loop:

  1. Hold the robot upright  → projected_gravity should read ≈ [0, 0, -1].
  2. Pitch the robot NOSE-DOWN → watch which projected_gravity component goes negative.
  3. Roll RIGHT / yaw LEFT     → confirm gyro sign matches the sim base-frame convention.

Cross-checks projected_gravity from the fused quaternion against gravity from the raw
accelerometer (-accel/|accel|) — they should agree when the robot is quasi-static. A
persistent disagreement means a wrong quaternion order/handedness that no fixed mounting
rotation can fix.

Usage:  python scripts/imu_monitor.py [--port /dev/ttyUSB0] [--baud 9600]
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0])
from humanoid_control.imu import WitMotionReader, SerialImuBaseState  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--hz", type=float, default=10.0, help="print rate")
    args = ap.parse_args()

    reader = WitMotionReader(args.port, args.baud).start()
    base = SerialImuBaseState(reader)  # identity mounting_rotation until calibrated
    print(f"reading {args.port} @ {args.baud} … Ctrl-C to stop\n", file=sys.stderr)
    time.sleep(0.3)

    period = 1.0 / args.hz
    t0 = time.monotonic()
    last_frames = 0
    try:
        while True:
            s = reader.latest()
            bs = base.get()
            now = time.monotonic()
            age_ms = (now - s.stamp) * 1e3 if s.stamp else float("inf")
            fr = reader.frames_total
            fps = (fr - last_frames) / period
            last_frames = fr

            euler = "        --        " if s.euler_deg is None else \
                f"r={s.euler_deg[0]:+6.1f} p={s.euler_deg[1]:+6.1f} y={s.euler_deg[2]:+6.1f}"
            gyro = "      --      " if s.gyro_dps is None else \
                f"{s.gyro_dps[0]:+6.1f} {s.gyro_dps[1]:+6.1f} {s.gyro_dps[2]:+6.1f}"
            pg_q = f"[{bs.projected_gravity[0]:+.2f} {bs.projected_gravity[1]:+.2f} {bs.projected_gravity[2]:+.2f}]"
            if s.accel_g is not None and np.linalg.norm(s.accel_g) > 1e-3:
                g_acc = -s.accel_g / np.linalg.norm(s.accel_g)
                pg_a = f"[{g_acc[0]:+.2f} {g_acc[1]:+.2f} {g_acc[2]:+.2f}]"
            else:
                pg_a = "   --   "

            flag = "VALID " if bs.valid else "STALE!"
            sys.stdout.write(
                f"\r{flag} age={age_ms:5.0f}ms {fps:4.0f}f/s | "
                f"euler {euler} | gyro(°/s) {gyro} | "
                f"pg(quat){pg_q} pg(accel){pg_a}   "
            )
            sys.stdout.flush()
            time.sleep(max(0.0, t0 + period - now))
            t0 += period
    except KeyboardInterrupt:
        print("\nstopped.", file=sys.stderr)
    finally:
        reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
