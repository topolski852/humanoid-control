#!/usr/bin/env python3
"""Compute the IMU mounting rotation for the daemon `imu.mounting_rotation` config.

The policy needs ``projected_gravity`` and ``angular_velocity`` in the ROBOT BASE frame,
but the sensor reports them in its own (however-it-is-bolted-on) frame. This helper reads
the live IMU, averages the measured gravity while the robot is held in its **upright,
forward-facing pose**, and prints the ``mounting_rotation`` quaternion [w,x,y,z] that
rotates the IMU frame into the base frame so that gravity reads [0,0,-1] upright.

    python scripts/imu_calibrate.py                 # /dev/humanoid_imu @ 921600
    python scripts/imu_calibrate.py --device /dev/ttyUSB0 --seconds 3

⚠️  Gravity fixes only ROLL and PITCH (2 of 3 DOF). YAW — which horizontal axis is
"robot forward" — is unobservable from gravity alone. If the sensor's X axis is not
already aligned with robot-forward, compose an extra yaw rotation about Z by hand and
confirm by turning the robot and watching that the yaw-rate sign matches. See the note
this prints at the end.

No pyserial dependency: baud is set via `stty`, bytes read from the raw device.
"""
from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import time

import numpy as np


def read_gravity_samples(device: str, baud: int, seconds: float) -> np.ndarray:
    """Return an (N,3) array of accelerometer gravity-direction unit vectors.

    Uses the WitMotion 0x51 acceleration frame (the direct gravity measurement while
    stationary), which is independent of the sensor's internal quaternion fusion — the
    right signal for a mounting reference.
    """
    subprocess.run(["stty", "-F", device, str(baud), "raw", "-echo"], check=True)
    samples: list[list[float]] = []
    deadline = time.monotonic() + seconds
    with open(device, "rb", buffering=0) as f:
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = f.read(256)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= 11:
                if buf[0] != 0x55:
                    del buf[0]
                    continue
                pkt = buf[:11]
                if (sum(pkt[:10]) & 0xFF) != pkt[10]:
                    del buf[0]
                    continue
                if pkt[1] == 0x51:  # acceleration ax,ay,az (raw/32768*16 g), temp
                    ax, ay, az, _ = struct.unpack("<hhhh", pkt[2:10])
                    samples.append([ax, ay, az])
                del buf[:11]
    if not samples:
        raise SystemExit(f"no acceleration frames read from {device} — check device/baud")
    return np.asarray(samples, dtype=np.float64)


def shortest_arc_quat(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Quaternion [w,x,y,z] rotating unit vector ``src`` onto unit vector ``dst``."""
    src = src / np.linalg.norm(src)
    dst = dst / np.linalg.norm(dst)
    d = float(np.dot(src, dst))
    if d >= 1.0 - 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if d <= -1.0 + 1e-9:
        # 180°: pick any axis orthogonal to src.
        axis = np.cross(src, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(src, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        return np.array([0.0, *axis])
    axis = np.cross(src, dst)
    w = 1.0 + d
    q = np.array([w, *axis])
    return q / np.linalg.norm(q)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/humanoid_imu")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    print(f"Hold the robot UPRIGHT and STILL. Averaging gravity for {args.seconds:.0f}s…",
          file=sys.stderr)
    raw = read_gravity_samples(args.device, args.baud, args.seconds)
    g_meas = raw.mean(axis=0)
    g_unit = g_meas / np.linalg.norm(g_meas)
    spread = raw.std(axis=0) / (np.abs(raw.mean(axis=0)) + 1e-6)

    # The accelerometer at rest measures +1g of reaction (opposite gravity). The gravity
    # DIRECTION in the sensor frame is therefore -g_unit; we want it to map to [0,0,-1].
    grav_dir_imu = -g_unit
    q = shortest_arc_quat(grav_dir_imu, np.array([0.0, 0.0, -1.0]))

    print(f"\nsamples:            {len(raw)}")
    print(f"mean accel (raw):   [{g_meas[0]:+.1f} {g_meas[1]:+.1f} {g_meas[2]:+.1f}]  (per-axis spread {spread.round(3)})")
    print(f"gravity dir in IMU: [{grav_dir_imu[0]:+.3f} {grav_dir_imu[1]:+.3f} {grav_dir_imu[2]:+.3f}]")
    print("\n--- paste into the daemon config's \"imu\" block ---")
    print(f'  "mounting_rotation": [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]')
    print("---------------------------------------------------")
    tilt_deg = np.degrees(np.arccos(np.clip(-grav_dir_imu[2], -1, 1)))
    print(f"\n(roll+pitch mounting tilt ≈ {tilt_deg:.1f}° from vertical)")
    print("YAW is NOT set by this — if the IMU's X axis isn't robot-forward, compose a")
    print("yaw rotation about Z by hand and verify by yawing the robot and checking the")
    print("angular_velocity Z sign in telemetry.")


if __name__ == "__main__":
    main()
