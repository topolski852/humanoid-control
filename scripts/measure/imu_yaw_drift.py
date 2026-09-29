#!/usr/bin/env python3
"""IMU heading drift at rest — how long can a heading loop trust the IMU's yaw?

    python scripts/measure/imu_yaw_drift.py --seconds 300 --label squat

The robot must not rotate about the vertical axis: resting, squatting, on a stand. Motor mode
does not matter. PASSIVE: it polls the web service's /api/status over HTTP (~20 Hz) and commands
nothing. It does not open a DaemonClient, which would steal the telemetry the web service runs
on.

The 6-axis IMU has no magnetometer, so its yaw is integrated gyro and must drift. Reported:

* fused yaw from the IMU quaternion ([w, x, y, z], base_state.quat_rotate_inverse convention):
  total change, linear drift rate (deg/min), residual wander
* gyro-z bias and its integral, which is the alternative heading source
* joint motion over the window, to prove the robot really was still

The heading loop in the robot-PC brief uses the fused yaw. The walks veer 6-11 deg/s, so drift
matters only if it approaches that scale over a walk's length.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

URL = "http://127.0.0.1:8000/api/status"


def yaw_deg(q):
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--label", default="rest")
    args = ap.parse_args()

    rows = []
    t0 = time.time()
    next_t = t0
    print(f"recording {args.seconds:.0f} s of IMU at ~{args.hz:.0f} Hz — robot must not rotate", flush=True)
    last_print = t0
    while time.time() - t0 < args.seconds:
        try:
            with urllib.request.urlopen(URL, timeout=1.0) as r:
                d = json.loads(r.read())["data"]
            b = d.get("base") or {}
            q = b.get("quaternion")
            if q and len(q) == 4:
                rows.append({"t": time.time() - t0, "q": q, "gyro": b.get("angular_velocity"),
                             "pg": b.get("projected_gravity"),
                             "pos": [j.get("position") for j in d.get("joints", [])[:12]],
                             "state": d.get("state")})
        except Exception:
            pass
        if time.time() - last_print > 30:
            last_print = time.time()
            if rows:
                y = yaw_deg(rows[-1]["q"]) - yaw_deg(rows[0]["q"])
                print(f"  t={rows[-1]['t']:5.0f}s  yaw change so far {y:+.3f}°", flush=True)
        next_t += 1.0 / args.hz
        time.sleep(max(0.0, next_t - time.time()))

    if len(rows) < 20:
        print("no IMU data from /api/status", file=sys.stderr)
        return 1
    t = np.array([r["t"] for r in rows])
    yaw = np.unwrap(np.radians([yaw_deg(r["q"]) for r in rows]))
    yaw = np.degrees(yaw - yaw[0])
    slope, icpt = np.polyfit(t / 60.0, yaw, 1)
    resid = yaw - (slope * t / 60.0 + icpt)
    gz = np.array([(r["gyro"] or [0, 0, 0])[2] for r in rows], dtype=float)
    gyro_int = np.degrees(np.cumsum(gz[:-1] * np.diff(t)))
    pos = np.array([[p if p is not None else np.nan for p in r["pos"]] for r in rows], dtype=float)
    joint_ptp = np.degrees(np.nanmax(pos, 0) - np.nanmin(pos, 0))
    pg = np.array([r["pg"] for r in rows if r["pg"]], dtype=float)
    tilt = np.degrees(np.arctan2(np.hypot(pg[:, 0], pg[:, 1]), -pg[:, 2])) if len(pg) else np.array([np.nan])

    res = {
        "samples": len(rows), "span_s": float(t[-1]), "rate_hz": float(len(rows) / t[-1]),
        "fused_yaw": {"total_change_deg": float(yaw[-1]), "drift_deg_per_min": float(slope),
                      "residual_std_deg": float(resid.std()), "peak_to_peak_deg": float(np.ptp(yaw))},
        "gyro_z": {"bias_deg_s": float(np.degrees(gz.mean())), "std_deg_s": float(np.degrees(gz.std())),
                   "integrated_change_deg": float(gyro_int[-1]) if len(gyro_int) else None},
        "stillness": {"max_joint_ptp_deg": float(np.nanmax(joint_ptp)),
                      "tilt_ptp_deg": float(np.ptp(tilt)), "states": sorted({r["state"] for r in rows})},
    }
    f = res["fused_yaw"]; g = res["gyro_z"]; s = res["stillness"]
    print(f"\n{res['samples']} samples over {res['span_s']:.0f} s ({res['rate_hz']:.1f} Hz)")
    print(f"FUSED YAW   total {f['total_change_deg']:+.3f}°   drift {f['drift_deg_per_min']:+.3f} °/min   "
          f"wander std {f['residual_std_deg']:.3f}°   p2p {f['peak_to_peak_deg']:.3f}°")
    print(f"GYRO Z      bias {g['bias_deg_s']:+.4f} °/s   std {g['std_deg_s']:.3f} °/s   integrated "
          f"{g['integrated_change_deg']:+.2f}°")
    print(f"STILLNESS   max joint p2p {s['max_joint_ptp_deg']:.2f}°   tilt p2p {s['tilt_ptp_deg']:.3f}°   "
          f"states {s['states']}")
    print(f"\nagainst the walks' 6-11 °/s veer: fused drift is "
          f"{abs(f['drift_deg_per_min']) / 60:.4f} °/s")
    out = C.default_outdir() / f"imu_yaw_drift_{time.strftime('%Y%m%dT%H%M%S')}_{args.label}.json"
    C.write_json(out, {"_meta": C.finish_meta(C.capture_meta("rest", measurement="imu_yaw_drift",
                                                              label=args.label, seconds=args.seconds)),
                       "result": res})
    print(f"wrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
