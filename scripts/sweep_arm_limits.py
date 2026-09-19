#!/usr/bin/env python3
"""Measure each arm joint's REAL mechanical range by hand-sweeping it.

WHY: the arm position_limits in the studio config are inherited from the URDF, and the URDF
describes the ORIGINAL arms. After a rebuild the real travel can differ. On 2026-09-19 both
shoulder_yaw joints were measured travelling 92.6-96.4 deg between two held poses through a
range the URDF calls 90 deg — a violation no calibration value can fix, because moving the
zero only slides the overflow from one end to the other. Only measurement settles it.

READ-ONLY. Polls the web server's /api/status; it never commands the robot and never opens a
CAN socket, so it is safe to run alongside the daemon.

The arm must be IDLE (zero torque, hand-movable) — connect in the web UI first, which leaves
every joint IDLE. Note the arm will NOT hold itself up in most poses; support it as you sweep.

Usage:
    python scripts/sweep_arm_limits.py                      # runs until Ctrl-C
    python scripts/sweep_arm_limits.py --out /tmp/sweep.txt # also tee a live snapshot

Sweep one joint at a time, slowly, to both mechanical stops. Speed is not the enemy —
ENCODER WRAP is: the AS5600 is single-turn behind 15:1 gearing, so a fast or jerky move can
skip. Any sample-to-sample jump larger than --jump deg is counted and flagged; if a joint
shows jumps, its min/max is not trustworthy and that joint needs re-sweeping.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request

DEG = 180.0 / math.pi

# Joints with NO mechanical hardstop — they spin freely, so a "range" is meaningless and
# sweeping them only wraps the single-turn encoder. Tracked but reported as FREE, never
# compared against a configured limit.
FREE_SPINNING = ("wrist_yaw",)


def is_free(name: str) -> bool:
    return any(name.endswith(f"{t}_joint") for t in FREE_SPINNING)


def fetch(url: str, timeout: float = 2.0) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        body = json.loads(r.read().decode())
    if not body.get("success"):
        raise RuntimeError(body.get("error") or "status call failed")
    return body["data"]


def joints_of(data: dict) -> dict:
    js = data.get("joints") or {}
    if isinstance(js, dict):
        return js
    return {j.get("name"): j for j in js}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://127.0.0.1:8000")
    ap.add_argument("--hz", type=float, default=25.0)
    ap.add_argument("--jump", type=float, default=15.0,
                    help="sample-to-sample delta (deg) that counts as a suspected encoder skip")
    ap.add_argument("--out", default=None, help="path for a live snapshot file")
    args = ap.parse_args()

    status_url = f"{args.url.rstrip('/')}/api/status"
    try:
        data = fetch(status_url)
    except Exception as exc:
        print(f"cannot reach {status_url}: {exc}", file=sys.stderr)
        return 2

    if data.get("state") != "CONNECTED":
        print(f"state is {data.get('state')} — connect in the web UI first.", file=sys.stderr)
        return 2

    cfg_limits = {}
    for n, j in joints_of(data).items():
        lim = j.get("limit") or {}
        if isinstance(lim.get("min"), (int, float)):
            cfg_limits[n] = (lim["min"] * DEG, lim["max"] * DEG)

    names = sorted((n for n in joints_of(data) if not is_free(n)),
                   key=lambda n: (not n.startswith("left"), n))
    skipped = sorted(n for n in joints_of(data) if is_free(n))
    lo = {n: float("inf") for n in names}
    hi = {n: float("-inf") for n in names}
    prev = {}
    jumps = {n: 0 for n in names}
    n_samples = 0
    started = time.time()
    period = 1.0 / max(args.hz, 1.0)

    def snapshot() -> str:
        out = []
        out.append(f"sweep running {time.time()-started:7.1f}s   samples {n_samples}")
        out.append("")
        out.append(f"{'joint':24s} {'min°':>9s} {'max°':>9s} {'range°':>9s} | "
                   f"{'config limits°':>20s} {'cfg range°':>11s} | {'skips':>6s}")
        out.append("-" * 100)
        for n in names:
            if lo[n] == float("inf"):
                out.append(f"{n.replace('_joint',''):24s}  (no samples yet)")
                continue
            rng = hi[n] - lo[n]
            c = cfg_limits.get(n)
            cs = f"[{c[0]:8.2f},{c[1]:8.2f}]" if c else "        n/a         "
            cr = f"{c[1]-c[0]:11.2f}" if c else "        n/a"
            flag = "  <-- skips" if jumps[n] else ""
            out.append(f"{n.replace('_joint',''):24s} {lo[n]:9.2f} {hi[n]:9.2f} {rng:9.2f} | "
                       f"{cs:>20s} {cr} | {jumps[n]:6d}{flag}")
        out.append("")
        if skipped:
            out.append("not swept (free-spinning, no hardstop): "
                       + ", ".join(s.replace("_joint", "") for s in skipped))
            out.append("")
        out.append("range° is what YOU swept. Compare against cfg range°: bigger means the")
        out.append("configured limit is too tight for this arm; smaller just means you did not")
        out.append("reach both stops on that joint yet.")
        return "\n".join(out)

    if skipped:
        print("ignoring free-spinning joints: "
              + ", ".join(s.replace("_joint", "") for s in skipped))
    print("sweeping — move one joint at a time to both mechanical stops. Ctrl-C to stop.")
    try:
        while True:
            t0 = time.time()
            try:
                d = fetch(status_url)
            except Exception:
                time.sleep(period)
                continue
            for n, j in joints_of(d).items():
                p = j.get("position")
                if not isinstance(p, (int, float)):
                    continue
                p *= DEG
                if n in prev and abs(p - prev[n]) > args.jump:
                    jumps[n] = jumps.get(n, 0) + 1
                prev[n] = p
                if n in lo:
                    lo[n] = min(lo[n], p)
                    hi[n] = max(hi[n], p)
            n_samples += 1

            if args.out and n_samples % 10 == 0:
                tmp = args.out + ".tmp"
                with open(tmp, "w") as f:
                    f.write(snapshot() + "\n")
                os.replace(tmp, args.out)

            time.sleep(max(0.0, period - (time.time() - t0)))
    except KeyboardInterrupt:
        pass

    final = snapshot()
    print()
    print(final)
    if args.out:
        with open(args.out, "w") as f:
            f.write(final + "\n")
        print(f"\nsaved: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
