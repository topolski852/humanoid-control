#!/usr/bin/env python3
"""E-stop the robot if a leg joint's reading freezes while it is being driven.

    python scripts/measure/freeze_guard.py            # run alongside any policy session

2026-09-29: right_knee_pitch's position froze bit-identical for 37.7 s during a walk, with no
EMCY until the end. Nothing noticed: the policy kept driving against a dead sensor and the ESC sat
at its torque cap 97% of the time. This guard closes that gap from outside the control path.

Rule, per leg joint, while the service is HOLDING or RUNNING: the position is bit-identical for
>= 0.5 s AND the last commanded target is > 0.15 rad away. Then POST /api/estop (all joints to
DAMPING) and exit. A healthy joint under load changes reading nearly every tick, and one that is
genuinely still is sitting on its target, so this does not false-trigger. Replayed over every tick
log from 2026-09-29, it fires on the frozen-knee run at t=5.64 s (0.52 s after the freeze) and
stays quiet on all seven healthy runs, including two 8-minute stands.

PASSIVE until it fires: it polls /api/status over HTTP (~20 Hz) and never opens a DaemonClient.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request

BASE = "http://127.0.0.1:8000"
MOTION = {"HOLDING", "RUNNING"}


def get_status():
    with urllib.request.urlopen(BASE + "/api/status", timeout=1.0) as r:
        return json.loads(r.read())["data"]


def estop():
    req = urllib.request.Request(BASE + "/api/estop", data=b"{}", method="POST",
                                 headers={"content-type": "application/json"})
    with urllib.request.urlopen(req, timeout=2.0) as r:
        return r.status


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hold", type=float, default=0.5, help="seconds bit-identical")
    ap.add_argument("--gap", type=float, default=0.15, help="rad from target")
    ap.add_argument("--hz", type=float, default=20.0)
    args = ap.parse_args()
    last: dict[str, tuple[float, float]] = {}      # joint -> (position, time it last changed)
    print(f"freeze guard armed: hold {args.hold}s, gap {args.gap} rad", flush=True)
    while True:
        now = time.monotonic()
        try:
            d = get_status()
        except Exception:
            time.sleep(0.2)
            continue
        if d.get("state") not in MOTION:
            last.clear()
            time.sleep(1.0 / args.hz)
            continue
        for j in d.get("joints", [])[:12]:
            name, pos, tgt = j.get("name"), j.get("position"), j.get("target")
            if pos is None:
                continue
            prev = last.get(name)
            if prev is None or pos != prev[0]:
                last[name] = (pos, now)
                continue
            frozen_for = now - prev[1]
            if frozen_for >= args.hold and tgt is not None and abs(tgt - pos) > args.gap:
                msg = (f"FROZEN READING: {name} at {pos:+.4f} for {frozen_for:.2f}s, target "
                       f"{tgt:+.4f} ({abs(tgt - pos):.3f} rad away) -> E-STOP")
                print(time.strftime("%H:%M:%S"), msg, flush=True)
                try:
                    print("estop:", estop(), flush=True)
                except Exception as exc:
                    print("ESTOP REQUEST FAILED:", exc, flush=True)
                return 3
        time.sleep(max(0.0, 1.0 / args.hz - (time.monotonic() - now)))


if __name__ == "__main__":
    raise SystemExit(main())
