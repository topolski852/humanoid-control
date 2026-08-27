#!/usr/bin/env python3
"""Measure what WebXR body tracking actually delivers on this headset.

Stage A of the whole-arm mirroring work: before any retargeting is written, find out
whether the data is good enough to drive a robot arm with. Run it, follow the prompts,
and read the verdict::

    .venv/bin/python scripts/analyse_body_tracking.py            # live, 20 s
    .venv/bin/python scripts/analyse_body_tracking.py --seconds 60

The headline number is SEGMENT-LENGTH STABILITY. Your upper arm and forearm do not change
length, so any variation in the measured distance between shoulder/elbow/wrist is pure
measurement error. It is the one quality metric that needs no ground truth, and it maps
directly onto how much the robot's joints would jitter: a centimetre of drift over a 30 cm
segment is roughly 2 degrees of phantom elbow angle, commanded straight into a motor.

Requires: HUMANOID_QUEST_ENABLE=1, the headset in an immersive session on /xr/, and
"WebXR Experiments" enabled in chrome://flags on the Quest.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

API = "http://127.0.0.1:8000/api/status"
REQUIRED = ("chest", "left-shoulder", "left-arm-upper", "left-arm-lower",
            "left-hand-wrist-twist", "left-hand-wrist")


def status() -> dict:
    with urllib.request.urlopen(API, timeout=3) as r:
        return json.loads(r.read())["data"]


def verdict(label: str, ok: bool, detail: str) -> bool:
    print(f"  {'PASS' if ok else 'FAIL'}  {label:<42} {detail}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=10.0, help="sampling rate of THIS script")
    args = ap.parse_args()

    try:
        d = status()
    except Exception as exc:                                    # noqa: BLE001
        print(f"cannot reach {API}: {exc}\nIs the web server running?")
        return 2

    q = d.get("quest") or {}
    if not q.get("enabled"):
        print("Quest bridge disabled — start the server with HUMANOID_QUEST_ENABLE=1")
        return 2
    if not q.get("connected"):
        print("No headset connected. Open http://localhost:8000/xr/ on the Quest\n"
              "(needs: adb reverse tcp:8000 tcp:8000)")
        return 2

    body = q.get("body") or {}
    if not body.get("available"):
        print("Headset is connected but frame.body is NULL — body tracking is not active.\n"
              "\n  On the Quest: open chrome://flags, enable 'WebXR Experiments', restart\n"
              "  the browser, then re-enter the session on /xr/.\n"
              "\n  Note the page requests body-tracking as OPTIONAL, so the session starts\n"
              "  fine without it — the controller path is unaffected.")
        return 1

    print(f"\nSampling {args.seconds:.0f}s at {args.hz:.0f} Hz.")
    print("HOLD YOUR LEFT ARM STILL in a comfortable driving posture — the point is to\n"
          "measure noise, so movement will look like error.\n")

    samples, t0, period = [], time.monotonic(), 1.0 / args.hz
    while time.monotonic() - t0 < args.seconds:
        try:
            b = (status().get("quest") or {}).get("body") or {}
            if b.get("available"):
                samples.append(b)
        except Exception:                                       # noqa: BLE001
            pass
        left = period - ((time.monotonic() - t0) % period)
        time.sleep(max(0.0, left))
        el = time.monotonic() - t0
        print(f"\r  {el:5.1f}s  {len(samples)} samples", end="", flush=True)
    print("\n")

    if len(samples) < 5:
        print("Too few samples — is the session still running?")
        return 1

    up = np.array([s["upper_arm_m"] for s in samples if s.get("upper_arm_m")])
    fo = np.array([s["forearm_m"] for s in samples if s.get("forearm_m")])
    usable = np.array([bool(s.get("usable")) for s in samples])
    emul = np.array([s.get("emulated") or 0 for s in samples])
    pres = np.array([s.get("present") or 0 for s in samples])

    print("── measurements ─────────────────────────────────────────────────")
    if len(up):
        print(f"  upper arm  mean {up.mean()*100:5.1f} cm   "
              f"spread {(up.max()-up.min())*1000:5.1f} mm   sd {up.std()*1000:4.1f} mm")
    if len(fo):
        print(f"  forearm    mean {fo.mean()*100:5.1f} cm   "
              f"spread {(fo.max()-fo.min())*1000:5.1f} mm   sd {fo.std()*1000:4.1f} mm")
    print(f"  joints present   {pres.mean():.1f} of {len(REQUIRED)} required")
    print(f"  frames emulated  {(emul>0).mean():.0%}")
    print(f"  frames usable    {usable.mean():.0%}   (present AND not emulated)")

    print("\n── go / no-go for whole-arm mirroring ───────────────────────────")
    ok = True
    # A centimetre of drift on a ~30 cm segment is ~2 deg of phantom joint angle. Anything
    # much beyond that gets commanded into a motor as jitter the operator did not produce.
    if len(up):
        ok &= verdict("upper-arm length stable (<10 mm spread)",
                      (up.max()-up.min()) < 0.010, f"{(up.max()-up.min())*1000:.1f} mm")
    if len(fo):
        ok &= verdict("forearm length stable (<10 mm spread)",
                      (fo.max()-fo.min()) < 0.010, f"{(fo.max()-fo.min())*1000:.1f} mm")
    ok &= verdict("required joints usable >95% of frames",
                  usable.mean() > 0.95, f"{usable.mean():.0%}")
    ok &= verdict("emulation rare (<5% of frames)",
                  (emul > 0).mean() < 0.05, f"{(emul>0).mean():.0%}")

    print()
    if ok:
        print("GO — the data is good enough to retarget from. Proceed to calibration.")
    else:
        print("NO-GO — see the failures above.\n"
              "  Noisy segments or frequent emulation mean the mirrored joints would\n"
              "  jitter or jump. Worth re-running in better lighting and with your arm\n"
              "  clearly in view before changing the plan: the Quest infers the upper body\n"
              "  from the headset and controllers, so an arm held behind you or close to\n"
              "  your torso tracks noticeably worse.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
