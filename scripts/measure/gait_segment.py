#!/usr/bin/env python3
"""Find the WALKING window inside a capture that is mostly arming and standing.

    python scripts/measure/gait_segment.py --capture docs/measurements/..._can.json

A walk attempt is a few seconds of gait buried in arm-up, ramp-to-pose, stand, a push, and
whatever happens after. Averaging M1-M6 over the whole file dilutes the part that matters, so
this classifies each second and reports the walking span separately.

### How a step is told apart from a wobble

Knee pitch is the discriminator, and the tell is the LEFT/RIGHT CORRELATION, not amplitude:

  * **stepping**  knees alternate -> correlation strongly NEGATIVE (sim gives -0.84..-0.92)
  * **sagging**   knees move together -> correlation POSITIVE (the 6.0 Nm torque-starved
                  failure measured +0.91 on this robot)
  * **standing**  knees barely move at all

So a window is "walking" when the knees are both moving appreciably AND anti-correlated. A
window where they move together is reported as SAGGING, which is a different failure and must
not be scored as a bad walk — it is the signature of running out of knee torque.

Gait frequency comes from the dominant FFT peak of knee pitch, for comparison against the
deployed policy's ~1.5-1.6 Hz sim gait.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

FS = 100.0                # PDO4 rate
WIN_S = 1.0
MOVE_CT = 25.0            # knee excursion (encoder counts) above which a window is "moving"
ANTI = -0.3               # correlation below this = alternating
TOGETHER = 0.3            # correlation above this = sagging together


def load(capture_json: Path):
    side = Path(str(capture_json).replace(".json", ".frames.jsonl"))
    if not side.exists():
        raise SystemExit(f"missing frames sidecar: {side}")
    d = {}
    for line in side.read_text().splitlines():
        if line.strip():
            j = json.loads(line)
            d[j["joint"]] = j
    return d


def resample(d, joint, grid):
    """Nearest-sample onto a common time grid (frames are ~100 Hz but not phase-aligned)."""
    ts = np.asarray(d[joint]["ts"])
    pos = np.asarray(d[joint]["pos"])
    idx = np.clip(np.searchsorted(ts, grid), 0, len(ts) - 1)
    return pos[idx]


def dominant_hz(x):
    x = x - x.mean()
    if x.size < 16 or not np.any(x):
        return None
    f = np.fft.rfftfreq(x.size, 1.0 / FS)
    a = np.abs(np.fft.rfft(x))
    band = (f > 0.3) & (f < 6.0)
    if not band.any() or not a[band].any():
        return None
    return float(f[band][np.argmax(a[band])])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    cap = Path(args.capture)
    d = load(cap)
    lk, rk = "left_knee_pitch_joint", "right_knee_pitch_joint"
    if lk not in d or rk not in d:
        print("knee data missing", file=sys.stderr)
        return 1

    t0 = max(d[j]["ts"][0] for j in d)
    t1 = min(d[j]["ts"][-1] for j in d)
    grid = np.arange(t0, t1, 1.0 / FS)
    L, R = resample(d, lk, grid), resample(d, rk, grid)

    w = int(WIN_S * FS)
    n = len(grid) // w
    rows = []
    for i in range(n):
        a, b = L[i * w:(i + 1) * w], R[i * w:(i + 1) * w]
        la = (a.max() - a.min()) / C.JOINT_QUANTUM_RAD
        ra = (b.max() - b.min()) / C.JOINT_QUANTUM_RAD
        corr = float(np.corrcoef(a, b)[0, 1]) if a.std() > 0 and b.std() > 0 else 0.0
        moving = la > MOVE_CT and ra > MOVE_CT
        if moving and corr < ANTI:
            state = "WALK"
        elif moving and corr > TOGETHER:
            state = "SAG"
        elif moving:
            state = "move"
        else:
            state = "stand"
        rows.append({"t_rel": i * WIN_S, "state": state, "corr": corr,
                     "left_ct": la, "right_ct": ra,
                     "hz": dominant_hz(a) if moving else None})

    print(f"{'t(s)':>6s} {'state':>6s} {'knee_corr':>10s} {'L_ct':>7s} {'R_ct':>7s} {'gait_Hz':>8s}")
    for r in rows:
        print(f"{r['t_rel']:>6.0f} {r['state']:>6s} {r['corr']:>10.2f} "
              f"{r['left_ct']:>7.0f} {r['right_ct']:>7.0f} "
              f"{(f'{r[chr(104)+chr(122)]:.2f}' if r['hz'] else '-'):>8s}")

    walk = [r for r in rows if r["state"] == "WALK"]
    sag = [r for r in rows if r["state"] == "SAG"]
    print(f"\nwindows: {len(rows)} total | {len(walk)} WALK | {len(sag)} SAG | "
          f"{sum(1 for r in rows if r['state']=='stand')} stand")
    if walk:
        sp = [r["t_rel"] for r in walk]
        hz = [r["hz"] for r in walk if r["hz"]]
        print(f"WALK span: t={min(sp):.0f}-{max(sp)+WIN_S:.0f}s ({len(walk)}s of gait)")
        print(f"  knee corr mean {np.mean([r['corr'] for r in walk]):+.2f} "
              f"(sim gait gives -0.84..-0.92)")
        if hz:
            print(f"  gait {np.mean(hz):.2f} Hz (deployed sim gait ~1.5-1.6 Hz)")
    else:
        print("NO WALK WINDOWS — the knees never alternated.")
    if sag:
        c = np.mean([r["corr"] for r in sag])
        if walk:
            print(f"!! {len(sag)}s in-phase knees (corr {c:+.2f}) alongside {len(walk)}s of "
                  f"gait — torque-starvation signature; compare against the 6.0 Nm cap result "
                  f"(+0.91)")
        else:
            print(f"   {len(sag)}s in-phase knees (corr {c:+.2f}) with NO gait in this capture "
                  f"— consistent with push/disturbance response, not torque starvation")

    out = Path(args.out) if args.out else cap.with_name(
        cap.stem.replace("_can", "") + "_gait.json")
    C.write_json(out, {"_meta": {"capture": str(cap), "window_s": WIN_S},
                       "windows": rows,
                       "walk_windows": len(walk), "sag_windows": len(sag)})
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
