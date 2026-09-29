#!/usr/bin/env python3
"""Learn the LEFT/RIGHT sign convention by watching the operator move the legs by hand.

    python scripts/measure/handmove_frames.py --seconds 45

Passive only: reads a copy of the CAN stream, commands nothing, and needs the joints IDLE so they
move freely by hand. It cannot move the robot.

### Why this exists

I reasoned about whether the same device-frame command moves both legs the same way in the world,
got it backwards, and swept the hips into the diverge-then-converge motion the operator had
explicitly warned about. The verification I used — that `error x tau_gravity` was consistently
negative on both sides — is blind to exactly this error: a sign flip on one side flips both terms,
leaving the product unchanged. It proved the physics self-consistent and said nothing about the
left/right relationship.

The robot can answer directly. Move both legs the same way in the world and read the reported
signs:

  * reported positions move the SAME sign  -> same device sign = same world direction
  * reported positions move OPPOSITE signs -> same device sign = MIRRORED (diverge/converge),
    so moving them together needs OPPOSITE device signs

Per joint pair, this reports the correlation of the two sides' motion and the sign of their
displacement, so the convention is measured rather than derived. It also reports the FK-predicted
foot separation over the recording, which is the second thing I got wrong and which this
validates against physical reality.

### Suggested protocol

1. Start it, then for a few seconds move BOTH legs the same way in the world — e.g. both swung to
   the operator's left, or both forward.
2. Return to hanging, pause a moment.
3. Then move them MIRRORED — apart, then together.

The first phase gives the convention. The second confirms it reads the opposite sign, and shows
what the earlier sweep actually did to the robot.
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402
import leg_gravity as LG  # noqa: E402

LINE = re.compile(r"\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s*([0-9A-Fa-f ]*)")
TYPES = ["hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"]
MOVE_RAD = np.radians(3.0)      # a pair is "moving" once both sides exceed this


def capture(chans, seconds):
    nodes = C.leg_node_map()
    proc = subprocess.Popen(["candump", "-t", "a"] + chans, stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, bufsize=1)
    ts, pos = defaultdict(list), defaultdict(list)
    end = time.time() + seconds
    try:
        for line in proc.stdout:
            if time.time() >= end:
                break
            m = LINE.search(line)
            if not m:
                continue
            arb = int(m.group(3), 16)
            name = nodes.get((m.group(2), arb & C.NODE_MASK))
            if name is None or ((arb >> C.FUNC_SHIFT) & C.FUNC_MASK) != C.FUNC_PDO4:
                continue
            raw = bytes.fromhex(m.group(5).replace(" ", ""))
            if len(raw) < 8:
                continue
            ts[name].append(float(m.group(1)))
            pos[name].append(struct.unpack("<ff", raw[:8])[0])
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    return dict(ts), dict(pos)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=45.0)
    args = ap.parse_args()
    chans = C.leg_channels()
    if not chans:
        print("no leg CAN interfaces are UP.", file=sys.stderr)
        return 1

    print(f"recording {args.seconds:.0f}s — move the legs by hand now.")
    print("  1) both legs the SAME way in the world, a few seconds")
    print("  2) back to hanging, pause")
    print("  3) then MIRRORED — apart, then together\n")
    ts, pos = capture(chans, args.seconds)
    if not pos:
        print("no PDO4 frames captured.", file=sys.stderr)
        return 1

    # common time grid
    t0 = max(v[0] for v in ts.values())
    t1 = min(v[-1] for v in ts.values())
    grid = np.arange(t0, t1, 0.01)
    P = {}
    for j in ts:
        a = np.asarray(ts[j]); p = np.asarray(pos[j])
        P[j] = p[np.clip(np.searchsorted(a, grid), 0, len(a) - 1)]

    print(f"{len(grid)} samples over {t1-t0:.1f}s\n")
    print(f"{'pair':14s} {'L range':>10s} {'R range':>10s} {'corr':>7s}  convention")
    verdicts = {}
    for t in TYPES:
        jl, jr = f"left_{t}_joint", f"right_{t}_joint"
        if jl not in P or jr not in P:
            continue
        L = P[jl] - np.median(P[jl]); R = P[jr] - np.median(P[jr])
        rngL, rngR = L.max() - L.min(), R.max() - R.min()
        if rngL < MOVE_RAD or rngR < MOVE_RAD:
            print(f"{t:14s} {np.degrees(rngL):>9.1f}° {np.degrees(rngR):>9.1f}°       -  "
                  f"not moved enough (need >3° on both)")
            continue
        corr = float(np.corrcoef(L, R)[0, 1])
        verdicts[t] = corr
        if corr > 0.6:
            v = "SAME device sign = SAME world direction"
        elif corr < -0.6:
            v = "SAME device sign = MIRRORED -> pair them with OPPOSITE signs"
        else:
            v = f"unclear (corr {corr:+.2f}) — move them more cleanly together"
        print(f"{t:14s} {np.degrees(rngL):>9.1f}° {np.degrees(rngR):>9.1f}° {corr:>+7.2f}  {v}")

    # FK foot separation from the MEASURED pose — validates the model I got wrong
    model = LG.LegModel()
    sep = []
    step = max(len(grid) // 200, 1)
    for i in range(0, len(grid), step):
        q = {LG.urdf_name(j): float(P[j][i]) for j in P}
        T, _ = model.fk(q)
        try:
            pl = T["leg_left_ankle_roll"][1] + T["leg_left_ankle_roll"][0] @ model.com["leg_left_ankle_roll"]
            pr = T["leg_right_ankle_roll"][1] + T["leg_right_ankle_roll"][0] @ model.com["leg_right_ankle_roll"]
            sep.append(float(np.linalg.norm(pl - pr)))
        except KeyError:
            break
    if sep:
        s = np.array(sep)
        print(f"\nFK foot separation from the MEASURED pose: "
              f"min {s.min():.3f} m  median {np.median(s):.3f}  max {s.max():.3f}")
        print("  (if this tracks what you saw, the FK model is sound and only my sign was wrong)")

    out = C.default_outdir() / f"handmove_{time.strftime('%Y%m%dT%H%M%S')}.json"
    C.write_json(out, {
        "_meta": C.finish_meta(C.capture_meta("handmove", measurement="LR_sign_convention",
                                              method="passive candump, joints IDLE, operator moves by hand")),
        "correlations": verdicts,
        "foot_separation_m": {"min": float(min(sep)), "max": float(max(sep)),
                              "median": float(np.median(sep))} if sep else None,
        "ranges_deg": {j: float(np.degrees(P[j].max() - P[j].min())) for j in P},
    })
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
