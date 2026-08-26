#!/usr/bin/env python3
"""TEMPORARY — summarise an arm teleop run log.

Answers the question "why did the arm not move the way I expected" from the numbers: what the
sticks asked for, how much of it the hand actually got, and what stopped the rest.

Usage:
    python scripts/tmp_read_arm_run.py                  # newest log
    python scripts/tmp_read_arm_run.py <file.jsonl>
    python scripts/tmp_read_arm_run.py --trace          # per-tick dump of the engaged frames
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

DEG = 180.0 / math.pi
STICKS = ("left_x", "left_y", "right_y", "right_x")
MEANS = ("azimuth", "elevation", "reach", "wrist")


def newest(root: str) -> str | None:
    files = glob.glob(os.path.join(root, "**", "arm_*.jsonl"), recursive=True)
    return max(files, key=os.path.getmtime) if files else None


def load(path: str):
    meta, rows, summary = {}, [], {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            if "_meta" in d:
                meta = d["_meta"]
            elif "_summary" in d:
                summary = d["_summary"]
            else:
                rows.append(d)
    return meta, rows, summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?")
    ap.add_argument("--dir", default="_arm_recording/runs")
    ap.add_argument("--trace", action="store_true", help="per-tick dump of engaged frames")
    ap.add_argument("--max-trace", type=int, default=60)
    args = ap.parse_args()

    path = args.path or newest(args.dir)
    if not path or not os.path.exists(path):
        sys.exit(f"no run log found (looked in {args.dir}/)")
    meta, rows, summary = load(path)
    if not rows:
        sys.exit(f"{path} has no frames")

    joints = meta.get("joint_order", [])
    short = [j.replace("left_", "").replace("right_", "").replace("_joint", "") for j in joints]
    eng = [r for r in rows if r.get("engaged")]

    print("=" * 78)
    print(f"{os.path.basename(path)}   limb={meta.get('limb')}  frame={eng[0].get('frame') if eng else '?'}")
    print("=" * 78)
    print(f"  {len(rows)} frames over {rows[-1]['t']:.1f}s   engaged {len(eng)} "
          f"({len(eng) / max(1, len(rows)) * 100:.0f}%)")
    if summary:
        print(f"  dropped frames: {summary.get('dropped_frames', 0)}")
    if not eng:
        print("\n  NEVER ENGAGED — a trigger was never held, so nothing was commanded.")
        return 0

    # --- what the sticks asked for ---------------------------------------
    print("\nSTICK INPUT (engaged frames only)")
    print(f"  {'axis':10s}{'means':11s}{'held':>7s}{'peak':>8s}{'mean|x|':>9s}")
    for i, (name, mean) in enumerate(zip(STICKS, MEANS)):
        vals = [r["sticks"][i] for r in eng if len(r.get("sticks", [])) > i]
        if not vals:
            continue
        held = sum(1 for v in vals if abs(v) > 0.15) / len(vals) * 100
        peak = max(vals, key=abs)
        print(f"  {name:10s}{mean:11s}{held:6.0f}%{peak:+8.2f}"
              f"{sum(abs(v) for v in vals) / len(vals):9.2f}")

    # --- what the arm did about it ---------------------------------------
    print("\nWHAT THE ARM DID")
    sp = [r["spherical"] for r in eng if r.get("spherical")]
    if sp:
        for key, label, unit in (("elevation_deg", "elevation", "deg"),
                                 ("azimuth_deg", "azimuth", "deg"),
                                 ("reach_m", "reach", "m")):
            v = [s[key] for s in sp]
            scale = 100 if unit == "m" else 1
            u = "cm" if unit == "m" else "deg"
            print(f"  {label:10s} {v[0]*scale:+7.1f} -> {v[-1]*scale:+7.1f} {u}"
                  f"   (range {min(v)*scale:+.1f} .. {max(v)*scale:+.1f})")
    errs = [r["error_m"] for r in eng if r.get("error_m") is not None]
    if errs:
        print(f"  tracking error: mean {sum(errs)/len(errs)*1000:.1f} mm, "
              f"worst {max(errs)*1000:.1f} mm")
    clip = sum(1 for r in eng if r.get("clipped"))
    print(f"  target clipped (leash / workspace edge): {clip}/{len(eng)} frames "
          f"({clip/len(eng)*100:.0f}%)")

    # --- the usual culprit ------------------------------------------------
    hits = summary.get("ticks_at_limit") or {}
    print("\nJOINTS PINNED AT A LIMIT  (the usual reason a command goes nowhere)")
    if not hits:
        print("  none — no joint hit a stop")
    for n, c in hits.items():
        print(f"  {n.replace('_joint',''):26s} {c:5d} frames  ({c/len(eng)*100:5.1f}% of engaged)")

    # --- per-joint travel --------------------------------------------------
    print("\nPER-JOINT TRAVEL (deg)")
    print(f"  {'joint':18s}{'start':>8s}{'end':>8s}{'min':>8s}{'max':>8s}{'moved':>8s}")
    for i, nm in enumerate(short):
        v = [r["joint_pos"][i] * DEG for r in eng if len(r.get("joint_pos", [])) > i]
        if not v:
            continue
        print(f"  {nm:18s}{v[0]:+8.1f}{v[-1]:+8.1f}{min(v):+8.1f}{max(v):+8.1f}"
              f"{max(v)-min(v):8.1f}")

    # --- commanded vs achieved --------------------------------------------
    print("\nCOMMANDED vs ACHIEVED (did the joint go where it was told?)")
    print(f"  {'joint':18s}{'mean |target-actual|':>22s}{'worst':>9s}")
    for i, nm in enumerate(short):
        d = [abs(r["joint_target"][i] - r["joint_pos"][i]) * DEG
             for r in eng if r.get("joint_target") and len(r["joint_target"]) > i]
        if not d:
            continue
        flag = "   <-- not tracking" if sum(d)/len(d) > 3.0 else ""
        print(f"  {nm:18s}{sum(d)/len(d):22.2f}{max(d):9.2f}{flag}")

    if args.trace:
        print("\nPER-TICK (engaged frames)")
        print(f"  {'t':>6s} {'sticks(lx,ly,ry,rx)':>26s} {'elev':>7s}{'azim':>7s}{'reach':>7s}"
              f"  {'err':>6s}  limits")
        for r in eng[:args.max_trace]:
            s = r.get("spherical") or {}
            st = " ".join(f"{v:+5.2f}" for v in r.get("sticks", []))
            print(f"  {r['t']:6.2f} {st:>26s} {s.get('elevation_deg',0):7.1f}"
                  f"{s.get('azimuth_deg',0):7.1f}{s.get('reach_m',0)*100:7.1f}"
                  f"  {(r.get('error_m') or 0)*1000:6.1f}  "
                  f"{','.join(n.replace('left_','').replace('_joint','') for n in (r.get('at_limit') or []))}")
        if len(eng) > args.max_trace:
            print(f"  ... {len(eng)-args.max_trace} more frames (raise --max-trace)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
