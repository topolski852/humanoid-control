#!/usr/bin/env python3
"""Standing quality from the policy runner's tick log — the numbers used to rank bundles.

    .venv/bin/python scripts/measure/stand_metrics.py --label measD-stand    # newest tick log
    python scripts/measure/stand_metrics.py --recording recordings/run_X.jsonl --label measC-stand
    python scripts/measure/stand_metrics.py --compare A_stand.json B_stand.json

The web service writes one tick log per policy session to `recordings/` (HUMANOID_RECORD_DIR),
25 Hz, including the IMU's `projected_gravity`. The passive CAN capture has no IMU, so tilt comes
from here and push-ring from `settle_analysis.py` on the CAN capture.

This is the code that produced REPORT_2026-09-28_measC.md §1, lifted verbatim so every bundle is
scored the same way. Two details matter for reproducing it:

* tilt = atan2(|g_xy|, -g_z), which does not assume |g| = 1. arccos(-g_z) does, and reads
  measC's untouched window as 2.94 deg instead of 2.71.
* tilt rate = np.gradient over the whole log at the median dt, then |.| p95 per phase.

Phases are seconds from the first tick of the log (policy engage), matching the protocol in
policies/walk_measD-fast_2026-09-28/NOTE.md: settle 0-120, UNTOUCHED 120-300, pushes 300+.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

PHASES = [("settle", 0, 120), ("untouched", 120, 300), ("pushes", 300, 1e9)]
EXC_DEG = 8.0


def load(path):
    t, pg, jv = [], [], []
    for ln in open(path):
        ln = ln.strip()
        if not ln:
            continue
        try:
            d = json.loads(ln)
        except Exception:
            continue
        if "_meta" in d or "_dropped_frames" in d:
            continue
        if isinstance(d.get("projected_gravity"), list):
            t.append(d["t"])
            pg.append(d["projected_gravity"])
            jv.append(d.get("joint_vel") or [np.nan] * 12)
    return np.array(t), np.array(pg), np.array(jv, dtype=float)


def excursions(arr):
    n, i = 0, 0
    while i < len(arr):
        if arr[i] > EXC_DEG:
            j = i
            while j < len(arr) and arr[j] > EXC_DEG:
                j += 1
            n += 1
            i = j
        else:
            i += 1
    return n


def metrics(path):
    t, g, jv = load(path)
    tilt = np.degrees(np.arctan2(np.hypot(g[:, 0], g[:, 1]), -g[:, 2]))
    dt = float(np.median(np.diff(t)))
    rate = np.gradient(tilt, dt)
    out = {"recording": os.path.basename(path), "ticks": int(len(t)),
           "span_s": float(t[-1] - t[0]), "dt_ms": dt * 1000, "phases": {}}
    for name, lo, hi in PHASES:
        m = (t - t[0] >= lo) & (t - t[0] < hi)
        if m.sum() < 20:
            out["phases"][name] = None
            continue
        ti = tilt[m]
        exc = excursions(ti)
        mins = (min(hi, t[-1] - t[0]) - lo) / 60.0
        out["phases"][name] = {
            "window_s": [lo, float(min(hi, t[-1] - t[0]))], "n": int(m.sum()),
            "tilt_med_deg": float(np.median(ti)), "tilt_p95_deg": float(np.percentile(ti, 95)),
            "tilt_max_deg": float(ti.max()),
            "tilt_rate_p95_deg_s": float(np.percentile(np.abs(rate[m]), 95)),
            "excursions_over_8deg": exc, "excursions_per_min": exc / mins if mins > 0 else None,
            "joint_vel_p99_rad_s": float(np.nanpercentile(np.abs(jv[m]), 99)),
        }
    return out


def identify_policy(path, n=400):
    """Which bundle produced this log? Replay the logged observations through every staged
    ONNX and compare against the logged actions. The right network reproduces them exactly
    (max |delta| 0 on measC's log); a different one misses by ~0.1-0.3 rad on average. The
    service does not log which checkpoint a session loaded, and the dropdown default ("walk")
    is a different network from measC and measD, so an A/B cannot trust the UI selection."""
    try:
        import onnxruntime as ort
    except ImportError:
        return {"error": "onnxruntime not importable -- run with .venv/bin/python"}
    O, A = [], []
    for ln in open(path):
        try:
            d = json.loads(ln)
        except Exception:
            continue
        if d.get("obs") is not None and d.get("action") is not None:
            O.append(d["obs"]); A.append(d["action"])
        if len(O) >= n:
            break
    O = np.asarray(O, dtype=np.float32); A = np.asarray(A, dtype=np.float32)
    scores = {}
    for onnx in sorted((C.REPO_ROOT / "policies").glob("*/policy.onnx")):
        try:
            s = ort.InferenceSession(str(onnx))
            name = s.get_inputs()[0].name
            if s.get_inputs()[0].shape[-1] != O.shape[1]:
                continue
            out = np.vstack([s.run(None, {name: o[None]})[0] for o in O])
            scores[onnx.parent.name] = float(np.abs(out - A).mean())
        except Exception:
            continue
    best = min(scores, key=scores.get) if scores else None
    return {"policy": best, "mean_abs_action_error": scores.get(best),
            "exact": best is not None and scores[best] < 1e-5, "all": scores}


def show(res, label=""):
    print(f"{label or res['recording']}: {res['ticks']} ticks, {res['span_s']:.1f} s, "
          f"dt {res['dt_ms']:.1f} ms")
    pid = res.get("policy_id") or {}
    if pid.get("policy"):
        print(f"  network: {pid['policy']} ({'EXACT match' if pid['exact'] else 'NOT exact'}, "
              f"mean |Δaction| {pid['mean_abs_action_error']:.1e})")
    for name, p in res["phases"].items():
        if not p:
            print(f"  {name:10s} (too few samples)")
            continue
        print(f"  {name:10s} n={p['n']:5d}  tilt med {p['tilt_med_deg']:5.2f}°  p95 "
              f"{p['tilt_p95_deg']:5.2f}°  max {p['tilt_max_deg']:6.2f}°  | rate p95 "
              f"{p['tilt_rate_p95_deg_s']:6.2f}°/s | exc>8° {p['excursions_over_8deg']} | "
              f"jvel p99 {p['joint_vel_p99_rad_s']:.2f}")


def compare(a, b):
    A, B = json.loads(Path(a).read_text()), json.loads(Path(b).read_text())
    la, lb = A["_meta"].get("label", a), B["_meta"].get("label", b)
    keys = [("tilt_med_deg", "tilt med °"), ("tilt_p95_deg", "tilt p95 °"),
            ("tilt_rate_p95_deg_s", "tilt rate p95 °/s"), ("excursions_over_8deg", "exc >8°"),
            ("joint_vel_p99_rad_s", "joint vel p99")]
    for ph in ("settle", "untouched", "pushes"):
        pa, pb = A["metrics"]["phases"].get(ph), B["metrics"]["phases"].get(ph)
        if not pa or not pb:
            continue
        print(f"\n{ph.upper()}")
        print(f"  {'metric':20s} {la:>18s} {lb:>18s}")
        for k, name in keys:
            print(f"  {name:20s} {pa[k]:>18.3f} {pb[k]:>18.3f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recording", help="tick log; default = newest in recordings/")
    ap.add_argument("--label", help="output name, e.g. measD-stand")
    ap.add_argument("--compare", nargs=2, metavar=("A", "B"))
    args = ap.parse_args()
    if args.compare:
        compare(*args.compare)
        return 0
    path = args.recording or max(glob.glob(str(C.REPO_ROOT / "recordings" / "run_*.jsonl")),
                                 key=os.path.getmtime)
    res = metrics(path)
    res["policy_id"] = identify_policy(path)
    show(res, args.label or "")
    if args.label:
        out = C.default_outdir() / f"{args.label}_stand.json"
        C.write_json(out, {"_meta": C.finish_meta(C.capture_meta(
            "stand", measurement="stand_metrics", label=args.label, recording=res["recording"])),
            "metrics": res})
        print(f"wrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
