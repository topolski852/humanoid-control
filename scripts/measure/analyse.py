#!/usr/bin/env python3
"""M2/M3/M6 — offline analysis of a passive CAN capture. Touches nothing live.

    python scripts/measure/analyse.py --capture docs/measurements/hold_..._can.json

Reads the capture written by m1_can_timing.py and derives:

  * **M2 sample age** — for each policy tick, how old is each joint's most recent PDO4 frame.
  * **M3 stale-hold** — ticks where NO new frame arrived for that joint since the previous tick.
    This is a transport fact from real frame gaps, not the "position value repeated" proxy the
    retired sampler used, which saturated whenever the robot stood still.
  * **M6 encoder** — quantisation step and residual noise, from the decoded PDO4 payload at the
    full 100 Hz.
  * **M8 buckets** — the same statistics recomputed over elapsed-time windows.

### The one assumption, stated plainly

Policy tick times are not recorded in the CAN stream, so M2 is evaluated against a **synthetic
25 Hz tick grid**. That is valid for the *distribution* of sample ages — the policy loop and the
motors' PDO4 timers run off independent clocks, so tick phase relative to frame arrival is
uniform, and sampling it on a synthetic grid draws from the same distribution the policy sees.

It would NOT be valid if the two were phase-locked. They are not (separate hardware, no shared
clock, and the measured PDO4 rate of ~100.2 Hz is not an exact multiple of 25 Hz).

If you want exact per-tick ages rather than the distribution, set `HUMANOID_RECORD_DIR` on the
policy process so `StepRecorder` logs real tick times, and correlate against those instead.
"""
from __future__ import annotations

import argparse
import bisect
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

POLICY_HZ = 25.0


def load_capture(capture_json: Path) -> tuple[dict, dict]:
    meta = json.loads(capture_json.read_text())
    sidecar = Path(str(capture_json).replace(".json", ".frames.jsonl"))
    if not sidecar.exists():
        raise SystemExit(f"missing frames sidecar: {sidecar}")
    data: dict[str, dict] = {}
    for line in sidecar.read_text().splitlines():
        if not line.strip():
            continue
        d = json.loads(line)
        data[d["joint"]] = {"ts": d["ts"], "pos": d.get("pos", []), "vel": d.get("vel", [])}
    return meta, data


def sample_age_and_stale(ts: list[float], grid: np.ndarray) -> dict:
    """M2 + M3 for one joint against a tick grid."""
    ages, stale, runs, run = [], 0, {}, 0
    prev_idx = None
    for t in grid:
        i = bisect.bisect_right(ts, t) - 1
        if i < 0:
            continue
        ages.append((t - ts[i]) * 1e3)
        if prev_idx is not None and i == prev_idx:
            stale += 1
            run += 1
        elif run:
            key = str(run) if run < 3 else "3+"
            runs[key] = runs.get(key, 0) + 1
            run = 0
        prev_idx = i
    if run:
        key = str(run) if run < 3 else "3+"
        runs[key] = runs.get(key, 0) + 1
    if not ages:
        return {"note": "no ticks inside frame window"}
    return {
        "ticks": len(ages),
        "sample_age_ms": C.stats(ages),
        "stale_hold_fraction": stale / max(len(ages) - 1, 1),
        "stale_run_lengths": runs,
    }


def encoder_stats(pos: list[float], vel: list[float]) -> dict:
    """M6 — quantisation and residual noise from the decoded payload."""
    p = np.asarray(pos, dtype=float)
    v = np.asarray(vel, dtype=float)
    if p.size < 4:
        return {"note": "insufficient samples", "n": int(p.size)}
    d = np.abs(np.diff(p))
    nz = d[d > 0]
    quantum = float(nz.min()) if nz.size else None
    x = np.arange(p.size, dtype=float)
    resid = p - np.polyval(np.polyfit(x, p, 1), x)
    return {
        "n": int(p.size),
        "distinct_positions": int(np.unique(p).size),
        "encoder_quantum_rad": quantum,
        "encoder_quantum_counts": (quantum / C.JOINT_QUANTUM_RAD) if quantum else None,
        "pos_p2p_rad": float(p.max() - p.min()),
        "pos_p2p_counts": float((p.max() - p.min()) / C.JOINT_QUANTUM_RAD),
        "pos_noise_std_rad": float(resid.std(ddof=0)),
        "pos_noise_std_counts": float(resid.std(ddof=0) / C.JOINT_QUANTUM_RAD),
        "vel_noise_std_rad_s": float(v.std(ddof=0)) if v.size else None,
        "vel_abs_max_rad_s": float(np.abs(v).max()) if v.size else None,
        "vel_distinct": int(np.unique(v).size) if v.size else 0,
        "vel_all_zero": bool(v.size and np.all(v == 0.0)),
    }


def per_joint(data: dict, joints: list[str], t0: float, t1: float) -> dict:
    grid = np.arange(t0, t1, 1.0 / POLICY_HZ)
    out = {}
    for j in joints:
        d = data.get(j)
        if not d or len(d["ts"]) < 4:
            out[j] = {"note": "no data", "frames": len(d["ts"]) if d else 0}
            continue
        sel = [(t, p, v) for t, p, v in zip(d["ts"], d["pos"], d["vel"]) if t0 <= t <= t1]
        if len(sel) < 4:
            out[j] = {"note": "no data in window", "frames": len(sel)}
            continue
        ts = [x[0] for x in sel]
        r = {"frames": len(sel)}
        r.update(sample_age_and_stale(ts, grid))
        r.update(encoder_stats([x[1] for x in sel], [x[2] for x in sel]))
        out[j] = r
    return out


def slot_structure(pj: dict) -> dict:
    means = {j: r["sample_age_ms"]["mean"] for j, r in pj.items() if "sample_age_ms" in r}
    if len(means) < 2:
        return {}
    vals = np.array(sorted(means.values()))
    spread = float(vals.max() - vals.min())
    NOMINAL_MS = 10.0
    MIN_SPREAD = 0.20 * NOMINAL_MS
    gaps = np.diff(vals)
    groups: list[list[str]] = []
    if spread >= MIN_SPREAD and gaps.size and gaps.max() > 0.25 * spread:
        cut = vals[int(np.argmax(gaps))]
        groups = [sorted([j for j, v in means.items() if v <= cut]),
                  sorted([j for j, v in means.items() if v > cut])]
    if groups:
        verdict = "joints cluster into distinct poll slots — model the grouping"
    elif spread < MIN_SPREAD:
        verdict = (f"no slot structure: spread {spread:.2f} ms is under {MIN_SPREAD:.1f} ms "
                   f"({100*spread/NOMINAL_MS:.0f}% of the {NOMINAL_MS:.0f} ms frame period). "
                   f"One delay term covers all joints; do NOT add a per-joint stagger.")
    else:
        verdict = "ages differ but do not separate cleanly; a single delay range covers all"
    return {"mean_age_ms": {j: round(v, 3) for j, v in sorted(means.items(), key=lambda kv: kv[1])},
            "spread_ms": spread, "min_meaningful_spread_ms": MIN_SPREAD,
            "clusters": groups, "verdict": verdict}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--capture", required=True, help="the *_can.json from m1_can_timing.py")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    cap_path = Path(args.capture)
    meta, data = load_capture(cap_path)
    joints = C.canonical_joint_order()

    all_ts = [t for d in data.values() for t in d["ts"][:1]] + \
             [d["ts"][-1] for d in data.values() if d["ts"]]
    t0 = max(d["ts"][0] for d in data.values() if d["ts"])
    t1 = min(d["ts"][-1] for d in data.values() if d["ts"])

    pj = per_joint(data, joints, t0, t1)

    buckets = {}
    for lo, hi in ((0, 120), (480, 600), (1080, 1200)):
        a, b = t0 + lo, min(t0 + hi, t1)
        if b - a >= 30:
            buckets[f"min_{lo//60}-{hi//60}"] = per_joint(data, joints, a, b)

    result = {
        "_meta": {"capture": str(cap_path), "capture_meta": meta.get("_meta"),
                  "window_s": t1 - t0, "policy_hz": POLICY_HZ,
                  "tick_grid": "synthetic 25 Hz — see module docstring"},
        "per_joint": pj,
        "slot_structure": slot_structure(pj),
        "m8_buckets": buckets,
        "m1_timing": meta.get("per_joint"),
        "thermal_series": meta.get("thermal_series"),
        "bus_counters_delta": meta.get("bus_counters_delta"),
    }
    out = Path(args.out) if args.out else cap_path.with_name(
        cap_path.stem.replace("_can", "") + "_M2M3M6.json")
    C.write_json(out, result)

    print(f"window {t1-t0:.1f}s   synthetic {POLICY_HZ:g} Hz tick grid\n")
    print(f"{'joint':26s} {'frames':>7s} {'age_mean':>9s} {'age_p95':>8s} {'age_max':>8s} "
          f"{'stale%':>7s} {'quant_ct':>9s} {'noise_ct':>9s} {'p2p_ct':>8s}")
    for j in joints:
        r = pj.get(j, {})
        if "sample_age_ms" not in r:
            print(f"{j:26s} {r.get('note','-')}")
            continue
        s = r["sample_age_ms"]
        q = r.get("encoder_quantum_counts")
        print(f"{j:26s} {r['frames']:>7d} {s['mean']:>9.2f} {s['p95']:>8.2f} {s['max']:>8.2f} "
              f"{100*r['stale_hold_fraction']:>7.2f} {(f'{q:.2f}' if q else '-'):>9s} "
              f"{r['pos_noise_std_counts']:>9.2f} {r['pos_p2p_counts']:>8.1f}")
    ss = result["slot_structure"]
    if ss:
        print(f"\nage spread {ss['spread_ms']:.2f} ms — {ss['verdict']}")
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
