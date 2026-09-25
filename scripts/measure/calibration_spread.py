#!/usr/bin/env python3
"""Measure how repeatable the leg zeroing is — the last unmeasured randomisation.

    python scripts/measure/calibration_spread.py --trials 10

`position_offset` resets on every power cycle and is recaptured by hand from the folded,
feet-together stance. Training randomises joint zero-offset by +-0.05 rad
(`add_all_joint_default_pos`) and that value was never measured; Asimov uses +-0.02.

This matters more than it looks. measA-full shrank observation noise to the measured SENSOR
floor and regressed badly on hardware, and the leading hypothesis is that the old wide noise was
also covering calibration-offset spread. That spread is exactly what this measures, so it
converts a guess into a number and tells the next training round how much of the removed
robustness needs putting back explicitly.

### The protocol matters more than the tool

You MUST break the stance and physically re-establish it between trials.

  * re-folding each time  -> measures OPERATOR + mechanical repeatability. This is the number
    training needs, because it is what actually happens after a power cycle.
  * holding still and re-running -> measures only the 1.5 s sampling noise. A lower bound, and
    a misleadingly small one.

The script pauses between trials and will not continue until you confirm the stance was rebuilt.

Talks to the web service over HTTP and never opens a daemon telemetry socket, so it cannot
starve the service that performs the calibration.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import common as C  # noqa: E402

BASE = "http://127.0.0.1:8000"


def api(path, method="GET", timeout=60):
    req = urllib.request.Request(f"{BASE}{path}", method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def preflight():
    try:
        st = api("/api/status")["data"]
    except Exception as exc:
        raise SystemExit(f"web service not reachable at {BASE}: {exc}")
    if st.get("state") == "DISCONNECTED":
        raise SystemExit("robot is DISCONNECTED — connect it first, then re-run.")
    if st.get("armed"):
        raise SystemExit("robot is ARMED — disarm before calibrating.")
    print(f"state {st.get('state')} | armed {st.get('armed')} | "
          f"all_calibrated {st.get('all_calibrated')}")
    return st


def one_trial(limb="both"):
    r = api(f"/api/calibrate/legs/{limb}", method="POST", timeout=120)
    cal = (r.get("data") or {}).get("cal") or {}
    rows = cal if isinstance(cal, list) else cal.get("results") or cal.get("joints") or []
    if isinstance(rows, dict):
        rows = list(rows.values())
    out, failed = {}, []
    for row in rows:
        if not isinstance(row, dict) or "joint" not in row:
            continue
        if row.get("ok") is False:
            failed.append((row["joint"], row.get("reason", "?")))
            continue
        if row.get("offset") is not None:
            out[row["joint"]] = float(row["offset"])
    return out, failed, cal


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trials", type=int, default=10)
    ap.add_argument("--limb", default="both")
    ap.add_argument("--out", default=None)
    ap.add_argument("--yes", action="store_true",
                    help="do not pause between trials (ONLY if something else re-folds the robot)")
    args = ap.parse_args()

    preflight()
    print(f"\n{args.trials} trials. Between each one: BREAK the stance and REBUILD it.\n"
          f"Folding it the same way every time is the point — that spread IS the measurement.\n")

    trials = []
    for i in range(1, args.trials + 1):
        if not args.yes:
            msg = (f"--- trial {i}/{args.trials}: fold the robot into the stance "
                   f"(feet together), then press Enter [q to stop] ")
            try:
                if input(msg).strip().lower() == "q":
                    print("stopping early; keeping what we have.")
                    break
            except EOFError:
                print("\nno TTY — re-run in a terminal, or pass --yes if something else "
                      "re-folds the robot.", file=sys.stderr)
                return 2
        try:
            offs, failed, _ = one_trial(args.limb)
        except urllib.error.HTTPError as e:
            print(f"  trial {i} REFUSED: {e.read().decode()[:200]}")
            continue
        except Exception as exc:
            print(f"  trial {i} failed: {exc}")
            continue
        if not offs:
            print(f"  trial {i}: no offsets returned; skipping")
            continue
        trials.append({"trial": i, "t": time.time(), "offsets": offs,
                       "failed": [{"joint": j, "reason": r} for j, r in failed]})
        print(f"  trial {i}: {len(offs)} joints"
              + (f"  ({len(failed)} failed: {', '.join(j for j, _ in failed)})" if failed else ""))

    if len(trials) < 2:
        print("\nneed at least 2 successful trials to measure a spread.")
        return 1

    joints = [j for j in C.canonical_joint_order()
              if all(j in t["offsets"] for t in trials)]
    print(f"\n=== offset spread across {len(trials)} trials ===")
    print(f"{'joint':26s} {'mean':>10s} {'std':>9s} {'range':>9s} {'min':>10s} {'max':>10s}")
    per = {}
    for j in joints:
        v = np.array([t["offsets"][j] for t in trials])
        per[j] = {"n": int(v.size), "mean_rad": float(v.mean()), "std_rad": float(v.std(ddof=1)),
                  "range_rad": float(v.max() - v.min()),
                  "min_rad": float(v.min()), "max_rad": float(v.max()),
                  "values_rad": v.tolist()}
        print(f"{j:26s} {v.mean():>+10.5f} {v.std(ddof=1):>9.5f} "
              f"{v.max()-v.min():>9.5f} {v.min():>+10.5f} {v.max():>+10.5f}")

    stds = np.array([per[j]["std_rad"] for j in joints])
    rngs = np.array([per[j]["range_rad"] for j in joints])
    # A symmetric uniform randomisation that spans the observed spread: half-range, worst joint.
    suggested = float(rngs.max() / 2.0)
    print(f"\nacross joints: std median {np.median(stds):.5f} max {stds.max():.5f} rad")
    print(f"               range median {np.median(rngs):.5f} max {rngs.max():.5f} rad")
    print(f"\nsuggested add_all_joint_default_pos: +-{suggested:.4f} rad "
          f"(half the worst joint's observed range)")
    print(f"  currently trained: +-0.05   |   Asimov: +-0.02")
    if suggested > 0.05:
        print("  -> WIDER than the current value: the guess was too small, not too large.")
    elif suggested > 0.02:
        print("  -> between Asimov's value and the current one.")
    else:
        print("  -> narrower than both; calibration is more repeatable than assumed.")

    out = Path(args.out) if args.out else C.default_outdir() / (
        f"calibration_spread_{time.strftime('%Y%m%dT%H%M%S')}.json")
    C.write_json(out, {
        "_meta": C.finish_meta(C.capture_meta("calibration", measurement="calibration_offset",
                                              trials=len(trials), limb=args.limb,
                                              protocol="stance broken and rebuilt between trials"
                                                       if not args.yes else "UNPAUSED — see docs")),
        "per_joint": per,
        "summary": {"std_median_rad": float(np.median(stds)), "std_max_rad": float(stds.max()),
                    "range_median_rad": float(np.median(rngs)),
                    "range_max_rad": float(rngs.max()),
                    "suggested_uniform_half_width_rad": suggested,
                    "currently_trained_rad": 0.05, "asimov_rad": 0.02},
        "trials": trials,
    })
    print(f"\nwrote {out}", file=sys.stderr)
    print("next: python scripts/measure/build_training_input.py   (folds this into "
          "TRAINING_INPUT.json)", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
