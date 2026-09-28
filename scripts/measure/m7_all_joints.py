#!/usr/bin/env python3
"""M7 — effective stiffness for every measurable leg joint, with L/R comparison.

    python scripts/measure/m7_all_joints.py --i-am-present            # both legs
    python scripts/measure/m7_all_joints.py --i-am-present --limb left

*** MOVES ONE JOINT AT A TIME. BOTH LEGS MUST HANG FREE — robot lifted, feet off the ground. ***

Runs the gravity sweep (see m7_gravity_sweep.py for the method and why the limb is a calibrated
load) across every joint gravity can actually load, then reports:

  * kp_eff per joint against the nominal 45 N·m/rad
  * LEFT vs RIGHT per joint type — a stiffness asymmetry would be a direct candidate explanation
    for the gait asymmetry measured on 2026-09-28 (right leg 19% more excursion, right knee swing
    72% of left, and the robot walking in a circle)
  * per motor family (M6C12 hip/knee vs MAD5010 ankle), since a family-wide number is what the
    sim actuator model would take

### Not every joint can be measured this way

Gravity can only load a joint whose axis has a horizontal component, and the torque has to be big
enough to produce a deflection well above the 1.02e-4 rad encoder quantum. Measured against this
robot's actual URDF geometry:

    hip_pitch    5.82 N·m   good        (this robot's hip roll/yaw axes are canted 45 degrees,
    hip_roll     4.82 N·m   good         not orthogonal, which is why hip_yaw is loadable at all)
    knee_pitch   2.55 N·m   good
    hip_yaw      2.38 N·m   good
    ankle_pitch  0.63 N·m   MARGINAL    ~0.8 deg expected deflection at kp=45
    ankle_roll   0.04 N·m   UNUSABLE    gravity cannot load it; needs a hung mass

So this covers 8 joints well and 2 marginally. **Both `ankle_roll` joints need
`m7_stiffness.py` with an external mass on the foot** — this script skips them and says so rather
than reporting a number derived from 0.04 N·m.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import _bootstrap  # noqa: F401,E402
import common as C  # noqa: E402
import leg_gravity as LG  # noqa: E402
from humanoid_control import resolve_robot_config_path  # noqa: E402
from humanoid_control.daemon import DaemonClient, RobotConfig  # noqa: E402

NOMINAL_KP = 45.0
RAMP_S, SETTLE_S, SAMPLE_S = 2.5, 3.0, 2.0
MIN_TAU_NM = 0.4          # below this the deflection is too small to fit
N_ANGLES = 6
SKIP = {"ankle_roll"}     # gravity gives 0.04 N·m — use m7_stiffness.py with a mass
FAMILY = {"hip_roll": "M6C12", "hip_yaw": "M6C12", "hip_pitch": "M6C12",
          "knee_pitch": "M6C12", "ankle_pitch": "MAD5010", "ankle_roll": "MAD5010"}
ORDER = ["hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch"]


def jtype(dj):
    return dj.replace("left_", "").replace("right_", "").replace("_joint", "")


def pick_angles(model, dj, limits):
    """Angles inside the limits that span the widest achievable gravity torque."""
    uj = LG.urdf_name(dj)
    lo, hi = limits["min"], limits["max"]
    cand = np.linspace(lo, hi, 80)
    taus = np.array([model.gravity_torque({uj: a}, [uj])[uj] for a in cand])
    i = int(np.argmax(np.abs(taus)))
    peak_a, peak_t = cand[i], taus[i]
    if abs(peak_t) < MIN_TAU_NM:
        return [], abs(peak_t)
    # walk from the zero-crossing side toward the peak so torque rises monotonically
    zero_i = int(np.argmin(np.abs(taus)))
    a0, a1 = cand[zero_i], peak_a
    angles = [float(x) for x in np.linspace(a0, a1, N_ANGLES)]
    # keep a small margin off the hard limits
    m = 0.03
    angles = [min(max(a, lo + m), hi - m) for a in angles]
    return angles, abs(peak_t)


async def sample(client, joint, seconds=SAMPLE_S):
    pos, tau, cur = [], [], []
    end = time.time() + seconds
    while time.time() < end:
        st = client.get_cached_joint_state(joint) or {}
        for k, acc in (("position", pos), ("torque", tau), ("current", cur)):
            if st.get(k) is not None:
                acc.append(float(st[k]))
        await asyncio.sleep(0.02)
    med = lambda a: float(np.median(a)) if a else None      # noqa: E731
    return {"position_rad": med(pos), "reported_torque_nm": med(tau),
            "reported_current_a": med(cur), "n": len(pos)}


async def ramp(client, joint, start, goal, seconds=RAMP_S, hz=50.0):
    n = max(int(seconds * hz), 1)
    for i in range(1, n + 1):
        client.set_position(joint, start + (goal - start) * (i / n))
        await asyncio.sleep(1.0 / hz)


async def do_joint(client, model, dj, limits) -> dict:
    uj = LG.urdf_name(dj)
    angles, peak = pick_angles(model, dj, limits)
    if not angles:
        return {"joint": dj, "skipped": f"max gravity torque {peak:.3f} N·m < {MIN_TAU_NM}"}
    base = await sample(client, dj, 1.0)
    if base["position_rad"] is None:
        return {"joint": dj, "skipped": "no telemetry"}
    start = base["position_rad"]
    trials = []
    try:
        client.set_mode(dj, "POSITION")
        prev = start
        for a in angles:
            await ramp(client, dj, prev, a); prev = a
            await asyncio.sleep(SETTLE_S)
            s = await sample(client, dj)
            tau_g = model.gravity_torque({uj: a}, [uj])[uj]
            err = a - s["position_rad"]
            trials.append({"commanded_rad": a, "tau_gravity_nm": tau_g, "error_rad": err, **s})
            print(f"    cmd {math.degrees(a):>+7.2f}°  err {math.degrees(err):>+6.2f}°  "
                  f"tau_g {tau_g:>+6.3f}  ESC tau {s['reported_torque_nm']:>+6.3f}")
    finally:
        cur = (await sample(client, dj, 0.3))["position_rad"] or start
        await ramp(client, dj, cur, start, seconds=3.0)
        client.set_mode(dj, "IDLE")

    good = [t for t in trials if abs(t["tau_gravity_nm"]) > MIN_TAU_NM]
    if len(good) < 2:
        return {"joint": dj, "trials": trials, "skipped": "fewer than 2 loaded points"}
    tau = np.array([abs(t["tau_gravity_nm"]) for t in good])
    err = np.array([abs(t["error_rad"]) for t in good])
    slope, icpt = np.polyfit(err, tau, 1)
    return {"joint": dj, "type": jtype(dj), "family": FAMILY[jtype(dj)], "trials": trials,
            "kp_eff": float(slope), "deadband_nm": float(icpt), "points": len(good),
            "peak_tau_nm": float(tau.max())}


async def run(args) -> int:
    cfgp = resolve_robot_config_path()
    rc = RobotConfig.from_json(str(cfgp))
    raw = json.loads(Path(cfgp).read_text())["joints"]
    model = LG.LegModel()
    sides = ["left", "right"] if args.limb == "both" else [args.limb]
    todo = [f"{s}_{t}_joint" for t in ORDER for s in sides
            if f"{s}_{t}_joint" in raw and t not in SKIP]
    skipped = [f"{s}_{t}_joint" for t in SKIP for s in sides if f"{s}_{t}_joint" in raw]

    per = (N_ANGLES * (RAMP_S + SETTLE_S + SAMPLE_S)) + 4
    print(f"{len(todo)} joints x ~{per:.0f}s = ~{len(todo)*per/60:.0f} min")
    print(f"skipping (gravity too weak, use m7_stiffness.py with a mass): {', '.join(skipped)}")
    print("\nBOTH LEGS MUST HANG FREE — feet off the ground. Only one joint moves at a time.")
    print("Confirm ('y'): ", end="")
    try:
        if input().strip().lower() != "y":
            print("aborted."); return 1
    except EOFError:
        print("\nno TTY — run in a terminal.", file=sys.stderr); return 2

    client = DaemonClient(rc)
    await client.start()
    results = []
    try:
        if not client.is_connected():
            print("not connected — connect in the web app first.", file=sys.stderr); return 2
        for dj in todo:
            print(f"\n--- {dj} ---")
            try:
                results.append(await do_joint(client, model, dj, raw[dj]["position_limits"]))
            except Exception as exc:
                print(f"    FAILED: {exc}")
                results.append({"joint": dj, "error": str(exc)})
    finally:
        await client.stop()

    ok = [r for r in results if r.get("kp_eff")]
    if not ok:
        print("\nno joints produced a fit."); return 1

    print(f"\n{'='*70}\nkp_eff PER JOINT (nominal {NOMINAL_KP})\n{'='*70}")
    print(f"{'joint':26s} {'family':9s} {'kp_eff':>8s} {'ratio':>6s} {'deadband':>9s} {'peak_tau':>9s}")
    for r in sorted(ok, key=lambda r: r["joint"]):
        print(f"{r['joint']:26s} {r['family']:9s} {r['kp_eff']:>8.2f} "
              f"{r['kp_eff']/NOMINAL_KP:>6.2f} {r['deadband_nm']:>+9.3f} {r['peak_tau_nm']:>9.3f}")

    print(f"\n{'='*70}\nLEFT vs RIGHT per joint type\n{'='*70}")
    lr = {}
    print(f"{'type':14s} {'LEFT':>9s} {'RIGHT':>9s} {'L/R':>7s}  asymmetry")
    for t in ORDER:
        L = next((r["kp_eff"] for r in ok if r["joint"] == f"left_{t}_joint"), None)
        R = next((r["kp_eff"] for r in ok if r["joint"] == f"right_{t}_joint"), None)
        if L is None or R is None:
            continue
        ratio = L / R
        lr[t] = {"left": L, "right": R, "ratio": ratio}
        flag = ("BALANCED" if 0.85 <= ratio <= 1.18 else
                f"LEFT {ratio:.2f}x stiffer" if ratio > 1 else f"RIGHT {1/ratio:.2f}x stiffer")
        print(f"{t:14s} {L:>9.2f} {R:>9.2f} {ratio:>7.2f}  {flag}")

    print(f"\n{'='*70}\nPER MOTOR FAMILY — what the sim actuator model would take\n{'='*70}")
    fam = {}
    for f in ("M6C12", "MAD5010"):
        v = [r["kp_eff"] for r in ok if r["family"] == f]
        if not v:
            continue
        fam[f] = {"mean": float(np.mean(v)), "median": float(np.median(v)),
                  "min": float(min(v)), "max": float(max(v)), "n": len(v)}
        print(f"{f:9s} n={len(v):2d}  mean {np.mean(v):6.2f}  median {np.median(v):6.2f}  "
              f"range {min(v):.2f}-{max(v):.2f}  ratio to nominal {np.mean(v)/NOMINAL_KP:.2f}")

    allk = np.array([r["kp_eff"] for r in ok])
    r_all = float(np.median(allk) / NOMINAL_KP)
    print(f"\nACROSS ALL {len(allk)} JOINTS: median kp_eff {np.median(allk):.2f}, "
          f"ratio to nominal {r_all:.2f}")
    if r_all < 0.5:
        print(f"  *** CONFIRMS THE HYPOTHESIS — joints are ~{1/r_all:.1f}x softer than sim believes.")
        print(f"      A PLANT correction, not a reward change. Set the sim stiffness to the")
        print(f"      per-family numbers above rather than 45.")
    elif r_all < 0.85:
        print("  PARTIAL — softer than nominal but short of the 3-5x the torque gap needs.")
    else:
        print("  REFUTES it — kp is near nominal. Look at gearbox efficiency, Kt, or the")
        print("  actuator model instead.")
    if skipped:
        print(f"\nstill unmeasured: {', '.join(skipped)} — gravity gives only ~0.04 N·m there.")
        print("  Run: python scripts/measure/m7_stiffness.py --joint <ankle_roll> --i-am-present")

    out = C.default_outdir() / f"m7_all_joints_{time.strftime('%Y%m%dT%H%M%S')}.json"
    C.write_json(out, {
        "_meta": C.finish_meta(C.capture_meta(
            "bench", measurement="M7_all_joints", nominal_kp=NOMINAL_KP, limb=args.limb,
            urdf_total_mass_kg=LG.total_mass(model), measured_robot_mass_kg=12.61,
            skipped_joints=skipped, requires="both legs hanging free, feet off the ground")),
        "per_joint": results, "left_right": lr, "per_family": fam,
        "overall": {"median_kp_eff": float(np.median(allk)), "ratio_to_nominal": r_all},
    })
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--limb", choices=["left", "right", "both"], default="both")
    ap.add_argument("--i-am-present", action="store_true")
    args = ap.parse_args()
    if not args.i_am_present:
        print("REFUSING: this moves joints. Re-run with --i-am-present, legs hanging free.",
              file=sys.stderr)
        return 2
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
