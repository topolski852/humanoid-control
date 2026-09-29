#!/usr/bin/env python3
"""M7 v2 — effective stiffness, measured properly. Paired sweeps, held pose, measured geometry.

    python scripts/measure/m7_paired.py --i-am-present

*** MOVES JOINTS IN LEFT/RIGHT PAIRS. BOTH LEGS MUST HANG FREE — feet off the ground. ***

### What went wrong in v1 (m7_all_joints.py) and is fixed here

1. **Non-swept joints were IDLE, so they sagged**, and the gravity torque was computed assuming
   they were at zero. That overestimated tau by up to 34% on `hip_roll` and 69% on `ankle_pitch`,
   which inflated `kp_eff` by the same factor and produced a physically odd 96-100 N·m/rad on
   hip_roll. Now every joint is held in POSITION, **and** tau is computed from the MEASURED pose
   of all 12 joints rather than an assumed one — so the answer does not depend on the hold having
   worked.
2. **Moving one hip alone risks a leg collision** — the hips sit close together. Forward kinematics
   on this robot's own URDF says the SAME device sign on both sides keeps the feet 0.11-0.16 m
   apart across the whole range, while opposite signs splay them to 0.93 m. So pairs move with the
   same sign, together. That is collision-safe and free: the legs are mechanically independent, so
   each joint still sees only its own leg's gravity, giving two measurements per sweep.
3. Any point where a non-swept joint drifted more than DRIFT_TOL from its hold target is flagged,
   because that invalidates the geometry for that point.

### The method, unchanged

The URDF link masses sum to 12.61191 kg against a 12.61 kg measured robot (0.015%), so the limb is
a calibrated load. `position_ki = 0` everywhere and velocity is zero once settled, so

    steady-state error = tau_gravity / kp_eff

This needs no torque_constant, current sensor or gearbox efficiency — unlike every reconstructed
`kp*err - kd*vel` figure in the reports, which is circular.

v1's one trustworthy result was `knee_pitch` (43.46 / 44.47 against nominal 45): structurally
immune to the sag error, since nothing upstream changes its own distal chain.
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
from humanoid_control import LegPolicyContract, resolve_robot_config_path  # noqa: E402
from humanoid_control.daemon import DaemonClient, RobotConfig  # noqa: E402

NOMINAL_KP = 45.0
RAMP_S, SETTLE_S, SAMPLE_S = 2.5, 3.0, 2.0
MIN_TAU_NM = 0.4
N_ANGLES = 6
DRIFT_TOL_RAD = math.radians(1.0)
AWAKE = {"IDLE", "ENABLED", "POSITION", "DAMPING"}
TYPES = ["hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch"]
SKIP = ["ankle_roll"]          # gravity gives 0.04 N·m; needs m7_stiffness.py with a mass
FAMILY = {"hip_roll": "M6C12", "hip_yaw": "M6C12", "hip_pitch": "M6C12",
          "knee_pitch": "M6C12", "ankle_pitch": "MAD5010"}


async def read_all(client, joints, seconds):
    """Median of every joint's position, plus torque/current for each."""
    acc = {j: {"pos": [], "tau": [], "cur": []} for j in joints}
    end = time.time() + seconds
    while time.time() < end:
        for j in joints:
            st = client.get_cached_joint_state(j) or {}
            for k, key in (("pos", "position"), ("tau", "torque"), ("cur", "current")):
                if st.get(key) is not None:
                    acc[j][k].append(float(st[key]))
        await asyncio.sleep(0.02)
    med = lambda a: float(np.median(a)) if a else None      # noqa: E731
    return {j: {"position_rad": med(v["pos"]), "reported_torque_nm": med(v["tau"]),
                "reported_current_a": med(v["cur"]), "n": len(v["pos"])}
            for j, v in acc.items()}


def urdf_pose(snapshot):
    """Measured device positions -> URDF joint angles, for the gravity model."""
    return {LG.urdf_name(j): (s["position_rad"] or 0.0) for j, s in snapshot.items()}


def pick_pair_angles(model, t, raw):
    """Angles valid for BOTH sides (limits intersected), spanning the widest gravity torque."""
    lo = max(raw[f"left_{t}_joint"]["position_limits"]["min"],
             raw[f"right_{t}_joint"]["position_limits"]["min"])
    hi = min(raw[f"left_{t}_joint"]["position_limits"]["max"],
             raw[f"right_{t}_joint"]["position_limits"]["max"])
    uL = LG.urdf_name(f"left_{t}_joint")
    cand = np.linspace(lo, hi, 80)
    taus = np.array([model.gravity_torque({uL: a}, [uL])[uL] for a in cand])
    i = int(np.argmax(np.abs(taus)))
    if abs(taus[i]) < MIN_TAU_NM:
        return [], abs(taus[i])
    z = int(np.argmin(np.abs(taus)))
    m = 0.03
    ang = [float(min(max(a, lo + m), hi - m))
           for a in np.linspace(cand[z], cand[i], N_ANGLES)]
    return ang, abs(taus[i])


async def ramp_pair(client, pair, starts, goal, seconds=RAMP_S, hz=50.0):
    n = max(int(seconds * hz), 1)
    for i in range(1, n + 1):
        f = i / n
        for j in pair:
            client.set_position(j, starts[j] + (goal - starts[j]) * f)
        await asyncio.sleep(1.0 / hz)


async def run(args) -> int:
    cfgp = resolve_robot_config_path()
    rc = RobotConfig.from_json(str(cfgp))
    raw = json.loads(Path(cfgp).read_text())["joints"]
    model = LG.LegModel()
    joints = list(LegPolicyContract.load().joint_order)

    plan = []
    for t in TYPES:
        ang, peak = pick_pair_angles(model, t, raw)
        plan.append((t, ang, peak))
    runnable = [(t, a, p) for t, a, p in plan if a]
    per = N_ANGLES * (RAMP_S + SETTLE_S + SAMPLE_S) + 5
    print(f"{len(runnable)} PAIRED sweeps x ~{per:.0f}s = ~{len(runnable)*per/60:.0f} min")
    for t, a, p in plan:
        if a:
            print(f"  {t:12s} {len(a)} angles {math.degrees(a[0]):+.0f}..{math.degrees(a[-1]):+.0f}° "
                  f"peak tau {p:.2f} N·m  (both sides, SAME sign, together)")
        else:
            print(f"  {t:12s} SKIP — peak tau {p:.3f} N·m")
    print(f"  {', '.join(SKIP):12s} SKIP — gravity ~0.04 N·m; use m7_stiffness.py with a mass")
    print("\nAll 12 joints are held in POSITION; only the swept pair moves.")
    print("BOTH LEGS MUST HANG FREE. Confirm ('y'): ", end="")
    try:
        if input().strip().lower() != "y":
            print("aborted."); return 1
    except EOFError:
        print("\nno TTY.", file=sys.stderr); return 2

    client = DaemonClient(rc)
    await client.start()
    results, home = {}, None
    try:
        snap0 = await read_all(client, joints, 1.5)
        bad = [(j, (client.get_cached_joint_state(j) or {}).get("state"))
               for j in joints if (client.get_cached_joint_state(j) or {}).get("state") not in AWAKE]
        if bad:
            print("not awake: " + ", ".join(f"{j}={s}" for j, s in bad), file=sys.stderr)
            return 2
        home = {j: snap0[j]["position_rad"] for j in joints}
        print("\nholding all 12 joints at their current positions ...")
        for j in joints:
            client.set_mode(j, "POSITION")
            client.set_position(j, home[j])
        await asyncio.sleep(SETTLE_S)

        for t, angles, _ in runnable:
            pair = [f"left_{t}_joint", f"right_{t}_joint"]
            print(f"\n--- {t} (pair, same sign) ---")
            prev = {j: home[j] for j in pair}
            trials = []
            for a in angles:
                await ramp_pair(client, pair, prev, a)
                prev = {j: a for j in pair}
                await asyncio.sleep(SETTLE_S)
                snap = await read_all(client, joints, SAMPLE_S)
                q = urdf_pose(snap)
                drift = {j: snap[j]["position_rad"] - home[j] for j in joints if j not in pair}
                worst = max(drift.items(), key=lambda kv: abs(kv[1])) if drift else (None, 0.0)
                row = {"commanded_rad": a, "worst_drift_joint": worst[0],
                       "worst_drift_rad": worst[1],
                       "drift_ok": abs(worst[1]) <= DRIFT_TOL_RAD, "per_side": {}}
                for j in pair:
                    uj = LG.urdf_name(j)
                    tau = model.gravity_torque(q, [uj])[uj]
                    err = a - snap[j]["position_rad"]
                    row["per_side"][j] = {"tau_gravity_nm": tau, "error_rad": err, **snap[j]}
                trials.append(row)
                L, R = row["per_side"][pair[0]], row["per_side"][pair[1]]
                print(f"  cmd {math.degrees(a):>+7.2f}°  L err {math.degrees(L['error_rad']):>+6.2f}° "
                      f"tau {L['tau_gravity_nm']:>+6.3f} | R err {math.degrees(R['error_rad']):>+6.2f}° "
                      f"tau {R['tau_gravity_nm']:>+6.3f} | drift {math.degrees(worst[1]):>+5.2f}° "
                      f"{'' if row['drift_ok'] else '<< DRIFT'}")
            for j in pair:
                good = [x for x in trials
                        if x["drift_ok"] and abs(x["per_side"][j]["tau_gravity_nm"]) > MIN_TAU_NM]
                if len(good) < 2:
                    results[j] = {"joint": j, "skipped": "fewer than 2 clean loaded points",
                                  "trials": trials}
                    continue
                tau = np.array([abs(x["per_side"][j]["tau_gravity_nm"]) for x in good])
                err = np.array([abs(x["per_side"][j]["error_rad"]) for x in good])
                slope, icpt = np.polyfit(err, tau, 1)
                results[j] = {"joint": j, "type": t, "family": FAMILY[t],
                              "kp_eff": float(slope), "deadband_nm": float(icpt),
                              "points": len(good), "peak_tau_nm": float(tau.max()),
                              "trials": trials}
            await ramp_pair(client, pair, {j: angles[-1] for j in pair}, home[pair[0]], seconds=3.0)
            for j in pair:
                client.set_position(j, home[j])
    finally:
        try:
            if home:
                for j in joints:
                    client.set_position(j, home[j])
                await asyncio.sleep(1.0)
            for j in joints:
                client.set_mode(j, "IDLE")
            print("\nall joints returned home and released to IDLE.")
        except Exception:
            pass
        await client.stop()

    ok = [r for r in results.values() if r.get("kp_eff")]
    if not ok:
        print("\nno fits."); return 1
    print(f"\n{'='*72}\nkp_eff PER JOINT (nominal {NOMINAL_KP}) — tau from MEASURED pose\n{'='*72}")
    print(f"{'joint':26s} {'family':9s} {'kp_eff':>8s} {'ratio':>6s} {'deadband':>9s} {'pts':>4s}")
    for r in sorted(ok, key=lambda r: r["joint"]):
        print(f"{r['joint']:26s} {r['family']:9s} {r['kp_eff']:>8.2f} "
              f"{r['kp_eff']/NOMINAL_KP:>6.2f} {r['deadband_nm']:>+9.3f} {r['points']:>4d}")
    print(f"\n{'='*72}\nLEFT vs RIGHT\n{'='*72}")
    lr = {}
    for t in TYPES:
        L = next((r["kp_eff"] for r in ok if r["joint"] == f"left_{t}_joint"), None)
        R = next((r["kp_eff"] for r in ok if r["joint"] == f"right_{t}_joint"), None)
        if L is None or R is None:
            continue
        ratio = L / R
        lr[t] = {"left": L, "right": R, "ratio": ratio}
        flag = ("BALANCED" if 0.85 <= ratio <= 1.18
                else (f"LEFT {ratio:.2f}x stiffer" if ratio > 1 else f"RIGHT {1/ratio:.2f}x stiffer"))
        print(f"{t:14s} L {L:>8.2f}  R {R:>8.2f}  L/R {ratio:>6.2f}  {flag}")
    print(f"\n{'='*72}\nPER FAMILY — the form the sim actuator model takes\n{'='*72}")
    fam = {}
    for f in ("M6C12", "MAD5010"):
        v = [r["kp_eff"] for r in ok if r["family"] == f]
        if not v:
            continue
        fam[f] = {"mean": float(np.mean(v)), "median": float(np.median(v)), "n": len(v),
                  "min": float(min(v)), "max": float(max(v))}
        print(f"{f:9s} n={len(v):2d}  mean {np.mean(v):6.2f}  median {np.median(v):6.2f}  "
              f"range {min(v):.2f}-{max(v):.2f}  ratio {np.mean(v)/NOMINAL_KP:.2f}")
    allk = np.array([r["kp_eff"] for r in ok])
    r_all = float(np.median(allk) / NOMINAL_KP)
    print(f"\nALL {len(allk)} JOINTS: median kp_eff {np.median(allk):.2f}, ratio {r_all:.2f}")
    out = C.default_outdir() / f"m7_paired_{time.strftime('%Y%m%dT%H%M%S')}.json"
    C.write_json(out, {"_meta": C.finish_meta(C.capture_meta(
        "bench", measurement="M7_paired_v2", nominal_kp=NOMINAL_KP,
        method="paired same-sign sweeps; all joints held in POSITION; tau from MEASURED pose",
        drift_tol_deg=1.0, skipped=SKIP, urdf_total_mass_kg=LG.total_mass(model),
        measured_robot_mass_kg=12.61, requires="legs hanging free")),
        "per_joint": list(results.values()), "left_right": lr, "per_family": fam,
        "overall": {"median_kp_eff": float(np.median(allk)), "ratio_to_nominal": r_all}})
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--i-am-present", action="store_true")
    args = ap.parse_args()
    if not args.i_am_present:
        print("REFUSING: this moves joints. Re-run with --i-am-present, legs hanging free.",
              file=sys.stderr)
        return 2
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
