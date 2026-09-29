#!/usr/bin/env python3
"""M7 — effective joint stiffness and torque delivery under a known static load.

    python scripts/measure/m7_stiffness.py --joint left_knee_pitch_joint --i-am-present

*** THIS ENERGISES ONE JOINT. *** It holds the joint where it already is (no ramp, no motion
on start) and you then hang a known mass on the limb. The joint will deflect under the load —
that deflection IS the measurement.

### Why this is the critical-path measurement

Sim under-predicts torque demand by 3-5x on six leg joints spanning two motor families and two
different torque caps (HARDWARE_PLAN_2026-09-28 section 1). A uniform gap across joints that
share little except `kp = 45` points at effective stiffness. If the real joints are materially
softer than 45 N·m/rad, sim believes a given position error produces ~3x more restoring torque
than the robot generates — which explains every row at once, and explains why raising caps and
adding reward penalties kept moving the problem instead of removing it.

### The measurement does not depend on the ESC's own reporting

With a known mass on a measured lever arm, the applied torque is known from physics:

    tau_applied = m * g * L * cos(theta)          theta = angle of the lever from horizontal

so

    kp_eff = tau_applied / (loaded_position - unloaded_position)

That is independent of `torque_constant`, gearbox efficiency and the current sensor. Every
previous "hardware torque" figure in these reports was RECONSTRUCTED as `kp*err - kd*vel`, which
is circular — a joint that cannot track shows large error, which inflates the inferred demand.
This breaks the circle.

Separately, comparing the ESC's own reported torque and current against the known applied load
validates `Kt * gear` end to end. One setup, two answers.

### Use several loads, not one

A single point cannot separate linear stiffness from stiction and deadband. Three or more masses
give a slope (the stiffness) and an intercept (the deadband). The tool fits both.
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
from humanoid_control import resolve_robot_config_path  # noqa: E402
from humanoid_control.daemon import DaemonClient, RobotConfig  # noqa: E402

G = 9.80665
SETTLE_S = 3.0
SAMPLE_S = 2.0
NOMINAL_KP = 45.0

AWAKE_STATES = {"IDLE", "ENABLED", "POSITION", "DAMPING"}


async def joints_awake(client, joints, seconds=1.5):
    """Are the motors actually awake? Do NOT use client.is_connected() for this.

    is_connected() is PER-CLIENT: it is set by wake_all/apply_all_configs on that instance, so a
    freshly constructed DaemonClient reports False even when the robot is connected and every
    joint is reporting IDLE. Checking it here refused to run against a perfectly ready robot.
    Telemetry joint state is the ground truth.
    """
    import asyncio as _a
    await _a.sleep(seconds)
    bad = []
    for j in joints:
        st = (client.get_cached_joint_state(j) or {}).get("state")
        if st not in AWAKE_STATES:
            bad.append((j, st))
    return bad



async def sample(client, joint, seconds=SAMPLE_S):
    """Median position / reported torque / reported current over a settled window."""
    pos, tau, cur = [], [], []
    end = time.time() + seconds
    while time.time() < end:
        st = client.get_cached_joint_state(joint) or {}
        if st.get("position") is not None:
            pos.append(float(st["position"]))
        if st.get("torque") is not None:
            tau.append(float(st["torque"]))
        if st.get("current") is not None:
            cur.append(float(st["current"]))
        await asyncio.sleep(0.02)
    med = lambda a: float(np.median(a)) if a else None          # noqa: E731
    return {"position_rad": med(pos), "reported_torque_nm": med(tau),
            "reported_current_a": med(cur), "n": len(pos),
            "position_std_rad": float(np.std(pos)) if pos else None}


async def run(args) -> int:
    cfgp = resolve_robot_config_path()
    rc = RobotConfig.from_json(str(cfgp))
    raw = json.loads(Path(cfgp).read_text())["joints"]
    if args.joint not in raw:
        print(f"unknown joint {args.joint}", file=sys.stderr)
        return 2
    jc = raw[args.joint]
    kt, gear, ilim, tlim = (jc["torque_constant"], abs(jc["gear_ratio"]),
                            jc["current_limit"], jc["torque_limit"])
    ceiling = kt * ilim * gear
    print(f"joint {args.joint}: Kt {kt} gear {gear:.0f} I_lim {ilim} "
          f"torque_limit {tlim} ceiling {ceiling:.2f} N·m")

    client = DaemonClient(rc)
    await client.start()
    trials = []
    try:
        _bad = await joints_awake(client, [args.joint])
        if _bad:
            print("these joints are not awake: "
                  + ", ".join(f"{j}={st}" for j, st in _bad)
                  + "\n  connect the robot in the web app first.", file=sys.stderr)
            return 2

        base = await sample(client, args.joint, 1.0)
        if base["position_rad"] is None:
            print("no telemetry for that joint.", file=sys.stderr)
            return 2
        hold = base["position_rad"]
        print(f"\nholding {args.joint} at its CURRENT position {hold:+.5f} rad "
              f"({math.degrees(hold):+.2f}°) — no motion on start")
        client.set_mode(args.joint, "POSITION")
        client.set_position(args.joint, hold)
        await asyncio.sleep(SETTLE_S)

        unloaded = await sample(client, args.joint)
        print(f"  unloaded: pos {unloaded['position_rad']:+.5f} rad  "
              f"reported tau {unloaded['reported_torque_nm']:+.3f} N·m  "
              f"I {unloaded['reported_current_a']:+.3f} A  "
              f"(pos std {unloaded['position_std_rad']:.2e})")

        print("\nNow apply loads. Enter each as:  <mass_kg> <lever_m> [lever_angle_deg_from_horizontal]")
        print("Blank line when done. 'a' aborts and releases the joint.\n")
        while True:
            try:
                line = input("load> ").strip()
            except EOFError:
                print("\nno TTY — run this in a terminal.", file=sys.stderr)
                break
            if not line:
                break
            if line.lower() == "a":
                print("aborted.")
                break
            parts = line.split()
            try:
                m = float(parts[0]); L = float(parts[1])
                ang = float(parts[2]) if len(parts) > 2 else 0.0
            except Exception:
                print("  need: <mass_kg> <lever_m> [angle_deg]")
                continue
            tau_applied = m * G * L * math.cos(math.radians(ang))
            if tau_applied > tlim:
                print(f"  !! {tau_applied:.2f} N·m exceeds this joint's {tlim} N·m cap — "
                      f"it will saturate and the stiffness fit will be invalid. Use less mass.")
                continue
            print(f"  applied tau = {tau_applied:.3f} N·m; letting it settle {SETTLE_S:.0f}s ...")
            await asyncio.sleep(SETTLE_S)
            s = await sample(client, args.joint)
            defl = s["position_rad"] - unloaded["position_rad"]
            kp_eff = (tau_applied / abs(defl)) if abs(defl) > 1e-5 else float("inf")
            d_tau = ((s["reported_torque_nm"] or 0) - (unloaded["reported_torque_nm"] or 0))
            d_cur = ((s["reported_current_a"] or 0) - (unloaded["reported_current_a"] or 0))
            tau_from_current = d_cur * kt * gear
            trials.append({"mass_kg": m, "lever_m": L, "angle_deg": ang,
                           "tau_applied_nm": tau_applied,
                           "deflection_rad": defl, "kp_eff_nm_per_rad": kp_eff,
                           "reported_torque_delta_nm": d_tau,
                           "reported_current_delta_a": d_cur,
                           "tau_from_current_nm": tau_from_current,
                           **{f"loaded_{k}": v for k, v in s.items()}})
            print(f"  deflection {defl:+.5f} rad ({math.degrees(defl):+.2f}°)  "
                  f"-> kp_eff {kp_eff:8.2f} N·m/rad   (nominal {NOMINAL_KP})")
            print(f"  ESC says: d_tau {d_tau:+.3f} N·m | d_I {d_cur:+.3f} A "
                  f"-> Kt·gear·I = {tau_from_current:+.3f} N·m vs {tau_applied:.3f} applied")
            print("  remove the load before the next entry.\n")
    finally:
        try:
            client.set_mode(args.joint, "IDLE")
            print("joint released to IDLE.")
        except Exception:
            pass
        await client.stop()

    if not trials:
        print("no trials recorded.")
        return 1

    tau = np.array([t["tau_applied_nm"] for t in trials])
    dfl = np.array([abs(t["deflection_rad"]) for t in trials])
    print(f"\n=== {args.joint}: {len(trials)} loads ===")
    print(f"{'tau_applied':>12s} {'deflection':>11s} {'kp_eff':>9s} {'ESC_tau':>9s} {'Kt·I·gear':>10s}")
    for t in trials:
        print(f"{t['tau_applied_nm']:>12.3f} {t['deflection_rad']:>+11.5f} "
              f"{t['kp_eff_nm_per_rad']:>9.2f} {t['reported_torque_delta_nm']:>+9.3f} "
              f"{t['tau_from_current_nm']:>+10.3f}")

    fit = None
    if len(trials) >= 2:
        # tau = kp_eff * deflection + deadband  -> slope is the stiffness
        slope, intercept = np.polyfit(dfl, tau, 1)
        fit = {"kp_eff_fitted_nm_per_rad": float(slope), "deadband_nm": float(intercept)}
        print(f"\nfit over {len(trials)} points: kp_eff = {slope:.2f} N·m/rad, "
              f"deadband {intercept:+.3f} N·m")
        r = slope / NOMINAL_KP
        print(f"  nominal kp is {NOMINAL_KP} -> ratio {r:.2f}")
        if r < 0.5:
            print(f"  *** CONFIRMS THE HYPOTHESIS: the joint is {1/r:.1f}x SOFTER than sim believes.")
            print(f"      Sim thinks a given error makes {1/r:.1f}x more restoring torque than it does.")
            print(f"      This is a PLANT correction, not a reward change.")
        elif r < 0.85:
            print("  partial: measurably softer than nominal, but less than the 3-5x torque gap needs.")
        else:
            print("  REFUTES the stiffness hypothesis — kp is close to nominal, so the 3-5x")
            print("  torque gap is elsewhere (gearbox efficiency, Kt, or the actuator model).")

    out = C.default_outdir() / f"m7_stiffness_{args.joint}_{time.strftime('%Y%m%dT%H%M%S')}.json"
    C.write_json(out, {
        "_meta": C.finish_meta(C.capture_meta("bench", measurement="M7_stiffness",
                                              joint=args.joint, nominal_kp=NOMINAL_KP,
                                              torque_constant=kt, gear=gear,
                                              current_limit=ilim, torque_limit=tlim,
                                              motor_ceiling_nm=ceiling)),
        "unloaded": unloaded, "trials": trials, "fit": fit,
    })
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", required=True)
    ap.add_argument("--i-am-present", action="store_true",
                    help="required: this energises a joint and you will be loading it by hand")
    args = ap.parse_args()
    if not args.i_am_present:
        print("REFUSING: this energises a joint. Re-run with --i-am-present, robot supported.",
              file=sys.stderr)
        return 2
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
