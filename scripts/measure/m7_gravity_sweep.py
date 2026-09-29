#!/usr/bin/env python3
"""M7 — effective joint stiffness from the leg's OWN weight. No hanging masses needed.

    python scripts/measure/m7_gravity_sweep.py --joint left_hip_pitch_joint --i-am-present

*** MOVES ONE JOINT. THE LEG MUST HANG FREE — robot lifted, foot off the ground. ***

### Why the limb is a valid calibrated load

The URDF's link masses sum to **12.61191 kg** against a **12.61 kg** measured robot — 0.015%
error. The masses are validated, not assumed, so the gravitational torque on a free-hanging leg is
known from geometry:

    tau_j = sum over links distal to j of ((p_i - o_j) x m_i g) . axis_j

Better than hanging weights in one respect: torque varies as cos(angle), so **sweeping the joint
sweeps the load** over a continuous range with no re-rigging. `left_hip_pitch` spans 0.39 to
5.80 N·m from 0 to 90 degrees.

### What makes the static reading clean

`position_ki = 0` on every joint, so there is no integrator to erase steady-state error, and
`velocity_kp` acts on velocity which is zero once settled. At rest, therefore:

    steady-state error = tau_gravity / kp_eff

which is a direct read of stiffness. This does NOT rely on torque_constant, the current sensor or
gearbox efficiency — unlike every "hardware torque" figure in the reports so far, which was
reconstructed as `kp*err - kd*vel` and is circular (a joint that cannot track shows large error,
which inflates the inferred demand).

### Reading the result

`kp_eff` near 45 refutes the stiffness hypothesis and sends the search to gearbox efficiency, Kt,
or the actuator model. Near 15-20 confirms it: sim believes a given error makes ~3x more restoring
torque than the robot makes, which explains a uniform 3-5x torque gap across six joints that share
little else, and explains why raising caps and adding penalties kept moving the problem.

CLOSED-CHAIN WARNING: this is only valid with the foot OFF THE GROUND. Standing puts the floor in
the load path and the model does not apply.
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

STEP_RAMP_S = 2.5
SETTLE_S = 3.0
SAMPLE_S = 2.0


async def sample(client, joint, seconds=SAMPLE_S):
    pos, tau, cur = [], [], []
    end = time.time() + seconds
    while time.time() < end:
        st = client.get_cached_joint_state(joint) or {}
        for key, acc in (("position", pos), ("torque", tau), ("current", cur)):
            if st.get(key) is not None:
                acc.append(float(st[key]))
        await asyncio.sleep(0.02)
    med = lambda a: float(np.median(a)) if a else None      # noqa: E731
    return {"position_rad": med(pos), "reported_torque_nm": med(tau),
            "reported_current_a": med(cur),
            "position_std_rad": float(np.std(pos)) if pos else None, "n": len(pos)}


async def ramp(client, joint, start, goal, seconds=STEP_RAMP_S, hz=50.0):
    n = max(int(seconds * hz), 1)
    for i in range(1, n + 1):
        client.set_position(joint, start + (goal - start) * (i / n))
        await asyncio.sleep(1.0 / hz)


async def run(args) -> int:
    cfgp = resolve_robot_config_path()
    rc = RobotConfig.from_json(str(cfgp))
    raw = json.loads(Path(cfgp).read_text())["joints"]
    if args.joint not in raw:
        print(f"unknown joint {args.joint}", file=sys.stderr)
        return 2
    jc = raw[args.joint]
    lim = jc["position_limits"]
    model = LG.LegModel()
    uj = LG.urdf_name(args.joint)
    print(f"joint {args.joint}  -> URDF {uj}")
    print(f"  distal mass {model.distal_mass(uj):.3f} kg | torque_limit {jc['torque_limit']} N·m")
    print(f"  position limits {lim['min']:+.4f} .. {lim['max']:+.4f} rad")

    client = DaemonClient(rc)
    await client.start()
    trials = []
    start_pos = None
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
        start_pos = base["position_rad"]
        print(f"\ncurrent position {start_pos:+.5f} rad ({math.degrees(start_pos):+.2f}°)")

        angles = [math.radians(a) for a in args.angles_deg]
        bad = [a for a in angles if not (lim["min"] <= a <= lim["max"])]
        if bad:
            print(f"REFUSING: {[round(math.degrees(a),1) for a in bad]}° is outside this joint's "
                  f"position limits.", file=sys.stderr)
            return 2

        print(f"\nsweeping {len(angles)} angles; each is ramped over {STEP_RAMP_S}s then settled.")
        print("Confirm the leg is hanging FREE (foot off the ground) — 'y' to proceed: ", end="")
        try:
            if input().strip().lower() != "y":
                print("aborted.")
                return 1
        except EOFError:
            print("\nno TTY — run this in a terminal.", file=sys.stderr)
            return 2

        client.set_mode(args.joint, "POSITION")
        prev = start_pos
        for a in angles:
            await ramp(client, args.joint, prev, a)
            prev = a
            await asyncio.sleep(SETTLE_S)
            s = await sample(client, args.joint)
            q = {uj: a}
            tau_g = model.gravity_torque(q, [uj])[uj]
            err = a - s["position_rad"]          # commanded - actual
            kp_eff = abs(tau_g) / abs(err) if abs(err) > 1e-5 else float("inf")
            trials.append({"commanded_rad": a, "commanded_deg": math.degrees(a),
                           "tau_gravity_nm": tau_g, "error_rad": err,
                           "kp_eff_nm_per_rad": kp_eff, **s})
            print(f"  cmd {math.degrees(a):>+7.2f}°  actual {math.degrees(s['position_rad']):>+7.2f}°  "
                  f"err {math.degrees(err):>+6.2f}°  tau_g {tau_g:>+6.3f} N·m  "
                  f"kp_eff {kp_eff:>7.2f}  ESC tau {s['reported_torque_nm']:+.3f}")
    finally:
        try:
            if start_pos is not None:
                cur = (await sample(client, args.joint, 0.3))["position_rad"] or start_pos
                await ramp(client, args.joint, cur, start_pos, seconds=3.0)
            client.set_mode(args.joint, "IDLE")
            print("\nreturned to the starting position and released to IDLE.")
        except Exception:
            pass
        await client.stop()

    good = [t for t in trials if abs(t["tau_gravity_nm"]) > 0.3 and math.isfinite(t["kp_eff_nm_per_rad"])]
    if len(good) < 2:
        print("need at least 2 points with meaningful gravity load (>0.3 N·m).")
        return 1

    tau = np.array([abs(t["tau_gravity_nm"]) for t in good])
    err = np.array([abs(t["error_rad"]) for t in good])
    slope, intercept = np.polyfit(err, tau, 1)
    print(f"\n=== {args.joint}: fit over {len(good)} loaded points ===")
    print(f"  kp_eff = {slope:.2f} N·m/rad   (nominal {NOMINAL_KP})   deadband {intercept:+.3f} N·m")
    r = slope / NOMINAL_KP
    print(f"  ratio to nominal: {r:.2f}")
    if r < 0.5:
        print(f"  *** CONFIRMS THE HYPOTHESIS — the joint is {1/r:.1f}x softer than sim believes.")
        print(f"      Sim expects {1/r:.1f}x more restoring torque from a given error than the robot")
        print(f"      produces. That is a PLANT correction, not a reward change, and it explains a")
        print(f"      uniform 3-5x torque gap across joints sharing only kp=45.")
    elif r < 0.85:
        print("  PARTIAL — measurably softer than nominal, but short of the 3-5x the gap needs.")
    else:
        print("  REFUTES the stiffness hypothesis — kp is close to nominal. Look instead at")
        print("  gearbox efficiency, Kt, or the actuator model.")

    out = C.default_outdir() / f"m7_gravity_{args.joint}_{time.strftime('%Y%m%dT%H%M%S')}.json"
    C.write_json(out, {
        "_meta": C.finish_meta(C.capture_meta(
            "bench", measurement="M7_gravity_sweep", joint=args.joint, urdf_joint=uj,
            nominal_kp=NOMINAL_KP, distal_mass_kg=model.distal_mass(uj),
            urdf_total_mass_kg=LG.total_mass(model), measured_robot_mass_kg=12.61,
            method="leg's own weight; position_ki=0 so steady-state error = tau/kp_eff",
            requires="leg hanging FREE, foot off the ground")),
        "trials": trials,
        "fit": {"kp_eff_nm_per_rad": float(slope), "deadband_nm": float(intercept),
                "ratio_to_nominal": float(r), "points": len(good)},
    })
    print(f"\nwrote {out}", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", default="left_hip_pitch_joint")
    ap.add_argument("--angles-deg", type=float, nargs="+",
                    default=[0, -15, -30, -45, -60, -75],
                    help="commanded angles to step through (default sweeps hip_pitch back)")
    ap.add_argument("--i-am-present", action="store_true")
    args = ap.parse_args()
    if not args.i_am_present:
        print("REFUSING: this moves a joint. Re-run with --i-am-present, leg hanging free.",
              file=sys.stderr)
        return 2
    return asyncio.run(run(args))


if __name__ == "__main__":
    raise SystemExit(main())
