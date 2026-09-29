#!/usr/bin/env python3
"""M7 — effective joint stiffness, every leg joint, both directions. Supersedes v1 and v2.

    python scripts/measure/m7_sweep.py --dry-run                 # print the schedule, move nothing
    python scripts/measure/m7_sweep.py --i-am-present            # full run, ~7 min
    python scripts/measure/m7_sweep.py --i-am-present --only-pass 1
    python scripts/measure/m7_sweep.py --analyse docs/measurements/m7_sweep_<ts>.json

*** MOVES JOINTS. BOTH LEGS MUST HANG FREE — robot lifted, feet off the ground. ***

Plan: ~/.claude/plans/okay-lets-step-back-happy-babbage.md

### Method

The limb's own weight is the load. URDF link masses sum to 12.61191 kg against 12.61 kg measured
(0.015%), so gravity torque on a free-hanging leg is known from geometry. `position_ki = 0` on every
joint and velocity is zero once settled, so at equilibrium the PD law balances gravity:

    kp_eff * (commanded - actual) = -tau_gravity        (device frame)

That reads stiffness directly, with no torque_constant, current sensor or gearbox efficiency in the
loop — unlike every reconstructed `kp*err - kd*vel` figure in the reports, which is circular.

### The operator's test structure

* Every joint is tested in BOTH directions, bounded by its own limits — direction asymmetry
  (stiction, backlash) is an explicit M7 requirement.
* hip_pitch and knee_pitch: both legs sweep together in the SAME device direction; they cannot
  collide. Run as separate passes, so each joint's load is set only by its own sweep.
* hip_roll and hip_yaw CAN collide, so the two sides sweep together in OPPOSITE device directions —
  one leg goes positive while the other goes negative — and a second pass swaps them. MEASURED,
  not derived: moving the legs the same way in the world gives anti-correlated device positions
  (hip_roll -0.88, hip_yaw -0.95, handmove_frames.py). Same device sign is mirrored motion; that is
  what v2 did, and the legs converged to within 2.6 cm.

### What v1 and v2 got wrong, and how this avoids it

* v1 left non-swept joints IDLE; they sagged while tau was computed assuming zeros, inflating
  kp_eff up to ~2x (hip_roll read 96-100 N·m/rad). Here ALL 12 joints are held in POSITION and tau
  is computed post-hoc from the MEASURED positions of all 12.
* The device->URDF sign per joint is not assumed. Reasoning from gear_ratio signs was wrong twice.
  Each joint's frame is chosen from data: the convention in which the deflection opposes gravity
  (err * tau_dev < 0, which equilibrium requires) for most points. Because a frame flip on one
  joint moves the distal links that load its neighbours, the choice is iterated until stable.
  NOTE the flipped torque must be converted back to the device frame before that test — comparing
  a device-frame error with a URDF-frame torque is exactly the kind of mismatch that fooled the
  earlier global `err * tau` check.
* Capture and analysis are separate. Capture stores raw measurements only; analysis is a pure
  function that can be re-run on the saved JSON with --analyse.

### Hardstops

A joint resting on its stop is held by the stop, not by its stiffness — that point measures the
stop (operator's point). Targets stay >= STOP_MARGIN from any stop, and interior torque peaks are
used instead of the stop where they exist (hip_pitch peaks at -86 deg against a -109 stop, knee at
+98 against +140). The deciding guard is physical, not geometric: a stop bearing load makes the
ESC's reported torque depart from the gravity torque, so points whose ESC/gravity ratio departs
more than DIVERGENCE_X from that joint's own median are excluded. Points within PROXIMITY of a stop
are excluded as a backstop.

STOP_MARGIN is 3 deg, not the 8 in the plan: at 8 deg the hip_roll adduction side (stop at -10 deg)
has no range left — 0.06 N·m on the right leg — and the plan's own schedule used a 2 deg margin
there. Deflection under load is away from the stop in every pass here (the load pulls the limb back
toward hanging), and the divergence guard catches the case where it is not.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import common as C  # noqa: E402
import leg_gravity as LG  # noqa: E402

NOMINAL_KP = 45.0
STEPS = 6
RAMP_S, SETTLE_S, SAMPLE_S = 2.5, 3.0, 2.0
STOP_MARGIN = math.radians(3.0)
PROXIMITY = math.radians(2.0)
# Held joints at kp=45 legitimately sag a few degrees as the swept leg shifts their load, and tau
# is computed from the measured pose, so drift is not an error source. A 1-degree gate rejected
# most of v2's points for no reason; this only catches a runaway.
DRIFT_TOL = math.radians(15.0)
MIN_TAU = 0.4
DIVERGENCE_X = 2.0
FRAME_AGREE = 0.8
# capture-time abort guards: the signature of a joint fighting a stop or an obstruction
ABORT_ERR = math.radians(20.0)
ABORT_NEAR_STOP = math.radians(1.0)
ABORT_TORQUE_FRAC = 0.9
# Dead-servo guard. ESC torque is Kt*gear*current (verified: tau/I matches Kt*gear within 0.5% on
# all 12 joints), so ESC_torque/error is an independent read of servo stiffness — 44.5-45.3 on
# every joint in v1. Far outside that means the joint is not holding. v2's ankles sat at their
# IDLE rest pose with 0.02 A while "error" grew to -62 deg; this aborts on the first such point.
SERVO_BAND = (0.5 * NOMINAL_KP, 2.0 * NOMINAL_KP)
SERVO_MIN_ERR = math.radians(1.0)
# Contact guard, model-free: torque the motor delivers beyond what gravity needs. Healthy joints
# sit within +-0.8 N·m (v2, every healthy point). Two legs pressing together grows it steadily —
# v2's mirrored hip_yaw pass went 1.2 -> 3.1 -> 5.5 N·m; this aborts it at the 3.1 step. Two consecutive over-limit steps abort the pass.
CONTACT_EXTRA_NM = 1.2
AWAKE_HOLDING = {"ENABLED", "POSITION"}
AWAKE = {"IDLE", "ENABLED", "POSITION", "DAMPING"}
FAMILY = {"hip_roll": "M6C12", "hip_yaw": "M6C12", "hip_pitch": "M6C12",
          "knee_pitch": "M6C12", "ankle_pitch": "MAD5010", "ankle_roll": "MAD5010"}
MARGINAL = {"ankle_pitch"}

# (joint type, left direction, right direction). Same direction = no collision risk;
# opposite direction = the collision-safe pairing for hip_roll / hip_yaw.
PASSES = [
    ("hip_pitch", "neg", "neg"),
    ("hip_pitch", "pos", "pos"),
    ("hip_roll", "pos", "neg"),
    ("hip_roll", "neg", "pos"),
    ("hip_yaw", "neg", "pos"),
    ("hip_yaw", "pos", "neg"),
    ("knee_pitch", "pos", "pos"),
    ("ankle_pitch", "neg", "neg"),
]

_STOP = {"flag": False}


def say(*a, **k):
    print(*a, **k, flush=True)


def jtype(j):
    return j.replace("left_", "").replace("right_", "").replace("_joint", "")


# ------------------------------------------------------------------ schedule


# Travel caps for pairs whose limbs meet before either joint reaches its stop. hip_yaw: turning
# both legs the same world direction brings one foot into the other — seen by the operator
# 2026-09-28 in pass 5, right side clean at 33.6 deg and in contact by 37.9 deg (8.3 N·m
# beyond gravity). 32 deg keeps a margin for the joint's own deflection under load.
PAIR_CAP = {"hip_yaw": math.radians(32.0)}


def plan_target(model, raw, joint, direction, home, cap=None):
    """Target for one side in one direction: the interior torque peak, or STOP_MARGIN short of
    the stop. Planning uses the plain frame with other joints at zero — only to CHOOSE targets;
    the measurement itself is computed from the measured pose in the chosen frame."""
    lim = raw[joint]["position_limits"]
    stop = lim["min"] if direction == "neg" else lim["max"]
    far = stop + STOP_MARGIN if direction == "neg" else stop - STOP_MARGIN
    if cap is not None:
        far = max(far, -cap) if direction == "neg" else min(far, cap)
    start = home if home is not None else 0.0
    if (direction == "neg" and far >= start) or (direction == "pos" and far <= start):
        return None, 0.0, stop
    uj = LG.urdf_name(joint)
    cand = np.linspace(0.0 if (0.0 - far) * (start - far) > 0 else start, far, 90)
    taus = np.array([abs(model.gravity_torque({uj: a}, [uj])[uj]) for a in cand])
    i = int(np.argmax(taus))
    return float(cand[i]), float(taus[i]), stop


def build_schedule(model, raw, home=None):
    sched = []
    for n, (t, dl, dr) in enumerate(PASSES, 1):
        sides = {}
        for side, d in (("left", dl), ("right", dr)):
            j = f"{side}_{t}_joint"
            h = home.get(j) if home else None
            tgt, tau, stop = plan_target(model, raw, j, d, h, PAIR_CAP.get(t))
            sides[j] = {"direction": d, "target_rad": tgt, "pred_peak_tau": tau,
                        "stop_rad": stop,
                        "margin_deg": (abs(math.degrees(stop - tgt)) if tgt is not None else None),
                        "usable": tgt is not None and tau >= MIN_TAU}
        sched.append({"pass": n, "type": t,
                      "relation": "same" if dl == dr else "opposite", "sides": sides})
    return sched


def print_schedule(sched):
    say(f"{'#':>2s} {'type':12s} {'rel':9s} {'left':>28s} {'right':>28s}")
    for p in sched:
        cells = []
        for j, s in p["sides"].items():
            if s["target_rad"] is None:
                cells.append(f"{s['direction']} — no range")
            else:
                flag = "" if s["usable"] else " (weak)"
                cells.append(f"{s['direction']} {math.degrees(s['target_rad']):+6.1f}° "
                             f"τ{s['pred_peak_tau']:.2f} m{s['margin_deg']:.0f}°{flag}")
        say(f"{p['pass']:>2d} {p['type']:12s} {p['relation']:9s} {cells[0]:>28s} {cells[1]:>28s}")
    say(f"\n{len(sched)} passes x ~{STEPS*(RAMP_S+SETTLE_S+SAMPLE_S)+5:.0f}s = "
        f"~{len(sched)*(STEPS*(RAMP_S+SETTLE_S+SAMPLE_S)+5)/60:.0f} min "
        f"| ankle_roll skipped (gravity ~0.04 N·m — use m7_stiffness.py with a mass)")


# ------------------------------------------------------------------ capture


async def read_all(client, joints, seconds):
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
    return {j: {"position_rad": med(v["pos"]), "esc_torque_nm": med(v["tau"]),
                "esc_current_a": med(v["cur"]), "n": len(v["pos"])} for j, v in acc.items()}


async def ramp(client, start, goal, seconds=RAMP_S, hz=50.0):
    n = max(int(seconds * hz), 1)
    for i in range(1, n + 1):
        f = i / n
        for j in goal:
            client.set_position(j, start[j] + (goal[j] - start[j]) * f)
        await asyncio.sleep(1.0 / hz)


def _on_signal(signum, _frame):
    if _STOP["flag"]:
        raise KeyboardInterrupt
    _STOP["flag"] = True
    say(f"\n[stop] signal {signum} — finishing this step, then returning home and releasing.")


async def capture(args, raw):
    sys.path.insert(0, str(C.REPO_ROOT / "scripts"))
    import _bootstrap  # noqa: F401
    from humanoid_control.daemon import DaemonClient, RobotConfig
    model = LG.LegModel()
    joints = C.canonical_joint_order()
    client = DaemonClient(RobotConfig.from_json(str(C.robot_config_path())))
    await client.start()
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _on_signal)
    home, record = None, {"passes": []}
    try:
        await asyncio.sleep(1.2)
        states = {j: (client.get_cached_joint_state(j) or {}).get("state") for j in joints}
        bad = {j: s for j, s in states.items() if s not in AWAKE}
        if bad:
            say("not awake — connect in the web app first: " +
                ", ".join(f"{j}={s}" for j, s in bad.items()))
            return None
        say("reading position_ki back from every ESC (the method assumes ki = 0) ...")
        ki_bad = {}
        for j in joints:
            try:
                ki = (await asyncio.get_running_loop().run_in_executor(
                    None, client.read_device_config, j) or {}).get("position_ki")
            except Exception as exc:
                ki = None
                say(f"  {j}: readback failed ({exc})")
            if ki is not None and abs(float(ki)) > 1e-9:
                ki_bad[j] = ki
        if ki_bad:
            say("REFUSING: position_ki is non-zero on the device — steady-state error would be "
                "integrated away and kp_eff = tau/err is invalid: " +
                ", ".join(f"{j}={v}" for j, v in ki_bad.items()))
            return None
        snap0 = await read_all(client, joints, 1.5)
        home = {j: snap0[j]["position_rad"] for j in joints}
        sched = build_schedule(model, raw, home)
        if args.only_pass:
            sched = [p for p in sched if p["pass"] in args.only_pass]
        print_schedule(sched)
        say("\nLegs must hang FREE. All 12 joints will be held; only each pass's pair moves.")
        say("Confirm ('y'): ", end="")
        if input().strip().lower() != "y":
            say("aborted.")
            return None
        record.update({"home_rad": home, "schedule": sched})
        say("holding all 12 at their current positions ...")
        for j in joints:
            client.set_mode(j, "POSITION")
            client.set_position(j, home[j])
        await asyncio.sleep(SETTLE_S)

        for p in sched:
            if _STOP["flag"]:
                break
            movers = [j for j, s in p["sides"].items() if s["target_rad"] is not None]
            if not movers:
                continue
            say(f"\n--- pass {p['pass']}: {p['type']} ({p['relation']}) ---")
            prec = {"pass": p["pass"], "type": p["type"], "relation": p["relation"],
                    "sides": p["sides"], "points": [], "aborted": None}
            prev = {j: home[j] for j in movers}
            contact_hits = {}
            for k in range(1, STEPS + 1):
                if _STOP["flag"]:
                    prec["aborted"] = "operator stop"
                    break
                cmd = {j: home[j] + (p["sides"][j]["target_rad"] - home[j]) * k / STEPS
                       for j in movers}
                await ramp(client, prev, cmd)
                prev = dict(cmd)
                await asyncio.sleep(SETTLE_S)
                snap = await read_all(client, joints, SAMPLE_S)
                prec["points"].append({"step": k, "t": time.time(), "commanded": cmd,
                                       "joints": snap})
                line, abort = [], None
                for j in movers:
                    pos = snap[j]["position_rad"]
                    err = cmd[j] - pos
                    lim = raw[j]["position_limits"]
                    tl = raw[j]["torque_limit"]
                    esc = snap[j]["esc_torque_nm"] or 0.0
                    state = (client.get_cached_joint_state(j) or {}).get("state")
                    if state not in AWAKE_HOLDING:
                        abort = f"{j} is {state}, not holding"
                    elif abs(err) > SERVO_MIN_ERR:
                        ks = abs(esc) / abs(err)
                        if not (SERVO_BAND[0] <= ks <= SERVO_BAND[1]):
                            abort = f"{j} servo not holding: ESC/err = {ks:.1f} (expect ~45)"
                    extra = min(abs(abs(esc) - abs(tau_dev(model, snap, j, {j: f})))
                                for f in (False, True))
                    contact_hits[j] = contact_hits.get(j, 0) + 1 if extra > CONTACT_EXTRA_NM else 0
                    if not abort and contact_hits[j] >= 2:
                        abort = f"{j} delivering {extra:.1f} N·m beyond gravity — contact?"
                    line.append(f"{j.split('_')[0][0].upper()} cmd {math.degrees(cmd[j]):+6.1f}° "
                                f"err {math.degrees(err):+5.2f}° esc {esc:+5.2f}")
                    if abort:
                        pass
                    elif abs(err) > ABORT_ERR:
                        abort = f"{j} error {math.degrees(err):.1f}° > {math.degrees(ABORT_ERR):.0f}°"
                    elif min(pos - lim["min"], lim["max"] - pos) < ABORT_NEAR_STOP:
                        abort = f"{j} within {math.degrees(ABORT_NEAR_STOP):.0f}° of a stop"
                    elif abs(esc) >= ABORT_TORQUE_FRAC * tl:
                        abort = f"{j} ESC torque {esc:.1f} N·m at {ABORT_TORQUE_FRAC:.0%} of cap"
                say(f"  step {k}: " + " | ".join(line))
                if abort:
                    prec["aborted"] = abort
                    say(f"  !! ABORTING PASS: {abort}")
                    break
            await ramp(client, prev, {j: home[j] for j in movers}, seconds=3.0)
            record["passes"].append(prec)
            if args.out_path:
                C.write_json(args.out_path, {"_meta": args.meta, "capture": record,
                                             "partial": True})
    finally:
        try:
            if home:
                for j in joints:
                    client.set_position(j, home[j])
                await asyncio.sleep(1.0)
        except Exception as exc:
            say(f"return-home failed: {exc}")
        for mode, wait in (("DAMPING", 0.5), ("IDLE", 0.0)):
            for j in joints:
                try:
                    client.set_mode(j, mode)
                except Exception:
                    pass
            await asyncio.sleep(wait)
        say("\nall joints returned home, damped, and released to IDLE.")
        await client.stop()
    return record


# ------------------------------------------------------------------ analysis (pure)


def tau_dev(model, joints_snap, joint, frames):
    """Gravity torque on `joint`, in its DEVICE frame, from the measured pose of all joints."""
    q = {LG.urdf_name(j): (-1.0 if frames.get(j) else 1.0) * (v["position_rad"] or 0.0)
         for j, v in joints_snap.items()}
    uj = LG.urdf_name(joint)
    t = model.gravity_torque(q, [uj])[uj]
    return (-1.0 if frames.get(joint) else 1.0) * float(t)


def _agreement(model, samples, joint, frames):
    good = [(s["err"], tau_dev(model, s["snap"], joint, frames)) for s in samples]
    good = [(e, t) for e, t in good if abs(t) > MIN_TAU]
    if len(good) < 2:
        return None
    return sum(1 for e, t in good if e * t < 0) / len(good)


def _fit_r2(model, samples, joint, frames):
    """R^2 of |tau| against |err|. An independent frame test: in the right frame the joint is a
    spring, so |tau| is linear in |err|; in the wrong frame tau is evaluated at the mirrored angle
    and the relation scatters. Needed because the sign test alone is weak for near-odd torque
    curves — on v1's knee data the WRONG frame still scored 0.83 sign agreement."""
    pts = [(abs(s["err"]), abs(tau_dev(model, s["snap"], joint, frames))) for s in samples]
    pts = [(e, t) for e, t in pts if t > MIN_TAU]
    if len(pts) < 3:
        return None
    x, y = np.array(pts).T
    if np.ptp(x) < 1e-6:
        return None
    k, b = np.polyfit(x, y, 1)
    ss_res = float(np.sum((y - (k * x + b)) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2)) or 1e-12
    return 1.0 - ss_res / ss_tot


def _fit(pts):
    if len(pts) < 2:
        return None
    x = np.array([abs(p["err"]) for p in pts])
    y = np.array([abs(p["tau"]) for p in pts])
    if np.ptp(x) < 1e-5:
        return None
    slope, icpt = np.polyfit(x, y, 1)
    return {"kp_eff": float(slope), "deadband_nm": float(icpt), "points": len(pts),
            "peak_tau_nm": float(y.max()), "ratio_to_nominal": float(slope / NOMINAL_KP)}


def analyse(record, raw):
    model = LG.LegModel()
    home = record["home_rad"]
    samples = {}
    for p in record["passes"]:
        movers = [j for j, s in p["sides"].items() if s["target_rad"] is not None]
        for pt in p["points"]:
            for j in movers:
                snap = pt["joints"]
                pos = snap[j]["position_rad"]
                if pos is None:
                    continue
                drift = max((abs((snap[o]["position_rad"] or home[o]) - home[o])
                             for o in snap if o not in movers), default=0.0)
                samples.setdefault(j, []).append({
                    "pass": p["pass"], "direction": p["sides"][j]["direction"],
                    "step": pt["step"], "cmd": pt["commanded"][j], "pos": pos,
                    "err": pt["commanded"][j] - pos, "esc": snap[j]["esc_torque_nm"],
                    "snap": snap, "drift": drift, "pass_aborted": p["aborted"]})

    # frame selection, iterated so each joint's choice uses its neighbours' chosen frames
    frames, agree, history = {}, {}, []
    for it in range(4):
        changed = False
        for j, ss in samples.items():
            a_plain = _agreement(model, ss, j, {**frames, j: False})
            a_flip = _agreement(model, ss, j, {**frames, j: True})
            r_plain = _fit_r2(model, ss, j, {**frames, j: False})
            r_flip = _fit_r2(model, ss, j, {**frames, j: True})
            if a_plain is None and a_flip is None:
                agree[j] = {"plain": None, "flipped": None, "frame": "unresolved"}
                continue
            ap, af = (a_plain or 0.0), (a_flip or 0.0)
            decided_by = "sign"
            if ap >= FRAME_AGREE and af >= FRAME_AGREE and abs(ap - af) < 0.10 \
                    and r_plain is not None and r_flip is not None:
                pick = r_flip > r_plain          # both plausible on sign: let linearity decide
                decided_by = "fit_r2"
            else:
                pick = af > ap
            best = af if pick else ap
            frame = ("flipped" if pick else "plain") if best >= FRAME_AGREE else "unresolved"
            if frames.get(j) != pick:
                changed = True
            frames[j] = pick
            agree[j] = {"plain": a_plain, "flipped": a_flip, "r2_plain": r_plain,
                        "r2_flipped": r_flip, "decided_by": decided_by, "frame": frame}
        history.append({j: v["frame"] for j, v in agree.items()})
        if not changed and it > 0:
            break

    # Contact is a PAIR event: when one foot pushes the other, the partner's torque is wrong too
    # (v3 pass 5: left hip_yaw needed LESS torque at step 5 while its gravity load rose — the
    # right foot was pushing it along). So the first step where any mover delivers more than
    # CONTACT_EXTRA_NM beyond gravity poisons that step and the rest of the pass for every mover.
    contact_from = {}
    for j, ss in samples.items():
        for s in ss:
            if s["esc"] is None:
                continue
            extra = abs(s["esc"]) - abs(tau_dev(model, s["snap"], j, frames))
            if extra > CONTACT_EXTRA_NM:
                contact_from[s["pass"]] = min(contact_from.get(s["pass"], 99), s["step"])

    per_joint, excluded = {}, []
    for j, ss in samples.items():
        lim = raw[j]["position_limits"]
        pts = []
        for s in ss:
            t = tau_dev(model, s["snap"], j, frames)
            s2 = {k: v for k, v in s.items() if k != "snap"}
            s2["tau"] = t
            s2["esc_over_gravity"] = (s["esc"] / -t) if (s["esc"] is not None and abs(t) > 1e-6) else None
            pts.append(s2)
        ratios = [p["esc_over_gravity"] for p in pts
                  if p["esc_over_gravity"] is not None and abs(p["tau"]) > MIN_TAU]
        med_ratio = float(np.median(ratios)) if ratios else None
        for p in pts:
            why = None
            if p["step"] >= contact_from.get(p["pass"], 99):
                why = "contact"
            elif abs(p["tau"]) <= MIN_TAU:
                why = "low_torque"
            elif min(p["pos"] - lim["min"], lim["max"] - p["pos"]) < PROXIMITY:
                why = "proximity"
            elif p["drift"] > DRIFT_TOL:
                why = "drift"
            elif (med_ratio and p["esc_over_gravity"] is not None and
                  (p["esc_over_gravity"] <= 0 or
                   max(p["esc_over_gravity"] / med_ratio, med_ratio / p["esc_over_gravity"]) > DIVERGENCE_X)):
                why = "divergence"
            elif p["err"] * p["tau"] >= 0:
                why = "wrong_sign"
            p["excluded"] = why
            if why:
                excluded.append({"joint": j, "pass": p["pass"], "step": p["step"], "reason": why})
        use = [p for p in pts if not p["excluded"]]
        byd = {d: _fit([p for p in use if p["direction"] == d]) for d in ("pos", "neg")}
        comb = _fit(use)
        asym = (byd["pos"]["kp_eff"] / byd["neg"]["kp_eff"]
                if byd["pos"] and byd["neg"] and byd["neg"]["kp_eff"] > 0 else None)
        esc_kp = [p["esc"] / p["err"] for p in use if p["esc"] is not None and abs(p["err"]) > 1e-4]
        per_joint[j] = {
            "type": jtype(j), "family": FAMILY[jtype(j)],
            "frame": agree.get(j, {}).get("frame"), "frame_agreement": agree.get(j),
            "by_direction": byd, "combined": comb, "direction_asymmetry": asym,
            "esc_over_gravity_median": med_ratio,
            "kp_as_seen_by_esc_median": float(np.median(esc_kp)) if esc_kp else None,
            "confidence": "marginal" if jtype(j) in MARGINAL else "normal",
            "points": pts}
        if per_joint[j]["frame"] == "unresolved":
            per_joint[j]["combined"] = None
            per_joint[j]["by_direction"] = {"pos": None, "neg": None}

    lr = {}
    for t in {jtype(j) for j in per_joint}:
        L = (per_joint.get(f"left_{t}_joint") or {}).get("combined")
        R = (per_joint.get(f"right_{t}_joint") or {}).get("combined")
        if L and R and R["kp_eff"] > 0:
            lr[t] = {"left": L["kp_eff"], "right": R["kp_eff"], "ratio": L["kp_eff"] / R["kp_eff"]}
    fam = {}
    for f in ("M6C12", "MAD5010"):
        v = [r["combined"]["kp_eff"] for r in per_joint.values()
             if r["family"] == f and r["combined"] and r["confidence"] == "normal"]
        if v:
            fam[f] = {"mean": float(np.mean(v)), "median": float(np.median(v)),
                      "min": float(min(v)), "max": float(max(v)), "n": len(v),
                      "ratio_to_nominal": float(np.median(v) / NOMINAL_KP)}
    allk = [r["combined"]["kp_eff"] for r in per_joint.values()
            if r["combined"] and r["confidence"] == "normal"]
    return {"per_joint": per_joint, "left_right": lr, "per_family": fam,
            "fk_vs_esc": {"esc_over_gravity_median": {j: r["esc_over_gravity_median"]
                                                      for j, r in per_joint.items()}},
            "frame_history": history, "excluded": excluded,
            "overall": ({"median_kp_eff": float(np.median(allk)),
                         "ratio_to_nominal": float(np.median(allk) / NOMINAL_KP),
                         "joints": len(allk)} if allk else None)}


def _num(v, fmt):
    return format(v, fmt) if v is not None else "—"


def report(res):
    say(f"\n{'='*84}")
    say(f"kp_eff per joint (nominal {NOMINAL_KP}) — tau from MEASURED pose, frame from data")
    say("=" * 84)
    say(f"{'joint':24s} {'frame':10s} {'agree':>6s} {'pos':>8s} {'neg':>8s} {'combined':>9s} "
        f"{'asym':>6s} {'esc/grav':>9s} {'kp@esc':>7s}")
    for j in C.canonical_joint_order():
        r = res["per_joint"].get(j)
        if not r:
            continue
        agreement = r["frame_agreement"] or {}
        scores = [x for x in (agreement.get("plain"), agreement.get("flipped")) if x is not None]
        best = max(scores) if scores else None
        pos = r["by_direction"]["pos"]
        neg = r["by_direction"]["neg"]
        comb = r["combined"]
        tag = " (marginal)" if r["confidence"] == "marginal" else ""
        say(f"{j:24s} {str(r['frame']):10s} {_num(best, '.2f'):>6s} "
            f"{_num(pos and pos['kp_eff'], '.2f'):>8s} {_num(neg and neg['kp_eff'], '.2f'):>8s} "
            f"{_num(comb and comb['kp_eff'], '.2f'):>9s} "
            f"{_num(r['direction_asymmetry'], '.2f'):>6s} "
            f"{_num(r['esc_over_gravity_median'], '.2f'):>9s} "
            f"{_num(r['kp_as_seen_by_esc_median'], '.1f'):>7s}{tag}")
    if res["left_right"]:
        say("\nLEFT vs RIGHT")
        for t, v in sorted(res["left_right"].items()):
            ratio = v["ratio"]
            if 0.85 <= ratio <= 1.18:
                flag = "balanced"
            elif ratio > 1:
                flag = f"LEFT {ratio:.2f}x stiffer"
            else:
                flag = f"RIGHT {1 / ratio:.2f}x stiffer"
            say(f"  {t:12s} L {v['left']:7.2f}  R {v['right']:7.2f}  L/R {ratio:.2f}  {flag}")
    if res["per_family"]:
        say("\nPER MOTOR FAMILY (normal-confidence joints)")
        for f, v in res["per_family"].items():
            say(f"  {f:8s} n={v['n']}  median {v['median']:.2f}  "
                f"range {v['min']:.2f}-{v['max']:.2f}  ratio {v['ratio_to_nominal']:.2f}")
    from collections import Counter
    counts = Counter(e["reason"] for e in res["excluded"])
    say(f"\nexcluded points: {dict(counts) if counts else 'none'}")
    o = res["overall"]
    if not o:
        return
    say(f"\nOVERALL: median kp_eff {o['median_kp_eff']:.2f} over {o['joints']} joints, "
        f"ratio to nominal {o['ratio_to_nominal']:.2f}")
    if o["ratio_to_nominal"] < 0.5:
        say("  -> CONFIRMS softness: a plant correction — use the per-family numbers in sim.")
    elif o["ratio_to_nominal"] < 0.85:
        say("  -> PARTIAL: softer than nominal, short of the 3-5x the torque gap needs.")
    else:
        say("  -> REFUTES softness: kp is near nominal. The 3-5x gap is in Kt, gearbox "
            "efficiency or the actuator model; see esc/grav.")


# ------------------------------------------------------------------ main


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="print the schedule; move nothing")
    ap.add_argument("--i-am-present", action="store_true")
    ap.add_argument("--only-pass", type=int, nargs="+")
    ap.add_argument("--analyse", help="re-run the analysis on a saved capture JSON")
    args = ap.parse_args()
    raw = json.loads(C.robot_config_path().read_text())["joints"]

    if args.analyse:
        doc = json.loads(Path(args.analyse).read_text())
        res = analyse(doc["capture"], raw)
        report(res)
        doc["analysis"] = res
        C.write_json(Path(args.analyse), doc)
        return 0
    if args.dry_run:
        print_schedule(build_schedule(LG.LegModel(), raw))
        return 0
    if not args.i_am_present:
        say("REFUSING: this moves joints. Use --dry-run first, then --i-am-present with the "
            "legs hanging free.")
        return 2

    meta = C.capture_meta("bench", measurement="M7_sweep", nominal_kp=NOMINAL_KP,
                          guards={"stop_margin_deg": 3, "proximity_deg": 2,
                                  "drift_deg": math.degrees(DRIFT_TOL),
                                  "servo_band_nm_per_rad": SERVO_BAND,
                                  "contact_extra_nm": CONTACT_EXTRA_NM,
                                  "min_tau_nm": MIN_TAU, "divergence_x": DIVERGENCE_X,
                                  "frame_agreement": FRAME_AGREE,
                                  "abort": {"err_deg": 20, "near_stop_deg": 1,
                                            "torque_frac_of_cap": ABORT_TORQUE_FRAC}},
                          urdf_total_mass_kg=LG.total_mass(), measured_robot_mass_kg=12.61,
                          requires="both legs hanging free")
    out = C.default_outdir() / f"m7_sweep_{time.strftime('%Y%m%dT%H%M%S')}.json"
    args.out_path, args.meta = out, meta
    rec = asyncio.run(capture(args, raw))
    if not rec or not rec["passes"]:
        say("no data captured.")
        return 1
    meta = C.finish_meta(meta)
    C.write_json(out, {"_meta": meta, "capture": rec})   # raw first, so a crash below loses nothing
    res = analyse(rec, raw)
    report(res)
    C.write_json(out, {"_meta": meta, "capture": rec, "analysis": res})
    say(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
