#!/usr/bin/env python3
"""Stuck events and swing-foot clearance from recorded walk tick logs (brief items B and C).

    .venv/bin/python scripts/measure/gait_contact.py --label contact-20260929 \
        recordings/run_A.jsonl recordings/run_B.jsonl ...

No robot needed; it reads the policy runner's tick logs. Only ticks with the trigger held and the
stick forward count (walk_metrics.bouts).

Foot heights: forward kinematics of the 12 logged joint angles (device frame x
policy_frame_sign = URDF frame; verified exactly against the logged observation). Uses the
`leg_*_ankle_roll` link origin per foot, projected on world-up = -projected_gravity. The URDF
root `base` shares the IMU's orientation, so only the left/right difference is used and the
root's position does not matter.

C. CLEARANCE, as defined in ROBOT_PC_BRIEF_2026-09-29 so the training PC computes the same
number in sim:
    d(t) = z_left(t) - z_right(t); steps split at every sign change of d; a step's clearance is
    max |d| between consecutive sign changes; half-steps shorter than 0.15 s are dropped.
    Reported: median and 10th percentile per bout, plus stall vs non-stall steps.

B. STALLS: >= 0.3 s where both knees move < 0.5 rad/s, OR the SAME hip_pitch or knee_pitch joint
sits at its torque cap for >= 0.3 s (kp*err - kd*vel >= 0.98 * contract cap). For each: start, duration, leg, sagittal
error and saturation, foot heights, IMU pitch/roll and yaw rate, action magnitude, and how it
ends (the first sagittal joint to leave the cap; the change in d over the last 0.2 s).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import common as C  # noqa: E402
import leg_gravity as LG  # noqa: E402
import stand_metrics as SM  # noqa: E402
import walk_metrics as WM  # noqa: E402
from humanoid_control.config import POLICY_FRAME_MIRRORED_JOINTS  # noqa: E402

KNEE_STILL = 0.5          # rad/s
SAT_FRAC = 0.98
STALL_MIN_S = 0.3
HALFSTEP_MIN_S = 0.15
SAG = ("hip_pitch", "knee_pitch")


def load(path):
    rows = [json.loads(l) for l in open(path)]
    order = rows[0]["_meta"]["joint_order"]
    rows = [r for r in rows[1:] if r.get("joint_pos") and r.get("command") is not None]
    A = lambda k: np.array([r[k] for r in rows], dtype=float)  # noqa: E731
    return order, rows, A("t"), A("command"), A("joint_pos"), A("joint_vel"), A("targets"), \
        A("action"), A("projected_gravity"), np.array([r["base_ang_vel"][2] for r in rows])


def foot_heights(order, q, g, model):
    sign = np.array([-1.0 if n in POLICY_FRAME_MIRRORED_JOINTS else 1.0 for n in order])
    zl, zr = np.empty(len(q)), np.empty(len(q))
    for i in range(len(q)):
        T, _ = model.fk({LG.urdf_name(n): float(s * v) for n, s, v in zip(order, sign, q[i])})
        up = -g[i] / np.linalg.norm(g[i])
        zl[i] = T["leg_left_ankle_roll"][1] @ up
        zr[i] = T["leg_right_ankle_roll"][1] @ up
    return zl, zr


def clearance_steps(t, d):
    """Per-step clearance, exactly as the brief defines it. Returns [(t0, t1, clearance)]."""
    s = np.sign(d)
    cross = [0] + [i for i in range(1, len(d)) if s[i] != 0 and s[i] != s[i - 1]] + [len(d)]
    out = []
    for a, b in zip(cross[:-1], cross[1:]):
        if b - a < 2 or t[b - 1] - t[a] < HALFSTEP_MIN_S:
            continue
        out.append((float(t[a]), float(t[b - 1]), float(np.max(np.abs(d[a:b])))))
    # the first and last segments are cut by the bout edges, not by a crossing
    return out[1:-1] if len(out) > 2 else []


def analyse_log(path, model):
    order, rows, t, cmd, q, v, tgt, act, g, gz = load(path)
    pid = SM.identify_policy(path)
    policy = pid.get("policy")
    cap, _ = WM.contract_caps(policy, order)
    dt = float(np.median(np.diff(t)))
    bouts = WM.bouts(t, cmd, dt)
    J = {n: i for i, n in enumerate(order)}
    tau = 45.0 * (tgt - q) - 1.5 * v
    sat = np.abs(tau) >= SAT_FRAC * cap
    kl, kr = J["left_knee_pitch_joint"], J["right_knee_pitch_joint"]
    sag_idx = {f"{s}_{j}": J[f"{s}_{j}_joint"] for s in ("left", "right") for j in SAG}
    zl, zr = foot_heights(order, q, g, model)
    d = zl - zr
    pitch = np.degrees(np.arctan2(g[:, 0], -g[:, 2]))
    roll = np.degrees(np.arctan2(g[:, 1], -g[:, 2]))
    nmin = max(1, int(round(STALL_MIN_S / dt)))

    res_bouts, stalls, steps_all = [], [], []
    for bi, (a, b) in enumerate(bouts, 1):
        idx = np.arange(a, b + 1)
        still = (np.abs(v[idx, kl]) < KNEE_STILL) & (np.abs(v[idx, kr]) < KNEE_STILL)

        def long_runs(mask):
            out, s0 = np.zeros(len(mask), bool), None
            for k, f in enumerate(np.r_[mask, False]):
                if f and s0 is None:
                    s0 = k
                elif not f and s0 is not None:
                    if k - s0 >= nmin:
                        out[s0:k] = True
                    s0 = None
            return out

        # "Sits at its cap" is per joint: the SAME joint capped for >= 0.3 s. OR-ing the four
        # joints tick by tick chains the alternating left/right stance saturation of a normal
        # gait into multi-second "stalls" (first version: median 0.68 s, max 2.28 s, 26-47% of
        # all walking), so each joint's run is found first and the runs are then unioned.
        capped = np.zeros(len(idx), bool)
        for j in sag_idx.values():
            capped |= long_runs(sat[idx, j])
        flag = long_runs(still) | capped
        # runs of flag >= nmin
        runs, s0 = [], None
        for k, f in enumerate(np.r_[flag, False]):
            if f and s0 is None:
                s0 = k
            elif not f and s0 is not None:
                if k - s0 >= nmin:
                    runs.append((s0, k - 1))
                s0 = None
        stall_mask = np.zeros(len(idx), bool)
        for s0, s1 in runs:
            stall_mask[s0:s1 + 1] = True
            ii = idx[s0:s1 + 1]
            satfrac = {k: float(sat[ii, j].mean()) for k, j in sag_idx.items()}
            left_sat = satfrac["left_hip_pitch"] + satfrac["left_knee_pitch"]
            right_sat = satfrac["right_hip_pitch"] + satfrac["right_knee_pitch"]
            leg = ("both-still" if still[s0:s1 + 1].mean() > 0.5 and max(left_sat, right_sat) < 0.2
                   else "left" if left_sat > right_sat else "right")
            # how it ends: first sagittal joint that was capped in the last 0.2 s and is not
            # capped on the first tick after the stall
            tail = ii[-max(1, int(round(0.2 / dt))):]
            after = idx[min(s1 + 1, len(idx) - 1)]
            released = [k for k, j in sag_idx.items() if sat[tail, j].any() and not sat[after, j]]
            stalls.append({
                "log": Path(path).name, "policy": policy, "bout": bi,
                "t_start": float(t[ii[0]]), "duration_s": float(t[ii[-1]] - t[ii[0]] + dt),
                "leg": leg, "kind": "knees_still" if still[s0:s1 + 1].mean() > 0.5 else "capped",
                "sag_error_mean_rad": {k: float(np.mean(tgt[ii, j] - q[ii, j])) for k, j in sag_idx.items()},
                "sag_saturation_frac": satfrac,
                "foot_z_left_m": [float(zl[ii].min()), float(zl[ii].max())],
                "foot_z_right_m": [float(zr[ii].min()), float(zr[ii].max())],
                "d_mean_mm": float(d[ii].mean() * 1000),
                "clearance_in_stall_mm": float(np.abs(d[ii]).max() * 1000),
                "imu_pitch_deg": float(pitch[ii].mean()), "imu_roll_deg": float(roll[ii].mean()),
                "yaw_rate_abs_mean_deg_s": float(np.degrees(np.abs(gz[ii]).mean())),
                "action_norm_mean": float(np.linalg.norm(act[ii], axis=1).mean()),
                "ends": {"released_first": released,
                         "d_change_last_0p2s_mm": float((d[tail[-1]] - d[tail[0]]) * 1000),
                         "ended_by_bout_end": bool(s1 >= len(idx) - 1)},
            })
        steps = clearance_steps(t[idx], d[idx])
        stall_times = [(t[idx[s0]], t[idx[s1]]) for s0, s1 in runs]
        for (t0, t1, c) in steps:
            in_stall = any(not (t1 < a0 or t0 > a1) for a0, a1 in stall_times)
            steps_all.append({"log": Path(path).name, "policy": policy, "bout": bi,
                              "t0": t0, "t1": t1, "clearance_mm": c * 1000, "stall": in_stall})
        cl = np.array([s[2] for s in steps]) * 1000
        dur = float(t[b] - t[a] + dt)
        res_bouts.append({
            "bout": bi, "duration_s": dur,
            "stall_fraction": float(stall_mask.mean()), "n_stalls": len(runs),
            "heading_rate_deg_s": float(np.degrees(np.sum(gz[idx]) * dt) / dur),
            "n_steps": len(steps),
            "clearance_median_mm": float(np.median(cl)) if len(cl) else None,
            "clearance_p10_mm": float(np.percentile(cl, 10)) if len(cl) else None,
        })
    return {"log": Path(path).name, "policy": policy, "exact": pid.get("exact"),
            "bouts": res_bouts, "walking_s": float(sum(b["duration_s"] for b in res_bouts))}, \
        stalls, steps_all


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--label")
    args = ap.parse_args()
    model = LG.LegModel()
    logs, stalls, steps = [], [], []
    for p in args.logs:
        L, S, St = analyse_log(p, model)
        logs.append(L); stalls += S; steps += St
        print(f"\n{L['log']}  {L['policy']}  ({'exact' if L['exact'] else 'NOT exact'})  "
              f"{L['walking_s']:.1f} s walking")
        for b in L["bouts"]:
            cm = f"{b['clearance_median_mm']:5.1f}" if b["clearance_median_mm"] is not None else "   - "
            cp = f"{b['clearance_p10_mm']:5.1f}" if b["clearance_p10_mm"] is not None else "   - "
            print(f"  bout {b['bout']}: {b['duration_s']:4.1f} s  stalls {b['n_stalls']} "
                  f"({b['stall_fraction'] * 100:4.0f}% of bout)  steps {b['n_steps']:2d}  "
                  f"clearance median {cm} mm  p10 {cp} mm  heading {b['heading_rate_deg_s']:+5.1f}°/s")

    print("\n=== per policy ===")
    summary = {}
    for pol in sorted({L["policy"] for L in logs}):
        Ls = [L for L in logs if L["policy"] == pol]
        B = [b for L in Ls for b in L["bouts"]]
        w = sum(b["duration_s"] for b in B)
        stall_s = sum(b["stall_fraction"] * b["duration_s"] for b in B)
        S = [s for s in stalls if s["policy"] == pol]
        st = [s for s in steps if s["policy"] == pol]
        cl = np.array([s["clearance_mm"] for s in st])
        cls = np.array([s["clearance_mm"] for s in st if s["stall"]])
        cln = np.array([s["clearance_mm"] for s in st if not s["stall"]])
        fr = [b["stall_fraction"] for b in B]; hr = [abs(b["heading_rate_deg_s"]) for b in B]
        corr = float(np.corrcoef(fr, hr)[0, 1]) if len(B) > 2 and np.std(fr) > 0 else None
        pct = lambda a, p: float(np.percentile(a, p)) if len(a) else None  # noqa: E731
        summary[pol] = {
            "walking_s": w, "n_bouts": len(B), "n_stalls": len(S),
            "stall_fraction": stall_s / w if w else None,
            "stalls_by_kind": {k: sum(1 for s in S if s["kind"] == k) for k in ("capped", "knees_still")},
            "stalls_by_leg": {k: sum(1 for s in S if s["leg"] == k) for k in ("left", "right", "both-still")},
            "corr_stall_fraction_vs_abs_heading_rate": corr,
            "n_steps": len(st),
            "clearance_mm": {"median": pct(cl, 50), "p10": pct(cl, 10),
                             "stall_median": pct(cls, 50), "stall_p10": pct(cls, 10), "n_stall_steps": len(cls),
                             "nonstall_median": pct(cln, 50), "nonstall_p10": pct(cln, 10)},
            "released_first": dict(sorted(
                {k: sum(1 for s in S if k in s["ends"]["released_first"])
                 for k in ("left_hip_pitch", "left_knee_pitch", "right_hip_pitch", "right_knee_pitch")}.items())),
        }
        s = summary[pol]; c = s["clearance_mm"]
        f = lambda x: f"{x:5.1f}" if x is not None else "  -  "  # noqa: E731
        print(f"{pol}: {s['walking_s']:.1f} s, {s['n_bouts']} bouts | stalls {s['n_stalls']} "
              f"({(s['stall_fraction'] or 0) * 100:.0f}% of walking) kind {s['stalls_by_kind']} leg {s['stalls_by_leg']}")
        print(f"   clearance: all median {f(c['median'])} p10 {f(c['p10'])} mm | in-stall median "
              f"{f(c['stall_median'])} p10 {f(c['stall_p10'])} (n={c['n_stall_steps']}) | not-stall median "
              f"{f(c['nonstall_median'])} p10 {f(c['nonstall_p10'])}")
        print(f"   corr(stall fraction, |heading rate|) per bout: "
              f"{s['corr_stall_fraction_vs_abs_heading_rate'] if s['corr_stall_fraction_vs_abs_heading_rate'] is None else round(s['corr_stall_fraction_vs_abs_heading_rate'], 2)}"
              f"   released first: {s['released_first']}")
    if args.label:
        out = C.default_outdir() / f"{args.label}_gait_contact.json"
        C.write_json(out, {"_meta": C.finish_meta(C.capture_meta(
            "offline", measurement="gait_contact_B_C", logs=[Path(p).name for p in args.logs],
            definitions={"stall": f"knees both |v|<{KNEE_STILL} rad/s OR hip/knee pitch |tau|>={SAT_FRAC}*cap, >= {STALL_MIN_S}s",
                         "clearance": "d=z_L-z_R (ankle_roll origin, world up = -projected_gravity); split at sign "
                                      f"changes; per-step max|d|; drop half-steps < {HALFSTEP_MIN_S}s; edge segments dropped"})),
            "summary": summary, "logs": logs, "stalls": stalls, "steps": steps})
        print(f"\nwrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
