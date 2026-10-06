#!/usr/bin/env python3
"""Walk quality from the policy runner's tick log — one scorer for every bundle.

    .venv/bin/python scripts/measure/walk_metrics.py --label measC-walk          # newest log
    .venv/bin/python scripts/measure/walk_metrics.py --recording recordings/run_X.jsonl

Only ticks with the trigger HELD and the stick FORWARD count. The tick log records only while
the trigger is held, so each release is a time gap; the log is split into bouts at every gap
and every stick release, and bouts under 1 s are dropped. Metrics are per bout, then pooled
(duration-weighted) — a short room means several short walks, and they must not be merged
across the time the robot was being repositioned:

* **knee swing** — median per-second peak-to-peak of knee_pitch (rad). smoothA sim: 0.849.
* **knee L/R correlation** — stepping alternates (strongly negative; sim -0.7..-0.9), sagging
  moves together (positive).
* **gait frequency** — dominant FFT peak of the left knee, 0.5-4 Hz.
* **hip_yaw drift** — mean over the last 0.5 s of the window minus the 1 s before it (deg).
* **heading change** — integral of gyro z over the window (deg). The walks are SUPPORTED, so
  the operator's hand constrains this; read it as an upper bound on what the policy did alone.

The network is identified by replaying logged observations through every staged ONNX
(stand_metrics.identify_policy) — the service does not log which checkpoint it loaded.
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
import stand_metrics as SM  # noqa: E402


def load(path):
    order, rows = None, []
    for ln in open(path):
        try:
            d = json.loads(ln)
        except Exception:
            continue
        if "_meta" in d:
            order = d["_meta"]["joint_order"]
            continue
        if d.get("command") is None or d.get("joint_pos") is None:
            continue
        rows.append(d)
    t = np.array([r["t"] for r in rows])
    cmd = np.array([r["command"] for r in rows], dtype=float)
    q = np.array([r["joint_pos"] for r in rows], dtype=float)
    v = np.array([r["joint_vel"] for r in rows], dtype=float)
    tgt = np.array([r["targets"] for r in rows], dtype=float)
    gz = np.array([(r.get("base_ang_vel") or [0, 0, 0])[2] for r in rows], dtype=float)
    g = np.array([r.get("projected_gravity") or [0, 0, -1] for r in rows], dtype=float)
    load.heading = [r.get("heading") for r in rows]          # None on logs before the loop existed
    return order, t, cmd, q, v, tgt, gz, g


def contract_caps(policy, order):
    """Per-joint effort limit from the identified bundle's contract; 12/7 if unknown."""
    try:
        c = json.loads((C.REPO_ROOT / "policies" / policy / "leg_policy_contract.json").read_text())
        lim = {j["joint_name"]: float(j["effort_limit"]) for j in c["joints"]}
        return np.array([lim[n] for n in order]), "contract"
    except Exception:
        return np.array([7.0 if "ankle" in n else 12.0 for n in order]), "assumed 12/7"


def bouts(t, cmd, dt):
    """Walking bouts: contiguous ticks with the stick forward (|vx| > 0.05) AND the trigger held.
    The tick log only records while the trigger is held, so a release shows up as a time gap;
    splitting on gaps > 3 ticks keeps separate walks separate instead of merging them across
    the time the robot was being repositioned."""
    mov = np.abs(cmd[:, 0]) > 0.05
    gap = np.r_[True, np.diff(t) > 3 * dt]
    out, start = [], None
    for i in range(len(t)):
        if mov[i] and (start is None or gap[i]):
            if start is not None:
                out.append((start, i - 1))
            start = i
        elif not mov[i] and start is not None:
            out.append((start, i - 1))
            start = None
    if start is not None:
        out.append((start, len(t) - 1))
    return [(a, b) for a, b in out if b - a + 1 >= int(round(1.0 / dt))]   # >= 1 s


def metrics(path, policy=None):
    order, t, cmd, q, v, tgt, gz, g = load(path)
    dt = float(np.median(np.diff(t)))
    n1s = int(round(1.0 / dt))
    B = bouts(t, cmd, dt)
    if not B:
        return {"recording": os.path.basename(path), "error": "no walking bout of >= 1 s"}
    J = {n: order.index(n) for n in order}
    kl, kr = J["left_knee_pitch_joint"], J["right_knee_pitch_joint"]
    tilt = np.degrees(np.arctan2(np.hypot(g[:, 0], g[:, 1]), -g[:, 2]))
    tau = 45.0 * (tgt - q) - 1.5 * v
    cap, cap_src = contract_caps(policy, order)

    def one(a, b):
        sl = slice(a, b + 1)
        L, R = q[sl, kl] - q[sl, kl].mean(), q[sl, kr] - q[sl, kr].mean()
        pp = lambda j: [np.ptp(q[a + i:a + i + n1s, j]) for i in range(0, b - a + 1 - n1s + 1, n1s)]  # noqa
        spec = np.abs(np.fft.rfft(L)); fr = np.fft.rfftfreq(len(L), dt)
        band = (fr >= 0.5) & (fr <= 4.0)
        # hip_yaw drift inside the bout: last 0.5 s minus first 0.5 s
        h = max(1, n1s // 2)
        drift = {s: float(np.degrees(q[b - h + 1:b + 1, J[f"{s}_hip_yaw_joint"]].mean() -
                                     q[a:a + h, J[f"{s}_hip_yaw_joint"]].mean())) for s in ("left", "right")}
        hd = [load.heading[i] for i in range(a, b + 1)]
        heading = None
        if all(h and h.get("yaw") is not None for h in hd):
            yaw = np.degrees(np.unwrap([h["yaw"] for h in hd]))
            wzl = np.array([h["wz_loop"] for h in hd]); sent = np.array([h["wz_sent"] for h in hd])
            err = np.degrees([h["heading_error"] for h in hd])
            heading = {"mode": hd[-1]["mode"],
                       "fused_yaw_change_deg": float(yaw[-1] - yaw[0]),
                       "gyro_yaw_change_deg": float(np.degrees(np.sum(gz[sl]) * dt)),
                       "heading_error_max_abs_deg": float(np.abs(err).max()),
                       "wz_loop_mean": float(wzl.mean()), "wz_loop_max_abs": float(np.abs(wzl).max()),
                       "wz_loop_clipped_pct": float((np.abs(wzl) >= hd[-1].get("wz_max", 0.5) * 0.999).mean() * 100)
                       if "wz_max" in hd[-1] else float((np.abs(wzl) >= 0.4995).mean() * 100),
                       "wz_sent_mean": float(sent.mean())}
        return {"t_start": float(t[a]), "duration_s": float(t[b] - t[a] + dt), "heading": heading,
                "knee_lr_corr": float(np.corrcoef(L, R)[0, 1]),
                "knee_swing_rad": {"left": float(np.median(pp(kl))) if pp(kl) else None,
                                   "right": float(np.median(pp(kr))) if pp(kr) else None},
                "gait_freq_hz": float(fr[band][np.argmax(spec[band])]) if (band.any() and len(L) >= 2 * n1s) else None,
                "hip_yaw_drift_deg": drift,
                "heading_change_deg": float(np.degrees(np.sum(gz[sl]) * dt)),
                "heading_rate_deg_s": float(np.degrees(np.sum(gz[sl]) * dt) / (t[b] - t[a] + dt))}

    per = [one(a, b) for a, b in B]
    W = np.concatenate([np.arange(a, b + 1) for a, b in B])
    w = np.array([p["duration_s"] for p in per])
    wavg = lambda vals: float(np.average([x for x in vals], weights=w))  # noqa
    return {
        "recording": os.path.basename(path),
        "bouts": per, "n_bouts": len(per), "walking_s": float(w.sum()),
        "command_vx_mean": float(cmd[W, 0].mean()), "command_wz_max_abs": float(np.abs(cmd[W, 2]).max()),
        "knee_swing_rad": {s: wavg([p["knee_swing_rad"][s] or 0 for p in per]) for s in ("left", "right")},
        "knee_lr_corr": wavg([p["knee_lr_corr"] for p in per]),
        "gait_freq_hz": (float(np.median([p["gait_freq_hz"] for p in per if p["gait_freq_hz"]]))
                         if any(p["gait_freq_hz"] for p in per) else None),
        "hip_yaw_drift_deg": {s: wavg([p["hip_yaw_drift_deg"][s] for p in per]) for s in ("left", "right")},
        "heading_abs_rate_deg_s": wavg([abs(p["heading_rate_deg_s"]) for p in per]),
        "tilt_p95_deg": float(np.percentile(tilt[W], 95)), "tilt_max_deg": float(tilt[W].max()),
        "joint_vel_max_rad_s": float(np.abs(v[W]).max()),
        # Foot-strike ankle spikes (measF's target). Same 25 Hz tick-log source as the
        # 24-29 rad/s figures reported for measC/measE, so they compare directly.
        "ankle_pitch_vel": {s: {"max_rad_s": float(np.abs(v[W, J[f"{s}_ankle_pitch_joint"]]).max()),
                                "ticks_over_13": int((np.abs(v[W, J[f"{s}_ankle_pitch_joint"]]) > 13.0).sum())}
                            for s in ("left", "right")},
        "torque_p95_nm": {n: float(np.percentile(np.abs(tau[W, i]), 95)) for i, n in enumerate(order)},
        "cap_source": cap_src,
        "saturation_pct": {n: float((np.abs(tau[W, i]) >= cap[i] * 0.999).mean() * 100)
                           for i, n in enumerate(order)},
    }


def show(m, label=""):
    print(f"{label or m['recording']}")
    pid = m.get("policy_id") or {}
    if pid.get("policy"):
        print(f"  network: {pid['policy']} ({'EXACT' if pid['exact'] else 'NOT exact'})")
    if "error" in m:
        print("  " + m["error"])
        return
    print(f"  {m['n_bouts']} bout(s), {m['walking_s']:.1f} s walking (trigger held, stick forward), "
          f"vx {m['command_vx_mean']:.2f}")
    for i, p in enumerate(m["bouts"], 1):
        ks = p["knee_swing_rad"]; hy = p["hip_yaw_drift_deg"]
        gf = f"{p['gait_freq_hz']:.2f} Hz" if p["gait_freq_hz"] else "  -  "
        print(f"   bout {i}: {p['duration_s']:4.1f} s  knee corr {p['knee_lr_corr']:+.2f}  swing "
              f"{ks['left'] or 0:.2f}/{ks['right'] or 0:.2f}  {gf}  hip_yaw {hy['left']:+.1f}/"
              f"{hy['right']:+.1f}°  heading {p['heading_change_deg']:+6.1f}° "
              f"({p['heading_rate_deg_s']:+.1f}°/s)")
        h = p.get("heading")
        if h:
            print(f"           loop[{h['mode']}] fused yaw {h['fused_yaw_change_deg']:+6.1f}° vs gyro "
                  f"{h['gyro_yaw_change_deg']:+6.1f}°  |err| max {h['heading_error_max_abs_deg']:5.1f}°  "
                  f"wz_loop mean {h['wz_loop_mean']:+.3f} max {h['wz_loop_max_abs']:.3f} "
                  f"(clipped {h['wz_loop_clipped_pct']:.0f}%)  sent {h['wz_sent_mean']:+.3f}")
    ks = m["knee_swing_rad"]; hy = m["hip_yaw_drift_deg"]
    print(f"  POOLED  knee corr {m['knee_lr_corr']:+.3f}  swing {ks['left']:.3f}/{ks['right']:.3f} rad  "
          f"gait {m['gait_freq_hz'] or float('nan'):.2f} Hz  |heading rate| {m['heading_abs_rate_deg_s']:.1f}°/s")
    print(f"  tilt p95 {m['tilt_p95_deg']:.1f}° max {m['tilt_max_deg']:.1f}°   |vel| max "
          f"{m['joint_vel_max_rad_s']:.2f} rad/s")
    ap = m.get("ankle_pitch_vel")
    if ap:
        print(f"  ankle_pitch |vel| max L/R {ap['left']['max_rad_s']:.1f} / {ap['right']['max_rad_s']:.1f} rad/s"
              f"   ticks >13 rad/s L/R {ap['left']['ticks_over_13']} / {ap['right']['ticks_over_13']}")
    sat = {k: v for k, v in m["saturation_pct"].items() if v > 0.5}
    print("  saturation >0.5%: " + (", ".join(f"{k.replace('_joint', '')} {v:.1f}%"
                                               for k, v in sat.items()) or "none"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recording")
    ap.add_argument("--label")
    args = ap.parse_args()
    path = args.recording or max(glob.glob(str(C.REPO_ROOT / "recordings" / "run_*.jsonl")),
                                 key=os.path.getmtime)
    pid = SM.identify_policy(path)
    m = metrics(path, pid.get("policy"))
    m["policy_id"] = pid
    show(m, args.label or "")
    if args.label:
        out = C.default_outdir() / f"{args.label}_walk.json"
        C.write_json(out, {"_meta": C.finish_meta(C.capture_meta(
            "walk", measurement="walk_metrics", label=args.label, recording=m["recording"])),
            "metrics": m})
        print(f"wrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
