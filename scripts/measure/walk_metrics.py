#!/usr/bin/env python3
"""Walk quality from the policy runner's tick log — one scorer for every bundle.

    .venv/bin/python scripts/measure/walk_metrics.py --label measC-walk          # newest log
    .venv/bin/python scripts/measure/walk_metrics.py --recording recordings/run_X.jsonl

The walk window is the span where the forward command |vx| > 0.05 (the operator's stick), so
arming, standing and the stop are excluded without hand-picked times. Everything is computed
over that window:

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
    return order, t, cmd, q, v, tgt, gz, g


def contract_caps(policy, order):
    """Per-joint effort limit from the identified bundle's contract; 12/7 if unknown."""
    try:
        c = json.loads((C.REPO_ROOT / "policies" / policy / "leg_policy_contract.json").read_text())
        lim = {j["joint_name"]: float(j["effort_limit"]) for j in c["joints"]}
        return np.array([lim[n] for n in order]), "contract"
    except Exception:
        return np.array([7.0 if "ankle" in n else 12.0 for n in order]), "assumed 12/7"


def metrics(path, policy=None):
    order, t, cmd, q, v, tgt, gz, g = load(path)
    mov = np.abs(cmd[:, 0]) > 0.05
    if mov.sum() < 25:
        return {"recording": os.path.basename(path), "error": "no walk window (|vx| > 0.05 < 1 s)"}
    idx = np.where(mov)[0]
    a, b = idx[0], idx[-1]
    dt = float(np.median(np.diff(t)))
    n1s = int(round(1.0 / dt))
    J = {n: order.index(n) for n in order}
    kl, kr = J["left_knee_pitch_joint"], J["right_knee_pitch_joint"]

    def swing(j):
        seg = q[a:b + 1, j]
        pp = [np.ptp(seg[i:i + n1s]) for i in range(0, len(seg) - n1s + 1, n1s)]
        return float(np.median(pp)) if pp else None

    L, R = q[a:b + 1, kl], q[a:b + 1, kr]
    corr = float(np.corrcoef(L - L.mean(), R - R.mean())[0, 1])
    spec = np.abs(np.fft.rfft(L - L.mean()))
    freqs = np.fft.rfftfreq(len(L), dt)
    band = (freqs >= 0.5) & (freqs <= 4.0)
    f_gait = float(freqs[band][np.argmax(spec[band])]) if band.any() else None
    pre = q[max(0, a - n1s):a].mean(0) if a > 0 else q[a]
    end = q[max(a, b - n1s // 2):b + 1].mean(0)
    tilt = np.degrees(np.arctan2(np.hypot(g[:, 0], g[:, 1]), -g[:, 2]))
    tau = 45.0 * (tgt - q) - 1.5 * v
    cap, cap_src = contract_caps(policy, order)
    W = slice(a, b + 1)
    return {
        "recording": os.path.basename(path),
        "walk_window_s": float(t[b] - t[a]),
        "command_vx_mean": float(cmd[W, 0].mean()), "command_wz_max_abs": float(np.abs(cmd[W, 2]).max()),
        "knee_swing_rad": {"left": swing(kl), "right": swing(kr)},
        "knee_lr_corr": corr, "gait_freq_hz": f_gait,
        "hip_yaw_drift_deg": {s: float(np.degrees(end[J[f"{s}_hip_yaw_joint"]] -
                                                  pre[J[f"{s}_hip_yaw_joint"]])) for s in ("left", "right")},
        "heading_change_deg": float(np.degrees(np.sum(gz[W]) * dt)),
        "tilt_p95_deg": float(np.percentile(tilt[W], 95)), "tilt_max_deg": float(tilt[W].max()),
        "joint_vel_max_rad_s": float(np.abs(v[W]).max()),
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
    ks = m["knee_swing_rad"]
    print(f"  walk window {m['walk_window_s']:.1f} s  vx {m['command_vx_mean']:.2f}  "
          f"|wz| max {m['command_wz_max_abs']:.2f}")
    print(f"  knee swing L/R {ks['left']:.3f} / {ks['right']:.3f} rad   knee corr {m['knee_lr_corr']:+.3f}"
          f"   gait {m['gait_freq_hz']:.2f} Hz")
    hy = m["hip_yaw_drift_deg"]
    print(f"  hip_yaw drift L/R {hy['left']:+.1f} / {hy['right']:+.1f}°   heading change "
          f"{m['heading_change_deg']:+.1f}° (supported)")
    print(f"  tilt p95 {m['tilt_p95_deg']:.1f}° max {m['tilt_max_deg']:.1f}°   |vel| max "
          f"{m['joint_vel_max_rad_s']:.2f} rad/s")
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
